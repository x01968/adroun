/**
 * @file smart_building_esp8266.ino
 * @brief 智慧楼宇系统 - ESP8266 Arduino 主控代码
 * @author 
 * @version 1.0
 * @date 2026-03-17
 * 
 * @copyright Copyright (c) 2026 智慧楼宇系统
 * 
 * @section description 项目描述
 * 本代码实现了基于ESP8266的智慧楼宇监控系统，主要功能包括：
 * - 与STM32微控制器进行串口通信，接收传感器数据并发送控制指令
 * - 通过WiFi连接实现网络通信，支持AP模式和STA模式
 * - 搭建Web服务器，提供可视化监控界面
 * - EEPROM数据持久化存储配置信息
 * 
 * @section protocol 通信协议

 * 从机 -> 主机（接收数据）：
 * - 传感器数据上报: {"led":状态, "ldr":值, "pir":状态, "pwm":亮度}
 * - 参数数据上报: {"lux_lower":值, "pwm_low":值, "pwm_upp":值}
 * - 命令响应: {cmd_OK}
 *
 * 主机 -> 从机（发送命令）：
 * - 命令格式: {"cmd":"命令名","val":数值}
 * 

 * ESP8266 -> STM32 串口通信协议：
 * - 配置修改: {"temp_upper":30} (一次只发送一个值)
 * - 网络信息: {"type":"net","net_mode":"AP","ip":"192.168.4.1"}
 * 

 * @section hardware 硬件连接
 * ESP8266 <-> STM32
 * - TX  -> RX (PA3)
 * - RX  -> TX (PA2)
 * - GND -> GND
 * 
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// 数据结构定义
// ============================================================================

struct DeviceData {
  int pir_status = 0;   // 0: 无人, 1: 有人
  int ldr_value = 0;    // 0-4095
  int led_status = 0;   // 0: 关, 1: 开, 2: 微亮, 3: 手动
  int led_pwm = 128;    // 0-254
  int mode = 0;         // 0: 自动, 1: 手动, 2: 夜灯
};

// ============================================================================
// 配置常量定义
// ============================================================================

/** @defgroup AP配置 AP模式配置 */
/** @{ */
const char* AP_SSID = "SmartBuilding_AP";        ///< AP模式下的WiFi名称
const char* AP_PASSWORD = "";                     ///< AP模式下的WiFi密码（空表示无密码）
/** @} */



/**
 * @brief 将运行时配置转换为持久化配置结构
 *
 * 将Config结构体中的WiFi配置信息转换并填充到
 * PersistedConfig结构中，准备写入EEPROM
 *
 * @param p 目标持久化配置结构指针
 */
static void PersistedFromRuntime(PersistedConfig* p) {
  memset(p, 0, sizeof(*p));
  p->magic = SB_EEPROM_MAGIC;
  p->version = SB_EEPROM_VERSION;
  CopyStringToBuf(config.wifiSSID, p->wifiSSID, sizeof(p->wifiSSID));
  CopyStringToBuf(config.wifiPassword, p->wifiPassword, sizeof(p->wifiPassword));
  p->checksum = CalcChecksum32((const uint8_t*)p, offsetof(PersistedConfig, checksum));
}

/**
 * @brief 将持久化配置转换为运行时配置结构
 * 
 * 从EEPROM读取的PersistedConfig结构验证并转换到Config结构体，
 * 包含魔数、版本号和校验和验证
 * 
 * @param p 源持久化配置结构引用
 * @return bool 验证成功返回true，失败返回false
 */
static bool RuntimeFromPersisted(const PersistedConfig& p) {
  if (p.magic != SB_EEPROM_MAGIC) return false;
  if (p.version != SB_EEPROM_VERSION) return false;
  uint32_t expected = CalcChecksum32((const uint8_t*)&p, offsetof(PersistedConfig, checksum));
  if (expected != p.checksum) return false;
  config.wifiSSID = String(p.wifiSSID);
  config.wifiPassword = String(p.wifiPassword);
  return true;
}

/**
 * @brief 从EEPROM加载持久化配置
 * 
 * 尝试从EEPROM地址0处读取配置数据，如果数据有效则
 * 加载到运行时config结构体中
 * 
 * @return bool 加载成功返回true，失败返回false
 */
static bool LoadPersistedConfig() {
  PersistedConfig p;
  EEPROM.get(0, p);
  return RuntimeFromPersisted(p);
}

/**
 * @brief 保存配置到EEPROM（仅在配置变化时）
 * 
 * 比较当前配置与EEPROM中存储的配置，仅在配置发生变化时才执行写入操作，
 * 减少EEPROM写入次数以延长使用寿命
 */
static void SavePersistedConfigIfChanged() {
  PersistedConfig current;
  bool has_current = false;
  EEPROM.get(0, current);
  if (current.magic == SB_EEPROM_MAGIC && current.version == SB_EEPROM_VERSION) {
    uint32_t expected = CalcChecksum32((const uint8_t*)&current, offsetof(PersistedConfig, checksum));
    has_current = (expected == current.checksum);
  }
  PersistedConfig next;
  PersistedFromRuntime(&next);
  if (has_current) {
    if (memcmp(&current, &next, sizeof(PersistedConfig)) == 0) return;
  }
  EEPROM.put(0, next);
  EEPROM.commit();
}



// ============================================================================
// 串口通信管理
// ============================================================================

/** @defgroup SerialBuffer 串口接收缓冲区定义 */
/** @{ */
#define SERIAL_RX_BUFFER_SIZE 512       ///< 串口接收缓冲区大小（字节）
char serialRxBuffer[SERIAL_RX_BUFFER_SIZE];  ///< 串口接收数据缓冲区
int serialRxIndex = 0;                  ///< 当前缓冲区写入位置索引
/** @} */

/**
 * @brief 配置发送队列项结构
 *
 * 用于存储待发送到STM32的配置参数队列
 */
struct ConfigQueueItem {
  String key;   ///< 配置参数键名（如"temp_upper"）
  int value;    ///< 配置参数值
};

/// 配置发送队列（最多10个待发送项）
ConfigQueueItem configQueue[10];
int queueHead = 0;                      ///< 队列头指针（读取位置）
int queueTail = 0;                      ///< 队列尾指针（写入位置）
bool waitingForComOk = false;           ///< 等待STM32确认响应的标志
unsigned long comOkTimeout = 0;         ///< 等待响应的超时时间戳
unsigned long lastConfigSendTime = 0;   ///< 最后一次发送配置的时间戳
#define CONFIG_SEND_INTERVAL 100        ///< 配置发送间隔（毫秒），避免发送过快
#define cobe 1

// ============================================================================
// 函数声明
// ============================================================================

void setup();
void loop();
void setupAPMode();
void setupWiFiMode();
void setupWebServer();
void handleRoot();
void handleConfig();
void handleSaveConfig();
void handleSaveThreshold();
void handleNotFound();
void sendConfigToSTM32(const String& key, int value);
void sendNetInfoToSTM32();
void handleSerial();
void sendCommand(const char* type, int value);


// ============================================================================
// Web界面HTML模板
// ============================================================================

/**
 * @brief 主监控页面HTML模板（存储在程序闪存中）
 * 
 * 包含完整的HTML5页面、CSS样式和JavaScript交互逻辑
 * 实现实时数据显示、阈值设置、舵机控制等功能
 */
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>物联网监控</title>
  <style>
    :root { --bg-color: #1a1c2c; --card-bg: #2d3047; --text-primary: #ffffff; --text-secondary: #a0a0a0; --accent: #4e73df; --success: #2ecc71; --warning: #f1c40f; --danger: #e74c3c; }
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; margin: 0; padding: 0; background-color: var(--bg-color); color: var(--text-primary); }
    .header { background: linear-gradient(135deg, #007bff, #0056b3); padding: 20px; text-align: center; box-shadow: 0 2px 10px rgba(0,0,0,0.3); }
    .header h1 { margin: 0; font-size: 1.2rem; font-weight: 500; letter-spacing: 1px; }
    .container { max-width: 500px; margin: 0 auto; padding: 15px; }
    .sub-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; padding: 0 5px; }
    .title-group { display: flex; align-items: center; gap: 10px; }
    .title-icon { background: #ff4d4d; padding: 8px; border-radius: 8px; width: 36px; height: 36px; display: flex; align-items: center; justify-content: center; }
    .title-text h2 { margin: 0; font-size: 1rem; }
    .time-text { font-size: 0.8rem; color: var(--text-secondary); text-align: right; }
    .section-title { font-size: 1rem; margin: 20px 0 15px; display: flex; align-items: center; gap: 8px; font-weight: 500; }
    .card { background: var(--card-bg); border-radius: 15px; padding: 18px; margin-bottom: 15px; position: relative; overflow: hidden; box-shadow: 0 4px 15px rgba(0,0,0,0.2); transition: transform 0.2s; }
    .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
    .card-title { display: flex; align-items: center; gap: 10px; color: var(--text-secondary); font-size: 0.9rem; }
    .online-tag { color: var(--success); font-size: 0.75rem; display: flex; align-items: center; gap: 4px; }
    .online-dot { width: 6px; height: 6px; background: var(--success); border-radius: 50%; box-shadow: 0 0 8px var(--success); }
    .card-value-row { display: flex; align-items: baseline; gap: 5px; }
    .card-value { font-size: 2rem; font-weight: 600; }
    .card-unit { color: var(--text-secondary); font-size: 0.9rem; }
    .threshold-info { position: absolute; right: 18px; bottom: 18px; text-align: right; font-size: 0.75rem; color: var(--text-secondary); line-height: 1.4; }
    .threshold-item { display: flex; align-items: center; gap: 5px; justify-content: flex-end; cursor: pointer; }
    .threshold-item:hover { color: var(--text-primary); }
    .switch-row { display: flex; justify-content: space-between; align-items: center; }
    .switch-info { display: flex; flex-direction: column; gap: 4px; }
    .switch-status { color: var(--success); font-size: 0.8rem; }
    .switch { position: relative; display: inline-block; width: 52px; height: 28px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #444; transition: .4s; border-radius: 34px; }
    .slider:before { position: absolute; content: ""; height: 22px; width: 22px; left: 3px; bottom: 3px; background-color: white; transition: .4s; border-radius: 50%; }
    input:checked + .slider { background-color: var(--accent); }
    input:checked + .slider:before { transform: translateX(24px); }
    .footer-info { margin-top: 30px; padding: 15px; background: rgba(255,255,255,0.05); border-radius: 12px; font-size: 0.75rem; color: var(--text-secondary); }
    .footer-row { display: flex; justify-content: space-between; margin-bottom: 8px; }
    .footer-row:last-child { margin-bottom: 0; }
    .modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.8); align-items: center; justify-content: center; }
    .modal-content { background: var(--card-bg); padding: 25px; border-radius: 15px; width: 90%; max-width: 400px; }
    .modal-header { margin-bottom: 20px; font-size: 1.1rem; border-bottom: 1px solid #444; padding-bottom: 10px; }
    input[type="number"] { background: #1a1c2c; border: 1px solid #444; color: white; padding: 10px; border-radius: 8px; width: 100%; margin-bottom: 15px; }
    .modal-btns { display: flex; gap: 10px; margin-top: 10px; }
    .btn-modal { flex: 1; padding: 12px; border-radius: 8px; border: none; font-weight: 500; cursor: pointer; }
    .btn-save { background: var(--accent); color: white; }
    .btn-cancel { background: #444; color: white; }
    .nav-tabs { display: flex; gap: 10px; margin-bottom: 15px; }
    .nav-tab { padding: 8px 15px; border-radius: 20px; font-size: 0.8rem; background: #333; color: var(--text-secondary); text-decoration: none; }
    .nav-tab.active { background: var(--accent); color: white; }
  </style>
</head>
<body>
  <div class="header">
    <h1>物联网监控</h1>
  </div>
  <div class="container">
    <div class="sub-header">
      <div class="title-group">
        <div class="title-icon">🛡️</div>
        <div class="title-text"><h2>物联网禁烟防火系统</h2></div>
      </div>
      <div class="time-text">
        <div id="current-date">2026年03月17日</div>
        <div id="current-time">10:50:40</div>
      </div>
    </div>

    <div class="nav-tabs">
      <a href="/" class="nav-tab active">首页监控</a>
      <a href="/config" class="nav-tab">网络设置</a>
    </div>

    <div class="section-title">📊 实时监测数据</div>

    <div class="card">
      <div class="card-header">
        <div class="card-title">🌡️ 温度</div>
        <div class="online-tag"><span class="online-dot"></span>在线</div>
      </div>
      <div class="card-value-row"><span class="card-value" id="temp">0</span><span class="card-unit">℃</span></div>
      <div class="threshold-info">
        <div class="threshold-item" onclick="openThresholdModal('temp')">上限: <span id="temp_up">0</span>℃ ⚙️</div>
        <div class="threshold-item" onclick="openThresholdModal('temp')">下限: <span id="temp_low">0</span>℃ ⚙️</div>
      </div>
    </div>

    <div class="card">
      <div class="card-header">
        <div class="card-title">💧 湿度</div>
        <div class="online-tag"><span class="online-dot"></span>在线</div>
      </div>
      <div class="card-value-row"><span class="card-value" id="hum">0</span><span class="card-unit">%</span></div>
      <div class="threshold-info">
        <div class="threshold-item" onclick="openThresholdModal('hum')">上限: <span id="hum_up">0</span>% ⚙️</div>
        <div class="threshold-item" onclick="openThresholdModal('hum')">下限: <span id="hum_low">0</span>% ⚙️</div>
      </div>
    </div>

    <div class="card">
      <div class="card-header">
        <div class="card-title">☁️ 烟雾浓度</div>
        <div class="online-tag"><span class="online-dot"></span>在线</div>
      </div>
      <div class="card-value-row"><span class="card-value" id="smoke">0</span><span class="card-unit">ppm</span></div>
      <div class="threshold-info">
        <div class="threshold-item" onclick="openThresholdModal('smoke')">上限: <span id="smoke_up">0</span>ppm ⚙️</div>
      </div>
    </div>

    <div class="card">
      <div class="card-header">
        <div class="card-title">☀️ 光照强度</div>
        <div class="online-tag"><span class="online-dot"></span>在线</div>
      </div>
      <div class="card-value-row"><span class="card-value" id="ldr">0</span><span class="card-unit">lux</span></div>
      <div class="threshold-info">
        <div class="threshold-item" onclick="openThresholdModal('ldr')">上限: <span id="ldr_up">0</span>lux ⚙️</div>
        <div class="threshold-item" onclick="openThresholdModal('ldr')">下限: <span id="ldr_low">0</span>lux ⚙️</div>
        <div class="threshold-item">补光: <span id="ldr_mode_text">关</span></div>
      </div>
    </div>

    <div class="card">
      <div class="card-header">
        <div class="card-title">🔄 舵机角度</div>
        <div class="online-tag"><span class="online-dot"></span>在线</div>
      </div>
      <div class="card-value-row"><span class="card-value" id="duoji_angle">0</span><span class="card-unit">度</span></div>
      <div class="threshold-info">
        <div class="threshold-item" onclick="openThresholdModal('duoji')">修改角度 ⚙️</div>
      </div>
    </div>

    <div class="footer-info">
      <div class="footer-row"><span>系统状态:</span><span id="footer-mode">运行中</span></div>
    </div>

    <div id="thresholdModal" class="modal">
      <div class="modal-content">
        <div class="modal-header"><span id="modal-title">修改阈值</span></div>
        <form action="/save_threshold" method="post" id="thresholdForm">
          <div id="modal-inputs"></div>
          <div class="modal-btns">
            <button type="button" class="btn-modal btn-cancel" onclick="closeThresholdModal()">取消</button>
            <button type="submit" class="btn-modal btn-save">保存修改</button>
          </div>
        </form>
      </div>
    </div>
  </div>

  <script>
    const _els = {};
    const $ = (id) => _els[id] || (_els[id] = document.getElementById(id));
    const setText = (id, val) => {
      const el = $(id);
      if(!el) return;
      const s = String(val);
      if(el.innerText !== s) el.innerText = s;
    };

    function updateTime() {
      const now = new Date();
      const d = now.getFullYear() + '年' + (now.getMonth()+1).toString().padStart(2,'0') + '月' + now.getDate().toString().padStart(2,'0') + '日';
      const t = now.getHours().toString().padStart(2,'0') + ':' + now.getMinutes().toString().padStart(2,'0') + ':' + now.getSeconds().toString().padStart(2,'0');
      setText('current-date', d);
      setText('current-time', t);
    }
    setInterval(updateTime, 1000);
    updateTime();

    let _initRev = -1;
    function initPage(force) {
      const url = force ? '/data/init' : ('/data/init?rev=' + _initRev);
      fetch(url, { cache: 'no-store' }).then(r => {
        if(r.status === 304) return null;
        if(!r.ok) return null;
        return r.json();
      }).then(data => {
        if(!data) return;
        _initRev = data.rev;
        setText('temp_up', data.temp_upper);
        setText('temp_low', data.temp_lower);
        setText('hum_up', data.hum_upper);
        setText('hum_low', data.hum_lower);
        setText('smoke_up', data.smoke_upper);
        setText('ldr_up', data.ldr_upper);
        setText('ldr_low', data.ldr_lower);
      }).catch(() => {});
    }

    let _liveRev = -1;
    let _liveDelay = 1000;
    let _liveTimer = null;
    function _scheduleLive(delay) {
      if(_liveTimer) clearTimeout(_liveTimer);
      _liveTimer = setTimeout(fetchLive, delay);
    }
    function fetchLive(force) {
      const url = force ? '/data/live' : ('/data/live?rev=' + _liveRev);
      fetch(url, { cache: 'no-store' }).then(r => {
        if(r.status === 304) {
          _liveDelay = Math.min(_liveDelay * 2, 10000);
          _scheduleLive(_liveDelay);
          return null;
        }
        if(!r.ok) {
          _liveDelay = Math.min(_liveDelay * 2, 10000);
          _scheduleLive(_liveDelay);
          return null;
        }
        return r.json();
      }).then(data => {
        if(!data) return;
        _liveRev = data.rev;
        _liveDelay = 1000;
        setText('temp', data.temperature);
        setText('hum', data.humidity);
        setText('smoke', data.smoke);
        setText('ldr', data.ldr);
        setText('duoji_angle', data.servo_angle || 0);
        setText('footer-mode', data.mode);

        // 更新补光模式显示
        const ldrMode = data.ldr || 0;
        let modeText;
        if(ldrMode < 25) modeText = '强';
        else if(ldrMode > 25 && ldrMode < 75) modeText = '弱';
        else modeText = '关';
        const ldrModeEl = $('ldr_mode_text');
        if(ldrModeEl && ldrModeEl.innerText !== modeText) ldrModeEl.innerText = modeText;

        _scheduleLive(_liveDelay);
      }).catch(() => {
        _liveDelay = Math.min(_liveDelay * 2, 10000);
        _scheduleLive(_liveDelay);
      });
    }

    initPage(true);
    fetchLive(true);

    function openThresholdModal(type) {
      const modal = document.getElementById('thresholdModal');
      const inputs = document.getElementById('modal-inputs');
      const title = document.getElementById('modal-title');
      let html = '';
      const getV = (id) => document.getElementById(id).innerText;
      if(type === 'temp') {
        title.innerText = '修改温度阈值';
        html = '<label>上限 (°C)</label><input type="number" min="0" max="99" name="temp_upper" value="' + getV('temp_up') + '">';
        html += '<label>下限 (°C)</label><input type="number" min="0" max="99" name="temp_lower" value="' + getV('temp_low') + '">';
      } else if(type === 'hum') {
        title.innerText = '修改湿度阈值';
        html = '<label>上限 (%)</label><input type="number" min="0" max="99" name="hum_upper" value="' + getV('hum_up') + '">';
        html += '<label>下限 (%)</label><input type="number" min="0" max="99" name="hum_lower" value="' + getV('hum_low') + '">';
      } else if(type === 'smoke') {
        title.innerText = '修改烟雾阈值';
        html = '<label>上限 (ppm)</label><input type="number" min="0" max="99" name="smoke_upper" value="' + getV('smoke_up') + '">';
      } else if(type === 'ldr') {
        title.innerText = '修改光照阈值';
        html = '<label>上限 (lux)</label><input type="number" min="0" max="100" name="ldr_upper" value="' + getV('ldr_up') + '">';
        html += '<label>下限 (lux)</label><input type="number" min="0" max="100" name="ldr_lower" value="' + getV('ldr_low') + '">';
      } else if(type === 'duoji') {
        title.innerText = '设置舵机角度';
        html = '<label>舵机角度 (0-180度)</label><input type="number" min="0" max="180" name="duoji_angle" value="' + getV('duoji_angle') + '">';
      }
      inputs.innerHTML = html;
      modal.style.display = 'flex';
    }

    function closeThresholdModal() {
      document.getElementById('thresholdModal').style.display = 'none';
    }

    document.getElementById('thresholdForm').addEventListener('submit', function(e) {
      e.preventDefault();
      var form = this;
      var valid = true;
      var errorMsg = '';
      
      // 获取所有输入值
      var tempUpper = parseInt(form.querySelector('[name="temp_upper"]')?.value || 0);
      var tempLower = parseInt(form.querySelector('[name="temp_lower"]')?.value || 0);
      var humUpper = parseInt(form.querySelector('[name="hum_upper"]')?.value || 0);
      var humLower = parseInt(form.querySelector('[name="hum_lower"]')?.value || 0);
      var smokeUpper = parseInt(form.querySelector('[name="smoke_upper"]')?.value || 0);
      var ldrUpper = parseInt(form.querySelector('[name="ldr_upper"]')?.value || 0);
      var ldrLower = parseInt(form.querySelector('[name="ldr_lower"]')?.value || 0);
      var duojiAngle = parseInt(form.querySelector('[name="duoji_angle"]')?.value || 0);
      
      // 验证温度范围 (0-99)
      if(tempUpper < 0 || tempUpper > 99 || tempLower < 0 || tempLower > 99) {
        valid = false;
        errorMsg = '温度值必须在0-99之间';
      }
      // 验证湿度范围 (0-99)
      else if(humUpper < 0 || humUpper > 99 || humLower < 0 || humLower > 99) {
        valid = false;
        errorMsg = '湿度值必须在0-99之间';
      }
      // 验证烟雾范围 (0-99)
      else if(smokeUpper < 0 || smokeUpper > 99) {
        valid = false;
        errorMsg = '烟雾值必须在0-99之间';
      }
      // 验证光照范围 (0-100)
      else if(ldrUpper < 0 || ldrUpper > 100 || ldrLower < 0 || ldrLower > 100) {
        valid = false;
        errorMsg = '光照值必须在0-100之间';
      }
      // 验证舵机角度范围 (0-180)
      else if(duojiAngle < 0 || duojiAngle > 180) {
        valid = false;
        errorMsg = '舵机角度必须在0-180之间';
      }
      
      // 验证上下限关系
      if(valid) {
        if(tempUpper < tempLower) {
          valid = false;
          errorMsg = '温度上限不能小于温度下限';
        } else if(humUpper < humLower) {
          valid = false;
          errorMsg = '湿度上限不能小于湿度下限';
        } else if(ldrUpper < ldrLower) {
          valid = false;
          errorMsg = '光照上限不能小于光照下限';
        }
      }
      
      if(!valid) {
        alert(errorMsg);
        return;
      }
      
      fetch('/save_threshold', { method: 'POST', body: new URLSearchParams(new FormData(this)) })
        .then(r => { if(r.ok) { alert('设置已保存'); closeThresholdModal(); initPage(true); fetchLive(true); } })
        .catch(() => {});
    });
  </script>
</body>
</html>)rawliteral";


// ============================================================================
// 系统初始化函数
// ============================================================================

/**
 * @brief 系统初始化函数
 * 
 * 在系统启动时执行一次，完成以下初始化工作：
 * 1. 串口通信初始化
 * 2. EEPROM配置加载
 * 3. WiFi网络连接（尝试STA模式，失败则切换到AP模式）
 * 4. Web服务器配置
 * 5. OTA升级功能配置
 */
void setup() {
  // 初始化串口通信，波特率115200
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("Smart Building ESP8266 Starting...");

  // 设置默认配置（如果EEPROM中没有有效数据将使用这些默认值）
  config.wifiSSID = "Redmi K50 Ultra";
  config.wifiPassword = "12345678";

  // 初始化EEPROM并尝试加载保存的配置
  EEPROM.begin(SB_EEPROM_SIZE);
  if (LoadPersistedConfig()) {
    Serial.println("Loaded WiFi config from EEPROM");
  } else {
    Serial.println("No valid EEPROM config, using defaults");
    SavePersistedConfigIfChanged();
  }

  // 尝试连接WiFi网络（STA模式）
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSSID.c_str(), config.wifiPassword.c_str());

  // 等待WiFi连接，最多等待10秒
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 10000) {
    delay(500);
    Serial.print(".");
  }

  // 根据WiFi连接结果选择工作模式
  if (WiFi.status() == WL_CONNECTED) {
    // WiFi连接成功，使用STA模式
    Serial.println();
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    systemState = STATE_WIFI_CONNECTED;
    setupWiFiMode();
    sendNetInfoToSTM32();
  } else {
    // WiFi连接失败，切换到AP模式
    Serial.println();
    Serial.println("WiFi Connection Failed, Starting AP Mode");
    systemState = STATE_AP_MODE;
    setupAPMode();
    sendNetInfoToSTM32();
  }

  // 配置Web服务器
  setupWebServer();

  Serial.println("Setup Complete!");
}


// ============================================================================
// 主循环函数
// ============================================================================

/**
 * @brief 主循环函数
 * 
 * 系统主循环，不断执行以下任务：
 * 1. 处理Web服务器客户端请求
 * 2. 解析STM32串口数据
 * 3. 处理配置发送队列
 * 4. 定期发送网络信息到STM32
 */
void loop() {
  // 处理Web服务器客户端请求
  server.handleClient();

  // 解析来自STM32的串口数据
  parseSerialData();

  // 处理待发送到STM32的配置队列
  processSerialQueue();

  // 每5秒向STM32发送一次网络状态信息
  static unsigned long lastNetSend = 0;
  if (millis() - lastNetSend > 5000) {
    sendNetInfoToSTM32();
    lastNetSend = millis();
  }

  // 维护WiFi连接
  if (WiFi.status() == WL_CONNECTED) {
    // WiFi已连接，正常运行
  }

  // 每5秒发送一次传感器数据（模拟数据）
  if ((millis() - lastDataSend) > 5000) {
    // 模拟传感器数据
    sensorData.temperature = random(20, 30);
    sensorData.humidity = random(40, 60);
    sensorData.smoke = random(0, 5);
    sensorData.ldr = random(30, 70);
    sensorData.servoAngle = 90;
    sensorData.lastUpdate = millis();
    live


// ============================================================================
// WiFi模式配置函数
// ============================================================================

/**
 * @brief 配置AP模式（热点模式）
 * 
 * 当无法连接到指定WiFi网络时，ESP8266将作为热点运行，
 * 允许用户直接连接到ESP8266进行配置
 */
void setupAPMode() {
  Serial.println("Setting up AP Mode...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.println("AP Mode Started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
  sendNetInfoToSTM32();
}

/**
 * @brief 配置STA模式（WiFi客户端模式）
 * 
 * 设置ESP8266作为WiFi客户端，连接到指定的WiFi网络
 */
void setupWiFiMode() {
  Serial.println("Setting up WiFi Mode...");
  isWiFiConnected = true;
  systemState = STATE_WIFI_CONNECTED;
  sendNetInfoToSTM32();
}


// ============================================================================
// Web服务器配置函数
// ============================================================================

/**
 * @brief 配置Web服务器
 * 
 * 设置HTTP服务器的所有路由和处理函数，包括：
 * - 主页（监控界面）
 * - 配置页面
 * - 数据API接口（JSON格式）
 * - 配置保存接口
 * - 404错误处理
 */
void setupWebServer() {
  Serial.println("Setting up Web Server...");

  // 主页路由 - 返回监控界面HTML
  server.on("/", HTTP_GET, handleRoot);
  
  // 配置页面路由
  server.on("/config", HTTP_GET, handleConfig);
  
  // 完整数据JSON API - 返回所有传感器数据和配置信息
  server.on("/data/json", HTTP_GET, []() {
    StaticJsonDocument<512> doc;
    doc["temperature"] = sensorData.temperature;
    doc["humidity"] = sensorData.humidity;
    doc["smoke"] = sensorData.smoke;
    doc["ldr"] = sensorData.ldr;
    doc["servo_angle"] = sensorData.servoAngle;
    doc["servo_enable"] = sensorData.servoEnable;
    doc["servo_open_angle"] = sensorData.servoOpenAngle;
    doc["alarm_active"] = sensorData.alarmActive ? 1 : 0;
    doc["temp_upper"] = config.tempUpper;
    doc["temp_lower"] = config.tempLower;
    doc["hum_upper"] = config.humUpper;
    doc["hum_lower"] = config.humLower;
    doc["smoke_upper"] = config.smokeUpper;
    doc["ldr_upper"] = config.ldrUpper;
    doc["ldr_lower"] = config.ldrLower;
    doc["ldr_mode"] = config.ldrMode;
    doc["cfg_rev"] = config.cfgRev;

    // 根据系统状态返回对应模式字符串
    switch (systemState) {
      case STATE_AP_MODE: doc["mode"] = "AP Mode"; break;
      case STATE_WIFI_CONNECTED: doc["mode"] = "WiFi Connected"; break;
      default: doc["mode"] = "Running"; break;
    }

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  // 初始化数据API - 返回阈值配置（支持ETag缓存验证）
  server.on("/data/init", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    if (server.hasArg("rev")) {
      uint32_t clientRev = (uint32_t)server.arg("rev").toInt();
      if (clientRev == initRev) {
        server.send(304, "text/plain", "");  // 数据未变化，返回304 Not Modified
        return;
      }
    }

    StaticJsonDocument<256> doc;
    doc["temp_upper"] = config.tempUpper;
    doc["temp_lower"] = config.tempLower;
    doc["hum_upper"] = config.humUpper;
    doc["hum_lower"] = config.humLower;
    doc["smoke_upper"] = config.smokeUpper;
    doc["ldr_upper"] = config.ldrUpper;
    doc["ldr_lower"] = config.ldrLower;
    doc["rev"] = initRev;

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  // 实时数据API - 返回传感器实时数据（支持ETag缓存验证）
  server.on("/data/live", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    if (server.hasArg("rev")) {
      uint32_t clientRev = (uint32_t)server.arg("rev").toInt();
      if (clientRev == liveRev) {
        server.send(304, "text/plain", "");  // 数据未变化，返回304 Not Modified
        return;
      }
    }

    StaticJsonDocument<256> doc;
    doc["temperature"] = sensorData.temperature;
    doc["humidity"] = sensorData.humidity;
    doc["smoke"] = sensorData.smoke;
    doc["ldr"] = sensorData.ldr;
    doc["servo_angle"] = sensorData.servoAngle;
    doc["servo_enable"] = sensorData.servoEnable;
    doc["servo_open_angle"] = sensorData.servoOpenAngle;
    doc["alarm_active"] = sensorData.alarmActive ? 1 : 0;
    doc["ldr_mode"] = config.ldrMode;
    doc["rev"] = liveRev;

    // 根据系统状态返回对应模式字符串
    switch (systemState) {
      case STATE_AP_MODE: doc["mode"] = "AP Mode"; break;
      case STATE_WIFI_CONNECTED: doc["mode"] = "WiFi Connected"; break;
      default: doc["mode"] = "Running"; break;
    }

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  // 配置保存路由
  server.on("/save_config", HTTP_POST, handleSaveConfig);
  
  // 阈值保存路由
  server.on("/save_threshold", HTTP_POST, handleSaveThreshold);
  
  // 404错误处理
  server.onNotFound(handleNotFound);

  // 启动Web服务器
  server.begin();
  Serial.println("Web Server Started");
}




// ============================================================================
// 串口通信函数
// ============================================================================

/**
 * @brief 发送单个配置值到STM32（加入队列）
 * 
 * 将配置参数加入发送队列，由processSerialQueue()函数按顺序发送
 * 优化策略：
 * 1. 队列满时覆盖最旧项
 * 2. 相同key只保留最新值（去重）
 * 
 * @param key 配置参数键名（如"temp_upper"、"hum_lower"等）
 * @param value 配置参数值（整数）
 */
void sendConfigToSTM32(const String& key, int value) {
  // 先检查队列中是否已有相同key，有则更新
  int i = queueHead;
  while (i != queueTail) {
    if (configQueue[i].key == key) {
      configQueue[i].value = value;
      return;  // 已存在，更新值即可
    }
    i = (i + 1) % 10;
  }

  // 检查队列是否已满
  int nextTail = (queueTail + 1) % 10;
  if (nextTail == queueHead) {
    // 队列满，覆盖最旧项
    Serial.println("Config queue full, overwriting oldest");
    queueHead = (queueHead + 1) % 10;
  }

  configQueue[queueTail].key = key;
  configQueue[queueTail].value = value;
  queueTail = nextTail;
}

/**
 * @brief 处理配置发送队列
 * 
 * 从队列中取出配置项发送到STM32，一次只发送一个值
 * 等待STM32返回{com_ok}确认后再发送下一个
 * 超时时间为500ms，超时后跳过当前项
 */
void processSerialQueue() {
  // 如果正在等待响应，检查超时
  if (waitingForComOk) {
    if (millis() - comOkTimeout > 500) {
      // 超时500ms，跳过当前项
      Serial.println("ComOk timeout, skip");
      waitingForComOk = false;
      queueHead = (queueHead + 1) % 10;
    }
    return;
  }

  // 队列为空
  if (queueHead == queueTail) {
    return;
  }

  // 控制发送间隔，避免太快
  if (millis() - lastConfigSendTime < CONFIG_SEND_INTERVAL) {
    return;
  }

  // 发送下一个配置
  ConfigQueueItem& item = configQueue[queueHead];
  String json = "{\"" + item.key + "\":" + String(item.value) + "}\n";
  Serial.print(json);

  waitingForComOk = true;
  comOkTimeout = millis();
  lastConfigSendTime = millis();

  Serial.print("Sent config: ");
  Serial.println(json);
}

/**
 * @brief 发送网络信息到STM32
 * 
 * 将当前网络状态（AP模式或STA模式）和IP地址发送到STM32
 * 数据格式：{"type":"net","net_mode":"AP/WIFI","ip":"x.x.x.x"}
 */
void sendNetInfoToSTM32() {
  StaticJsonDocument<128> doc;
  doc["type"] = "net";

  if (WiFi.getMode() == WIFI_AP) {
    doc["net_mode"] = "AP";
    doc["ip"] = WiFi.softAPIP().toString();
  } else if (WiFi.status() == WL_CONNECTED) {
    doc["net_mode"] = "WIFI";
    doc["ip"] = WiFi.localIP().toString();
  } else {
    doc["net_mode"] = "AP";
    doc["ip"] = WiFi.softAPIP().toString();
  }

  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);
}

/**
 * @brief 解析来自STM32的串口数据
 * 
 * 从串口缓冲区读取数据，按行解析JSON格式的数据
 * 支持的数据类型：
 * - 传感器数据：包含温度、湿度、烟雾、光照、舵机角度
 * - 配置数据：包含阈值配置和系统状态
 * - 确认响应：{com_ok}表示配置已接收
 */
void parseSerialData() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || serialRxIndex >= SERIAL_RX_BUFFER_SIZE - 1) {
      serialRxBuffer[serialRxIndex] = '\0';
      serialRxIndex = 0;

      // 处理接收到的数据
      String line = String(serialRxBuffer);
      line.trim();

      if (line.length() > 0) {
        // 检查是否是{com_ok}响应
        if (line == "{com_ok}") {
          if (waitingForComOk) {
            waitingForComOk = false;
            queueHead = (queueHead + 1) % 10;
            Serial.println("Config acknowledged by STM32");
          }
          // 继续处理后续数据
        } else {
          // 尝试解析JSON
          StaticJsonDocument<512> doc;
          DeserializationError error = deserializeJson(doc, line);
          if (error) {
            parseConfigData(line);
          } else {
            // 检查是否是传感器数据
            if (doc.containsKey("type") && strcmp(doc["type"], "sensor_data") == 0) {
              if (doc.containsKey("temp")) sensorData.temperature = doc["temp"];
              if (doc.containsKey("hum")) sensorData.humidity = doc["hum"];
              if (doc.containsKey("smoke")) sensorData.smoke = doc["smoke"];
              if (doc.containsKey("ldr")) sensorData.ldr = doc["ldr"];
              if (doc.containsKey("servo_angle")) sensorData.servoAngle = doc["servo_angle"];
              sensorData.lastUpdate = millis();
              liveRev++;
            } else if (doc.containsKey("type") && strcmp(doc["type"], "config_data") == 0) {
              // 解析配置数据
              parseConfigDataFromJson(doc);
            }
          }
        }
      }
    } else {
      serialRxBuffer[serialRxIndex++] = c;
    }
  }
}

/**
 * @brief 解析STM32发送的配置数据（JSON格式）
 * 
 * 从JSON文档中提取配置参数并更新本地config结构
 * 包含回显检测机制：如果收到的配置数据与本地相同则忽略
 * 
 * @param doc ArduinoJson文档对象引用
 */

void parseConfigDataFromJson(StaticJsonDocument<512>& doc) {
  bool cfgChanged = false;

  // 检查是否是ESP8266自己发送的配置回显
  // 如果是回显数据，所有字段都应该与本地一致，此时不处理
  bool isEcho = true;

  if (doc.containsKey("lux_lower")) {
    int v = doc["lux_lower"];
    if (v != config.luxLower) {
      isEcho = false;
      config.luxLower = v;
      cfgChanged = true;
    }
  }
  if (doc.containsKey("pwm_low")) {
    int v = doc["pwm_low"];
    if (v != config.pwmLow) {
      isEcho = false;

/**
 * @brief 解析STM32发送的配置数据
 * 
 * 处理不带type字段的配置数据，添加{}使其成为有效JSON
 * 用于兼容通信协议
 * 
 * @param line 接收到的原始字符串
 */
void parseConfigData(const String& line) {
  // 尝试解析配置数据
  // 添加{ }使其成为有效的JSON
  String jsonStr = "{" + line;

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    return;
  }

  parseConfigDataFromJson(doc);
}


// ============================================================================
// HTML页面生成函数
// ============================================================================

/**
 * @brief 生成HTML页面框架
 * 
 * 生成包含完整HTML5结构、CSS样式和页面框架的HTML字符串
 * 
 * @param title 页面标题
 * @param content 页面内容（HTML片段）
 * @return String 完整的HTML页面字符串
 */
String generateHTMLPage(const String& title, const String& content) {
  String html;
  html.reserve(2600 + title.length() + content.length());
  html = "<!DOCTYPE html>\n";
  html += "<html lang=\"zh-CN\">\n";
  html += "<head>\n";
  html += "  <meta charset=\"UTF-8\">\n";
  html += "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n";
  html += "  <title>" + title + "</title>\n";
  html += "  <style>\n";
  html += "    :root { --bg-color: #1a1c2c; --card-bg: #2d3047; --text-primary: #ffffff; --text-secondary: #a0a0a0; --accent: #4e73df; --success: #2ecc71; --warning: #f1c40f; --danger: #e74c3c; }\n";
  html += "    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }\n";
  html += "    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; margin: 0; padding: 0; background-color: var(--bg-color); color: var(--text-primary); }\n";
  html += "    .header { background: linear-gradient(135deg, #007bff, #0056b3); padding: 20px; text-align: center; box-shadow: 0 2px 10px rgba(0,0,0,0.3); }\n";
  html += "    .header h1 { margin: 0; font-size: 1.2rem; font-weight: 500; letter-spacing: 1px; }\n";
  html += "    .container { max-width: 500px; margin: 0 auto; padding: 15px; }\n";
  html += "    .sub-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; padding: 0 5px; }\n";
  html += "    .title-group { display: flex; align-items: center; gap: 10px; }\n";
  html += "    .title-icon { background: #ff4d4d; padding: 8px; border-radius: 8px; width: 36px; height: 36px; display: flex; align-items: center; justify-content: center; }\n";
  html += "    .title-text h2 { margin: 0; font-size: 1rem; }\n";
  html += "    .time-text { font-size: 0.8rem; color: var(--text-secondary); text-align: right; }\n";
  html += "    .section-title { font-size: 1rem; margin: 20px 0 15px; display: flex; align-items: center; gap: 8px; font-weight: 500; }\n";
  html += "    .card { background: var(--card-bg); border-radius: 15px; padding: 18px; margin-bottom: 15px; position: relative; overflow: hidden; box-shadow: 0 4px 15px rgba(0,0,0,0.2); transition: transform 0.2s; }\n";
  html += "    .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }\n";
  html += "    .card-title { display: flex; align-items: center; gap: 10px; color: var(--text-secondary); font-size: 0.9rem; }\n";
  html += "    .online-tag { color: var(--success); font-size: 0.75rem; display: flex; align-items: center; gap: 4px; }\n";
  html += "    .online-dot { width: 6px; height: 6px; background: var(--success); border-radius: 50%; box-shadow: 0 0 8px var(--success); }\n";
  html += "    .card-value-row { display: flex; align-items: baseline; gap: 5px; }\n";
  html += "    .card-value { font-size: 2rem; font-weight: 600; }\n";
  html += "    .card-unit { color: var(--text-secondary); font-size: 0.9rem; }\n";
  html += "    .threshold-info { position: absolute; right: 18px; bottom: 18px; text-align: right; font-size: 0.75rem; color: var(--text-secondary); line-height: 1.4; }\n";
  html += "    .threshold-item { display: flex; align-items: center; gap: 5px; justify-content: flex-end; cursor: pointer; }\n";
  html += "    .threshold-item:hover { color: var(--text-primary); }\n";
  html += "    .switch-row { display: flex; justify-content: space-between; align-items: center; }\n";
  html += "    .switch-info { display: flex; flex-direction: column; gap: 4px; }\n";
  html += "    .switch-status { color: var(--success); font-size: 0.8rem; }\n";
  html += "    .switch { position: relative; display: inline-block; width: 52px; height: 28px; }\n";
  html += "    .switch input { opacity: 0; width: 0; height: 0; }\n";
  html += "    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #444; transition: .4s; border-radius: 34px; }\n";
  html += "    .slider:before { position: absolute; content: \"\"; height: 22px; width: 22px; left: 3px; bottom: 3px; background-color: white; transition: .4s; border-radius: 50%; }\n";
  html += "    input:checked + .slider { background-color: var(--accent); }\n";
  html += "    input:checked + .slider:before { transform: translateX(24px); }\n";
  html += "    .footer-info { margin-top: 30px; padding: 15px; background: rgba(255,255,255,0.05); border-radius: 12px; font-size: 0.75rem; color: var(--text-secondary); }\n";
  html += "    .footer-row { display: flex; justify-content: space-between; margin-bottom: 8px; }\n";
  html += "    .footer-row:last-child { margin-bottom: 0; }\n";
  html += "    .modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.8); align-items: center; justify-content: center; }\n";
  html += "    .modal-content { background: var(--card-bg); padding: 25px; border-radius: 15px; width: 90%; max-width: 400px; }\n";
  html += "    .modal-header { margin-bottom: 20px; font-size: 1.1rem; border-bottom: 1px solid #444; padding-bottom: 10px; }\n";
  html += "    input[type=\"number\"] { background: #1a1c2c; border: 1px solid #444; color: white; padding: 10px; border-radius: 8px; width: 100%; margin-bottom: 15px; }\n";
  html += "    .modal-btns { display: flex; gap: 10px; margin-top: 10px; }\n";
  html += "    .btn-modal { flex: 1; padding: 12px; border-radius: 8px; border: none; font-weight: 500; cursor: pointer; }\n";
  html += "    .btn-save { background: var(--accent); color: white; }\n";
  html += "    .btn-cancel { background: #444; color: white; }\n";
  html += "    .nav-tabs { display: flex; gap: 10px; margin-bottom: 15px; }\n";
  html += "    .nav-tab { padding: 8px 15px; border-radius: 20px; font-size: 0.8rem; background: #333; color: var(--text-secondary); text-decoration: none; }\n";
  html += "    .nav-tab.active { background: var(--accent); color: white; }\n";
  html += "  </style>\n";
  html += "</head>\n";
  html += "<body>\n";
  html += "  <div class=\"header\">\n";
  html += "    <h1>" + title + "</h1>\n";
  html += "  </div>\n";
  html += "  <div class=\"container\">\n";
  html += "    " + content + "\n";
  html += "  </div>\n";
  html += "</body>\n";
  html += "</html>";
  return html;
}

/**
 * @brief 生成网络配置页面HTML
 * 
 * 生成包含WiFi配置表单的HTML页面
 * 
 * @return String 配置页面HTML字符串
 */
String generateConfigPage() {
  String content = "<div class=\"nav-tabs\">\n";
  content += "  <a href=\"/\" class=\"nav-tab\">首页监控</a>\n";
  content += "  <a href=\"/config\" class=\"nav-tab active\">网络设置</a>\n";
  content += "</div>\n";

  content += "<div class=\"card\">\n";
  content += "  <div class=\"card-title\">📡 WiFi 设置</div>\n";
  content += "  <form action=\"/save_config\" method=\"post\">\n";
  content += "    <label style=\"display:block;margin:10px 0 5px;font-size:0.8rem;color:var(--text-secondary)\">WiFi 名称 (SSID):</label>\n";
  content += "    <input type=\"text\" name=\"wifi_ssid\" value=\"" + config.wifiSSID + "\" required>\n";
  content += "    <label style=\"display:block;margin:10px 0 5px;font-size:0.8rem;color:var(--text-secondary)\">WiFi 密码:</label>\n";
  content += "    <input type=\"password\" name=\"wifi_password\" value=\"" + config.wifiPassword + "\">\n";
  content += "    <button type=\"submit\" class=\"btn-modal btn-save\" style=\"margin-top:10px;width:100%\">保存 WiFi 设置</button>\n";
  content += "  </form>\n";
  content += "</div>\n";


  return generateHTMLPage("网络配置", content);
}