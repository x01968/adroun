/**
 * @file congji.ino
 * @brief 从机（终端设备）- 字符串双向传输
 * @details 接收主机发送的字符串并打印，也能通过串口输入发送字符串给主机
 */
#define DEBUG 1
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_endpoint.h"
#include "esp_zigbee_attribute.h"
#include <ArduinoJson.h>

#include "Preferences.h"  // 断电保存库
// ===================== 硬件配置宏定义 =====================
#define LDR_ADC_PIN 3                // 光敏电阻ADC引脚 (GPIO3)
#define PIR_PIN 19                   // PIR传感器引脚 (GPIO4)
#define LED_PIN 15                   // LED控制引脚 (GPIO15)
#define LDR_SEND_INTERVAL_MS 1000    // LDR数据发送间隔 (1秒)
#define PIR_SEND_INTERVAL_MS 500     // PIR数据发送间隔 (0.5秒)
#define ADC_SAMPLE_COUNT 10          // ADC采样数量
#define PIR_SAMPLE_COUNT 5           // PIR采样数量
#define ADC_FILTER_WINDOW 5         // ADC滑动窗口大小
#define LUX_LOWER_BOUND 1000         // 光照下限阈值（暗光）
#define LUX_UPPER_BOUND 3000         // 光照上限阈值（正常光）
// ===================== 自定义集群配置 =====================
#define LDR_ENDPOINT 20
#define LDR_CLUSTER_ID 0xFF00
#define STRING_CMD_ID 0x0004
#define HA_ESP_LIGHT_ENDPOINT 10
#define COORD_SHORT_ADDR 0x0000  // 协调器短地址


// ===================== 全局变量 =====================
volatile uint8_t g_last_pir = 0;        // 最后PIR状态
volatile uint16_t g_last_ldr = 0;       // 最后LDR值
volatile uint8_t g_last_led = 0;        // 最后LED状态
volatile uint8_t g_last_pwm = 128;      // 最后PWM值
volatile bool g_manual_override = false; // 手动覆盖标志
volatile uint32_t g_target_countdown_ms = 5000; // 目标倒计时毫秒数
volatile uint32_t g_countdown_start_time = 0; // 倒计时开始时间
volatile bool g_countdown_active = false; // 倒计时是否激活
volatile uint16_t g_lux_lower = LUX_LOWER_BOUND; // 光照下限阈值
volatile uint16_t g_lux_upper = LUX_UPPER_BOUND; // 光照上限阈值
static volatile bool cmd_return = false;//收到指令标志
static volatile bool g_zb_joined = false;
static volatile uint16_t g_coord_addr = COORD_SHORT_ADDR;

// 断电保存相关变量
Preferences preferences;
static uint8_t saved_pwm_value = 128;  // 默认PWM值
static uint16_t saved_lux_lower = LUX_LOWER_BOUND;//光照中间阈值
static uint16_t saved_lux_upper = LUX_UPPER_BOUND;
static uint8_t saved_pwm_low = 5;      // LED微亮亮度PWM值
static uint8_t saved_pwm_upp = 254;    // LED全亮亮度PWM值

char serial_rx_buffer[256];          // 串口接收缓冲区

// ===================== Zigbee配置宏 =====================
#define ESP_ZB_ZED_CONFIG()                                     \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                   \
        .install_code_policy = false,                           \
        .nwk_cfg = {                                            \
            .zed_cfg = {                                        \
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,    \
                .keep_alive = 3000,                             \
            },                                                  \
        },                                                      \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

// ===================== 发送字符串到主机 =====================
void send_string_to_coordinator(const char* str) {
    if (!g_zb_joined) {
        Serial.println("Error: 未加入网络！");
        return;
    }

    uint16_t str_len = strlen(str);
    if (str_len > 100) {
        Serial.println("Error: 字符串过长（最大100字符）");
        return;
    }

    esp_zb_zcl_custom_cluster_cmd_req_t req = {};
    req.zcl_basic_cmd.src_endpoint = LDR_ENDPOINT;
    req.zcl_basic_cmd.dst_endpoint = LDR_ENDPOINT;
    req.zcl_basic_cmd.dst_addr_u.addr_short = g_coord_addr;
    req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_id = LDR_CLUSTER_ID;
    req.custom_cmd_id = STRING_CMD_ID;
    req.data.type = ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING;
    req.data.size = str_len;
    req.data.value = (uint8_t*)str;

    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t tsn = esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();
    
    #if DEBUG
    Serial.printf("已发送到主机: %s (tsn=%u)\n", str, tsn);
    #endif
}

// ===================== 串口任务 - 处理从机串口输入 =====================
void serial_task(void *pvParameters) {

    char data_packet[128];
    TickType_t last_led_tick = xTaskGetTickCount();
    TickType_t last_lux_tick = xTaskGetTickCount();
    const TickType_t led_interval = 1000 / portTICK_PERIOD_MS;      // 1秒
    const TickType_t lux_interval = 10000 / portTICK_PERIOD_MS;    // 10秒
    while (1) {
        TickType_t now = xTaskGetTickCount();
        if (g_zb_joined) { 
            // 每1秒发送一次led/ldr/pir/pwm数据包
            if (now - last_led_tick >= led_interval) {
                snprintf(data_packet, sizeof(data_packet),
                    "{led:%d,ldr:%d,pir:%d,pwm:%d}",
                    g_last_led, g_last_ldr, g_last_pir, g_last_pwm);
                send_string_to_coordinator(data_packet);
                last_led_tick = now;
            }

            // 每10秒发送一次lux_lower/pwm_low/pwm_upp数据包
            if (now - last_lux_tick >= lux_interval) {
                snprintf(data_packet, sizeof(data_packet),
                    "{lux_lower:%d,pwm_low:%d,pwm_upp:%d}",
                    g_lux_lower, saved_pwm_low, saved_pwm_upp);
                send_string_to_coordinator(data_packet);
                last_lux_tick = now;
            }
            if(cmd_return==1){
            cmd_return=0;
            send_string_to_coordinator("{cmd_OK}");
            }
        }    
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

                char str_buf[128];
                int a;
// ===================== Zigbee动作处理 =====================
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    esp_err_t ret = ESP_OK;
    
    switch (callback_id) {
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID: {
        const esp_zb_zcl_custom_cluster_command_message_t *m = 
            (const esp_zb_zcl_custom_cluster_command_message_t *)message;
        
        if (m && m->info.cluster == LDR_CLUSTER_ID) {
            // 处理字符串命令
            if ((uint16_t)m->info.command.id == (STRING_CMD_ID & 0xFF) && 
                m->data.value && m->data.size > 0) {
                uint16_t copy_len = (m->data.size < sizeof(str_buf) - 1) ?
                                    m->data.size : sizeof(str_buf) - 1;
                memcpy(str_buf, m->data.value, copy_len);
                a=1;
                // 打印接收到的字符串
                Serial.printf("%s\n",str_buf);
                #if DEBUG
                Serial.printf("收到字符串: %s (来自 0x%04x)\n", 
                             str_buf, m->info.src_address.u.short_addr);
                #endif
            }
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

static void usart_task(void *pvParameters)
{

    while (1) {
    if(a==1){    
      // 创建缓冲区存储字符串
      a=0;
        char cmd[20];
        int val = 0;
        
        // 使用sscanf解析JSON格式字符串
        if (sscanf(str_buf, "{\"cmd\":\"%19[^\"]\",\"val\":%d}", cmd, &val) == 2) {
            // 成功解析
            if (strcmp(cmd, "light_mid") == 0 && val >= 0 && val <= 3000) {
                g_lux_lower = val;
                Serial.printf("设置光照阈值: %d\n", g_lux_lower);
            }
            else if (strcmp(cmd, "pwm_low") == 0 && val >= 0 && val <= 255) {
                saved_pwm_low = val;
                Serial.printf("设置微亮PWM: %d\n", saved_pwm_low);
            }
            else if (strcmp(cmd, "pwm_high") == 0 && val >= 0 && val <= 255) {
                saved_pwm_upp = val;
                Serial.printf("设置全亮PWM: %d\n", saved_pwm_upp);
            }
            else if (strcmp(cmd, "manual") == 0 && ( val == 1)) {
                g_manual_override = val;
                Serial.printf("%s\n", val);
                Serial.printf("切换到手动模式");
            }
            else {
                Serial.printf("无效命令或数值: cmd=%s, val=%d\n", cmd, val);
            }

        } else {
            Serial.println("解析失败: 格式错误");
        } 
        cmd_return=1;
      }
      vTaskDelay(LDR_SEND_INTERVAL_MS / portTICK_PERIOD_MS);// 控制数据采集的频率，避免过于频繁
    }
}











// 重试入网回调函数实现
static void retry_join_network(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

// ===================== Zigbee应用信号处理 =====================
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)(*p_sg_p);

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        Serial.println("Zigbee栈初始化完成");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            Serial.println("开始入网扫描");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            Serial.printf("初始化失败: %s\n", esp_err_to_name(err_status));
            
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            g_zb_joined = true;
            Serial.printf("入网成功 (我的短地址: 0x%04x, 协调器地址: 0x%04x)\n", 
                         esp_zb_get_short_address(), g_coord_addr);
        } else {
            Serial.printf("入网失败: %s\n", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)retry_join_network,ESP_ZB_BDB_MODE_NETWORK_STEERING, 200);
        }
        break;

    default:
        break;
    }
}

// ===================== Zigbee主任务 =====================
void esp_zb_task(void *pvParameters) {
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // 创建灯泡设备
    esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_dimmable_light_ep = 
        esp_zb_color_dimmable_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);

    // 添加自定义集群用于字符串接收和发送
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(LDR_CLUSTER_ID);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = LDR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(esp_zb_dimmable_light_ep, cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_dimmable_light_ep);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}
/**
 * @brief ADC滑动平均滤波
 * @param new_value 新的ADC采样值
 * @return 滤波后的ADC值
 */
static uint16_t adc_filter(uint16_t new_value) {
  static uint16_t buffer[ADC_FILTER_WINDOW] = { 0 };
  static uint8_t index = 0;
  static bool initialized = false;

  buffer[index] = new_value;
  index = (index + 1) % ADC_FILTER_WINDOW;

  if (!initialized && index > 0) {
    initialized = true;
  }

  if (!initialized) {
    return new_value;
  }

  uint32_t sum = 0;
  for (int i = 0; i < ADC_FILTER_WINDOW; i++) {
    sum += buffer[i];
  }

  return sum / ADC_FILTER_WINDOW;
}
/**
 * @brief 读取滤波后的LDR值
 * @return 滤波后的ADC值 (0-4095)
 */
static uint16_t read_ldr_filtered() {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
    sum += analogRead(LDR_ADC_PIN);
    delay(5);
  }
  uint16_t raw_value = sum / ADC_SAMPLE_COUNT;

  uint16_t filtered_value = adc_filter(raw_value);

  // 限制ADC值范围在0-4095之间
  if (filtered_value > 4095) {
    filtered_value = 4095;
  }

  // LDR的值改为3000-检测值，这样保证光强和测得的值正相关
  // 但要确保结果不会超出合理范围
  int32_t result = 3300 - filtered_value;
  if (result < 0) {
    result = 0;
  } else if (result > 3300) {
    result = 3300;
  }

  return (uint16_t)result;
}
/**
 * @brief 光照传感器（LDR）数据采集任务函数
 *
 * 功能：周期性地读取光敏电阻的ADC值，进行滤波处理后，通过串口发送给ESP8266。
 */
static void ldr_task(void *pvParameters)
{
    analogReadResolution(12);//ADC初始化配置，设置ADC采样分辨率为12位，对应ADC值范围为0-4095
    pinMode(LDR_ADC_PIN, INPUT);// 引脚模式初始化
    while (1) {
        
       if (g_zb_joined) { 
            uint16_t ldr_value = read_ldr_filtered();// 数据采集与滤波处理
            g_last_ldr = ldr_value;// 更新全局变量
            #if DEBUG
            Serial.printf("LDR:%u\n", ldr_value);// 调试信息输出
            #endif
       }
        vTaskDelay(LDR_SEND_INTERVAL_MS / portTICK_PERIOD_MS);// 控制数据采集的频率，避免过于频繁
    }
}

/**
 * @brief PIR多数表决滤波
 * @return 滤波后的PIR状态 (0=无人, 1=有人)
 */
static uint8_t read_pir_filtered() {
  uint8_t state_sum = 0;
  for (int i = 0; i < PIR_SAMPLE_COUNT; i++) {
    state_sum += digitalRead(PIR_PIN);
    delay(10);
  }
  return (state_sum >= (PIR_SAMPLE_COUNT / 2 + 1)) ? 1 : 0;
}
/**
 * @brief PIR传感器任务函数
 * @param pvParameters FreeRTOS任务参数（此处未使用）
 * @note 运行在FreeRTOS任务中，负责PIR传感器数据采集
 * 核心功能：
 * 1. 初始化PIR传感器引脚（使用内置下拉电阻）
 * 2. 读取滤波后的PIR状态
 * 3. 更新全局PIR状态变量供LED控制任务使用
 */
static void pir_task(void *pvParameters) {
    // 1. 初始化PIR输入引脚为下拉输入，避免开机或空闲时引脚浮空
    pinMode(PIR_PIN, INPUT_PULLDOWN);  // 使用内置下拉电阻，解决引脚悬空导致的1/0交变不稳定问题

    // 2. 记录上次PIR状态，初始假定无人
    static uint8_t last_pir_state = 0;

    while (1) {
    if (g_zb_joined) { 
        // 读取滤波后的PIR状态
        uint8_t current_pir_state = read_pir_filtered();
        // 存储全局PIR状态，供led_control_task等使用
        g_last_pir = current_pir_state;
        // 只有当状态发生变化时才打印调试信息
        if (current_pir_state != last_pir_state) {
        last_pir_state = current_pir_state;
        #if DEBUG
        Serial.printf("PIR state changed: %u\n", current_pir_state);
        }
        #endif
    }
    vTaskDelay(PIR_SEND_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 设置LED亮度（带淡入淡出效果）
 * @param target_pwm 目标PWM值 (0-254)
 * @param fade_time_ms 淡入/淡出时间 (ms)
 */
static void set_led_brightness(uint8_t target_pwm, uint16_t fade_time_ms) {
  uint8_t current_pwm = g_last_pwm;

  // 立即设置目标亮度，避免卡死
  analogWrite(LED_PIN, target_pwm);
  g_last_pwm = target_pwm;

  // 如果需要淡入淡出效果
  if (fade_time_ms > 0 && target_pwm != current_pwm) {
    int16_t step = (target_pwm > current_pwm) ? 1 : -1;
    uint16_t steps = abs(target_pwm - current_pwm);
    uint16_t delay_ms = fade_time_ms / steps;

    for (uint16_t i = 0; i <= steps; i++) {
      uint8_t pwm = current_pwm + (step * i);
      analogWrite(LED_PIN, pwm);
      delay(delay_ms);
    }

    g_last_pwm = target_pwm;
  }
}
/**
 * @brief LED控制任务函数
 * @param pvParameters FreeRTOS任务参数（此处未使用）
 * @note 运行在FreeRTOS任务中，负责LED照明控制逻辑
 * 核心功能：
 * 1. 根据当前模式执行不同的照明控制逻辑
 * 2. 处理倒计时逻辑和LED控制
 * 3. 与PIR传感器任务分离，独立控制LED
 */
static void led_control_task(void *pvParameters)
{
  // 静态变量，保持状态跨循环
  static uint8_t current_mode = 0;             // 当前模式 (0=自动, 1=手动, 2=夜灯)
  static uint32_t last_trigger_time = 0;       // 最后触发时间
  static uint32_t countdown_last_print = 0;    // 上次打印倒计时时间
  static bool light_on = false;                // 灯是否开启
  while (1) {
    if (g_zb_joined) { 
        // 主卧照明控制逻辑
        //手动模式：APP/Web控制LED亮灭，传感器数据仅上报不控制
        if(g_manual_override){
          g_manual_override=0;
            //设置返回点亮成功
        g_countdown_start_time = millis();
        g_target_countdown_ms = 5000;//手动模式默认5秒
        g_last_led = 3;//手动模式特殊状态
        set_led_brightness(saved_pwm_upp, 300); // 全亮
        }
        //自动模式：根据PIR和LDR联动控制LED，PIR触发后根据LDR值设置不同的倒计时和亮度
        if (g_last_pir == 1&&g_manual_override==0&&g_last_led != 3) {
        // 有人存在，一直刷新倒计时
        g_countdown_start_time = millis();
        g_countdown_active = true;
        set_led_brightness(saved_pwm_upp, 300); // 全亮
        g_last_led = 1;//全亮
        if (g_last_ldr < saved_lux_lower) {
            // 弱光环境：有人来强光2秒
            g_target_countdown_ms = 2000;
        } else if (g_last_ldr >= saved_lux_lower && g_last_ldr < 3000) {
            // 强光环境：有人来亮5秒
            g_target_countdown_ms = 5000;
        }
        }
        if(g_target_countdown_ms > 0){
        uint32_t elapsed = millis() - g_countdown_start_time;
        if (elapsed >= 1000 && (elapsed / 1000) != (countdown_last_print / 1000)) {//每秒打印一次倒计时
            #if DEBUG
            if(g_target_countdown_ms>elapsed)            
            Serial.printf("倒计时剩余：%us\n", (g_target_countdown_ms - elapsed) / 1000);
            else
            Serial.printf("倒计时剩余：%us\n", 0);
            #endif
            countdown_last_print = elapsed;
        }
        if (elapsed >= g_target_countdown_ms) {//倒计时结束
            // 恢复自动默认状态
            if (g_last_ldr < saved_lux_lower) {
                set_led_brightness(saved_pwm_low, 300); // 弱光默认微量PWM=5
                g_last_led = 2;//微亮
            } else if (g_last_ldr >= saved_lux_lower && g_last_ldr < 3000) {
                set_led_brightness(0, 300); // 强光默认不亮
                g_last_led = 0;
            }
            g_target_countdown_ms = 0;
            g_manual_override = false; // 清除手动覆盖
            Serial.println("倒计时结束");
        }
        } else {
        // 无倒计时，保持默认状态
        if (g_last_ldr < saved_lux_lower) {
            set_led_brightness(saved_pwm_low, 0); // 弱光默认微量PWM=5
            g_last_led = 2;//微亮
        } else if (g_last_ldr >= saved_lux_lower && g_last_ldr < 3000) {
            set_led_brightness(0, 0); 
            g_last_led = 0;//关闭
        }
        }
    }
    // 延迟100ms后继续下一次循环
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
// ===================== Arduino标准函数 =====================
void setup() {
    Serial.begin(115200);
    Serial.println("=== 从机（双向字符串传输）启动 ===");

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // 初始化Preferences（断电保存）
    preferences.begin("bedroom_config", false);
    saved_pwm_value = preferences.getUInt("saved_pwm", 128);
    saved_lux_lower = preferences.getUInt("saved_lux_lower", LUX_LOWER_BOUND);
    saved_lux_upper = preferences.getUInt("saved_lux_upper", LUX_UPPER_BOUND);
    saved_pwm_low = preferences.getUInt("saved_pwm_low", 5);
    saved_pwm_upp = preferences.getUInt("saved_pwm_upp", 254);
    preferences.end();
    Serial.println("从Preferences加载配置:");
    Serial.printf("  PWM: %u\n", saved_pwm_value);
    Serial.printf("  光照下限: %u\n", saved_lux_lower);
    Serial.printf("  光照上限: %u\n", saved_lux_upper);
    Serial.printf("  微亮PWM: %u\n", saved_pwm_low);
    Serial.printf("  全亮PWM: %u\n", saved_pwm_upp);
    // 初始化LED引脚（输出模式），设置初始状态
    pinMode(LED_PIN, OUTPUT);
    pinMode(LDR_ADC_PIN, INPUT);
    pinMode(PIR_PIN, INPUT_PULLDOWN);

    // 设置初始LED状态
    analogWrite(LED_PIN, 0);
    g_last_led = 0;
    g_last_pwm = 0;

    xTaskCreate(serial_task, "serial_task", 2048, NULL, 3, NULL);
    xTaskCreate(usart_task, "usart_task", 2048, NULL, 3, NULL);
    xTaskCreate(ldr_task, "ldr_task", 2048, NULL, 3, NULL);
    xTaskCreate(pir_task, "pir_task", 2048, NULL, 3, NULL);
    xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 2, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}


