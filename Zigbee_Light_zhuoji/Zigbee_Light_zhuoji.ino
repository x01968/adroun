/**
 * @file Zigbee_String_Sender.ino
 * @brief Zigbee字符串发送器（协调器）
 */

#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee coordinator mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_zigbee_core.h"            // ESP Zigbee核心协议栈
#include "freertos/FreeRTOS.h"          // FreeRTOS实时操作系统
#include "freertos/task.h"              // FreeRTOS任务管理
#include "ha/esp_zigbee_ha_standard.h"  // Zigbee家庭自动化(HA)标准库
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_endpoint.h"
#include "esp_zigbee_attribute.h"

// 自定义集群配置
#define LDR_ENDPOINT 20                 // 可以改成其他端点号，比如21、22等
#define LDR_CLUSTER_ID 0xFF00           //  这是自定义集群ID
#define STRING_CMD_ID 0x0004            // 自定义字符串命令ID
#define HA_ONOFF_SWITCH_ENDPOINT 1

// 全局变量
volatile uint16_t g_bulb_short_addr = 0;
volatile bool g_zigbee_ready = false;

// Zigbee配置宏
#define ESP_ZB_ZC_CONFIG()                                      \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,          \
        .install_code_policy = false,                           \
        .nwk_cfg = {                                            \
            .zczr_cfg = {                                       \
                .max_children = 10,                             \
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

/**
 * @brief 发送字符串到从机函数
 * @details 通过Zigbee网络将字符串发送到已连接的从机设备
 * 
 * @param str 要发送的字符串，格式为"{内容}"
 * @return 无返回值
 * 
 * 函数流程：
 * 1. 检查从机是否已连接
 * 2. 验证字符串长度（最大100字符）
 * 3. 构建Zigbee自定义集群命令
 * 4. 发送字符串到从机
 * 5. 打印发送状态
 */
void send_string_to_slave(const char* str) {
    if (!g_zigbee_ready || g_bulb_short_addr == 0) {
        Serial.println("Error: 从机未连接！");
        return;
    }

    uint16_t str_len = strlen(str);
    if (str_len > 100) {
        Serial.println("Error: 字符串过长！");
        return;
    }

    esp_zb_zcl_custom_cluster_cmd_req_t req = {};
    req.zcl_basic_cmd.src_endpoint = LDR_ENDPOINT;
    req.zcl_basic_cmd.dst_endpoint = LDR_ENDPOINT;
    req.zcl_basic_cmd.dst_addr_u.addr_short = g_bulb_short_addr;
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
    
    Serial.printf("已发送: %s\n", str);
}

// 串口任务
void serial_task(void *pvParameters) {
    Serial.println("字符串发送器就绪，请输入以{}包围的字符串：");
    
    while (1) {
        while (Serial.available() > 0) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) continue;

            if (line.startsWith("{") && line.endsWith("}")) {
                send_string_to_slave(line.c_str());
            } else {
                Serial.println("格式错误！请使用 {内容} 格式");
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Zigbee应用信号处理函数
 * @details 处理Zigbee协议栈发送的各种事件信号
 * 
 * 函数流程：
 * 1. 获取事件类型和状态
 * 2. 根据事件类型处理不同情况
 * 3. 更新全局变量
 * 4. 打印调试信息
- 这个函数在 `esp_zb_task()` 函数中被注册了

- `esp_zb_core_action_handler_register(zb_action_handler);` 注册了动作处理函数

- `esp_zb_device_register(esp_zb_on_off_switch_ep);` 注册了设备

- 当有事件发生时，Zigbee协议栈会自动调用这个函数

 */

// Zigbee应用信号处理
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;           // 事件指针
    esp_err_t err_status = signal_struct->esp_err_status;     // 事件状态（成功/失败）
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)(*p_sg_p);
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        Serial.println("Zigbee栈初始化完成");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            Serial.println("开始组建网络");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            Serial.printf("网络组建成功 (PAN ID: 0x%04x)\n", esp_zb_get_pan_id());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        Serial.printf("发现新设备: 0x%04x\n", dev_annce_params->device_short_addr);
        g_bulb_short_addr = dev_annce_params->device_short_addr;  // 保存从机地址
        g_zigbee_ready = true;  // 设置网络就绪标志
        Serial.printf("从机已连接，地址: 0x%04x\n", g_bulb_short_addr);
        break;
    default:
        break;
    }
}

/**
 * @brief Zigbee动作处理函数 - 接收从机发送的字符串
 * @details 处理从机通过自定义集群发送的字符串消息
 * 
 * @param callback_id 回调ID，用于识别消息类型
 * @param message 消息内容
 * @return esp_err_t 处理结果
 * 
 * 函数流程：
 * 1. 检查回调ID是否为自定义集群请求
 * 2. 验证消息是否来自自定义集群
 * 3. 检查命令ID是否为字符串命令
 * 4. 提取字符串内容并打印
 * 5. 显示发送方地址
 */
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

                char str_buf[128];
                uint16_t copy_len = (m->data.size < sizeof(str_buf) - 1) ?
                                    m->data.size : sizeof(str_buf) - 1;
                memcpy(str_buf, m->data.value, copy_len);
                str_buf[copy_len] = '\0';

                Serial.printf("收到从机字符串: %s (来自 0x%04x)\n",
                             str_buf, m->info.src_address.u.short_addr);
            }
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

// Zigbee主任务
void esp_zb_task(void *pvParameters) {
    // 初始化Zigbee平台配置
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // 创建开关设备
    esp_zb_on_off_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_switch_ep = 
        esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &switch_cfg);

// 添加自定义集群
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
    esp_zb_ep_list_add_ep(esp_zb_on_off_switch_ep, cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_on_off_switch_ep);

    // 注册动作处理函数来接收从机消息
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);// 可以改成特定信道，比如0x00000800（只扫描信道11）
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();

}


void setup() {
    Serial.begin(115200);
    Serial.println("Zigbee字符串发送器启动");

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(serial_task, "serial_task", 2048, NULL, 3, NULL);
}


void loop() {
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
}