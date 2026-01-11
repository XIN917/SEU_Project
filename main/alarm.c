#include "alarm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "ALARM";

/* GPIO configuration */
#define ALARM_BUTTON_GPIO 1
#define ALARM_LED_GPIO    10

/* BLE GATT Server Configuration */
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4
#define DEVICE_NAME          "ESP32_ALARM"
#define ADV_CONFIG_FLAG      (1 << 0)

static volatile uint32_t blink_phase_ms = 250; // Time per phase (half of full cycle)
static TaskHandle_t alarm_task_handle = NULL;
static bool alarm_state = false;

/* BLE GATT Server Variables */
static uint8_t adv_config_done = 0;
static uint16_t gatts_if_notify = ESP_GATT_IF_NONE;
static uint16_t conn_id_notify = 0xFFFF;
static uint16_t char_handle_notify = 0;
static bool is_ble_connected = false;

static uint8_t service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Send BLE notification with alarm state
static void send_ble_notification(const char *message) {
    if (!is_ble_connected || gatts_if_notify == ESP_GATT_IF_NONE || char_handle_notify == 0) {
        ESP_LOGW(TAG, "BLE not connected - notification not sent");
        return;
    }

    esp_ble_gatts_send_indicate(gatts_if_notify, conn_id_notify, char_handle_notify,
                                strlen(message), (uint8_t*)message, false);
    
    ESP_LOGI(TAG, "📱 BLE Notification sent: %s", message);
}

static void alarm_task(void *arg)
{
    bool alarm_on = false;
    int last_button_level = 1;
    int stable_count = 0;
    const int debounce_reads = 5;
    const TickType_t delay_tick = pdMS_TO_TICKS(10);

    gpio_set_level(ALARM_LED_GPIO, 0);

    ESP_LOGI(TAG, "Alarm task started (button on GPIO%d)", ALARM_BUTTON_GPIO);

    while (1) {
        int level = gpio_get_level(ALARM_BUTTON_GPIO);

        if (level == last_button_level) {
            if (stable_count < debounce_reads) stable_count++;
        } else {
            stable_count = 0;
            last_button_level = level;
        }

        // Detect button press
        if (stable_count >= debounce_reads && level == 0) {
            alarm_state = !alarm_state;
            ESP_LOGI(TAG, "🔔 Alarm %s (button pressed)", alarm_state ? "ON" : "OFF");
            
            send_ble_notification(alarm_state ? "ALARM ON" : "ALARM OFF");
            
            while (gpio_get_level(ALARM_BUTTON_GPIO) == 0) {
                vTaskDelay(delay_tick);
            }
            ESP_LOGI(TAG, "Button released");
            vTaskDelay(pdMS_TO_TICKS(50)); // Anti-bounce after release
            stable_count = 0;
            last_button_level = 1;
        }

        if (alarm_state) {
            gpio_set_level(ALARM_LED_GPIO, 1); 
            vTaskDelay(pdMS_TO_TICKS(blink_phase_ms));
            
            gpio_set_level(ALARM_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(blink_phase_ms));
        } else {
            // Ensure LED is off
            gpio_set_level(ALARM_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// BLE GAP Event Handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "✓ BLE Advertising started - Device visible");
        } else {
            ESP_LOGE(TAG, "Advertising start failed");
        }
        break;
    default:
        break;
    }
}

// BLE GATT Server Event Handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                               esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "✓ GATT Server registered");
        
        esp_ble_gap_set_device_name("ESP32_ALARM");
        esp_ble_gap_config_adv_data(&adv_data);
        
        // Create service
        esp_gatt_srvc_id_t service_id;
        service_id.is_primary = true;
        service_id.id.inst_id = 0;
        service_id.id.uuid.len = ESP_UUID_LEN_16;
        service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        break;
        
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "✓ Service created");
        esp_ble_gatts_start_service(param->create.service_handle);
        
        // Add characteristic with notification
        esp_bt_uuid_t char_uuid;
        char_uuid.len = ESP_UUID_LEN_16;
        char_uuid.uuid.uuid16 = GATTS_CHAR_UUID;
        
        esp_gatt_perm_t perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
        esp_gatt_char_prop_t property = ESP_GATT_CHAR_PROP_BIT_READ | 
                                        ESP_GATT_CHAR_PROP_BIT_WRITE |
                                        ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        
        esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                               perm, property, NULL, NULL);
        break;
        
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "✓ Characteristic added");
        char_handle_notify = param->add_char.attr_handle;
        gatts_if_notify = gatts_if;
        
        // Add CCC descriptor (required for notifications)
        esp_bt_uuid_t descr_uuid;
        descr_uuid.len = ESP_UUID_LEN_16;
        descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(param->add_char.service_handle, &descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;
        
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "*** 📱 BLE Client Connected ***");
        is_ble_connected = true;
        conn_id_notify = param->connect.conn_id;
        gatts_if_notify = gatts_if;
        break;
    
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(TAG, "✓ Descriptor added (notifications enabled)");
        break;
        
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "📱 BLE Client Disconnected");
        is_ble_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;
        
    default:
        break;
    }
}

esp_err_t alarm_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "ESP32-C6 BLE Alarm System");
    ESP_LOGI(TAG, "====================================");

    // Release BT Classic memory
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ BT controller initialized");

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ BT controller enabled");

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Init bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ Bluedroid initialized");

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Enable bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ Bluedroid enabled");

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ GATTS callback registered");

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register error: %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "✓ GAP callback registered");

    // Set device name
    ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    if (ret) {
        ESP_LOGE(TAG, "Set device name failed: %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "✓ Device name set: %s", DEVICE_NAME);

    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ GATTS app registered");

    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ Local MTU set to 500");
    
    // Configure button as input with pull-up
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << ALARM_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);
    ESP_LOGI(TAG, "✓ Button GPIO%d configured", ALARM_BUTTON_GPIO);
    
    // Configure LEDs as output
    gpio_reset_pin(ALARM_LED_GPIO);
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << ALARM_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);
    
    // Force LED to OFF
    gpio_set_level(ALARM_LED_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ALARM_LED_GPIO, 0);
    ESP_LOGI(TAG, "✓ LED GPIO%d configured (OFF)", ALARM_LED_GPIO);

    ESP_LOGI(TAG, "✓ BLE initialized and ready");

    if (alarm_task_handle == NULL) {
        BaseType_t res = xTaskCreate(alarm_task, "alarm_task", 3072, NULL, 5, &alarm_task_handle);
        if (res != pdPASS) {
            ESP_LOGE(TAG, "Failed to create alarm task");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "✓ Alarm task created");
    }

    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "✓ Alarm system ready!");
    ESP_LOGI(TAG, "  Device: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "  Button: GPIO%d", ALARM_BUTTON_GPIO);
    ESP_LOGI(TAG, "  LED:    GPIO%d", ALARM_LED_GPIO);
    ESP_LOGI(TAG, "====================================");
    
    return ESP_OK;
}

void alarm_set_period_ms(uint32_t period_ms)
{
    if (period_ms < 50) period_ms = 50;
    blink_phase_ms = period_ms / 2;
    ESP_LOGI(TAG, "Blink period set to %lu ms (phase: %lu ms)", period_ms, blink_phase_ms);
}
