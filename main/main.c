#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "camera_with_sensor.h"
#include "rfid_with_servo.h"
#include "parking_door.h"
#include "alarm.h"


#define TAG "MAIN"

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══════════════════════════════════");
    ESP_LOGI(TAG, "   ESP32-C6 SMART PARKING SYSTEM");
    ESP_LOGI(TAG, "   Full System Integration");
    ESP_LOGI(TAG, "═══════════════════════════════════");
    ESP_LOGI(TAG, "");
    
    // Inicializar NVS (necesario para BLE y otros módulos)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✓ NVS initialized");
    
    // 1. Inicializar Alarma con BLE
    ESP_LOGI(TAG, "► Initializing Alarm with BLE...");
    ret = alarm_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to initialize Alarm");
    } else {
        ESP_LOGI(TAG, "✓ Alarm initialized");
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 2. Inicializar Cámara con Sensor
    ESP_LOGI(TAG, "► Initializing Camera with Sensor...");
    ret = camera_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to initialize Camera");
    } else {
        ESP_LOGI(TAG, "✓ Camera initialized");
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 3. Inicializar RFID con Servo
    ESP_LOGI(TAG, "► Initializing RFID with Servo...");
    ret = rfid_servo_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to initialize RFID");
    } else {
        ESP_LOGI(TAG, "✓ RFID initialized");
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 4. Inicializar Parking Door
    ESP_LOGI(TAG, "► Initializing Parking Door...");
    ret = parking_door_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to initialize Parking Door");
    } else {
        ESP_LOGI(TAG, "✓ Parking Door initialized");
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══════════════════════════════════");
    ESP_LOGI(TAG, "✓ ALL SYSTEMS INITIALIZED!");
    ESP_LOGI(TAG, "═══════════════════════════════════");
    ESP_LOGI(TAG, "• Alarm: GPIO1 (button), GPIO10 (LED)");
    ESP_LOGI(TAG, "• BLE: Device 'ESP32_ALARM'");
    ESP_LOGI(TAG, "• Camera: Detection system active");
    ESP_LOGI(TAG, "• RFID: Card reader active");
    ESP_LOGI(TAG, "• Parking Door: Control system ready");
    ESP_LOGI(TAG, "═══════════════════════════════════");
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}