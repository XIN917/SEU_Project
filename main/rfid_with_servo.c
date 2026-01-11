#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rfid_with_servo.h"

#define TAG "RFID_SERVO"

/* Pines SPI */
#define RFID_MISO 18
#define RFID_MOSI 19
#define RFID_SCK  20
#define RFID_CS   21
#define RFID_RST  9

/* Servo */
#define SERVO_GPIO     23
#define SERVO_FREQ_HZ  50
#define SERVO_TIMER    LEDC_TIMER_1
#define SERVO_CHANNEL  LEDC_CHANNEL_1

/* Tarjetas autorizadas */
static const uint8_t AUTH_UID1[4] = { 0x73, 0x63, 0x53, 0x2E };
static const uint8_t AUTH_UID2[4] = { 0x11, 0x22, 0x33, 0x44 };

static rc522_handle_t scanner;

/* ---------------- SERVOS ---------------- */
static void servo_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = LEDC_TIMER_16_BIT,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .gpio_num       = SERVO_GPIO,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = SERVO_CHANNEL,
        .timer_sel      = SERVO_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .flags.output_invert = 0
    };
    ledc_channel_config(&ch_conf);
}

static void servo_set_angle(uint8_t angle)
{
    // Limitar ángulo entre 0 y 180
    if(angle > 180) angle = 180;
    
    // Mapeo: 0° = 500us, 180° = 2500us
    float duty_us = 500.0f + ((float)angle / 180.0f) * 2000.0f;
    uint32_t duty = (uint32_t)((duty_us * 65535.0f) / (1000000.0f / SERVO_FREQ_HZ));
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

/* ---------------- RFID EVENT HANDLER ---------------- */
static void rc522_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    rc522_event_t event = (rc522_event_t)event_id;

    switch(event) {
        case RC522_EVENT_PICC_STATE_CHANGED: {
            rc522_picc_state_changed_event_t* state_changed = (rc522_picc_state_changed_event_t*)event_data;
            rc522_picc_t* picc = state_changed->picc;
            
            // Solo procesar cuando la tarjeta entra en estado ACTIVE (detectada y seleccionada)
            if(picc->state == RC522_PICC_STATE_ACTIVE && 
               state_changed->old_state != RC522_PICC_STATE_ACTIVE) {
                
                rc522_picc_uid_t* uid = &picc->uid;
                
                ESP_LOGI(TAG, "═══════════════════════════════════");
                ESP_LOGI(TAG, "✓ Card detected!");
                ESP_LOGI(TAG, "Type: %s", rc522_picc_type_name(picc->type));
                ESP_LOGI(TAG, "UID Length: %d bytes", uid->length);
                
                if(uid->length >= 4) {
                    ESP_LOGI(TAG, "UID: %02X %02X %02X %02X%s", 
                             uid->value[0], uid->value[1], uid->value[2], uid->value[3],
                             uid->length > 4 ? " ..." : "");
                }
                
                ESP_LOGI(TAG, "═══════════════════════════════════");
                
                // Verificar si es una tarjeta autorizada
                if(uid->length >= 4 && 
                   (memcmp(uid->value, AUTH_UID1, 4) == 0 || 
                    memcmp(uid->value, AUTH_UID2, 4) == 0)) {
                    ESP_LOGI(TAG, "🔓 AUTHORIZED! Opening servo to 90°");
                    servo_set_angle(90);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    servo_set_angle(0);
                    ESP_LOGI(TAG, "🔒 Servo closed");
                } else {
                    ESP_LOGW(TAG, "⛔ UNAUTHORIZED card!");
                    if(uid->length >= 4) {
                        ESP_LOGW(TAG, "Update AUTH_UID1 or AUTH_UID2 with:");
                        ESP_LOGW(TAG, "{ 0x%02X, 0x%02X, 0x%02X, 0x%02X }",
                                 uid->value[0], uid->value[1], uid->value[2], uid->value[3]);
                    }
                }
                ESP_LOGI(TAG, "");
            }
            break;
        }
        case RC522_EVENT_NONE:
            break;
        default:
            ESP_LOGW(TAG, "Unknown event: %d", event);
            break;
    }
}

/* ---------------- MAIN ---------------- */
esp_err_t rfid_servo_init(void)
{
    ESP_LOGI(TAG, "Initializing RFID + Servo module...");
    
    esp_log_level_set("rc522", ESP_LOG_WARN);
    
    // Inicializar servo
    servo_init();
    servo_set_angle(0);
    ESP_LOGI(TAG, "✓ Servo initialized at 0°");
    
    // Configuración del bus SPI
    spi_bus_config_t bus_config = {
        .miso_io_num = RFID_MISO,
        .mosi_io_num = RFID_MOSI,
        .sclk_io_num = RFID_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 5000000,
        .mode = 0,
        .spics_io_num = RFID_CS,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    
    rc522_spi_config_t spi_config = {
        .host_id = SPI2_HOST,
        .bus_config = &bus_config,
        .dev_config = dev_config,
        .dma_chan = SPI_DMA_CH_AUTO,
        .rst_io_num = RFID_RST,
    };
    
    rc522_driver_handle_t driver;
    esp_err_t err = rc522_spi_create(&spi_config, &driver);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to create SPI driver: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "✓ SPI driver created");
    
    err = rc522_driver_install(driver);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to install driver: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "✓ Driver installed");
    
    rc522_config_t config = {
        .driver = driver,
        .poll_interval_ms = 500,
        .task_stack_size = 4096,
        .task_priority = 5,
        .task_mutex = NULL,
    };
    
    err = rc522_create(&config, &scanner);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to create RC522: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "✓ RC522 created");
    
    err = rc522_register_events(scanner, RC522_EVENT_ANY, rc522_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to register events: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "✓ Event handler registered");
    
    err = rc522_start(scanner);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to start RC522: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "✓ RFID + Servo module ready!");
    return ESP_OK;
}

// Función opcional para actualizar tarjetas autorizadas
void rfid_servo_set_authorized_cards(const uint8_t uid1[4], const uint8_t uid2[4])
{
    ESP_LOGI(TAG, "Note: To change authorized UIDs, modify AUTH_UID1/AUTH_UID2 in source");
}