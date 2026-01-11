#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "parking_door.h"

#define TAG "PARKING_DOOR"

/* ----------- SERVO ----------- */
#define SERVO_GPIO     15
#define SERVO_FREQ_HZ  50
#define SERVO_TIMER    LEDC_TIMER_0
#define SERVO_CHANNEL  LEDC_CHANNEL_0

/* ----------- BOTÓN ----------- */
#define BUTTON_GPIO    7

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
        .hpoint         = 0
    };
    ledc_channel_config(&ch_conf);
}

static void servo_set_angle(uint8_t angle)
{
    if (angle > 180) angle = 180;

    float duty_us = 500.0f + ((float)angle / 180.0f) * 2000.0f;
    uint32_t duty = (uint32_t)((duty_us * 65535.0f) /
                     (1000000.0f / SERVO_FREQ_HZ));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

/* ---------------- BOTÓN ---------------- */
static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

/* ---------------- TAREA ---------------- */
static void parking_door_task(void *arg)
{
    bool busy = false;

    ESP_LOGI(TAG, "Button → Servo task started");

    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0 && !busy) {
            busy = true;

            ESP_LOGI(TAG, "Button pressed → Opening door");
            servo_set_angle(90);

            vTaskDelay(pdMS_TO_TICKS(7000));

            ESP_LOGI(TAG, "Closing door");
            servo_set_angle(0);

            vTaskDelay(pdMS_TO_TICKS(1000)); // antirrebote
            busy = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---------------- INIT PUBLICA ---------------- */
esp_err_t parking_door_init(void)
{
    ESP_LOGI(TAG, "Initializing Parking Door module");

    servo_init();
    servo_set_angle(0);

    button_init();

    xTaskCreate(
        parking_door_task,
        "parking_door_task",
        2048,
        NULL,
        5,
        NULL
    );

    return ESP_OK;
}
