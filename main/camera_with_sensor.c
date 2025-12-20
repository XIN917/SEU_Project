#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SENSOR";

/* GPIO configuration */
#define TRIG_GPIO 4
#define ECHO_GPIO 5
/* Camera LED */
#define LED_GPIO 2
/* Proximity threshold in centimeters (trigger if measured distance is less than this) */
#define PROXIMITY_THRESHOLD_CM 300.0f

/* Speed of sound in cm/us */
#define SOUND_SPEED_CM_PER_US 0.0343

/* Maximum waiting time for echo (in microseconds) */
#define MAX_ECHO_TIME_US 30000  // ~5 meters

static void ultrasonic_init(void)
{
    gpio_reset_pin(TRIG_GPIO);
    gpio_set_direction(TRIG_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_GPIO, 0);

    gpio_reset_pin(ECHO_GPIO);
    gpio_set_direction(ECHO_GPIO, GPIO_MODE_INPUT);
    /* Ensure ECHO defaults to LOW when idle to avoid floating reads */
    gpio_set_pull_mode(ECHO_GPIO, GPIO_PULLDOWN_ONLY);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    ESP_LOGI(TAG, "Ultrasonic sensor initialized");
}

static float ultrasonic_measure_distance(void)
{
    int64_t start_time = 0;
    int64_t echo_start = 0;
    int64_t echo_end = 0;

    /* Send trigger pulse */
    gpio_set_level(TRIG_GPIO, 0);
    int64_t _tstart = esp_timer_get_time();
    while ((esp_timer_get_time() - _tstart) < 2) {
        ;
    }
    gpio_set_level(TRIG_GPIO, 1);
    _tstart = esp_timer_get_time();
    while ((esp_timer_get_time() - _tstart) < 10) {
        ;
    }
    gpio_set_level(TRIG_GPIO, 0);

    /* Wait for echo to go HIGH */
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if ((esp_timer_get_time() - start_time) > MAX_ECHO_TIME_US) {
            ESP_LOGW(TAG, "Echo start timeout");
            /* Diagnostic sampling: sample ECHO for a short period to detect noise or stuck level */
            {
                const int sample_count = 150; /* ~30ms with 200us interval */
                const int sample_interval_us = 200;
                int transitions = 0;
                int last_level = gpio_get_level(ECHO_GPIO);
                int max_high_len = 0;
                int cur_high_len = 0;
                int highs_seen = 0;
                int lows_seen = 0;
                int i;
                int64_t samp_start = esp_timer_get_time();
                for (i = 0; i < sample_count; i++) {
                    int lvl = gpio_get_level(ECHO_GPIO);
                    if (lvl != last_level) {
                        transitions++;
                    }
                    if (lvl) {
                        cur_high_len++;
                        highs_seen++;
                    } else {
                        if (cur_high_len > max_high_len) max_high_len = cur_high_len;
                        cur_high_len = 0;
                        lows_seen++;
                    }
                    last_level = lvl;
                    /* busy wait interval */
                    int64_t t0 = esp_timer_get_time();
                    while ((esp_timer_get_time() - t0) < sample_interval_us) {
                        ;
                    }
                }
                if (cur_high_len > max_high_len) max_high_len = cur_high_len;
                int64_t samp_end = esp_timer_get_time();
                int64_t samp_dur = samp_end - samp_start;
                ESP_LOGI(TAG, "Diag: initial_level=%d transitions=%d highs=%d lows=%d max_high_samples=%d samp_dur=%lldus",
                         gpio_get_level(ECHO_GPIO), transitions, highs_seen, lows_seen, max_high_len, samp_dur);
                ESP_LOGI(TAG, "Diag: max_high_duration_us=%d", max_high_len * sample_interval_us);
            }
            return -1;
        }
    }

    echo_start = esp_timer_get_time();
    ESP_LOGD(TAG, "Echo went HIGH at %lld us", echo_start);

    /* Wait for echo to go LOW */
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if ((esp_timer_get_time() - echo_start) > MAX_ECHO_TIME_US) {
            ESP_LOGW(TAG, "Echo end timeout");
            return -1;
        }
    }

    echo_end = esp_timer_get_time();
    ESP_LOGD(TAG, "Echo went LOW at %lld us", echo_end);

    /* Calculate distance */
    int64_t echo_duration = echo_end - echo_start;
    float distance_cm = (echo_duration * SOUND_SPEED_CM_PER_US) / 2.0;
    ESP_LOGD(TAG, "Echo duration: %lld us -> %.2f cm", echo_duration, distance_cm);

    return distance_cm;
}

void app_main(void)
{
    ESP_LOGI(TAG, "HC-SR04 test started");

    ultrasonic_init();

    int64_t led_off_time_us = 0;

    while (1) {
        const int max_attempts = 2;
        float samples[max_attempts];
        int valid = 0;
        for (int a = 0; a < max_attempts; a++) {
            float d = ultrasonic_measure_distance();
            if (d > 0) {
                samples[valid++] = d;
                ESP_LOGD(TAG, "Attempt %d valid: %.2f cm", a + 1, d);
                /* If we detect proximity on any attempt, trigger LED hold immediately and stop further attempts */
                if (d < PROXIMITY_THRESHOLD_CM) {
                    int64_t now = esp_timer_get_time();
                    const int64_t hold_us = 10 * 1000000LL;
                    led_off_time_us = now + hold_us;
                    ESP_LOGI(TAG, "Proximity quick event: %.0f cm threshold crossed, LED will stay on until %lld us", (double)PROXIMITY_THRESHOLD_CM, led_off_time_us);
                    break;
                }
            } else {
                ESP_LOGW(TAG, "Attempt %d failed", a + 1);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

    float distance = -1;
        if (valid > 0) {
            /* compute median */
            if (valid == 1) {
                distance = samples[0];
            } else if (valid == 2) {
                /* average two */
                distance = (samples[0] + samples[1]) / 2.0f;
            } else {
                /* sort three values (simple) and take middle */
                float a = samples[0], b = samples[1], c = samples[2];
                if (a > b) { float t = a; a = b; b = t; }
                if (b > c) { float t = b; b = c; c = t; }
                if (a > b) { float t = a; a = b; b = t; }
                distance = b;
            }
            ESP_LOGI(TAG, "Distance: %.2f cm (from %d valid samples)", distance, valid);
            /* If proximity detected, set LED timeout to 10 seconds from now. If already on,
               this resets the countdown (retrigger). */
            if (distance < PROXIMITY_THRESHOLD_CM) {
                int64_t now = esp_timer_get_time();
                const int64_t hold_us = 10 * 1000000LL;
                /* set or reset led off time */
                led_off_time_us = now + hold_us;
                ESP_LOGI(TAG, "Proximity event: %.0f cm threshold crossed, LED will stay on until %lld us (in ~%lld ms)", (double)PROXIMITY_THRESHOLD_CM, led_off_time_us, (led_off_time_us - now) / 1000);
            }
        } else {
            ESP_LOGI(TAG, "Measurement failed (all attempts)");
        }

        {
            int64_t now = esp_timer_get_time();
            if (now < led_off_time_us) {
                gpio_set_level(LED_GPIO, 1);
            } else {
                gpio_set_level(LED_GPIO, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
