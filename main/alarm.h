#ifndef ALARM_H
#define ALARM_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize alarm module (button + 2 alternating LEDs)
 * @return ESP_OK on success
 */
esp_err_t alarm_init(void);

/**
 * @brief Set the blink period for alternating LEDs
 * @param period_ms Total period in milliseconds (time for full cycle A-on/B-off then A-off/B-on)
 */
void alarm_set_period_ms(uint32_t period_ms);

#endif // ALARM_H
