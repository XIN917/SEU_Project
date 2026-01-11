#pragma once

#include "esp_err.h"

esp_err_t rfid_servo_init(void);

void rfid_servo_set_authorized_cards(const uint8_t uid1[4], const uint8_t uid2[4]);