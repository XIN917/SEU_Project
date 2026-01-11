#ifndef PARKING_DOOR_H
#define PARKING_DOOR_H

#include "esp_err.h"

/**
 * @brief Inicializa el sistema Botón + Servo (puerta de parking)
 *        Crea la tarea que gestiona el botón
 */
esp_err_t parking_door_init(void);

#endif
