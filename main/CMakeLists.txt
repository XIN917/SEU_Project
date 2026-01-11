idf_component_register(SRCS "main.c" "parking_door.c" "camera_with_sensor.c" "rfid_with_servo.c" "alarm.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash)
