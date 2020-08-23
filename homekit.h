#pragma once

typedef struct _callbacks {
    float (*current_relative_humidity_get)();
    float (*current_temperature_get)();
    uint8_t (*target_door_state_get)();
    void (*target_door_state_set)(homekit_value_t value);
    uint8_t (*current_door_state_get)();
    bool (*obstruction_detected_get)();
} CALLBACKS;

void init_homekit( float (*current_relative_humidity_get)(),
                  float (*current_temperature_get)(),
                  uint8_t (*target_door_state_get)(),
                  void (*target_door_state_set)(homekit_value_t value),
                  uint8_t (*current_door_state_get)(),
                  bool (*obstruction_detected_get)());
void current_door_state_update(uint8_t state);
void target_door_state_update(uint8_t state);
