#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "config.h"
#include "homekit.h"

CALLBACKS callbacks;

homekit_value_t current_relative_humidity_get();
homekit_value_t current_temperature_get();
homekit_value_t target_door_state_get();
void target_door_state_set(homekit_value_t value);
homekit_value_t current_door_state_get();
homekit_value_t obstruction_detected_get();
void identify(homekit_value_t _value);

homekit_characteristic_t cha_temperature = HOMEKIT_CHARACTERISTIC_(
    CURRENT_TEMPERATURE, 
    0,
    .getter=current_temperature_get
);

homekit_characteristic_t cha_humidity = HOMEKIT_CHARACTERISTIC_(
    CURRENT_RELATIVE_HUMIDITY, 
    0,
    .getter=current_relative_humidity_get
);

homekit_characteristic_t cha_current_door_state = HOMEKIT_CHARACTERISTIC_(
    CURRENT_DOOR_STATE, 
    0,
    .getter=current_door_state_get
);

homekit_characteristic_t cha_target_door_state = HOMEKIT_CHARACTERISTIC_(
    TARGET_DOOR_STATE, 
    0, 
    .setter=target_door_state_set,
    .getter=target_door_state_get
);

homekit_characteristic_t cha_obstruction_detected = HOMEKIT_CHARACTERISTIC_(
    OBSTRUCTION_DETECTED, 
    0,
    .getter=obstruction_detected_get
);

homekit_accessory_t *homekit_accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_garage, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Garage Controller"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Arduino HomeKit"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "0123456"),
            HOMEKIT_CHARACTERISTIC(MODEL, "ESP8266"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1.0"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            &cha_temperature,
            NULL
        }),  
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"),
            &cha_humidity,
            NULL
        }),
        HOMEKIT_SERVICE(GARAGE_DOOR_OPENER, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Garage Door"),
            &cha_current_door_state,
            &cha_target_door_state,
            &cha_obstruction_detected,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t homekit_server_config = {
    .accessories = homekit_accessories,
    .password = HOMEKIT_PASSWORD
};


homekit_value_t current_relative_humidity_get(){
    cha_humidity.value.float_value = callbacks.current_relative_humidity_get();
    return cha_humidity.value;
}

void current_relative_humidity_update(float value){
    cha_humidity.value.float_value = value;
    homekit_characteristic_notify(&cha_humidity, cha_humidity.value);
}

homekit_value_t current_temperature_get(){
    cha_temperature.value.float_value = callbacks.current_temperature_get();
    return cha_temperature.value;
}

void current_temperature_update(float value){
    cha_temperature.value.float_value = value;
    homekit_characteristic_notify(&cha_temperature, cha_temperature.value);
}
  
homekit_value_t target_door_state_get(){
    cha_target_door_state.value.uint8_value = callbacks.target_door_state_get();
    return cha_target_door_state.value;
}

void target_door_state_update(uint8_t value){
    cha_target_door_state.value.uint8_value = value;
    homekit_characteristic_notify(&cha_target_door_state, cha_target_door_state.value);
}

void target_door_state_set(homekit_value_t value){
    callbacks.target_door_state_set(value.uint8_value);
}

homekit_value_t current_door_state_get(){
    cha_current_door_state.value.uint8_value = callbacks.current_door_state_get();
    return cha_current_door_state.value;
}

void current_door_state_update(uint8_t value){
    cha_current_door_state.value.uint8_value = value;
    homekit_characteristic_notify(&cha_current_door_state, cha_current_door_state.value);
}

homekit_value_t obstruction_detected_get(){
    cha_obstruction_detected.value.bool_value = callbacks.obstruction_detected_get();
    return cha_obstruction_detected.value;
}

void obstruction_detected_update(bool state){
    cha_obstruction_detected.value.bool_value = state;
    homekit_characteristic_notify(&cha_obstruction_detected, cha_obstruction_detected.value);
}

void identify(homekit_value_t _value) {
    // nothing to do
}
  
void init_homekit(float (*current_relative_humidity_get)(),
                  float (*current_temperature_get)(),
                  uint8_t (*target_door_state_get)(),
                  void (*target_door_state_set)(uint8_t value),
                  uint8_t (*current_door_state_get)(),
                  bool (*obstruction_detected_get)()) {
                    
    callbacks.current_relative_humidity_get = current_relative_humidity_get;
    callbacks.current_temperature_get = current_temperature_get;
    callbacks.target_door_state_get = target_door_state_get;
    callbacks.target_door_state_set = target_door_state_set;
    callbacks.current_door_state_get = current_door_state_get;
    callbacks.obstruction_detected_get = obstruction_detected_get;

    arduino_homekit_setup(&homekit_server_config);
}
