#include <Arduino.h>
#include <arduino_homekit_server.h>
#include <ESP8266WiFi.h>
#include <Arduino_DebugUtils.h>
#include <DHT.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi.h>

extern "C" {
  #include "config.h"
  #include "homekit.h"
}

#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING 2
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING 3
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED 4
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN 255

#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN 255

void ICACHE_RAM_ATTR on_contact_sensor_change();

DHT dht(DHT_PIN, DHT_TYPE);
volatile bool current_door_state_changed = false;
volatile bool target_door_state_changed = false;
volatile long target_door_state_changed_timestamp = 0;
volatile uint8_t target_door_state;
static uint32_t next_report_ms = 0;

void init_debug() {
  Serial.begin(DEBUG_PORT);
  Debug.setDebugLevel(DEBUG_LEVEL);
}

void init_wifi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (!WiFi.isConnected()) {
    Debug.print(DBG_VERBOSE, "WiFi connecting..."); 
    delay(250);
  }
  Debug.print(DBG_VERBOSE, "WiFi connected (IP is %s)", WiFi.localIP().toString().c_str());
}

void init_dht(){
  dht.begin();
}

void on_contact_sensor_change(){
  current_door_state_changed = true;
}

void init_contact_sensor(){
  pinMode(CONTACT_SENSOR_PIN, INPUT);
  target_door_state = current_door_state_get();
  attachInterrupt(digitalPinToInterrupt(CONTACT_SENSOR_PIN), on_contact_sensor_change, CHANGE);
}

void init_actuator(){
  pinMode(ACTUATOR_PIN, OUTPUT);
  digitalWrite(ACTUATOR_PIN, HIGH);
}

void setup() {
	init_debug();
	init_wifi();
  init_dht();
  init_contact_sensor();
  init_actuator();

  init_homekit(&current_relative_humidity_get,
               &current_temperature_get,
               &target_door_state_get,
               &target_door_state_set,
               &current_door_state_get,
               &obstruction_detected_get
  );
}

void loop() {
	arduino_homekit_loop();

  uint8_t current_door_state = current_door_state_get();
  const uint32_t timestamp = millis();
  
  if(current_door_state != target_door_state && target_door_state_changed_timestamp + ACTUATOR_TIMEOUT_MS < millis() ){
    Debug.print(DBG_ERROR, "Defaulting target door state to current door state due to inativity after %d msecs", millis() - target_door_state_changed_timestamp); 
    target_door_state = current_door_state;
    target_door_state_changed = true;
  }
  
  if(current_door_state_changed){
    Debug.print(DBG_VERBOSE, "Current door state %d", current_door_state); 
    target_door_state = current_door_state;
    target_door_state_changed = true;
  }

  if (timestamp > next_report_ms) {
    // report sensor values every 5 seconds
    next_report_ms = timestamp + 5 * 1000;

    current_door_state_update(current_door_state);
    current_door_state_changed = false;
    
    target_door_state_update(target_door_state_get());
    target_door_state_changed = false;
    
    current_temperature_update(current_temperature_get());
    current_relative_humidity_update(current_relative_humidity_get());
    obstruction_detected_update(obstruction_detected_get());
  } else {
    if(current_door_state_changed){
      current_door_state_update(current_door_state);
      current_door_state_changed = false;
    }
  
    if(target_door_state_changed){
      target_door_state_update(target_door_state_get());
      target_door_state_changed = false;
    } 
  }

  delay(50);
}

float current_relative_humidity_get(){
  float humidity = dht.readHumidity();
  if (isnan(humidity)) {
    Debug.print(DBG_ERROR, "Error reading humidity"); 
  }else {
    Debug.print(DBG_VERBOSE, "Humidity %.1f", humidity); 
    return humidity;
  }
}

float current_temperature_get(){
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Debug.print(DBG_ERROR, "Error reading temperature"); 
  }else {
    Debug.print(DBG_VERBOSE, "Temperature %.1fº", temperature); 
    return temperature;
  }
}

uint8_t target_door_state_get(){
  return target_door_state;
}

void trigger_actuator(){
  digitalWrite(ACTUATOR_PIN, LOW);
  delay(200); 
  digitalWrite(ACTUATOR_PIN, HIGH);
}

void target_door_state_set(uint8_t value){
  Debug.print(DBG_VERBOSE, "Set target door state to %d", value); 
  target_door_state_changed_timestamp = millis();
  target_door_state_changed = true;
  target_door_state = value;  
  trigger_actuator();
}

uint8_t current_door_state_get(){
  uint8_t contact = (1 + digitalRead(CONTACT_SENSOR_PIN)) % 2;
  Debug.print(DBG_VERBOSE, "Current door state %d", contact); 
  return contact;
}

bool obstruction_detected_get(){
  return false;
}
