.PHONY: setup install

BOARD_CORE_URL:="http://arduino.esp8266.com/stable/package_esp8266com_index.json"
BOARD_CORE:="esp8266:esp8266"
BOARD_FQBN:="esp8266:esp8266:nodemcuv2"
BOARD_PORT:="/dev/cu.SLAB_USBtoUART"

setup:
	arduino-cli core update-index --additional-urls $(BOARD_CORE_URL)
	arduino-cli core install $(BOARD_CORE) --additional-urls $(BOARD_CORE_URL)

	arduino-cli lib install "HomeKit-ESP8266"
	arduino-cli lib install "Adafruit_Unified_Sensor"
	arduino-cli lib install "Arduino_DebugUtils"

install:
	arduino-cli compile --fqbn $(BOARD_FQBN)

deploy:
	arduino-cli upload -p $(BOARD_PORT) --fqbn $(BOARD_FQBN)

