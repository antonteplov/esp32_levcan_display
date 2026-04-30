PROJECT := can_oled_monitor
PORT ?= /dev/ttyUSB0
FQBN ?= esp32:esp32:lolin32
BUILD_DIR := build
ARDUINO_CLI ?= arduino-cli

all: compile

compile:
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --output-dir $(BUILD_DIR) .

upload: compile
	$(ARDUINO_CLI) upload -p $(PORT) --fqbn $(FQBN) --input-dir $(BUILD_DIR) .

monitor:
	$(ARDUINO_CLI) monitor -p $(PORT) -c baudrate=115200

clean:
	rm -rf $(BUILD_DIR)

core-install:
	$(ARDUINO_CLI) config init || true
	$(ARDUINO_CLI) config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json || true
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install esp32:esp32

lib-install:
	$(ARDUINO_CLI) lib install "Adafruit SSD1306"
	$(ARDUINO_CLI) lib install "Adafruit GFX Library"
	$(ARDUINO_CLI) lib install "Adafruit BusIO"

setup: core-install lib-install
