BOARD?=arduino:avr:nano:cpu=atmega328old
PORT?=/dev/cu.usbserial-10 

BUILD-PATH?=build

# Unset environment vars only for arduino-cli to avoid macOS include conflicts
ARDUINO_ENV = env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH

.PHONY: default lint all flash clean

default: all flash

all:
	$(ARDUINO_ENV) arduino-cli compile --fqbn $(BOARD) --build-path $(BUILD-PATH) ./

flash:
	$(ARDUINO_ENV) arduino-cli upload --fqbn $(BOARD) --build-path $(BUILD-PATH) --port $(PORT)

monitor:
	arduino-cli monitor --port $(PORT) --fqbn $(BOARD) --config 9600

clean:
	rm -rf build
