FQBN=arduino:avr:leonardo
PORT=$(shell arduino-cli board list | grep Leonard | awk '{print $$1}')

.PHONY: build all flash wait mon

build:
	arduino-cli compile -e -b $(FQBN) .

all: build flash wait mon

flash:
	arduino-cli upload -b $(FQBN) -p $(PORT) .

wait:
	sleep 2

mon:
	pyserial-miniterm $(PORT) 230400