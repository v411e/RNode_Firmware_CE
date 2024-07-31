# Board support
If you wish to add support for a specific board to the project, all you have to do (if it's ESP32 or nRF52), is write an additional entry for `Boards.h` and `Utilities.h` and the `Makefile` .

### Boards.h
This entry in `Boards.h` should include, at a minimum, the following:
* whether the device has bluetooth / BLE
* whether the device has a PMU
* whether the device has an EEPROM (false in all cases for nRF52, true for ESP32)
* pin mappings for SPI NSS, SCLK, MOSI, MISO, modem reset and dio0
* the type of modem on the board (if undefined it defaults to SX127x)
* whether the modem has a busy pin
* RX and TX leds (preferably LEDs of different colours)

here is an example :
the new entry to add whoses number should not be used aldready
```
#define BOARD_GENERIC_ESP32_C3 0x3B
```
and the board definition
```
#elif BOARD_MODEL == BOARD_GENERIC_ESP32_C3
      #define HAS_BLUETOOTH false
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 9;
      const int pin_led_tx = 8;
      const uint8_t interfaces[INTERFACE_COUNT] = {SX127X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX127X
          {
              true, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX127X
          {
              7, // pin_ss
              4, // pin_sclk
              6, // pin_mosi
              5, // pin_miso
              -1, // pin_busy
              2, // pin_dio
              3, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

```

see https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-spi.h#L39
Effectively, there are multiple SPI buses we can map to pins on these
devices (including the hardware SPI bus)

An example of a minimal entry can be seen below:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
  #define HAS_BLUETOOTH true
  #define HAS_PMU true
  #define HAS_EEPROM true
  #define EEPROM_SIZE 296 // minimum EEPROM size
  #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
  const int pin_cs = 20;
  const int pin_reset = 19;
  // const int pin_cs = 1; not needed here
  // const int pin_sclk = 2; not needed here
  // const int pin_mosi = 3; not needed here
  // const int pin_miso = 4; not needed here
  const int pin_dio = 18;
  // const int pin_busy = 0; not present
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
```

In some cases the SPI pins will not be required, as they will be the default pins for the SPI library supporting the board anyway, and therefore do not need overriding in the config.

If the SX1262 is being used the following should also be considered:
* whether DIO2 should be used as the RF switch (DIO2_AS_RF_SWITCH)
* whether an rf on/off switch also has to be operated (through the pin pin_rxen)
* whether a TCXO is connected to the modem (HAS_TCXO)
* the enable pin for the TCXO (if present)

An example of an entry using the SX1262 modem can be seen below:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
  #define HAS_BLUETOOTH true
  #define HAS_PMU true
  #define HAS_EEPROM true
  #define EEPROM_SIZE 296 // minimum EEPROM size
  #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
  #define MODEM SX1262
  #define DIO2_AS_RF_SWITCH true
  #define HAS_TCXO true
  #define HAS_BUSY true
  const int pin_cs = 20;
  const int pin_reset = 19;
  const int pin_rxen = 10;
  // const int pin_cs = 1; not needed here
  // const int pin_sclk = 2; not needed here
  // const int pin_mosi = 3; not needed here
  // const int pin_miso = 4; not needed here
  const int pin_dio = 18;
  const int pin_busy = 7;
  const int pin_tcxo_enable = -1;
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
```

If the SX1280 is being used, the following should also be added:
* the TXEN and RXEN pins

An example of an entry using the SX1280 modem can be seen below:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
  #define HAS_BLUETOOTH true
  #define HAS_PMU true
  #define HAS_EEPROM true
  #define EEPROM_SIZE 296 // minimum EEPROM size
  #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
  #define MODEM SX1280
  #define HAS_BUSY true
  #define HAS_RF_SWITCH_RX_TX true
  const int pin_cs = 20;
  const int pin_reset = 19;
  const int pin_rxen = 10;
  const int pin_txen = 11;
  // const int pin_cs = 1; not needed here
  // const int pin_sclk = 2; not needed here
  // const int pin_mosi = 3; not needed here
  // const int pin_miso = 4; not needed here
  const int pin_dio = 18;
  const int pin_busy = 7;
  const int pin_tcxo_enable = -1;
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
```


### Utilities.h
you should add something like this to drive the led :

```
#elif BOARD_MODEL == BOARD_GENERIC_ESP32_C3
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
```

### Makefile

one entry to build the firmware
```
firmware-genericesp32c3:
	arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3B\""
```
pay attention the the DBOARD_MODEL= value as you must choose an unsigned one for your board.

one entry to upload the firmware to the board

```
upload-genericesp32c3:
	arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32c3
	@sleep 1
	rnodeconf /dev/ttyACM0 --firmware-hash $$(./partition_hashes ./build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python ./Release/esptool/esptool.py --chip esp32c3 --port /dev/ttyACM0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000 ./Release/console_image.bin
```

and finally one entry to make a realease for the firmware 

```
release-genericesp32c3:
	arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3B\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ESP_IDF_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_esp32_generic-c3.boot_app0
	cp build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.bin build/rnode_firmware_esp32c3_generic.bin
	cp build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_esp32c3_generic.bootloader
	cp build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_esp32c3_generic.partitions
	zip --junk-paths ./Release/rnode_firmware_esp32c3_generic.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_esp32c3_generic.boot_app0 build/rnode_firmware_esp32c3_generic.bin build/rnode_firmware_esp32c3_generic.bootloader build/rnode_firmware_esp32c3_generic.partitions
	rm -r build
```
dont forget to add this entry to the `release-all` action. 

```
release-all: console-site spiffs-image release-tbeam release-tbeam_sx1262 release-lora32_v10 release-lora32_v20 release-lora32_v21 release-lora32_v10_extled release-lora32_v20_extled release-lora32_v21_extled release-lora32_v21_tcxo release-featheresp32 release-genericesp32 release-genericesp32c3 release-heltec32_v2 release-heltec32_v3 release-heltec32_v2_extled release-rnode_ng_20 release-rnode_ng_21 release-t3s3 release-hashe
```

Please submit this, and any other support in different areas of the project your board may require, as a PR for my consideration.

# Feature request
Feature requests are welcomed, given that those requesting it are happy to write it themselves, or a contributor considers it to be important enough to them to write it themselves. They must be written and **properly** tested before being proposed as a pull request for the project on [GitHub](https://github.com/liberatedsystems/RNode_Firmware_CE). **Manufacturers are encouraged to contribute support for their products back to this repository**, and such support will be received gladly, given it does not effect support for other products or boards.

# Caveat
All contributions must not be written using **any** LLM (ChatGPT, etc.), please handwrite them **only**. Any PRs with proposed contributions which have been discovered to be written using an LLM will **NOT** be merged. The contributor concerned may rewrite their entire pull request **by hand** and it may be reconsidered for merging in the future. 
