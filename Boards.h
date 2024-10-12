// Copyright (C) 2024, Mark Qvist

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "Interfaces.h"
#include "ROM.h"

#ifndef BOARDS_H
  #define BOARDS_H

  #define PLATFORM_ESP32      0x80
  #define PLATFORM_NRF52      0x70

  #define MCU_ESP32           0x81
  #define MCU_NRF52           0x71

  // Products, boards and models. Grouped by manufacturer.
  // Below are the original RNodes, sold by Mark Qvist.
  #define PRODUCT_RNODE       0x03 // RNode devices
  #define BOARD_RNODE         0x31 // Original v1.0 RNode
  #define MODEL_A4            0xA4 // RNode v1.0, 433 MHz
  #define MODEL_A9            0xA9 // RNode v1.0, 868 MHz

  #define BOARD_RNODE_NG_20   0x40 // RNode hardware revision v2.0
  #define MODEL_A3            0xA3 // RNode v2.0, 433 MHz
  #define MODEL_A8            0xA8 // RNode v2.0, 868 MHz

  #define BOARD_RNODE_NG_21   0x41 // RNode hardware revision v2.1
  #define MODEL_A2            0xA2 // RNode v2.1, 433 MHz
  #define MODEL_A7            0xA7 // RNode v2.1, 868 MHz

  #define BOARD_RNODE_NG_22   0x42 // RNode hardware revision v2.2 (T3S3)
  #define MODEL_A1            0xA1 // RNode v2.2, 433 MHz with SX1268
  #define MODEL_A5            0xA5 // RNode v2.2, 433 MHz with SX1278
  #define MODEL_A6            0xA6 // RNode v2.2, 868 MHz with SX1262
  #define MODEL_AA            0xAA // RNode v2.2, 868 MHz with SX1276

  #define PRODUCT_TBEAM       0xE0 // T-Beam - sold by LilyGO
  #define BOARD_TBEAM         0x33
  #define MODEL_E4            0xE4 // T-Beam SX1278, 433 Mhz
  #define MODEL_E9            0xE9 // T-Beam SX1276, 868 Mhz
  #define MODEL_E3            0xE3 // T-Beam SX1268, 433 Mhz
  #define MODEL_E8            0xE8 // T-Beam SX1262, 868 Mhz

  #define PRODUCT_TDECK_V1    0xD0 // T-Deck - sold by LilyGO
  #define BOARD_TDECK         0x3B
  #define MODEL_D4            0xD4 // LilyGO T-Deck, 433 MHz
  #define MODEL_D9            0xD9 // LilyGO T-Deck, 868 MHz

  #define PRODUCT_TBEAM_S_V1  0xEA // T-Beam Supreme - sold by LilyGO
  #define BOARD_TBEAM_S_V1    0x3D
  #define MODEL_DB            0xDB // LilyGO T-Beam Supreme, 433 MHz
  #define MODEL_DC            0xDC // LilyGO T-Beam Supreme, 868 MHz

  #define PRODUCT_T32_10      0xB2 // T3 v1.0 - sold by LilyGO
  #define BOARD_LORA32_V1_0   0x39
  #define MODEL_BA            0xBA // LilyGO T3 v1.0, 433 MHz
  #define MODEL_BB            0xBB // LilyGO T3 v1.0, 868 MHz

  #define PRODUCT_T32_20      0xB0 // T3 v2.0 - sold by LilyGO
  #define BOARD_LORA32_V2_0   0x36
  #define MODEL_B3            0xB3 // LilyGO T3 v2.0, 433 MHz
  #define MODEL_B8            0xB8 // LilyGO T3 v2.0, 868 MHz

  #define PRODUCT_T32_21      0xB1 // T3 v2.1 - sold by LilyGO
  #define BOARD_LORA32_V2_1   0x37
  #define MODEL_B4            0xB4 // LilyGO T3 v2.1, 433 MHz
  #define MODEL_B9            0xB9 // LilyGO T3 v2.1, 868 MHz

  #define BOARD_T3S3          0x42 // T3S3 - sold by LilyGO
  #define MODEL_A1 0xA1            // T3S3 SX1262 868/915 MHz
  #define MODEL_AB 0xAB            // T3S3 SX1276 868/915 MHz
  #define MODEL_A5 0xA5            // T3S3 SX1280 PA (2.4GHz)

  #define PRODUCT_TECHO 0x15       // LilyGO T-Echo devices
  #define BOARD_TECHO         0x43
  #define MODEL_16 0x16            // T-Echo 433 MHz
  #define MODEL_17 0x17            // T-Echo 868/915 MHz


  #define PRODUCT_H32_V2      0xC0 // LoRa32 v2 - sold by Heltec
  #define BOARD_HELTEC32_V2   0x38
  #define MODEL_C4            0xC4 // Heltec Lora32 v2, 433 MHz
  #define MODEL_C9            0xC9 // Heltec Lora32 v2, 868 MHz

  #define PRODUCT_H32_V3      0xC1 // LoRa32 v3 - sold by Heltec
  #define BOARD_HELTEC32_V3   0x3A
  #define MODEL_C5            0xC5 // Heltec Lora32 v3, 433 MHz
  #define MODEL_CA            0xCA // Heltec Lora32 v3, 868 MHz

  #define PRODUCT_RAK4631     0x10 // RAK4631 - sold by RAKWireless
  #define BOARD_RAK4631       0x51
  #define MODEL_11            0x11 // RAK4631, 433 MHz
  #define MODEL_12            0x12 // RAK4631, 868 MHz
  #define MODEL_13            0x13 // RAK4631, 433MHz with WisBlock SX1280 module (LIBSYS002)
  #define MODEL_14            0x14 // RAK4631, 868/915 MHz with WisBlock SX1280 module (LIBSYS002)

  #define PRODUCT_OPENCOM_XL  0x20 // openCom XL - sold by Liberated Embedded Systems
  #define MODEL_21            0x21 // openCom XL, 868/915 MHz

  #define BOARD_E22_ESP32     0x44 // Custom Ebyte E22 board design for meshtastic, source:
                                   // https://github.com/NanoVHF/Meshtastic-DIY/blob/main/Schematics/E-Byte_E22/Mesh_Ebyte_E22-XXXM30S.pdf

  #define PRODUCT_HMBRW       0xF0
  #define BOARD_HMBRW         0x32
  #define BOARD_HUZZAH32      0x34
  #define BOARD_GENERIC_ESP32 0x35
  #define BOARD_GENERIC_NRF52 0x50
  #define MODEL_FE            0xFE // Homebrew board, max 17dBm output power
  #define MODEL_FF            0xFF // Homebrew board, max 14dBm output power

  // Displays
  #define OLED 0x01
  #define EINK_BW 0x02
  #define EINK_3C 0x03

  #if defined(ESP32)
    #define PLATFORM PLATFORM_ESP32
    #define MCU_VARIANT MCU_ESP32
  #elif defined(NRF52840_XXAA) || defined(_VARIANT_PCA10056_)
    #include <variant.h>
    #define PLATFORM PLATFORM_NRF52
    #define MCU_VARIANT MCU_NRF52
  #else
      #error "The firmware cannot be compiled for the selected MCU variant"
  #endif

  #define HAS_DISPLAY false
  #define HAS_BLUETOOTH false
  #define HAS_BLE false
  #define HAS_TCXO false
  #define HAS_PMU false
  #define HAS_NP false
  #define HAS_EEPROM false
  #define HAS_INPUT false
  #define HAS_SLEEP false
  #define PIN_DISP_SLEEP -1
  #define VALIDATE_FIRMWARE true

  #if defined(ENABLE_TCXO)
      #define HAS_TCXO true
  #endif

  #if MCU_VARIANT == MCU_ESP32

    // Board models for ESP32 based builds are
    // defined by the build target in the makefile.
    // If you are not using make to compile this
    // firmware, you can manually define model here.
    //
    // #define BOARD_MODEL BOARD_GENERIC_ESP32
    #define CONFIG_UART_BUFFER_SIZE 6144
    #define CONFIG_QUEUE_0_SIZE 6144
    #define CONFIG_QUEUE_MAX_LENGTH 200

    #define EEPROM_SIZE 1024
    #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED

    #define GPS_BAUD_RATE 9600
    #define PIN_GPS_TX 12
    #define PIN_GPS_RX 34

    #if BOARD_MODEL == BOARD_GENERIC_ESP32
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 14;
      const int pin_led_tx = 32;
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
              4, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              39, // pin_dio
              36, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_TBEAM
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_PMU true
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_SD false
      #define HAS_EEPROM true
      #define I2C_SDA 21
      #define I2C_SCL 22
      #define PMU_IRQ 35
      #define INTERFACE_COUNT 1
      #define HAS_INPUT true
      const int pin_btn_usr1 = 38;
      const int pin_led_rx = 2;
      const int pin_led_tx = 4;

      #if BOARD_VARIANT == MODEL_E4 || BOARD_VARIANT == MODEL_E9
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
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

      #elif BOARD_VARIANT == MODEL_E3 || BOARD_VARIANT == MODEL_E8
      const uint8_t interfaces[INTERFACE_COUNT] = {SX126X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              true, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              32, // pin_busy
              33, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };
      #endif

    #elif BOARD_MODEL == BOARD_HUZZAH32
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 14;
      const int pin_led_tx = 32;

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
                  // SX1262
          {
               4, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              39, // pin_dio
              36, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_LORA32_V1_0
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_cs = 18;
      const int pin_reset = 14;
      const int pin_dio = 26;
      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 25;
        const int pin_led_tx = 2;
      #else
        const int pin_led_rx = 2;
        const int pin_led_tx = 2;
      #endif

      const uint8_t interfaces[INTERFACE_COUNT] = {SX1276};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1276
          {
              true, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1276
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              14, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_LORA32_V2_0
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_cs = 18;
      const int pin_reset = 12;
      const int pin_dio = 26;
      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 2;
        const int pin_led_tx = 0;
      #else
        const int pin_led_rx = 22;
        const int pin_led_tx = 22;
      #endif


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
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              14, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_LORA32_V2_1
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_PMU true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1

      const uint8_t interfaces[INTERFACE_COUNT] = {SX127X};
      #if HAS_TCXO == true
        const bool interface_cfg[INTERFACE_COUNT][3] = { 
                        // SX127X
              {
                  true, // DEFAULT_SPI
                  true, // HAS_TCXO
                  false  // DIO2_AS_RF_SWITCH
              }, 
        };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX127X
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              33  // pin_tcxo_enable
          }
      };
      #endif
      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 15;
        const int pin_led_tx = 4;
      #else
        const int pin_led_rx = 25;
        const int pin_led_tx = 25;
      #endif

      #if HAS_TCXO == false
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
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };
      #endif

    #elif BOARD_MODEL == BOARD_HELTEC32_V2
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      #define HAS_INPUT true
      #define HAS_SLEEP true
      #define PIN_WAKEUP GPIO_NUM_0
      #define WAKEUP_LEVEL 0

      const int pin_btn_usr1 = 0;

      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 36;
        const int pin_led_tx = 37;
      #else
        const int pin_led_rx = 25;
        const int pin_led_tx = 25;
      #endif

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
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              14, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_HELTEC32_V3
      #define IS_ESP32S3 true
      #define HAS_DISPLAY true
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_PMU true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define HAS_INPUT true
      #define HAS_SLEEP true
      #define PIN_WAKEUP GPIO_NUM_0
      #define WAKEUP_LEVEL 0
      #define INTERFACE_COUNT 1

      const int pin_btn_usr1 = 0;

      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 13;
        const int pin_led_tx = 14;
      #else
        const int pin_led_rx = 35;
        const int pin_led_tx = 35;
      #endif

      const uint8_t interfaces[INTERFACE_COUNT] = {SX1262};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              true, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              8, // pin_ss
              9, // pin_sclk
              10, // pin_mosi
              11, // pin_miso
              13, // pin_busy
              14, // pin_dio
              12, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_RNODE_NG_20
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_NP true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_cs = 18;
      const int pin_reset = 12;
      const int pin_dio = 26;
      const int pin_np = 4;
      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 2;
          const int pin_led_tx = 0;
        #else
          const int pin_led_rx = 22;
          const int pin_led_tx = 22;
        #endif
      #endif


      const uint8_t interfaces[INTERFACE_COUNT] = {SX1276};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1276
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      const uint8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1276
          {
              8, // pin_ss
              9, // pin_sclk
              10, // pin_mosi
              11, // pin_miso
              13, // pin_busy
              14, // pin_dio
              12, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_RNODE_NG_21
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_PMU true
      #define HAS_NP true
      #define HAS_SD false
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_np = 12;
      const int pin_dac = 25;
      const int pin_adc = 34;
      const int SD_MISO = 2;
      const int SD_MOSI = 15;
      const int SD_CLK = 14;
      const int SD_CS = 13;
      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 12;
          const int pin_led_tx = 4;
        #else
          const int pin_led_rx = 25;
          const int pin_led_tx = 25;
        #endif
      #endif


      const uint8_t interfaces[INTERFACE_COUNT] = {SX127X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX127X
          {
              true, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      const uint8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX127X
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_T3S3
      #define IS_ESP32S3 true
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_CONSOLE false
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_PMU true
      #define HAS_NP false
      #define HAS_SD false
      #define HAS_EEPROM true

      #define HAS_INPUT true
      #define HAS_SLEEP true
      #define PIN_WAKEUP GPIO_NUM_0
      #define WAKEUP_LEVEL 0
      #define INTERFACE_COUNT 1
      // #define PIN_DISP_SLEEP 21
      // #define DISP_SLEEP_LEVEL HIGH
      const int pin_btn_usr1 = 0;
      
      const int pin_np = 38;
      const int pin_dac = 25;
      const int pin_adc = 1;

      const int SD_MISO = 2;
      const int SD_MOSI = 11;
      const int SD_CLK = 14;
      const int SD_CS = 13;

      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 37;
          const int pin_led_tx = 37;
        #else
          const int pin_led_rx = 37;
          const int pin_led_tx = 37;
        #endif
      #endif

      #if BOARD_VARIANT == MODEL_A1
      const uint8_t interfaces[INTERFACE_COUNT] = {SX1262};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      #elif BOARD_VARIANT == MODEL_A5 // SX1280 with PA 
      const uint8_t interfaces[INTERFACE_COUNT] = {SX1280};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1280
          {
              false, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      #endif
      const uint8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
               7, // pin_ss
               5, // pin_sclk
               6, // pin_mosi
               3, // pin_miso
              34, // pin_busy
              33, // pin_dio
               8, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_E22_ESP32
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_BLE true
      #define HAS_CONSOLE true
      #define HAS_SD false
      #define HAS_EEPROM true
      #define I2C_SDA 21
      #define I2C_SCL 22
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 2;
      const int pin_led_tx = 4;

      const uint8_t interfaces[INTERFACE_COUNT] = {SX1262};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              true, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              18, // pin_ss
              5, // pin_sclk
              27, // pin_mosi
              19, // pin_miso
              32, // pin_busy
              33, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              14, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_TDECK
      #define IS_ESP32S3 true
      #define MODEM SX1262
      #define DIO2_AS_RF_SWITCH true
      #define HAS_BUSY true
      #define HAS_TCXO true

      #define HAS_DISPLAY false
      #define HAS_CONSOLE false
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_PMU true
      #define HAS_NP false
      #define HAS_SD false
      #define HAS_EEPROM true

      #define HAS_INPUT true
      #define HAS_SLEEP true
      #define PIN_WAKEUP GPIO_NUM_0
      #define WAKEUP_LEVEL 0

      const int pin_poweron = 10;
      const int pin_btn_usr1 = 0;

      const int pin_cs = 9;
      const int pin_reset = 17;
      const int pin_sclk = 40;
      const int pin_mosi = 41;
      const int pin_miso = 38;
      const int pin_tcxo_enable = -1;
      const int pin_dio = 45;
      const int pin_busy = 13;
      
      const int SD_MISO = 38;
      const int SD_MOSI = 41;
      const int SD_CLK = 40;
      const int SD_CS = 39;

      const int DISPLAY_DC = 11;
      const int DISPLAY_CS = 12;
      const int DISPLAY_MISO = 38;
      const int DISPLAY_MOSI = 41;
      const int DISPLAY_CLK = 40;
      const int DISPLAY_BL_PIN = 42;

      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 43;
          const int pin_led_tx = 43;
        #else
          const int pin_led_rx = 43;
          const int pin_led_tx = 43;
        #endif
      #endif

    #elif BOARD_MODEL == BOARD_TBEAM_S_V1
      #define IS_ESP32S3 true
      #define MODEM SX1262
      #define DIO2_AS_RF_SWITCH true
      #define HAS_BUSY true
      #define HAS_TCXO true

      #define HAS_DISPLAY true
      #define HAS_CONSOLE true
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_PMU true
      #define HAS_NP false
      #define HAS_SD false
      #define HAS_EEPROM true

      #define HAS_INPUT true
      #define HAS_SLEEP false
      
      #define PMU_IRQ 40
      #define I2C_SCL 41
      #define I2C_SDA 42

      const int pin_btn_usr1 = 0;

      const int pin_cs = 10;
      const int pin_reset = 5;
      const int pin_sclk = 12;
      const int pin_mosi = 11;
      const int pin_miso = 13;
      const int pin_tcxo_enable = -1;
      const int pin_dio = 1;
      const int pin_busy = 4;
      
      const int SD_MISO = 37;
      const int SD_MOSI = 35;
      const int SD_CLK = 36;
      const int SD_CS = 47;

      const int IMU_CS = 34;

      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 43;
          const int pin_led_tx = 43;
        #else
          const int pin_led_rx = 43;
          const int pin_led_tx = 43;
        #endif
      #endif
  #endif

  
  #elif MCU_VARIANT == MCU_NRF52
     #if BOARD_MODEL == BOARD_TECHO
      #define VALIDATE_FIRMWARE false
      #define HAS_INPUT true
      //#define GPS_BAUD_RATE 115200
      //#define PIN_GPS_TX 41
      //#define PIN_GPS_RX 40
      #define EEPROM_SIZE 296
      #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
      //#define HAS_EEPROM true
      //#define HAS_SD true
      #define HAS_DISPLAY true
      #define DISPLAY EINK_BW
      #define DISPLAY_MODEL GxEPD2_154_D67
      //#define HAS_CONSOLE true
      //#define HAS_TXCO true
      //#define HAS_BLE true
      //#define HAS_PMU true
      #define CONFIG_UART_BUFFER_SIZE 40000
      #define CONFIG_QUEUE_0_SIZE 6144
      #define CONFIG_QUEUE_MAX_LENGTH 200
      //#define BLE_MANUFACTURER "LilyGO"
      //#define BLE_MODEL "T-Echo"
      #define INTERFACE_COUNT 1
      //#define I2C_SDA 26
      //#define I2C_SCL 27
      #define CONFIG_QUEUE_1_SIZE 40000
      // first interface in list is the primary
      const uint8_t interfaces[INTERFACE_COUNT] = {SX126X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                  // SX1262
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }
      };

      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              24, // pin_ss 
              19, // pin_sclk
              22, // pin_mosi
              23, // pin_miso
              17, // pin_busy
              20, // pin_dio
              25, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              21  // pin_tcxo_enable
          }
      };

      const int pin_disp_cs = 30;
      const int pin_disp_dc = 28;
      const int pin_disp_reset = 2;
      const int pin_disp_busy = 3;
      const int pin_disp_en = 43;
      const int pin_disp_sck = 31;
      const int pin_disp_mosi = 29;
      const int pin_disp_miso = -1;

      #define HAS_BACKLIGHT true
      const int pin_btn_usr1 = 42;
      const int pin_backlight = 43;

      const int pin_led_rx = LED_BLUE;
      const int pin_led_tx = LED_RED;
    #elif BOARD_MODEL == BOARD_RAK4631 || BOARD_MODEL == BOARD_OPENCOM_XL
      #define HAS_EEPROM false
      #define HAS_DISPLAY true
      #define DISPLAY EINK_BW
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_CONSOLE false
      #define HAS_PMU true
      #define HAS_NP false
      #define HAS_SD false
      #define CONFIG_UART_BUFFER_SIZE 6144
      #define CONFIG_QUEUE_0_SIZE 6144
      #define HAS_INPUT true
      #define CONFIG_QUEUE_MAX_LENGTH 200
      #define EEPROM_SIZE 296
      #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
      #define BLE_MANUFACTURER "RAK Wireless"
      #define BLE_MODEL "RAK4640"

      #if BOARD_VARIANT == MODEL_11 || BOARD_VARIANT == MODEL_12
      #define INTERFACE_COUNT 1

      // first interface in list is the primary
      const uint8_t interfaces[INTERFACE_COUNT] = {SX126X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              42, // pin_ss
              43, // pin_sclk
              44, // pin_mosi
              45, // pin_miso
              46, // pin_busy
              47, // pin_dio
              38, // pin_reset
              -1, // pin_txen
              37, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };
      #elif BOARD_VARIANT == MODEL_13 || BOARD_VARIANT == MODEL_14 || BOARD_VARIANT == MODEL_21
      #define INTERFACE_COUNT 2

      #define CONFIG_QUEUE_1_SIZE 40000
      #define CONFIG_UART_BUFFER_SIZE 40000 // \todo, does it have to be this big?

      // first interface in list is the primary
      const uint8_t interfaces[INTERFACE_COUNT] = {SX126X, SX128X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
                    // SX1280
          {
              true, // DEFAULT_SPI
              false,// HAS_TCXO
              false // DIO2_AS_RF_SWITCH
          } 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              42, // pin_ss
              43, // pin_sclk
              44, // pin_mosi
              45, // pin_miso
              46, // pin_busy
              47, // pin_dio
              38, // pin_reset
              -1, // pin_txen
              37, // pin_rxen
              -1  // pin_tcxo_enable
          },
                  // SX1280
          {
              24, // pin_ss
               3, // pin_sclk
              30, // pin_mosi
              29, // pin_miso
              25, // pin_busy
              15, // pin_dio
              16, // pin_reset
              20, // pin_txen
              19, // pin_rxen
              -1  // pin_tcxo_enable
          } 
      };
      #endif


      const int pin_disp_cs = SS;
      const int pin_disp_dc = WB_IO1;
      const int pin_disp_reset = -1;
      const int pin_disp_busy = WB_IO4;
      const int pin_disp_en = WB_IO2;

      const int pin_btn_usr1 = 9;
      const int pin_led_rx = LED_BLUE;
      const int pin_led_tx = LED_GREEN;

    #else
      #error An unsupported nRF board was selected. Cannot compile RNode firmware.
    #endif

  #endif
#endif
