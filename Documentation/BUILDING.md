# Building
## Prerequisites
The build system of this repository is based on GNU Make. The `Makefile` is in the base of the repository. Please ensure you have `arduino-cli`, `python3` and `make` installed before proceeding.

Firstly, figure out which MCU platform your supported board is based on. The table below can help you.

| Board name | Link | Transceiver | MCU | Description | 
| :--- | :---: | :---: | :---: | :---: |
| Handheld v2.x RNodes | [Buy here](https://unsigned.io/shop/product/handheld-rnode) | SX1276 | ESP32 |
| RAK4631 | [Buy here](https://store.rakwireless.com/products/rak4631-lpwan-node?m=5&h=wisblock-core) | SX1262 | nRF52 |
| LilyGO LoRa32 v1.0 | [Buy here](https://www.lilygo.cc/products/lora32-v1-0) | SX1276/8 | ESP32 |
| LilyGO T-BEAM v1.1 | [Buy here](https://www.lilygo.cc/products/t-beam-v1-1-esp32-lora-module) | SX1276/8 | ESP32 |
| LilyGO LoRa32 v2.0 | No link | SX1276/8 | ESP32 | Discontinued? |
| LilyGO LoRa32 v2.1 |  [Buy here](https://www.lilygo.cc/products/lora3) | SX1276/8 | ESP32 | With and without TCXO |
| Heltec LoRa32 v2 | No link | SX1276/8 | ESP32 | Discontinued? |
| Heltec LoRa32 v3 | [Buy here](https://heltec.org/project/wifi-lora-32-v3/) | SX1262 | ESP32 | 
| Homebrew ESP32 boards | | Any supported | ESP32 | This can be any board with an Adafruit Feather (or generic) ESP32 chip |

### ESP32
If your board is ESP32-based, please run `make prep-esp32` to install the required BSP and libraries for that target.

### nRF52
If your board is nRF52-based, please run `make prep-nrf` to install the required BSP and libraries for that target.

## Compiling
Next, you need to find the name of the target for your board. Please reference the table below to do so:
| Board name | Target | 
| :--- | :---: |
| Handheld v2.x RNodes | `rnode_ng_20` |
| RAK4631 | `rak4631` |
| LilyGO T-BEAM v1.1 | `tbeam` |
| LilyGO T-BEAM v1.1 (SX1262) | `tbeam_sx126x` |
| LilyGO LoRa32 v1.0 | `lora32_v10` |
| LilyGO LoRa32 v2.0 | `lora32_v20` |
| LilyGO LoRa32 v2.1 | `lora32_v21` |
| Heltec LoRa32 v2 | `heltec32_v2` |
| Heltec LoRa32 v3 | `heltec32_v3` | 
| Homebrew ESP32 boards | `genericesp32` |

After you've ascertained the target for the board simply run the following to compile for the board:

`make firmware-[target]`

Ensure you replace [target] with the target you selected. For example:

`make firmware-rak4631`

## Flashing
To flash the chosen board (if you have one), simply connect it to your computer over USB, and then run the following:

`make upload-[target]`

Ensure you replace [target] with the target you selected. For example:

`make upload-rak4631`

**Please note**, you must re-compile the firmware each time you make changes **before** you flash it, else you will just be flashing the previous version of the firmware without the new changes!

These commands can also be run as a one liner. For example:

`make firmware-[target] && make upload-[target]`

This is especially helpful when making continuous changes to the firmware and testing them out.
