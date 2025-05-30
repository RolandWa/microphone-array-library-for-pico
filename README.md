# microphone-array-library-for-pico

Raspberry pi pico library for 16+1 microphone array connection.

Capture audio from a microphone array on your [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/) or any [RP2040](https://www.raspberrypi.org/products/rp2040/) based board. 🎤


## Hardware

 * RP2040 board
   * [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/)
 * Microphone array
   * [16 +1 Microphone Array] (https://www.digikey.pl/pl/products/detail/3s-solid-state-system/3SM222FMB1VA/25559041?_gl)
     

### Default Pinout

| Raspberry Pi Pico / RP2040 | Microphone | Configuration Define |
| --- | --- | --- |
| MIC_CK | 0 | PIN_SCK |
| MIC_WS | 1 | PIN_WS = PIN_SCK + 1 |
| MIC_D0 | 2 | PIN_SD0 |
| MIC_D1 | 3 | PIN_SD1 =  PIN_SD0 + 1 |
| MIC_D2 | 4 | PIN_SD2 =  PIN_SD1 + 1 |
| MIC_D3 | 5 | PIN_SD3 =  PIN_SD2 + 1 |
| LED_CK | 15 | LED_PIN_CLK |
| LED_DA | 14 | LED_PIN_DIN |

VIN of the microphobe should be connected to the 3.3V of the power supply.
GND of the microphobe should be connected to the GND of the power supply and Raspberry Pi Pico.
Since the library is based on pio functionality, different ports on raspberry pi microcontroller can be used.

GPIO pins are configurable in examples or API.

## Examples

See [examples](examples/) folder.

## Cloning

```sh
git clone https://github.com/RolandWa/microphone-array-library-for-pico.git
```

## Building

1. [Set up the Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
2. Set `PICO_SDK_PATH`
```sh
export PICO_SDK_PATH=/path/to/pico-sdk
```
3. Create `build` dir, run `cmake` and `make`:
```
mkdir build
cd build
cmake .. 
make
```
4. Copy example `.uf2` to Pico when in BOOT mode.

## Acknowledgements

To create this project, following references were used:
 * The [TinyUSB](https://github.com/hathach/tinyusb) library is used in the `usb_microphone_array` examples.
 * Machine I2S  https://github.com/sfera-labs/arduino-pico-i2s-audio
 * Microphone library for pico https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico.git
 * PDM Microphones and Sigma-Delta A/D Conversion https://tomverbeure.github.io/2020/10/04/PDM-Microphones-and-Sigma-Delta-Conversion.html
 * DIY Acoustic Camera using UMA-16 https://navat.substack.com/p/diy-acoustic-camera-using-uma-16
 * Acoular – Acoustic testing and source mapping software https://www.acoular.org/
---
