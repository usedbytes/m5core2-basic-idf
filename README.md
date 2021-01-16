Basic M5Stack Core2 Example
===========================

This repository is intended as a straightforward example or base project for
using the M5Stack Core2 with ESP-IDF directly in C - i.e. No Arduino.

Because it's intended as an example to learn and copy-paste from, the code
favors simplicity and "obviousness" over clean abstractions and good practice.
It exists only to provide a minimum-viable reference for how to bring up the
board in ESP-IDF.

Functionality
-------------

At present, a subset of the board's features are supported:

- AXP192 PMIC
  - Voltage rail adjustment, enable/disable
  - Configure charger(s)
  - Read battery voltage
  - GPIOs
  - etc...
- LED control
  - via AXP192 `GPIO1`
- Vibration motor control
  - via AXP192 `LDO3`
- LCD backlight control
  - via AXP192 `DCDC3`
- 5 V "Bus"/Grove power control
  - via AXP192 `EXTEN` / `N_VBUSEN`
- LCD and Touchscreen
  - GUI supported using `lvgl` and `lvgl_esp32_drivers`

"Out of the box", it will boot to a simple GUI where you can toggle the LED,
vibration motor, and 5 V bus power on and off.

![Screenshot - see "Known Issues" wrt. CPU usage](screenshot.jpg)

Building
--------

This project builds using ESP-IDF. I have been building it with a random git
snapshot of ESP-IDF from mid 2020. Specifically, the version is reported as:

```
ESP-IDF v4.2-dev-1099-g38102d0e4
```

I believe it should work with any v4 version (where `idf.py` is used instead of
`make`), but I will be happy to receive pull requests to fix the build if there
are any issues.

To build it, in a shell where you have already set up ESP-IDF (i.e. have sourced
`export.sh`):

```
# Clone the repository, and its submodules
git clone https://github.com/usedbytes/m5core2-basic-idf.git
cd m5core2-basic-idf
git submodule update --init --recursive

# Build it, flash it, and see the serial output
idf.py build flash monitor
```

The `sdkconfig` is set up for the M5Core2, and so it should work out-of-the box.

External Dependencies
---------------------

To provide drivers for the different devices, this project pulls in a few
external components as sub-modules. I have had to make adjustments to most of
them to make them work with this board. Some would be suitable for upstreaming,
others are quick hacks.

### `components/esp_i2c_hal`

This provides a dead simple i2c abstraction, which is used by the AXP192 driver.

- Upstream: `https://github.com/tuupola/esp_i2c_hal`
- Downstream: `https://github.com/usedbytes/esp_i2c_hal`

#### Modifications:
- Not using the most recent upstream version, which changed the API in a
  non-backwards compatible way, and I didn't update my `axp192` fork to support
  it yet
- Switch the i2c bus to `0`. Needed to match the (non-configurable)
  `lvgl_esp_drivers` bus.

### `components/axp192`

Driver for the AXP192 PMIC.

- Upstream: `https://github.com/tuupola/axp192`
- Downstream: `https://github.com/usedbytes/axp192`

#### Modifications:
- Not using the "init commands" functionality from upstream, which isn't
  very flexible
- Ignoring most of the Kconfig support, which is a bit cumbersome
- Add voltage rail configuration and direct register access
- Add IRQ handling (though not used in this project! See also
  [my T-Beam project](https://github.com/usedbytes/tbeam). The IRQ pin isn't
  wired up on the M5Core2 board.

### `components/lvgl`

[Light and Versatile Graphics Library](https://lvgl.io/)

- Upstream: `https://github.com/lvgl/lvgl`

#### Modifications:
- None!

### `components/lvgl_esp32_drivers`

Drivers for common ESP32 LCD and touch controllers, for use with LVGL.
See "known issues".

- Upstream: `https://github.com/lvgl/lvgl_esp32_drivers`
- Downstream: `https://github.com/usedbytes/lvgl_esp32_drivers`

#### Modifications:
- Hack the correct screen orientation for M5Core2
- Allow skipping LCD reset control (upstreamable)
   - The reset pin isn't connected to the ESP32, it's connected to the AXP192.

Known Issues
------------

### PMIC / LCD initialisation

The primary issue is that the `lvgl_esp32_drivers` implementation is very
"opinionated", and almost everything is done via KConfig. That means that if
your board isn't already supported, it's quite likely that you're going to
have trouble supporting it cleanly.

Also, it takes complete ownership of setting up the i2c and SPI buses, which
doesn't gel well with the shared i2c bus of the M5Core2.

The main manifestation of this is related to the PMIC and LCD initialisation,
which is a bit hacked. See the [comment above `lvgl_driver_init()` in main.c](main/main.c#L265)
for more details.

### Apparent high CPU usage

The LVGL CPU-usage icon shows ~50% CPU utilisation most of the time. However
during development, I have seen this sometimes show ~0% most of the time. I'm
not sure if there's some strangeness around how FreeRTOS is scheduling the LVGL
work which means it really is using 50% CPU, or if it's just a quirk of how
LVGL is measuring utilisation.

Needs more investigation.
