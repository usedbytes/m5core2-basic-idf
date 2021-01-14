#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "axp192.h"
#include "i2c_helper.h"

void set_axp192_gpio_012(const axp192_t *axp, int gpio, bool low) {
	if ((gpio < 0) || (gpio > 2)) {
		return;
	}

	uint8_t val = 0;
	axp192_read_reg(axp, AXP192_GPIO20_SIGNAL_STATUS, &val);

	uint8_t mask = (1 << gpio);
	if (low) {
		// Value of 0 activates the NMOS, pulling the pin low
		val &= ~mask;
	} else {
		// Value of 1 sets floating
		val |= mask;
	}

	axp192_write_reg(axp, AXP192_GPIO20_SIGNAL_STATUS, val);
}

void set_internal_5v_bus(const axp192_t *axp, bool enable) {
	// To enable the on-board 5V supply, first N_VBUSEN needs to be pulled
	// high using GPIO0, then we can enable the EXTEN output, to enable
	// the SMPS.
	// To disable it (so either no 5V, or externally supplied 5V), we
	// do the opposite: First disable EXTEN, then leave GPIO0 floating.
	// N_VBUSEN will be pulled down by the on-board resistor.
	// Side note: The pull down is 10k according to the schematic, so that's
	// a 0.5 mA drain from the GPIO0 LDO as long as the bus supply is active.

	if (enable) {
		// GPIO0_LDOIO0_VOLTAGE:
		// Bits 7-4: Voltage. 1v8-3v3 in 0.1 V increments
		// Set the GPIO0 LDO voltage to 3v3
		axp192_write_reg(axp, AXP192_GPIO0_LDOIO0_VOLTAGE, 0xf0);

		// GPIO0_CONTROL
		// Bits 7-3: Reserved
		// Bits 2-0: 000: NMOS open drain
		//           001: GPIO
		//           010: Low noise LDO
		//           011: Reserved
		//           100: ADC input
		//           101: Output low
		//           11x: Floating
		// Set to LDO (3v3)
		axp192_write_reg(axp, AXP192_GPIO0_CONTROL, (2 << 0));

		// Enable EXTEN
		axp192_set_rail_state(axp, AXP192_RAIL_EXTEN, true);
	} else {
		// Disable EXTEN
		axp192_set_rail_state(axp, AXP192_RAIL_EXTEN, false);

		// Set GPIO0 to float
		axp192_write_reg(axp, AXP192_GPIO0_CONTROL, (7 << 0));
	}
}

void app_main(void)
{
	printf("Hello world!\n");

	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
			CONFIG_IDF_TARGET,
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	printf("Free heap: %d\n", esp_get_free_heap_size());

	const axp192_t axp = {
		.read = &i2c_read,
		.write = &i2c_write,
	};

	i2c_init();

	// Voltage configuration
	{

		// don't axp192_init() because we don't wan't to run the initial commands
		// axp192_init(&axp);

		// DCDC1: MCU_VDD  (3v3)
		// DCDC2: Not connected
		// DCDC3: LCD_BL   (2v8)
		// LDO1: RTC, non-configured
		// LDO2: PERI_VDD  (3v3)
		// LDO3: VIB_MOTOR (2v0)
		struct rail_entry {
			const char *name;
			axp192_rail_t rail;
			uint16_t millivolts;
		};

		// The voltages are based on the Arduino demo code.
		struct rail_entry rails[] = {
			{ "DCDC1 (MCU_VDD)",  AXP192_RAIL_DCDC1, 3300 },
			// DCDC2 not connected
			{ "DCDC3 (LCD_BL)",   AXP192_RAIL_DCDC3, 2800 },
			// LDO1 not configurable
			{ "LDO2 (PERI_VDD)",  AXP192_RAIL_LDO2,  3300 },
			{ "LDO3 (VIB_MOTOR)", AXP192_RAIL_LDO3,  2000 },
		};

		for (int i = 0; i < sizeof(rails) / sizeof(rails[0]); i++) {
			bool enabled;
			uint16_t millivolts;
			axp192_err_t err = axp192_get_rail_millivolts(&axp, rails[i].rail, &millivolts);
			if (err != AXP192_ERROR_OK) {
				printf("%s: get failed\n", rails[i].name);
				continue;
			}

			enabled = false;
			err = axp192_get_rail_state(&axp, rails[i].rail, &enabled);
			if (err != AXP192_ERROR_OK) {
				printf("%s: get state failed\n", rails[i].name);
				continue;
			}

			printf("%s: get %d mV (%s)\n", rails[i].name, millivolts, enabled ? "enabled" : "disabled");

			err = axp192_set_rail_millivolts(&axp, rails[i].rail, rails[i].millivolts);
			if (err != AXP192_ERROR_OK) {
				printf("%s: set failed\n", rails[i].name);
				continue;
			}

			printf("%s: set %d mV\n", rails[i].name, rails[i].millivolts);
		}
	}

	// Battery
	{
		// Configure charger, 4.2V 190mA
		axp192_write_reg(&axp, AXP192_CHARGE_CONTROL_1, 0xC1);
		// Default values - 40 min precharge, 8 hour constant current
		axp192_write_reg(&axp, AXP192_CHARGE_CONTROL_2, 0x41);

		float volts;
		axp192_read(&axp, AXP192_BATTERY_VOLTAGE, &volts);
		printf("Battery voltage: %.2f volts\n", volts);
	}

	// AXP192 GPIOs and pins
	{
		// GPIO1_CONTROL and GPIO2_CONTROL
		// (Direct translations from the datasheet)
		// Bits 7-3: Reserved
		// Bits 2-0: 000: NMOS open-drain output
		//           001: Universal input function
		//           010: PWM1 output, high level is VINT, no pull down less than 100k can be added
		//           011: Reserved
		//           100: ADC input
		//           101: Low output
		//           11X: Floating

		// Open-drain outputs
		// GPIO1: LED sink
		axp192_write_reg(&axp, AXP192_GPIO1_CONTROL, 0);
		// GPIO2: SPK_EN
		axp192_write_reg(&axp, AXP192_GPIO2_CONTROL, 0);

		// GPIO[4:3]_CONTROL
		// (Direct translations from the datasheet)
		// Bit    7: 1: GPIO function
		// Bits 6-4: Reserved
		// Bits 3-2: GPIO4 Function
		//           00: External charging control
		//           01: NMOS open drain
		//           10: GPIO
		//           11: Reserved
		// Bits 1-0: GPIO3 Function
		//           00: External charging control
		//           01: NMOS open drain
		//           10: GPIO
		//           11: ADC Input
		// Set GPIO4 (LCD_RST) and GPIO3 (NC) to GPIO
		axp192_write_reg(&axp, AXP192_GPIO40_FUNCTION_CONTROL, (1 << 7) | (2 << 2) | (2 << 0));

		// PEK (power enable key)
		// Bits 7-6: Boot time
		//           00: 128 ms
		//           01: 512 ms
		//           10: 1 s
		//           11: 2 s
		// Bits 5-4: Long press time
		//           00: 1 s
		//           01: 1.5 s
		//           10: 2 s
		//           11: 2.5 s
		// Bit 3: Shutdown on long press
		// Bit 2: PWROK delay
		//           0: 32 ms
		//           1: 64 ms
		// Bit 1-0: Shutdown button duration
		//           00: 4 s
		//           01: 6 s
		//           10: 8 s
		//           11: 10 s
		// Copy setting from Arduino (0x4c)
		axp192_write_reg(&axp, AXP192_PEK, (1 << 6) | (0 << 4) | (1 << 3) | (1 << 2) | (0 << 0));

		// Arduino code enables all ADC channels
		//axp192_write_reg(&axp, AXP192_ADC_ENABLE_1, 0xff);

		// BATTERY_CHARGE_CONTROL
		// Bit    7: Backup battery charging enable
		// Bits 6-5: Backup battery target voltage
		//           00: 3.1 V
		//           01: 3.0 V
		//           10: 3.0 V (duplicated?)
		//           11: 2.5 V
		// Bits 4-2: Reserved
		// Bits 1-0: Backup battery charge current
		//           00: 50 uA
		//           01: 100 uA
		//           10: 200 uA
		//           11: 400 uA
		axp192_write_reg(&axp, AXP192_BATTERY_CHARGE_CONTROL, (1 << 7) | (1 << 5) | (0 << 0));
	}

	// Test LED
	// Low side is on AXP GPIO1
	{
		printf("LED");
		for (int i = 0; i < 5; i++) {
			set_axp192_gpio_012(&axp, 1, true);
			vTaskDelay(300 / portTICK_PERIOD_MS);
			set_axp192_gpio_012(&axp, 1, false);
			vTaskDelay(300 / portTICK_PERIOD_MS);

			printf(".");
			fflush(stdout);
		}
		printf("Done.\n");
		// Leave the LED on
		set_axp192_gpio_012(&axp, 1, true);
	}

	// Test vibration
	{
		printf("Vibration");
		for (int i = 0; i < 5; i++) {
			axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, true);
			vTaskDelay(300 / portTICK_PERIOD_MS);
			axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
			vTaskDelay(300 / portTICK_PERIOD_MS);

			printf(".");
			fflush(stdout);
		}
		printf("Done.\n");
	}

	// Test LCD backlight
	{
		printf("LCD Backlight");
		for (int i = 0; i < 5; i++) {
			axp192_set_rail_state(&axp, AXP192_RAIL_DCDC3, true);
			vTaskDelay(300 / portTICK_PERIOD_MS);
			axp192_set_rail_state(&axp, AXP192_RAIL_DCDC3, false);
			vTaskDelay(300 / portTICK_PERIOD_MS);

			printf(".");
			fflush(stdout);
		}
		printf("Done.\n");
	}

	// Test bus 5V
	{
		printf("5V Bus");
		for (int i = 0; i < 5; i++) {
			set_internal_5v_bus(&axp, true);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			set_internal_5v_bus(&axp, false);
			vTaskDelay(1000 / portTICK_PERIOD_MS);

			printf(".");
			fflush(stdout);
		}
		printf("Done.\n");
	}

	printf("Finished.\n");
	fflush(stdout);

	for ( ; ; ) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	printf("Restarting now.\n");
	esp_restart();
}
