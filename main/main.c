#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "axp192.h"
#include "i2c_helper.h"

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

	printf("Finished.\n");
	fflush(stdout);

	for ( ; ; ) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	printf("Restarting now.\n");
	esp_restart();
}
