// SPDX: MIT
// Copyright 2021 Brian Starkey <stark3y@gmail.com>
// Portions from lvgl example: https://github.com/lvgl/lv_port_esp32/blob/master/main/main.c

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "axp192.h"
#include "i2c_helper.h"

#include "lvgl.h"
#include "lvgl_helpers.h"

#define LV_TICK_PERIOD_MS 1

// If true, we'll run through the tests at boot
#define TEST_BEFORE_GUI_START 0
// If true, we'll bring up the GUI
#define START_GUI             1

// Global for convenience
const axp192_t axp = {
	.read = &i2c_read,
	.write = &i2c_write,
};

static void set_led(const axp192_t *axp, bool on);
static void set_vibration(const axp192_t *axp, bool on);
static void set_internal_5v_bus(const axp192_t *axp, bool on);

enum toggle_id {
	TOGGLE_LED = 0,
	TOGGLE_VIB,
	TOGGLE_5V,
};

static void toggle_event_cb(lv_obj_t *toggle, lv_event_t event)
{
	if(event == LV_EVENT_VALUE_CHANGED) {
		bool state = lv_switch_get_state(toggle);
		enum toggle_id *id = lv_obj_get_user_data(toggle);

		// Note: This is running in the GUI thread, so prolonged i2c
		// comms might cause some jank
		switch (*id) {
		case TOGGLE_LED:
			set_led(&axp, state);
			break;
		case TOGGLE_VIB:
			set_vibration(&axp, state);
			break;
		case TOGGLE_5V:
			set_internal_5v_bus(&axp, state);
			break;
		}
	}
}

static void gui_timer_tick(void *arg)
{
	// Unused
	(void) arg;

	lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void gui_thread(void *pvParameter)
{
	(void) pvParameter;

	static lv_color_t bufs[2][DISP_BUF_SIZE];
	static lv_disp_buf_t disp_buf;
	uint32_t size_in_px = DISP_BUF_SIZE;

	// Set up the frame buffers
	lv_disp_buf_init(&disp_buf, &bufs[0], &bufs[1], size_in_px);

	// Set up the display driver
	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.flush_cb = disp_driver_flush;
	disp_drv.buffer = &disp_buf;
	lv_disp_drv_register(&disp_drv);

	// Register the touch screen. All of the properties of it
	// are set via the build config
	lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.read_cb = touch_driver_read;
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	lv_indev_drv_register(&indev_drv);

	// Timer to drive the main lvgl tick
	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &gui_timer_tick,
		.name = "periodic_gui"
	};
	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));


	// Full screen root container
	lv_obj_t *root = lv_cont_create(lv_scr_act(), NULL);
	lv_obj_set_size(root, 320, 240);
	lv_cont_set_layout(root, LV_LAYOUT_COLUMN_MID);
	// Don't let the containers be clicked on
	lv_obj_set_click(root, false);

	// Create rows of switches for different functions
	struct {
		const char *label;
		bool init;
		enum toggle_id id;
	} switches[] = {
		{ "LED",     true,  TOGGLE_LED },
		{ "Vibrate", false, TOGGLE_VIB },
		{ "5V Bus",  false, TOGGLE_5V },
	};
	for (int i = 0; i < sizeof(switches) / sizeof(switches[0]); i++) {
		lv_obj_t *row = lv_cont_create(root, NULL);
		lv_cont_set_layout(row, LV_LAYOUT_ROW_MID);
		lv_obj_set_size(row, 200, 0);
		lv_cont_set_fit2(row, LV_FIT_NONE, LV_FIT_TIGHT);
		// Don't let the containers be clicked on
		lv_obj_set_click(row, false);

		lv_obj_t *toggle = lv_switch_create(row, NULL);
		if (switches[i].init) {
			lv_switch_on(toggle, LV_ANIM_OFF);
		}
		lv_obj_set_user_data(toggle, &switches[i].id);
		lv_obj_set_event_cb(toggle, toggle_event_cb);

		lv_obj_t *label = lv_label_create(row, NULL);
		lv_label_set_text(label, switches[i].label);
	}

	while (1) {
		vTaskDelay(10 / portTICK_PERIOD_MS);

		lv_task_handler();
	}

	// Never returns
}

static void set_axp192_gpio_012(const axp192_t *axp, int gpio, bool low)
{
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

static void set_led(const axp192_t *axp, bool on)
{
	set_axp192_gpio_012(axp, 1, on);
}

static void set_vibration(const axp192_t *axp, bool on)
{
	axp192_set_rail_state(axp, AXP192_RAIL_LDO3, on);
}

static void set_internal_5v_bus(const axp192_t *axp, bool on)
{
	// To enable the on-board 5V supply, first N_VBUSEN needs to be pulled
	// high using GPIO0, then we can enable the EXTEN output, to enable
	// the SMPS.
	// To disable it (so either no 5V, or externally supplied 5V), we
	// do the opposite: First disable EXTEN, then leave GPIO0 floating.
	// N_VBUSEN will be pulled down by the on-board resistor.
	// Side note: The pull down is 10k according to the schematic, so that's
	// a 0.5 mA drain from the GPIO0 LDO as long as the bus supply is active.

	if (on) {
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

	// Have to initialise these here, because lvgl_driver_init unconditionally
	// sets up the i2c bus
	lv_init();
	lvgl_driver_init();

	// Don't i2c_init()
	// i2c_init();

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
		// Set GPIO4 (LCD_RST) and GPIO3 (NC) to NMOS OD
		axp192_write_reg(&axp, AXP192_GPIO40_FUNCTION_CONTROL, (1 << 7) | (1 << 2) | (1 << 0));

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
	if (TEST_BEFORE_GUI_START) {
		printf("LED");
		for (int i = 0; i < 5; i++) {
			set_led(&axp, true);
			vTaskDelay(300 / portTICK_PERIOD_MS);
			set_led(&axp, false);
			vTaskDelay(300 / portTICK_PERIOD_MS);

			printf(".");
			fflush(stdout);
		}
		printf("Done.\n");

	}
	// Always turn the LED on
	set_led(&axp, true);

	// Test vibration
	if (TEST_BEFORE_GUI_START) {
		printf("Vibration");
		for (int i = 0; i < 5; i++) {
			set_vibration(&axp, true);
			vTaskDelay(300 / portTICK_PERIOD_MS);
			set_vibration(&axp, false);
			vTaskDelay(300 / portTICK_PERIOD_MS);

			printf(".");
			fflush(stdout);
		}
		printf("Done.\n");
	}

	// Test LCD backlight
	if (TEST_BEFORE_GUI_START) {
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
	if (TEST_BEFORE_GUI_START) {
		printf("5V Bus");
		set_internal_5v_bus(&axp, true);
		printf(".");
		fflush(stdout);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		set_internal_5v_bus(&axp, false);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("Done.\n");
	}

	// Graphics
	if (START_GUI) {
		// Backlight
		axp192_set_rail_state(&axp, AXP192_RAIL_DCDC3, true);

		// Logic
		axp192_set_rail_state(&axp, AXP192_RAIL_LDO2, true);

		// Wait a bit for everything to settle
		vTaskDelay(100 / portTICK_PERIOD_MS);

		// Needs to be pinned to a core
		xTaskCreatePinnedToCore(gui_thread, "gui", 4096*2, NULL, 0, NULL, 1);
	}

	printf("Running...\n");
	fflush(stdout);

	for ( ; ; ) {
		vTaskDelay(portMAX_DELAY);
	}
	printf("Restarting now.\n");
	esp_restart();
}
