#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <sensor.h>

#include <logging/log.h>
#define LOG_MODULE_NAME main_module
#define LOG_LEVEL CONFIG_MAIN_MODULE_LOG_LEVEL
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "devices.h"
#include "mesh.h"

struct device *led_device[2];
struct device *ccs811;
struct device *bme280;

static void devices_init(void)
{
        LOG_DBG("Initialising...");

	led_device[0] = device_get_binding(LED0_GPIO_CONTROLLER);
	gpio_pin_configure(led_device[0], LED0_GPIO_PIN,
			   GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
	gpio_pin_write(led_device[0], LED0_GPIO_PIN, 0);

	led_device[1] = device_get_binding(LED1_GPIO_CONTROLLER);
	gpio_pin_configure(led_device[1], LED1_GPIO_PIN,
			   GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
	gpio_pin_write(led_device[1], LED1_GPIO_PIN, 0);

	ccs811 = device_get_binding(CONFIG_CCS811_NAME);
	if (!ccs811) {
		LOG_ERR("Failed to get CCS811 binding");
	}

	bme280 = device_get_binding("BME280");
	if (!bme280) {
		LOG_ERR("Failed to get BME280 binding");
	}
}

void main(void)
{
        int ret;

        devices_init();

        ret = bt_enable(bt_ready);
        if (ret) {
                LOG_ERR("Bluetooth init failed (err %d)", ret);
        }
}