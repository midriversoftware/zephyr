#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <zephyr.h>
#include <arch/cpu.h>
#include <sys/byteorder.h>
#include <logging/log.h>
#include <sys/util.h>

#include <device.h>
#include <init.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "leds.h"

#define LOG_MODULE_NAME leds
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);


/* The devicetree node identifier for the "led?" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0	DT_GPIO_FLAGS(LED0_NODE, gpios)
#endif

static const struct device *_led_dev = NULL;
void LED_Init()
{
	int ret;
	_led_dev = device_get_binding(LED0);
	if (_led_dev == NULL) {
		LOG_WRN("No LED0");
	}
	ret = gpio_pin_configure(_led_dev, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
	if (ret < 0) {
		LOG_WRN("No cfg LED0");
	}

}

void LED_Red(bool make_on)
{
	if (_led_dev == NULL)
	{
		LOG_WRN("Missing LED init");
	}
	else
	{
		gpio_pin_set(_led_dev, PIN0, (int)make_on);
	}
}
