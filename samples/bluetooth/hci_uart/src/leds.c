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
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0	DT_GPIO_FLAGS(LED0_NODE, gpios)
#endif

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1	DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1	DT_GPIO_FLAGS(LED1_NODE, gpios)
#endif

#if DT_NODE_HAS_STATUS(LED2_NODE, okay)
#define LED2	DT_GPIO_LABEL(LED2_NODE, gpios)
#define PIN2	DT_GPIO_PIN(LED2_NODE, gpios)
#define FLAGS2	DT_GPIO_FLAGS(LED2_NODE, gpios)
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

	_led_dev = device_get_binding(LED1);
	if (_led_dev == NULL) {
		LOG_WRN("No LED1");
	}
	ret = gpio_pin_configure(_led_dev, PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);
	if (ret < 0) {
		LOG_WRN("No cfg LED1");
	}

	_led_dev = device_get_binding(LED2);
	if (_led_dev == NULL) {
		LOG_WRN("No LED2");
	}
	ret = gpio_pin_configure(_led_dev, PIN2, GPIO_OUTPUT_ACTIVE | FLAGS2);
	if (ret < 0) {
		LOG_WRN("No cfg LED2");
	}
}

void LED_Blue(bool make_on)
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
void LED_Yellow(bool make_on)
{
	if (_led_dev == NULL)
	{
		LOG_WRN("Missing LED init");
	}
	else
	{
		gpio_pin_set(_led_dev, PIN1, (int)make_on);
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
		gpio_pin_set(_led_dev, PIN2, (int)make_on);
	}
}
