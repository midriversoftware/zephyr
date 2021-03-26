/*
* Driver for the MicroChip EEPROM 24AA64
*/


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

#define LOG_MODULE_NAME eeprom
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#include <device.h>
#include <drivers/i2c.h>

#include "eeprom/eeprom_mct24aa64.h"

static const struct device *_i2c_dev = NULL;

#define EEPROM_I2C_ADDR	    (0xA0>>1)
#define EEPROM_I2C_BUS_DEV   "I2C_0"
#define EEPROM_BIT_SIZE     (64*1024)
#define EEPROM_MAX_ADDR    ((EEPROM_BIT_SIZE/8)-1)

static inline uint16_t makeBigEndian16(uint16_t valLittle)
{
	return ((valLittle>>8) | (valLittle<<8));
}

static int read_reg8(const struct device *i2c_dev, uint16_t reg_addr, uint8_t *data)
{
	uint16_t wr_addr;
	struct i2c_msg msgs[2];

	/* Register address */
	wr_addr = makeBigEndian16(reg_addr);

	/* Setup I2C messages */

	/* Send the address to read */
	msgs[0].buf = (uint8_t *)&wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. RESTART as neededm and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = 1U;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP;

	// 0 = success
	return i2c_transfer(i2c_dev, &msgs[0], 2, EEPROM_I2C_ADDR);
}


static int write_reg8(const struct device *i2c_dev, uint16_t reg_addr, uint8_t *data)
{
	uint16_t wr_addr;
	struct i2c_msg msgs[2];

	/* Register address */
	wr_addr = makeBigEndian16( reg_addr );
//	wr_addr = reg_addr;

	/* Setup I2C messages */

	/* Send the address to read */
	msgs[0].buf = (uint8_t *)&wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. RESTART as neededm and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = 1U;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	// 0 = success
	return i2c_transfer(i2c_dev, &msgs[0], 2, EEPROM_I2C_ADDR);
}



int eepromInit()
{
	LOG_DBG("%s dev = %p", EEPROM_I2C_BUS_DEV, device_get_binding(EEPROM_I2C_BUS_DEV));
	_i2c_dev = device_get_binding( EEPROM_I2C_BUS_DEV );
	if (!_i2c_dev) 
	{
		LOG_INF("I2C: Device not found.");
		return -1;
	}
    return 0;
}


int eepromRead(uint16_t address, uint8_t *dest, int bytecnt)
{
	int err = 0;
    if (address > EEPROM_MAX_ADDR || address +bytecnt > EEPROM_MAX_ADDR)
        return -1;
    
    while (bytecnt-- > 0)
    {
		err = read_reg8(_i2c_dev, address++, dest);
		if (err != 0)
			break;
        dest++;
        // Note: for large read block consider implementing "Sequential Read" cmds
    }
    return err;
}

int eepromWrite(uint16_t address, uint8_t *src, int bytecnt)
{
	int err = 0;
    if (address > EEPROM_MAX_ADDR || address +bytecnt > EEPROM_MAX_ADDR)
        return -1;
    
    while (bytecnt-- > 0)
    {
		err = write_reg8(_i2c_dev, address, src);
		k_msleep(10);	// time for the burn, spec'd at 5ms
		if (err != 0)
			break;
		address++;
        src++;

        // Note: for large write block consider implementing "Page Write" cmds
    }
    return err;
}
