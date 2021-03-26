/*
 *
 *
 * 
 */

#define DT_DRV_COMPAT mct_24AA64_eeprom

asdfasdfasdf

/**
 * @file
 * @brief EEPROM driver for MicroChip 24AA64
 *
 * This driver supports the EEPROM found on the Delphian Bridge board
 *
 * @note 
 */

#include <drivers/eeprom.h>

#define LOG_LEVEL       CONFIG_EEPROM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(eeprom_24aa64);

struct eeprom_24aa64_config {
	size_t size;
};

static int eeprom_24aa64_read(const struct device *dev,
				off_t offset, void *data, size_t len)
{
	const struct eeprom_24aa64_config *config = dev->config;
	uint32_t cmd[5];
	int ret;

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}


// MRS TODO 
		LOG_WRN("NOT IMPLEMENTED: eeprom_24aa64_read");

	// cmd[0] = IAP_CMD_EEPROM_READ;
	// cmd[1] = offset;
	// cmd[2] = (uint32_t) data;
	// cmd[3] = len;
	// cmd[4] = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000;

	// ret = iap_cmd(cmd);

	// if (ret != IAP_STATUS_CMD_SUCCESS) {
	// 	LOG_ERR("failed to read EEPROM (offset=%08x len=%d err=%d)",
	// 		(unsigned int) offset, len, ret);
	// 	return -EINVAL;
	// }

	return 0;
}

static int eeprom_24aa64_write(const struct device *dev,
				 off_t offset, const void *data, size_t len)
{
	const struct eeprom_24aa64_config *config = dev->config;
	uint32_t cmd[5];
	int ret;

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to write past device boundary");
		return -EINVAL;
	}
// MRS TODO
		LOG_WRN("NOT IMPLEMENTED: eeprom_24aa64_write");

	// cmd[0] = IAP_CMD_EEPROM_WRITE;
	// cmd[1] = offset;
	// cmd[2] = (uint32_t) data;
	// cmd[3] = len;
	// cmd[4] = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000;

	// ret = iap_cmd(cmd);

	// if (ret != IAP_STATUS_CMD_SUCCESS) {
	// 	LOG_ERR("failed to write EEPROM (offset=%08x len=%d err=%d)",
	// 		(unsigned int) offset, len, ret);
	// 	return -EINVAL;
	// }

	return 0;
}

static size_t eeprom_24aa64_size(const struct device *dev)
{
	const struct eeprom_24aa64_config *config = dev->config;

	return config->size;
}

static int eeprom_24aa64_init(const struct device *dev)
{
	return 0;
}

static const struct eeprom_driver_api eeprom_24aa64_api = {
	.read = eeprom_24aa64_read,
	.write = eeprom_24aa64_write,
	.size = eeprom_24aa64_size,
};

static const struct eeprom_24aa64_config eeprom_config = {
	.size = DT_INST_PROP(0, size),
};

DEVICE_DT_INST_DEFINE(0, &eeprom_24aa64_init, device_pm_control_nop, NULL,
		    &eeprom_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &eeprom_24aa64_api);
