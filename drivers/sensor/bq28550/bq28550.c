/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq28550

#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>

#include "bq28550.h"

#define BQ28550_SUBCLASS_DELAY 5 /* subclass 64 & 82 needs 5ms delay */


static int bq28550_command_reg_read(struct bq28550_data *bq28550, uint8_t command_code,
				    int16_t *val)
{
	uint8_t i2c_data[2];
	int status;

	status = i2c_write_read(bq28550->i2c, DT_INST_REG_ADDR(0), &command_code, 1 , i2c_data, 2);

	if (status < 0) {
		LOG_ERR("Unable to read register");
		return -EIO;
	}

	*val = (i2c_data[1] << 8) | i2c_data[0];

	return 0;
}

// static int bq28550_command_reg_read(struct bq28550_data *bq28550, uint8_t reg_addr,
// 				    int16_t *val)
// {
// 	uint8_t i2c_data[2];
// 	int status;
//
// 	status = i2c_burst_read(bq28550->i2c, DT_INST_REG_ADDR(0), reg_addr,
// 				i2c_data, 2);
// 	if (status < 0) {
// 		LOG_ERR("Unable to read register");
// 		return -EIO;
// 	}
//
// 	*val = (i2c_data[1] << 8) | i2c_data[0];
//
// 	return 0;
// }
//
// static int bq28550_control_reg_write(struct bq28550_data *bq28550,
// 				     uint16_t subcommand)
// {
// 	uint8_t i2c_data, reg_addr;
// 	int status = 0;
//
// 	reg_addr = BQ28550_COMMAND_CONTROL_LOW;
// 	i2c_data = (uint8_t)((subcommand)&0x00FF);
//
// 	status = i2c_reg_write_byte(bq28550->i2c, DT_INST_REG_ADDR(0), reg_addr,
// 				    i2c_data);
// 	if (status < 0) {
// 		LOG_ERR("Failed to write into control low register");
// 		return -EIO;
// 	}
//
// 	k_msleep(BQ28550_SUBCLASS_DELAY);
//
// 	reg_addr = BQ28550_COMMAND_CONTROL_HIGH;
// 	i2c_data = (uint8_t)((subcommand >> 8) & 0x00FF);
//
// 	status = i2c_reg_write_byte(bq28550->i2c, DT_INST_REG_ADDR(0), reg_addr,
// 				    i2c_data);
// 	if (status < 0) {
// 		LOG_ERR("Failed to write into control high register");
// 		return -EIO;
// 	}
//
// 	return 0;
// }
//
// static int bq28550_command_reg_write(struct bq28550_data *bq28550, uint8_t command,
// 				     uint8_t data)
// {
// 	uint8_t i2c_data, reg_addr;
// 	int status = 0;
//
// 	reg_addr = command;
// 	i2c_data = data;
//
// 	status = i2c_reg_write_byte(bq28550->i2c, DT_INST_REG_ADDR(0), reg_addr,
// 				    i2c_data);
// 	if (status < 0) {
// 		LOG_ERR("Failed to write into control register");
// 		return -EIO;
// 	}
//
// 	return 0;
// }
//
// static int bq28550_read_data_block(struct bq28550_data *bq28550, uint8_t offset,
// 				   uint8_t *data, uint8_t bytes)
// {
// 	uint8_t i2c_data;
// 	int status = 0;
//
// 	i2c_data = BQ28550_EXTENDED_BLOCKDATA_START + offset;
//
// 	status = i2c_burst_read(bq28550->i2c, DT_INST_REG_ADDR(0), i2c_data,
// 				data, bytes);
// 	if (status < 0) {
// 		LOG_ERR("Failed to read block");
// 		return -EIO;
// 	}
//
// 	k_msleep(BQ28550_SUBCLASS_DELAY);
//
// 	return 0;
// }
//
// static int bq28550_get_device_type(struct bq28550_data *bq28550, uint16_t *val)
// {
// 	int status;
//
// 	status =
// 		bq28550_control_reg_write(bq28550, BQ28550_CONTROL_DEVICE_TYPE);
// 	if (status < 0) {
// 		LOG_ERR("Unable to write control register");
// 		return -EIO;
// 	}
//
// 	status = bq28550_command_reg_read(bq28550, BQ28550_COMMAND_CONTROL_LOW,
// 					  val);
//
// 	if (status < 0) {
// 		LOG_ERR("Unable to read register");
// 		return -EIO;
// 	}
//
// 	return 0;
// }

/**
 * @brief sensor value get
 *
 * @return -ENOTSUP for unsupported channels
 */
static int bq28550_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct bq28550_data *bq28550 = dev->data;
	float int_temp;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		val->val1 = ((bq28550->voltage / 1000));
		val->val2 = ((bq28550->voltage % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_AVG_CURRENT:
		val->val1 = ((bq28550->avg_current / 1000));
		val->val2 = ((bq28550->avg_current % 1000) * 1000U);
		break;

	case SENSOR_CHAN_CURRENT:
		val->val1 = ((bq28550->current / 1000));
		val->val2 = ((bq28550->current % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_TEMP:
		int_temp = (bq28550->temperature * 0.1);
		int_temp = int_temp - 273.15;
		val->val1 = (int32_t)int_temp;
		val->val2 = (int_temp - (int32_t)int_temp) * 1000000;
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		val->val1 = bq28550->rel_state_of_charge;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
		val->val1 = (bq28550->full_charge_capacity / 1000);
		val->val2 = ((bq28550->full_charge_capacity % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
		val->val1 = (bq28550->remaining_charge_capacity / 1000);
		val->val2 = ((bq28550->remaining_charge_capacity % 1000) * 1000U);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int bq28550_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct bq28550_data *bq28550 = dev->data;
	int status = 0;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		status = bq28550_command_reg_read(
			bq28550, BQ28550_COMMAND_VOLTAGE, &bq28550->voltage);
		if (status < 0) {
			LOG_ERR("Failed to read voltage");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_AVG_CURRENT:
		status = bq28550_command_reg_read(bq28550,
						  BQ28550_COMMAND_AVG_CURRENT,
				    &bq28550->avg_current);
		if (status < 0) {
			LOG_ERR("Failed to read average current ");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_CURRENT:
		status = bq28550_command_reg_read(bq28550,
						  BQ28550_COMMAND_CURRENT,
				    &bq28550->current);
		if (status < 0) {
			LOG_ERR("Failed to read current ");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_TEMP:
		status = bq28550_command_reg_read(
			bq28550, BQ28550_COMMAND_TEMP,
			&bq28550->temperature);
		if (status < 0) {
			LOG_ERR("Failed to read temperature");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		status = bq28550_command_reg_read(bq28550, BQ28550_COMMAND_REL_SOC,
						  &bq28550->rel_state_of_charge);
		if (status < 0) {
			LOG_ERR("Failed to read state of charge");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
		status = bq28550_command_reg_read(bq28550, BQ28550_COMMAND_REM_CAP,
						  &bq28550->remaining_charge_capacity);
		if (status < 0) {
			LOG_ERR("Failed to read state of charge");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
		status = bq28550_command_reg_read(bq28550, BQ28550_COMMAND_FULL_CAP,
						  &bq28550->full_charge_capacity);
		if (status < 0) {
			LOG_ERR("Failed to read state of charge");
			return -EIO;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief initialise the fuel gauge
 *
 * @return 0 for success
 */
static int bq28550_gauge_init(const struct device *dev)
{
	struct bq28550_data *bq28550 = dev->data;
	const struct bq28550_config *const config = dev->config;
	int status = 0;
	bq28550->i2c = device_get_binding(config->bus_name);
	if (bq28550->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
			config->bus_name);
		return -EINVAL;
	}

// 	status = bq28550_get_device_type(bq28550, &id);
// 	if (status < 0) {
// 		LOG_ERR("Unable to get device ID");
// 		return -EIO;
// 	}
//
// 	if (id != BQ28550_DEVICE_ID) {
// 		LOG_ERR("Invalid Device");
// 		return -EINVAL;
// 	}

// 	uint8_t i2c_data[2];
// 	int status;
//
// 	i2c_write_data[0] = command_code;
//
// 	status = i2c_write_read(bq28550->i2c, DT_INST_REG_ADDR(0), &command_code, 1 , i2c_read_data, 2)

	return 0;
}

static const struct sensor_driver_api bq28550_battery_driver_api = {
	.sample_fetch = bq28550_sample_fetch,
	.channel_get = bq28550_channel_get,
};

#define BQ28550_INIT(index)                                                    \
	static struct bq28550_data bq28550_driver_##index;                     \
										\
	static const struct bq28550_config bq28550_config_##index = {          \
		.bus_name = DT_INST_BUS_LABEL(index),                          \
	};                 							\
										\
	DEVICE_DT_INST_DEFINE(index, &bq28550_gauge_init, device_pm_control_nop,\
			    &bq28550_driver_##index,				\
			    &bq28550_config_##index, POST_KERNEL,              \
			    CONFIG_SENSOR_INIT_PRIORITY,                       \
			    &bq28550_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ28550_INIT)
