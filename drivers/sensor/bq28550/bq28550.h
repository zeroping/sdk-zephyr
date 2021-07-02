/*
 * Copyright(c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BATTERY_BQ28550_H_
#define ZEPHYR_DRIVERS_SENSOR_BATTERY_BQ28550_H_

#include <logging/log.h>
LOG_MODULE_REGISTER(bq28550, CONFIG_SENSOR_LOG_LEVEL);

/*** General Constant ***/
// #define BQ28550_UNSEAL_KEY 0x8000 /* Secret code to unseal the BQ27441-G1A */
// #define BQ28550_DEVICE_ID 0x0421 /* Default device ID */
//
// #define BQ28550_DELAY 1000


/*** Standard Commands ***/
#define BQ28550_COMMAND_TEMP 0x08 /* Temperature() */
#define BQ28550_COMMAND_VOLTAGE 0x09 /* Voltage() */
#define BQ28550_COMMAND_CURRENT 0x0a /* Current() */
#define BQ28550_COMMAND_AVG_CURRENT 0x0b /* AverageCurrent() */
#define BQ28550_COMMAND_REL_SOC 0x0d /* RelativeStateOfCharge() */
#define BQ28550_COMMAND_REM_CAP 0x0f /* RemainingCapacity() */
#define BQ28550_COMMAND_FULL_CAP 0x10 /* FullChargeCapacity() */

// /*** Standard Commands ***/
// #define BQ28550_COMMAND_CONTROL_LOW 0x00 /* Control() low register */
// #define BQ28550_COMMAND_CONTROL_HIGH 0x01 /* Control() high register */
// #define BQ28550_COMMAND_TEMP 0x02 /* Temperature() */
// #define BQ28550_COMMAND_VOLTAGE 0x04 /* Voltage() */
// #define BQ28550_COMMAND_FLAGS 0x06 /* Flags() */
// #define BQ28550_COMMAND_NOM_CAPACITY 0x08 /* NominalAvailableCapacity() */
// #define BQ28550_COMMAND_AVAIL_CAPACITY 0x0A /* FullAvailableCapacity() */
// #define BQ28550_COMMAND_REM_CAPACITY 0x0C /* RemainingCapacity() */
// #define BQ28550_COMMAND_FULL_CAPACITY 0x0E /* FullChargeCapacity() */
// #define BQ28550_COMMAND_AVG_CURRENT 0x10 /* AverageCurrent() */
// #define BQ28550_COMMAND_STDBY_CURRENT 0x12 /* StandbyCurrent() */
// #define BQ28550_COMMAND_MAX_CURRENT 0x14 /* MaxLoadCurrent() */
// #define BQ28550_COMMAND_AVG_POWER 0x18 /* AveragePower() */
// #define BQ28550_COMMAND_SOC 0x1C /* StateOfCharge() */
// #define BQ28550_COMMAND_INT_TEMP 0x1E /* InternalTemperature() */
// #define BQ28550_COMMAND_SOH 0x20 /* StateOfHealth() */
// #define BQ28550_COMMAND_REM_CAP_UNFL 0x28 /* RemainingCapacityUnfiltered() */
// #define BQ28550_COMMAND_REM_CAP_FIL 0x2A /* RemainingCapacityFiltered() */
// #define BQ28550_COMMAND_FULL_CAP_UNFL 0x2C /* FullChargeCapacityUnfiltered() */
// #define BQ28550_COMMAND_FULL_CAP_FIL 0x2E /* FullChargeCapacityFiltered() */
// #define BQ28550_COMMAND_SOC_UNFL 0x30 /* StateOfChargeUnfiltered() */
//
// /*** Control Sub-Commands ***/
// #define BQ28550_CONTROL_STATUS 0x0000
// #define BQ28550_CONTROL_DEVICE_TYPE 0x0001
// #define BQ28550_CONTROL_FW_VERSION 0x0002
// #define BQ28550_CONTROL_DM_CODE 0x0004
// #define BQ28550_CONTROL_PREV_MACWRITE 0x0007
// #define BQ28550_CONTROL_CHEM_ID 0x0008
// #define BQ28550_CONTROL_BAT_INSERT 0x000C
// #define BQ28550_CONTROL_BAT_REMOVE 0x000D
// #define BQ28550_CONTROL_SET_HIBERNATE 0x0011
// #define BQ28550_CONTROL_CLEAR_HIBERNATE 0x0012
// #define BQ28550_CONTROL_SET_CFGUPDATE 0x0013
// #define BQ28550_CONTROL_SHUTDOWN_ENABLE 0x001B
// #define BQ28550_CONTROL_SHUTDOWN 0x001C
// #define BQ28550_CONTROL_SEALED 0x0020
// #define BQ28550_CONTROL_PULSE_SOC_INT 0x0023
// #define BQ28550_CONTROL_RESET 0x0041
// #define BQ28550_CONTROL_SOFT_RESET 0x0042
// #define BQ28550_CONTROL_EXIT_CFGUPDATE 0x0043
// #define BQ28550_CONTROL_EXIT_RESIM 0x0044
//
// /*** Extended Data Commands ***/
// #define BQ28550_EXTENDED_OPCONFIG 0x3A /* OpConfig() */
// #define BQ28550_EXTENDED_CAPACITY 0x3C /* DesignCapacity() */
// #define BQ28550_EXTENDED_DATA_CLASS 0x3E /* DataClass() */
// #define BQ28550_EXTENDED_DATA_BLOCK 0x3F /* DataBlock() */
// #define BQ28550_EXTENDED_BLOCKDATA_START 0x40 /* BlockData_start() */
// #define BQ28550_EXTENDED_BLOCKDATA_END 0x5F /* BlockData_end() */
// #define BQ28550_EXTENDED_CHECKSUM 0x60 /* BlockDataCheckSum() */
// #define BQ28550_EXTENDED_DATA_CONTROL 0x61 /* BlockDataControl() */
// #define BQ28550_EXTENDED_BLOCKDATA_DESIGN_CAP_HIGH 0x4A /* BlockData */
// #define BQ28550_EXTENDED_BLOCKDATA_DESIGN_CAP_LOW 0x4B
// #define BQ28550_EXTENDED_BLOCKDATA_DESIGN_ENR_HIGH 0x4C
// #define BQ28550_EXTENDED_BLOCKDATA_DESIGN_ENR_LOW 0x4D
// #define BQ28550_EXTENDED_BLOCKDATA_TERMINATE_VOLT_HIGH 0x50
// #define BQ28550_EXTENDED_BLOCKDATA_TERMINATE_VOLT_LOW 0x51
// #define BQ28550_EXTENDED_BLOCKDATA_TAPERRATE_HIGH 0x5B
// #define BQ28550_EXTENDED_BLOCKDATA_TAPERRATE_LOW 0x5C

struct bq28550_data {
	const struct device *i2c;
	uint16_t voltage;
	int16_t avg_current;
	int16_t current;
// 	int16_t stdby_current;
// 	int16_t max_load_current;
// 	int16_t avg_power;
	uint16_t rel_state_of_charge;
// 	int16_t state_of_health;
	uint16_t temperature;
 	uint16_t full_charge_capacity;
	uint16_t remaining_charge_capacity;
// 	uint16_t nom_avail_capacity;
// 	uint16_t full_avail_capacity;
};


struct bq28550_config {
	char *bus_name;
};

#endif
