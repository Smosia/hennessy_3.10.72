/* BMA255 motion sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#ifndef BMA255_H
#define BMA255_H

#include <linux/ioctl.h>

/* 7-bit addr: 0x10 (SDO connected to GND); 0x11 (SDO connected to VDDIO) */
#define BMA255_I2C_ADDR				0x11
/* chip ID */
#define BMA255_FIXED_DEVID			0xFA

 /* BMA255 Register Map  (Please refer to BMA255 Specifications) */
#define BMA255_REG_DEVID			0x00
#define BMA255_REG_OFSX				0x16
#define BMA255_REG_OFSX_HIGH		0x1A
#define BMA255_REG_BW_RATE			0x10
#define BMA255_BW_MASK				0x1f

#define BMA255_BW_250HZ				0x0d
#define BMA255_BW_125HZ				0x0c
#define BMA255_BW_62_5HZ			0x0b
#define BMA255_BW_31_25HZ			0x0a
#define BMA255_BW_15_63HZ			0x09
#define BMA255_BW_7_81HZ			0x08
#define BMA255_BW_200HZ				0x0d
#define BMA255_BW_100HZ				0x0c
#define BMA255_BW_50HZ				0x0b
#define BMA255_BW_25HZ				0x0a

#define BMA255_REG_POWER_CTL		0x11
#define BMA255_REG_DATA_FORMAT		0x0f
#define BMA255_RANGE_MASK			0x0f
#define BMA255_RANGE_2G				0x03
#define BMA255_RANGE_4G				0x05
#define BMA255_RANGE_8G				0x08
#define BMA255_REG_DATAXLOW			0x02
#define BMA255_REG_DATA_RESOLUTION	0x14
#define BMA255_MEASURE_MODE			0x80
#define BMA255_SELF_TEST           	0x32
#define BMA255_SELF_TEST_AXIS_X		0x01
#define BMA255_SELF_TEST_AXIS_Y		0x02
#define BMA255_SELF_TEST_AXIS_Z		0x03
#define BMA255_SELF_TEST_POSITIVE	0x00
#define BMA255_SELF_TEST_NEGATIVE	0x04
#define BMA255_INT_REG_1           	0x16
#define BMA255_INT_REG_2          	0x17

#define BMA255_X_AXIS_LSB_REG		0x02
#define BMA255_X_AXIS_MSB_REG		0x03
#define BMA255_Y_AXIS_LSB_REG		0x04
#define BMA255_Y_AXIS_MSB_REG		0x05
#define BMA255_Z_AXIS_LSB_REG		0x06
#define BMA255_Z_AXIS_MSB_REG		0x07

#define BMA255_EN_SOFT_RESET__POS	0
#define BMA255_EN_SOFT_RESET__LEN	8
#define BMA255_EN_SOFT_RESET__MSK	0xFF
#define BMA255_EN_SOFT_RESET__REG	BMA255_RESET_REG

#define BMA255_EEPROM_CTRL_REG		0x33
#define BMA255_OFFSET_CTRL_REG		0x36
#define BMA255_OFFSET_PARAMS_REG	0x37
#define BMA255_OFFSET_X_AXIS_REG	0x38
#define BMA255_OFFSET_Y_AXIS_REG	0x39
#define BMA255_OFFSET_Z_AXIS_REG	0x3A

#define BMA255_CUT_OFF				0
#define BMA255_OFFSET_TRIGGER_X		1
#define BMA255_OFFSET_TRIGGER_Y		2
#define BMA255_OFFSET_TRIGGER_Z		3

#define BMA255_FAST_CAL_RDY_S__POS	4
#define BMA255_FAST_CAL_RDY_S__LEN	1
#define BMA255_FAST_CAL_RDY_S__MSK	0x10
#define BMA255_FAST_CAL_RDY_S__REG	BMA255_OFFSET_CTRL_REG

#define BMA255_CAL_TRIGGER__POS		5
#define BMA255_CAL_TRIGGER__LEN		2
#define BMA255_CAL_TRIGGER__MSK		0x60
#define BMA255_CAL_TRIGGER__REG		BMA255_OFFSET_CTRL_REG

#define BMA255_COMP_CUTOFF__POS		0
#define BMA255_COMP_CUTOFF__LEN		1
#define BMA255_COMP_CUTOFF__MSK		0x01
#define BMA255_COMP_CUTOFF__REG		BMA255_OFFSET_PARAMS_REG

#define BMA255_COMP_TARGET_OFFSET_X__POS		1
#define BMA255_COMP_TARGET_OFFSET_X__LEN		2
#define BMA255_COMP_TARGET_OFFSET_X__MSK		0x06
#define BMA255_COMP_TARGET_OFFSET_X__REG		BMA255_OFFSET_PARAMS_REG

#define BMA255_COMP_TARGET_OFFSET_Y__POS		3
#define BMA255_COMP_TARGET_OFFSET_Y__LEN		2
#define BMA255_COMP_TARGET_OFFSET_Y__MSK		0x18
#define BMA255_COMP_TARGET_OFFSET_Y__REG		BMA255_OFFSET_PARAMS_REG

#define BMA255_COMP_TARGET_OFFSET_Z__POS		5
#define BMA255_COMP_TARGET_OFFSET_Z__LEN		2
#define BMA255_COMP_TARGET_OFFSET_Z__MSK		0x60
#define BMA255_COMP_TARGET_OFFSET_Z__REG		BMA255_OFFSET_PARAMS_REG

#define BMA255_SUCCESS					0
#define BMA255_ERR_I2C					-1
#define BMA255_ERR_STATUS				-3
#define BMA255_ERR_SETUP_FAILURE		-4
#define BMA255_ERR_GETGSENSORDATA		-5
#define BMA255_ERR_IDENTIFICATION		-6

#define BMA255_BUFSIZE					256

/* soft reset */
#define BMA255_RESET_REG                        0x14
#define BMA255_EN_SOFT_RESET_VALUE        	0xB6

#define BMA255_EN_SOFT_RESET__POS         0
#define BMA255_EN_SOFT_RESET__LEN         8
#define BMA255_EN_SOFT_RESET__MSK         0xFF
#define BMA255_EN_SOFT_RESET__REG         BMA255_RESET_REG

/* self test */
#define BMA255_SELF_TEST_REG                        0x32

#define BMA255_SELF_TEST_AMP__POS               4
#define BMA255_SELF_TEST_AMP__LEN               1
#define BMA255_SELF_TEST_AMP__MSK               0x10
#define BMA255_SELF_TEST_AMP__REG               BMA255_SELF_TEST_REG

#define BMA255_EN_SELF_TEST__POS                0
#define BMA255_EN_SELF_TEST__LEN                2
#define BMA255_EN_SELF_TEST__MSK                0x03
#define BMA255_EN_SELF_TEST__REG                BMA255_SELF_TEST_REG

#define BMA255_NEG_SELF_TEST__POS               2
#define BMA255_NEG_SELF_TEST__LEN               1
#define BMA255_NEG_SELF_TEST__MSK               0x04
#define BMA255_NEG_SELF_TEST__REG               BMA255_SELF_TEST_REG

/* bandwidth */
#define BMA255_BW_7_81HZ			0x08
#define BMA255_BW_15_63HZ			0x09
#define BMA255_BW_31_25HZ			0x0A
#define BMA255_BW_62_50HZ			0x0B
#define BMA255_BW_125HZ				0x0C
#define BMA255_BW_250HZ				0x0D
#define BMA255_BW_500HZ				0x0E
#define BMA255_BW_1000HZ			0x0F

#endif

