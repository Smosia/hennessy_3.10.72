/* BMA255 motion sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/module.h>

#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "bma255.h"
#include <linux/hwmsen_helper.h>

#include <accel.h>

#if defined(TARGET_C90)
#define CONFIG_OF_DT
#endif

/* Use CALIBRATION_TO_FILE define for saving cal data to file      */
/* Without CALIBRATION_TO_FILE define, Cal data is saved to MISC2 */
#define CALIBRATION_TO_FILE 1

#if defined(CALIBRATION_TO_FILE)
#include <linux/syscalls.h>
#include <linux/fs.h>
#endif

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{.compatible = "mediatek,gsensor",},
	{},
};
#endif

/* range for selftest */
enum BMA_RANGE_ENUM {
	BMA_RANGE_2G = 0x0,	/* +/- 2G */
	BMA_RANGE_4G,		/* +/- 4G */
	BMA_RANGE_8G,		/* +/- 8G */

	BMA_UNDEFINED_RANGE = 0xff
};


#if defined(TARGET_C90)
/* #include "../../tc1_interface/gpt/lg_partition.h" //for MT6732 */
#else
/* #include "../../tc1_interface/pmt/lg_partition.h" //for MT6582 */
#endif

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA255 255
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
/* #define CONFIG_BMA255_LOWPASS */   /*apply low pass filter on output*/
#define SW_CALIBRATION
/*----------------------------------------------------------------------------*/
#define BMA255_AXIS_X          0
#define BMA255_AXIS_Y          1
#define BMA255_AXIS_Z          2
#define BMA255_AXES_NUM        3
#define BMA255_DATA_LEN        6
#define BMA255_DEV_NAME        "BMA255"
#define BMA255_MODE_NORMAL      0
#define BMA255_MODE_LOWPOWER    1
#define BMA255_MODE_SUSPEND     2
#define BMA255_ACC_X_LSB__POS           4
#define BMA255_ACC_X_LSB__LEN           4
#define BMA255_ACC_X_LSB__MSK           0xF0
#define BMA255_ACC_X_LSB__REG           BMA255_X_AXIS_LSB_REG
#define BMA255_ACC_X_MSB__POS           0
#define BMA255_ACC_X_MSB__LEN           8
#define BMA255_ACC_X_MSB__MSK           0xFF
#define BMA255_ACC_X_MSB__REG           BMA255_X_AXIS_MSB_REG
#define BMA255_ACC_Y_LSB__POS           4
#define BMA255_ACC_Y_LSB__LEN           4
#define BMA255_ACC_Y_LSB__MSK           0xF0
#define BMA255_ACC_Y_LSB__REG           BMA255_Y_AXIS_LSB_REG
#define BMA255_ACC_Y_MSB__POS           0
#define BMA255_ACC_Y_MSB__LEN           8
#define BMA255_ACC_Y_MSB__MSK           0xFF
#define BMA255_ACC_Y_MSB__REG           BMA255_Y_AXIS_MSB_REG
#define BMA255_ACC_Z_LSB__POS           4
#define BMA255_ACC_Z_LSB__LEN           4
#define BMA255_ACC_Z_LSB__MSK           0xF0
#define BMA255_ACC_Z_LSB__REG           BMA255_Z_AXIS_LSB_REG
#define BMA255_ACC_Z_MSB__POS           0
#define BMA255_ACC_Z_MSB__LEN           8
#define BMA255_ACC_Z_MSB__MSK           0xFF
#define BMA255_ACC_Z_MSB__REG           BMA255_Z_AXIS_MSB_REG
#define BMA255_EN_LOW_POWER__POS          6
#define BMA255_EN_LOW_POWER__LEN          1
#define BMA255_EN_LOW_POWER__MSK          0x40
#define BMA255_EN_LOW_POWER__REG          BMA255_REG_POWER_CTL
#define BMA255_EN_SUSPEND__POS            7
#define BMA255_EN_SUSPEND__LEN            1
#define BMA255_EN_SUSPEND__MSK            0x80
#define BMA255_EN_SUSPEND__REG            BMA255_REG_POWER_CTL
#define BMA255_RANGE_SEL__POS             0
#define BMA255_RANGE_SEL__LEN             4
#define BMA255_RANGE_SEL__MSK             0x0F
#define BMA255_RANGE_SEL__REG             BMA255_REG_DATA_FORMAT
#define BMA255_BANDWIDTH__POS             0
#define BMA255_BANDWIDTH__LEN             5
#define BMA255_BANDWIDTH__MSK             0x1F
#define BMA255_BANDWIDTH__REG             BMA255_REG_BW_RATE
/* #ifdef CONFIG_MACH_LGE */
#define BMA255_ACCEL_CALIBRATION 1
#ifdef BMA255_ACCEL_CALIBRATION
#define BMA255_SHAKING_DETECT_THRESHOLD	(300)	/* clubsh cal2 50 -> 200 */
/* Calibration zero-g offset check */
#define BMA255_AXIS_X             0
#define BMA255_AXIS_Y             1
#define BMA255_AXIS_Z             2
#define CALIBRATION_DATA_AMOUNT         10
#if 0				/* vendor recommand */
#define TESTLIMIT_XY                     (175)
#define TESTLIMIT_Z_USL_LSB             (1270)
#define TESTLIMIT_Z_LSL_LSB             (778)
#endif
#if defined(TARGET_Y70)
#if 1
#define TESTLIMIT_XY                     (240)
#else
int TESTLIMIT_XY = 240;
#endif
#define TESTLIMIT_Z_USL_LSB             (1226)
#define TESTLIMIT_Z_LSL_LSB             (822)
#elif defined(TARGET_Y90)
#define TESTLIMIT_XY                     (232)
#define TESTLIMIT_Z_USL_LSB             (1226)
#define TESTLIMIT_Z_LSL_LSB             (822)
#elif defined(TARGET_C90)
#define TESTLIMIT_XY                     (237)
#define TESTLIMIT_Z_USL_LSB             (1226)
#define TESTLIMIT_Z_LSL_LSB             (822)
#else				/* Y50 */
#define TESTLIMIT_XY                     (233)
#define TESTLIMIT_Z_USL_LSB             (1226)
#define TESTLIMIT_Z_LSL_LSB             (822)
#endif
/* Calibration zero-g offset check */
#endif
struct bma255acc {
	s16 x;
    s16 y;
    s16 z;
};
/* LGE_CHANGE_E : 2012-12-07 taebum81.kim@lge.com AT%SURV : (1) */


#define BMA255_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMA255_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id bma255_i2c_id[] = { {BMA255_DEV_NAME, 0}, {} };

static struct i2c_board_info bma255_i2c_info __initdata = {
	I2C_BOARD_INFO(BMA255_DEV_NAME, BMA255_I2C_ADDR)
};

/*----------------------------------------------------------------------------*/
static int bma255_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bma255_i2c_remove(struct i2c_client *client);
static int bma255_do_calibration(void);


/*----------------------------------------------------------------------------*/
typedef enum {
	BMA_TRC_FILTER = 0x01,
	BMA_TRC_RAWDATA = 0x02,
	BMA_TRC_IOCTL = 0x04,
	BMA_TRC_CALI = 0X08,
	BMA_TRC_INFO = 0X10,
} BMA_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][BMA255_AXES_NUM];
	int sum[BMA255_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct bma255_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert cvt;

	/* for selftest */
	enum BMA_RANGE_ENUM range;
	u8 bandwidth;

	/*misc */
	struct data_resolution *reso;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
	s16 cali_sw[BMA255_AXES_NUM + 1];

	/*data */
	s8 offset[BMA255_AXES_NUM + 1];	/*+1: for 4-byte alignment */
	s16 data[BMA255_AXES_NUM + 1];

#if defined(CONFIG_BMA255_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
/* LGE_CHANGE_S : 2012-12-07 taebum81.kim@lge.com sensor SURV shake detection : (2) */
	atomic_t fast_calib_x_rslt;
	atomic_t fast_calib_y_rslt;
	atomic_t fast_calib_z_rslt;
	atomic_t fast_calib_rslt;
/* LG_CHANGE_E : 2012-12-07 taebum81.kim@lge.com sensor SURV shake detection : (2) */
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver bma255_i2c_driver = {
	.driver = {
		   .name = BMA255_DEV_NAME,
		   },
	.probe = bma255_i2c_probe,
	.remove = bma255_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = bma255_suspend,
	.resume = bma255_resume,
#endif
	.id_table = bma255_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *bma255_i2c_client;
static struct platform_driver bma255_gsensor_driver;
static struct bma255_i2c_data *obj_i2c_data;
static bool sensor_power = true;
static int test_status;
static GSENSOR_VECTOR3D gsensor_gain;
/* static char selftestRes[8]= {0}; */

static int data_count;

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               pr_debug("[Gsensor] %s\n", __func__)
#define GSE_ERR(fmt, args...)    pr_debug("[Gsensor] %s %d : "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_debug(fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution bma255_data_resolution[1] = {
	/* combination by {FULL_RES,RANGE} */
	{{4, 0}, 512},
	/* dataformat +/-2g  in 12-bit resolution;
	   { 1, 0} = 1.0= (2*2*1000)/(2^12);  1024 = (2^12)/(2*2) */
};

/*----------------------------------------------------------------------------*/
/* static struct data_resolution bma255_offset_resolution = {{1, 0}, 1024}; */

/*--------------------BMA255 power control function----------------------------------*/
static void BMA255_power(struct acc_hw *hw, unsigned int on)
{
	static unsigned int power_on;

	if (hw->power_id != POWER_NONE_MACRO) {	/* have externel LDO */
		GSE_LOG("power %s\n", on ? "on" : "off");
		if (power_on == on) {	/* power status not change */
			GSE_LOG("ignore power control: %d\n", on);
		} else if (on) {	/* power on */
			if (!hwPowerOn(hw->power_id, hw->power_vol, "BMA255"))
				GSE_ERR("power on fails!!\n");

		} else {	/* power off */

			if (!hwPowerDown(hw->power_id, "BMA255"))
				GSE_ERR("power off fail!!\n");

		}
	}
	power_on = on;
}

/*----------------------------------------------------------------------------*/
static int bma255_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr,
				  unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma255_smbus_write_byte(struct i2c_client *client, unsigned char reg_addr,
				   unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma255_smbus_read_byte_block(struct i2c_client *client, unsigned char reg_addr,
					unsigned char *data, unsigned char len)
{
	s32 dummy;

	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int BMA255_SetDataResolution(struct bma255_i2c_data *obj)
{

/* set g sensor data resolution here */

/* BMA255 only can set to 10-bit dataresolution, so do nothing in bma255 driver here */

/* end of set dataresolution */

/* we set measure range from -2g to +2g in BMA255_SetDataFormat(client, BMA255_RANGE_2G),
						    and set 10-bit dataresolution BMA255_SetDataResolution() */

/* so bma255_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here */

	obj->reso = &bma255_data_resolution[0];
	return 0;

/* if you changed the measure range, for example call: BMA255_SetDataFormat(client, BMA255_RANGE_4G),
    you must set the right value to bma255_data_resolution */

}

/*----------------------------------------------------------------------------*/
static int BMA255_ReadData(struct i2c_client *client, s16 data[BMA255_AXES_NUM])
{
#ifdef CONFIG_BMA255_LOWPASS
	struct bma255_i2c_data *priv = i2c_get_clientdata(client);
#endif
	u8 addr = BMA255_REG_DATAXLOW;
	u8 buf[BMA255_DATA_LEN] = { 0 };
	int err = 0;

	err = hwmsen_read_block(client, addr, buf, BMA255_DATA_LEN);

	if (NULL == client) {
		err = -EINVAL;
	} else if (err) {
		GSE_ERR("error: %d\n", err);
	} else {
		/* Convert sensor raw data to 16-bit integer */
		data[BMA255_AXIS_X] = BMA255_GET_BITSLICE(buf[0], BMA255_ACC_X_LSB)
		    | (BMA255_GET_BITSLICE(buf[1], BMA255_ACC_X_MSB) << BMA255_ACC_X_LSB__LEN);
		data[BMA255_AXIS_X] =
		    data[BMA255_AXIS_X] << (sizeof(short) * 8 -
					    (BMA255_ACC_X_LSB__LEN + BMA255_ACC_X_MSB__LEN));
		data[BMA255_AXIS_X] =
		    data[BMA255_AXIS_X] >> (sizeof(short) * 8 -
					    (BMA255_ACC_X_LSB__LEN + BMA255_ACC_X_MSB__LEN));
		data[BMA255_AXIS_Y] = BMA255_GET_BITSLICE(buf[2], BMA255_ACC_Y_LSB)
		    | (BMA255_GET_BITSLICE(buf[3], BMA255_ACC_Y_MSB) << BMA255_ACC_Y_LSB__LEN);
		data[BMA255_AXIS_Y] =
		    data[BMA255_AXIS_Y] << (sizeof(short) * 8 -
					    (BMA255_ACC_Y_LSB__LEN + BMA255_ACC_Y_MSB__LEN));
		data[BMA255_AXIS_Y] =
		    data[BMA255_AXIS_Y] >> (sizeof(short) * 8 -
					    (BMA255_ACC_Y_LSB__LEN + BMA255_ACC_Y_MSB__LEN));
		data[BMA255_AXIS_Z] = BMA255_GET_BITSLICE(buf[4], BMA255_ACC_Z_LSB)
		    | (BMA255_GET_BITSLICE(buf[5], BMA255_ACC_Z_MSB) << BMA255_ACC_Z_LSB__LEN);
		data[BMA255_AXIS_Z] =
		    data[BMA255_AXIS_Z] << (sizeof(short) * 8 -
					    (BMA255_ACC_Z_LSB__LEN + BMA255_ACC_Z_MSB__LEN));
		data[BMA255_AXIS_Z] =
		    data[BMA255_AXIS_Z] >> (sizeof(short) * 8 -
					    (BMA255_ACC_Z_LSB__LEN + BMA255_ACC_Z_MSB__LEN));

#ifdef CONFIG_BMA255_LOWPASS
		if (atomic_read(&priv->filter)) {
			if (atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend)) {
				int idx, firlen = atomic_read(&priv->firlen);

				if (priv->fir.num < firlen) {
					priv->fir.raw[priv->fir.num][BMA255_AXIS_X] =
					    data[BMA255_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA255_AXIS_Y] =
					    data[BMA255_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA255_AXIS_Z] =
					    data[BMA255_AXIS_Z];
					priv->fir.sum[BMA255_AXIS_X] += data[BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] += data[BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] += data[BMA255_AXIS_Z];
					if (atomic_read(&priv->trace) & BMA_TRC_FILTER) {
						GSE_LOG
						    ("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n",
						     priv->fir.num,
						     priv->fir.raw[priv->fir.num][BMA255_AXIS_X],
						     priv->fir.raw[priv->fir.num][BMA255_AXIS_Y],
						     priv->fir.raw[priv->fir.num][BMA255_AXIS_Z],
						     priv->fir.sum[BMA255_AXIS_X],
						     priv->fir.sum[BMA255_AXIS_Y],
						     priv->fir.sum[BMA255_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				} else {
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA255_AXIS_X] -=
					    priv->fir.raw[idx][BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] -=
					    priv->fir.raw[idx][BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] -=
					    priv->fir.raw[idx][BMA255_AXIS_Z];
					priv->fir.raw[idx][BMA255_AXIS_X] = data[BMA255_AXIS_X];
					priv->fir.raw[idx][BMA255_AXIS_Y] = data[BMA255_AXIS_Y];
					priv->fir.raw[idx][BMA255_AXIS_Z] = data[BMA255_AXIS_Z];
					priv->fir.sum[BMA255_AXIS_X] += data[BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] += data[BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] += data[BMA255_AXIS_Z];
					priv->fir.idx++;
					data[BMA255_AXIS_X] = priv->fir.sum[BMA255_AXIS_X] / firlen;
					data[BMA255_AXIS_Y] = priv->fir.sum[BMA255_AXIS_Y] / firlen;
					data[BMA255_AXIS_Z] = priv->fir.sum[BMA255_AXIS_Z] / firlen;
					if (atomic_read(&priv->trace) & BMA_TRC_FILTER) {
						GSE_LOG
						    ("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n",
						     idx, priv->fir.raw[idx][BMA255_AXIS_X],
						     priv->fir.raw[idx][BMA255_AXIS_Y],
						     priv->fir.raw[idx][BMA255_AXIS_Z],
						     priv->fir.sum[BMA255_AXIS_X],
						     priv->fir.sum[BMA255_AXIS_Y],
						     priv->fir.sum[BMA255_AXIS_Z],
						     data[BMA255_AXIS_X], data[BMA255_AXIS_Y],
						     data[BMA255_AXIS_Z]);
					}
				}
			}
		}
#endif
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int BMA255_ResetCalibration(struct i2c_client *client)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
#ifndef SW_CALIBRATION
	u8 ofs[4] = { 0, 0, 0, 0 };
#endif
	int err;

#ifdef SW_CALIBRATION
	err = 0;
#else
	err = hwmsen_write_block(client, BMA255_REG_OFSX, ofs, 4);
	if (err)
		GSE_ERR("error: %d\n", err);

#endif

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

/*----------------------------------------------------------------------------*/
static int BMA255_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 2);
	databuf[0] = BMA255_REG_DEVID;

	res = i2c_master_send(client, databuf, 0x1);
	if (res <= 0) {
		res = i2c_master_send(client, databuf, 0x1);
		if (res <= 0)
			goto exit_BMA255_CheckDeviceID;

	}

	udelay(500);

	databuf[0] = 0x0;
	res = i2c_master_recv(client, databuf, 0x01);
	if (res <= 0)
		goto exit_BMA255_CheckDeviceID;


	if (databuf[0] != BMA255_FIXED_DEVID) {
		GSE_ERR("BMA255_CheckDeviceID %d failt!\n ", databuf[0]);
		return BMA255_ERR_IDENTIFICATION;
	} else {
		GSE_LOG("BMA255_CheckDeviceID %d pass!\n ", databuf[0]);
	}

exit_BMA255_CheckDeviceID:
	if (res <= 0)
		return BMA255_ERR_I2C;


	return BMA255_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int BMA255_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;
/* struct bma255_i2c_data *obj = i2c_get_clientdata(client); */

	GSE_FUN();

	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return BMA255_SUCCESS;
	}

	if (enable == TRUE)
		databuf[1] = 0;
	else
		databuf[1] = BMA255_MEASURE_MODE;


	databuf[0] = BMA255_REG_POWER_CTL;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_LOG("set power mode failed!\n");
		return BMA255_ERR_I2C;
	} else {
		GSE_LOG("set power mode ok: 0x%02x\n", databuf[1]);
	}
	/* GSE_LOG("BMA255_SetPowerMode ok!\n"); */

	sensor_power = enable;
	test_status = sensor_power;

	mdelay(10);

	return BMA255_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int BMA255_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2] = { 0 };
	int res = 0;

	databuf[1] = dataformat;
	databuf[0] = BMA255_REG_DATA_FORMAT;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0)
		return BMA255_ERR_I2C;
	else
		GSE_LOG("Set DataFormat: 0x%02x\n", dataformat);

	/* pr_debug("BMA255_SetDataFormat OK!\n"); */

	return BMA255_SetDataResolution(obj);
}

/*----------------------------------------------------------------------------*/
static int BMA255_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	databuf[1] = bwrate;
	databuf[0] = BMA255_REG_BW_RATE;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0)
		return BMA255_ERR_I2C;
	else
		GSE_LOG("Set BWrate: 0x%02x\n", bwrate);


	return BMA255_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int BMA255_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	int res = 0;

	res = hwmsen_write_byte(client, BMA255_INT_REG_1, intenable);
	if (res != BMA255_SUCCESS)
		return res;

	res = hwmsen_write_byte(client, BMA255_INT_REG_2, intenable);
	if (res != BMA255_SUCCESS)
		return res;

	GSE_LOG("BMA255 interrupt was disabled\n");

	/*for disable interrupt function */

	return BMA255_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int bma255_init_client(struct i2c_client *client, int reset_cali)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();

	res = BMA255_CheckDeviceID(client);
	if (res != BMA255_SUCCESS)
		return res;

	pr_debug("BMA255_CheckDeviceID ok\n");

	res = BMA255_SetBWRate(client, BMA255_BW_100HZ);
	if (res != BMA255_SUCCESS)
		return res;

	pr_debug("BMA255_SetBWRate OK!\n");

	res = BMA255_SetDataFormat(client, BMA255_RANGE_4G);
	if (res != BMA255_SUCCESS)
		return res;

	pr_debug("BMA255_SetDataFormat OK!\n");

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = BMA255_SetIntEnable(client, 0x00);
	if (res != BMA255_SUCCESS)
		return res;

	pr_debug("BMA255 disable interrupt function!\n");

	res = BMA255_SetPowerMode(client, false);
	if (res != BMA255_SUCCESS)
		return res;

	pr_debug("BMA255_SetPowerMode OK!\n");

	if (0 != reset_cali) {
		/*reset calibration only in power on */
		res = BMA255_ResetCalibration(client);
		if (res != BMA255_SUCCESS)
			return res;

	}
	pr_debug("bma255_init_client OK!\n");
#ifdef CONFIG_BMA255_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	mdelay(20);

	return BMA255_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int BMA255_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;


	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "BMA255 Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int BMA255_CompassReadData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA255_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_power == FALSE) {
		res = BMA255_SetPowerMode(client, true);
		if (res)
			GSE_ERR("Power on bma255 error %d!\n", res);

	}

	res = BMA255_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	} else {
		/*remap coordinate */
		acc[obj->cvt.map[BMA255_AXIS_X]] =
		    obj->cvt.sign[BMA255_AXIS_X] * obj->data[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] =
		    obj->cvt.sign[BMA255_AXIS_Y] * obj->data[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] =
		    obj->cvt.sign[BMA255_AXIS_Z] * obj->data[BMA255_AXIS_Z];

		sprintf(buf, "%d %d %d", (s16) acc[BMA255_AXIS_X], (s16) acc[BMA255_AXIS_Y],
			(s16) acc[BMA255_AXIS_Z]);
		if (atomic_read(&obj->trace) & BMA_TRC_IOCTL)
			GSE_LOG("gsensor data for compass: %s!\n", buf);

	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int BMA255_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA255_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_power == FALSE) {
		res = BMA255_SetPowerMode(client, true);
		if (res)
			GSE_ERR("Power on bma255 error %d!\n", res);

	}

	res = BMA255_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	} else {
		obj->data[BMA255_AXIS_X] += obj->cali_sw[BMA255_AXIS_X];
		obj->data[BMA255_AXIS_Y] += obj->cali_sw[BMA255_AXIS_Y];
		obj->data[BMA255_AXIS_Z] += obj->cali_sw[BMA255_AXIS_Z];

		/*remap coordinate */
		acc[obj->cvt.map[BMA255_AXIS_X]] =
		    obj->cvt.sign[BMA255_AXIS_X] * obj->data[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] =
		    obj->cvt.sign[BMA255_AXIS_Y] * obj->data[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] =
		    obj->cvt.sign[BMA255_AXIS_Z] * obj->data[BMA255_AXIS_Z];

		/* Out put the mg */
		acc[BMA255_AXIS_X] =
		    acc[BMA255_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA255_AXIS_Y] =
		    acc[BMA255_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA255_AXIS_Z] =
		    acc[BMA255_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		data_count++;
		if (data_count > 100000)
			data_count = 0;


		sprintf(buf, "%04x %04x %04x %04x", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y],
			acc[BMA255_AXIS_Z], data_count);

		/* Add for accel sensor data error debugging : start */
		if ((data_count % 500) == 0)
			GSE_LOG("gsensor data: debug count = %d, x=%d, y=%d, z=%d!\n", data_count,
				acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);
		/* Add for accel sensor data error debugging : end */

		if (atomic_read(&obj->trace) & BMA_TRC_IOCTL)
			GSE_LOG("gsensor data: %s!\n", buf);

	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int BMA255_ReadRawData(struct i2c_client *client, char *buf)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data *)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return -EINVAL;

	res = BMA255_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	} else {
		sprintf(buf, "BMA255_ReadRawData %04x %04x %04x", obj->data[BMA255_AXIS_X],
			obj->data[BMA255_AXIS_Y], obj->data[BMA255_AXIS_Z]);
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int bma255_set_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data[2] = { BMA255_EN_LOW_POWER__REG };

	if ((client == NULL) || (mode >= 3))
		return -1;


	comres = hwmsen_read_block(client, BMA255_EN_LOW_POWER__REG, data + 1, 1);
	switch (mode) {
	case BMA255_MODE_NORMAL:
		data[1] = BMA255_SET_BITSLICE(data[1], BMA255_EN_LOW_POWER, 0);
		data[1] = BMA255_SET_BITSLICE(data[1], BMA255_EN_SUSPEND, 0);
		break;
	case BMA255_MODE_LOWPOWER:
		data[1] = BMA255_SET_BITSLICE(data[1], BMA255_EN_LOW_POWER, 1);
		data[1] = BMA255_SET_BITSLICE(data[1], BMA255_EN_SUSPEND, 0);
		break;
	case BMA255_MODE_SUSPEND:
		data[1] = BMA255_SET_BITSLICE(data[1], BMA255_EN_LOW_POWER, 0);
		data[1] = BMA255_SET_BITSLICE(data[1], BMA255_EN_SUSPEND, 1);
		break;
	default:
		break;
	}

	comres = i2c_master_send(client, data, 2);

	if (comres <= 0)
		return BMA255_ERR_I2C;
	else
		return comres;

}

/*----------------------------------------------------------------------------*/
static int bma255_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;

	if (client == NULL)
		return -1;

	comres = hwmsen_read_block(client, BMA255_EN_LOW_POWER__REG, mode, 1);
	*mode = (*mode) >> 6;

	return comres;
}

/*----------------------------------------------------------------------------*/
static int bma255_set_range(struct i2c_client *client, unsigned char range)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data *)i2c_get_clientdata(client);
	int comres = 0;
	unsigned char data[2] = { BMA255_RANGE_SEL__REG };

	if (client == NULL)
		return -1;

	if (range == obj->range)
		return 0;


	comres = hwmsen_read_block(client, BMA255_RANGE_SEL__REG, data + 1, 1);

	data[1] = BMA255_SET_BITSLICE(data[1], BMA255_RANGE_SEL, range);

	comres = i2c_master_send(client, data, 2);
	mdelay(1);

	if (comres <= 0) {
		return BMA255_ERR_I2C;
	} else {
		obj->range = range;
		return comres;
	}
}

/*----------------------------------------------------------------------------*/
static int bma255_get_range(struct i2c_client *client, unsigned char *range)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL)
		return -1;


	comres = hwmsen_read_block(client, BMA255_RANGE_SEL__REG, &data, 1);
	*range = BMA255_GET_BITSLICE(data, BMA255_RANGE_SEL);

	return comres;
}

/*----------------------------------------------------------------------------*/
static int bma255_set_bandwidth(struct i2c_client *client, unsigned char bandwidth)
{
	int comres = 0;
	unsigned char data[2] = { BMA255_BANDWIDTH__REG };

	if (client == NULL)
		return -1;


	comres = hwmsen_read_block(client, BMA255_BANDWIDTH__REG, data + 1, 1);

	data[1] = BMA255_SET_BITSLICE(data[1], BMA255_BANDWIDTH, bandwidth);

	comres = i2c_master_send(client, data, 2);
	mdelay(1);

	if (comres <= 0)
		return BMA255_ERR_I2C;
	else
		return comres;

}

/*----------------------------------------------------------------------------*/
static int bma255_get_bandwidth(struct i2c_client *client, unsigned char *bandwidth)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL)
		return -1;


	comres = hwmsen_read_block(client, BMA255_BANDWIDTH__REG, &data, 1);
	data = BMA255_GET_BITSLICE(data, BMA255_BANDWIDTH);

	if (data < 0x08) {	/* 7.81Hz */
		*bandwidth = BMA255_BW_7_81HZ;
	} else if (data > 0x0f) {	/* 1000Hz */
		*bandwidth = BMA255_BW_1000HZ;
	} else {
		*bandwidth = data;
	}
	return comres;
}

/*----------------------------------------------------------------------------*/
#ifdef BMA255_ACCEL_CALIBRATION
static int bma255_set_offset_target(struct i2c_client *client, unsigned char channel,
				    unsigned char offset)
{
	unsigned char data = 0;
	int comres = 0;

	switch (channel) {
	case BMA255_CUT_OFF:
		comres = bma255_smbus_read_byte(client, BMA255_COMP_CUTOFF__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_COMP_CUTOFF, offset);
		comres = bma255_smbus_write_byte(client, BMA255_COMP_CUTOFF__REG, &data);
		break;

	case BMA255_OFFSET_TRIGGER_X:
		comres = bma255_smbus_read_byte(client, BMA255_COMP_TARGET_OFFSET_X__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_COMP_TARGET_OFFSET_X, offset);
		comres = bma255_smbus_write_byte(client, BMA255_COMP_TARGET_OFFSET_X__REG, &data);
		break;

	case BMA255_OFFSET_TRIGGER_Y:
		comres = bma255_smbus_read_byte(client, BMA255_COMP_TARGET_OFFSET_Y__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_COMP_TARGET_OFFSET_Y, offset);
		comres = bma255_smbus_write_byte(client, BMA255_COMP_TARGET_OFFSET_Y__REG, &data);
		break;

	case BMA255_OFFSET_TRIGGER_Z:
		comres = bma255_smbus_read_byte(client, BMA255_COMP_TARGET_OFFSET_Z__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_COMP_TARGET_OFFSET_Z, offset);
		comres = bma255_smbus_write_byte(client, BMA255_COMP_TARGET_OFFSET_Z__REG, &data);
		break;

	default:
		comres = -1;
		break;
	}

	return comres;
}

static int bma255_get_cal_ready(struct i2c_client *client, unsigned char *calrdy)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma255_smbus_read_byte(client, BMA255_FAST_CAL_RDY_S__REG, &data);
	data = BMA255_GET_BITSLICE(data, BMA255_FAST_CAL_RDY_S);
	*calrdy = data;

	return comres;
}

static int bma255_set_cal_trigger(struct i2c_client *client, unsigned char caltrigger)
{
	int comres = 0;
	unsigned char data = 0;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if (atomic_read(&bma255->fast_calib_rslt) != 0) {
		atomic_set(&bma255->fast_calib_rslt, 0);
		GSE_LOG(KERN_INFO "[set] bma2X2->fast_calib_rslt:%d\n",
			atomic_read(&bma255->fast_calib_rslt));
	}

	comres = bma255_smbus_read_byte(client, BMA255_CAL_TRIGGER__REG, &data);
	data = BMA255_SET_BITSLICE(data, BMA255_CAL_TRIGGER, caltrigger);
	comres = bma255_smbus_write_byte(client, BMA255_CAL_TRIGGER__REG, &data);

	return comres;
}

static int bma255_set_offset_x(struct i2c_client *client, unsigned char offsetfilt)
{
	int comres = 0;
	unsigned char data;

	data = offsetfilt;
	comres = bma255_smbus_write_byte(client, BMA255_OFFSET_X_AXIS_REG, &data);

	return comres;
}

static int bma255_get_offset_x(struct i2c_client *client, unsigned char *offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma255_smbus_read_byte(client, BMA255_OFFSET_X_AXIS_REG, &data);
	*offsetfilt = data;

	return comres;
}

static int bma255_set_offset_y(struct i2c_client *client, unsigned char offsetfilt)
{
	int comres = 0;
	unsigned char data;

	data = offsetfilt;
	comres = bma255_smbus_write_byte(client, BMA255_OFFSET_Y_AXIS_REG, &data);

	return comres;
}

static int bma255_get_offset_y(struct i2c_client *client, unsigned char *offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma255_smbus_read_byte(client, BMA255_OFFSET_Y_AXIS_REG, &data);
	*offsetfilt = data;

	return comres;
}

static int bma255_set_offset_z(struct i2c_client *client, unsigned char offsetfilt)
{
	int comres = 0;
	unsigned char data;

	data = offsetfilt;
	comres = bma255_smbus_write_byte(client, BMA255_OFFSET_Z_AXIS_REG, &data);

	return comres;
}

static int bma255_get_offset_z(struct i2c_client *client, unsigned char *offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma255_smbus_read_byte(client, BMA255_OFFSET_Z_AXIS_REG, &data);
	*offsetfilt = data;

	return comres;
}

static int bma255_read_accel_xyz(struct i2c_client *client, struct bma255acc *acc)
{
	int comres = 0;
	unsigned char data[6];

	comres = bma255_smbus_read_byte_block(client, BMA255_ACC_X_LSB__REG, data, 6);

	acc->x = BMA255_GET_BITSLICE(data[0], BMA255_ACC_X_LSB) 
	    | (BMA255_GET_BITSLICE(data[1],		 BMA255_ACC_X_MSB) <<     (BMA255_ACC_X_LSB__LEN));
	acc->x = acc->x << (sizeof(short) * 8 - (BMA255_ACC_X_LSB__LEN + BMA255_ACC_X_MSB__LEN));
	acc->x = acc->x >> (sizeof(short) * 8 - (BMA255_ACC_X_LSB__LEN + BMA255_ACC_X_MSB__LEN));

	acc->y = BMA255_GET_BITSLICE(data[2], BMA255_ACC_Y_LSB) 
        | (BMA255_GET_BITSLICE(data[3], 	 BMA255_ACC_Y_MSB) <<     (BMA255_ACC_Y_LSB__LEN));
	acc->y = acc->y << (sizeof(short) * 8 - (BMA255_ACC_Y_LSB__LEN + BMA255_ACC_Y_MSB__LEN));
	acc->y = acc->y >> (sizeof(short) * 8 - (BMA255_ACC_Y_LSB__LEN + BMA255_ACC_Y_MSB__LEN));

	acc->z = BMA255_GET_BITSLICE(data[4], BMA255_ACC_Z_LSB) 
        | (BMA255_GET_BITSLICE(data[5],		 BMA255_ACC_Z_MSB) <<     (BMA255_ACC_Z_LSB__LEN));
	acc->z = acc->z << (sizeof(short) * 8 - (BMA255_ACC_Z_LSB__LEN + BMA255_ACC_Z_MSB__LEN));
	acc->z = acc->z >> (sizeof(short) * 8 - (BMA255_ACC_Z_LSB__LEN + BMA255_ACC_Z_MSB__LEN));

	return comres;
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	unsigned char offset_x, offset_y, offset_z;

	if (bma255_get_offset_x(bma255->client, &offset_x) < 0)
		return -EINVAL;
	if (bma255_get_offset_y(bma255->client, &offset_y) < 0)
		return -EINVAL;
	if (bma255_get_offset_z(bma255->client, &offset_z) < 0)
		return -EINVAL;

	GSE_LOG("offset_x: %d, offset_y: %d, offset_z: %d\n", offset_x, offset_y, offset_z);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", (unsigned int)offset_x,
			(unsigned int)offset_y, (unsigned int)offset_z);
}

#if defined(CALIBRATION_TO_FILE)
static int bma255_calibration_save(int *cal)
{
	int fd;
	int i;
	int res;
	char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
	mm_segment_t old_fs = get_fs();
	char temp_str[5];

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY | O_CREAT | S_IROTH, 0666);
	if (fd < 0) {
		GSE_LOG("[%s] File Open Error !!!(%d)\n", __func__, fd);
		sys_close(fd);
		return -EINVAL;
	}
	for (i = 0; i < 6; i++) {
		memset(temp_str, 0x00, sizeof(temp_str));
		sprintf(temp_str, "%d", cal[i]);
		res = sys_write(fd, temp_str, sizeof(temp_str));

		if (res < 0) {
			GSE_LOG("[%s] Write Error !!!\n", __func__);
			sys_close(fd);
			return -EINVAL;
		}
	}
	sys_fsync(fd);
	sys_close(fd);

	sys_chmod(fname, 0664);
	set_fs(old_fs);
	GSE_LOG("bma255_calibration_save Done.\n");

	return 0;
}

static int bma255_calibration_read(int *cal_read)
{
	int fd;
	int i;
	int res;
	char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
	mm_segment_t old_fs = get_fs();
	char temp_str[5];
	int ret = 0;

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_RDONLY, 0);
	if (fd < 0) {
		GSE_LOG("[%s] File Open Error !!!\n", __func__);
		sys_close(fd);
		return -EINVAL;
	}
	for (i = 0; i < 6; i++) {
		memset(temp_str, 0x00, sizeof(temp_str));
		res = sys_read(fd, temp_str, sizeof(temp_str));
		if (res < 0) {
			GSE_LOG("[%s] Read Error !!!\n", __func__);
			sys_close(fd);
			return -EINVAL;
		}
		ret = sscanf(temp_str, "%d", &cal_read[i]);
		if (ret)
			GSE_LOG("bma255_calibration_read : cal_read[%d]=%d\n", i, cal_read[i]);
	}
	sys_close(fd);
	set_fs(old_fs);
	GSE_LOG("bma255_calibration_read Done.\n");

	return 0;
}

/* Make gsensor cal file to save calibration data */
/* 1. If file exist, do nothing                  */
/* 2. If file does not exist, read cal data from misc2 (for L-OS upgrade model) */
static int make_cal_data_file(void)
{
	int fd;
	int i;
	int res;
	char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
	mm_segment_t old_fs = get_fs();
	char temp_str[5];
	int cal_misc[6] = { 0, };

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_RDONLY, 0);	/* Cal file exist check */

	if (fd == -2)
		GSE_LOG("[%s] Open Cal File Error. Need to make file !!!(%d)\n", __func__, fd);

	if (fd < 0) {
		fd = sys_open(fname, O_WRONLY | O_CREAT | S_IROTH, 0666);
		if (fd < 0) {
			GSE_LOG("[%s] Open or Make Cal File Error !!!(%d)\n", __func__, fd);
			sys_close(fd);
			return -EINVAL;
		}
#if 0
		if (LGE_FacReadAccelerometerCalibration((unsigned int *)cal_misc) == TRUE) {
			GSE_LOG("Read Cal from misc Old x: %d, y: %d, z: %d\n", cal_misc[3],
				cal_misc[4], cal_misc[5]);
			GSE_LOG("Read Cal from misc Now x: %d, y: %d, z: %d\n", cal_misc[0],
				cal_misc[1], cal_misc[2]);
		} else {
			GSE_LOG("Read Cal from misc Error !!!\n");
		}
#endif

		/* set default cal */
		if ((cal_misc[0] == 0) && (cal_misc[1] == 0) && (cal_misc[2] == 0)) {
#if defined(TARGET_Y90)
			cal_misc[0] = 3;
			cal_misc[1] = 6;
			cal_misc[2] = 5;
#endif
			GSE_LOG("Default Cal is set : %d, %d, %d\n", cal_misc[0], cal_misc[1],
				cal_misc[2]);
		}
		/* set default cal */

		for (i = 0; i < 6; i++) {
			memset(temp_str, 0x00, sizeof(temp_str));
			sprintf(temp_str, "%d", cal_misc[i]);
			res = sys_write(fd, temp_str, sizeof(temp_str));

			if (res < 0) {
				GSE_LOG("[%s] Write Cal Error !!!\n", __func__);
				sys_close(fd);
				return -EINVAL;
			}
		}
		GSE_LOG("make_cal_data_file Done.\n");
	} else
		GSE_LOG("Sensor Cal File exist.\n");

	sys_fsync(fd);
	sys_close(fd);

	sys_chmod(fname, 0664);
	set_fs(old_fs);
	return 0;
}
#endif

#if 1				/* motion sensor cal backup */
static ssize_t show_cali_backup_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	unsigned char offset_x, offset_y, offset_z;
	int cal_backup[6] = { 0, };
/* int i; */

#if defined(CALIBRATION_TO_FILE)
	if (bma255_calibration_read(cal_backup) == 0)
		GSE_LOG("previous offset_x: %d,  offset_y: %d,  offset_z: %d\n", cal_backup[3],
			cal_backup[4], cal_backup[5]);
	else
		GSE_LOG("Fail to read previous gsensor cal value from Cal File\n");
#else
	if (LGE_FacReadAccelerometerCalibration((unsigned int *)cal_backup) == TRUE)
		GSE_LOG
		    ("Read from LGPServer gsensor previous offset_x: %d,  offset_y: %d,  offset_z: %d\n",
		     cal_backup[3], cal_backup[4], cal_backup[5]);
	else
		GSE_LOG("Fail to read previous gsensor cal value from LGPserver\n");
#endif

	if (bma255_get_offset_x(bma255->client, &offset_x) < 0)
		return -EINVAL;
	if (bma255_get_offset_y(bma255->client, &offset_y) < 0)
		return -EINVAL;
	if (bma255_get_offset_z(bma255->client, &offset_z) < 0)
		return -EINVAL;

	GSE_LOG("Current offset_x: %d, offset_y: %d, offset_z: %d\n", offset_x, offset_y, offset_z);

	return snprintf(buf, PAGE_SIZE, "Old %d %d %d   Now %d %d %d\n",
			(unsigned int)cal_backup[3], (unsigned int)cal_backup[4],
			(unsigned int)cal_backup[5], (unsigned int)offset_x, (unsigned int)offset_y,
			(unsigned int)offset_z);
}
#endif

static ssize_t store_cali_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	int err = 0;
	int offset_x, offset_y, offset_z;
	unsigned char offsets[3];
/* int dat[BMA255_AXES_NUM]; */
	int ret = 0;

	ret = strncmp(buf, "rst", 3);
	if (!ret) {
		err = BMA255_ResetCalibration(client);
		if (err)
			GSE_ERR("reset offset err = %d\n", err);

	} else if (3 == sscanf(buf, "%d %d %d", &offset_x, &offset_y, &offset_z)) {
		GSE_LOG("store_cali_value: x=%d, y=%d, z=%d\n", offset_x, offset_y, offset_z);
		offsets[0] = (unsigned char)offset_x;
		offsets[1] = (unsigned char)offset_y;
		offsets[2] = (unsigned char)offset_z;
		if (bma255_set_offset_x(bma255->client, (unsigned char)offsets[0]) < 0)
			return -EINVAL;
		if (bma255_set_offset_y(bma255->client, (unsigned char)offsets[1]) < 0)
			return -EINVAL;
		if (bma255_set_offset_z(bma255->client, (unsigned char)offsets[2]) < 0)
			return -EINVAL;
		GSE_LOG("store_cali_value success\n");
	} else {
		GSE_ERR("invalid format\n");
	}

	return count;
}

static ssize_t bma255_fast_calibration_x_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_x_rslt));
}

static ssize_t bma255_fast_calibration_x_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	test_status = 4;	/* calibration status */

	error = kstrtoul(buf, 10, &data);

	if (error)
		return error;
	atomic_set(&bma255->fast_calib_x_rslt, 0);

	if (bma255_do_calibration() != BMA255_SUCCESS) {
		atomic_set(&bma255->fast_calib_x_rslt, 0);
		return -EINVAL;
	}

	if (bma255_set_offset_target(bma255->client, 1, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma255_set_cal_trigger(bma255->client, 1) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(KERN_INFO "x wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_LOG(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);

	atomic_set(&bma255->fast_calib_x_rslt, 1);
	GSE_LOG(KERN_INFO "x axis fast calibration finished\n");

	return count;
}

static ssize_t bma255_fast_calibration_y_show(struct device_driver *ddri, char *buf)
{

	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_y_rslt));
}

static ssize_t bma255_fast_calibration_y_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);

	if (error)
		return error;
	atomic_set(&bma255->fast_calib_y_rslt, 0);

	if (bma255_set_offset_target(bma255->client, 2, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma255_set_cal_trigger(bma255->client, 2) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(KERN_INFO "y wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_LOG(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);

	atomic_set(&bma255->fast_calib_y_rslt, 1);
	GSE_LOG(KERN_INFO "y axis fast calibration finished\n");

	return count;
}

static ssize_t bma255_fast_calibration_z_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_z_rslt));
}

static ssize_t bma255_fast_calibration_z_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	atomic_set(&bma255->fast_calib_z_rslt, 0);

	if (bma255_set_offset_target(bma255->client, 3, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma255_set_cal_trigger(bma255->client, 3) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(KERN_INFO " z wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_LOG(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);

	atomic_set(&bma255->fast_calib_z_rslt, 1);
	GSE_LOG(KERN_INFO "z axis fast calibration finished\n");

	test_status = sensor_power;

	return count;
}

/* LGE_BSP_COMMON LGE_CHANGE_S 140228 : Calibration for user Apps */
static int bma255_runCalibration(void)
{
/* signed char tmp; */
/* unsigned char timeout = 0; */
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	unsigned char backup_offset_x, backup_offset_y, backup_offset_z;
	int res = 0;
	int res2 = 0;
#if 0				/* motion sensor cal backup */
	int cali[3];
#else
	int cali[6];
#endif

	GSE_FUN();
	if (bma255_get_offset_x(bma255->client, &backup_offset_x) < 0)
		return FALSE;
	if (bma255_get_offset_y(bma255->client, &backup_offset_y) < 0)
		return FALSE;
	if (bma255_get_offset_z(bma255->client, &backup_offset_z) < 0)
		return FALSE;

	GSE_LOG("backup_offset_x: %d, backup_offset_y: %d, backup_offset_z: %d\n", backup_offset_x,
		backup_offset_y, backup_offset_z);

	if (sensor_power == FALSE) {
		res = BMA255_SetPowerMode(client, true);
		if (res) {
			GSE_ERR("Power on bma255 error %d!\n", res);
			return FALSE;
		}
	}

	res2 = BMA255_SetDataFormat(client, BMA255_RANGE_2G);
	if (res2 < 0) {		/* change range before calibration */
		GSE_ERR("SetDataFormat 2G error");
	}
	res = bma255_do_calibration();

	res2 = BMA255_SetDataFormat(client, BMA255_RANGE_4G);
	if (res2 < 0) {		/* change range before calibration */
		GSE_ERR("SetDataFormat 4G error");
	}

	if (res != BMA255_SUCCESS) {
		GSE_LOG("Fail flatDetection, backup offset\n");
		if (bma255_set_offset_x(bma255->client, (unsigned char)backup_offset_x) < 0)
			return -EINVAL;
		if (bma255_set_offset_y(bma255->client, (unsigned char)backup_offset_y) < 0)
			return -EINVAL;
		if (bma255_set_offset_z(bma255->client, (unsigned char)backup_offset_z) < 0)
			return -EINVAL;
		GSE_LOG("Recovery backup cal value: x=%d, y=%d, z=%d\n", backup_offset_x,
			backup_offset_y, backup_offset_z);

		if (res == BMA255_ERR_SETUP_FAILURE)
			return BMA255_ERR_SETUP_FAILURE;
		else
			return BMA255_ERR_STATUS;
	} else {
#if 1				/* motion sensor cal backup */
		cali[3] = (int)backup_offset_x;
		cali[4] = (int)backup_offset_y;
		cali[5] = (int)backup_offset_z;
#endif

		bma255_get_offset_x(bma255->client, &backup_offset_x);
		bma255_get_offset_y(bma255->client, &backup_offset_y);
		bma255_get_offset_z(bma255->client, &backup_offset_z);
		GSE_LOG("new_offset_x: %d, new_offset_y: %d, new_offset_z: %d\n",
			(int)backup_offset_x, (int)backup_offset_y, (int)backup_offset_z);
		cali[0] = (int)backup_offset_x;
		cali[1] = (int)backup_offset_y;
		cali[2] = (int)backup_offset_z;
#if defined(CALIBRATION_TO_FILE)
		bma255_calibration_save(cali);
#else
		if (LGE_FacWriteAccelerometerCalibration((unsigned int *)cali) == TRUE) {
			atomic_set(&bma255->fast_calib_rslt, 1);
			GSE_LOG("Calibration factory write END\n");
		}
#endif
	}

	return BMA255_SUCCESS;
}

/* LGE_BSP_COMMON LGE_CHANGE_E 140228 : Calibration for user Apps */

static int bma255_do_calibration(void)
{
	signed char tmp;
	unsigned char timeout = 0;
	unsigned int timeout_shaking = 0;
	int sum[3] = { 0, };
	int err = 0;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	struct bma255acc acc_cal;
	struct bma255acc acc_cal_pre;

	GSE_FUN();
	test_status = 4;	/* calibration status */
	/* set axis off set to zero */
	if (bma255_set_offset_x(bma255->client, (unsigned char)0) < 0)
		return -EINVAL;
	if (bma255_set_offset_y(bma255->client, (unsigned char)0) < 0)
		return -EINVAL;
	if (bma255_set_offset_z(bma255->client, (unsigned char)0) < 0)
		return -EINVAL;
	/* set axis off set to zero */

	mdelay(20);

	bma255_read_accel_xyz(bma255->client, &acc_cal_pre);
	do {
		mdelay(20);
		bma255_read_accel_xyz(bma255->client, &acc_cal);

		GSE_LOG(KERN_INFO "===============moved x=============== timeout = %d\n",
			timeout_shaking);
		GSE_LOG(KERN_INFO "(%d, %d, %d) (%d, %d, %d)\n", acc_cal_pre.x, acc_cal_pre.y,
			acc_cal_pre.z, acc_cal.x, acc_cal.y, acc_cal.z);

		if ((abs(acc_cal.x - acc_cal_pre.x) > BMA255_SHAKING_DETECT_THRESHOLD)
		    || (abs((acc_cal.y - acc_cal_pre.y)) > BMA255_SHAKING_DETECT_THRESHOLD)
		    || (abs((acc_cal.z - acc_cal_pre.z)) > BMA255_SHAKING_DETECT_THRESHOLD)) {
			atomic_set(&bma255->fast_calib_rslt, 0);
			GSE_LOG(KERN_INFO "===============shaking x===============\n");
			return -EINVAL;
		} else {
			/* Calibration zero-g offset check */
			sum[BMA255_AXIS_X] += acc_cal.x;
			sum[BMA255_AXIS_Y] += acc_cal.y;
			sum[BMA255_AXIS_Z] += acc_cal.z;
			/* Calibration zero-g offset check */

			acc_cal_pre.x = acc_cal.x;
			acc_cal_pre.y = acc_cal.y;
			acc_cal_pre.z = acc_cal.z;
		}
		timeout_shaking++;
		GSE_LOG(KERN_INFO "===============timeout_shaking: %d===============\n",
			timeout_shaking);
	} while (timeout_shaking < 10);

	GSE_LOG(KERN_INFO "===============complete shaking check===============\n");
#if 1				/* Calibration zero-g offset check */
	/* check zero-g offset */
	if ((abs(sum[BMA255_AXIS_X] / CALIBRATION_DATA_AMOUNT) > TESTLIMIT_XY) ||
	    (abs(sum[BMA255_AXIS_Y] / CALIBRATION_DATA_AMOUNT) > TESTLIMIT_XY) ||
	    ((abs(sum[BMA255_AXIS_Z] / CALIBRATION_DATA_AMOUNT) > TESTLIMIT_Z_USL_LSB)
	     || (abs(sum[BMA255_AXIS_Z] / CALIBRATION_DATA_AMOUNT) < TESTLIMIT_Z_LSL_LSB))) {
		GSE_LOG("Calibration zero-g offset check failed (%d, %d, %d)\n",
			sum[BMA255_AXIS_X] / CALIBRATION_DATA_AMOUNT,
			sum[BMA255_AXIS_Y] / CALIBRATION_DATA_AMOUNT,
			sum[BMA255_AXIS_Z] / CALIBRATION_DATA_AMOUNT);
		atomic_set(&bma255->fast_calib_rslt, 0);
		return BMA255_ERR_SETUP_FAILURE;
	}
#endif				/* Calibration zero-g offset check */

	GSE_LOG(KERN_INFO "===============complete zero-g check===============\n");

	atomic_set(&bma255->fast_calib_x_rslt, 0);
	if (bma255_set_offset_target(bma255->client, 1, (unsigned char)0) < 0)
		return -EINVAL;
	if (bma255_set_cal_trigger(bma255->client, 1) < 0)
		return -EINVAL;
	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(KERN_INFO "x wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_LOG(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);
	atomic_set(&bma255->fast_calib_x_rslt, 1);
	GSE_LOG(KERN_INFO "===============x axis fast calibration finished\n");

	atomic_set(&bma255->fast_calib_y_rslt, 0);
	if (bma255_set_offset_target(bma255->client, 2, (unsigned char)0) < 0)
		return -EINVAL;
	if (bma255_set_cal_trigger(bma255->client, 2) < 0)
		return -EINVAL;
	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(KERN_INFO "y wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_LOG(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);
	atomic_set(&bma255->fast_calib_y_rslt, 1);
	GSE_LOG(KERN_INFO "===============y axis fast calibration finished\n");

	atomic_set(&bma255->fast_calib_z_rslt, 0);
	if (bma255_set_offset_target(bma255->client, 3, (unsigned char)2) < 0)
		return -EINVAL;
	if (bma255_set_cal_trigger(bma255->client, 3) < 0)
		return -EINVAL;
	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(KERN_INFO " z wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_LOG(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);
	atomic_set(&bma255->fast_calib_z_rslt, 1);
	GSE_LOG(KERN_INFO "===============z axis fast calibration finished\n");


	test_status = sensor_power;

	return err;
}

#endif

#if 0
static int bma255_get_selftest(struct i2c_client *client)
{
	int value = 0;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);

	return value;
}
#endif

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE] = { 0, };

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	BMA255_ReadChipInfo(client, strbuf, BMA255_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE] = { 0, };

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	bma255_init_client(client, 1);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
#endif

/*----------------------------------------------------------------------------*/
/* g sensor opmode for compass tilt compensation
 */
static ssize_t show_cpsopmode_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;
	int ret = 0;

	ret = bma255_get_mode(bma255_i2c_client, &data);
	if (ret < 0)
		return sprintf(buf, "Read error\n");
	else
		return sprintf(buf, "%d\n", data);

}

/*----------------------------------------------------------------------------*/
/* g sensor opmode for compass tilt compensation
 */
static ssize_t store_cpsopmode_value(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (data == BMA255_MODE_NORMAL)
		BMA255_SetPowerMode(bma255_i2c_client, true);
	else if (data == BMA255_MODE_SUSPEND)
		BMA255_SetPowerMode(bma255_i2c_client, false);
	else if (bma255_set_mode(bma255_i2c_client, (unsigned char)data) < 0)
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);

	return count;
}

/*----------------------------------------------------------------------------*/
/* g sensor range for compass tilt compensation
 */
static ssize_t show_cpsrange_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_range(bma255_i2c_client, &data) < 0)
		return sprintf(buf, "Read error\n");
	else
		return sprintf(buf, "%d\n", data);

}

/*----------------------------------------------------------------------------*/
/* g sensor range for compass tilt compensation
 */
static ssize_t store_cpsrange_value(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;
	int ret;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	ret = bma255_set_range(bma255_i2c_client, (unsigned char)data);
	    if (ret < 0)
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);

	return count;
}

/*----------------------------------------------------------------------------*/
/* g sensor bandwidth for compass tilt compensation
 */
static ssize_t show_cpsbandwidth_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;
	int ret;

	ret = bma255_get_bandwidth(bma255_i2c_client, &data);
	if (ret < 0)
		return sprintf(buf, "Read error\n");
	else
		return sprintf(buf, "%d\n", data);

}

/*----------------------------------------------------------------------------*/
/* g sensor bandwidth for compass tilt compensation
 */
static ssize_t store_cpsbandwidth_value(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;
	int ret;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	ret = bma255_set_bandwidth(bma255_i2c_client, (unsigned char)data);
	if (ret < 0)
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);


	return count;
}

/* jwseo_test */
#if 0
static ssize_t show_limit_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", (int)TESTLIMIT_XY);
}

/*----------------------------------------------------------------------------*/
/* g sensor range for compass tilt compensation
 */
static ssize_t store_limit_value(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	TESTLIMIT_XY = (int)data;

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
/* g sensor data for compass tilt compensation
 */
static ssize_t show_cpsdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE] = { 0, };

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA255_CompassReadData(client, strbuf, BMA255_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE] = { 0, };

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA255_ReadSensorData(client, strbuf, BMA255_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE] = { 0, };

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA255_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_BMA255_LOWPASS
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);

	if (atomic_read(&obj->firlen)) {
		int idx, len = atomic_read(&obj->firlen);

		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for (idx = 0; idx < len; idx++) {
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][BMA255_AXIS_X],
				obj->fir.raw[idx][BMA255_AXIS_Y], obj->fir.raw[idx][BMA255_AXIS_Z]);
		}

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[BMA255_AXIS_X],
			obj->fir.sum[BMA255_AXIS_Y], obj->fir.sum[BMA255_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[BMA255_AXIS_X] / len,
			obj->fir.sum[BMA255_AXIS_Y] / len, obj->fir.sum[BMA255_AXIS_Z] / len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, char *buf, size_t count)
{
#ifdef CONFIG_BMA255_LOWPASS
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if (1 != sscanf(buf, "%d", &firlen)) {
		GSE_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		GSE_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&obj->firlen, firlen);
		if (NULL == firlen) {
			atomic_set(&obj->fir_en, 0);
		} else {
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bma255_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);


	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct bma255_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
				obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id,
				obj->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	if (sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_teststatus_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;

	if (client == NULL) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", test_status);
}

/*----------------------------------------------------------------------------*/
static ssize_t bma255_bandwidth_show(struct device_driver *ddri, char *buf)
{
	unsigned char data;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	int ret;

	ret = bma255_get_bandwidth(bma255->client, &data);
	if (ret < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

/*----------------------------------------------------------------------------*/
static ssize_t bma255_bandwidth_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	int ret;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	ret = bma255_set_bandwidth(bma255->client, (unsigned char)data);
	if (ret < 0)
		return -EINVAL;

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t bma255_eeprom_writing_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_rslt));
}

static ssize_t bma255_eeprom_writing_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned char offset_x, offset_y, offset_z;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if (bma255_get_offset_x(bma255->client, &offset_x) < 0)
		return -EINVAL;
	if (bma255_get_offset_y(bma255->client, &offset_y) < 0)
		return -EINVAL;
	if (bma255_get_offset_z(bma255->client, &offset_z) < 0)
		return -EINVAL;

	atomic_set(&bma255->fast_calib_rslt, 1);

	return count;
}

static int bma255_soft_reset(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data = BMA255_EN_SOFT_RESET_VALUE;

	comres = bma255_smbus_write_byte(client, BMA255_EN_SOFT_RESET__REG, &data);

	return comres;
}

static ssize_t bma255_softreset_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if (bma255_soft_reset(bma255->client) < 0)
		return -EINVAL;

	return count;
}

/*----------------------------------------------------------------------------*/
/* LGE_BSP_COMMON LGE_CHANGE_S 140228 : Calibration for user Apps */
static int selfCalibration = 1;
static ssize_t bma255_runCalibration_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int res = 0;

	res = bma255_runCalibration();
	if (res == BMA255_SUCCESS)
		selfCalibration = BMA255_SUCCESS;
	else if (res == BMA255_ERR_SETUP_FAILURE)
		selfCalibration = 2;	/* cal fail(flat) */
	else
		selfCalibration = 1;	/* cal fail */

	return count;
}

static ssize_t bma255_runCalibration_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", selfCalibration);
}

/* LGE_BSP_COMMON LGE_CHANGE_E 140228 : Calibration for user Apps */

/*----------------------------------------------------------------------------*/
/* for self test */
static int bma255_set_selftest_st(struct i2c_client *client, unsigned char selftest)
{
	int comres = 0;
	unsigned char data;

	comres = bma255_smbus_read_byte_block(client, BMA255_EN_SELF_TEST__REG, &data, 1);
	data = BMA255_SET_BITSLICE(data, BMA255_EN_SELF_TEST, selftest);
	comres = bma255_smbus_write_byte(client, BMA255_EN_SELF_TEST__REG, &data);
	/* comres = i2c_master_send(client, &data, 1); */
	GSE_LOG("selftest_st comres : %d\n", comres);
	return comres;
}

static int bma255_set_selftest_stn(struct i2c_client *client, unsigned char stn)
{
	int comres = 0;
	unsigned char data;

	comres = bma255_smbus_read_byte_block(client, BMA255_NEG_SELF_TEST__REG, &data, 1);
	data = BMA255_SET_BITSLICE(data, BMA255_NEG_SELF_TEST, stn);
	comres = bma255_smbus_write_byte(client, BMA255_NEG_SELF_TEST__REG, &data);
	/* comres = i2c_master_send(client, &data, 1); */
	GSE_LOG("selftest_stN comres : %d\n", comres);
	return comres;
}

/*
Read:
0 ------ success
1 ------ x  axis failed
2 ------ y  axis failed
3 ------ x and y axis failed
4 ------ z  axis failed
5 ------ x and z axis failed
6 ------ y and z axis failed
7 ------ x, y and z axis failed
*/
static ssize_t bma255_SelfTest_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data = 0;
	int error = -1;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	enum BMA_RANGE_ENUM range;
	bool power_mode;
	u8 bandwidth = 0;
	u8 value = 0;
	u8 amp = 0;
	s16 value1[BMA255_AXES_NUM];
	s16 value2[BMA255_AXES_NUM];
	s16 diff = 0;
	u8 result = 0;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	GSE_LOG("Self test CMD value : %d\n", (int)data);

	if (data == 1) {	/* self test start command */
		/*backup settings */
		range = obj->range;
		power_mode = sensor_power;
		bandwidth = obj->bandwidth;
		/*Step 1:Soft Reset */
		bma255_soft_reset(obj->client);
		/*Step 2:Clear Selftest Register */
		{
			value = 0;
			bma255_smbus_write_byte(obj->client, BMA255_SELF_TEST_REG, &value);
		}

		/*Step 3:Set to +/-4G Range */
		BMA255_SetDataFormat(obj->client, BMA255_RANGE_4G);
		/* bma255_set_range(obj->client, BMA_RANGE_4G); */
		/*Step 4:Set Amplitude of Deflection */
		{
			value = 0;
			amp = 1;
			bma255_smbus_read_byte(obj->client, BMA255_SELF_TEST_AMP__REG, &value);
			value = BMA255_SET_BITSLICE(value, BMA255_SELF_TEST_AMP, amp);
			bma255_smbus_write_byte(obj->client, BMA255_SELF_TEST_AMP__REG, &value);
		}

		/*Step 5:X-axis Selftest */
		bma255_set_selftest_st(obj->client, 1);	/*1 for x-axis */
		bma255_set_selftest_stn(obj->client, 0);	/*positive direction */
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if (error) {
			GSE_ERR("I2C error: ret value=%d", error);
			return -EIO;
		} else {
			value1[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value1[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value1[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		bma255_set_selftest_stn(obj->client, 1);	/*negative direction */
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if (error) {
			GSE_ERR("I2C error: ret value=%d", error);
			return -EIO;
		} else {
			value2[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value2[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value2[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		diff = value1[BMA255_AXIS_X] - value2[BMA255_AXIS_X];

		GSE_LOG("diff x is %d,value1 is %d, value2 is %d\n", diff,
			value1[BMA255_AXIS_X], value2[BMA255_AXIS_X]);

		if (abs(diff) < 204)
			result |= 1;

		/*Step 6:Y-axis Selftest */
		bma255_set_selftest_st(obj->client, 2);	/*2 for y-axis */
		bma255_set_selftest_stn(obj->client, 0);	/*positive direction */
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if (error) {
			GSE_ERR("I2C error: ret value=%d", error);
			return -EIO;
		} else {
			value1[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value1[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value1[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		bma255_set_selftest_stn(obj->client, 1);	/*negative direction */
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if (error) {
			GSE_ERR("I2C error: ret value=%d", error);
			return -EIO;
		} else {
			value2[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value2[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value2[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		diff = value1[BMA255_AXIS_Y] - value2[BMA255_AXIS_Y];
		GSE_LOG("diff y is %d,value1 is %d, value2 is %d\n", diff,
			value1[BMA255_AXIS_Y], value2[BMA255_AXIS_Y]);

		if (abs(diff) < 204)
			result |= 2;

		/*Step 7:Z-axis Selftest */
		bma255_set_selftest_st(obj->client, 3);	/*3 for z-axis */
		bma255_set_selftest_stn(obj->client, 0);	/*positive direction */
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if (error) {
			GSE_ERR("I2C error: ret value=%d", error);
			return -EIO;
		} else {
			value1[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value1[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value1[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		bma255_set_selftest_stn(obj->client, 1);	/*negative direction */
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if (error) {
			GSE_ERR("I2C error: ret value=%d", error);
			return -EIO;
		} else {
			value2[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value2[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value2[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		diff = value1[BMA255_AXIS_Z] - value2[BMA255_AXIS_Z];

		GSE_LOG("diff z is %d,value1 is %d, value2 is %d\n", diff,
			value1[BMA255_AXIS_Z], value2[BMA255_AXIS_Z]);

		if (abs(diff) < 204)
			result |= 4;

		/*Step 8:Soft Reset */
		bma255_soft_reset(obj->client);
		/*Sync sw settings to hw settings */
		obj->range = range;	/* fix for 4G */
		obj->bandwidth = bandwidth;

		atomic_set(&obj->selftest, result);

		/*restore settings */
		BMA255_SetDataFormat(obj->client, range);
		/* bma255_set_range(obj->client, range); */
		bma255_set_bandwidth(obj->client, bandwidth);
		BMA255_SetPowerMode(obj->client, power_mode);
		GSE_LOG("self test finished. result : %d\n", result);
	} else {		/* wrong input */

		GSE_LOG("SelfTest command FAIL\n");
		return -EINVAL;
	}
	return count;
}

static ssize_t bma255_SelfTest_show(struct device_driver *ddri, char *buf)
{
	int selftest_rslt = 1;	/* fail */

	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);

	if (obj == NULL) {
		GSE_ERR("bma i2c data pointer is null\n");
		return 0;
	}
/* if(atomic_read(&obj->selftest) == 1)  // selftest success */
	if (atomic_read(&obj->selftest) == 0) {	/* selftest success */
		selftest_rslt = 0;	/* success */
	} else {
		GSE_LOG("Self Test Fail : %d\n", atomic_read(&obj->selftest));
		selftest_rslt = 1;	/* fail */
	}
	return sprintf(buf, "%d\n", selftest_rslt);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(cpsdata, S_IWUSR | S_IRUGO, show_cpsdata_value, NULL);
static DRIVER_ATTR(cpsopmode, S_IWUSR | S_IRUGO | S_IWGRP, show_cpsopmode_value,
		   store_cpsopmode_value);
static DRIVER_ATTR(cpsrange, S_IWUSR | S_IRUGO | S_IWGRP, show_cpsrange_value,
		   store_cpsrange_value);
static DRIVER_ATTR(cpsbandwidth, S_IWUSR | S_IRUGO | S_IWGRP, show_cpsbandwidth_value,
		   store_cpsbandwidth_value);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO | S_IWGRP, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO | S_IWGRP, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO | S_IWGRP, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
/* LGE_CHANGE_S : 2012-12-07 taebum81.kim@lge.com AT%SURV : (4) */
static DRIVER_ATTR(softreset, S_IWUSR | S_IWGRP, NULL, bma255_softreset_store);
static DRIVER_ATTR(teststatus, S_IWUSR | S_IRUGO, show_teststatus_value, NULL);
static DRIVER_ATTR(fast_calibration_x, S_IRUGO | S_IWUSR | S_IWGRP, bma255_fast_calibration_x_show,
		   bma255_fast_calibration_x_store);
static DRIVER_ATTR(fast_calibration_y, S_IRUGO | S_IWUSR | S_IWGRP, bma255_fast_calibration_y_show,
		   bma255_fast_calibration_y_store);
static DRIVER_ATTR(fast_calibration_z, S_IRUGO | S_IWUSR | S_IWGRP, bma255_fast_calibration_z_show,
		   bma255_fast_calibration_z_store);
static DRIVER_ATTR(run_fast_calibration, S_IRUGO | S_IWUSR | S_IWGRP, bma255_runCalibration_show,
		   bma255_runCalibration_store);
static DRIVER_ATTR(eeprom_writing, S_IRUGO | S_IWUSR | S_IWGRP, bma255_eeprom_writing_show,
		   bma255_eeprom_writing_store);
static DRIVER_ATTR(bandwidth, S_IRUGO | S_IWUSR | S_IWGRP, bma255_bandwidth_show,
		   bma255_bandwidth_store);
static DRIVER_ATTR(selftest, S_IRUGO | S_IWUSR | S_IWGRP, bma255_SelfTest_show,
		   bma255_SelfTest_store);
#if 1				/* motion sensor cal backup */
static DRIVER_ATTR(cali_backup, S_IWUSR | S_IRUGO | S_IWGRP, show_cali_backup_value, NULL);
#endif
#if 0
static DRIVER_ATTR(limit, S_IWUSR | S_IRUGO | S_IWGRP, show_limit_value, store_limit_value);	/* jwseo_test */
#endif
/* LGE_CHANGE_E : 2012-12-07 taebum81.kim@lge.com AT%SURV : (4) */
/*----------------------------------------------------------------------------*/
static struct driver_attribute *bma255_attr_list[] = {
	&driver_attr_chipinfo,	/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_cali,	/*show calibration data */
	&driver_attr_firlen,	/*filter length: 0: disable, others: enable */
	&driver_attr_trace,	/*trace log */
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_cpsdata,	/*g sensor data for compass tilt compensation */
	&driver_attr_cpsopmode,	/*g sensor opmode for compass tilt compensation */
	&driver_attr_cpsrange,	/*g sensor range for compass tilt compensation */
	&driver_attr_cpsbandwidth,	/*g sensor bandwidth for compass tilt compensation */
	&driver_attr_bandwidth,
	&driver_attr_softreset,
	&driver_attr_teststatus,
	&driver_attr_fast_calibration_x,
	&driver_attr_fast_calibration_y,
	&driver_attr_fast_calibration_z,
	&driver_attr_run_fast_calibration,
	&driver_attr_eeprom_writing,
	&driver_attr_selftest,
#if 1				/* motion sensor cal backup */
	&driver_attr_cali_backup,	/*show calibration backup data */
#endif
#if 0
	&driver_attr_limit,	/* jwseo_test */
#endif
};

/*----------------------------------------------------------------------------*/
static int bma255_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bma255_attr_list) / sizeof(bma255_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bma255_attr_list[idx]);
		if (err) {
			GSE_ERR("driver_create_file (%s) = %d\n", bma255_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma255_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bma255_attr_list) / sizeof(bma255_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, bma255_attr_list[idx]);


	return err;
}

/*----------------------------------------------------------------------------*/
int gsensor_operate(void *self, uint32_t command, void *buff_in, int size_in,
		    void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value, sample_delay;
	struct bma255_i2c_data *priv = (struct bma255_i2c_data *)self;
	hwm_sensor_data *gsensor_data;
	char buff[BMA255_BUFSIZE];
	int ret;

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 5)
				sample_delay = BMA255_BW_200HZ;
			else if (value <= 10)
				sample_delay = BMA255_BW_100HZ;
			else
				sample_delay = BMA255_BW_50HZ;


			err = BMA255_SetBWRate(priv->client, sample_delay);
			if (err != BMA255_SUCCESS) {	/* 0x2C->BW=100Hz */
				GSE_ERR("Set delay parameter error!\n");
			}

			if (value >= 50) {
				atomic_set(&priv->filter, 0);
			} else {
#if defined(CONFIG_BMA255_LOWPASS)
				priv->fir.num = 0;
				priv->fir.idx = 0;
				priv->fir.sum[BMA255_AXIS_X] = 0;
				priv->fir.sum[BMA255_AXIS_Y] = 0;
				priv->fir.sum[BMA255_AXIS_Z] = 0;
				atomic_set(&priv->filter, 1);
#endif
			}
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (((value == 0) && (sensor_power == false))
			    || ((value == 1) && (sensor_power == true))) {
				GSE_LOG("Gsensor device have updated!, power: %d\n", sensor_power);
			} else {
				err = BMA255_SetPowerMode(priv->client, !sensor_power);
			}
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data))) {
			GSE_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			gsensor_data = (hwm_sensor_data *) buff_out;
			BMA255_ReadSensorData(priv->client, buff, BMA255_BUFSIZE);
			ret = sscanf(buff, "%x %x %x", &gsensor_data->values[0],
				     &gsensor_data->values[1], &gsensor_data->values[2]);
			if (ret <= 0)
				GSE_ERR("scanf error!\n");
			gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			gsensor_data->value_divide = 1000;
		}
		break;
	default:
		GSE_ERR("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int bma255_open(struct inode *inode, struct file *file)
{
	file->private_data = bma255_i2c_client;

	if (file->private_data == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int bma255_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long bma255_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct bma255_i2c_data *obj = (struct bma255_i2c_data *)i2c_get_clientdata(client);
	char strbuf[BMA255_BUFSIZE] = { 0, };
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
#if defined(CALIBRATION_TO_FILE)
	int cali[6];
#endif
/* uint32_t enable = 0; */

	GSE_FUN(f);
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));


	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
#if 0
	case GSENSOR_IOCTL_SET_ENABLE:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_from_user(&enable, data, sizeof(enable))) {
			return -EFAULT;
		} else {
			if (enable == 1)
				BMA255_SetPowerMode(obj_i2c_data->client, 1);
			else if (enable == 0)
				BMA255_SetPowerMode(obj_i2c_data->client, 0);

		}
		break;
#endif

	case GSENSOR_IOCTL_INIT:
		bma255_init_client(client, 0);
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		BMA255_ReadChipInfo(client, strbuf, BMA255_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		BMA255_ReadSensorData(client, strbuf, BMA255_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D))) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		BMA255_ReadRawData(client, strbuf);
		if (copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		/* LGE_CHANGE_S : 2012-12-07 taebum81.kim@lge.com AT%SURV : (6) */

#if defined(CALIBRATION_TO_FILE)
		make_cal_data_file();

		err = bma255_calibration_read(cali);
		if (err != 0) {
			GSE_LOG("Read Cal Fail from file !!!\n");
			break;
		} else {
			sensor_data.x = cali[0];
			sensor_data.y = cali[1];
			sensor_data.z = cali[2];
		}
#endif

		if ((sensor_data.x == 0) && (sensor_data.y == 0) && (sensor_data.z == 0)) {
			GSE_LOG("Read Cal Data x : %d / y : %d / z : %d\n", sensor_data.x,
				sensor_data.y, sensor_data.z);
			GSE_LOG("Read Cal Data all Zero, Do not set register\n");
			err = -EINVAL;
			break;
		}

		if (bma255_set_offset_x(obj->client, (unsigned char)sensor_data.x) < 0) {
			err = -EINVAL;
			break;
		}
		if (bma255_set_offset_y(obj->client, (unsigned char)sensor_data.y) < 0) {
			err = -EINVAL;
			break;
		}
		if (bma255_set_offset_z(obj->client, (unsigned char)sensor_data.z) < 0) {
			err = -EINVAL;
			break;
		}

		GSE_LOG("-------------- set sensor cal --------------\n");
		GSE_LOG("x : %d / y : %d / z : %d\n", sensor_data.x, sensor_data.y, sensor_data.z);
		/* LGE_CHANGE_S : 2012-12-07 taebum81.kim@lge.com AT%SURV : (6) */
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		err = BMA255_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		GSE_LOG("GSENSOR_IOCTL_GET_CALI\n");
		break;

	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}


/*----------------------------------------------------------------------------*/
static const struct file_operations bma255_fops = {
	/* .owner = THIS_MODULE, */
	.open = bma255_open,
	.release = bma255_release,
	.unlocked_ioctl = bma255_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static const struct miscdevice bma255_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bma255_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int bma255_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
#if 0
		err = BMA255_SetPowerMode(obj->client, false);
		if (err) {
			GSE_ERR("write power control fail!!\n");
			return;
		}
#endif
		BMA255_power(obj->hw, 0);
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma255_resume(struct i2c_client *client)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	BMA255_power(obj->hw, 1);
#if 0				/* already call PowerManagerSer */
	err = bma255_init_client(client, 0);
	if (err) {
		GSE_ERR("initialize client fail!!\n");
		return err;
	}
#endif
	atomic_set(&obj->suspend, 0);

	return 0;
}

/*----------------------------------------------------------------------------*/
#else				/*CONFIG_HAS_EARLY_SUSPEND is defined */
/*----------------------------------------------------------------------------*/
static void bma255_early_suspend(struct early_suspend *h)
{
	struct bma255_i2c_data *obj = container_of(h, struct bma255_i2c_data, early_drv);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);
#if 0
	err = BMA255_SetPowerMode(obj->client, false);
	if (err) {
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;
#endif
	BMA255_power(obj->hw, 0);
}

/*----------------------------------------------------------------------------*/
static void bma255_late_resume(struct early_suspend *h)
{
	struct bma255_i2c_data *obj = container_of(h, struct bma255_i2c_data, early_drv);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}

	BMA255_power(obj->hw, 1);
#if 0				/* already call PowerManagerSer */
	err = bma255_init_client(obj->client, 0);
	if (err) {
		GSE_ERR("initialize client fail!!\n");
		return;
	}
#endif
	atomic_set(&obj->suspend, 0);
}

/*----------------------------------------------------------------------------*/
#endif				/*CONFIG_HAS_EARLYSUSPEND */
/*----------------------------------------------------------------------------*/
static int bma255_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct bma255_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;

	GSE_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!(obj)) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct bma255_i2c_data));

	obj->hw = get_cust_acc_hw();
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

#ifdef CONFIG_BMA255_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);


	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);

#endif

	bma255_i2c_client = new_client;

	err = bma255_init_client(new_client, 1);
	if (err) {
		GSE_ERR("failed to init BMA255 ( err = %d )\n", err);
		goto exit_init_failed;
	}

	err = misc_register(&bma255_device);
	if (err) {
		GSE_ERR("bma255_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	err = bma255_create_attr(&bma255_gsensor_driver.driver);
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	sobj.self = obj;
	sobj.polling = 1;
	sobj.sensor_operate = gsensor_operate;
	err = hwmsen_attach(ID_ACCELEROMETER, &sobj);
	if (err) {
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	    obj->early_drv.suspend = bma255_early_suspend,
	    obj->early_drv.resume = bma255_late_resume, register_early_suspend(&obj->early_drv);
#endif

	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&bma255_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(new_client); */
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma255_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = bma255_delete_attr(&bma255_gsensor_driver.driver);
	if (err)
		GSE_ERR("bma150_delete_attr fail: %d\n", err);

	err = misc_deregister(&bma255_device);
	if (err)
		GSE_ERR("misc_deregister fail: %d\n", err);

	err = hwmsen_detach(ID_ACCELEROMETER);
	if (err)
		GSE_ERR("hwmsen_detach fail: %d\n", err);

	bma255_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int bma255_probe(struct platform_device *pdev)
{
	struct acc_hw *hw = get_cust_acc_hw();

	GSE_FUN();

	BMA255_power(hw, 1);
	if (i2c_add_driver(&bma255_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int bma255_remove(struct platform_device *pdev)
{
	struct acc_hw *hw = get_cust_acc_hw();

	GSE_FUN();
	BMA255_power(hw, 0);
	i2c_del_driver(&bma255_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static struct platform_driver bma255_gsensor_driver = {
	.probe = bma255_probe,
	.remove = bma255_remove,
	.driver = {
		   .name = "gsensor",
#ifdef CONFIG_OF
		   .of_match_table = gsensor_of_match,
#endif
		   }
};

/*----------------------------------------------------------------------------*/
static int __init bma255_init(void)
{
	struct acc_hw *hw = get_cust_acc_hw();
	unsigned short bma255_i2c_addr = 0x0;

	GSE_FUN();
	bma255_i2c_addr = 0x11;
	bma255_i2c_info.addr = bma255_i2c_addr;
	i2c_register_board_info(hw->i2c_num, &bma255_i2c_info, 1);
	if (platform_driver_register(&bma255_gsensor_driver)) {
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit bma255_exit(void)
{
	GSE_FUN();
	platform_driver_unregister(&bma255_gsensor_driver);
}

/*----------------------------------------------------------------------------*/
module_init(bma255_init);
module_exit(bma255_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA255 I2C driver");
MODULE_AUTHOR("hongji.zhou@bosch-sensortec.com");
