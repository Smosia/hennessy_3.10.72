#ifndef _KD_IMGSENSOR_H
#define _KD_IMGSENSOR_H

#include <linux/ioctl.h>
/* #define CONFIG_COMPAT */
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#ifndef ASSERT
#define ASSERT(expr)        BUG_ON(!(expr))
#endif

#define IMGSENSORMAGIC 'i'
/* IOCTRL(inode * ,file * ,cmd ,arg ) */
/* S means "set through a ptr" */
/* T means "tell by a arg value" */
/* G means "get by a ptr" */
/* Q means "get by return a value" */
/* X means "switch G and S atomically" */
/* H means "switch T and Q atomically" */

/*******************************************************************************
*
********************************************************************************/
#define YUV_INFO(_id, name, getCalData)\
	{ \
		_id, name, \
NSFeature :  : YUVSensorInfo < _id >  :  : createInstance(name, #name), \
		(NSFeature :  : SensorInfoBase*(*)()) \
NSFeature :  : YUVSensorInfo < _id >  :  : getInstance, \
NSFeature :  : YUVSensorInfo < _id >  :  : getDefaultData, \
		getCalData, \
NSFeature :  : YUVSensorInfo < _id >  :  : getNullFlickerPara \
	}
#define RAW_INFO(_id, name, getCalData)\
	{ \
		_id, name, \
NSFeature :  : RAWSensorInfo < _id >  :  : createInstance(name, #name), \
		(NSFeature :  : SensorInfoBase*(*)()) \
NSFeature :  : RAWSensorInfo < _id >  :  : getInstance, \
NSFeature :  : RAWSensorInfo < _id >  :  : getDefaultData, \
		getCalData, \
NSFeature :  : RAWSensorInfo < _id >  :  : getFlickerPara \
	}
/*******************************************************************************
*
********************************************************************************/

/* sensorOpen */
#define KDIMGSENSORIOC_T_OPEN                       _IO(IMGSENSORMAGIC, 0)
/* sensorGetInfo */
#define KDIMGSENSORIOC_X_GETINFO                    _IOWR(IMGSENSORMAGIC, 5, ACDK_SENSOR_GETINFO_STRUCT)
/* sensorGetResolution */
#define KDIMGSENSORIOC_X_GETRESOLUTION              _IOWR(IMGSENSORMAGIC, 10, ACDK_SENSOR_RESOLUTION_INFO_STRUCT)
/* For kernel 64-bit */
#define KDIMGSENSORIOC_X_GETRESOLUTION2             _IOWR(IMGSENSORMAGIC, 10, ACDK_SENSOR_PRESOLUTION_STRUCT)
/* sensorFeatureControl */
#define KDIMGSENSORIOC_X_FEATURECONCTROL            _IOWR(IMGSENSORMAGIC, 15, ACDK_SENSOR_FEATURECONTROL_STRUCT)
/* sensorControl */
#define KDIMGSENSORIOC_X_CONTROL                    _IOWR(IMGSENSORMAGIC, 20, ACDK_SENSOR_CONTROL_STRUCT)
/* sensorClose */
#define KDIMGSENSORIOC_T_CLOSE                      _IO(IMGSENSORMAGIC, 25)
/* sensorSearch */
#define KDIMGSENSORIOC_T_CHECK_IS_ALIVE             _IO(IMGSENSORMAGIC, 30)
/* set sensor driver */
#define KDIMGSENSORIOC_X_SET_DRIVER                 _IOWR(IMGSENSORMAGIC, 35, SENSOR_DRIVER_INDEX_STRUCT)
/* get socket postion */
#define KDIMGSENSORIOC_X_GET_SOCKET_POS             _IOWR(IMGSENSORMAGIC, 40, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_SET_I2CBUS                 _IOWR(IMGSENSORMAGIC, 45, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK   _IO(IMGSENSORMAGIC, 50)
/* Set Shutter Gain Wait Done */
#define KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE _IOWR(IMGSENSORMAGIC, 55, u32)
/* set mclk */
#define KDIMGSENSORIOC_X_SET_MCLK_PLL               _IOWR(IMGSENSORMAGIC, 60, ACDK_SENSOR_MCLK_STRUCT)
#define KDIMGSENSORIOC_X_GETINFO2                   _IOWR(IMGSENSORMAGIC, 65, IMAGESENSOR_GETINFO_STRUCT)
/* set open/close sensor index */
#define KDIMGSENSORIOC_X_SET_CURRENT_SENSOR         _IOWR(IMGSENSORMAGIC, 70, u32)
/* set GPIO */
#define KDIMGSENSORIOC_X_SET_GPIO                   _IOWR(IMGSENSORMAGIC, 75, IMGSENSOR_GPIO_STRUCT)
/* Get ISP CLK */
#define KDIMGSENSORIOC_X_GET_ISP_CLK                _IOWR(IMGSENSORMAGIC, 80, u32)

#ifdef CONFIG_COMPAT
#define COMPAT_KDIMGSENSORIOC_X_GETINFO            _IOWR(IMGSENSORMAGIC, 5, COMPAT_ACDK_SENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL    _IOWR(IMGSENSORMAGIC, 15, COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_CONTROL            _IOWR(IMGSENSORMAGIC, 20, COMPAT_ACDK_SENSOR_CONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETINFO2           _IOWR(IMGSENSORMAGIC, 65, COMPAT_IMAGESENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETRESOLUTION2     _IOWR(IMGSENSORMAGIC, 10, COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT)
#endif

/*******************************************************************************
*
********************************************************************************/
/* SENSOR CHIP VERSION */
#define S5K3M2_SENSOR_ID                        0x30D2
#define S5K3M2_2ND_SENSOR_ID                    0x30D3
#define OV13853_SENSOR_ID                       0xD853
#define OV13853_SENSOR_ID_MTS                   0xD854
#define OV5670MIPI_SENSOR_ID                    0x5670
#define OV5670_2ND_MIPI_SENSOR_ID               0x5671
#define OV5670_FLT_2ND_MIPI_SENSOR_ID           0x5673
#define S5K5E8YX_SENSOR_ID                      0x5e80

/* CAMERA DRIVER NAME */
#define CAMERA_HW_DEVNAME                       "kd_camera_hw"
/* SENSOR DEVICE DRIVER NAME */
#define SENSOR_DRVNAME_S5K3M2_MIPI_RAW          "s5k3m2mipiraw"
#define SENSOR_DRVNAME_S5K3M2_2ND_MIPI_RAW      "s5k3m2_2ndmipiraw"
#define SENSOR_DRVNAME_OV13853_MIPI_RAW         "ov13853mipiraw"
#define SENSOR_DRVNAME_OV13853_MIPI_RAW_MTS     "ov13853mipiraw_mts"
#define SENSOR_DRVNAME_OV5670_MIPI_RAW          "ov5670mipi"
#define SENSOR_DRVNAME_OV5670_2ND_MIPI_RAW      "ov5670_2nd_mipi"
#define SENSOR_DRVNAME_OV5670_FLT_2ND_MIPI_RAW  "ov5670_flt_2nd_mipi"
#define SENSOR_DRVNAME_S5K5E8YX_MIPI_RAW        "s5k5e8yxmipiraw"

/*******************************************************************************
*
********************************************************************************/
void KD_IMGSENSOR_PROFILE_INIT(void);
void KD_IMGSENSOR_PROFILE(char *tag);

#define mDELAY(ms)     mdelay(ms)
#define uDELAY(us)       udelay(us)
#endif              /* _KD_IMGSENSOR_H */
