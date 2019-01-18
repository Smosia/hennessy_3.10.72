
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_STK3X1X_NEW),y)
obj-y	+=  alsps/stk3x1x-new1/
endif

ifeq ($(CONFIG_MTK_LTR559),y)
obj-y	+=  alsps/ltr559/
endif

ifeq ($(CONFIG_MTK_BMA160),y)
obj-y   +=  accelerometer/bma160/
endif

ifeq ($(CONFIG_MTK_LSM6DS3),y)
obj-y   +=  accelerometer/lsm6ds3/
endif

ifeq ($(CONFIG_MTK_BMI160_GYRO),y)
obj-y   +=  gyroscope/bmi160_gyro/
endif

ifeq ($(CONFIG_MTK_LSM6DS3_GYRO),y)
obj-y   +=  gyroscope/lsm6ds3_gyro/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif

ifeq ($(CONFIG_MTK_YAS537),y)
obj-y	+=  magnetometer/yamaha537/
endif

ifeq ($(CONFIG_MTK_BMP180_NEW),y)
obj-y	+=  barometer/BMP180-new/
endif
endif
obj-y   +=  camera/camera/
obj-y   +=  vibrator/
obj-y   +=  leds/
obj-y   +=  keypad/
obj-y   +=  sound/
obj-y   +=  accdet/

ifeq ($(CONFIG_MTK_HDMI_SUPPORT),y)
obj-y	+=  hdmi/
endif

