
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif

ifeq ($(CONFIG_MTK_BMA160),y)
obj-y   +=  accelerometer/bma160/
endif

ifeq ($(CONFIG_MTK_LSM6DS3),y)
obj-y   +=  accelerometer/lsm6ds3/
endif

ifeq ($(CONFIG_MTK_MPU6050GY_NEW),y)
obj-y   +=  gyroscope/mpu6050gy-new/
endif

ifeq ($(CONFIG_MTK_AKM8963_NEW),y)
obj-y	+=  magnetometer/akm8963-new/
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

