include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors

obj-y   +=  accel.o accel_factory.o

obj-$(CONFIG_MTK_LSM6DS3)      +=  lsm6ds3/

obj-$(CONFIG_MTK_BMA160)       +=  bma160/

obj-$(CONFIG_MTK_BMI160_STC)   +=  bmi160_stc/

obj-$(CONFIG_MTK_BMA050_NEW)   +=  bma050-new/