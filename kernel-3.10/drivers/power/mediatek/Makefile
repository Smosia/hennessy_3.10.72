include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MTK_HAFG_20),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_fg_20.o battery_common_fg_20.o
else
    obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter.o battery_common.o
endif

ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else
    obj-$(CONFIG_MTK_SMART_BATTERY) += linear_charging.o
endif
