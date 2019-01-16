include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o da9210.o tps6128x.o

ifeq ($(MTK_PMIC_DVT_SUPPORT),y)
    obj-$(CONFIG_MTK_PMIC)      += pmic_dvt.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o 

ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += bq24296.o charging_hw_bq24296.o
else
     ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
          obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_mt_sw_charger.o
     else
          obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o
     endif
endif

obj-$(CONFIG_CW2015_SUPPORT) += cw2015_battery.o