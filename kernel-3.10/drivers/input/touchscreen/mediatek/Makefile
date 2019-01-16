include $(srctree)/drivers/misc/mediatek/Makefile.custom


# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

ifeq ($(CONFIG_TOUCHSCREEN_MTK_FT5346_CX865),y)
obj-y	+=  ft5346_cx865/
endif

ifeq ($(CONFIG_TOUCHSCREEN_MTK_ATMEL336T_CX865),y)
obj-y	+=  atmel336t_cx865/
endif
