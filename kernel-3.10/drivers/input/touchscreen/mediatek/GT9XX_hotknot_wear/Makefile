include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT9XX_hotknot_wear/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT9XX_hotknot_wear/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/vibrator/$(MTK_PLATFORM)/

obj-y	+=  goodix_tool.o
obj-y	+=  gt9xx_driver.o
obj-y	+=  gt9xx_update.o

