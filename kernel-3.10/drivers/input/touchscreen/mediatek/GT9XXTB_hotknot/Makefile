include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
#ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT9XXTB_hotknot/
ifeq ($(CONFIG_LCM_WIDTH),"600")
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT9XXTB_hotknot_wsvga /
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT9XXTB_hotknot /
endif

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT9XXTB_hotknot/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  goodix_tool.o
obj-y	+=  gt9xx_driver.o
obj-y	+=  gt9xx_update.o

