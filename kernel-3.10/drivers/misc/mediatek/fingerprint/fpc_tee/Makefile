# ----------------------------------------------------------------------------- 
# export these for cross compiling:
# export ARCH=arm 
# export CROSS_COMPILE=arm-linux-eabi-
# export KERNELDIR=[path to kernel source]
# ----------------------------------------------------------------------------- 
include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/include/trustzone/utils/tlutils \
                   -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/spi/Tlspi/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/include \
             -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/public 
endif

obj-y +=fpc_irq.o
fpc_irq-y := fpc_irq_main.o fpc_irq_supply.o fpc_irq_ctrl.o fpc_irq_pm.o fpc_irq_common.o
