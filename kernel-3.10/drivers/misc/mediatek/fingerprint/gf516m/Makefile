#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/fingerprint/include \
             -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/include/trustzone/utils/tlutils \
             -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/FpTrustlet/DrFingerPrint/inc \
             -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/FpTrustlet/TlTest/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/include \
 #            -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/public 

		ifndef CONFIG_TRUSTONIC_TEE_VERSION
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/include \
             -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/public 

		else

ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(CONFIG_TRUSTONIC_TEE_VERSION)/gud/MobiCoreKernelApi/include \
             -I$(srctree)/drivers/misc/mediatek/gud/$(CONFIG_TRUSTONIC_TEE_VERSION)/gud/MobiCoreKernelApi/public 
		endif
endif

obj-$(CONFIG_MTK_GF516)	+= gf516m_tee.o
gf516m_tee-$(CONFIG_MTK_GF516) := mt6795_evb_fp.o gf516m_spi_tee.o
