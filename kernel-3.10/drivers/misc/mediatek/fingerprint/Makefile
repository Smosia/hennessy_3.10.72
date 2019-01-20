include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y   +=  mt6795_evb_fp.o

ifeq ($(CONFIG_FINGERPRINT_GF516),y)
obj-y   += gf516m/
endif
ifeq ($(CONFIG_FINGERPRINT_FPC1022),y)
obj-y	+=  FPC_1022/
endif
ifeq ($(CONFIG_FINGERPRINT_FPC_IRQ),y)
obj-y	+=  fpc_irq/
endif