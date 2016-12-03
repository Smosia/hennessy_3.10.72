#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_FINGERPRINT_GOODIX_GF_TEE)	+= gf_tee.o
gf_tee-$(CONFIG_FINGERPRINT_GOODIX_GF_TEE) := gf_spi_tee.o
