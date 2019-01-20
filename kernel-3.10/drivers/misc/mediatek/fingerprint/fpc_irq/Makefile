include $(srctree)/drivers/misc/mediatek/Makefile.custom
# ----------------------------------------------------------------------------- 
# Web source:
# http://www.cs.fsu.edu/~baker/devices/lxr/http/source/ldd-examples/misc-modules/Makefile
# ----------------------------------------------------------------------------- 
# To build modules outside of the kernel tree, we run "make"
# in the kernel source tree; the Makefile these then includes this
# Makefile once again.
# This conditional selects whether we are being included from the
# kernel Makefile or not.

# ----------------------------------------------------------------------------- 
# export these for cross compiling
# export ARCH=arm 
# export CROSS_COMPILE=arm-linux-eabi-
# export KERNELDIR=[path to kernel source]
# ----------------------------------------------------------------------------- 

fpc_irq-objs := fpc_irq_main.o fpc_irq_common.o fpc_irq_ctrl.o fpc_irq_pm.o fpc_irq_supply.o
# called from kernel build system: just declare what our modules are
obj-y := fpc_irq.o
