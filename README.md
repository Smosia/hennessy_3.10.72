# Custom kernel for Xiaomi Redmi Note 3 (Hennessy)
## Kernel from Vanzo ALPS (ALPS-MP-M0.MP11-V1_VZ6795_LWT_M)

=========================================================================
* Works:

=========================================================================
* Don't tested:
    * Everything

=========================================================================
* Don't work:

=========================================================================
* TODO:

=========================================================================
# BUILD
export TOP=$(pwd)
export CROSS_COMPILE=/home/smosia/Android/utility/aarch64-linux-android-4.9-linaro-master/bin/aarch64-linux-android-
mkdir -p $TOP/KERNEL_OBJ
make -C kernel-3.10 O=$TOP/KERNEL_OBJ ARCH=arm64 MTK_TARGET_PROJECT=hennessy TARGET_BUILD_VARIANT=eng CROSS_COMPILE=$TOOLCHAIN ROOTDIR=$TOP hennessy_defconfig
make -j4 -C kernel-3.10 O=$TOP/KERNEL_OBJ ROOTDIR=$TOP						

# AUTHOR:
* Smosia