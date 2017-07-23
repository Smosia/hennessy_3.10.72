export TOP=$(pwd)
export CROSS_COMPILE=/home/smosia/Android/utilites/aarch64-linux-android-4.9-linaro-master/bin/aarch64-linux-android-
mkdir -p $TOP/KERNEL_OBJ
make -C kernel-3.10 O=$TOP/KERNEL_OBJ ARCH=arm64 MTK_TARGET_PROJECT=hennessy TARGET_BUILD_VARIANT=eng CROSS_COMPILE=$TOOLCHAIN ROOTDIR=$TOP hennessy_defconfig
make -j4 -C kernel-3.10 O=$TOP/KERNEL_OBJ ROOTDIR=$TOP

# userfriendly :)
rm /media/psf/Home/Android/CarlivImageKitchen64/boot/boot.img-kernel
cp /media/psf/Home/Android/kernel/test_3.10.72/KERNEL_OBJ/arch/arm64/boot/Image.gz-dtb /media/psf/Home/Android/CarlivImageKitchen64/boot/boot.img-kernel
