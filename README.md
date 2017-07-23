# Custom kernel for Xiaomi Redmi Note 3 (Hennessy)
# Kernel version 3.10.72
# Vendor Vanzo (ALPS-MP-M0.MP11-V1_VZ6795_LWT_M)

Works in rom(CM13): http://4pda.ru/forum/index.php?showtopic=716960&view=findpost&p=55392848

=========================================================================
* Works:
	* Rill
	* LCM (NT35596_TIANMA, NT35532_BOE, NT35532_BOE_S, R63315_SHARP)
	* Touch (ATMEL, FT5206)
	* BQ24296 (small bug with charging)
	* CW2015
	* Wi-fi
	* BT
	* GPS
	* FM
	* Button-backlight
	* Brightness 
	* Leds indication
	* Alsps (LT559 and STK)
	* Accel (BMI160_ACC LSM6DS3_ACCEL)
	* Giro
	* Vibrator
	* Battery 4000mah
	* Camera (s5k3m2, OV5670)
	* Lens
	* Sound(Speaker, Headphones)
	* OTG
	* Flashlight
	* Fixed graphics bug

=========================================================================
* Disabled drivers:
    * MAGNETOMETER             (AKM09911_NEW)

=========================================================================
* Don't work:
	* IR Blaster
	* HALL sensor (ah1903_probe: GPIO11)
	* Fingerprint:
		* GF516M (reset=115, irq=3, pwr_gpio=94, gf516m_probe: 94,167,115,166,168,169, gf516m_debug_store:166,167,168,169,115,3)
		* FPC1020 (reset=115, irq=3)
    * MAGNETOMETER             (YAS537)

=========================================================================
* TODO:
	* battery_meter.c: correct current mesurements in amperka apk
	* fix charging bug
	* fix problems with LSM6DS3_ACCEL
	* fix problems with camera
	* make flashlight brighter

=========================================================================
# BUILD
export TOP=$(pwd)
export CROSS_COMPILE=/home/smosia/Android/utility/aarch64-linux-android-4.9-linaro-master/bin/aarch64-linux-android-
mkdir -p $TOP/KERNEL_OBJ
make -C kernel-3.10 O=$TOP/KERNEL_OBJ ARCH=arm64 MTK_TARGET_PROJECT=hennessy TARGET_BUILD_VARIANT=eng CROSS_COMPILE=$TOOLCHAIN ROOTDIR=$TOP hennessy_defconfig
make -j4 -C kernel-3.10 O=$TOP/KERNEL_OBJ ROOTDIR=$TOP

# Busses
* I2C0:
	* DW9761BAF 	            (0018)
	* BU6429AF 	            	(0019)
	* CAM_CAL_DRV           	(0036)
	* OV13853_CAM_CAL_DRV    	(0037)
	* tps65132              	(003e)
	* kd_camera_hw          	(007f)
* I2C1:
	* da9210                	(0068)	
	* tps6128x              	(0075)	
* I2C2:
	* FT						(0038) 	
	* atmel_mxt_ts           	(004a)	
	* kd_camera_hw_bus2    		(007f)		
* I2C3:
	* akm09911               	(000c)	
	* LTR_559ALS				(0023)	
	* yas537                	(002e)	
	* LSM6DS3_GYRO				(0034)	
	* stk3x1x               	(0048) 	
	* lm3643					(0063)	
	* bmi160_gyro				(0066)	
	* bmi160_acc				(0068)	
	* LSM6DS3_ACCEL         	(006a)	
	* bq24296         			(006b)	
* I2C4:
	* CW2015 					(0062)
* SPI0.0:
	* fpc1020							
* SPI0.1:
	* fp_spi							

# AUTHOR:
* Smosia

# Original kernel by:
* nofearnohappy
* LazyC0DEr
* Anomalchik