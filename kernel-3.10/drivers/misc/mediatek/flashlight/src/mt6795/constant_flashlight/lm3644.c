/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include <mach/upmu_common.h>
//#include <mach/mt6333.h>

#include "kd_flashlight.h"

// ============================================================ //
// Debug configuration
// ============================================================ //
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)

// #define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
#else
	#define PK_DBG(a, ...)
#endif

// ============================================================ //
// Define 
// ============================================================ //
#define REG_ENABLE          0x01
#define REG_FLASH_LED1_BR   0x03
#define REG_FLASH_LED2_BR   0x04
#define REG_TORCH_LED1_BR   0x05
#define REG_TORCH_LED2_BR   0x06
#define REG_FLASH_TOUT      0x08
#define REG_FLAG0           0x0a
#define REG_FLAG1           0x0b
#define REG_ID              0x0c

#define LM3644_NAME         "leds-lm3644"
#define LM3644_ADDR			0x63
#define I2C_BUSNUM			0x03

#define DUTY_NUM   			39

// ============================================================ //
// Global variables
// ============================================================ //
static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */

static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;
static BOOL g_is_ktd2684 = false;

static int gDuty=0;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif

static struct work_struct workTimeOut;

// static int torchLEDReg[DUTY_NUM]=		{16,35,66,82,106,127,16,35,66,82,106,-1,16,35,66,-1,106,127,66,-1,106,127,66,-1,106,127,66,-1,106,127,66,-1,127,-1,127,-1,127,82,127};
// static int torchLEDReg_ktd1[DUTY_NUM] = {8,16,30,41,50,60,8,16,30,41,50,-1,8,16,30,-1,50,60,30,-1,50,60,30,-1,50,60,30,-1,50,60,30,-1,60,-1,60,-1,60,41,60};
// static int torchLEDReg_ktd2[DUTY_NUM] = {4,8,15,20,25,30,4,8,15,20,25,-1,4, 8,15,-1,25,30,15,-1,25,30,15,-1,25,30,15,-1,25,30,15,-1,30,-1,30,-1,30,20,30};
// static int flashLEDReg[DUTY_NUM]=		{-1,-1,-1,-1,-1,-1,7,7,7,7,7,22,22,22,22,37,22,22,37,52,37,37,52,67,52,52,67,82,67,67,82,97,82,112,97,127,112,127,127};
// static int flashLEDReg_ktd[DUTY_NUM]=	{-1,-1,-1,-1,-1,-1,3,3,3,3,3,12,12,12,12,25,12,12,25,34,25,25,34,47,34,34,47,60,47,47,60,63,60,63, 63, 63, 63, 63,63};

static int torchLEDReg[DUTY_NUM]=		{35,35,66,82,106,127,35,35,66,82,106,-1,35,35,66,-1,106,127,66,-1,106,127,66,-1,106,127,66,-1,106,127,66,-1,127,-1,127,-1,127,82,127};
static int torchLEDReg_ktd1[DUTY_NUM] = {16,16,30,41, 50,60, 16,16,30,41, 50,-1,16,16,30,-1, 50, 60,30,-1, 50, 60,30,-1, 50, 60,30,-1, 50, 60,30,-1, 60,-1, 60,-1, 60,41, 60};
static int torchLEDReg_ktd2[DUTY_NUM] = { 8, 8,15,20, 25,30,  8, 8,15,20, 25,-1, 8, 8,15,-1, 25, 30,15,-1, 25, 30,15,-1, 25, 30,15,-1, 25, 30,15,-1, 30,-1, 30,-1, 30,20, 30};
static int flashLEDReg[DUTY_NUM]=		{-1,-1,-1,-1,-1,-1,18,18,18,18,18,38,38,38,38,50,38,38,50,64,50,50,64,76,64,64,76,90,76,76,90,102,90,114,102,127,114,127,127};
static int flashLEDReg_ktd[DUTY_NUM]=	{-1,-1,-1,-1,-1,-1,9, 9, 9, 9, 9, 19,19,19,19,25,19,19,25,32,25,25,32,38,32,32,38,45,38,38,45,51, 45, 57, 51, 63, 57, 63, 63};

static struct i2c_client *LM3644_i2c_client = NULL;
static int flash_enable1 = 0;
static int flash_enable2 = 0;

struct LM3644_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3644_chip_data {
	struct i2c_client *client;

	struct LM3644_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

// ============================================================ //
// Function prototypes
// ============================================================ //
static void work_timeOutFunc(struct work_struct *data);


// ============================================================ //
// Functions
// ============================================================ //
static void work_timeOutFunc(struct work_struct *data);

static int LM3644_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct LM3644_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writting at 0x%02x, client->addr:0x%x, adapt:%d\n", reg, client->addr, client->adapter->nr);
	return ret;
}

static int LM3644_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3644_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

void Enable_GPIO(void)
{
	/* Enable hardware EN pin,LM3644 must Enable this to achieve i2c tranfer */
	PK_DBG("Enable Flash HWEN GPIO\n");
	mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, GPIO_OUT_ONE);

	return;
}

void Disable_GPIO(void)
{
	/* Disable hardware EN pin */
	PK_DBG("Disable Flash HWEN GPIO\n");
	mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, GPIO_OUT_ZERO);
}

static int LM3644_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3644_chip_data *chip;
	struct LM3644_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3644_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG(KERN_ERR  "LM3644 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3644_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL) { //values are set to Zero.
		PK_DBG("LM3644 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3644_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	LM3644_i2c_client = client;

    if ((LM3644_read_reg(LM3644_i2c_client, REG_ID) & 0b1111) == 0x8)
        g_is_ktd2684 = 1;
    else
        g_is_ktd2684 = 0;
    
	PK_DBG("LM3644 Initializing is done -\n");
    return 0;
}

static int LM3644_remove(struct i2c_client *client)
{
	struct LM3644_chip_data *chip = i2c_get_clientdata(client);

   	if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


static const struct i2c_device_id LM3644_id[] = {
	{LM3644_NAME, 0},
	{}
};


#ifdef CONFIG_OF
static const struct of_device_id LM3644_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif
static struct i2c_driver LM3644_i2c_driver = {
	.driver = {
		.name  = LM3644_NAME,
#ifdef CONFIG_OF
		.of_match_table = LM3644_of_match,
#endif
	},
	.probe	= LM3644_probe,
	.remove   = LM3644_remove,
	.id_table = LM3644_id,
};


struct LM3644_platform_data LM3644_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3644={ I2C_BOARD_INFO(LM3644_NAME, LM3644_ADDR), \
													.platform_data = &LM3644_pdata,};

static int __init LM3644_init(void)
{
    i2c_register_board_info(I2C_BUSNUM, &i2c_LM3644, 1);

    if (i2c_add_driver(&LM3644_i2c_driver))
    {
        PK_DBG("Failed to register lm3644_i2c_driver\n");
        return -1;
    }
    PK_DBG("LM3644_init done\n");
    return 0;
}

static void __exit LM3644_exit(void)
{
   Disable_GPIO();
   i2c_del_driver(&LM3644_i2c_driver);
}

module_init(LM3644_init);
module_exit(LM3644_exit);

MODULE_DESCRIPTION("Flash driver for LM3644");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int FL_dim_duty_led1(kal_uint32 duty)
{
    u8 buf[2];
    int torchInd;
    int flashInd;

    if(duty>DUTY_NUM-1)
        duty=DUTY_NUM-1;
    if(duty<0)
        duty=0;
    gDuty=duty;

    if (g_is_ktd2684)
    {
    	torchInd=torchLEDReg_ktd1[gDuty];
    	flashInd=flashLEDReg_ktd[gDuty];
    }
    else
    {
    	torchInd=torchLEDReg[gDuty];
    	flashInd=flashLEDReg[gDuty];
    }
	
    if (torchInd >= 0)
    {
        buf[0]=REG_TORCH_LED1_BR;
    	buf[1]=torchInd;
    	PK_DBG("[LM3644] set  flashlight led1 duty=0x%X,torch mode\n",buf[1]);
    	LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);
    }
    if (flashInd >= 0)
    {
    	buf[0]=REG_FLASH_LED1_BR;
    	buf[1]=flashInd;
    	PK_DBG("[LM3644] set  flashlight led1 duty=0x%X,flash mode\n",buf[1]);
    	LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);
    }
    return 0;
}
int FL_dim_duty_led2(kal_uint32 duty)
{
    u8 buf[2];
    int torchInd;
    int flashInd;

    if(duty>DUTY_NUM-1)
        duty=DUTY_NUM-1;
    if(duty<0)
        duty=0;
    gDuty=duty;

    if (g_is_ktd2684)
    {
    	torchInd=torchLEDReg_ktd2[gDuty];
    	flashInd=flashLEDReg_ktd[gDuty];
    }
    else
    {
    	torchInd=torchLEDReg[gDuty];
    	flashInd=flashLEDReg[gDuty];
    }

    if (torchInd >= 0)
    {
        buf[0]=REG_TORCH_LED1_BR;                       //must set REG_TORCH_LED1_BR bit7 as 0
		buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);
		//выключаем tx pin
		buf[1]=buf[1]&0x7f;
    	LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);

		buf[0]=REG_TORCH_LED2_BR;
		buf[1]=torchInd;
    	LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);
    	PK_DBG("[LM3644] set  flashlight led2 duty=0x%X,torch mode\n",buf[1]);
    }
    if (flashInd >= 0)
    {
        buf[0]=REG_FLASH_LED1_BR;                       //must set REG_FLASH_LED1_BR bit7 as 0
		buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);
		buf[1]=buf[1]&0x7f;
    	LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);

		buf[0]=REG_FLASH_LED2_BR;
    	buf[1]=flashInd;
    	LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);
    	PK_DBG("[LM3644] set  flashlight led2 duty=0x%X,flash mode\n",buf[1]);
    }
    
    return 0;
}

static int user1 = 0;
static int user2 = 0;
int FL_Enable_led1(void)
{
    u8 buf[2];
    int torchInd;
    int flashInd;

    buf[0]=REG_ENABLE;
    buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);

    if (g_is_ktd2684)
    {
    	torchInd=torchLEDReg_ktd1[gDuty];
    	flashInd=flashLEDReg_ktd[gDuty];
    }
    else
    {
    	torchInd=torchLEDReg[gDuty];
    	flashInd=flashLEDReg[gDuty];
    }

    if (torchInd >= 0)
    {
    	buf[1]=(buf[1]|0x09)&0x0f;
    	PK_DBG("[LM3644] Enable the flashlight led1,torch mode\n");	
    }
    if (flashInd >= 0)
    {
		buf[1]=(buf[1]|0x0d)&0x0f;
    	PK_DBG("[LM3644] Enable the flashlight led1,flash mode\n");	
    }
    LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);

    user1 = 1;

    return 0;
}

int FL_Enable_led2(void)
{
    u8 buf[2];
    int torchInd;
    int flashInd;

    buf[0]=REG_ENABLE;
    buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);
    
    if (g_is_ktd2684)
    {
    	torchInd=torchLEDReg_ktd1[gDuty];
    	flashInd=flashLEDReg_ktd[gDuty];
    }
    else
    {
    	torchInd=torchLEDReg[gDuty];
    	flashInd=flashLEDReg[gDuty];
    }

    if (torchInd >= 0)
    {
    	buf[1]=(buf[1]|0x0a)&0x0f;
    	PK_DBG("[LM3644] Enable the flashlight led2,torch mode\n");	
    }
    if (flashInd >= 0)
    {
		buf[1]=(buf[1]|0x0e)&0x0f;
    	PK_DBG("[LM3644] Enable the flashlight led2,flash mode\n");	
    }
    LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);

    user2 = 1;
    
    return 0;
}


int FL_Disable_led1(void)
{
    u8 buf[2];
    buf[0]=REG_ENABLE;
    buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);
    
    //tx =off, led 1=off
    buf[1]=buf[1]&0xfe;
    LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);
    
    user1 = 0;
    if ((user1 + user2) == 0) 
    {
		buf[0]=REG_ENABLE;
		buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);
		//tx = off, standby = on
		buf[1]=buf[1]&0xf3;
		LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);
    }

    return 0;
}

int FL_Disable_led2(void)
{
    u8 buf[2];
    buf[0]=REG_ENABLE;
    buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);
    
    //tx =off, led 2=off
    buf[1]=buf[1]&0xfd;
    LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);

    user2 = 0;
    if ((user1 + user2) == 0) 
    {
		buf[0]=REG_ENABLE;
		buf[1]=LM3644_read_reg(LM3644_i2c_client,buf[0]);
		//tx = off, standby = on
		buf[1]=buf[1]&0xf3;
		LM3644_write_reg(LM3644_i2c_client, buf[0], buf[1]);
    }
    
    return 0;
}

int FL_current_timeout_set(void)
{
	PK_DBG("%s. line=%d\n", __func__, __LINE__);
	if (g_is_ktd2684)
		LM3644_write_reg(LM3644_i2c_client, REG_FLASH_TOUT, 0x1a);
	else
		LM3644_write_reg(LM3644_i2c_client, REG_FLASH_TOUT, 0x1f);
	return 0;
}

int FL_Enable(void)
{
	FL_Enable_led1();
	FL_Enable_led2();
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	FL_dim_duty_led1(duty);
	FL_dim_duty_led2(duty);
	return 0;
}

int FL_Disable(void)
{
	FL_Disable_led1();
	FL_Disable_led2();
	return 0;
}

static int FL_Init(void)
{
    Enable_GPIO();
    FL_current_timeout_set();

    PK_DBG("[LM3644] init done, FL_Init line=%d\n",__LINE__);
    return 0;
}


static int FL_Uninit(void)
{
    PK_DBG("%s. line=%d\n", __func__, __LINE__);
    FL_Disable();
    flash_enable1 = 0;
    flash_enable2 = 0;
    Disable_GPIO();
    
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    flash_enable1 = 0;
    PK_DBG("ledTimeOut_callback\n");
}

static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}

static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
    switch(cmd)
    {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
		g_timeOutTimeMs=arg;
		break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASH_IOC_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;

    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);
    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASH_IOC_ONOFF: %d\n",(int)arg);
    		if(arg==1 && flash_enable1 == 0)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
				flash_enable1 = 1;
				flash_enable2 = 1;
    		}
    		else if (arg == 0 && flash_enable1 == 1)
    		{
    			FL_Disable();
				flash_enable1 = 0;
				flash_enable2 = 0;
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
	default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}

static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
