
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
//#include "lm3644.h"

// ============================================================ //
// Debug configuration
// ============================================================ //
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG printk
	#define PK_VER printk
	#define PK_ERR printk
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

#define LM3644_NAME         "leds-lm3644"
#define LM3644_ADDR         0x63
#define I2C_BUSNUM          3

/* registers definitions */
#define REG_ENABLE          0x01
#define REG_FLASH_LED0_BR   0x03
#define REG_FLASH_LED1_BR   0x04
#define REG_TORCH_LED0_BR   0x05
#define REG_TORCH_LED1_BR   0x06
#define REG_FLASH_TOUT      0x08
#define REG_FLAG0           0x0a
#define REG_FLAG1           0x0b
#define REG_ID              0x0c

#define to_lm3644(_ctrl, _no) container_of(_ctrl, struct lm3644, cdev[_no])


// ============================================================ //
// Global variable
// ============================================================ //
static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */

static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif

enum
{
	e_DutyNum = 26,
};

//ok
static int isMovieMode[e_DutyNum]   = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchLEDReg[e_DutyNum] = {35,71,106,127,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchLEDReg_ktd1[e_DutyNum] = {16,33,50,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchLEDReg_ktd2[e_DutyNum] = {8,16,25,30,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int flashLEDReg[e_DutyNum] = {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110,118,127};
static int flashLEDReg_ktd[e_DutyNum] = {2,5,9,10,12,16,18,21,25,28,31,34,37,41,44,47,50,54,57,60,63,63,63,63,63,63};
static int m_duty1_flash2=0;
int m_duty2_flash2=0;
static int LED1Closeflag_flash2 = 0;
int LED2Closeflag_flash2 = 0;


static struct work_struct workTimeOut;
static struct hrtimer g_timeOutTimer;

static int g_is_ktd2684 = 0;
static struct i2c_client *lm3644_i2c_client = NULL;

// ============================================================ //
//	extern variable
// ============================================================ //


// ============================================================ //
// extern function
// ============================================================ //
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

// ============================================================ //
// function prototype
// ============================================================ //
static void work_timeOutFunc(struct work_struct *data);

// ============================================================ //
// local function
// ============================================================ //
/*
int readReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR, 1, LM3644_ADDR);
    PK_DBG("readReg reg=0x%x val=0x%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
}

int writeReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
    PK_DBG("writeReg reg=0x%x val=0x%x qq\n", buf[0],buf[1]);
    iWriteRegI2C(buf, 2, LM3644_ADDR);
    return 0;
}
*/
//

//w0 -?
//w1 - переменная какой регистр
//w2 - 1
//w3 - переменная куда читать
//w4 - 1
//w5 - адрес
/*
static int iReadRegI2C_lm(su8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
    int ret=0;

    LM3643_i2c_client->ext_flag &= ~I2C_DMA_FLAG;
    LM3643_i2c_client->addr = i2cId;

    ret = i2c_master_send(LM3643_i2c_client, a_pSendData, a_sizeSendData);
    if (ret != 1) {
        PK_ERR("LM3643] %s failed writting at 0x%02x\n", __func__, a_pSendData);
        return 0xFFFFFFFF;
    }
    ret = i2c_master_recv(LM3643_i2c_client, a_pRecvData, a_sizeRecvData);
    if (ret != 1) {
        PK_ERR("LM3643] %s failed reading at 0x%02x\n", __func__, a_pRecvData);
        return 0xFFFFFFFF;
    }

    return 0;
}
*/

//w0 - client_i2c
//w1 - регистр какой читать
static int read_data_lm3644(struct i2c_client *client, u8 reg)
{
    char val = 0;
    int ret = 0;

    client->ext_flag &= ~I2C_DMA_FLAG;
    client->addr = LM3644_ADDR;

    ret = i2c_master_send(client, &reg, 1);
    if (ret != 1) {
        PK_ERR("LM3644] %s failed writting at 0x%02x\n", __func__, reg);
        return 0xFFFFFFFF;
    }

    ret = i2c_master_recv(client, &val, 1);
    if (ret != 1) {
        PK_ERR("LM3644] %s failed reading at 0x%02x\n", __func__, val);
        return 0xFFFFFFFF;
    }
    //struct lm3644 *chip = i2c_get_clientdata(client);

    //mutex_lock(&chip->lock);
    //val = iReadRegI2C_lm(reg, 0x1, val, 0x1, STROBE_DEVICE_ID);
    //mutex_unlock(&chip->lock);
    return val;
}

//reversed
static int write_data_lm3644(struct i2c_client *client, u8 reg, u8 val)
{
    int ret=0;
    char buf[2] = {(char) reg, (char)val};

    PK_DBG("<%s:%d>reg[0x%x]= %d\n", __FUNCTION__, __LINE__, reg, val);
    client->ext_flag &= ~I2C_DMA_FLAG;
    client->addr = LM3644_ADDR;

    ret = i2c_master_send(client, buf, 2);

    if (ret < 0)
        PK_ERR("[LM3644] failed writting at 0x%02x\n", reg);
    return ret;
}

int readReg(int reg)
{
    int val=0;

    val = read_data_lm3644(lm3644_i2c_client, reg);
    PK_DBG("<%s:%d>reg[0x%x]= %d\n", __func__, __LINE__, reg, val);

    return val;
}

//reversed
int writeReg(int reg, int data)
{
    int val=0;
    
    val = write_data_lm3644(lm3644_i2c_client, reg, data);
    PK_DBG("<%s:%d>reg[0x%x]= %d\n", __func__, __LINE__, reg, data);

    return val;
}


//reversed
int flashEnable_LM3643_1(void)
{
    return 0;
}

//reversed
int setDuty_LM3643_1(int duty)
{
    if(duty<0)
        duty=0;
    else if(duty>=e_DutyNum)
        duty=e_DutyNum-1;
    m_duty1_flash2=duty;
    
    return 0;
}

//ok
int flashEnable_LM3643_2(void)
{
    int temp;
    PK_DBG("flashEnable_LM3643_2");
    PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d", LED1Closeflag_flash2, LED2Closeflag_flash2);

    temp = readReg(REG_ENABLE);

    if((LED1Closeflag_flash2 == 1) && (LED2Closeflag_flash2 == 1))
    {
        writeReg(REG_ENABLE, temp & 0xF0);//close       
    }
    else if(LED1Closeflag_flash2 == 1)
    {
        if(isMovieMode[m_duty2_flash2] == 1)
            writeReg(REG_ENABLE, 0xFA);//torch mode
        else
            writeReg(REG_ENABLE, 0xFE);//flash mode
    }
    else if(LED2Closeflag_flash2 == 1)
    {
        if(isMovieMode[m_duty1_flash2] == 1)
            writeReg(REG_ENABLE, 0xF9);//torch mode
        else
            writeReg(REG_ENABLE, 0xFD);//flash mode     
    }
    else
    {
        if((isMovieMode[m_duty1_flash2] == 1) & (isMovieMode[m_duty2_flash2] == 1))
            writeReg(REG_ENABLE, 0xFB);//torch mode
        else
            writeReg(REG_ENABLE, 0xFF);//flash mode
    }
    return 0;
}

//ok
int flashDisable_LM3643_2(void)
{
    flashEnable_LM3643_2();
    return 0;
}

//ok
int flashDisable_LM3643_1(void)
{
    return 0;
}

//ok
int setDuty_LM3643_2(int duty)
{
    if(duty<0)
        duty=0;
    else if(duty>=e_DutyNum)
        duty=e_DutyNum-1;
    m_duty2_flash2=duty;

    PK_DBG("setDuty_LM3643_2:m_duty = %d, m_duty2_flash2 = %d!\n", m_duty1_flash2, m_duty2_flash2);
    PK_DBG("LED1Closeflag_flash2 = %d, LED2Closeflag_flash2 = %d\n", LED1Closeflag_flash2, LED2Closeflag_flash2);

    if((LED1Closeflag_flash2 == 1) && (LED2Closeflag_flash2 == 1))
    {
        
    }
    else if(LED1Closeflag_flash2 == 1)
    {
        if(isMovieMode[m_duty2_flash2] == 1)
        {
            if (g_is_ktd2684)
                writeReg(REG_TORCH_LED1_BR, torchLEDReg_ktd2[m_duty2_flash2]);
            else
                writeReg(REG_TORCH_LED1_BR, torchLEDReg[m_duty2_flash2]);
        }
        else
        {
            if (g_is_ktd2684)
                writeReg(REG_FLASH_LED1_BR, flashLEDReg_ktd[m_duty2_flash2]);
            else
                writeReg(REG_FLASH_LED1_BR, flashLEDReg[m_duty2_flash2]);
        }
    }
    else if(LED2Closeflag_flash2 == 1)
    {
        if(isMovieMode[m_duty1_flash2] == 1)
        {
            if (g_is_ktd2684)
                writeReg(REG_TORCH_LED0_BR, torchLEDReg_ktd1[m_duty1_flash2]);
            else
                writeReg(REG_TORCH_LED0_BR, torchLEDReg[m_duty1_flash2]);
        }
        else
        {
            writeReg(REG_FLASH_LED0_BR, flashLEDReg[m_duty1_flash2]);   
        }       
    }
    else
    {
        if((isMovieMode[m_duty1_flash2] == 1) && ((isMovieMode[m_duty2_flash2] == 1)))
        {
            if (g_is_ktd2684)
            {
                writeReg(REG_TORCH_LED0_BR, torchLEDReg_ktd1[m_duty1_flash2]);
                writeReg(REG_TORCH_LED1_BR, torchLEDReg_ktd2[m_duty2_flash2]);
            }
            else
            {
                writeReg(REG_TORCH_LED0_BR, torchLEDReg[m_duty1_flash2]);
                writeReg(REG_TORCH_LED1_BR, torchLEDReg[m_duty2_flash2]);
            }
        }
        else
        {
            writeReg(REG_FLASH_LED0_BR, flashLEDReg[m_duty1_flash2]);
            if (g_is_ktd2684)
                writeReg(REG_FLASH_LED1_BR, flashLEDReg_ktd[m_duty2_flash2]);
            else
                writeReg(REG_FLASH_LED1_BR, flashLEDReg[m_duty2_flash2]);
        }
    }
    return 0;
}
/*
//reversed
static int write_data_lm3643(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	char buf[2] = {(char) reg, (char)val};

	PK_DBG("<%s:%d>reg[0x%x]= %d\n", __func__, __LINE__, reg, val);
	LM3643_i2c_client->ext_flag &= ~I2C_DMA_FLAG;
	LM3643_i2c_client->addr = 0x63;

	ret = i2c_master_send(LM3643_i2c_client, buf, 2);

	if (ret < 0)
		PK_ERR("LM3643] failed writting at 0x%02x\n", reg);
	return ret;
}

//reversed
static int iReadRegI2C_lm(su8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int ret=0;

	LM3643_i2c_client->ext_flag &= ~I2C_DMA_FLAG;
	LM3643_i2c_client->addr = i2cId;

	ret = i2c_master_send(LM3643_i2c_client, a_pSendData, a_sizeSendData);
	if (ret != 1) {
		PK_ERR("LM3643] %s failed writting at 0x%02x\n", __func__, a_pSendData);
		return 0xFFFFFFFF;
	}
	ret = i2c_master_recv(LM3643_i2c_client, a_pRecvData, a_sizeRecvData);
	if (ret != 1) {
		PK_ERR("LM3643] %s failed reading at 0x%02x\n", __func__, a_pRecvData);
		return 0xFFFFFFFF;
	}

	return 0;
}

//reversed need to check
static int read_data_lm3643(struct i2c_client *client, u8 reg)
{
	int val = 0;

	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = iReadRegI2C_lm(reg, 0x1, val, 0x1, STROBE_DEVICE_ID);
	mutex_unlock(&chip->lock);

	return val;
}
*/
int flashlight_clear_brightness(void) 
{
    //PK_DBG("[flashlight] clear[%s] [%d][%d]",,current_level,)
    //current_level = 0;
    return 0;
}

void flashlight_onoff(int arg)
{
    if (strobe_Res == 0)
    {
        PK_DBG("<%s:%d>level[%x]", __FUNCTION__, __LINE__, arg);
        //if (arg == 0x64)
    }
    else
        flashlight_clear_brightness();
    return; 
}

//ok
int FL_Init(void)
{
	PK_DBG("LED1_FL_Init!\n");

	if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN, GPIO_MODE_00)) {PK_DBG("set gpio mode failed!!\n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN, GPIO_DIR_OUT)) {PK_DBG("set gpio dir failed!!\n");}
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, GPIO_OUT_ONE)) {PK_DBG("set gpio failed!!\n");}		
	
    if (g_is_ktd2684 == 1)
		writeReg(REG_FLASH_TOUT, 0x1A);
	else
		writeReg(REG_FLASH_TOUT, 0x1F);

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}

//ok
int FL_Disable(void)
{
	PK_DBG("FL_Disable line=%d\n", __LINE__);
    return 0;
}

//ok
int FL_Enable(void)
{
    PK_DBG(" FL_Enable line=%d\n",__LINE__);

    return 0;
}


// /*****************************************************************************
// User interface
// *****************************************************************************/
//ok
enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

//ok
static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("LED1TimeOut_callback\n");
}

static struct hrtimer g_timeOutTimer;

//ok
void timerInit(void)
{
    INIT_WORK(&workTimeOut, work_timeOutFunc);
    g_timeOutTimeMs=1000; //1s
    hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    g_timeOutTimer.function=ledTimeOutCallback;
}

//ok
static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3643_LED1_constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {
    	//ok
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;

		//ok
    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		m_duty1_flash2=arg;
    		break;

    	//ok
    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);
    		break;

        //ok
    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
                LED1Closeflag_flash2 = 0;
    			FL_Enable();
    		}
    		else
    		{
    			LED1Closeflag_flash2 = 1;
                FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
		//i4RetValue=readReg(arg);
		//PK_DBG("  arg=%d,i4RetValue=%d\n",arg,i4RetValue);
    	    break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    
    return i4RetValue;
}

//ok
static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;

    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	if (strobe_Res == 0)
	{
	    FL_Init();
		timerInit();
		flashlight_clear_brightness();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	
    spin_lock_irq(&g_strobeSMPLock);
    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res = 1;
    }
    spin_unlock_irq(&g_strobeSMPLock);

    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}

//ok
static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

        PK_DBG(" LED1_FL_Uninit!\n");
    }
    PK_DBG(" Done\n");

    return 0;
}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};

//ok
MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);

/*
 * drivers/leds/leds-lm3644.c
 * General device driver for TI LM3644, FLASH LED Driver
 *
 * Copyright (C) 2014 Texas Instruments
 *
 * Contact: Daniel Jeong <gshark.jeong@gmail.com>
 *          Ldd-Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_gpio_usage.h>

enum lm3644_devid {
    ID_FLASH0 = 0x0,
    ID_FLASH1,
    ID_TORCH0,
    ID_TORCH1,
    ID_MAX
};

enum lm3644_mode {
    MODE_STDBY = 0x0,
    MODE_IR,
    MODE_TORCH,
    MODE_FLASH,
    MODE_MAX
};

enum lm3644_devfile {
    DFILE_FLASH0_ENABLE = 0,
    DFILE_FLASH0_ONOFF,
    DFILE_FLASH0_SOURCE,
    DFILE_FLASH0_TIMEOUT,
    DFILE_FLASH1_ENABLE,
    DFILE_FLASH1_ONOFF,
    DFILE_TORCH0_ENABLE,
    DFILE_TORCH0_ONOFF,
    DFILE_TORCH0_SOURCE,
    DFILE_TORCH1_ENABLE,
    DFILE_TORCH1_ONOFF,
    DFILE_MAX
};


struct lm3644_platform_data {
    u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
    u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
    u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
    u8 strobe_pin_disable;  // 1 : STROBE Input disabled
    u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct lm3644 {
    struct device *dev;

    u8 brightness[ID_MAX];
    struct work_struct work[ID_MAX];
    struct led_classdev cdev[ID_MAX];

    struct lm3644_platform_data *pdata;
    struct regmap *regmap;
    struct mutex lock;
};

static void lm3644_read_flag(struct lm3644 *pchip)
{

    int rval;
    unsigned int flag0, flag1;

    rval = regmap_read(pchip->regmap, REG_FLAG0, &flag0);
    rval |= regmap_read(pchip->regmap, REG_FLAG1, &flag1);

    if (rval < 0)
        dev_err(pchip->dev, "i2c access fail.\n");

    dev_info(pchip->dev, "[flag1] 0x%x, [flag0] 0x%x\n",
         flag1 & 0x1f, flag0);
}

/* torch0 brightness control */
static void lm3644_deferred_torch0_brightness_set(struct work_struct *work)
{
    struct lm3644 *pchip = container_of(work,
                        struct lm3644, work[ID_TORCH0]);

    if (regmap_update_bits(pchip->regmap,
                   REG_TORCH_LED0_BR, 0x7f,
                   pchip->brightness[ID_TORCH0]))
        dev_err(pchip->dev, "i2c access fail.\n");
    lm3644_read_flag(pchip);
}

static void lm3644_torch0_brightness_set(struct led_classdev *cdev,
                     enum led_brightness brightness)
{
    struct lm3644 *pchip =
        container_of(cdev, struct lm3644, cdev[ID_TORCH0]);

    pchip->brightness[ID_TORCH0] = brightness;
    schedule_work(&pchip->work[ID_TORCH0]);
}

/* torch1 brightness control */
static void lm3644_deferred_torch1_brightness_set(struct work_struct *work)
{
    struct lm3644 *pchip = container_of(work,
                        struct lm3644, work[ID_TORCH1]);

    if (regmap_update_bits(pchip->regmap,
                   REG_TORCH_LED1_BR, 0x7f,
                   pchip->brightness[ID_TORCH1]))
        dev_err(pchip->dev, "i2c access fail.\n");
    lm3644_read_flag(pchip);
}

static void lm3644_torch1_brightness_set(struct led_classdev *cdev,
                     enum led_brightness brightness)
{
    struct lm3644 *pchip =
        container_of(cdev, struct lm3644, cdev[ID_TORCH1]);

    pchip->brightness[ID_TORCH1] = brightness;
    schedule_work(&pchip->work[ID_TORCH1]);
}

/* flash0 brightness control */
static void lm3644_deferred_flash0_brightness_set(struct work_struct *work)
{
    struct lm3644 *pchip = container_of(work,
                        struct lm3644, work[ID_FLASH0]);

    if (regmap_update_bits(pchip->regmap,
                   REG_FLASH_LED0_BR, 0x7f,
                   pchip->brightness[ID_FLASH0]))
        dev_err(pchip->dev, "i2c access fail.\n");
    lm3644_read_flag(pchip);
}

static void lm3644_flash0_brightness_set(struct led_classdev *cdev,
                     enum led_brightness brightness)
{
    struct lm3644 *pchip =
        container_of(cdev, struct lm3644, cdev[ID_FLASH0]);

    pchip->brightness[ID_FLASH0] = brightness;
    schedule_work(&pchip->work[ID_FLASH0]);
}

/* flash1 brightness control */
static void lm3644_deferred_flash1_brightness_set(struct work_struct *work)
{
    struct lm3644 *pchip = container_of(work,
                        struct lm3644, work[ID_FLASH1]);

    if (regmap_update_bits(pchip->regmap,
                   REG_FLASH_LED1_BR, 0x7f,
                   pchip->brightness[ID_FLASH1]))
        dev_err(pchip->dev, "i2c access fail.\n");
    lm3644_read_flag(pchip);
}

static void lm3644_flash1_brightness_set(struct led_classdev *cdev,
                     enum led_brightness brightness)
{
    struct lm3644 *pchip =
        container_of(cdev, struct lm3644, cdev[ID_FLASH1]);

    pchip->brightness[ID_FLASH1] = brightness;
    schedule_work(&pchip->work[ID_FLASH1]);
}

struct lm3644_devices {
    struct led_classdev cdev;
    work_func_t func;
};

static struct lm3644_devices lm3644_leds[ID_MAX] = {
    [ID_FLASH0] = {
               .cdev.name = "flash0",
               .cdev.brightness = 0,
               .cdev.max_brightness = 0x7f,
               .cdev.brightness_set = lm3644_flash0_brightness_set,
               .cdev.default_trigger = "flash0",
               .func = lm3644_deferred_flash0_brightness_set},
    [ID_FLASH1] = {
               .cdev.name = "flash1",
               .cdev.brightness = 0,
               .cdev.max_brightness = 0x7f,
               .cdev.brightness_set = lm3644_flash1_brightness_set,
               .cdev.default_trigger = "flash1",
               .func = lm3644_deferred_flash1_brightness_set},
    [ID_TORCH0] = {
               .cdev.name = "torch0",
               .cdev.brightness = 0,
               .cdev.max_brightness = 0x7f,
               .cdev.brightness_set = lm3644_torch0_brightness_set,
               .cdev.default_trigger = "torch0",
               .func = lm3644_deferred_torch0_brightness_set},
    [ID_TORCH1] = {
               .cdev.name = "torch1",
               .cdev.brightness = 0,
               .cdev.max_brightness = 0x7f,
               .cdev.brightness_set = lm3644_torch1_brightness_set,
               .cdev.default_trigger = "torch1",
               .func = lm3644_deferred_torch1_brightness_set},
};

static void lm3644_led_unregister(struct lm3644 *pchip, enum lm3644_devid id)
{
    int icnt;

    for (icnt = id; icnt > 0; icnt--)
        led_classdev_unregister(&pchip->cdev[icnt - 1]);
}

static int lm3644_led_register(struct lm3644 *pchip)
{
    int icnt, rval;

    for (icnt = 0; icnt < ID_MAX; icnt++) {
        INIT_WORK(&pchip->work[icnt], lm3644_leds[icnt].func);
        pchip->cdev[icnt].name = lm3644_leds[icnt].cdev.name;
        pchip->cdev[icnt].max_brightness =
            lm3644_leds[icnt].cdev.max_brightness;
        pchip->cdev[icnt].brightness =
            lm3644_leds[icnt].cdev.brightness;
        pchip->cdev[icnt].brightness_set =
            lm3644_leds[icnt].cdev.brightness_set;
        pchip->cdev[icnt].default_trigger =
            lm3644_leds[icnt].cdev.default_trigger;
        rval = led_classdev_register((struct device *)
                         pchip->dev, &pchip->cdev[icnt]);
        if (rval < 0) {
            lm3644_led_unregister(pchip, icnt);
            return rval;
        }
    }
    return 0;
}

/* device files to control registers */
struct lm3644_commands {
    char *str;
    int size;
};

enum lm3644_cmd_id {
    CMD_ENABLE = 0,
    CMD_DISABLE,
    CMD_ON,
    CMD_OFF,
    CMD_IRMODE,
    CMD_OVERRIDE,
    CMD_MAX
};

struct lm3644_commands cmds[CMD_MAX] = {
    [CMD_ENABLE] = {"enable", 6},
    [CMD_DISABLE] = {"disable", 7},
    [CMD_ON] = {"on", 2},
    [CMD_OFF] = {"off", 3},
    [CMD_IRMODE] = {"irmode", 6},
    [CMD_OVERRIDE] = {"override", 8},
};

struct lm3644_files {
    enum lm3644_devid id;
    struct device_attribute attr;
};

static size_t lm3644_ctrl(struct device *dev,
              const char *buf, enum lm3644_devid id,
              enum lm3644_devfile dfid, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct lm3644 *pchip = to_lm3644(led_cdev, id);
    enum lm3644_cmd_id icnt;
    int tout, rval;

    mutex_lock(&pchip->lock);
    for (icnt = 0; icnt < CMD_MAX; icnt++) {
        if (strncmp(buf, cmds[icnt].str, cmds[icnt].size) == 0)
            break;
    }

    switch (dfid) {
        /* led 0 enable */
    case DFILE_FLASH0_ENABLE:
    case DFILE_TORCH0_ENABLE:
        if (icnt == CMD_ENABLE)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x1,
                           0x1);
        else if (icnt == CMD_DISABLE)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x1,
                           0x0);
        break;
        /* led 1 enable, flash override */
    case DFILE_FLASH1_ENABLE:
        if (icnt == CMD_ENABLE) {
            rval = regmap_update_bits(pchip->regmap,
                          REG_FLASH_LED0_BR, 0x80, 0x0);
            rval |=
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x2,
                           0x2);
        } else if (icnt == CMD_DISABLE) {
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x2,
                           0x0);
        } else if (icnt == CMD_OVERRIDE) {
            rval = regmap_update_bits(pchip->regmap,
                          REG_FLASH_LED0_BR, 0x80,
                          0x80);
            rval |=
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x2,
                           0x2);
        }
        break;
        /* led 1 enable, torch override */
    case DFILE_TORCH1_ENABLE:
        if (icnt == CMD_ENABLE) {
            rval = regmap_update_bits(pchip->regmap,
                          REG_TORCH_LED0_BR, 0x80, 0x0);
            rval |=
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x2,
                           0x2);
        } else if (icnt == CMD_DISABLE) {
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x2,
                           0x0);
        } else if (icnt == CMD_OVERRIDE) {
            rval = regmap_update_bits(pchip->regmap,
                          REG_TORCH_LED0_BR, 0x80,
                          0x80);
            rval |=
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x2,
                           0x2);
        }
        break;
        /* mode control flash/ir */
    case DFILE_FLASH0_ONOFF:
    case DFILE_FLASH1_ONOFF:
        if (icnt == CMD_ON)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0xc,
                           0xc);
        else if (icnt == CMD_OFF)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0xc,
                           0x0);
        else if (icnt == CMD_IRMODE)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0xc,
                           0x4);
        break;
        /* mode control torch */
    case DFILE_TORCH0_ONOFF:
    case DFILE_TORCH1_ONOFF:
        if (icnt == CMD_ON)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0xc,
                           0x8);
        else if (icnt == CMD_OFF)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0xc,
                           0x0);
        break;
        /* strobe pin control */
    case DFILE_FLASH0_SOURCE:
        if (icnt == CMD_ON)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x20,
                           0x20);
        else if (icnt == CMD_OFF)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x20,
                           0x0);
        break;
    case DFILE_TORCH0_SOURCE:
        if (icnt == CMD_ON)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x10,
                           0x10);
        else if (icnt == CMD_OFF)
            rval =
                regmap_update_bits(pchip->regmap, REG_ENABLE, 0x10,
                           0x0);
        break;
        /* flash time out */
    case DFILE_FLASH0_TIMEOUT:
        rval = kstrtouint((const char *)buf, 10, &tout);
        if (rval < 0)
            break;
        rval = regmap_update_bits(pchip->regmap,
                      REG_FLASH_TOUT, 0x0f, tout);
        break;
    default:
        dev_err(pchip->dev, "error : undefined dev file\n");
        break;
    }
    lm3644_read_flag(pchip);
    mutex_unlock(&pchip->lock);
    return size;
}

/* flash enable control */
static ssize_t lm3644_flash0_enable_store(struct device *dev,
                      struct device_attribute *devAttr,
                      const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_FLASH0, DFILE_FLASH0_ENABLE, size);
}

static ssize_t lm3644_flash1_enable_store(struct device *dev,
                      struct device_attribute *devAttr,
                      const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_FLASH1, DFILE_FLASH1_ENABLE, size);
}

/* flash onoff control */
static ssize_t lm3644_flash0_onoff_store(struct device *dev,
                     struct device_attribute *devAttr,
                     const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_FLASH0, DFILE_FLASH0_ONOFF, size);
}

static ssize_t lm3644_flash1_onoff_store(struct device *dev,
                     struct device_attribute *devAttr,
                     const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_FLASH1, DFILE_FLASH1_ONOFF, size);
}

/* flash timeout control */
static ssize_t lm3644_flash0_timeout_store(struct device *dev,
                       struct device_attribute *devAttr,
                       const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_FLASH0, DFILE_FLASH0_TIMEOUT, size);
}

/* flash source control */
static ssize_t lm3644_flash0_source_store(struct device *dev,
                      struct device_attribute *devAttr,
                      const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_FLASH0, DFILE_FLASH0_SOURCE, size);
}

/* torch enable control */
static ssize_t lm3644_torch0_enable_store(struct device *dev,
                      struct device_attribute *devAttr,
                      const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_FLASH0, DFILE_TORCH0_ENABLE, size);
}

static ssize_t lm3644_torch1_enable_store(struct device *dev,
                      struct device_attribute *devAttr,
                      const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_TORCH1, DFILE_TORCH1_ENABLE, size);
}

/* torch onoff control */
static ssize_t lm3644_torch0_onoff_store(struct device *dev,
                     struct device_attribute *devAttr,
                     const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_TORCH0, DFILE_TORCH0_ONOFF, size);
}

static ssize_t lm3644_torch1_onoff_store(struct device *dev,
                     struct device_attribute *devAttr,
                     const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_TORCH1, DFILE_TORCH1_ONOFF, size);
}

/* torch source control */
static ssize_t lm3644_torch0_source_store(struct device *dev,
                      struct device_attribute *devAttr,
                      const char *buf, size_t size)
{
    return lm3644_ctrl(dev, buf, ID_TORCH0, DFILE_TORCH0_SOURCE, size);
}

#define lm3644_attr(_name, _show, _store)\
{\
    .attr = {\
        .name = _name,\
        .mode = 0644,\
    },\
    .show = _show,\
    .store = _store,\
}

static struct lm3644_files lm3644_devfiles[DFILE_MAX] = {
    [DFILE_FLASH0_ENABLE] = {
                 .id = ID_FLASH0,
                 .attr =
                 lm3644_attr("enable", NULL,
                         lm3644_flash0_enable_store),
                 },
    [DFILE_FLASH0_ONOFF] = {
                .id = ID_FLASH0,
                .attr =
                lm3644_attr("onoff", NULL,
                        lm3644_flash0_onoff_store),
                },
    [DFILE_FLASH0_SOURCE] = {
                 .id = ID_FLASH0,
                 .attr =
                 lm3644_attr("source", NULL,
                         lm3644_flash0_source_store),
                 },
    [DFILE_FLASH0_TIMEOUT] = {
                  .id = ID_FLASH0,
                  .attr =
                  lm3644_attr("timeout", NULL,
                          lm3644_flash0_timeout_store),
                  },
    [DFILE_FLASH1_ENABLE] = {
                 .id = ID_FLASH1,
                 .attr =
                 lm3644_attr("enable", NULL,
                         lm3644_flash1_enable_store),
                 },
    [DFILE_FLASH1_ONOFF] = {
                .id = ID_FLASH1,
                .attr =
                lm3644_attr("onoff", NULL,
                        lm3644_flash1_onoff_store),
                },
    [DFILE_TORCH0_ENABLE] = {
                 .id = ID_TORCH0,
                 .attr =
                 lm3644_attr("enable", NULL,
                         lm3644_torch0_enable_store),
                 },
    [DFILE_TORCH0_ONOFF] = {
                .id = ID_TORCH0,
                .attr =
                lm3644_attr("onoff", NULL,
                        lm3644_torch0_onoff_store),
                },
    [DFILE_TORCH0_SOURCE] = {
                 .id = ID_TORCH0,
                 .attr =
                 lm3644_attr("source", NULL,
                         lm3644_torch0_source_store),
                 },
    [DFILE_TORCH1_ENABLE] = {
                 .id = ID_TORCH1,
                 .attr =
                 lm3644_attr("enable", NULL,
                         lm3644_torch1_enable_store),
                 },
    [DFILE_TORCH1_ONOFF] = {
                .id = ID_TORCH1,
                .attr =
                lm3644_attr("onoff", NULL,
                        lm3644_torch1_onoff_store),
                }
};

static void lm3644_df_remove(struct lm3644 *pchip, enum lm3644_devfile dfid)
{
    enum lm3644_devfile icnt;

    for (icnt = dfid; icnt > 0; icnt--)
        device_remove_file(pchip->cdev[lm3644_devfiles[icnt - 1].id].
                   dev, &lm3644_devfiles[icnt - 1].attr);
}

static int lm3644_df_create(struct lm3644 *pchip)
{
    enum lm3644_devfile icnt;
    int rval;

    for (icnt = 0; icnt < DFILE_MAX; icnt++) {
        rval =
            device_create_file(pchip->cdev[lm3644_devfiles[icnt].id].
                       dev, &lm3644_devfiles[icnt].attr);
        if (rval < 0) {
            lm3644_df_remove(pchip, icnt);
            return rval;
        }
    }
    return 0;
}

static const struct regmap_config lm3644_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xff,
};

static int lm3644_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct lm3644 *pchip;
    int rval;
    struct lm3644_platform_data *pdata = client->dev.platform_data;

    /* i2c check */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "i2c functionality check fail.\n");
        return -EOPNOTSUPP;
    }

    pchip = devm_kzalloc(&client->dev, sizeof(struct lm3644), GFP_KERNEL);
    if (!pchip)
        return -ENOMEM;

    /////
    if (pdata == NULL) 
    {
        dev_err(&client->dev, "[LM3644] needs Platform Data.\n");
        return -ENODATA;
    }

    mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN, GPIO_MODE_00); //gpio_mode
    mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, GPIO_OUT_ONE);

    mt_set_gpio_mode(GPIO95 | 0x80000000, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO95 | 0x80000000, GPIO_DIR_OUT);

    mt_set_gpio_mode(GPIO96 | 0x80000000, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO96 | 0x80000000, GPIO_DIR_OUT);
    pchip->pdata  = pdata;
    lm3644_i2c_client = client;

    int reg_val = 0;
    //reg_val = read_data_lm3643(chip->client, LM3643_REG_DEVICE_ID);
    reg_val = readReg(REG_ID);
    if ((reg_val & 0b1111) == 0x8)
        g_is_ktd2684 = 1;
    else
        g_is_ktd2684 = 0;
    printk("[LM3644] g_is_ktd2684 %x\n", reg_val);
    
////

    pchip->dev = &client->dev;
    pchip->regmap = devm_regmap_init_i2c(client, &lm3644_regmap);
    if (IS_ERR(pchip->regmap)) {
        rval = PTR_ERR(pchip->regmap);
        dev_err(&client->dev, "Failed to allocate register map: %d\n",
            rval);
        return rval;
    }
    mutex_init(&pchip->lock);
    i2c_set_clientdata(client, pchip);

    // /* led class register */
    // rval = lm3644_led_register(pchip);
    // if (rval < 0)
    //     return rval;

    // //create dev files 
    // rval = lm3644_df_create(pchip);
    // if (rval < 0) {
    //     lm3644_led_unregister(pchip, ID_MAX);
    //     return rval;
    // }

    dev_info(pchip->dev, "lm3644 leds initialized\n");
    return 0;
}

static int lm3644_remove(struct i2c_client *client)
{
    struct lm3644 *pchip = i2c_get_clientdata(client);

    lm3644_df_remove(pchip, DFILE_MAX);
    lm3644_led_unregister(pchip, ID_MAX);

    return 0;
}

struct lm3644_platform_data lm3644_pdata = {0, 0, 0, 0, 0};

static struct i2c_board_info __initdata i2c_lm3644={ I2C_BOARD_INFO(LM3644_NAME, LM3644_ADDR), \
                                                    .platform_data = &lm3644_pdata,};
static const struct i2c_device_id lm3644_id[] = {
    {LM3644_NAME, 0},
    {}
};
//MODULE_DEVICE_TABLE(i2c, lm3644_id);

static struct i2c_driver lm3644_i2c_driver = {
    .driver = {
           .name = LM3644_NAME,
           //.owner = THIS_MODULE,
           //.pm = NULL,
           },
    .probe = lm3644_probe,
    .remove = lm3644_remove,
    .id_table = lm3644_id,
};

static int __init lm3644_init(void)
{
    i2c_register_board_info(I2C_BUSNUM, &i2c_lm3644, 1);

    if (i2c_add_driver(&lm3644_i2c_driver))
    {
        printk("[LM3644] Failed to register lm3644_i2c_driver\n");
        return -1;
    }
    printk("[LM3644] LM3644_init done\n");
    return 0;
}

static void __exit lm3644_exit(void)
{
    i2c_del_driver(&lm3644_i2c_driver);
}
//pure_initcall(lm3644_init);
module_init(lm3644_init);
module_exit(lm3644_exit);

//module_i2c_driver(lm3644_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments Flash Lighting driver for LM3644");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");


