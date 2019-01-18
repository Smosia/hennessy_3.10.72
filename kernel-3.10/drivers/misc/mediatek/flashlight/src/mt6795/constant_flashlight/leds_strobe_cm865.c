#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
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
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */
extern void flashlight_clear_brightness(void);

static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;


static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0x63


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF GPIO12
//#define FLASH_GPIO_ENT GPIO13
#define GPIO_LED_EN  GPIO_CAMERA_FLASH_EXT1_PIN


#define LM3643_REG_ENABLE      0x01
#define LM3643_REG_LED1_FLASH  0x03
#define LM3643_REG_LED2_FLASH  0x04
#define LM3643_REG_LED1_TORCH  0x05
#define LM3643_REG_LED2_TORCH  0x06
#define LM3643_REG_TIMING	   0x08		

struct i2c_client *g_lm3643_i2c_client = NULL;
EXPORT_SYMBOL(g_lm3643_i2c_client);

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);
extern kal_uint16 read_data_lm3643(struct i2c_client *client, kal_uint8 addr);
extern kal_uint16 write_data_lm3643(struct i2c_client *client, kal_uint8 addr, kal_uint8 data);
int readReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    //iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    bufR[0] = read_data_lm3643(g_lm3643_i2c_client, (char)reg);
    printk("qq reg=%x val=%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
}

int writeReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
    printk("<%s:%d>reg[%d][%d]\n", __FUNCTION__, __LINE__, reg, data);
	write_data_lm3643(g_lm3643_i2c_client, buf[0], buf[1]);
    //iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);

   return 0;
}

#define e_DutyNum 26
#define TORCHDUTYNUM 4
static int isMovieMode[e_DutyNum] = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static int torchLEDReg[e_DutyNum] = {35,71,106,127,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 35 17-->25ma

static int torchLEDReg_ktd1[e_DutyNum] = {16,33,50,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 35 17-->25ma
static int torchLEDReg_ktd2[e_DutyNum] = {8,16,25,30,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 35 17-->25ma
//50,100,150,179ma
static int flashLEDReg[e_DutyNum] = {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110,118,127};
//200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500ma
static int flashLEDReg_ktd[e_DutyNum] = {2,5,9,10,12,15,18,21,25,28,31,34,37,41,44,47,50,54,57,60,63,63,63,63,63,63};
static int *flashLEDReg_led2=NULL;
static int *torchLEDReg_led2=NULL;
extern int g_is_ktd2684;


int m_duty1=0;
int m_duty2=0;
int LED1Closeflag = 0;
int LED2Closeflag = 0;


int flashEnable_LM3643_1(void)
{
	int temp;
	return 0;
}
int flashDisable_LM3643_1(void)
{
	int temp;
    return 0;
}

int setDuty_LM3643_1(int duty)
{
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	
	return 0;
}

int flashEnable_LM3643_2(void)
{
	int temp;

	PK_DBG("flashEnable_LM3643_2\n");
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	temp = readReg(LM3643_REG_ENABLE);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		writeReg(LM3643_REG_ENABLE, temp & 0xF0);//close		
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFA);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFE);//flash mode
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xF9);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFD);//flash mode		
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) & (isMovieMode[m_duty2] == 1))
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFB);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFF);//flash mode
	}
	return 0;

}
int flashDisable_LM3643_2(void)
{
	flashEnable_LM3643_2();
	return 0;
}


int setDuty_LM3643_2(int duty)
{
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty2=duty;

	PK_DBG("setDuty_LM3643_2:m_duty = %d, m_duty2 = %d!\n", m_duty1, m_duty2);
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
		{
			if (g_is_ktd2684 == 1)
			{
				writeReg(LM3643_REG_LED2_TORCH, torchLEDReg_ktd2[m_duty2]);
			}
			else
			{
				writeReg(LM3643_REG_LED2_TORCH, torchLEDReg[m_duty2]);
			}
		}
		else
		{
			if (flashLEDReg_led2 == NULL)
			{
				if (g_is_ktd2684 == 1)
					flashLEDReg_led2 = flashLEDReg_ktd;
				else
					flashLEDReg_led2 = flashLEDReg;
			}
	 	    writeReg(LM3643_REG_LED2_FLASH, flashLEDReg_led2[m_duty2]);
		}
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
		{
			if (g_is_ktd2684 == 1)
			{
				writeReg(LM3643_REG_LED1_TORCH, torchLEDReg_ktd1[m_duty1]);
			}
			else
			{
				writeReg(LM3643_REG_LED1_TORCH, torchLEDReg[m_duty1]);
			}
		}
		else
		{
			writeReg(LM3643_REG_LED1_FLASH, flashLEDReg[m_duty1]);	
		}		
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) && ((isMovieMode[m_duty2] == 1)))
		{
			if (g_is_ktd2684 == 1)
			{
				writeReg(LM3643_REG_LED1_TORCH, torchLEDReg_ktd1[m_duty1]);
				writeReg(LM3643_REG_LED2_TORCH, torchLEDReg_ktd2[m_duty2]);
			}
			else
			{
				writeReg(LM3643_REG_LED1_TORCH, torchLEDReg[m_duty1]);
				writeReg(LM3643_REG_LED2_TORCH, torchLEDReg[m_duty2]);
			}
		}
		else
		{
			if (flashLEDReg_led2 == NULL)
			{
				if (g_is_ktd2684 == 1)
					flashLEDReg_led2 = flashLEDReg_ktd;
				else
					flashLEDReg_led2 = flashLEDReg;
			}
	 	    writeReg(LM3643_REG_LED1_FLASH, flashLEDReg[m_duty1]);
			writeReg(LM3643_REG_LED2_FLASH, flashLEDReg_led2[m_duty2]);
		}
	}
	return 0;
}


int FL_Enable(void)
{
	PK_DBG(" FL_Enable line=%d\n",__LINE__);
    return 0;
}

int FL_Disable(void)
{
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    setDuty_LM3643_1(duty);

    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}

int FL_Init(void)
{
	PK_DBG("LED1_FL_Init!\n");
    if(mt_set_gpio_mode(GPIO_LED_EN,GPIO_MODE_00)){PK_DBG(" set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_LED_EN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ONE)){PK_DBG(" set gpio failed!! \n");}
    if (g_is_ktd2684 == 1)
    	writeReg(LM3643_REG_TIMING, 0x1A);
    else
		writeReg(LM3643_REG_TIMING, 0x1F);

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}

int FL_Uninit(void)
{
	PK_DBG("LED1_FL_Uninit!\n");
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("LED1TimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
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
	PK_DBG("LM3643_LED1_constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;
    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
			m_duty1 = arg;
    		break;
    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);
    		break;
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
				LED1Closeflag = 0;
    			FL_Enable();
    		}
    		else
    		{
    			LED1Closeflag = 1;
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

	if (0 == strobe_Res)
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
        strobe_Res += 1;
    }

    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("<%s:%d>g_is_ktd2684[%d]\n", __func__, __LINE__, g_is_ktd2684);
    if (g_is_ktd2684 == 1)
    	flashLEDReg_led2 = flashLEDReg_ktd;
    else
    	flashLEDReg_led2 = flashLEDReg;
    PK_DBG("constant_flashlight_open line=%d g_is_ktd2684[%d]\n", __LINE__, g_is_ktd2684);

    return i4RetValue;
}


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
    	FL_Uninit();
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

void flashlight_onoff(int level)
{
	if (strobe_Res)
    {
        flashlight_clear_brightness();
        return;
    }

    printk("<%s:%d>level[%x]\n", __func__, __LINE__, level);
    if (level == 100)
    	level = 0x31;
    if (level == 0x33)
        level = 0x31;

    FL_Init();
    if (level & 0xf0)
    	LED1Closeflag = 0;
    else
    	LED1Closeflag = 1;
    
    if (level & 0xf)
    	LED2Closeflag = 0;
    else
    	LED2Closeflag = 1;
    setDuty_LM3643_1((level & 0xf0) >> 4);
    setDuty_LM3643_2((level & 0xf));
    mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, 1);
	mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 1);
    flashEnable_LM3643_2();
    return;
}

EXPORT_SYMBOL(flashlight_onoff);
