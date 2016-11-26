/****************************************
 *Function: 
 *    leds_control.c
 *Decription:
 *    For control 5 LM3643 IC interface
 *    M80 Loop Flashlight
 *Author:
 *    lcz@meizu.com
 * **************************************/
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
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_flashlight_type.h"
#include "meizu_flash.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/

#define TAG_NAME "[leds_control.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
#else
	#define PK_DBG(a, ...)
#endif

extern bool torch_flag1;
extern bool led1_onoff;
extern int  led1_duty;

extern int new_hw_version;

/*Global variables*/
bool led1_onoff = false;
bool torch_flag = true;
int  led1_duty = 0;

/******************************************************************************
 * local variables
******************************************************************************/

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

static struct work_struct workTimeOut;

//========leds control function=================//

int Flash_Setduty(unsigned char led_num,int duty,bool mode){
	/*control 5 cool or warm leds at the same time*/
    if(new_hw_version){
	Flash_Setduty_IC1(led_num,duty,mode);
	Flash_Setduty_IC2(led_num,duty,mode);
	Flash_Setduty_IC3(led_num,duty,mode);
	Flash_Setduty_IC4(led_num,duty,mode);
    }
	Flash_Setduty_IC5(led_num,duty,mode);

	return 0;
}

int Flash_Enable(unsigned char led_num,bool mode){
	/*control 5 cool or warm leds at the same time*/
    if(new_hw_version){
	Flash_Enable_IC1(led_num,mode);
	Flash_Enable_IC2(led_num,mode);
	Flash_Enable_IC3(led_num,mode);
	Flash_Enable_IC4(led_num,mode);
    }
	Flash_Enable_IC5(led_num,mode);
	
	return 0;
}

int Flash_Disable(unsigned char led_num){
	/*control 5 cool or warm leds at the same time*/
    if(new_hw_version){
	Flash_Disable_IC1(led_num);
	Flash_Disable_IC2(led_num);
	Flash_Disable_IC3(led_num);
	Flash_Disable_IC4(led_num);
    }
	Flash_Disable_IC5(led_num);

	return 0;
}

//========function end===========================//

int FL_Init(void)
{
    //Enable_GPIO();
    //LM3644_write_reg(LM3644_i2c_client,REG_FLASH_TOUT, 0x0f);

    printk("[LM3644] init done, FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
    //Flash_Disable(COOL_LED);
    led1_onoff = false;
    
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    Flash_Disable(COOL_LED);
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
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
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	int led_ic,led_num,factory_arg;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	//PK_DBG("LM3644 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    
	switch(cmd){

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
				
			g_timeOutTimeMs=arg ;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
		
		led1_duty = (int)arg;
	
		if(led1_duty < TORCH_NUM){
				torch_flag = true;
			}
			else{
				torch_flag = false;
			}
		//Flash_Setduty(COOL_LED,arg,torch_flag);
    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1){
			
			if(g_timeOutTimeMs!=0){
	            		ktime_t ktime;
				ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
				hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
			led1_onoff = true;
    			//Flash_Enable(COOL_LED,torch_flag);
    		}
    		else{
			led1_onoff = false;
    			//Flash_Disable(COOL_LED);
			hrtimer_cancel( &g_timeOutTimer );
    		}
    	
		break;
    	
	/*This CMD is for flash device file control*/
	case FLASH_IOC_SET_TORCH_DEV :
    		PK_DBG("FLASH_SET_TORCH_DEV: %d\n",(int)arg);
		temp = (int)arg;
		if(temp >= 0){
			Flash_Setduty(COOL_LED,arg,TORCH_MODE);
    			Flash_Enable(COOL_LED,TORCH_MODE);
		}
		else{
    			Flash_Disable(COOL_LED);
		}

		break;
    
	/*This CMD is for factory test M80 loop flashlight*/
	case FLASH_IOC_SET_FACTORY_TEST:
    
		PK_DBG("FLASH_IOC_SET_FACTORY_TEST: %d\n",(int)arg);
	
		factory_arg = (int)arg;

		/*light up one led at each times*/
		if(factory_arg > 0){
		
			led_ic = (factory_arg+1)/2;
			led_num  = (factory_arg+1)%2+1;

	            switch(led_ic){
			case 1:
				Flash_Setduty_IC1(led_num,FACTORY_TEST_DUTY,TORCH_MODE);
				Flash_Enable_IC1(led_num,TORCH_MODE);
				break;
			case 2:
				Flash_Setduty_IC2(led_num,FACTORY_TEST_DUTY,TORCH_MODE);
				Flash_Enable_IC2(led_num,TORCH_MODE);
				break;
			case 3:
				Flash_Setduty_IC3(led_num,FACTORY_TEST_DUTY,TORCH_MODE);
				Flash_Enable_IC3(led_num,TORCH_MODE);
				break;
			case 4:
				Flash_Setduty_IC4(led_num,FACTORY_TEST_DUTY,TORCH_MODE);
				Flash_Enable_IC4(led_num,TORCH_MODE);
				break;
			case 5:
				Flash_Setduty_IC5(led_num,FACTORY_TEST_DUTY,TORCH_MODE);
				Flash_Enable_IC5(led_num,TORCH_MODE);
				break;
			default:
				break;
		    }
		}
	
		/*light down all led*/
		else{
#if 0
			led_ic = (-factory_arg+1)/2;
			led_num  = (-factory_arg+1)%2+1;

		     switch(led_ic){
			case 1:
				Flash_Disable_IC1(led_num);
				break;
			case 2:
				Flash_Disable_IC2(led_num);
				break;
			case 3:
				Flash_Disable_IC3(led_num);
				break;
			case 4:
				Flash_Disable_IC4(led_num);
				break;
			case 5:
				Flash_Disable_IC5(led_num);
				break;
			default:
				break;
		     }
#else
			led_num  = 0;
			Flash_Disable_IC1(led_num);
			Flash_Disable_IC2(led_num);
			Flash_Disable_IC3(led_num);
			Flash_Disable_IC4(led_num);
			Flash_Disable_IC5(led_num);
#endif
		}

    		break;

	case FLASH_IOC_SET_STEP:
    
		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

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
