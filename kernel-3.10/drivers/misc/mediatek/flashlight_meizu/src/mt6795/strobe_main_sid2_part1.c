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
#include "kd_flashlight_type.h"
#include "meizu_flash.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>

#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define TAG_NAME "[strobe_main_sid2_part1.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

extern bool led1_onoff;
extern bool torch_flag;
extern int led1_duty;

int led2_duty;

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */
static struct work_struct workTimeOut;
static int g_timeOutTimeMs;
static u32 strobe_Res;
static int flash_enable2 = 0;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);
extern int FL_Enable_led1(void);
extern int FL_dim_duty_led1(kal_uint32 duty);
extern int FL_Disable_led1(void);
extern void Enable_GPIO(void);
extern void Disable_GPIO(void);
extern int FL_current_timeout_set(void);


static int FL_Enable(void)
{
       FL_Enable_led1();
       return 0;
}

static int FL_Disable(void)
{

       FL_Disable_led1();
       return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
       FL_dim_duty_led1(duty);
       return 0;
}


static int FL_Init(void)
{
	PK_DBG(" FL_Init line=%d\n", __LINE__);
	Enable_GPIO();
	FL_current_timeout_set();
	return 0;
}

static int FL_Uninit(void)
{
	FL_Disable();
	flash_enable2 = 0;
	Disable_GPIO();
	return 0;
}

static int FL_hasLowPowerDetect(void)
{

	return 1;
}

static int detLowPowerStart(void)
{
	return 0;
}


static int detLowPowerEnd(void)
{
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	flash_enable2 = 0;
	PK_DBG("ledTimeOut_callback\n");
}

static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static void timerInit(void)
{
	/* ktime_t ktime; */


	/* mt6333_set_rg_chrwdt_en(0); */

	/* mt6333_set_rg_chrwdt_td(0); //4 sec */
	/* mt6333_set_rg_chrwdt_en(1); */

	/* mt6333_set_rg_chrwdt_en(0); */


	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	/* kal_uint8 valTemp; */
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASH_IOC_DUTY: %d\n", (int)arg);
	
		FL_dim_duty(arg);
		break;

	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASH_IOC_ONOFF: %d\n", (int)arg);
    		if(arg==1 && flash_enable2 == 0) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
			flash_enable2 = 1;
    		} else if (arg == 0 && flash_enable2 == 1) {
			FL_Disable();
			flash_enable2 = 0;
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;

	/*This CMD is for flash device file control*/
	case FLASH_IOC_SET_TORCH_DEV :
    	PK_DBG("FLASH_IOC_SET_TORCH_DEV: %d\n",(int)arg);
		break;

	case FLASH_IOC_SET_REG_ADR:
		PK_DBG("FLASH_IOC_SET_REG_ADR: %d\n", (int)arg);
		/* g_reg = arg; */
		break;
	case FLASH_IOC_SET_REG_VAL:
		PK_DBG("FLASH_IOC_SET_REG_VAL: %d\n", (int)arg);
		/* g_val = arg; */
		break;
	case FLASH_IOC_SET_REG:
		/* PK_DBG("FLASH_IOC_SET_REG: %d %d\n",g_reg, g_val); */

		break;

	case FLASH_IOC_GET_REG:
		PK_DBG("FLASH_IOC_GET_REG: %d\n", (int)arg);

		/* i4RetValue = valTemp; */
		/* PK_DBG("FLASH_IOC_GET_REG: v=%d\n",valTemp); */
		break;

	case FLASH_IOC_HAS_LOW_POWER_DETECT:
		PK_DBG("FLASH_IOC_HAS_LOW_POWER_DETECT");
		temp = FL_hasLowPowerDetect();
		if (copy_to_user((void __user *)arg, (void *)&temp, 4)) {
			PK_DBG(" ioctl copy to user failed\n");
			return -1;
		}
		break;
	case FLASH_IOC_LOW_POWER_DETECT_START:
		detLowPowerStart();
		break;
	case FLASH_IOC_LOW_POWER_DETECT_END:
		i4RetValue = detLowPowerEnd();
		break;

	default:
		PK_DBG(" No such command\n");
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


		spin_unlock_irq(&g_strobeSMPLock);
		FL_Uninit();
	}
	PK_DBG(" Done\n");
	return 0;
}


static FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 strobeInit_main_sid2_part1(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
