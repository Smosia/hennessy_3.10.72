/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <mach/upmu_hw.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/irqs.h>
#include <mach/mtk_rtc_hal.h>
#include <mach/mtk_rtc_hal_common.h>
#include "mach/mt_rtc_hw.h"
#include <mach/mt_typedefs.h>
#include <mach/mt_pmic_wrap.h>
#if defined CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
#include <mach/system.h>
#include <mach/mt_boot.h>
#endif
#include <rtc-mt.h>		/* custom file */

#include <mach/mt_gpio.h>


/* we map HW YEA 0 (2000) to 1968 not 1970 because 2000 is the leap year */
#define RTC_MIN_YEAR		1968
#define RTC_NUM_YEARS		128
/* #define RTC_MAX_YEAR          (RTC_MIN_YEAR + RTC_NUM_YEARS - 1) */


#define hal_rtc_xinfo(fmt, args...)		\
	pr_notice(fmt, ##args)

#define hal_rtc_xerror(fmt, args...)	\
	pr_err(fmt, ##args)

#define hal_rtc_xfatal(fmt, args...)	\
	pr_emerg(fmt, ##args)

/* Causion, for SRCLKENA drop speed too slow (align VIO18) to cause current leakage for 32K less */
#define GPIO_SRCLKEN_PIN (GPIO148|0x80000000) /*for readability*/


/*
	RTC_FGSOC = 0,
	RTC_ANDROID,
	RTC_RECOVERY,
	RTC_FAC_RESET,
	RTC_BYPASS_PWR,
	RTC_PWRON_TIME,
	RTC_FAST_BOOT,
	RTC_KPOC,
	RTC_DEBUG,
	RTC_PWRON_AL,
	RTC_UART,
	RTC_AUTOBOOT,
	RTC_PWRON_LOGO,
	RTC_32K_LESS,
	RTC_LP_DET,
	RTC_SPAR_NUM

*/
/*
 * RTC_PDN1:
 *     bit 0 - 3  : Android bits
 *     bit 4 - 5  : Recovery bits (0x10: factory data reset)
 *     bit 6      : Bypass PWRKEY bit
 *     bit 7      : Power-On Time bit
 *     bit 8      : RTC_GPIO_USER_WIFI bit
 *     bit 9      : RTC_GPIO_USER_GPS bit
 *     bit 10     : RTC_GPIO_USER_BT bit
 *     bit 11     : RTC_GPIO_USER_FM bit
 *     bit 12     : RTC_GPIO_USER_PMIC bit
 *     bit 13     : Fast Boot
 *     bit 14	  : Kernel Power Off Charging
 *     bit 15     : Debug bit
 */
/*
 * RTC_PDN2:
 *     bit 0 - 3 : MTH in power-on time
 *     bit 4	 : Power-On Alarm bit
 *     bit 5 - 6 : UART bits
 *     bit 7	 : autoboot bit
 *     bit 8 - 14: YEA in power-on time
 *     bit 15	 : Power-On Logo bit
 */
/*
 * RTC_SPAR0:
 *     bit 0 - 5 : SEC in power-on time
 *     bit 6	 : 32K less bit. True:with 32K, False:Without 32K
 *     bit 7 - 15: reserved bits
 */

u16 rtc_spare_reg[][3] = {
	{RTC_AL_HOU,	0x7f,	8},
	{RTC_PDN1,	0xf,	0},
	{RTC_PDN1,	0x3,	4},
	{RTC_PDN1,	0x1,	6},
	{RTC_PDN1,	0x1,	7},
	{RTC_PDN1,	0x1,	13},
	{RTC_PDN1,	0x1,	14},
	{RTC_PDN1,	0x1,	15},
	{RTC_PDN2,	0x1,	4},
	{RTC_PDN2,	0x3,	5},
	{RTC_PDN2,	0x1,	7},
	{RTC_PDN2,	0x1,	15},
	{RTC_SPAR0,	0x1,	6},
	{RTC_SPAR0,	0x1,	7}
};

void hal_rtc_set_abb_32k(u16 enable)
{

	hal_rtc_xinfo("ABB 32k not support\n");
}

u16 hal_rtc_get_gpio_32k_status(void)
{

	u16 con;
	con = rtc_read(RTC_CON);

	hal_rtc_xinfo("RTC_GPIO 32k status(RTC_CON=0x%x)\n", con);

	if (con & RTC_CON_F32KOB)
		return 0;
	else
		return 1;
}

void hal_rtc_set_gpio_32k_status(u16 user, bool enable)
{
	u16 con, pdn1;

	if (enable) {
		pdn1 = rtc_read(RTC_PDN1);
	} else {
		pdn1 = rtc_read(RTC_PDN1) & ~(1U << user);
		rtc_write(RTC_PDN1, pdn1);
		rtc_write_trigger();
	}

	con = rtc_read(RTC_CON);
	if (enable) {
		con &= ~RTC_CON_F32KOB;
	} else {
		if (!(pdn1 & RTC_GPIO_USER_MASK)) {	/* no users */
			con |= RTC_CON_F32KOB;
		}
	}
	rtc_write(RTC_CON, con);
	rtc_write_trigger();


	if (enable) {
		pdn1 |= (1U << user);
		rtc_write(RTC_PDN1, pdn1);
		rtc_write_trigger();
	}
	hal_rtc_xinfo("RTC_GPIO user %d enable = %d 32k (0x%x)\n", user, enable, pdn1);
}

static void hal_rtc_set_auto_bit(int val)
{
	if (val == 0)
		rtc_write(RTC_BBPU, RTC_BBPU_KEY | RTC_BBPU_BBPU | RTC_BBPU_PWREN);
	else
		rtc_write(RTC_BBPU, RTC_BBPU_KEY | RTC_BBPU_AUTO | RTC_BBPU_BBPU | RTC_BBPU_PWREN);

	rtc_write_trigger();
	hal_rtc_xinfo("hal_rtc_set_auto_bit(RTC_BBPU=0x%x)\n", rtc_read(RTC_BBPU));
}

void hal_rtc_set_register_status(const char *cmd, u16 val)
{
	if (!strcmp(cmd, "AUTO"))
		hal_rtc_set_auto_bit(val);
}

void hal_rtc_bbpu_pwdn(void)
{
	u16 ret_val, con;

	/* disable 32K export if there are no RTC_GPIO users */
	if (!(rtc_read(RTC_PDN1) & RTC_GPIO_USER_MASK)) {
		con = rtc_read(RTC_CON) | RTC_CON_F32KOB;
		rtc_write(RTC_CON, con);
		rtc_write_trigger();
	}
	ret_val = hal_rtc_get_spare_register(RTC_32K_LESS);
	#if !defined(CONFIG_MTK_FPGA)
	if (!ret_val  && pmic_chrdet_status() == FALSE) {
		/* 1.	Set SRCLKENAs GPIO GPIO as Output Mode, Output Low */
		mt_set_gpio_dir(GPIO_SRCLKEN_PIN , GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_SRCLKEN_PIN , GPIO_OUT_ZERO);
		/* 2. pull PWRBB low */
		rtc_bbpu_pwrdown(TRUE);

		/* 3.	Switch SRCLKENAs GPIO MUX function to GPIO Mode */
		mt_set_gpio_mode(GPIO_SRCLKEN_PIN , GPIO_MODE_GPIO);
	} else
	#endif
	{
		rtc_bbpu_pwrdown(TRUE);
	}
}

void hal_rtc_get_pwron_alarm(struct rtc_time *tm, struct rtc_wkalrm *alm)
{
	u16 pdn1, pdn2;


	pdn1 = rtc_read(RTC_PDN1);
	pdn2 = rtc_read(RTC_PDN2);

	alm->enabled = (pdn1 & RTC_PDN1_PWRON_TIME ? (pdn2 & RTC_PDN2_PWRON_LOGO ? 3 : 2) : 0);
	alm->pending = !!(pdn2 & RTC_PDN2_PWRON_ALARM);	/* return Power-On Alarm bit */

	hal_rtc_get_alarm_time(tm);
}

bool hal_rtc_is_lp_irq(void)
{
	u16 irqsta;

	irqsta = rtc_read(RTC_IRQ_STA);		/* read clear */
	if (unlikely(!(irqsta & RTC_IRQ_STA_AL))) {
#ifndef USER_BUILD_KERNEL
		if (irqsta & RTC_IRQ_STA_LP)
			rtc_lp_exception();
#endif
		return true;
	}

	return false;
}

bool hal_rtc_is_pwron_alarm(struct rtc_time *nowtm, struct rtc_time *tm)
{
	u16 pdn1;

	pdn1 = rtc_read(RTC_PDN1);
	hal_rtc_xinfo("pdn1 = 0x%4x\n", pdn1);

	if (pdn1 & RTC_PDN1_PWRON_TIME) {	/* power-on time is available */

		hal_rtc_xinfo("pdn1 = 0x%4x\n", pdn1);
		hal_rtc_get_tick_time(nowtm);
		hal_rtc_xinfo("pdn1 = 0x%4x\n", pdn1);
		if (rtc_read(RTC_TC_SEC) < nowtm->tm_sec) {	/* SEC has carried */
			hal_rtc_get_tick_time(nowtm);
		}

		hal_rtc_get_pwron_alarm_time(tm);

		return true;
	}

	return false;
}

void hal_rtc_get_alarm(struct rtc_time *tm, struct rtc_wkalrm *alm)
{
	u16 irqen, pdn2;

	irqen = rtc_read(RTC_IRQ_EN);
	hal_rtc_get_alarm_time(tm);
	pdn2 = rtc_read(RTC_PDN2);
	alm->enabled = !!(irqen & RTC_IRQ_EN_AL);
	alm->pending = !!(pdn2 & RTC_PDN2_PWRON_ALARM);	/* return Power-On Alarm bit */
}

void hal_rtc_set_alarm(struct rtc_time *tm)
{
	u16 irqen;

	hal_rtc_set_alarm_time(tm);

	irqen = rtc_read(RTC_IRQ_EN) | RTC_IRQ_EN_ONESHOT_AL;
	rtc_write(RTC_IRQ_EN, irqen);
	rtc_write_trigger();
}

void hal_rtc_clear_alarm(struct rtc_time *tm)
{
	u16 irqsta, irqen, pdn2;

	irqen = rtc_read(RTC_IRQ_EN) & ~RTC_IRQ_EN_AL;
	pdn2 = rtc_read(RTC_PDN2) & ~RTC_PDN2_PWRON_ALARM;
	rtc_write(RTC_IRQ_EN, irqen);
	rtc_write(RTC_PDN2, pdn2);
	rtc_write_trigger();
	irqsta = rtc_read(RTC_IRQ_STA);		/* read clear */

	hal_rtc_set_alarm_time(tm);
}

void hal_rtc_set_lp_irq(void)
{
	u16 irqen;

#ifndef USER_BUILD_KERNEL
	irqen = rtc_read(RTC_IRQ_EN) | RTC_IRQ_EN_LP;
#else
	irqen = rtc_read(RTC_IRQ_EN) & ~RTC_IRQ_EN_LP;
#endif
	rtc_write(RTC_IRQ_EN, irqen);
	rtc_write_trigger();
}

void hal_rtc_save_pwron_time(bool enable, struct rtc_time *tm, bool logo)
{
	u16 pdn1, pdn2;

	hal_rtc_set_pwron_alarm_time(tm);

	if (logo)
		pdn2 = rtc_read(RTC_PDN2) | RTC_PDN2_PWRON_LOGO;
	else
		pdn2 = rtc_read(RTC_PDN2) & ~RTC_PDN2_PWRON_LOGO;

	rtc_write(RTC_PDN2, pdn2);

	if (enable)
		pdn1 = rtc_read(RTC_PDN1) | RTC_PDN1_PWRON_TIME;
	else
		pdn1 = rtc_read(RTC_PDN1) & ~RTC_PDN1_PWRON_TIME;

	rtc_write(RTC_PDN1, pdn1);
	rtc_write_trigger();
}


#ifdef VRTC_PWM_ENABLE
void hal_rtc_pwm_enable(void)
{
	rtc_write(MT_VRTC_PWM_CON0, 0);		/*clear*/

	hal_rtc_xinfo("hal_rtc_pwm_enable(), RTC_CAP_SEL=%d\n", RTC_CAP_SEL);

	switch	(RTC_CAP_SEL) {
	case 0:	/*0.1uF only*/
			rtc_write(MT_VRTC_PWM_CON0, VRTC_PWM_H_DUTY_0_64_MS|VRTC_PWM_L_DUTY_6_4_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=0.64ms,L=6.4ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	case 1:  /*0.1uF + 1uF + 1.5Kohm*/
			rtc_write(MT_VRTC_PWM_CON0, VRTC_PWM_H_DUTY_5_12_MS|VRTC_PWM_L_DUTY_51_2_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=5.12ms,L=51.2ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	case 2: /*0.1uF + 2.2uF + 1.5Kohm*/
			rtc_write(MT_VRTC_PWM_CON0,
			    VTRC_CAP_SEL|VRTC_PWM_H_DUTY_25_6_MS|VRTC_PWM_L_DUTY_128_0_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=25.6ms,L=128ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	case 3: /*0.1uF + 4.7uF + 1.5Kohm*/
			rtc_write(MT_VRTC_PWM_CON0,
			    VTRC_CAP_SEL|VRTC_PWM_H_DUTY_51_2_MS|VRTC_PWM_L_DUTY_256_0_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=51.2ms,L=256ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	case 4: /*0.1uF + 10uF + 1.5Kohm*/
			rtc_write(MT_VRTC_PWM_CON0,
			    VTRC_CAP_SEL|VRTC_PWM_H_DUTY_102_4_MS|VRTC_PWM_L_DUTY_512_0_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=102.4ms,L=512ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	case 5: /*0.1uF + 22uF + 1.5Kohm*/
			rtc_write(MT_VRTC_PWM_CON0,
			    VTRC_CAP_SEL|VRTC_PWM_H_DUTY_204_8_MS|VRTC_PWM_L_DUTY_1024_0_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=204.8ms,L=1024ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	case 6: /*0.1uF + super cap(>>22uF) + 1.5Kohm*/
			rtc_write(MT_VRTC_PWM_CON0,
			    VTRC_CAP_SEL|VRTC_PWM_H_DUTY_204_8_MS|VRTC_PWM_L_DUTY_1024_0_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=204.8ms,L=1024ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	case 7:	/*0.1uF + little Li battery + 1.5Kohm*/
			rtc_write(MT_VRTC_PWM_CON0,
			    VTRC_CAP_SEL|VRTC_PWM_H_DUTY_204_8_MS|VRTC_PWM_L_DUTY_512_0_MS|VRTC_PWM_MODE);
			hal_rtc_xinfo("H=204.8ms,L=512ms, VRTC_PWM=%x\n", rtc_read(MT_VRTC_PWM_CON0));
			break;
	default:
			hal_rtc_xinfo("RTC CAP SEL is wrong !!!!");
			break;
	}

}
#endif
