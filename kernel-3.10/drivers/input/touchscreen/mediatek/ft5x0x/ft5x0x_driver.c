/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#include "tpd_custom_ft5x0x.h"
#include "cust_gpio_usage.h"
//#include "ft5x06_ex_fun.h"

#include "tpd.h"

//#define TIMER_DEBUG 

#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif


#define TPD_SUPPORT_POINTS	5

//#define CUST_FTS_APK_DEBUG 1

extern struct tpd_device *tpd;

struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);

static void tpd_eint_interrupt_handler(void);

extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_mask(unsigned int eint_num);

extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag,
				void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);
#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
		kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
		kal_bool auto_umask);
#endif
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
#if CUST_FTS_APK_DEBUG
extern int ft_rw_iic_drv_init(struct i2c_client *client);
extern void  ft_rw_iic_drv_exit(void);
#endif

#ifdef TPD_AUTO_UPGRADE_SUPPORT
u8 *tpd_i2c_dma_va = NULL;
u32 tpd_i2c_dma_pa = NULL;
extern int tpd_auto_upgrade(struct i2c_client *client);
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;

#define TPD_OK 0

//Register define
#define DEVICE_MODE	0x00
#define GEST_ID		0x01
#define TD_STATUS	0x02

#define TOUCH1_XH	0x03
#define TOUCH1_XL	0x04
#define TOUCH1_YH	0x05
#define TOUCH1_YL	0x06

#define TOUCH2_XH	0x09
#define TOUCH2_XL	0x0A
#define TOUCH2_YH	0x0B
#define TOUCH2_YL	0x0C

#define TOUCH3_XH	0x0F
#define TOUCH3_XL	0x10
#define TOUCH3_YH	0x11
#define TOUCH3_YL	0x12

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3

#ifdef TIMER_DEBUG

static struct timer_list test_timer;

static void timer_func(unsigned long data)
{	
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	mod_timer(&test_timer, jiffies + 100*(1000/HZ));
}

static int init_test_timer(void)
{
	memset((void*)&test_timer, 0, sizeof(test_timer));
	test_timer.expires  = jiffies + 100*(1000/HZ);   
	test_timer.function = timer_func;
	test_timer.data     = 0;
	init_timer(&test_timer);
	add_timer(&test_timer);
	return 0;
}
#endif

#if defined(TPD_ROTATE_90) || defined(TPD_ROTATE_270)|| defined(TPD_ROTATE_180)
static void tpd_swap_xy(int *x, int *y)
{
        int temp = 0;

        temp = *x;
        *x = *y;
        *y = temp;
}

static void tpd_rotate_90(int *x, int *y)
{
	int temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
static void tpd_rotate_180(int *x, int *y)
{
        *y = TPD_RES_Y + 1 - *y;
        *x = TPD_RES_X + 1 - *x;
}
static void tpd_rotate_270(int *x, int *y)
{
        int temp;

        *y = TPD_RES_Y + 1 - *y;

        *x = (*x * TPD_RES_Y) / TPD_RES_X;
        *y = (*y * TPD_RES_X) / TPD_RES_Y;

        tpd_swap_xy(x, y);
}
#endif
struct touch_info
{
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int count;
};

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
//static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
//static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id ft5x0x_tpd_id[] = {{"ft5x0x",0},{}};
static struct i2c_board_info __initdata ft5x0x_i2c_tpd={ I2C_BOARD_INFO("ft5x0x", (FT5X0X_I2C_ADDR >> 1))};

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
                 .name = "ft5x0x",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5x0x_tpd_id,
	.detect = tpd_detect,
};

static void tpd_down(int x, int y, int p)
{
#if defined(TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
    if (0 != x)
#endif
    {
	    TPD_DEBUG("%s x:%d y:%d p:%d\n", __func__, x, y, p);
	    input_report_key(tpd->dev, BTN_TOUCH, 1);
	    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	    input_mt_sync(tpd->dev);
    }
}

static void tpd_up(int x, int y)
{
#if defined(TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
    if (0 != x)
#endif
    {
	    TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
	    input_report_key(tpd->dev, BTN_TOUCH, 0);
	    input_mt_sync(tpd->dev);
    }
}

/*Coordination mapping*/
static void tpd_calibrate_driver(int *x, int *y)
{
	int tx;	

	tx = ( (tpd_def_calmat[0] * (*x)) + (tpd_def_calmat[1] * (*y)) + (tpd_def_calmat[2]) ) >> 12;
	*y = ( (tpd_def_calmat[3] * (*x)) + (tpd_def_calmat[4] * (*y)) + (tpd_def_calmat[5]) ) >> 12;
	*x = tx;
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[40] = {0};
	u8 report_rate = 0;
	u16 high_byte, low_byte;

	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[32]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);

	TPD_DEBUG("FW version=%x]\n",data[32]);

#if 0
	TPD_DEBUG("received raw data from touch panel as following:\n");
	for(i = 0; i < 8; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for(i = 8; i < 16; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for(i = 16; i < 24; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for(i = 24; i < 32; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
#endif
	if(report_rate < 8)
	{
		report_rate = 0x8;
		if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if(data[0] & 0x70 != 0)
		return false; 

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	for(i = 0; i < TPD_SUPPORT_POINTS; i++)
		cinfo->p[i] = 1;	//Put up

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;

	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);

	TPD_DEBUG("Procss raw data...\n");

	for(i = 0; i < cinfo->count; i++)
	{
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; //event flag 

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;

		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i , cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}

		
		
	
#ifdef TPD_HAVE_CALIBRATION
	for (i = 0; i < cinfo->count; i++)
	{
        tpd_calibrate_driver(&(cinfo->x[i]), &(cinfo->y[i]));
		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i , cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);	
	}
#endif	

	return true;

};

static int touch_event_handler(void *unused)
{
	int i = 0;
	struct touch_info cinfo, pinfo;

#ifdef TPD_HAVE_BUTTON
	struct touch_info finfo;

	memset(&finfo, 0, sizeof(struct touch_info));
	for(i = 0; i < TPD_SUPPORT_POINTS; i++)
		finfo.p[i] = 1;
#endif

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

		TPD_DEBUG("touch_event_handler start \n");

		if(tpd_touchinfo(&cinfo, &pinfo))
		{
#ifdef TPD_HAVE_BUTTON
			if(cinfo.p[0] == 0)
				memcpy(&finfo, &cinfo, sizeof(struct touch_info));
#endif

			if((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y) && ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
			{
				TPD_DEBUG("Dummy release --->\n");
				tpd_up(pinfo.x[0], pinfo.y[0]);
				input_sync(tpd->dev);
				continue;
			}

#ifdef TPD_HAVE_BUTTON
			if((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y) 
				&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
			{
				TPD_DEBUG("Dummy key release --->\n");
				tpd_button(pinfo.x[0], pinfo.y[0], 0);
				input_sync(tpd->dev);
				continue;
			}

			if((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y))
			{
				if(finfo.y[0] > TPD_RES_Y)
				{
					if((cinfo.p[0] == 0) || (cinfo.p[0] == 2))
					{
						TPD_DEBUG("Key press --->\n");
						tpd_button(pinfo.x[0], pinfo.y[0], 1);
					}
					else if((cinfo.p[0] == 1) && ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
					{
						TPD_DEBUG("Key release --->\n");
						tpd_button(pinfo.x[0], pinfo.y[0], 0);
					}
					input_sync(tpd->dev);
				}
				continue;
			}
#endif

			if(cinfo.count > 0)
			{
				for(i = 0; i < cinfo.count; i++)
					tpd_down(cinfo.x[i], cinfo.y[i], i + 1);
			}
			else
			{
#ifdef TPD_SOLVE_CHARGING_ISSUE
				tpd_up(1, 48);
#else
				tpd_up(cinfo.x[0], cinfo.y[0]);
#endif
				
			}
			input_sync(tpd->dev);

		}
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit \n");

	return 0;
}

static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	

	return 0;
}

static void tpd_eint_interrupt_handler(void)
{
	TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int retval = TPD_OK;
	int err = 0;
	int report_rate = 0;
	int reset_count = 0;
	char data;

	i2c_client = client;
	client->timing=400;//liuhuan

#ifdef GPIO_CTP_PWR_PIN
	mt_set_gpio_mode(GPIO_CTP_PWR_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_PWR_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_PWR_PIN, GPIO_OUT_ONE);
	msleep(50);
#endif

	TPD_DMESG("mtk_tpd: tpd_probe ft5x0x \n");
#if 0
	mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	msleep(50);
#endif
#if 0	
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif

#if 0
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
#else
	hwPowerDown(MT6323_POWER_LDO_VGP2,  "TP");
#endif
#endif 

#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	TPD_DMESG("mtk_tpd: power MT65XX_POWER_LDO_VGP2 \n");
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_3000, "TP");
	TPD_DMESG("mtk_tpd: end power MT65XX_POWER_LDO_VGP2 \n");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif

#if 0
	//reset
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(1);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
#endif
	//set INT mode
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	msleep(100);

#ifdef TPD_AUTO_UPGRADE_SUPPORT
	tpd_i2c_dma_va = (u8 *)dma_alloc_coherent(&client->dev, 4096, &tpd_i2c_dma_pa, GFP_KERNEL);
	if(!tpd_i2c_dma_va)
		TPD_DMESG("TPD dma_alloc_coherent error!\n");
	else
		TPD_DMESG("TPD dma_alloc_coherent success!\n");
#endif

reset_proc:
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
		if(reset_count < TPD_MAX_RESET_COUNT)
		{
			reset_count++;			
			goto reset_proc;
		}
#endif
		return -1; 
	}
    tpd_load_status = 1;
#if CUST_FTS_APK_DEBUG
	ft_rw_iic_drv_init(client);

	ft5x0x_create_sysfs(client);
	
	ft5x0x_create_apk_debug_channel(client);
#endif
#ifdef TPD_AUTO_UPGRADE_SUPPORT
	//msleep(200);
	msleep(10);//liuhuan
	tpd_auto_upgrade(client); 
	msleep(200);//liuhuan
	//Reset CTP
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	//msleep(1);
	msleep(20);//liuhuan
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	//msleep(50);//liuhuan
#endif
	msleep(200);//liuhuan
	//Set report rate 80Hz
	report_rate = 0x8;
	if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate)) < 0)
	{
		if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}

	//tpd_load_status = 1;

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}

	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef TIMER_DEBUG
    init_test_timer();
#endif

	{
		int ver;

		i2c_smbus_read_i2c_block_data(client, 0xA6, 1, &ver);

		TPD_DMESG(TPD_DEVICE " i2c_smbus_read_i2c_block_data version : %d\n", ver);
	}
	


	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
#if CUST_FTS_APK_DEBUG
	ft_rw_iic_drv_exit();
#endif

#ifdef TPD_AUTO_UPGRADE_SUPPORT
	if(tpd_i2c_dma_va)
	{
		dma_free_coherent(NULL, 4096, tpd_i2c_dma_va, tpd_i2c_dma_pa);
		tpd_i2c_dma_va = NULL;
		tpd_i2c_dma_pa = 0;
	}
#endif

	return 0;
}

static int tpd_local_init(void)
{
	TPD_DMESG("Focaltech FT5x0x I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
     //tpd_load_status = 1;
#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	
	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);
			
	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);
	
#endif

	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;

	return 0; 
}

static int tpd_resume(struct i2c_client *client)
{
	int retval = TPD_OK;

	TPD_DEBUG("TPD wake up\n");

#if 0	
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_3000, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif
	msleep(100);

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(1);  
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  

	return retval;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	int retval = TPD_OK;
	static char data = 0x3;

	TPD_DEBUG("TPD enter sleep\n");
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode


#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
	hwPowerDown(MT6323_POWER_LDO_VGP2, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif

	return retval;
} 

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT5x0x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("MediaTek FT5x0x touch panel driver init\n");
        i2c_register_board_info(0, &ft5x0x_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5x0x driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
	TPD_DMESG("MediaTek FT5x0x touch panel driver exit\n");
       	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

