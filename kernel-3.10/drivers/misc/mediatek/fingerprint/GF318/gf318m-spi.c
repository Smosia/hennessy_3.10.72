/*
 *Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_pmic_wrap.h>
//#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>

#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/eint.h>
#include <cust_eint.h>
//#include <cust_gpio_usage.h>

#include <asm/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <mach/mt_spi.h>
#include <mach/mt_gpio.h>
#include <mach/mt_clkmgr.h>
#include "gf318m-spi.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 

#define SPI_BUS_DYNAMIC
#define GF66XX_PID              "GFx18M"
#define SPI_DEV_NAME            "goodix_fp"
#define GF66XX_INPUT_DEV_NAME   "mtk_goodix_fp"
//"gf66xx_input_key"
#define GF66XX_INPUT_MENU_KEY   KEY_MENU
#define GF66XX_INPUT_BACK_KEY   KEY_BACK
#define GF66XX_INPUT_HOME_KEY   KEY_HOME
#define GF66XX_FF_KEY           KEY_POWER
/*device name after register in charater*/
#define DEV_NAME "goodix_fp"

#define CHRD_DRIVER_NAME        "gf66xx"
#define CLASS_NAME              "gf66xx-spi"
#define SPIDEV_MAJOR			66	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
#define FW_LENGTH               (42*1024)
#define CFG_UPDATE               1
#define FW_UPDATE               1
#define ESD_PROTECT             0
#define CONFIG_COMPAT
#if FW_UPDATE
static unsigned char  GF66XX_FW[] =
{
        #include "gfx18m.i"
};
#endif

static char product_id_name[5]={0};

static int gf66xx_probe(struct spi_device *spi);
static int gf66xx_remove(struct spi_device *spi);

static DECLARE_BITMAP(minors, N_SPI_MINORS);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define     MTK_SPI_ALIGN_BITS  (9)
#define     MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_BITS) - 1)
/**************************debug******************************/
#define GF66XX_DEBUG   1
//#undef GF66XX_DEBUG
#define DEBUG_LOG 1
#define SPI_TEST 0
#ifdef GF66XX_DEBUG
#define   gf66xx_dbg(fmt, args...) do{ \
					pr_warn("gf66xx:" fmt, ##args);\
				}while(0)
#define FUNC_ENTRY()  pr_warn("gf66xx:%s, entry\n", __func__)
#define FUNC_EXIT()  pr_warn("gf66xx:%s, exit\n", __func__)
#else
#define gf66xx_dbg(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif

#if CFG_UPDATE
const u8 ofilm_vendor_id[]={0xA5, 0xE3, 0x81};
const u8 truly_vendor_id[]={0xA6, 0xE3, 0x81};
const u8 toptouch_vendor_id[] = {0xA8, 0xE3, 0x81};
/***********************************************************
gf66xx config for GF66XX_2030(6BC2).BIN 
************************************************************/
u8 gf66xx_config_ofilm[] = {
	0x00,0x3C,0x3C,0xE4,0x0C,0x30,0x3F,0x02,0x00,0x50,0x40,0x50,0x50,0xE4,0x0C,0x30,0x2F,0x03,0x00,0x03,	
	0x11,0xA0,0x0D,0x00,0x14,0x03,0x0F,0x0F,0x0F,0xB2,0x3F,0xB3,0x33,0x03,0x90,0x01,0x40,0x05,0x0E,0x80,	
	0x20,0x0F,0x22,0x00,0x08,0x07,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,	
	0x01,0x25,0x04,0xCA,0xA4,0x26,0x66,0x00,0x00,0x00,0x01,0x00,0x01,0x0F,0x96,0x00,0x01,0x02,0x85,0x00,	
	0x03,0x20,0x20,0x50,0x3E,0x11,0x01,0x00,0x00,0x00,0x00,0x03,0x09,0x00,0x31,0x00,0x07,0x14,0x41,0x00,	
	0x50,0x00,0x00,0x00,0x65,0x00,0x04,0x00,0x32,0x01,0xA0,0x00,0x00,0x79,0xC8,0x00,0x00,0x00,0x28,0x00,	
	0x05,0x04,0x45,0x00,0x08,0x00,0x07,0x00,0x20,0x00,0x18,0x00,0x3D,0x00,0x48,0x00,0x22,0x00,0x00,0x00,	
	0x03,0x07,0x80,0x00,0x20,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE8,0x01
};
u8 gf66xx_config_truly[] = {
	0x00, 0x3C, 0x3C, 0xE4, 0x0C, 0x30, 0x3F, 0x02, 0x00, 0x50, 0x40, 0x50, 0x50, 0xE4, 0x0C, 0x30, 
	0x2F, 0x03, 0x00, 0x03, 0x11, 0xA0, 0x0D, 0x00, 0x14, 0x03, 0x0F, 0x0F, 0x0F, 0xB2, 0x3F, 0xB3, 
	0x33, 0x03, 0x90, 0x01, 0x40, 0x05, 0x0E, 0x80, 0x20, 0x0F, 0x22, 0x00, 0x08, 0x07, 0x08, 0x06, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x25, 0x04, 0xCA, 0xA4, 0x26, 0x66, 0x00, 
	0x00, 0x00, 0x01, 0x00, 0x01, 0x0F, 0x96, 0x00, 0x01, 0x02, 0x85, 0x00, 0x03, 0x20, 0x20, 0x50, 
	0x3E, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x09, 0x00, 0x31, 0x00, 0x07, 0x14, 0x41, 0x00, 
	0x50, 0x00, 0x00, 0x00, 0x50, 0x00, 0x04, 0x00, 0x32, 0x01, 0xA0, 0x00, 0x00, 0x79, 0xC8, 0x00, 
	0x00, 0x00, 0x28, 0x00, 0x05, 0x04, 0x30, 0x00, 0x08, 0x00, 0x07, 0x00, 0x20, 0x00, 0x18, 0x00, 
	0x3D, 0x00, 0x48, 0x00, 0x22, 0x00, 0x00, 0x00, 0x03, 0x07, 0x80, 0x00, 0x20, 0x00, 0x50, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x01
};

u8 gf66xx_config_toptouch[] = {
				0x00,0x3c,0x3c,0xe4,0x0c,0x30,0x3f,0x02,0x00,0x50,0x40,0x50,0x50,0xe4,0x0c,0x30,
				0x2f,0x03,0x00,0x03,0x11,0xa0,0x0d,0x00,0x14,0x03,0x0f,0x0f,0x0f,0xb2,0x3f,0xb3,
				0x33,0x03,0x90,0x01,0x40,0x05,0x0e,0x80,0x20,0x0f,0x22,0x00,0x08,0x07,0x08,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x01,0x25,0x04,0xca,0xa4,0x26,0x66,0x00,
				0x00,0x00,0x01,0x00,0x01,0x0f,0x96,0x00,0x01,0x02,0x85,0x00,0x03,0x20,0x20,0x50,
				0x3e,0x11,0x01,0x00,0x00,0x00,0x00,0x03,0x09,0x00,0x31,0x00,0x07,0x14,0x41,0x00,
				0x50,0x00,0x00,0x00,0x50,0x00,0x04,0x00,0x32,0x01,0xa0,0x00,0x00,0x79,0xc8,0x00,
				0x00,0x00,0x28,0x00,0x05,0x04,0x45,0x00,0x08,0x00,0x07,0x00,0x20,0x00,0x18,0x00,
				0x3d,0x00,0x48,0x00,0x22,0x00,0x00,0x00,0x03,0x07,0x80,0x00,0x20,0x00,0x50,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfd,0x01,
};
#endif

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf66xx_dev gf66xx;
static struct task_struct *gf66xx_irq_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int suspend_flag = 0;
static int irq_flag = 0;
static int thread_flag = 0;

/*************************data stream***********************
*	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
*             1B      |   1B       |  1B        |    2048B           |  2B            |
************************************************************/
static unsigned int  bufsiz =8*(2048+5);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

#if 0
static struct mt_chip_conf spi_conf_mt65xx = {

	.setuptime = 60,
	.holdtime = 60,
	.high_time = 15, //15--3m   25--2m   50--1m  [ 100--0.5m]
	.low_time = 15,
	.cs_idletime = 8,
	.ulthgh_thrsh = 0,
	
	.cpol = 0,
	.cpha = 0,
	
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	//.com_mod = FIFO_TRANSFER,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#else 
static struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 15,
	.holdtime = 15,
	.high_time = 21, //for mt6582, 104000khz/(4+4) = 130000khz
	.low_time = 21,
	.cs_idletime = 20,
	.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,
	.tx_mlsb = 1,

	.tx_endian = 0,
	.rx_endian = 0,

	.com_mod = FIFO_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

struct mt_chip_conf spi_conf={
	.setuptime=60,
       .holdtime=60,
       .high_time=50,
       .low_time=50,
       .cs_idletime=8,
       .ulthgh_thrsh=0,
       .cpol=SPI_CPOL_0,
       .cpha=SPI_CPHA_0,
       .rx_mlsb=SPI_MSB,
        .tx_mlsb=SPI_MSB,
        .tx_endian=SPI_LENDIAN,
        .rx_endian=SPI_LENDIAN,
        .com_mod=DMA_TRANSFER,
        .pause=0,
        .deassert=0,
        .ulthigh=0,
        .tckdly=0,
};

typedef enum {
	SPEED_500KHZ=0,
	SPEED_1MHZ,
	SPEED_2MHZ,
	SPEED_3MHZ,
	SPEED_4MHZ,
	SPEED_6MHZ,
	SPEED_8MHZ,
	SPEED_KEEP,
	SPEED_UNSUPPORTED
}SPI_SPEED;

static void gf66xx_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
	struct mt_chip_conf *mcc = &spi_conf_mt65xx;
	if(flag == 0) {
		mcc->com_mod = FIFO_TRANSFER;
	} else {
		mcc->com_mod = DMA_TRANSFER;
	}

	switch(speed)
	{
		case SPEED_500KHZ:
			mcc->high_time = 120;
			mcc->low_time = 120;
			break;
		case SPEED_1MHZ:
			mcc->high_time = 60;
			mcc->low_time = 60;
			break;
		case SPEED_2MHZ:
			mcc->high_time = 30;
			mcc->low_time = 30;
			break;
		case SPEED_3MHZ:
			mcc->high_time = 20;
			mcc->low_time = 20;
			break;
		case SPEED_4MHZ:
			mcc->high_time = 15;
			mcc->low_time = 15;
			break;

		case SPEED_6MHZ:
			mcc->high_time = 10;
			mcc->low_time = 10;
			break;
		case SPEED_8MHZ:
		    mcc->high_time = 8;
			mcc->low_time = 8;
			break;
		case SPEED_KEEP:
		case SPEED_UNSUPPORTED:
			break;
	}
	if(spi_setup(spi) < 0){
		pr_warn("gf66xx:Failed to set spi.\n");
	}
}

/**********************************************************
*Message format:
*	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
*    1B         |   1B    |  1B    |  length       |
*
* write buffer length should be 1 + 1 + 1 + data_length
***********************************************************/
int gf66xx_spi_write_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + GF66XX_WDATA_OFFSET)>>MTK_SPI_ALIGN_BITS;
	u32  reminder = (data_len + GF66XX_WDATA_OFFSET) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;

	/*set spi mode.*/
	if((data_len + GF66XX_WDATA_OFFSET) > 32) {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 0); //FIFO
	}

	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		/*copy the reminder data to temporarity buffer.*/
		reminder_buf = kzalloc(reminder + GF66XX_WDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			pr_err("gf66xx:No memory for exter data.\n");
			return -ENOMEM;
		}
		memcpy(reminder_buf + GF66XX_WDATA_OFFSET, tx_buf + GF66XX_WDATA_OFFSET+data_len - reminder, reminder);
        gf66xx_dbg("gf66xx:w-reminder:0x%x-0x%x,0x%x\n", reminder_buf[GF66XX_WDATA_OFFSET],reminder_buf[GF66XX_WDATA_OFFSET+1],
                reminder_buf[GF66XX_WDATA_OFFSET + 2]);
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	}
	if( xfer == NULL){
		gf66xx_dbg("gf66xx:No memory for command.\n");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}

	gf66xx_dbg("gf66xx:write twice = %d. data_len = %d, package_num = %d, reminder = %d\n", (int)twice, (int)data_len, (int)package_num, (int)reminder);
	/*if the length is not align with 1024. Need 2 transfer at least.*/
	spi_message_init(&msg);
	tx_buf[0] = GF66XX_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
//    xfer[0].delay_usecs = 5;
	if(twice == 1) {
		xfer[0].len = package_num << 10;
		spi_message_add_tail(&xfer[0], &msg);
		
		addr += data_len - reminder;
		reminder_buf[0] = GF66XX_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[1].tx_buf = reminder_buf;
		xfer[1].len = reminder + GF66XX_WDATA_OFFSET;
//        xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
	} else {
		xfer[0].len = data_len + GF66XX_WDATA_OFFSET;
		spi_message_add_tail(&xfer[0], &msg);
	}

	ret = spi_sync(gf66xx_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1)
			ret = msg.actual_length - 2*GF66XX_WDATA_OFFSET;
		else
			ret = msg.actual_length - GF66XX_WDATA_OFFSET;
	} else 	{
		pr_warn("gf66xx:write async failed. ret = %d\n", ret);
	}

	if(xfer != NULL) {
		kfree(xfer);
		xfer = NULL;
	}
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}
	
	return ret;
}

/*************************************************************
*First message:
*	write cmd   |  ADDR_H |ADDR_L  |
*    1B         |   1B    |  1B    |
*Second message:
*	read cmd   |  data stream  |
*    1B        |   length    |
*
* read buffer length should be 1 + 1 + 1 + 1 + data_length
**************************************************************/
int gf66xx_spi_read_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + 1)>>MTK_SPI_ALIGN_BITS;
	u32  reminder = (data_len + 1) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;
	
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		reminder_buf = kzalloc(reminder + GF66XX_RDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			pr_err("No memory for exter data.\n");
			return -ENOMEM;
		}
		xfer = kzalloc(sizeof(*xfer)*4, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	}
	if( xfer == NULL){
		gf66xx_dbg("No memory for command.\n");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}
	/*set spi mode.*/
	if((data_len + GF66XX_RDATA_OFFSET) > 32) {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	spi_message_init(&msg);
    /*send GF66XX command to device.*/
	rx_buf[0] = GF66XX_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
	spi_message_add_tail(&xfer[0], &msg);
       spi_sync(gf66xx_dev->spi, &msg);
       spi_message_init(&msg);

	/*if wanted to read data from GF66XX. 
	 *Should write Read command to device
	 *before read any data from device.
	 */
		//memset(rx_buf, 0xff, data_len);
	rx_buf[4] = GF66XX_R;
	xfer[1].tx_buf = &rx_buf[4];
	xfer[1].rx_buf = &rx_buf[4];
	if(twice == 1)
		xfer[1].len = (package_num << 10);
	else
		xfer[1].len = data_len + 1;
	spi_message_add_tail(&xfer[1], &msg);

	if(twice == 1) {
		addr += data_len - reminder;
		reminder_buf[0] = GF66XX_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[2].tx_buf = reminder_buf;
		xfer[2].len = 3;
		spi_message_add_tail(&xfer[2], &msg);
        
    spi_sync(gf66xx_dev->spi, &msg);
    spi_message_init(&msg);
		reminder_buf[4] = GF66XX_R;
		xfer[3].tx_buf = &reminder_buf[4];
		xfer[3].rx_buf = &reminder_buf[4];
		xfer[3].len = reminder + 1;
		spi_message_add_tail(&xfer[3], &msg);
	}
/*
    if(twice == 1) {
        xfer[3].delay_usecs = 5;
    } else {
        xfer[1].delay_usecs = 5;
    }
*/
	ret = spi_sync(gf66xx_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1) {
            gf66xx_dbg("gf66xx:reminder:0x%x:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", reminder_buf[0], reminder_buf[1], 
                    reminder_buf[2], reminder_buf[3],reminder_buf[4],reminder_buf[5],reminder_buf[6],reminder_buf[7]);
			memcpy(rx_buf + GF66XX_RDATA_OFFSET + data_len - reminder, reminder_buf + GF66XX_RDATA_OFFSET, reminder);
			ret = data_len;//msg.actual_length - 1; //8 
		} else {
			ret = data_len;//msg.actual_length - 1; //4
		}
	}else {
        pr_warn("gf66xx: read failed. ret = %d\n", ret);
    }

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}	
	
	gf66xx_dbg("gf66xx:read twice = %d, data_len = %d, package_num = %d, reminder = %d\n",(int)twice, (int)data_len, (int)package_num, (int)reminder);
	gf66xx_dbg("gf66xx:data_len = %d, msg.actual_length = %d, ret = %d\n", (int)data_len, (int)msg.actual_length, ret);
	return ret;
}

static int gf66xx_spi_read_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 *value)
{
	int status = 0;
	mutex_lock(&gf66xx_dev->buf_lock);
	
	status = gf66xx_spi_read_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	*value = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	gf66xx_dbg("status = %d, value = 0x%x, buffer[4] = 0x%x, buffer[5] = %d\n",(int)status, *value, 
			gf66xx_dev->buffer[4], (int)gf66xx_dev->buffer[5]);
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}
static int gf66xx_spi_write_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 value)
{
	int status = 0;
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = value;
	status = gf66xx_spi_write_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t
gf66xx_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
	long int t1, t2;
	long int t3, t4;
	int i=0;
	FUNC_ENTRY();
	if(count > bufsiz) {
		pr_warn("gf66xx:Max size for write buffer is %d\n", (int)bufsiz);
		return -EMSGSIZE;
	}FUNC_EXIT();

	gf66xx_dev = filp->private_data;
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_4MHZ, 0);
	t1 = ktime_to_us(ktime_get());
	status = gf66xx_spi_read_bytes(gf66xx_dev, GF66XX_BUFFER_DATA, count, gf66xx_dev->buffer);
	t2 = ktime_to_us(ktime_get());
	if(status > 0) {
		unsigned long missing = 0;
		t3 = ktime_to_us(ktime_get());
#if DEBUG_LOG
		printk(KERN_ERR"gf66xx_read: [copy_to_user] count=%d\n",(int)count);
		for(i=0;i<count;i++){
			printk(KERN_ERR"gf66xx_read: [copy_to_user] gf66xx_dev->buffer + GF66XX_RDATA_OFFSET=0x%x\n", (gf66xx_dev->buffer + GF66XX_RDATA_OFFSET)[i]);
			if(count>16)
				break;
		}
#endif		
		missing = copy_to_user(buf, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, status);
		t4 = ktime_to_us(ktime_get());
		gf66xx_dbg("gf66xx:spi use : %ld us, copy use : %ld us, status = %d, count = %d\n", (t2-t1), (t4-t3), (int)status, (int)count);
		if(missing == status)
			status = -EFAULT;
	} else {
		pr_err("Failed to read data from SPI device.\n");
		status = -EFAULT;
	}

	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}

/* Write-only message with current device setup */
static ssize_t
gf66xx_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
#if DEBUG_LOG
	int i=0;
#endif
	FUNC_ENTRY();
	if(count > bufsiz) {
		pr_warn("Max size for write buffer is %d\n", (int)bufsiz);
		return -EMSGSIZE;
	} 

	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_4MHZ, 0);
	status = copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, buf, count);
#if DEBUG_LOG       
	printk(KERN_ERR"gf66xx_write: [copy_from_user] count=%d\n",(int)count);
	for(i=0;i<count;i++){
		printk(KERN_ERR"gf66xx_write: [copy_from_user] gf66xx_dev->buffer + GF66XX_RDATA_OFFSET=0x%x\n",(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET)[i]);
		if(count>16)
			break;
	}
#endif	
	if(status == 0) {
		status = gf66xx_spi_write_bytes(gf66xx_dev, 0x8000, count, gf66xx_dev->buffer);
	} else {
		pr_err("Failed to xfer data through SPI bus.\n");
		status = -EFAULT;
	}
	mutex_unlock(&gf66xx_dev->buf_lock);
	FUNC_EXIT();
	return status;
}

static long
gf66xx_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf66xx_dev *gf66xx_dev = NULL;
	struct gf66xx_ioc_transfer *ioc = NULL;
	int			err = 0;
	int 		retval = 0;
	
#if DEBUG_LOG
	u8 *temp_buf;
	int i=0;
#endif
	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF66XX_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;
	
	gf66xx_dev = (struct gf66xx_dev *)filp->private_data;
#if DEBUG_LOG
	printk(KERN_ERR "cmd in IOCTL is %d\n",(int)cmd);
#endif	
	switch(cmd) {
	case GF66XX_IOC_CMD:
		ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
        	if(ioc == NULL) {
          	  	pr_err("Failed to allocate memory.\n");
	            retval = -ENOMEM;
	            break;
        	}
		/*copy command data from user to kernel.*/
		err=copy_from_user(ioc, (struct gf66xx_ioc_transfer*)arg, sizeof(*ioc));
#if DEBUG_LOG
		printk(KERN_ERR "[gf66xx_ioctl] GF66XX_IOC_CMD [copy_from_user] , [ioc->len]=%d,[ioc->buf]address=%x,ioc->cmd=%x\n",(int)ioc->len, ioc->buf,ioc->cmd);
#endif
		if(err){
			printk(KERN_ERR "[gf66xx_ioctl] err of [copy_from_user] is =%d ,Failed to copy command from user to kernel. \n",(int)err);
			retval = -EFAULT;
            		kfree(ioc);
			break;
		}
		    mutex_lock(&gf66xx_dev->buf_lock);
		    gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_1MHZ, 0);
		    if(ioc->cmd == GF66XX_R) {
				/*if want to read data from hardware.*/
				//gf66xx_dbg("Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, ioc->buf);
				gf66xx_dbg("gf66xx_ioctl:Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, (void __user*)((unsigned long)ioc->buf));
				gf66xx_spi_read_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
#if DEBUG_LOG
				printk(KERN_ERR"gf66xx_ioctl: [copy_to_user] ioc->len=%d\n",(int)ioc->len);
				for(i=0;i<ioc->len;i++){
					printk(KERN_ERR"gf66xx_ioctl: [copy_to_user] gf66xx_dev->buffer[%d]=0x%x\n", i,gf66xx_dev->buffer[i]);
					if(ioc->len>16)
						break;
				}
#endif				
				//if(copy_to_user(ioc->buf, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, ioc->len)) {
				err=copy_to_user((void __user*)((unsigned long) ioc->buf), gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, ioc->len);
				if(err) {
					printk(KERN_ERR "[GF66XX]err of [copy_to_user] is= %d, [ioc->buf]address=%p\n",(int)err,(void *)((unsigned long)ioc->buf));
					pr_err("Failed to copy data from kernel to user.\n");
					retval = -EFAULT;
	                     	kfree(ioc);
		            		mutex_unlock(&gf66xx_dev->buf_lock);
					break;
				}
			} else if (ioc->cmd == GF66XX_W) {
				/*if want to read data from hardware.*/		
#if DEBUG_LOG
				temp_buf=(void __user*)(unsigned long)ioc->buf;
				printk(KERN_ERR"gf66xx_ioctl: before [copy_from_user]Write data from 0x%x, len = 0x%x, ioc_buf:0x%x,0x%x,0x%x,0x%x\n", ioc->addr, ioc->len,
				temp_buf[0],
				temp_buf[1],
				temp_buf[2],
				temp_buf[3]);
/*compile error*/
#if 0
				*((void __user*)(unsigned long)ioc->buf), 
                   		*(((void __user*)(unsigned long)ioc->buf)+1),  
                   		*(((void __user*)(unsigned long)ioc->buf)+2),
                   		*(((void __user*)(unsigned long)ioc->buf)+3));
#elif 0
				*(void __user*)ioc->buf, 
                   		*((void __user*)ioc->buf+1),  
                   		*((void __user*)ioc->buf+2),
                   		*((void __user*)ioc->buf+3));
#elif 0
				ioc->buf[0], 
                   		ioc->buf[1], 
                   		ioc->buf[2],
                   		ioc->buf[3]);
#endif
/*compile error*/
#endif
			//if(copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, ioc->buf, ioc->len)){
			err=copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, (void __user*)((unsigned long) ioc->buf), ioc->len);
#if DEBUG_LOG
			printk(KERN_ERR"gf66xx_ioctl:after [copy_from_user]Write data from 0x%x, len = 0x%x\n", ioc->addr, (int)ioc->len);
			for(i=0;i<ioc->len;i++){
				printk(KERN_ERR"gf66xx_ioctl: [copy_from_user] gf66xx_dev->buffer + GF66XX_WDATA_OFFSET[%d]=0x%x\n", i,(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET)[i]);
				if(ioc->len>16)
					break;
			}
#endif			
			if(err){
				printk(KERN_ERR "[GF66XX] err of [copy_from_user] is =%d ,Failed to copy command from user to kernel. \n",(int)err);
				retval = -EFAULT;
               	 	kfree(ioc);
	            		mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}

			gf66xx_spi_write_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
		} else {
			pr_warn("Error command for GF66XX.\n");
		}
		if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
		}
	    mutex_unlock(&gf66xx_dev->buf_lock);
		break;
	case GF66XX_IOC_REINIT:
		gf66xx_irq_close(gf66xx_dev->spi->irq);
        gf66xx_hw_reset(gf66xx_dev, 60);
		gf66xx_irq_open(gf66xx_dev->spi->irq);
		break;
    case GF66XX_IOC_STOPTIMER:
#if ESD_PROTECT
        del_timer_sync(&gf66xx_dev->gf66xx_timer);
#endif
        break;
    case GF66XX_IOC_STARTTIMER:
#if ESD_PROTECT
        gf66xx_dev->gf66xx_timer.expires = jiffies + 2*HZ;
        add_timer(&gf66xx_dev->gf66xx_timer);
#endif
        break;
	default:
		pr_warn("gf66xx doesn't support this command(%d)\n", (int)cmd);
		break;
	}
	FUNC_EXIT();
	return retval;
}

#ifdef CONFIG_COMPAT
static long
gf66xx_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return gf66xx_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define  gf66xx_compat_ioctl        NULL
#endif
static unsigned int gf66xx_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	u8 status = 0;
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &status);
	if((status & GF66XX_BUF_STA_MASK) == GF66XX_BUF_STA_READY) {
		return (POLLIN|POLLRDNORM);
	} else {
		gf66xx_dbg("Poll no data.\n");
	}
	return 0;
}

static void gf66xx_irq(void)
{
#if 0
	struct gf66xx_dev *gf66xx_dev = &gf66xx;
			if(gf66xx_dev->async) {
				kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
			}
#else
    irq_flag = 1;
//    pr_info("gf66xx: in irq, thread_flag = %d, suspend_flag = %d\n", thread_flag, suspend_flag);
    wake_up_interruptible(&waiter);
#endif
}

// irq_thread_callback_func
static int gf66xx_irq_work(void *data)
{
	struct gf66xx_dev* gf66xx_dev = &gf66xx;
    struct sched_param param = {.sched_priority = REG_RT_PRIO(1)};
	unsigned char status;
	unsigned char mode = 0x80;

    sched_setscheduler(current, SCHED_RR, &param);
    thread_flag = 0;
    do{

        gf66xx_dbg("gf66xx: wait event.\n");
        wait_event_interruptible(waiter, ((irq_flag!=0)&&(suspend_flag != 1)));
       
        gf66xx_irq_close(gf66xx_dev->spi->irq);
        irq_flag = 0;

    gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &status);
    if(!(status & GF66XX_BUF_STA_MASK)) {
        pr_info("gf66xx:BAD IRQ = 0x%x\n", status);
        gf66xx_irq_open(gf66xx_dev->spi->irq);
        continue;
    }

    if(!(status & (GF66XX_IMAGE_MASK | GF66XX_KEY_MASK))) {
        gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, (status & 0x7F));
        pr_info("gf66xx:Invalid IRQ = 0x%x\n", status);
        gf66xx_irq_open(gf66xx_dev->spi->irq);
        continue;
    }
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &mode);
	pr_info("gf66xx:status = 0x%x, mode = 0x%x\n", status, mode);
	switch(mode)
		{
		case GF66XX_FF_MODE:
            if((status & GF66XX_HOME_KEY_MASK) && (status & GF66XX_HOME_KEY_STA)){
                pr_info("gf66xx: wake device.\n");
			    gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, 0x00);
			    input_report_key(gf66xx_dev->input, GF66XX_FF_KEY, 1);
			    input_sync(gf66xx_dev->input);			
			    input_report_key(gf66xx_dev->input, GF66XX_FF_KEY, 0);
			    input_sync(gf66xx_dev->input);
            } else {
                break;
            }
        
		case GF66XX_IMAGE_MODE:
			#ifdef GF66XX_FASYNC
			if(gf66xx_dev->async) {
				kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
			}
			#endif
			break;
		case GF66XX_KEY_MODE:
			gf66xx_dbg("gf66xx:Key mode: status = 0x%x\n", status);
			if  ((status & GF66XX_KEY_MASK) && (status & GF66XX_BUF_STA_MASK)) {
                if (status & GF66XX_HOME_KEY_MASK) {
				input_report_key(gf66xx_dev->input, GF66XX_INPUT_HOME_KEY, (status & GF66XX_HOME_KEY_STA)>>4);
			    	input_sync(gf66xx_dev->input);
                }

                else if (status & GF66XX_MENU_KEY_MASK){
				    input_report_key(gf66xx_dev->input, GF66XX_INPUT_MENU_KEY, (status & GF66XX_MENU_KEY_STA)>>2);
			    	input_sync(gf66xx_dev->input);

                }else if (status & GF66XX_MENU_KEY_MASK){
				    input_report_key(gf66xx_dev->input, GF66XX_INPUT_BACK_KEY, (status & GF66XX_BACK_KEY_STA));
			    	input_sync(gf66xx_dev->input);
                }

			}
			gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, (status & 0x7F));
			break;
		case GF66XX_SLEEP_MODE:
			pr_warn("gf66xx:Should not happen in sleep mode.\n");
			break;
        case GF66XX_DEBUG_MODE:
            #ifdef GF66XX_FASYNC
			if(gf66xx_dev->async) {
				kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
			}
			#endif
            break;
		default:
			pr_warn("gf66xx:Unknown mode. mode = 0x%x\n", mode);
			break;
			
		}
            
        gf66xx_irq_open(gf66xx_dev->spi->irq);
    } while(!kthread_should_stop());
    thread_flag = 1;
    pr_info("gf66xx:thread finished.\n");
    return 0;
}

static int gf66xx_open(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = -ENXIO;

    u8 version[16] = {0};
	FUNC_ENTRY();

	mutex_lock(&device_list_lock);
	list_for_each_entry(gf66xx_dev, &device_list, device_entry) {
		if(gf66xx_dev->devt == inode->i_rdev) {
			pr_info("gf66xx:device Found\n");
			status = 0;
			break;
		}
	}

	if(status == 0){
		mutex_lock(&gf66xx_dev->buf_lock);
		if( gf66xx_dev->buffer == NULL) {
			gf66xx_dev->buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(bufsiz+2*GF66XX_RDATA_OFFSET));
			//kzalloc(bufsiz + 2*GF66XX_RDATA_OFFSET, GFP_KERNEL);
			if(gf66xx_dev->buffer == NULL) {
				dev_dbg(&gf66xx_dev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
            pr_info("gf66xx: alloc memory.\n");
		}
		mutex_unlock(&gf66xx_dev->buf_lock);
		if(status == 0) {
			gf66xx_dev->users++;
			filp->private_data = gf66xx_dev;
			nonseekable_open(inode, filp);
			pr_info("gf66xx:Succeed to open device. irq = %d\n", (int)gf66xx_dev->spi->irq);
            if(gf66xx_dev->users == 1)
                gf66xx_irq_open(gf66xx_dev->spi->irq);
		}
	} else {
		pr_info("gf66xx:No device for minor %d\n", (int)iminor(inode));
	}

	mutex_unlock(&device_list_lock);

    gf66xx_spi_read_bytes(gf66xx_dev,0x8000,16,gf66xx_dev->buffer);
    memcpy(version, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, 16);

    gf66xx_dbg("version:%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x\n", 
    version[0], version[1],version[2],version[3],version[4],version[5]
   ,version[6],version[7]);


	FUNC_EXIT();

	return status;
}

#ifdef GF66XX_FASYNC
static int gf66xx_fasync(int fd, struct file *filp, int mode)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	int ret;

	FUNC_ENTRY();
	ret = fasync_helper(fd, filp, mode, &gf66xx_dev->async);
	FUNC_EXIT();
	gf66xx_dbg("ret = %d\n", ret);
	return ret;
}
#endif

static int gf66xx_release(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf66xx_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close??*/
	gf66xx_dev->users --;
	if(!gf66xx_dev->users) {
		
		gf66xx_dbg("disble_irq. irq = %d\n", (int)gf66xx_dev->spi->irq);
		gf66xx_irq_close(gf66xx_dev->spi->irq);

        pr_info("gf66xx:release.\n");
//		kfree(gf66xx_dev->buffer);
//		gf66xx_dev->buffer = NULL;
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

#ifdef CHARGER_DETECT
static int usb_charger_notify(struct notify_block *nb, unsigned long val, void *v)
{
    struct gf66xx_dev *gf66xx_dev = container_of(nb, struct gf66xx_dev, nb);
    pr_info("BEN: last_event:%d, val=%d\n", (int)gf66xx_dev->phy->last_event, (int)val);
    /*
    switch(gf66xx_dev->phy->last_event) {
        case USB_EVENT_NONE:
            pr_info("BEN: USB_EVENT_NONE.\n");
            break;
        case USB_EVENT_VBUS:
            pr_info("BEN: USB_EVENT_VBUS.\n");
            break;
        case USB_EVENT_ID:
            pr_info("BEN: USB_EVENT_ID.\n");
            break;
        case USB_EVENT_CHARGER:
            pr_info("BEN: USB_EVENT_CHARGER.\n");
            break;
        case USB_EVENT_ENUMERATED:
            pr_info("BEN: USB_EVENT_ENUMERATED.\n");
            break;
    }
    */
    return NOTIFY_OK;
}
#endif // CHARGER_DETECT

static const struct file_operations gf66xx_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	gf66xx_write,
	.read =		gf66xx_read,
	.unlocked_ioctl = gf66xx_ioctl,
        .compat_ioctl   = gf66xx_compat_ioctl,
	.open =		gf66xx_open,
	.release =	gf66xx_release,
	.poll   = gf66xx_poll,
#ifdef GF66XX_FASYNC
	.fasync = gf66xx_fasync,
#endif
};

#ifdef SPI_BUS_DYNAMIC

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
    .modalias=SPI_DEV_NAME,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.controller_data=&spi_conf_mt65xx,//&spi_conf
	},
};
#endif


#if FW_UPDATE
static int isUpdate(struct gf66xx_dev *gf66xx_dev)
{
	
	unsigned char version[16]; 
	unsigned int ver_fw = 0; 
	unsigned int ver_file = 0; 
	unsigned char* fw = GF66XX_FW; 
	unsigned char fw_running = 0; 
	const char OFFSET = 7; 

	
	printk(KERN_ERR"gf66xx isUpdate!! \n");
	msleep(300); 
	gf66xx_spi_read_byte(gf66xx_dev, 0x41e4, &fw_running); 
	printk("%s: 0x41e4 = 0x%x\n", __func__, fw_running); 
	if(fw_running == 0xbe) { 
		/*firmware running*/ 
		ver_file = (int)(fw[12] & 0xF0) <<12; 
		ver_file |= (int)(fw[12] & 0x0F)<<8; 
		ver_file |= fw[13]; //get the fw version in the i file; 
	
		/*In case we want to upgrade to a special firmware. Such as debug firmware.*/ 
		if(ver_file != 0x5a5a) { 
			mutex_lock(&gf66xx_dev->buf_lock); 
			gf66xx_spi_read_bytes(gf66xx_dev,0x8000,16,gf66xx_dev->buffer); 
			memcpy(version, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, 16); 
			mutex_unlock(&gf66xx_dev->buf_lock); 
			if(memcmp(version, GF66XX_PID, 6)) { 
				printk("jason add version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1], 
				version[2], version[3], version[4], version[5]); 
				return 1; 
			} 
			if((version[OFFSET]>9) || ((version[OFFSET + 1])>9)) { 
				printk("version: 8-0x%x; 9-0x%x\n", version[OFFSET], version[OFFSET + 1]); 
				return 1; 
			} 
	
			//get the current fw version 
			ver_fw = (unsigned int)version[OFFSET] << 16; 
			ver_fw |= (unsigned int)version[OFFSET + 1] << 8; 
			ver_fw |= (unsigned int)version[OFFSET + 2]; 
			printk("ver_fw: 0x%06x; ver_file:0x%06x\n", ver_fw, ver_file); 
	
			if(ver_fw == ver_file){ 
			/*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/ 
				return 0; 
			} 
		} 
		printk("Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file); 
	}else { 
		/*no firmware.*/ 
		printk("No running firmware. Value = 0x%x\n", fw_running); 
	} 
	return 1; 
	
}

static int gf66xx_fw_update_init(struct gf66xx_dev *gf66xx_dev)
{
	u8 retry_cnt = 5;
	u8 value[2];
	while(retry_cnt--)
	{
        gf66xx_hw_reset(gf66xx_dev, 5);
		/*4.Hold SS51 and DSP(0x4180 == 0x0C)*/
		gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0x0c;
		gf66xx_spi_write_bytes(gf66xx_dev, 0x4180, 1, gf66xx_dev->buffer);
		gf66xx_spi_read_bytes(gf66xx_dev, 0x4180,1, gf66xx_dev->buffer);
		value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		gf66xx_spi_read_bytes(gf66xx_dev, 0x4030,1, gf66xx_dev->buffer);
		value[1] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		pr_info("[info] %s hold SS51 and DSP,0x4180=0x%x,0x4030=0x%x,retry_cnt=%d !\n",__func__,value[0] ,value[1],(int)retry_cnt);
		if (value[0] == 0x0C)/* && value[1] == 0*/
		{
			pr_info("[info] %s hold SS51 and DSP successfully!\n",__func__);
			break;
		}
	}
	pr_info("Hold retry_cnt=%d\n",(int)retry_cnt);
	/*5.enable DSP and MCU power(0x4010 == 0x00)*/
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0x0;
	gf66xx_spi_write_bytes(gf66xx_dev, 0x4010, 1, gf66xx_dev->buffer);
	return 1;
}
#endif

#if ESD_PROTECT
static void gf66xx_timer_work(struct work_struct *work)
{
	unsigned char value[3];
	int ret = 0;
	struct gf66xx_dev *gf66xx_dev;
#if FW_UPDATE
	unsigned char* p_fw = GF66XX_FW;
#endif
	if(work == NULL)
	{
		pr_info("[info] %s wrong work\n",__func__);
		return;
	}
    ret = power_supply_is_system_supplied();
    pr_info("BEN: power_supply ret = %d\n", ret);
#if 0
	gf66xx_dev = container_of(work, struct gf66xx_dev, spi_work);
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->spi->max_speed_hz= 1000*1000;
	spi_setup(gf66xx_dev->spi);
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8040,1, gf66xx_dev->buffer);
	value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 1, gf66xx_dev->buffer);
	value[1] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8046, 1, gf66xx_dev->buffer); //&& value[1] == 0x47 && value[2] == 0x56
	value[2] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	if(value[0] == 0xC6&& value[1] == 0x47){
		//printk("######Jason no need to kick dog 111!\n");
		gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0xAA;
		gf66xx_spi_write_bytes(gf66xx_dev, 0x8040, 1, gf66xx_dev->buffer);
	}else{
        unsigned char version[16] = {0};
        pr_info("hardware works abnormal, do reset! 0x8040 = 0x%x  0x8000 = 0x%x 0x8046 = 0x%x \n",value[0],value[1],value[2]);
        gf66xx_irq_close(gf66xx_dev->spi->irq);
        gf66xx_hw_reset(gf66xx_dev, 360);

        gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
        value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
        gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 16, gf66xx_dev->buffer);
        memcpy(version, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, 16);
        pr_info("[info] %s read 0x41e4 finish value = 0x%x, version[0]=%x\n", __func__,value[0], version[0]);
        if((value[0] != 0xbe) || memcmp(version, GF66XX_PID, 6)) {
            gf66xx_power_off(gf66xx_dev);
            mdelay(10);
            ret = gf66xx_power_on(gf66xx_dev);
            if(ret)
                pr_info("[info] %s power on fail\n", __func__);
        }
#if FW_UPDATE
        if((value[0] != 0xbe) || memcmp(version, GF66XX_PID, 6)) {
            gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
            value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
            if(value[0] != 0xbe) {
                /***********************************firmware update*********************************/
                pr_info("[info] %s firmware update start\n", __func__);
                del_timer_sync(&gf66xx_dev->gf66xx_timer);
                gf66xx_fw_update_init(gf66xx_dev);
                ret = gf66xx_fw_update(gf66xx_dev, p_fw, FW_LENGTH);
                gf66xx_hw_reset(gf66xx_dev, 60);
                gf66xx_dev->gf66xx_timer.expires = jiffies + 2 * HZ;
                add_timer(&gf66xx_dev->gf66xx_timer);
            }
        }
#endif
#if CFG_UPDATE 
        /***************************************update config********************************/
        pr_info("[info] %s write config \n", __func__);
        gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0xAA;
        ret = gf66xx_spi_write_bytes(gf66xx_dev, 0x8040, 1, gf66xx_dev->buffer);
        if(!ret)
            pr_info("[info] %s write 0x8040 fail\n", __func__);
        memcpy(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, gf66xx_config, GF66XX_CFG_LEN);
        ret = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
        if(ret <= 0)
            pr_info("[info] %s write config fail\n", __func__);
#endif
        gf66xx_irq_open(gf66xx_dev->spi->irq);
    }
    mutex_unlock(&gf66xx_dev->buf_lock);
#endif //if 0
}

static void gf66xx_timer_func(unsigned long arg)
{
	struct gf66xx_dev *gf66xx_dev = (struct gf66xx_dev*)arg;
	if(gf66xx_dev == NULL)
	{
		pr_info("[info] %s can't get the gf66xx_dev\n",__func__);
		return;
	}
	schedule_work(&gf66xx_dev->spi_work);
	mod_timer(&gf66xx_dev->gf66xx_timer, jiffies + 2*HZ);
}
#endif

static struct spi_device_id spi_id_table = {"goodix_fp", 0};
static struct spi_driver gf66xx_spi_driver = {
	.driver = {
		.name =	SPI_DEV_NAME,
		.owner =THIS_MODULE,
	},
        .id_table = &spi_id_table,
	.probe = gf66xx_probe,
      //  .suspend = gf66xx_suspend,
      //  .resume = gf66xx_resume,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	.remove =__devexit_p(gf66xx_remove),
#else
        .remove =gf66xx_remove,
#endif
	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

#ifdef CONFIG_HAS_EARLYSUSPEND
  static void gf318m_early_suspend( struct early_suspend *h )
  {
 struct gf66xx_dev *gf66xx_dev = &gf66xx;
 printk("gf318m_early_suspend\n");
#if 0
 gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, GF66XX_SLEEP_MODE);	  // 进入Sleep mode
#else
	 mt_set_gpio_mode(gf66xx_dev->irq_gpio, GPIO_MODE_00);// disable interrupt 
	 mt_set_gpio_dir(gf66xx_dev->irq_gpio, GPIO_DIR_OUT); 
	 mt_set_gpio_out(gf66xx_dev->irq_gpio, GPIO_OUT_ZERO); 
	 msleep(10);
#if 1
	 gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, GF66XX_SLEEP_MODE);	  // 进入Sleep mode 
#else
 mt_set_gpio_out(gf66xx_dev->reset_gpio, GPIO_OUT_ZERO);
 hwPowerDown(MT6331_POWER_LDO_VMCH, "GF518M");
#endif
#endif
 } 

 
 static void gf318m_late_resume( struct early_suspend *h )
  {
 unsigned char mode = 0x80;
 struct gf66xx_dev *gf66xx_dev = &gf66xx;
 printk("gf318m_early_resume\n");
 
#if  1  
	 mt_set_gpio_mode(gf66xx_dev->irq_gpio, GF66XX_SPI_EINT_PIN_M_EINT); 
	 mt_set_gpio_dir(gf66xx_dev->irq_gpio, GPIO_DIR_IN); 
	 mt_set_gpio_pull_enable(gf66xx_dev->irq_gpio, GPIO_PULL_DISABLE); 
	 gf66xx_hw_reset(gf66xx_dev, 60);  // reset the fp sensor  
#else
 hwPowerOn(MT6331_POWER_LDO_VMCH, VOL_3000, "GF518M");
	  msleep(20);
	 mt_set_gpio_mode(gf66xx_dev->irq_gpio, GF66XX_SPI_EINT_PIN_M_EINT); 
	 mt_set_gpio_dir(gf66xx_dev->irq_gpio, GPIO_DIR_IN); 
	 mt_set_gpio_pull_enable(gf66xx_dev->irq_gpio, GPIO_PULL_DISABLE); 
 msleep(10);
	 gf66xx_hw_reset(gf66xx_dev, 60);  // reset the fp sensor  
 
#endif
 }


static struct early_suspend gf318m_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1,
	.suspend = gf318m_early_suspend,
	.resume = gf318m_late_resume,
};
#endif

#if CFG_UPDATE 
int gf66xx_config_update(struct gf66xx_dev *gf66xx_dev)
{
	int rery;
	int ret;
	unsigned char tmp;
	unsigned char buffer[16]={0};
	unsigned char readBuf[256];
	
	printk(KERN_ERR"gf66xx_config_update!! \n");
	
	//init Vendor ID mode		
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET]=0x08;
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET+1]=0xF8;	
	gf66xx_spi_write_bytes(gf66xx_dev, 0x8041, 2, gf66xx_dev->buffer);
	
	rery = 10;
	while(rery--){
		gf66xx_spi_read_byte(gf66xx_dev, 0x8042, &tmp);
		if (0xAA == tmp){
			printk("gf66xx: %s, read Vendor ID\n", __func__);
			break;
		}
		printk("gf66xx: read 0x8042=0x%02X retry=%d\n", tmp, rery);
		msleep(5);
	}

	//enter Vendor ID mode
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET]=0x0C;
	gf66xx_spi_write_bytes(gf66xx_dev, 0x5094, 1, gf66xx_dev->buffer);
	
	rery = 10;
	while(rery--){
		gf66xx_spi_read_byte(gf66xx_dev, 0x5094, &tmp);
		if (0 == tmp){
			printk("gf66xx: %s, read Vendor ID is ready!!!\n", __func__);
			break;
		}
		printk("gf66xx: read 0x5094=0x%02X retry=%d\n", tmp, rery);
		msleep(5);
	}
	
	memset(product_id_name, 0, 5);
	//read Vendor ID
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8148, 10, buffer);
	if(!memcmp(buffer+GF66XX_RDATA_OFFSET, ofilm_vendor_id, 1))
	{
		printk("gf66xx: %s, read Vendor ID is ofilm !!!\n", __func__);
		memcpy(gf66xx_dev->config + GF66XX_WDATA_OFFSET, gf66xx_config_ofilm, GF66XX_CFG_LEN);
		memcpy(product_id_name, "ofilm", 5);
	}
	else if (!memcmp(buffer+GF66XX_RDATA_OFFSET, truly_vendor_id, 1))
	{
		printk("gf66xx: %s, read Vendor ID is truly!!!\n", __func__);
		memcpy(gf66xx_dev->config + GF66XX_WDATA_OFFSET, gf66xx_config_truly, GF66XX_CFG_LEN);
		memcpy(product_id_name, "truly", 5);
	}
	else if (!memcmp(buffer+GF66XX_RDATA_OFFSET, toptouch_vendor_id, 1))
	{
		printk("gf66xx: %s, read Vendor ID is toptouch!!!\n", __func__);
		memcpy(gf66xx_dev->config + GF66XX_WDATA_OFFSET, gf66xx_config_toptouch, GF66XX_CFG_LEN);
		memcpy(product_id_name, "touch", 5);
	}
	else
	{
		printk("gf66xx: Vendor ID not matched!!! send o-film default 0x%02X\n", buffer[GF66XX_RDATA_OFFSET]);
		memcpy(gf66xx_dev->config + GF66XX_WDATA_OFFSET, gf66xx_config_ofilm, GF66XX_CFG_LEN);
	//	return -1;
	}

	//leave Vendor ID mode
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET]=0x00;
	gf66xx_spi_write_bytes(gf66xx_dev, 0x8041, 1, gf66xx_dev->buffer);	

	/*write config*/
	rery = 6;
	while(rery --) {
			ret = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->config);
			if(ret <= 0) {
				pr_info("[info] %s write config fail\n", __func__);
				continue;
			}
			msleep(10);
			gf66xx_spi_read_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, readBuf);
			int tmp = 0;
			for (tmp = 0; tmp < GF66XX_CFG_LEN - 2; tmp++)
					{
						if (tmp != 0 && tmp != 119 && tmp != 120)
						{
							if (gf66xx_dev->config[tmp] != readBuf[tmp])
							{
								pr_info("[info] %s write config fail,cfg read back is wrong\n", __func__);
								msleep(10);
								break;
							}
						}
					}
			if(tmp==GF66XX_CFG_LEN - 2) {
				pr_info("[info] %s write config success\n", __func__);
				break;
			}
	}

	return 0;
}
#endif

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gf66xx_spi_class;
/*-------------------------------------------------------------------------*/


static ssize_t show_product_id_value(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s\n", product_id_name);            
}

static DEVICE_ATTR(product_id, S_IRUGO|S_IWUSR, show_product_id_value, NULL);

static struct attribute *gf516m_product_id_attrs[] = {
        &dev_attr_product_id.attr,
        NULL
};
static const struct attribute_group gf318_attr_group = {
        .attrs = gf516m_product_id_attrs,
        .name = "product_id"
};
static int gf66xx_probe(struct spi_device *spi)
{
	struct gf66xx_dev	*gf66xx_dev = &gf66xx;
	int			status = -EINVAL;
	unsigned long		minor;
    unsigned char       version = 0;
    char buffer[16]={0};
    int ret = 0;
    u8 j = 0;
	FUNC_ENTRY();

	/* Initialize the driver data */
	gf66xx_dev->spi = spi;
	spin_lock_init(&gf66xx_dev->spi_lock);
	mutex_init(&gf66xx_dev->buf_lock);
	printk("Process step0 \n");
	INIT_LIST_HEAD(&gf66xx_dev->device_entry);
    gf66xx_dev->spi=spi;
    gf66xx_dev->irq_gpio = -EINVAL;
    gf66xx_dev->reset_gpio = -EINVAL;
    gf66xx_dev->power_gpio = -EINVAL;
    printk(KERN_ERR"Process step1 \n");

#if 0
    if(gf66xx_parse_dts(gf66xx_dev) || gf66xx_power_on(gf66xx_dev)) {
        pr_info("gf66xx:Failed to parse dts.\n");
        goto error;
    }
#else

	hwPowerOn(MT6331_POWER_LDO_VMCH, VOL_3000, "GF518M");

	gf66xx_dev->irq_gpio =  GF66XX_INT_PIN;
	gf66xx_dev->reset_gpio =  GF66XX_RST_PIN;
	  
	printk(KERN_ERR"Process step2 \n");
	/*init the reset*/
    ret = mt_set_gpio_mode(GF66XX_RST_PIN, GPIO_MODE_00);
	printk("mt_set_gpio_mode Ok.\n");
	if (ret!= 0) {
		printk(KERN_ERR"[GF318M]gpio_request (reset) failed.\n");
		return ret;
	}
	ret = mt_set_gpio_dir(GF66XX_RST_PIN, GPIO_DIR_OUT);
    if (ret!= 0) {
		printk(KERN_ERR "gpio_direction_output(reset) failed.\n");
		return ret;
	} 
    printk(KERN_ERR"init Reset pin done in gf66xx_parse_dts \n");
	/*init the reset*/

	/*setup SPI added jone*/
	gf66xx_spi_pins_config();
	gf66xx_dev->spi->mode = SPI_MODE_0;
	gf66xx_dev->spi->bits_per_word = 8;
	gf66xx_dev->spi->controller_data = (void*)&spi_conf_mt65xx;
       
	ret=spi_setup(gf66xx_dev->spi);
	if (ret) {
		printk(KERN_ERR "spi_setup failed\n");
	}
	/*setup SPI added jone */
	
#endif    
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf66xx_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf66xx_spi_class, &spi->dev, gf66xx_dev->devt,
				    gf66xx_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf66xx_dev->device_entry, &device_list);
	} else {
        gf66xx_dev->devt = 0;
    }
	mutex_unlock(&device_list_lock);

	if (status == 0){
		gf66xx_dev->buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(bufsiz+2*GF66XX_RDATA_OFFSET));
		//kzalloc(bufsiz + 2*GF66XX_RDATA_OFFSET, GFP_KERNEL);
		if(gf66xx_dev->buffer == NULL) {
			gf66xx_dbg("Failed to allocate data buffer.\n");
			status = -ENOMEM;
			goto error;
		}
		spi_set_drvdata(spi, gf66xx_dev);
	
		/*register device within input system.*/
		gf66xx_dev->input = input_allocate_device();
		printk("Process step3 \n");
		if(gf66xx_dev->input == NULL) {
			gf66xx_dbg("Failed to allocate input device.\n");
			status = -ENOMEM;
			goto error;
		}

		__set_bit(EV_KEY, gf66xx_dev->input->evbit);
		__set_bit(GF66XX_FF_KEY, gf66xx_dev->input->keybit);
		__set_bit(GF66XX_INPUT_MENU_KEY, gf66xx_dev->input->keybit);
		__set_bit(GF66XX_INPUT_BACK_KEY, gf66xx_dev->input->keybit);
		__set_bit(GF66XX_FF_KEY, gf66xx_dev->input->keybit);
		printk("Process step4 \n");
		gf66xx_dev->input->name = GF66XX_INPUT_DEV_NAME;
		if(input_register_device(gf66xx_dev->input)) {
			gf66xx_dbg("Failed to register input device.\n");
		}

        suspend_flag = 0;
        irq_flag = 0;

        printk(KERN_ERR "gf66xx:chip reset.\n");
		
		printk("Process step5 \n");
		/*SPI parameters.*/
		spi->irq = gf66xx_irq_num(gf66xx_dev); //gpio_to_irq(GF66XX_IRQ_PIN);//
		spi->bits_per_word = 8; //?
		spi->controller_data  = (void*)&spi_conf_mt65xx;
		gf66xx_spi_set_mode(gf66xx_dev->spi, SPEED_1MHZ, 0);
		/*SPI parameters.*/

        /*GF66XX hardware initialize.*/
		printk("Process step6 \n");
		gf66xx_hw_reset(gf66xx_dev, 360);
		////////////////////////////////////////////////////////

#if SPI_TEST
		while(1){
			gf66xx_spi_read_byte(gf66xx_dev,0x8000,&version);
			if(version != 0x47)
			{
				pr_err("gf66xx:[ERR] %s device detect error!!\n", __func__);
				status = -ENODEV;	
				goto error;
			}else{
				printk(KERN_ERR" [0x8000]=%2x",version);
				break;
			}
		}
#endif

		gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 16, buffer);
		for(j=0; j<16; j++){
			//gf66xx_spi_read_byte(gf66xx_dev,0x8000 + j,&version);
			   printk(KERN_ERR"GF318M IC Version=%02x\n", buffer[j]);
		}
		/*GF66XX hardware initialize.*/

#if FW_UPDATE
		if(isUpdate(gf66xx_dev)) {
			unsigned char* fw = GF66XX_FW;
			printk(KERN_ERR"gf66xx probe need to FW Update");
			/*Do upgrade action.*/
			gf66xx_fw_update_init(gf66xx_dev);
			gf66xx_fw_update(gf66xx_dev, fw, FW_LENGTH);
			gf66xx_hw_reset(gf66xx_dev, 60);
		}
#endif

#if CFG_UPDATE 
	gf66xx_config_update(gf66xx_dev);

#endif
             
	status = sysfs_create_group(&spi->dev.kobj,&gf318_attr_group);
        if(status){
                printk("gf66xx finger filed to create attr files!\n");
                status = -ENODEV;
                goto error;
        }

        /*install irq handler.*/    
        gf66xx_irq_thread = kthread_run(gf66xx_irq_work, 0, "gf66xx");
        if(IS_ERR(gf66xx_irq_thread)) {
            printk(KERN_ERR"gf66xx: Failed to create kernel thread: %d\n", (int)PTR_ERR(gf66xx_irq_thread));
        }
		gf66xx_irq_setup(gf66xx_dev, gf66xx_irq);
		printk(KERN_ERR "[gf66xx_irq_setup] init done!\n");
#ifdef CHARGER_DETECT
        pr_info("BEN: USB PHY.\n");
        gf66xx_dev->phy = usb_get_transceiver();
        if(gf66xx_dev->phy != NULL) {
            //devm_usb_get_phy(&spi->dev, USB_PHY_TYPE_USB2);
            gf66xx_dev->nb.notifier_call = usb_charger_notify;
            //ret = usb_register_notifier(gf66xx_dev->phy, &gf66xx_dev->nb);
            usb_register_notify(&gf66xx_dev->nb);
            /*
            if(ret) {
                pr_info("BEN:Failed to register usb notify. ret = %d\n", ret);
            } else {
                pr_info("BEN:Sucess to register notifier.\n");
            }
            */
        } else {
            pr_info("BEN: Failed to get USB PHY.\n");
        }
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&gf318m_early_suspend_handler);
#endif

#if ESD_PROTECT
		INIT_WORK(&gf66xx_dev->spi_work, gf66xx_timer_work);
		init_timer(&gf66xx_dev->gf66xx_timer);
		gf66xx_dev->gf66xx_timer.function = gf66xx_timer_func;
		gf66xx_dev->gf66xx_timer.expires = jiffies + 20*HZ;
		gf66xx_dev->gf66xx_timer.data = (unsigned long)gf66xx_dev;
		add_timer(&gf66xx_dev->gf66xx_timer);
#endif	

	}

    return status;
error:
    if(gf66xx_dev->devt != 0) {
        pr_info("gf66xx:Err: status = %d\n", (int)status);
        mutex_lock(&device_list_lock);
        //sysfs_remove_group(&spi->dev.kobj, &gf66xx_debug_attr_group);
        list_del(&gf66xx_dev->device_entry);
        device_destroy(gf66xx_spi_class, gf66xx_dev->devt);
        clear_bit(MINOR(gf66xx_dev->devt), minors);
        mutex_unlock(&device_list_lock);
    }
    if(gf66xx_dev->input != NULL)
        input_unregister_device(gf66xx_dev->input);

    if(gf66xx_dev->buffer != NULL) {
        //kfree(gf66xx_dev->buffer);
        free_pages((unsigned long)gf66xx_dev->buffer, get_order(bufsiz + GF66XX_RDATA_OFFSET));
    }
 
	FUNC_EXIT();
	return status;
}

static int gf66xx_remove(struct spi_device *spi)
{
	struct gf66xx_dev	*gf66xx_dev = spi_get_drvdata(spi);
	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if(gf66xx_dev->spi->irq) {
		gf66xx_irq_release(gf66xx_dev);
	}
    
    if(gf66xx_irq_thread != NULL) {
        irq_flag = 1;
        suspend_flag = 0;
        kthread_stop(gf66xx_irq_thread);
        gf66xx_irq_thread = NULL;
    }

	spin_lock_irq(&gf66xx_dev->spi_lock);
	gf66xx_dev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&gf66xx_dev->spi_lock);
#if ESD_PROTECT
    del_timer_sync(&gf66xx_dev->gf66xx_timer);
#endif

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf66xx_dev->device_entry);
	device_destroy(gf66xx_spi_class, gf66xx_dev->devt);
	clear_bit(MINOR(gf66xx_dev->devt), minors);
    pr_info("gf66xx:remove. users = %d\n", (int)gf66xx_dev->users);
	if (gf66xx_dev->users == 0) {
		if(gf66xx_dev->input != NULL) 
			input_unregister_device(gf66xx_dev->input);

		if(gf66xx_dev->buffer != NULL)
//			kfree(gf66xx_dev->buffer);
			free_pages((unsigned long)gf66xx_dev->buffer,
			   get_order(bufsiz+2*GF66XX_RDATA_OFFSET));
        memset(gf66xx_dev, 0, sizeof(struct gf66xx_dev));
	}
	mutex_unlock(&device_list_lock);

	FUNC_EXIT();
	return 0;
}

static int gf66xx_suspend(struct spi_device *spi, pm_message_t msg)
{
    struct gf66xx_dev *gf66xx_dev = spi_get_drvdata(spi);
    if(gf66xx_dev == NULL) {
        pr_warn("gf66xx: NULL pointer in suspend.\n");
        return -ENODEV;
    }

    pr_info("gf66xx:gf66xx_suspend\n");
    return 0;
}

static int gf66xx_resume(struct spi_device *spi)
{
     struct gf66xx_dev *gf66xx_dev = spi_get_drvdata(spi);
    if(gf66xx_dev == NULL) {
        pr_warn("gf66xx: NULL pointer in resume.\n");
        return -ENODEV;
    }

    pr_info("gf66xx: gf66xx_resume.\n");
    return 0;
}

/*-------------------------------------------------------------------------*/
static int __init gf66xx_init(void)
{
	int status;
	FUNC_ENTRY();
	printk("GF318M init start! %s\n",__func__);
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "gf66xx", &gf66xx_fops);
	if (status < 0){
		gf66xx_dbg("Failed to register char device!\n");
		FUNC_EXIT();
		return status;
	}
	   
	gf66xx_spi_class = class_create(THIS_MODULE, "gf66xx_spi");
	if (IS_ERR(gf66xx_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		gf66xx_dbg("Failed to create class.\n");
		FUNC_EXIT();
		return PTR_ERR(gf66xx_spi_class);
	}

#ifdef SPI_BUS_DYNAMIC
	/*added jone at 2015-0512*/
	gf66xx_dbg("SPI_BUS_DYNAMIC:register spi_board_info!\n");
	spi_register_board_info(spi_board_devs,ARRAY_SIZE(spi_board_devs));
	/*added jone at 2015-0512*/
#endif

	status = spi_register_driver(&gf66xx_spi_driver);
	if (status < 0) {
		class_destroy(gf66xx_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		gf66xx_dbg("Failed to register SPI driver.\n");
	}
	FUNC_EXIT();
	return status;
}
module_init(gf66xx_init);

static void __exit gf66xx_exit(void)
{
       FUNC_ENTRY();
	spi_unregister_driver(&gf66xx_spi_driver);
	class_destroy(gf66xx_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
	FUNC_EXIT();
}
module_exit(gf66xx_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:gf66xx-spi");

