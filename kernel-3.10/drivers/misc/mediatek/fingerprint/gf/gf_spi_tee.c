/* Goodix's GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF3208/GF5216
 *  fingerprint sensor linux driver for TEE
 *
 * 2010 - 2015 Goodix Technology.
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
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
/*#include <mach/irqs.h>*/
#include <linux/completion.h>
#include <linux/gpio.h>
/*#include <mach/gpio.h>*/
/*#include <plat/gpio-cfg.h>*/
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*#include <mach/hardware.h>*/
/*#include <mach/mt_irq.h>*/
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_spi.h>
#include <mach/upmu_common.h>
#include <mach/eint.h>
#include <mach/gpio_const.h>

#include <mach/memory.h>
#include <mach/mt_clkmgr.h>
#include <net/sock.h>

#include <cust_eint.h>
#include <cust_gpio_usage.h>

#include "gf_spi_tee.h"

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */

/*spi device name*/
#define SPI_DEV_NAME   "gf_spi"
/*device name after register in charater*/
#define DEV_NAME "goodix_fp"

#define CHRD_DRIVER_NAME		"goodix_fp"
#define CLASS_NAME			"goodix_fp"
#define SPIDEV_MAJOR			158	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define GF_SPI_VERSION "gf_spi_tee_v0.9"

/**************************feature control******************************/
/* #define GF_FASYNC		1 */  /* If support fasync mechanism */
#define GF_NETLINK		1 /* If support netlink mechanism */

#define GF_NETLINK_ROUTE 27   /* for GF test only, need defined in include/uapi/linux/netlink.h */
#define MAX_NL_MSG_LEN 16

/***********************GPIO setting port layer*************************/
/* customer hardware port layer, please change according to customer's hardware */
#ifndef GPIO_FP_INT_PIN
#define GPIO_FP_INT_PIN                 (GPIO3 | 0x80000000)
#define GPIO_FP_INT_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_INT_PIN_M_EINT  GPIO_FP_INT_PIN_M_GPIO
#endif

#define GPIO_FP_SPICLK_PIN		   (GPIO166 | 0x80000000)
#define GPIO_FP_SPICLK_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPICLK_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPICLK_PIN_M_SPI_CK   GPIO_MODE_01

#define GPIO_FP_SPIMISO_PIN			(GPIO167 | 0x80000000)
#define GPIO_FP_SPIMISO_PIN_M_GPIO	GPIO_MODE_00
#define GPIO_FP_SPIMISO_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPIMISO_PIN_M_SPI_MI   GPIO_MODE_01

#define GPIO_FP_SPIMOSI_PIN			(GPIO168 | 0x80000000)
#define GPIO_FP_SPIMOSI_PIN_M_GPIO	GPIO_MODE_00
#define GPIO_FP_SPIMOSI_PIN_M_MDEINT  GPIO_MODE_02
#define GPIO_FP_SPIMOSI_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPIMOSI_PIN_M_SPI_MO   GPIO_MODE_01

#define GPIO_FP_SPICS_PIN		  (GPIO169 | 0x80000000)
#define GPIO_FP_SPICS_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPICS_PIN_M_MDEINT	GPIO_MODE_02
#define GPIO_FP_SPICS_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPICS_PIN_M_SPI_CS	 GPIO_MODE_01

/* GPIO122 used for VOLUMN_UP & VOLUMN_DOWN, change to GPIO8 temporary */
//#define GPIO_FP_RESET_PIN		  (GPIO122 | 0x80000000)
#define GPIO_FP_RESET_PIN                 (GPIO192 | 0x80000000)
#define GPIO_FP_RESET_PIN_M_GPIO  GPIO_MODE_00

#define GPIO_FP_SPI_BYPASS_PIN		   (GPIO133 | 0x80000000)
#define GPIO_FP_SPI_BYPASS_M_GPIO  GPIO_MODE_00

#ifndef	CUST_EINT_FP_EINT_NUM
#define CUST_EINT_FP_EINT_NUM                      3
#define CUST_EINT_FP_EINT_DEBOUNCE_CN	   0
#define CUST_EINT_FP_EINT_TYPE			   EINTF_TRIGGER_RISING /* EINTF_TRIGGER_LOW */
#define CUST_EINT_FP_EINT_DEBOUNCE_EN	   0  /* CUST_EINT_DEBOUNCE_DISABLE */
#endif

#ifndef GF_INPUT_HOME_KEY
/* on MTK 6795 EVB board, home key has been redefine to KEY_HOMEPAGE!!! */
/* double check the define on customer board!!! */
#define GF_INPUT_HOME_KEY KEY_HOME/*KEY_HOME*///KEY_HOMEPAGE

#define GF_INPUT_MENU_KEY  KEY_MENU
#define GF_INPUT_BACK_KEY  KEY_BACK

#define GF_INPUT_NAV_UP_KEY  KEY_UP
#define GF_INPUT_NAV_DOWN_KEY  KEY_DOWN
#define GF_INPUT_NAV_LEFT_KEY  KEY_LEFT
#define GF_INPUT_NAV_RIGHT_KEY  KEY_RIGHT
#define GF_INPUT_CAMERA_KEY  KEY_CAMERA

#define GF_INPUT_FF_KEY  KEY_POWER

#define GF_INPUT_OTHER_KEY KEY_VOLUMEDOWN  /* temporary key value for capture use */
#endif

/*************************************************************/
struct gf_dev *g_gf_dev = NULL;

/* align=2, 2 bytes align */
/* align=4, 4 bytes align */
/* align=8, 8 bytes align */
#define ROUND_UP(x, align)		((x+(align-1))&~(align-1))

/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)

/* debug log setting */
u8 g_debug_level = DEBUG_LOG;

#define gf_debug(level, fmt, args...) do { \
			if (g_debug_level >= level) {\
				printk(KERN_ERR "[gf] " fmt, ##args); \
			} \
		} while (0)

#define FUNC_ENTRY()  gf_debug(DEBUG_LOG, "%s, %d, entry\n", __func__, __LINE__)
#define FUNC_EXIT()  gf_debug(DEBUG_LOG, "%s, %d, exit\n", __func__, __LINE__)

/*************************************************************/
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = (150*150);

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");


/* -------------------------------------------------------------------- */
/* timer function								*/
/* -------------------------------------------------------------------- */
#define TIME_START	   0
#define TIME_STOP	   1

static long int prev_time, cur_time;

long int kernel_time(unsigned int step)
{
	cur_time = ktime_to_us(ktime_get());
	if (step == TIME_START) {
		prev_time = cur_time;
		return 0;
	} else if (step == TIME_STOP) {
		gf_debug(DEBUG_LOG, "%s, use: %ld us\n", __func__, (cur_time - prev_time));
		return cur_time - prev_time;
	} else {
		prev_time = cur_time;
		return -1;
	}
}

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration								  */
/* -------------------------------------------------------------------- */
static void gf_hw_power_enable(u8 bonoff)
{
	// TODO: LDO configure
#if 1
	if (bonoff)
            hwPowerOn(MT6331_POWER_LDO_VMCH, VOL_3000, "GF518M");
	else
            hwPowerDown(MT6331_POWER_LDO_VMCH, "GF518M");
#endif
	return;
}

static void gf_spi_clk_enable(u8 bonoff)
{
#if 1
	if (bonoff)
		enable_clock(MT_CG_PERI_SPI0, "spi");
	else
		disable_clock(MT_CG_PERI_SPI0, "spi");
#endif
	return;
}


static void gf_bypass_flash_gpio_cfg(void)
{
	/* TODO: by pass flash IO config, default connect to GND */
	return;
}


/*Confure gpio for SPI if necessary*/
static void gf_spi_gpio_cfg(void)
{
	mt_set_gpio_mode(GPIO_FP_SPICLK_PIN, GPIO_FP_SPICLK_PIN_M_SPI_CK);
	mt_set_gpio_mode(GPIO_FP_SPIMISO_PIN, GPIO_FP_SPIMISO_PIN_M_SPI_MI);
	mt_set_gpio_mode(GPIO_FP_SPIMOSI_PIN, GPIO_FP_SPIMOSI_PIN_M_SPI_MO);
	mt_set_gpio_mode(GPIO_FP_SPICS_PIN, GPIO_FP_SPICS_PIN_M_SPI_CS);
	return;
}

/* only configure pull up or pull disable, not change functioin mode! */
static void gf_miso_gpio_cfg(u8 pullhigh)
{
	if (pullhigh) {
		mt_set_gpio_pull_select(GPIO_FP_SPIMISO_PIN, GPIO_PULL_UP);
		mt_set_gpio_pull_enable(GPIO_FP_SPIMISO_PIN, GPIO_PULL_ENABLE);
	} else {
		mt_set_gpio_pull_enable(GPIO_FP_SPIMISO_PIN, GPIO_PULL_DISABLE);
	}
	return;
}


static void gf_irq_gpio_cfg(void)
{
	mt_set_gpio_mode(GPIO_FP_INT_PIN, GPIO_FP_INT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_FP_INT_PIN, GPIO_PULL_DISABLE);
	/* mt_set_gpio_dir(GPIO_FP_INT_PIN, GPIO_DIR_IN); */
	/* mt_eint_set_hw_debounce(CUST_EINT_FP_EINT_NUM, CUST_EINT_FP_EINT_DEBOUNCE_CN); */

#if 0
	mt_eint_registration(CUST_EINT_FP_EINT_NUM, CUST_EINT_FP_EINT_TYPE, gf_irq, 0);
	mt_eint_unmask(CUST_EINT_FP_EINT_NUM);
#endif

	return;
}

static void gf_reset_gpio_cfg(void)
{
	mt_set_gpio_mode(GPIO_FP_RESET_PIN, GPIO_FP_RESET_PIN_M_GPIO);
	mt_set_gpio_pull_select(GPIO_FP_RESET_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(GPIO_FP_RESET_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_FP_RESET_PIN, GPIO_DIR_OUT);

	return;
}

/* delay ms after reset */
static void gf_hw_reset(u8 delay)
{
	mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ZERO);
	mdelay(5);
	mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ONE);
	if (delay) {
		/* delay is configurable */
		mdelay(delay);
	}
	return;
}

static void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (0 == gf_dev->device_available) {
		gf_debug(ERR_LOG, "%s, devices not available\n", __func__);
	} else {
		if (1 == gf_dev->irq_count) {
			gf_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
		} else {
			enable_irq(gf_dev->spi->irq);
			gf_dev->irq_count = 1;
			gf_debug(DEBUG_LOG, "%s enable interrupt!\n", __func__);
		}
	}
	return;
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (0 == gf_dev->device_available) {
		gf_debug(ERR_LOG, "%s, devices not available\n", __func__);
	} else {
		if (0 == gf_dev->irq_count) {
			gf_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
		} else {
			disable_irq(gf_dev->spi->irq);
			gf_dev->irq_count = 0;
			gf_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
		}
	}
	return;
}

/* -------------------------------------------------------------------- */
/* netlink functions		     */
/* -------------------------------------------------------------------- */

void gf_netlink_send(const int command)
{
	struct nlmsghdr *nlh;
	struct sk_buff *skb;
	int ret;

	gf_debug(INFO_LOG, "[%s] : enter, send command %d\n", __func__, command);
	if (NULL == g_gf_dev->nl_sk) {
		gf_debug(ERR_LOG, "[%s] : invalid socket\n", __func__);
		return;
	}

	if (0 == g_gf_dev->pid) {
		gf_debug(ERR_LOG, "[%s] : invalid native process pid\n", __func__);
		return;
	}

	/*alloc data buffer for sending to native*/
	/*malloc data space at least 1500 bytes, which is ethernet data length*/
	skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL) {
		gf_debug(ERR_LOG, "[%s] : allocate skb failed\n", __func__);
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		gf_debug(ERR_LOG, "[%s] : nlmsg_put failed\n", __func__);
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = command;
	ret = netlink_unicast(g_gf_dev->nl_sk, skb, g_gf_dev->pid, MSG_DONTWAIT);
	if (ret == 0) {
		gf_debug(ERR_LOG, "[%s] : send failed\n", __func__);
		return;
	} else {
		gf_debug(INFO_LOG, "[%s] : send done, data length is %d\n", __func__, ret);
		return;
	}
}

static void gf_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char str[128];

	gf_debug(INFO_LOG, "[%s] : enter\n", __func__);

	skb = skb_get(__skb);
	if (skb == NULL) {
		gf_debug(ERR_LOG, "[%s] : skb_get return NULL\n", __func__);
		return;
	}

	/*presume there is 5byte payload at leaset*/
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		g_gf_dev->pid = nlh->nlmsg_pid;
		gf_debug(INFO_LOG, "[%s] : pid: %d, msg: %s\n", __func__, g_gf_dev->pid, str);

	} else {
		gf_debug(ERR_LOG, "[%s] : not enough data length\n", __func__);
	}

	kfree_skb(skb);
}

static int gf_netlink_init(void)
{
	struct netlink_kernel_cfg cfg;
	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = gf_netlink_recv;

	g_gf_dev->nl_sk = netlink_kernel_create(&init_net, GF_NETLINK_ROUTE, &cfg);
	if (g_gf_dev->nl_sk == NULL) {
		gf_debug(ERR_LOG, "[%s] : netlink create failed\n", __func__);
		return -1;
	} else {
		gf_debug(INFO_LOG, "[%s] : netlink create success\n", __func__);
		return 0;
	}
}

static int gf_netlink_destory(void)
{
	if (g_gf_dev->nl_sk != NULL) {
		/* sock_release(g_gf_dev->nl_sk->sk_socket); */
		netlink_kernel_release(g_gf_dev->nl_sk);
		g_gf_dev->nl_sk = NULL;
		return 0;
	} else {
		gf_debug(ERR_LOG, "[%s] : no netlink socket yet\n", __func__);
		return -1;
	}
}

/* -------------------------------------------------------------------- */
/* early suspend callback and suspend/resume functions		*/
/* -------------------------------------------------------------------- */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gf_early_suspend(struct early_suspend *handler)
{
	struct gf_dev *gf_dev;

	gf_dev = container_of(handler, struct gf_dev, early_suspend);
	gf_debug(INFO_LOG, "[%s] enter\n", __func__);

	gf_dev->is_sleep_mode = 0;
	gf_netlink_send(GF_NETLINK_SCREEN_OFF);
	gf_dev->device_available = 0;
}

static void gf_late_resume(struct early_suspend *handler)
{
	struct gf_dev *gf_dev;

	gf_dev = container_of(handler, struct gf_dev, early_suspend);
	gf_debug(INFO_LOG, "[%s] enter\n", __func__);

	/* first check whether chip is still in sleep mode */
	if (gf_dev->is_sleep_mode == 1) {
		gf_hw_reset(60);
		gf_dev->is_sleep_mode = 0;
	}

	gf_netlink_send(GF_NETLINK_SCREEN_ON);
	gf_dev->device_available = 1;
}
#endif

static int gf_suspend(struct spi_device *spi, pm_message_t msg)
{
	/* do nothing while enter suspend */
	gf_debug(INFO_LOG, "%s: enter\n", __func__);
	return 0;
}

static int gf_resume(struct spi_device *spi)
{
	struct gf_dev	 *gf_dev;
	gf_dev = spi_get_drvdata(spi);

	gf_debug(INFO_LOG, "%s: enter\n", __func__);

	if (gf_dev->is_sleep_mode == 1) {
		/* wake up sensor... */
		gf_hw_reset(5);
		gf_dev->is_sleep_mode = 0;
	}
	return 0;
}

/* -------------------------------------------------------------------- */
/* SPI ree operation functions											     */
/* -------------------------------------------------------------------- */

/* gf_spi_setup_conf_ree, configure spi speed and tranfer mode in REE mode
  *
  * speed: 1, 4, 6, 8 unit:MHz
  * mode: DMA mode or FIFO mode
  */
void gf_spi_setup_conf_ree(u32 speed, enum spi_transfer_mode mode)
{
	struct mt_chip_conf *mcc = &g_gf_dev->spi_mcc;
	switch (speed) {
	case 1:
		/* set to 1MHz clock */
		mcc->high_time = 50;
		mcc->low_time = 50;
		break;
	case 4:
		/* set to 4MHz clock */
		mcc->high_time = 15;
		mcc->low_time = 15;
		break;
	case 6:
		/* set to 6MHz clock */
		mcc->high_time = 10;
		mcc->low_time = 10;
		break;
	case 8:
		/* set to 8MHz clock */
		mcc->high_time = 8;
		mcc->low_time = 8;
		break;
	default:
		/* default set to 1MHz clock */
		mcc->high_time = 50;
		mcc->low_time = 50;
	}

	if ((mode == DMA_TRANSFER) || (mode == FIFO_TRANSFER)) {
		mcc->com_mod = mode;
	} else {
		/* default set to FIFO mode */
		mcc->com_mod = FIFO_TRANSFER;
	}

	if (spi_setup(g_gf_dev->spi)) {
		gf_debug(ERR_LOG, "%s, failed to setup spi conf\n", __func__);
	}
}

int gf_spi_read_bytes_ree(u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u8 *tmp_buf;
	u32 package, reminder, retry;

	if (g_gf_dev->spi_ree_enable) {
		package = (data_len + 2) / 1024;
		reminder = (data_len + 2) % 1024;

		if ((package > 0) && (reminder != 0)) {
			xfer = kzalloc(sizeof(*xfer)*4, GFP_KERNEL);
			retry = 1;
		} else {
			xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
			retry = 0;
		}
		if (xfer == NULL) {
			gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
			return -ENOMEM;
		}

		tmp_buf = g_gf_dev->spi_buffer;

		/* switch to DMA mode if transfer length larger than 32 bytes */
		if ((data_len + 1) > 32) {
			g_gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
		}
		spi_setup(g_gf_dev->spi);

		spi_message_init(&msg);
		*tmp_buf = 0xF0;
		*(tmp_buf + 1) = (u8)((addr>>8)&0xFF);
		*(tmp_buf + 2) = (u8)(addr&0xFF);
		xfer[0].tx_buf = tmp_buf;
		xfer[0].len = 3;
		xfer[0].delay_usecs = 5;
		spi_message_add_tail(&xfer[0], &msg);
		spi_sync(g_gf_dev->spi, &msg);

		spi_message_init(&msg);
		//memset((tmp_buf + 4), 0x00, data_len + 1);
		/* 4 bytes align */
		*(tmp_buf + 4) = 0xF1;
		xfer[1].tx_buf = tmp_buf + 4;
		xfer[1].rx_buf = tmp_buf + 4;
		if (retry) {
			xfer[1].len = package * 1024;
		} else {
			xfer[1].len = data_len + 1;
		}
		xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
		spi_sync(g_gf_dev->spi, &msg);

		/* copy received data */
		if (retry) {
			memcpy(rx_buf, (tmp_buf + 5), (package * 1024-1));
		} else {
			memcpy(rx_buf, (tmp_buf + 5), data_len);
		}

		/* send reminder SPI data */
		if (retry) {
			addr = addr + package * 1024 - 2;
			spi_message_init(&msg);
			*tmp_buf = 0xF0;
			*(tmp_buf + 1) = (u8)((addr>>8)&0xFF);
			*(tmp_buf + 2) = (u8)(addr&0xFF);
			xfer[2].tx_buf = tmp_buf;
			xfer[2].len = 3;
			xfer[2].delay_usecs = 5;
			spi_message_add_tail(&xfer[2], &msg);
			spi_sync(g_gf_dev->spi, &msg);

			spi_message_init(&msg);
			*(tmp_buf + 4) = 0xF1;
			xfer[3].tx_buf = tmp_buf + 4;
			xfer[3].rx_buf = tmp_buf + 4;
			xfer[3].len = reminder + 1;
			xfer[3].delay_usecs = 5;
			spi_message_add_tail(&xfer[3], &msg);
			spi_sync(g_gf_dev->spi, &msg);

			memcpy((rx_buf + package * 1024 - 1), (tmp_buf + 6), (reminder - 1));
		}

		/* restore to FIFO mode if has used DMA */
		if ((data_len + 1) > 32) {
			g_gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
		}
		spi_setup(g_gf_dev->spi);

		kfree(xfer);
		if(xfer != NULL) {
			xfer = NULL;
		}
	} else {
		gf_debug(ERR_LOG, "%s, ree spi has been disabled\n", __func__);
	}
	return 0;
}

int gf_spi_write_bytes_ree(u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u8 *tmp_buf;
	u32 package, reminder, retry;

	if (g_gf_dev->spi_ree_enable) {
		package = (data_len + 3) / 1024;
		reminder = (data_len + 3) % 1024;

		if ((package > 0) && (reminder != 0)) {
			xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
			retry = 1;
		} else {
			xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
			retry = 0;
		}
		if (xfer == NULL) {
			gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
			return -ENOMEM;
		}
		tmp_buf = g_gf_dev->spi_buffer;

		/* switch to DMA mode if transfer length larger than 32 bytes */
		if ((data_len + 3) > 32) {
			g_gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
		}
		spi_setup(g_gf_dev->spi);

		spi_message_init(&msg);
		*tmp_buf = 0xF0;
		*(tmp_buf + 1) = (u8)((addr>>8)&0xFF);
		*(tmp_buf + 2) = (u8)(addr&0xFF);
		if (retry) {
			memcpy(tmp_buf + 3, tx_buf, (package * 1024 -3));
			xfer[0].len = package*1024;
		} else {
			memcpy(tmp_buf + 3, tx_buf, data_len);
			xfer[0].len = data_len + 3;
		}
		xfer[0].tx_buf = tmp_buf;
		xfer[0].delay_usecs = 5;
		spi_message_add_tail(&xfer[0], &msg);
		spi_sync(g_gf_dev->spi, &msg);

		if (retry) {
			addr = addr + package*1024 - 3;
			spi_message_init(&msg);
			*tmp_buf = 0xF0;
			*(tmp_buf + 1) = (u8)((addr>>8)&0xFF);
			*(tmp_buf + 2) = (u8)(addr&0xFF);
			memcpy(tmp_buf + 3, (tx_buf + package * 1024 -3), reminder);
			xfer[1].tx_buf = tmp_buf;
			xfer[1].len = reminder + 3;
			xfer[1].delay_usecs = 5;
			spi_message_add_tail(&xfer[1], &msg);
			spi_sync(g_gf_dev->spi, &msg);
		}

		/* restore to FIFO mode if has used DMA */
		if ((data_len + 3) > 32) {
			g_gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
		}
		spi_setup(g_gf_dev->spi);

		kfree(xfer);
		if(xfer != NULL) {
			xfer = NULL;
		}
	} else {
		gf_debug(ERR_LOG, "%s, ree spi has been disabled\n", __func__);
	}

	return 0;
}

int gf_spi_read_byte_ree(u16 addr, u8 *value)
{
	struct spi_message msg;
	struct spi_transfer *xfer;

	if (g_gf_dev->spi_ree_enable) {
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
		if (xfer == NULL) {
			gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
			return -ENOMEM;
		}

		spi_message_init(&msg);
		*g_gf_dev->spi_buffer = 0xF0;
		*(g_gf_dev->spi_buffer + 1) = (u8)((addr>>8)&0xFF);
		*(g_gf_dev->spi_buffer + 2) = (u8)(addr&0xFF);

		xfer[0].tx_buf = g_gf_dev->spi_buffer;
		xfer[0].len = 3;
		xfer[0].delay_usecs = 5;
		spi_message_add_tail(&xfer[0], &msg);
		spi_sync(g_gf_dev->spi, &msg);

		spi_message_init(&msg);
		/* 4 bytes align */
		*(g_gf_dev->spi_buffer + 4) = 0xF1;
		xfer[1].tx_buf = g_gf_dev->spi_buffer + 4;
		xfer[1].rx_buf =g_gf_dev->spi_buffer + 4;
		xfer[1].len = 2;
		xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
		spi_sync(g_gf_dev->spi, &msg);

		*value = *(g_gf_dev->spi_buffer + 5);

		kfree(xfer);
		if(xfer != NULL) {
			xfer = NULL;
		}
	} else {
		gf_debug(ERR_LOG, "%s, ree spi has been disabled\n", __func__);
	}
	return 0;
}


int gf_spi_write_byte_ree(u16 addr, u8 value)
{
	struct spi_message msg;
	struct spi_transfer *xfer;

	if (g_gf_dev->spi_ree_enable) {
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
		if (xfer == NULL) {
			gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
			return -ENOMEM;
		}

		spi_message_init(&msg);
		*g_gf_dev->spi_buffer = 0xF0;
		*(g_gf_dev->spi_buffer + 1) = (u8)((addr>>8)&0xFF);
		*(g_gf_dev->spi_buffer + 2) = (u8)(addr&0xFF);
		*(g_gf_dev->spi_buffer + 3) = value;

		xfer[0].tx_buf = g_gf_dev->spi_buffer;
		xfer[0].len = 3 + 1;
		xfer[0].delay_usecs = 5;
		spi_message_add_tail(&xfer[0], &msg);
		spi_sync(g_gf_dev->spi, &msg);

		kfree(xfer);
		if(xfer != NULL) {
			xfer = NULL;
		}
	} else {
		gf_debug(ERR_LOG, "%s, ree spi has been disabled\n", __func__);
	}
	return 0;
}


/* -------------------------------------------------------------------- */
/* file operation function											 */
/* -------------------------------------------------------------------- */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	u8 status;
	u8 *transfer_buf = NULL;
	int retval = 0;
	u16 checksum = 0;
	int i;

	FUNC_ENTRY();

	if (!g_gf_dev->spi_ree_enable) {
		gf_debug(ERR_LOG, "%s: Not support read opertion in TEE mode\n", __func__);
		return -EFAULT;
	} else {
		gf_spi_read_byte_ree(0x8140, &status);
		if ((status&0xF0) != 0xC0) {
			gf_debug(ERR_LOG, "%s: no image data available\n", __func__);
			return 0;
		} else {
			if ((count > bufsiz) || (count == 0)) {
				gf_debug(ERR_LOG, "%s: request transfer length larger than maximum buffer\n", __func__);
				return -EINVAL;
			} else {
				transfer_buf = kzalloc((count + 10), GFP_KERNEL);
				if (transfer_buf == NULL) {
					gf_debug(ERR_LOG, "%s: failed to allocate transfer buffer\n", __func__);
					return -EMSGSIZE;
				}
			}
		}

		/* set spi to high speed */
		gf_spi_setup_conf_ree(6, DMA_TRANSFER);

		gf_spi_read_bytes_ree(0x8140, count+10, transfer_buf);

		/* check checksum */
		checksum = 0;
		for(i=0; i<(count+6); i++) {
			checksum += *(transfer_buf+2+i);
		}
		if (checksum != ((*(transfer_buf+count+8)<<8) | *(transfer_buf+count+9))){
			gf_debug(ERR_LOG, "%s: raw data checksum check failed, cal[0x%x], recevied[0x%x]\n", __func__,
						checksum, ((*(transfer_buf+count+8)<<8) | *(transfer_buf+count+9)));
			retval = 0;
		} else {
			gf_debug(INFO_LOG, "%s: checksum check passed[0x%x], copy_to_user\n", __func__, checksum);
			if (copy_to_user(buf, transfer_buf + 8, count)) {
				gf_debug(ERR_LOG, "%s: Failed to copy gf_ioc_transfer from kernel to user\n", __func__);
				retval = -EFAULT;
			} else {
				retval = count;
			}
		}

		/* restore to low speed */
		gf_spi_setup_conf_ree(1, FIFO_TRANSFER);

		kfree(transfer_buf);
		FUNC_EXIT();
		return retval;
	}
}

static ssize_t gf_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	gf_debug(ERR_LOG, "Not support write opertion in TEE version\n");
	return -EFAULT;
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_dev *gf_dev = (struct gf_dev *)handle;
	/* struct gf_dev *dev = g_gf_dev; */

	FUNC_ENTRY();

#ifdef GF_FASYNC
	kill_fasync(&dev->async, SIGIO, POLL_IN);
#endif

	gf_debug(ERR_LOG, "%s, irq is 0x%x\n", __func__, irq);

#ifdef GF_NETLINK
	gf_netlink_send(GF_NETLINK_IRQ);
#endif
	gf_dev->sig_count++;

#if 0
	u8 status[2];
	gf_spi_read_bytes_ree(0x8140, 2, status);
	gf_debug(INFO_LOG, "%s, read status is 0x%x, 0x%x\n", __func__, status[0], status[1]);

	gf_spi_write_byte_ree(0x8140, (status[0]&0x7F));
#endif

	FUNC_EXIT();
	return IRQ_HANDLED;
}


static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = NULL;
	struct gf_key gf_key;
	uint32_t key_event;
	struct gf_ioc_transfer ioc;
	u8 *transfer_buf = NULL;
	int retval = 0;
	u8 netlink_route = GF_NETLINK_ROUTE;

	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	* IOC_DIR is from the user perspective, while access_ok is
	* from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

	if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (retval)
		return -EFAULT;

	gf_dev = (struct gf_dev *)filp->private_data;

	switch (cmd) {
	case GF_IOC_INIT:
		gf_debug(INFO_LOG, "%s: gf init started======\n", __func__);

		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
			retval = -EFAULT;
			break;
		}

		if (gf_dev->system_status) {
			gf_debug(INFO_LOG, "%s: system re-started======\n", __func__);
			gf_dev->device_available = 1;
			break;
		}
		gf_irq_gpio_cfg();
		retval = request_threaded_irq(gf_dev->spi->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&(gf_dev->spi->dev)), gf_dev);
		if (!retval)
			gf_debug(INFO_LOG, "%s irq thread request success!\n", __func__);
		else
			gf_debug(ERR_LOG, "%s irq thread request failed, retval=%d\n", __func__, retval);

			gf_dev->device_available = 1;
			gf_dev->irq_count = 1;
			gf_disable_irq(gf_dev);

			#if defined(CONFIG_HAS_EARLYSUSPEND)
			gf_debug(INFO_LOG, "[%s] : register_early_suspend\n", __func__);
			gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
			gf_dev->early_suspend.suspend = gf_early_suspend,
			gf_dev->early_suspend.resume = gf_late_resume,
			register_early_suspend(&gf_dev->early_suspend);
			#endif
			gf_dev->sig_count = 0;
			gf_dev->spi_ree_enable = 0;
			gf_dev->system_status = 1;

			gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);

		break;

	case GF_IOC_EXIT:
		gf_disable_irq(gf_dev);
		if (gf_dev->spi->irq) {
			free_irq(gf_dev->spi->irq, gf_dev);
			gf_dev->irq_count = 0;
			//gf_dev->spi->irq = 0;
		}
		gf_dev->device_available = 0;
		gf_dev->spi_ree_enable = 1;

		#ifdef CONFIG_HAS_EARLYSUSPEND
		if (gf_dev->early_suspend.suspend)
			unregister_early_suspend(&gf_dev->early_suspend);
		#endif
		gf_dev->system_status = 0;
		gf_debug(INFO_LOG, "%s: gf exit finished======\n", __func__);

		break;

	case GF_IOC_RESET:
		gf_debug(INFO_LOG, "%s: chip reset command\n", __func__);
		gf_hw_reset(60);
		break;

	case GF_IOC_ENABLE_IRQ:
		gf_enable_irq(gf_dev);
		break;

	case GF_IOC_DISABLE_IRQ:
		gf_disable_irq(gf_dev);
		break;

	case GF_IOC_ENABLE_SPI_CLK:
		gf_spi_clk_enable(1);
		break;

	case GF_IOC_DISABLE_SPI_CLK:
		gf_spi_clk_enable(0);
		break;

	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			gf_debug(ERR_LOG, "Failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		if (GF_KEY_HOME == gf_key.key) {
			key_event = GF_INPUT_HOME_KEY;
		} else if (GF_KEY_POWER == gf_key.key) {
			key_event = GF_INPUT_FF_KEY;
		} else if (GF_KEY_CAPTURE == gf_key.key) {
			key_event = GF_INPUT_CAMERA_KEY;
		} else if (GF_KEY_UP == gf_key.key) {
			key_event = GF_INPUT_NAV_UP_KEY;
		} else if (GF_KEY_DOWN == gf_key.key) {
			key_event = GF_INPUT_NAV_DOWN_KEY;
		} else if (GF_KEY_LEFT == gf_key.key) {
			key_event = GF_INPUT_NAV_LEFT_KEY;
		} else if (GF_KEY_RIGHT == gf_key.key) {
			key_event = GF_INPUT_NAV_RIGHT_KEY;
		} else {
			/* add special key define */
			key_event = GF_INPUT_OTHER_KEY;
		}
		gf_debug(INFO_LOG, "%s: received key event[%d], key=%d, value=%d\n", __func__, key_event, gf_key.key, gf_key.value);

        if ((GF_KEY_POWER == gf_key.key) && (gf_key.value == 1)) {
/*
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
*/
        }
        else if ((GF_KEY_CAPTURE == gf_key.key) && (gf_key.value == 1)) {
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
		} else if (GF_KEY_UP == gf_key.key) {
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
		} else if (GF_KEY_DOWN == gf_key.key) {
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
		} else if (GF_KEY_RIGHT == gf_key.key) {
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
		} else if (GF_KEY_LEFT == gf_key.key) {
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
		} else if ((GF_KEY_POWER != gf_key.key) && (GF_KEY_CAPTURE != gf_key.key)) {
			input_report_key(gf_dev->input, key_event, gf_key.value);
			input_sync(gf_dev->input);
		}
		break;

	case GF_IOC_ENTER_SLEEP_MODE:
		gf_dev->is_sleep_mode = 1;
		break;

	case GF_IOC_TRANSFER_CMD:
		if (copy_from_user(&ioc, (struct gf_ioc_transfer *)arg, sizeof(struct gf_ioc_transfer))) {
			gf_debug(ERR_LOG, "%s: Failed to copy gf_ioc_transfer from user to kernel\n", __func__);
			retval = -EFAULT;
			break;
		}

		if ((ioc.len > bufsiz) || (ioc.len == 0)) {
			gf_debug(ERR_LOG, "%s: request transfer length larger than maximum buffer\n", __func__);
			retval = -EINVAL;
			break;
		} else {
			transfer_buf = kzalloc(ioc.len, GFP_KERNEL);
			if (transfer_buf == NULL) {
				gf_debug(ERR_LOG, "%s: failed to allocate transfer buffer\n", __func__);
				retval = -EMSGSIZE;
				break;
			}
		}

		mutex_lock(&g_gf_dev->buf_lock);
		if (ioc.cmd) {
			/* spi write operation */
			gf_debug(DEBUG_LOG, "%s: write data to 0x%x, len = 0x%x\n", __func__, ioc.addr, ioc.len);
			if (copy_from_user(transfer_buf, ioc.buf, ioc.len)) {
				gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer from user to kernel\n");
				retval = -EFAULT;
			} else {
				gf_spi_write_bytes_ree(ioc.addr, ioc.len, transfer_buf);
			}
		} else {
			/* spi read operation */
			gf_debug(DEBUG_LOG, "%s: read data from 0x%x, len = 0x%x\n", __func__, ioc.addr, ioc.len);
			gf_spi_read_bytes_ree(ioc.addr, ioc.len, transfer_buf);
			if (copy_to_user(ioc.buf, transfer_buf, ioc.len)) {
				gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer from kernel to user\n");
				retval = -EFAULT;
			}
		}
		kfree(transfer_buf);
		mutex_unlock(&g_gf_dev->buf_lock);
		break;

	default:
		gf_debug(ERR_LOG, "gf doesn't support this command(%d)\n", cmd);
		break;
	}

	FUNC_EXIT();
	return retval;
}

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
	gf_debug(ERR_LOG, "Not support poll opertion in TEE version\n");
	return -EFAULT;
}


/* -------------------------------------------------------------------- */
/* devfs							      */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	gf_debug(INFO_LOG, "%s: Show debug_level = 0x%x\n", __func__, g_debug_level);
	return sprintf(buf, "0x%x\n", g_debug_level);
}

static ssize_t gf_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gf_dev *gf_dev =  dev_get_drvdata(dev);
	int retval = 0;
	u8 tmp[16];

	if (!strncmp(buf, "-10", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -10, gf init start===============\n", __func__);

		gf_dev->device_available = 1;
		gf_irq_gpio_cfg();
		retval = request_threaded_irq(gf_dev->spi->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&(gf_dev->spi->dev)), gf_dev);
		if (!retval)
			gf_debug(INFO_LOG, "%s irq thread request success!\n", __func__);
		else
			gf_debug(ERR_LOG, "%s irq thread request failed, retval=%d\n", __func__, retval);

		gf_dev->irq_count = 1;
		gf_disable_irq(gf_dev);

#if defined(CONFIG_HAS_EARLYSUSPEND)
		gf_debug(INFO_LOG, "[%s] : register_early_suspend\n", __func__);
		gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
		gf_dev->early_suspend.suspend = gf_early_suspend,
		gf_dev->early_suspend.resume = gf_late_resume,
		register_early_suspend(&gf_dev->early_suspend);
#endif
		gf_dev->sig_count = 0;

		gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);

	} else if (!strncmp(buf, "-11", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -11, enable irq===============\n", __func__);
		gf_enable_irq(gf_dev);

	} else if (!strncmp(buf, "-12", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -12, REE SPI read/write test=======\n", __func__);
		gf_spi_read_bytes_ree(0x4220, 4, tmp);
		gf_debug(INFO_LOG, "%s, 9p chp version is 0x%x, 0x%x, 0x%x, 0x%x\n", __func__,
				tmp[0], tmp[1], tmp[2], tmp[3]);

		gf_spi_read_bytes_ree(0x8000, 10, tmp);
		gf_debug(INFO_LOG, "%s, read fw version 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x, %02d.%02d.%02d\n", __func__,
				tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[7], tmp[8], tmp[9]);

	} else if (!strncmp(buf, "-13", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -13, set/read sensor mode==========\n", __func__);
		gf_spi_read_byte_ree(0x8043, &tmp[0]);
		gf_debug(INFO_LOG, "%s, read out chip mode=0x%x\n", __func__, tmp[0]);

		gf_spi_write_byte_ree(0x8043, 0);
		gf_spi_read_byte_ree(0x8043, &tmp[0]);
		gf_debug(INFO_LOG, "%s, read out chip mode=0x%x after set to IMAGE mode\n", __func__, tmp[0]);

		gf_spi_write_byte_ree(0x8043, 1);
		gf_spi_read_byte_ree(0x8043, &tmp[0]);
		gf_debug(INFO_LOG, "%s, read out chip mode=0x%x after set to KEY mode\n", __func__, tmp[0]);

	} else {
		gf_debug(ERR_LOG, "%s: wrong paramter!===============\n", __func__);
	}

	return count;
}


static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
	&dev_attr_debug.attr,
	NULL
};

static const struct attribute_group gf_debug_attr_group = {
	.attrs = gf_debug_attrs,
	.name = "debug"
};

/* -------------------------------------------------------------------- */
/* device function								  */
/* -------------------------------------------------------------------- */
static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			gf_debug(INFO_LOG, "%s, Found\n", __func__);
			status = 0;
			break;
		}
	}

	if (status == 0) {
		mutex_lock(&gf_dev->buf_lock);
		if (gf_dev->spi_buffer == NULL) {
			gf_dev->spi_buffer = kzalloc(bufsiz, GFP_KERNEL);
			if (gf_dev->spi_buffer == NULL) {
				gf_debug(ERR_LOG, "%s, allocate dev->buffer failed\n", __func__);
				status = -ENOMEM;
			}
		}
		mutex_unlock(&gf_dev->buf_lock);

		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			gf_debug(INFO_LOG, "%s, Succeed to open device. irq = %d\n", __func__, gf_dev->spi->irq);
		}
	} else {
		gf_debug(ERR_LOG, "%s, No device for minor %d\n", __func__, iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gf_dev = filp->private_data;
	int ret;

	FUNC_ENTRY();
	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	FUNC_EXIT();
	return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int    status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close??*/
	gf_dev->users--;
	if (!gf_dev->users) {
		gf_debug(INFO_LOG, "%s, disble_irq. irq = %d\n", __func__, gf_dev->spi->irq);
		gf_disable_irq(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	gf_dev->device_available = 0;
	FUNC_EXIT();
	return status;
}


static struct class *gf_spi_class;

static const struct file_operations gf_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	* gets more complete API coverage.	It'll simplify things
	* too, except for the locking.
	*/
	.write =	gf_write,
	.read =		gf_read,
	.unlocked_ioctl = gf_ioctl,
	/* .compat_ioctl = gf_compat_ioctl, */
	.open =		gf_open,
	.release =	gf_release,
	.poll	= gf_poll,
#ifdef GF_FASYNC
	.fasync = gf_fasync,
#endif
};

/*-------------------------------------------------------------------------*/
extern struct mt_chip_conf spi_ctrdata;
static int gf_probe(struct spi_device *spi)
{
	struct gf_dev *gf_dev = NULL;
	unsigned long minor;
	int status = -EINVAL;

	FUNC_ENTRY();
	/* Allocate driver data */
	gf_dev = kzalloc(sizeof(struct gf_dev), GFP_KERNEL);
	if (!gf_dev) {
		gf_debug(ERR_LOG, "%s, Failed to alloc memory for gf device.\n", __func__);
		FUNC_EXIT();
		return -ENOMEM;
	}

	g_gf_dev = gf_dev;
	/* Initialize the driver data */
	gf_dev->spi = spi;
	spin_lock_init(&gf_dev->spi_lock);
	mutex_init(&gf_dev->buf_lock);

	INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->device_available = 0;
	gf_dev->spi->irq = 0;
	gf_dev->probe_finish = 0;
        gf_dev->system_status = 0;
	gf_dev->spi_ree_enable = 1;

	/*setup gf configurations.*/
	gf_debug(INFO_LOG, "%s, Setting gf device configuration.\n", __func__);

	/*SPI parameters.*/
	/* CPOL=CPHA=0, speed 1MHz */
	gf_dev->spi->mode = SPI_MODE_0;
	gf_dev->spi->bits_per_word = 8;
	gf_dev->spi->max_speed_hz = 1*1000*1000;

	gf_dev->spi->irq = mt_gpio_to_irq(CUST_EINT_FP_EINT_NUM);

	memcpy(&gf_dev->spi_mcc, &spi_ctrdata, sizeof(struct mt_chip_conf));
	gf_dev->spi->controller_data = (void *)&gf_dev->spi_mcc;

	/* not work for platform */
	spi_setup(gf_dev->spi);
	spi_set_drvdata(spi, gf_dev);
	gf_debug(INFO_LOG, "%s, gf interrupt NO. = %d\n", __func__, gf_dev->spi->irq);

	/*enable the power*/
	gf_hw_power_enable(1);
	gf_bypass_flash_gpio_cfg();

	/* gpio function setting */
	gf_spi_gpio_cfg();
	gf_reset_gpio_cfg();

	/* put miso high to select SPI transfer */
	gf_miso_gpio_cfg(1);
	gf_hw_reset(60);
	gf_miso_gpio_cfg(0);

	gf_irq_gpio_cfg();
	gf_spi_clk_enable(1);

	/* If we can allocate a minor number, hook up this device.
	* Reusing minors is fine so long as udev or mdev is working.
	*/
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		status = sysfs_create_group(&spi->dev.kobj, &gf_debug_attr_group);
		if (status) {
			gf_debug(ERR_LOG, "%s, Failed to create sysfs file.\n", __func__);
			status = -ENODEV;
			mutex_unlock(&device_list_lock);
			goto err;
		}

		gf_debug(INFO_LOG, "%s, Success create sysfs file.\n", __func__);

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_spi_class, &spi->dev, gf_dev->devt, gf_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		gf_debug(ERR_LOG, "%s, no minor number available!\n", __func__);
		mutex_unlock(&device_list_lock);
		goto err;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	}

	mutex_unlock(&device_list_lock);
	if (status == 0) {
		gf_dev->spi_buffer = kzalloc(bufsiz, GFP_KERNEL);
		if (gf_dev->spi_buffer == NULL) {
			kfree(gf_dev);
			status = -ENOMEM;
			goto err;
		}

		/*register device within input system.*/
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			gf_debug(ERR_LOG, "%s, Failed to allocate input device.\n", __func__);
			status = -ENOMEM;
			kfree(gf_dev->spi_buffer);
			kfree(gf_dev);
			goto err;
		}

		__set_bit(EV_KEY, gf_dev->input->evbit);
		__set_bit(GF_INPUT_HOME_KEY, gf_dev->input->keybit);

		__set_bit(GF_INPUT_MENU_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_BACK_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_FF_KEY, gf_dev->input->keybit);

		__set_bit(GF_INPUT_NAV_UP_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_NAV_DOWN_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_NAV_LEFT_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_NAV_RIGHT_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_CAMERA_KEY, gf_dev->input->keybit);
		
		gf_dev->input->name = "gf-keys";
		if (input_register_device(gf_dev->input)) {
			gf_debug(ERR_LOG, "%s, Failed to register input device.\n", __func__);
			goto err;
		}
	} else {
		gf_debug(ERR_LOG, "%s, device create failed", __func__);
		goto err;
	}

	/* netlink interface init */
	gf_netlink_init();

	gf_dev->probe_finish = 1;
	gf_dev->is_sleep_mode = 0;
	gf_debug(INFO_LOG, "%s probe finished, normal driver version: %s\n", __func__, GF_SPI_VERSION);
	gf_spi_clk_enable(0);

	FUNC_EXIT();
	return 0;

err:
	device_destroy(gf_spi_class, gf_dev->devt);

	gf_spi_clk_enable(0);
	kfree(gf_dev);
	FUNC_EXIT();
	return status;
}

static int gf_remove(struct spi_device *spi)
{
	struct gf_dev *gf_dev = spi_get_drvdata(spi);
	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->spi->irq) {
		free_irq(gf_dev->spi->irq, gf_dev);
		gf_dev->irq_count = 0;
		gf_dev->spi->irq = 0;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	if (gf_dev->early_suspend.suspend)
		unregister_early_suspend(&gf_dev->early_suspend);
#endif

	gf_netlink_destory();

	spin_lock_irq(&gf_dev->spi_lock);
	gf_dev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&gf_dev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_spi_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	if (gf_dev->users == 0) {
		if (gf_dev->input != NULL)
			input_unregister_device(gf_dev->input);

		if (gf_dev->spi_buffer != NULL)
			kfree(gf_dev->spi_buffer);

		kfree(gf_dev);
	}
	mutex_unlock(&device_list_lock);

	gf_debug(INFO_LOG, "%s remove finished\n", __func__);

	FUNC_EXIT();
	return 0;
}


static struct spi_driver gf_spi_driver = {
	.driver = {
		.name = SPI_DEV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = gf_probe,
	.remove = gf_remove,
	.suspend = gf_suspend,
	.resume = gf_resume,
};

struct mt_chip_conf spi_ctrdata = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 50, /* 1MHz */
	.low_time = 50,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,

	.cpol = SPI_CPOL_0,
	.cpha = SPI_CPHA_0,

	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,

	.tx_endian = SPI_LENDIAN,
	.rx_endian = SPI_LENDIAN,

	.com_mod = FIFO_TRANSFER,
	/* .com_mod = DMA_TRANSFER, */

	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

static struct spi_board_info spi_fp_board_info[] __initdata = {
	[0] = {
		.modalias		= "gf_spi",
		.platform_data		= NULL,
		.chip_select = 0,
		.bus_num		= 0,
		.controller_data	= &spi_ctrdata,
	},
};

/*-------------------------------------------------------------------------*/
static int __init gf_init(void)
{
	int status;
	FUNC_ENTRY();

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	*/
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s, Failed to register char device!\n", __func__);
		FUNC_EXIT();
		return status;
	}
	gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		gf_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
		FUNC_EXIT();
		return PTR_ERR(gf_spi_class);
	}

	spi_register_board_info(spi_fp_board_info, ARRAY_SIZE(spi_fp_board_info));

	status = spi_register_driver(&gf_spi_driver);
	if (status < 0) {
		class_destroy(gf_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		gf_debug(ERR_LOG, "%s, Failed to register SPI driver.\n", __func__);
	}

	FUNC_EXIT();
	return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	FUNC_ENTRY();
	spi_unregister_driver(&gf_spi_driver);
	class_destroy(gf_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
	FUNC_EXIT();
}
module_exit(gf_exit);


MODULE_AUTHOR("Ke Yang<yangke@goodix.com>");
MODULE_DESCRIPTION("Goodix Fingerprint chip GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF3208/GF5216 TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf_spi");
