#ifndef __GF516M_SPI_TEE_H
#define __GF516M_SPI_TEE_H

#include <linux/types.h>
#include "drFp_Api_gf516m.h"
#include "gf_spi_tee.h"

/**********************GF516M ops****************************/
#define GF516M_W			0xF0
#define GF516M_R			0xF1
#define GF516M_WDATA_OFFSET	(0x3)
#define GF516M_RDATA_OFFSET	(0x1)  //for MTK platform

/***************************GF516M mapping****************************/
#define GF516M_BASE						 (0x8000)
#define GF516M_OFFSET(x)					 (GF516M_BASE + x)

#define GF516M_FW_VERSION		(0x8000)
#define GF516M_MODE_STATUS		(0x8043)

/****** GF516M buffer status register *******/
#define GF516M_BUFFER_STATUS		(0x8140)

#define GF516M_BUF_STA_MASK		(0x1<<7)
#define GF516M_BUF_STA_READY		(0x1<<7)
#define GF516M_BUF_STA_BUSY		(0x0<<7)

#define GF516M_IMAGE_MASK				(0x1<<6)
#define GF516M_IMAGE_ENABLE		(0x1<<6)
#define GF516M_IMAGE_DISABLE		(0x0)

#define GF516M_KEY_MASK			(0x1<<5)
#define GF516M_KEY_ENABLE				(0x1<<5)
#define GF516M_KEY_DISABLE				(0x0)

#define GF516M_KEY_STA				(0x1<<4)

struct gf516m_dev {
	dev_t devt;
	spinlock_t	spi_lock;
	struct spi_device *spi;
	struct list_head device_entry;

	struct input_dev *input;

	struct workqueue_struct *spi_wq;
	struct work_struct spi_work;
	struct timer_list gf516m_timer;

	struct work_struct irq_work;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	struct task_struct *daemon_task;
	unsigned users;
	u8 *buffer;
	u8 buf_status;
	u8 device_available;	//changed during fingerprint chip sleep and wakeup phase
	int reset_gpio;
	int irq_gpio;
	u32 cs_gpio;
	u32 update_sw_gpio;
	GF516M_MODE mode;
        GF516M_MODE save_mode;
	u8 config_need_clear;
	struct spi_speed_setting spi_speed;

	//fasync support used
	struct fasync_struct *async;
	struct regulator *gx_power;
	struct notifier_block notifier;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	/* for trustonic DCI interface */
	//TODO: move dci, session_handle variable to here later
	struct mutex dci_lock;
	//spinlock_t dci_lock;
	u8 secspi_init_ok;	  //has changed SPI to secure world, no access in normal world
	u8 has_esd_watchdog;  //0: not need esd check, 1: do esd check
	u8 probe_finish;
	u8 irq_count;

	/* bit24-bit32 of signal count */
	/* bit16-bit23 of event type, 1: key down; 2: key up; 3: fp data ready; 4: home key */
	/* bit0-bit15 of event type, buffer status register */
	u32 event_type;
	u8 fw_need_upgrade;
	u8 is_ff_mode;    //1: chip in ff mode,  0: chip in active mode

	EVENT_TYPE lastEvent;
	u8 sig_count;
};

#endif	//__GF516M_SPI_TEE_H
