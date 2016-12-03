#ifndef __GF_SPI_TEE_H
#define __GF_SPI_TEE_H

#include <linux/types.h>
#include <linux/netlink.h>
#include "gf_spi_tee.h"

/**********************IO Magic**********************/
#define GF_IOC_MAGIC	'g'

typedef enum gf_key_event {
	GF_KEY_NONE = 0,
	GF_KEY_HOME,
	GF_KEY_POWER,
	GF_KEY_MENU,
	GF_KEY_BACK,
	GF_KEY_CAPTURE,
	GF_KEY_UP,
	GF_KEY_DOWN,
	GF_KEY_RIGHT,
	GF_KEY_LEFT,
	GF_KEY_TAP,
	GF_KEY_HEAVY
} gf_key_event_t;

struct gf_key {
	enum gf_key_event key;
	uint32_t value;   /* key down = 1, key up = 0 */
};

enum gf_netlink_cmd {
	GF_NETLINK_TEST = 0,
	GF_NETLINK_IRQ = 1,
	GF_NETLINK_SCREEN_OFF,
	GF_NETLINK_SCREEN_ON
};

struct gf_ioc_transfer {
	u8 cmd;    /* spi read = 0, spi  write = 1 */
	u8 reserved;
	u16 addr;
	u32 len;
	u8 *buf;
};

/* define commands */
#define GF_IOC_INIT             _IOR(GF_IOC_MAGIC, 0, uint8_t)

#define GF_IOC_EXIT			_IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET			_IO(GF_IOC_MAGIC, 2)

#define GF_IOC_ENABLE_IRQ		_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ		_IO(GF_IOC_MAGIC, 4)

#define GF_IOC_ENABLE_SPI_CLK		_IO(GF_IOC_MAGIC, 5)
#define GF_IOC_DISABLE_SPI_CLK		_IO(GF_IOC_MAGIC, 6)

#define GF_IOC_ENABLE_POWER		_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER		_IO(GF_IOC_MAGIC, 8)

#define GF_IOC_INPUT_KEY_EVENT		_IOW(GF_IOC_MAGIC, 9, struct gf_key)

/* fp sensor has change to sleep mode while screen off */
#define GF_IOC_ENTER_SLEEP_MODE		_IO(GF_IOC_MAGIC, 10)

/* for SPI REE transfer */
#define GF_IOC_TRANSFER_CMD			_IOWR(GF_IOC_MAGIC, 15, struct gf_ioc_transfer)
#define  GF_IOC_MAXNR    16  /* THIS MACRO IS NOT USED NOW... */


struct gf_dev {
	dev_t devt;
	spinlock_t	spi_lock;
	struct spi_device *spi;
	struct mt_chip_conf spi_mcc;
	struct list_head device_entry;

	struct input_dev *input;

	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	u8 *spi_buffer;
	struct mutex buf_lock;
	u8 buf_status;
	u8 device_available;	/* changed during fingerprint chip sleep and wakeup phase */

	/* fasync support used */
	struct fasync_struct *async;

	struct early_suspend early_suspend;

	u8 probe_finish;
	u8 spi_ree_enable;
	u8 irq_count;
	u8 system_status;

	/* bit24-bit32 of signal count */
	/* bit16-bit23 of event type, 1: key down; 2: key up; 3: fp data ready; 4: home key */
	/* bit0-bit15 of event type, buffer status register */
	u32 event_type;
	u8 sig_count;
	u8 is_sleep_mode;

	/* for netlink use */
	u8  need_update;
	struct sock *nl_sk;
	int pid;
};

#endif	/* __GF_SPI_TEE_H */
