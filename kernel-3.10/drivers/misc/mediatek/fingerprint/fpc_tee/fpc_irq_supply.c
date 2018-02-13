/* Fingerprint Cards, Hybrid Touch sensor driver
 *
 * Copyright (c) 2014,2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 *
 * Software license : "Dual BSD/GPL"
 * see <linux/module.h> and ./Documentation
 * for  details.
 *
*/

#define DEBUG

#include <linux/device.h>
#include <linux/wait.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include <asm/siginfo.h>
#include <asm/uaccess.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/rcupdate.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/version.h>
#include <linux/wakelock.h>


#include "fpc_irq_common.h"
#include "fpc_irq_supply.h"
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/mt_clkmgr.h>

static void enable_clk(void)
{
	//#if (!defined(CONFIG_MT_SPI_FPGA_ENABLE))
		enable_clock(MT_CG_PERI_SPI0, "spi");
		printk("clock enabled !!\n");
	///#endif
		return;
}

static void spi_gpio_set(struct mt_spi_t *ms)
{
	#if 0
	/* lenovo-sw, chenzz3, change for fingerprint, begin */
	mt_set_gpio_mode(GPIO_SPI_CS_PIN, 1);
	mt_set_gpio_mode(GPIO_SPI_SCK_PIN, 1);
	mt_set_gpio_mode(GPIO_SPI_MISO_PIN, 1);
	mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, 1);
	/* lenovo-sw, chenzz3, change for fingerprint, end */
	#endif
	mt_set_gpio_mode(0x80000000|166, 1);
	mt_set_gpio_mode(0x80000000|167, 1);
	mt_set_gpio_mode(0x80000000|168, 1);
	mt_set_gpio_mode(0x80000000|169, 1);
	return;
}

/* -------------------------------------------------------------------------- */
int fpc_irq_supply_init(fpc_irq_data_t *fpc_irq_data)
{
	int ret = 0;

	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);
	spi_gpio_set(0);
	enable_clk();
	// Todo: Acquire required regulators

	return ret;
}


/* -------------------------------------------------------------------------- */
int fpc_irq_supply_destroy(fpc_irq_data_t *fpc_irq_data)
{
	int ret = 0;

	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);

	// Todo: release used regulators

	return ret;
}


/* -------------------------------------------------------------------------- */
extern int fpc_irq_supply_set(fpc_irq_data_t *fpc_irq_data, bool req_state)
{
	int ret = 0;
	bool curr_state = fpc_irq_data->pm.supply_on;

	dev_dbg(fpc_irq_data->dev, "%s %s => %s\n",
						__func__,
						(curr_state) ? "ON" : "OFF",
						(req_state) ? "ON" : "OFF");

	if (curr_state != req_state) {

		fpc_irq_data->pm.supply_on = req_state;

		// Todo: enable/disable used regulators
		// Todo: If state == off, also set I/O as required fo not sourcing the sensor.
	}

	return ret;
}


/* -------------------------------------------------------------------------- */
