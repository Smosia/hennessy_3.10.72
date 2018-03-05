/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

//#define DEBUG

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#define DEBUG
#include <linux/platform_device.h>
#include <linux/wakelock.h>

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250

#define FPC_TTW_HOLD_TIME 1000

//#include <mach/mt_clkmgr.h>

/*** phg ***/
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include "mt_spi_hal.h"
#include <linux/spi/spi.h>

#define GPIO_FPC_SPI_CS             (GPIO169 | 0x80000000)//(65 | 0x80000000)
#define GPIO_FPC_SPI_CLK            (GPIO166 | 0x80000000)//(66 | 0x80000000)
#define GPIO_FPC_SPI_MISO           (GPIO167 | 0x80000000)//(67 | 0x80000000)
#define GPIO_FPC_SPI_MOSI           (GPIO168 | 0x80000000)//(68 | 0x80000000)
#define GPIO_PIN_IRQ                (GPIO3   | 0x80000000)
#define GPIO_PIN_RESET              (GPIO115 | 0x80000000)

/** end by phg **/

extern void mt_spi_enable_clk(struct mt_spi_t *ms);
extern void mt_spi_disable_clk(struct mt_spi_t *ms);

struct mt_spi_t *fpc_ms;

typedef struct {
	struct spi_device      *spi;
	struct class           *class;
	struct device          *device;
	struct input_dev       *input_dev;
} fpc1020_data_t;

static int fpc1020_spi_probe(struct spi_device *spi);

// Platform specific

#define GPIO_CONFIGURE_IN(pin)  mt_set_gpio_mode(pin, GPIO_MODE_00) | mt_set_gpio_dir(pin, GPIO_DIR_IN)
#define GPIO_CONFIGURE_OUT(pin, default) mt_set_gpio_mode(pin, GPIO_MODE_00) | mt_set_gpio_dir(pin, GPIO_DIR_OUT) | mt_set_gpio_out(pin, default)
#define GPIO_SET(pin, data) mt_set_gpio_out(pin, data)
#define GPIO_GET(pin) mt_get_gpio_in(pin)
/***
void fpc_spi_enable_clk(void)
{
	enable_clock(MT_CG_PERI_SPI0, "spi");
}
void fpc_spi_disable_clk(void)
{
	disable_clock(MT_CG_PERI_SPI0, "spi");
}
******/
struct fpc1020_data {
	struct device *dev;
	struct platform_device *pldev;
	int irq_gpio;
	int rst_gpio;
        int irq_num;
	struct mutex lock;
	bool wakeup_enabled;
  	struct wake_lock ttw_wl;
};


static int hw_reset(struct  fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;

	GPIO_SET(fpc1020->rst_gpio, 0);
	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	GPIO_SET(fpc1020->rst_gpio, 1);
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	irq_gpio = GPIO_GET(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);
	dev_info( dev, "Using GPIO#%d as IRQ.\n", fpc1020->irq_gpio );
	dev_info( dev, "Using GPIO#%d as RST.\n", fpc1020->rst_gpio );

	return 0;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(fpc1020);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
	{
		fpc1020->wakeup_enabled = true;
		smp_wmb();
	}
	else if (!strncmp(buf, "disable", strlen("disable")))
	{
		fpc1020->wakeup_enabled = false;
		smp_wmb();
	}
	else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

static ssize_t clk_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	//struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	printk(KERN_INFO "%s\n", __func__);//damon
	if (!strncmp(buf, "1", strlen("1")))
	{
		mt_spi_enable_clk(fpc_ms);
	}
	else if (!strncmp(buf, "0", strlen("0")))
	{
		mt_spi_disable_clk(fpc_ms);
	}
	return count;
}

static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			     struct device_attribute *attribute,
			     char* buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	int irq = GPIO_GET(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}


/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			     struct device_attribute *attribute,
			     const char *buffer, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	return count;
}

static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);


static struct attribute * fpc1020_attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group const fpc1020_attribute_group = {
	.attrs = fpc1020_attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;
	dev_dbg(fpc1020->dev, "%s\n", __func__);

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();

	if (fpc1020->wakeup_enabled) {
		wake_lock_timeout(&fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}


int fpc1020_get_irqNum(void)
{
        unsigned int gpiopin, debounce, fpc_irq = 0;
        u32 ints[2] = {0, 0};
        struct device_node *node;

        node = of_find_compatible_node(NULL, NULL, "fpc,fpc_irq");
        if (node) {
                of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
                gpiopin = ints[0];
                debounce = ints[1];
                /*mt_gpio_set_debounce(gpiopin, debounce);*/
                fpc_irq = irq_of_parse_and_map(node, 0);
                pr_err("fpc_irq = %u\n", fpc_irq);
        } else
                pr_err("[fpc]%s can't find compatible node\n", __func__);

return fpc_irq;
}


static int fpc1020_probe(struct platform_device *pldev)
{
	struct device *dev = &pldev->dev;
	int rc = 0;
	int irqf;
	struct device_node *np = dev->of_node;
	struct fpc1020_data *fpc1020;


	mt_set_gpio_mode(GPIO_FPC_SPI_CS, GPIO_MODE_01);
        mt_set_gpio_mode(GPIO_FPC_SPI_CLK, GPIO_MODE_01);
        mt_set_gpio_mode(GPIO_FPC_SPI_MISO, GPIO_MODE_01);
        mt_set_gpio_mode(GPIO_FPC_SPI_MOSI, GPIO_MODE_01);


	fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);
	fpc1020->pldev = pldev;

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

#if 0
		goto exit;
#else
	fpc1020->irq_gpio = GPIO_PIN_IRQ;
        fpc1020->rst_gpio = GPIO_PIN_RESET;
#endif
	dev_dbg( dev, "Using GPIO#%d as IRQ.\n", fpc1020->irq_gpio );
	dev_dbg( dev, "Using GPIO#%d as RST.\n", fpc1020->rst_gpio );

	/* Configure the direction of the gpios */
	rc = GPIO_CONFIGURE_IN(fpc1020->irq_gpio);
	{
		rc = GPIO_CONFIGURE_OUT( fpc1020->rst_gpio, 1 );
	}

	if (rc < 0) {
		dev_err(&fpc1020->pldev->dev,
				"gpio_direction_output failed for RST.\n");
		return rc;
	}

	fpc1020->irq_num=fpc1020_get_irqNum();
	fpc1020->wakeup_enabled = false;

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}
	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, fpc1020->irq_num,
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
					fpc1020->irq_num);
		goto exit;
	}
	dev_dbg(dev, "requested irq %d\n",fpc1020->irq_num);

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(fpc1020->irq_num);
	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	dev_dbg(dev, "%s\n", __func__);
	mt_spi_enable_clk(fpc_ms);
	dev_dbg(dev, "spi enabled\n");

	rc = sysfs_create_group(&dev->kobj, &fpc1020_attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}

static int fpc1020_remove(struct platform_device *pldev)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(&pldev->dev);

	sysfs_remove_group(&pldev->dev.kobj, &fpc1020_attribute_group);
	mutex_destroy(&fpc1020->lock);
	wake_lock_destroy(&fpc1020->ttw_wl);
	dev_info(&pldev->dev, "%s\n", __func__);
	return 0;
}

#if 1   //fiona

static int fpc1020_spi_probe(struct spi_device *spi)
{
	int error = 0;
	fpc1020_data_t *fpc1020 = NULL;
	/* size_t buffer_size; */

	pr_err("fpc1020_spi_probe enter++++++\n");


	fpc1020 = kzalloc(sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		/*
		dev_err(&spi->dev,
		"failed to allocate memory for struct fpc1020_data\n");
		*/
		return -ENOMEM;
	}

	pr_alert("%s\n", __func__);


	spi_set_drvdata(spi, fpc1020);
	fpc1020->spi = spi;

        fpc_ms=spi_master_get_devdata(spi->master);



//err_chrdev:
//	unregister_chrdev_region(fpc1020->devno, 1);

//	fpc1020_manage_sysfs(fpc1020, spi, false);

	return error;
}

/* -------------------------------------------------------------------- */
#if 1
static int fpc1020_spi_remove(struct spi_device *spi)
{
//	fpc1020_data_t *fpc1020 = spi_get_drvdata(spi);

//	pr_debug("%s\n", __func__);

//	fpc1020_manage_sysfs(fpc1020, spi, false);

	//fpc1020_sleep(fpc1020, true);

	//cdev_del(&fpc1020->cdev);

	//unregister_chrdev_region(fpc1020->devno, 1);

	//fpc1020_cleanup(fpc1020, spi);

	return 0;
}
#endif

#endif



static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc_irq", },
	{}
};

//add by peihonggang

static struct of_device_id fpc1020_spi_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};

MODULE_DEVICE_TABLE(of, fpc1020_spi_of_match);
static struct spi_driver spi_driver = {
	.driver = {
		.name	= "fpcspi",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_spi_of_match,
        .bus	= &spi_bus_type,
	},
	.probe	= fpc1020_spi_probe,
	.remove	= fpc1020_spi_remove
};
// end by


MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc_irq",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove
};

static int __init fpc1020_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
	if (spi_register_driver(&spi_driver))
	{
		printk(KERN_INFO "register spi driver fail%s\n", __func__);
		return -EINVAL;
	}	

	return (platform_driver_register(&fpc1020_driver) != 0)? EINVAL : 0;
}

static void __exit fpc1020_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
	spi_unregister_driver(&spi_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");