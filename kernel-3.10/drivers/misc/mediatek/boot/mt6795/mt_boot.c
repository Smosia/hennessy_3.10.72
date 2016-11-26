#define pr_fmt(fmt) "["KBUILD_MODNAME"] " fmt
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/of.h>

#include <mach/mt_boot.h>
#include <mach/mt_typedefs.h>
#include <asm/setup.h>

META_COM_TYPE g_meta_com_type = META_UNKNOWN_COM;
unsigned int g_meta_com_id = 0;

static struct platform_driver meta_com_type_info = {
	.driver = {
		   .name = "meta_com_type_info",
		   .bus = &platform_bus_type,
		   .owner = THIS_MODULE,
		   },
	.id_table = NULL,
};

static struct platform_driver meta_com_id_info = {
	.driver = {
		   .name = "meta_com_id_info",
		   .bus = &platform_bus_type,
		   .owner = THIS_MODULE,
		   },
	.id_table = NULL,
};

#ifdef CONFIG_OF
struct boot_tag_meta_com {
	u32 size;
	u32 tag;
	u32 meta_com_type;	/* identify meta via uart or usb */
	u32 meta_com_id;	/* multiple meta need to know com port id */
};
#endif

/* usb android will check whether is com port enabled default.
   in normal boot it is default enabled. */
bool com_is_enable(void)
{
	if (get_boot_mode() == NORMAL_BOOT)
		return false;
	else
		return true;
}

void set_meta_com(META_COM_TYPE type, unsigned int id)
{
	g_meta_com_type = type;
	g_meta_com_id = id;
}

META_COM_TYPE get_meta_com_type(void)
{
	return g_meta_com_type;
}

unsigned int get_meta_com_id(void)
{
	return g_meta_com_id;
}

static ssize_t meta_com_type_show(struct device_driver *driver, char *buf)
{
	return sprintf(buf, "%d\n", g_meta_com_type);
}

static ssize_t meta_com_type_store(struct device_driver *driver, const char *buf, size_t count)
{
	/*Do nothing */
	return count;
}

DRIVER_ATTR(meta_com_type_info, 0644, meta_com_type_show, meta_com_type_store);


static ssize_t meta_com_id_show(struct device_driver *driver, char *buf)
{
	return sprintf(buf, "%d\n", g_meta_com_id);
}

static ssize_t meta_com_id_store(struct device_driver *driver, const char *buf, size_t count)
{
	/*Do nothing */
	return count;
}

DRIVER_ATTR(meta_com_id_info, 0644, meta_com_id_show, meta_com_id_store);

static int __init create_sysfs(void)
{
	int ret;
	BOOTMODE bm = get_boot_mode();

#ifdef CONFIG_OF
	if (of_chosen) {
		struct boot_tag_meta_com *tags;

		tags = (struct boot_tag_meta_com *)of_get_property(of_chosen, "atag,meta", NULL);
		if (tags) {
			g_meta_com_type = tags->meta_com_type;
			g_meta_com_id = tags->meta_com_id;
			pr_debug("[%s] g_meta_com_type = %d, g_meta_com_id = %d.\n", __func__,
				  g_meta_com_type, g_meta_com_id);
		} else
			pr_warn("[%s] No atag,meta found !\n", __func__);
	} else
		pr_warn("[%s] of_chosen is NULL !\n", __func__);
#endif

	if (bm == META_BOOT || bm == ADVMETA_BOOT || bm == ATE_FACTORY_BOOT || bm == FACTORY_BOOT) {
		/* register driver and create sysfs files */
		ret = driver_register(&meta_com_type_info.driver);
		if (ret)
			pr_warn("fail to register META COM TYPE driver\n");
		ret =
		    driver_create_file(&meta_com_type_info.driver, &driver_attr_meta_com_type_info);
		if (ret)
			pr_warn("fail to create META COM TPYE sysfs file\n");

		ret = driver_register(&meta_com_id_info.driver);
		if (ret)
			pr_warn("fail to register META COM ID driver\n");
		ret = driver_create_file(&meta_com_id_info.driver, &driver_attr_meta_com_id_info);
		if (ret)
			pr_warn("fail to create META COM ID sysfs file\n");
	}

	return 0;
}

static int __init boot_mod_init(void)
{
	create_sysfs();
	return 0;
}

static void __exit boot_mod_exit(void)
{
}

module_init(boot_mod_init);
module_exit(boot_mod_exit);
MODULE_DESCRIPTION("MTK Boot Information Querying Driver");
MODULE_LICENSE("GPL");
