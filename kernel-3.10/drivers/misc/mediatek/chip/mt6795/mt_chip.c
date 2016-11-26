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
#include <linux/of_address.h>
#include <linux/printk.h>

#include <mach/mt_chip.h>
#include <mach/mt_typedefs.h>
#include <asm/setup.h>
#include <mach/mt_devinfo.h>

void __iomem *APHW_CODE = NULL;
void __iomem *APHW_SUBCODE = NULL;
void __iomem *APHW_VER = NULL;
void __iomem *APSW_VER = NULL;

enum {
	CID_UNINIT = 0,
	CID_INITIALIZING = 1,
	CID_INITIALIZED = 2,
} CID_INIT_STATE;

static atomic_t g_cid_init = ATOMIC_INIT(CID_UNINIT);

static void init_chip_id(unsigned int line)
{
	if (CID_INITIALIZED == atomic_read(&g_cid_init))
		return;

	if (CID_INITIALIZING == atomic_read(&g_cid_init)) {
		pr_warn("%s (%d) state(%d)\n", __func__, line, atomic_read(&g_cid_init));
		return;
	}

	atomic_set(&g_cid_init, CID_INITIALIZING);
#ifdef CONFIG_OF
	{
		struct device_node *node = of_find_compatible_node(NULL, NULL, "mediatek,CHIPID");

		if (node) {
			APHW_CODE = of_iomap(node, 0);
			WARN(!APHW_CODE, "unable to map APHW_CODE registers\n");
			APHW_SUBCODE = of_iomap(node, 1);
			WARN(!APHW_SUBCODE, "unable to map APHW_SUBCODE registers\n");
			APHW_VER = of_iomap(node, 2);
			WARN(!APHW_VER, "unable to map APHW_VER registers\n");
			APSW_VER = of_iomap(node, 3);
			WARN(!APSW_VER, "unable to map APSW_VER registers\n");
			atomic_set(&g_cid_init, CID_INITIALIZED);
		} else {
			atomic_set(&g_cid_init, CID_UNINIT);
			pr_warn("node not found\n");
		}
	}
#endif
	pr_debug("0x%04X 0x%04X 0x%04X 0x%04X (%04d)\n",
		 readl(IOMEM(APHW_CODE)), readl(IOMEM(APHW_VER)),
		 readl(IOMEM(APSW_VER)), readl(IOMEM(APHW_SUBCODE)), line);
}

/* return hardware version */
unsigned int __chip_hw_code(void)
{
	init_chip_id(__LINE__);
	return (APHW_CODE) ? readl(IOMEM(APHW_CODE)) : (C_UNKNOWN_CHIP_ID);
}

unsigned int __chip_hw_ver(void)
{
	init_chip_id(__LINE__);
	return (APHW_VER) ? readl(IOMEM(APHW_VER)) : (C_UNKNOWN_CHIP_ID);
}

unsigned int __chip_sw_ver(void)
{
	init_chip_id(__LINE__);
	return (APSW_VER) ? readl(IOMEM(APSW_VER)) : (C_UNKNOWN_CHIP_ID);
}

unsigned int __chip_hw_subcode(void)
{
	init_chip_id(__LINE__);
	return (APHW_SUBCODE) ? readl(IOMEM(APHW_SUBCODE)) : (C_UNKNOWN_CHIP_ID);
}

unsigned int __chip_func_code(void)
{
	unsigned int val = get_devinfo_with_index(24) & 0xFF000000;	/*[31:24]*/

	return val >> 24;
}

unsigned int __chip_date_code(void)
{
	unsigned int val = get_devinfo_with_index(24) & 0x00FFC000;	/*[23:14]*/

	return val >> 14;
}

unsigned int __chip_proj_code(void)
{
	unsigned int val = get_devinfo_with_index(24) & 0x00003FFF;	/*[13:0]*/

	return val;
}

unsigned int __chip_fab_code(void)
{
	unsigned int val = get_devinfo_with_index(25) & 0x70000000;	/*[30:28]*/

	return val >> 28;
}

unsigned int __chip_wafer_big_ver(void)
{
	unsigned int val = get_devinfo_with_index(16) & 0x03000000;	/*[25:24]*/

	return val >> 24;
}

unsigned int mt_get_chip_id(void)
{
	unsigned int chip_id = __chip_hw_code();
	/*convert id if necessary */
	return chip_id;
}
EXPORT_SYMBOL(mt_get_chip_id);

unsigned int mt_get_chip_hw_code(void)
{
	return __chip_hw_code();
}
EXPORT_SYMBOL(mt_get_chip_hw_code);

unsigned int mt_get_chip_hw_ver(void)
{
	return __chip_hw_ver();
}

unsigned int mt_get_chip_hw_subcode(void)
{
	return __chip_hw_subcode();
}

chip_sw_ver_t mt_get_chip_sw_ver(void)
{
	return (chip_sw_ver_t) __chip_sw_ver();
}
EXPORT_SYMBOL(mt_get_chip_sw_ver);

static chip_info_cb g_cbs[CHIP_INFO_MAX] = {
	NULL,
	mt_get_chip_hw_code,
	mt_get_chip_hw_subcode,
	mt_get_chip_hw_ver,
	(chip_info_cb) mt_get_chip_sw_ver,

	__chip_hw_code,
	__chip_hw_subcode,
	__chip_hw_ver,
	__chip_sw_ver,

	__chip_func_code,
	__chip_date_code,
	__chip_proj_code,
	__chip_fab_code,
	__chip_wafer_big_ver,
};

unsigned int mt_get_chip_info(chip_info_t id)
{
	if ((id <= CHIP_INFO_NONE) || (id >= CHIP_INFO_MAX))
		return 0;
	else if (NULL == g_cbs[id])
		return 0;
	return g_cbs[id] ();
}
EXPORT_SYMBOL(mt_get_chip_info);

int __init chip_mod_init(void)
{
	struct mt_chip_drv *p_drv = get_mt_chip_drv();

	pr_debug("CODE = %04x %04x %04x %04x, %04x %04x %04x %04x, %04X %04X\n",
		 __chip_hw_code(), __chip_hw_subcode(), __chip_hw_ver(),
		 __chip_sw_ver(), __chip_func_code(), __chip_proj_code(),
		 __chip_date_code(), __chip_fab_code(), mt_get_chip_hw_ver(), mt_get_chip_sw_ver());

	p_drv->info_bit_mask |= CHIP_INFO_BIT(CHIP_INFO_HW_CODE) |
	    CHIP_INFO_BIT(CHIP_INFO_HW_SUBCODE) |
	    CHIP_INFO_BIT(CHIP_INFO_HW_VER) |
	    CHIP_INFO_BIT(CHIP_INFO_SW_VER) |
	    CHIP_INFO_BIT(CHIP_INFO_FUNCTION_CODE) |
	    CHIP_INFO_BIT(CHIP_INFO_DATE_CODE) |
	    CHIP_INFO_BIT(CHIP_INFO_PROJECT_CODE) |
	    CHIP_INFO_BIT(CHIP_INFO_FAB_CODE) | CHIP_INFO_BIT(CHIP_INFO_WAFER_BIG_VER);

	p_drv->get_chip_info = mt_get_chip_info;

	pr_debug("CODE = %08X %p", p_drv->info_bit_mask, p_drv->get_chip_info);

	return 0;
}

core_initcall(chip_mod_init);
MODULE_DESCRIPTION("MTK Chip Information");
MODULE_LICENSE("GPL");
