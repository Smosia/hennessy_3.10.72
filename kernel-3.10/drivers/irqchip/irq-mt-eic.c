#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/rbtree.h>
#include <linux/irqchip/mt-eic.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <mach/sync_write.h>
#include <mach/mt_gpio.h>

#define EINT_DEBUG 0
#if (EINT_DEBUG == 1)
#define dbgmsg printk
#else
#define dbgmsg(...)
#endif

static unsigned int EINT_IRQ_BASE;

#if 0
#define EINT_TEST_V2
#endif

#ifdef CONFIG_MTK_LEGACY
#define MD_EINT
#endif

#if 0
#define EINT_TEST
#endif
#if 0
#define EINT_TEST_SOFT
#endif
#if 0
#define DEINT_SUPPORT
#endif

#ifdef MD_EINT
#include <mach/md_eint.h>
/*For sim_hot_plug*/

enum {
	SIM_HOT_PLUG_EINT_NUMBER,
	SIM_HOT_PLUG_EINT_DEBOUNCETIME,
	SIM_HOT_PLUG_EINT_POLARITY,
	SIM_HOT_PLUG_EINT_SENSITIVITY,
	SIM_HOT_PLUG_EINT_SOCKETTYPE,
	SIM_HOT_PLUG_EINT_DEDICATEDEN,
	SIM_HOT_PLUG_EINT_SRCPIN,
} sim_hot_plug_eint_queryType;

enum {
	ERR_SIM_HOT_PLUG_NULL_POINTER = -13,
	ERR_SIM_HOT_PLUG_QUERY_TYPE,
	ERR_SIM_HOT_PLUG_QUERY_STRING,
} sim_hot_plug_eint_queryErr;
#endif

typedef void (*eint_funct) (void);
struct eint_func {
	eint_funct *eint_func;
	unsigned int *eint_auto_umask;
	/*is_deb_en: 1 means enable, 0 means disable */
	unsigned int *is_deb_en;
	unsigned int *deb_time;
	struct timer_list *eint_sw_deb_timer;
	unsigned int *count;
};

static struct eint_func EINT_FUNC;

static unsigned int EINT_MAX_CHANNEL;
static unsigned int MAX_DEINT_CNT;

static void __iomem *EINT_BASE;

static unsigned int mapping_table_entry;

#define MT_EINT_IRQ_ID  185
struct wake_lock EINT_suspend_lock;
#if defined(EINT_TEST_V2)
static unsigned int cur_eint_num;
#endif
static DEFINE_SPINLOCK(eint_lock);

struct deint_des {
	int eint_num;
	int irq_num;
	int used;
};

static u32 *deint_possible_irq;
static struct deint_des *deint_descriptors;

static void mt_eint_clr_deint_selection(u32 deint_mapped)
{
	if (deint_mapped < 4) {
		writel(0xff << (deint_mapped * 8), IOMEM(DEINT_SEL_CLR_BASE));
	} else if ((deint_mapped >= 4) && (deint_mapped < 8)) {
		writel(0xff << (deint_mapped * 8),
		       IOMEM(DEINT_SEL_CLR_BASE + 4));
	}
}

static void mt_eint_set_deint_selection(u32 eint_num, u32 deint_mapped)
{
	/* set our new deint_sel setting */
	if (deint_mapped < 4) {
		writel((eint_num << (deint_mapped * 8)),
		       IOMEM(DEINT_SEL_SET_BASE));
	} else if ((deint_mapped >= 4) && (deint_mapped < 8)) {
		writel((eint_num << (deint_mapped * 8)),
		       IOMEM(DEINT_SEL_SET_BASE + 4));
	}
}

static void mt_eint_enable_deint_selection(u32 deint_mapped)
{
	writel(readl(IOMEM(DEINT_CON_BASE)) | (1 << deint_mapped),
	       IOMEM(DEINT_CON_BASE));
}

int mt_eint_clr_deint(u32 eint_num)
{
	u32 deint_mapped = 0;

	if (eint_num >= EINT_MAX_CHANNEL) {
		pr_err("%s: eint_num(%u) is not in (0, %d)\n", __func__,
		       eint_num, EINT_MAX_CHANNEL);
		return -1;
	}

	for (deint_mapped = 0; deint_mapped < MAX_DEINT_CNT; ++deint_mapped) {
		if (deint_descriptors[deint_mapped].eint_num == eint_num) {
			deint_descriptors[deint_mapped].eint_num = 0;
			deint_descriptors[deint_mapped].irq_num = 0;
			deint_descriptors[deint_mapped].used = 0;
			break;
		}
	}

	if (deint_mapped == MAX_DEINT_CNT) {
		pr_err("%s: no deint(%d) used now\n", __func__, eint_num);
		return -1;
	}

	mt_eint_clr_deint_selection(deint_mapped);

	return 0;
}
EXPORT_SYMBOL(mt_eint_clr_deint);

int mt_eint_set_deint(u32 eint_num, u32 irq_num)
{
	u32 deint_mapped = 0;
	int i = 0;

	if (eint_num >= EINT_MAX_CHANNEL) {
		pr_err("%s: eint_num(%u) is not in (0, %d)\n", __func__,
		       eint_num, EINT_MAX_CHANNEL);
		return -1;
	}

	/* first usable deint descriptor */
	for (deint_mapped = 0; deint_mapped < MAX_DEINT_CNT; ++deint_mapped) {
		if (deint_descriptors[deint_mapped].used == 0) {
			deint_descriptors[deint_mapped].eint_num = eint_num;
			deint_descriptors[deint_mapped].irq_num = irq_num;
			deint_descriptors[deint_mapped].used = 1;
			break;
		}
	}

	if (deint_mapped == MAX_DEINT_CNT) {
		pr_err("%s: no idle deint now\n", __func__);
		return -1;
	}

	for (i = 0; i < MAX_DEINT_CNT; ++i) {
		if (deint_possible_irq[i] == irq_num)
			break;
	}

	if (i == MAX_DEINT_CNT) {
		pr_err("%s: no matched possible deint irq for %u\n", __func__,
		       irq_num);
		dump_stack();
		mt_eint_clr_deint(eint_num);
		return -1;
	}

	/* mask from eintc, only triggered by GIC */
	mt_eint_mask(eint_num);

	/* set eint part as high-level to bypass signal to GIC */
	mt_eint_set_polarity(eint_num, MT_POLARITY_HIGH);
	mt_eint_set_sens(eint_num, MT_LEVEL_SENSITIVE);

	mt_eint_clr_deint_selection(deint_mapped);
	mt_eint_set_deint_selection(eint_num, deint_mapped);
	mt_eint_enable_deint_selection(deint_mapped);

	return 0;
}
EXPORT_SYMBOL(mt_eint_set_deint);

/*
 * mt_eint_get_mask: To get the eint mask
 * @eint_num: the EINT number to get
 */
static unsigned int mt_eint_get_mask(unsigned int eint_num)
{
	unsigned long base;
	unsigned int st;
	unsigned int bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_MASK_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return 0;
	}

	st = readl(IOMEM(base));
	if (st & bit)
		st = 1;		/* masked */
	else
		st = 0;		/* unmasked */

	return st;
}

#if 0
void mt_eint_mask_all(void)
{
	unsigned long base;
	unsigned int val = 0xFFFFFFFF, ap_cnt = (EINT_MAX_CHANNEL / 32), i;
	if (EINT_MAX_CHANNEL % 32)
		ap_cnt++;
	dbgmsg("[EINT] cnt:%d\n", ap_cnt);

	base = EINT_MASK_SET_BASE;
	for (i = 0; i < ap_cnt; i++) {
		writel(val, IOMEM(base + (i * 4)));
		dbgmsg("[EINT] mask addr:%x = %x\n", EINT_MASK_BASE + (i * 4),
		       readl(IOMEM(EINT_MASK_BASE + (i * 4))));
	}
}

/*
 * mt_eint_unmask_all: Mask the specified EINT number.
 */
void mt_eint_unmask_all(void)
{
	unsigned long base;
	unsigned int val = 0xFFFFFFFF, ap_cnt = (EINT_MAX_CHANNEL / 32), i;
	if (EINT_MAX_CHANNEL % 32)
		ap_cnt++;
	dbgmsg("[EINT] cnt:%d\n", ap_cnt);

	base = EINT_MASK_CLR_BASE;
	for (i = 0; i < ap_cnt; i++) {
		writel(val, IOMEM(base + (i * 4)));
		dbgmsg("[EINT] unmask addr:%x = %x\n", EINT_MASK_BASE + (i * 4),
		       readl(IOMEM(EINT_MASK_BASE + (i * 4))));
	}
}

/*
 * mt_eint_get_soft: To get the eint mask
 * @eint_num: the EINT number to get
 */
unsigned int mt_eint_get_soft(unsigned int eint_num)
{
	unsigned long base;
	unsigned int st;

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_SOFT_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return 0;
	}
	st = readl(IOMEM(base));

	return st;
}
#endif

#if 0
/*
 * mt_eint_emu_set: Trigger the specified EINT number.
 * @eint_num: EINT number to set
 */
void mt_eint_emu_set(unsigned int eint_num)
{
	unsigned long base = 0;
	unsigned int bit = 1 << (eint_num % 32);
	unsigned int value = 0;

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_EMUL_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}
	value = readl(IOMEM(base));
	value = bit | value;
	writel(value, IOMEM(base));
	value = readl(IOMEM(base));

	dbgmsg("[EINT] emul set addr:%x = %x, bit=%x\n", base, value, bit);

}

/*
 * mt_eint_emu_clr: Trigger the specified EINT number.
 * @eint_num: EINT number to clr
 */
void mt_eint_emu_clr(unsigned int eint_num)
{
	unsigned long base = 0;
	unsigned int bit = 1 << (eint_num % 32);
	unsigned int value = 0;

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_EMUL_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}
	value = readl(IOMEM(base));
	value = (~bit) & value;
	writel(value, IOMEM(base));
	value = readl(IOMEM(base));

	dbgmsg("[EINT] emul clr addr:%x = %x, bit=%x\n", base, value, bit);

}

/*
 * eint_send_pulse: Trigger the specified EINT number.
 * @eint_num: EINT number to send
 */
inline void mt_eint_send_pulse(unsigned int eint_num)
{
	unsigned long base_set = (eint_num / 32) * 4 + EINT_SOFT_SET_BASE;
	unsigned long base_clr = (eint_num / 32) * 4 + EINT_SOFT_CLR_BASE;
	unsigned int bit = 1 << (eint_num % 32);
	if (eint_num < EINT_MAX_CHANNEL) {
		base_set = (eint_num / 32) * 4 + EINT_SOFT_SET_BASE;
		base_clr = (eint_num / 32) * 4 + EINT_SOFT_CLR_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}

	writel(bit, IOMEM(base_set));
	writel(bit, IOMEM(base_clr));
}
#endif

#ifdef EINT_TEST_V2
/*
 * mt_eint_soft_set: Trigger the specified EINT number.
 * @eint_num: EINT number to set
 */
static void mt_eint_soft_set(unsigned int eint_num)
{

	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_SOFT_SET_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}
	writel(bit, IOMEM(base));

	dbgmsg("[EINT] soft set addr:%x = %x\n", base, bit);

}

/*
 * mt_eint_soft_clr: Unmask the specified EINT number.
 * @eint_num: EINT number to clear
 */
static void mt_eint_soft_clr(unsigned int eint_num)
{
	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_SOFT_CLR_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}
	writel(bit, IOMEM(base));

	dbgmsg("[EINT] soft clr addr:%x = %x\n", base, bit);

}
#endif /* end of EINT_TEST_V2 */

/*
 * mt_eint_mask: Mask the specified EINT number.
 * @eint_num: EINT number to mask
 */
void mt_eint_mask(unsigned int eint_num)
{
	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_MASK_SET_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}
	mt_reg_sync_writel(bit, base);

	dbgmsg("[EINT] mask addr:%lx = %x\n", base, bit);
}
EXPORT_SYMBOL(mt_eint_mask);

/*
 * mt_eint_unmask: Unmask the specified EINT number.
 * @eint_num: EINT number to unmask
 */
void mt_eint_unmask(unsigned int eint_num)
{
	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_MASK_CLR_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}
	mt_reg_sync_writel(bit, base);

	dbgmsg("[EINT] unmask addr:%lx = %x\n", base, bit);
}
EXPORT_SYMBOL(mt_eint_unmask);

/*
 * mt_eint_set_polarity: Set the polarity for the EINT number.
 * @eint_num: EINT number to set
 * @pol: polarity to set
 */
void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol)
{
	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);

	if (pol == MT_EINT_POL_NEG) {
		if (eint_num < EINT_MAX_CHANNEL) {
			base = (eint_num / 32) * 4 + EINT_POL_CLR_BASE;
		} else {
			dbgmsg
			    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
			     __func__, eint_num);
			return;
		}
	} else {
		if (eint_num < EINT_MAX_CHANNEL) {
			base = (eint_num / 32) * 4 + EINT_POL_SET_BASE;
		} else {
			dbgmsg
			    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
			     __func__, eint_num);
			return;
		}
	}
	mt_reg_sync_writel(bit, base);
	dbgmsg("[EINT] %s :%lx, bit: %x\n", __func__, base, bit);

	/* accodring to DE's opinion, the longest latency need is about 250 ns */
	ndelay(300);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_INTACK_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return;
	}
	mt_reg_sync_writel(bit, base);
}
EXPORT_SYMBOL(mt_eint_set_polarity);

/*
 * mt_eint_get_polarity: Set the polarity for the EINT number.
 * @eint_num: EINT number to get
 * Return: polarity type
 * EINT driver INTERNAL use
 */
int mt_eint_get_polarity(unsigned int eint_num)
{
	unsigned int val;
	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);
	unsigned int pol;

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_POL_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return -1;
	}
	val = readl(IOMEM(base));

	dbgmsg("[EINT] %s :%lx, bit:%x, val:%x\n", __func__, base, bit, val);
	if (val & bit)
		pol = MT_EINT_POL_POS;
	else
		pol = MT_EINT_POL_NEG;

	return pol;
}

/* For new EINT SW arch. input is virtual irq */
int mt_eint_get_polarity_external(unsigned int irq_num)
{
	unsigned int val;
	unsigned long base;
	unsigned int bit;
	unsigned int pol;
	unsigned int eint_num;

	eint_num = irq_num - EINT_IRQ_BASE;

	bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_POL_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return -1;
	}
	val = readl(IOMEM(base));

	dbgmsg("[EINT] %s :%lx, bit:%x, val:%x\n", __func__, base, bit, val);
	if (val & bit)
		pol = MT_EINT_POL_POS;
	else
		pol = MT_EINT_POL_NEG;

	return pol;
}

void mt_eint_revert_polarity(unsigned int eint_num)
{
	unsigned int pol;
	if (mt_eint_get_polarity(eint_num))
		pol = 0;
	else
		pol = 1;

	mt_eint_set_polarity(eint_num, pol);
}

/*
 * mt_eint_set_sens: Set the sensitivity for the EINT number.
 * @eint_num: EINT number to set
 * @sens: sensitivity to set
 * Always return 0.
 */
unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens)
{
	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);

	if (sens == MT_EDGE_SENSITIVE) {
		if (eint_num < EINT_MAX_CHANNEL) {
			base = (eint_num / 32) * 4 + EINT_SENS_CLR_BASE;
		} else {
			dbgmsg
			    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
			     __func__, eint_num);
			return 0;
		}
	} else if (sens == MT_LEVEL_SENSITIVE) {
		if (eint_num < EINT_MAX_CHANNEL) {
			base = (eint_num / 32) * 4 + EINT_SENS_SET_BASE;
		} else {
			dbgmsg
			    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
			     __func__, eint_num);
			return 0;
		}
	} else {
		pr_err("%s invalid sensitivity value\n", __func__);
		return 0;
	}
	mt_reg_sync_writel(bit, base);
	dbgmsg("[EINT] %s :%lx, bit: %x\n", __func__, base, bit);
	return 0;
}
EXPORT_SYMBOL(mt_eint_set_sens);

/*
 * mt_eint_get_sens: To get the eint sens
 * @eint_num: the EINT number to get
 */
static int mt_eint_get_sens(unsigned int eint_num)
{
	unsigned long base, sens;
	unsigned int bit = 1 << (eint_num % 32), st;

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_SENS_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return -1;
	}
	st = readl(IOMEM(base));
	if (st & bit)
		sens = MT_LEVEL_SENSITIVE;
	else
		sens = MT_EDGE_SENSITIVE;

	return sens;
}

/*
 * mt_eint_ack: To ack the interrupt
 * @eint_num: the EINT number to set
 */
unsigned int mt_eint_ack(unsigned int eint_num)
{
	unsigned long base;
	unsigned int bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_INTACK_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return 0;
	}
	mt_reg_sync_writel(bit, base);

	dbgmsg("[EINT] %s :%lx, bit: %x\n", __func__, base, bit);
	return 0;
}

/*
 * mt_eint_read_status: To read the interrupt status
 * @eint_num: the EINT number to set
 */
static unsigned int mt_eint_read_status(unsigned int eint_num)
{
	unsigned long base;
	unsigned int st;
	unsigned int bit = 1 << (eint_num % 32);

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_STA_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return 0;
	}
	st = readl(IOMEM(base));

	return st&bit;
}

/*
 * mt_eint_get_status: To get the interrupt status
 * @eint_num: the EINT number to get
 */
static int mt_eint_get_status(unsigned int eint_num)
{
	unsigned long base;
	unsigned int st;

	if (eint_num < EINT_MAX_CHANNEL) {
		base = (eint_num / 32) * 4 + EINT_STA_BASE;
	} else {
		dbgmsg
		    ("Error in %s [EINT] num:%d is larger than EINT_MAX_CHANNEL\n",
		     __func__, eint_num);
		return -1;
	}

	st = readl(IOMEM(base));
	return st;
}

#ifdef EINT_TEST_V2
/*
 * mt_eint_en_hw_debounce: To enable hw debounce
 * @eint_num: the EINT number to set
 */
static void mt_eint_en_hw_debounce(unsigned int eint_num)
{
	unsigned long base, bit;
	base = (eint_num / 4) * 4 + EINT_DBNC_SET_BASE;
	bit =
	    (EINT_DBNC_SET_EN << EINT_DBNC_SET_EN_BITS) << ((eint_num % 4) * 8);
	writel(bit, IOMEM(base));
	EINT_FUNC.is_deb_en[eint_num] = 1;
}
#endif /* end of EINT_TEST_V2 */

/*
 * mt_eint_dis_hw_debounce: To disable hw debounce
 * @eint_num: the EINT number to set
 */
static void mt_eint_dis_hw_debounce(unsigned int eint_num)
{
	unsigned long clr_base, bit;
	clr_base = (eint_num / 4) * 4 + EINT_DBNC_CLR_BASE;
	bit =
	    (EINT_DBNC_CLR_EN << EINT_DBNC_CLR_EN_BITS) << ((eint_num % 4) * 8);
	writel(bit, IOMEM(clr_base));
	EINT_FUNC.is_deb_en[eint_num] = 0;
}

/*
 * mt_eint_dis_sw_debounce: To set EINT_FUNC.is_deb_en[eint_num] disable
 * @eint_num: the EINT number to set
 */
static void mt_eint_dis_sw_debounce(unsigned int eint_num)
{
	if (eint_num < EINT_MAX_CHANNEL)
		EINT_FUNC.is_deb_en[eint_num] = 0;
}

/*
 * mt_eint_en_sw_debounce: To set EINT_FUNC.is_deb_en[eint_num] enable
 * @eint_num: the EINT number to set
 */
static void mt_eint_en_sw_debounce(unsigned int eint_num)
{
	if (eint_num < EINT_MAX_CHANNEL)
		EINT_FUNC.is_deb_en[eint_num] = 1;
}

/*
 * mt_can_en_debounce: Check the EINT number is able to enable debounce or not
 * @eint_num: the EINT number to set
 */
static unsigned int mt_can_en_debounce(unsigned int eint_num)
{
	unsigned int sens = mt_eint_get_sens(eint_num);
	/* debounce: debounce time is not 0 && it is not edge sensitive */
	if (EINT_FUNC.deb_time[eint_num] != 0 && sens != MT_EDGE_SENSITIVE)
		return 1;
	else {
		dbgmsg
		    ("Can't enable debounce of eint_num:%d, deb_time:%d, sens:%d\n",
		     eint_num, EINT_FUNC.deb_time[eint_num], sens);
		return 0;
	}
}

/*
 * mt_eint_set_hw_debounce: Set the de-bounce time for the specified EINT number.
 * @gpio_pin: EINT number to acknowledge
 * @ms: the de-bounce time to set (in miliseconds)
 */
void mt_eint_set_hw_debounce(unsigned int gpio_pin, unsigned int ms)
{
	unsigned int dbnc, bit, clr_bit, rst, unmask = 0, eint_num;
	unsigned long base, clr_base;

	eint_num = mt_gpio_to_irq(gpio_pin) - EINT_IRQ_BASE;

	if (eint_num >= EINT_MAX_CHANNEL) {
		pr_err("%s: eint_num %d invalid\n", __func__, eint_num);
		return;
	}

	base = (eint_num / 4) * 4 + EINT_DBNC_SET_BASE;
	clr_base = (eint_num / 4) * 4 + EINT_DBNC_CLR_BASE;
	EINT_FUNC.deb_time[eint_num] = ms;

	/*
	 * Don't enable debounce once debounce time is 0 or
	 * its type is edge sensitive.
	 */
	if (!mt_can_en_debounce(eint_num)) {
		dbgmsg("Can't enable debounce of eint_num:%d in %s\n", eint_num,
		       __func__);
		return;
	}

	if (ms == 0) {
		dbnc = 0;
		dbgmsg("ms should not be 0. eint_num:%d in %s\n", eint_num,
		       __func__);
	} else if (ms <= 1) {
		dbnc = 1;
	} else if (ms <= 16) {
		dbnc = 2;
	} else if (ms <= 32) {
		dbnc = 3;
	} else if (ms <= 64) {
		dbnc = 4;
	} else if (ms <= 128) {
		dbnc = 5;
	} else if (ms <= 256) {
		dbnc = 6;
	} else {
		dbnc = 7;
	}

	/* setp 1: mask the EINT */
	if (!mt_eint_get_mask(eint_num)) {
		mt_eint_mask(eint_num);
		unmask = 1;
	}
	/* step 2: Check hw debouce number to decide which type should be used */
	if (eint_num >= MAX_HW_DEBOUNCE_CNT)
		mt_eint_en_sw_debounce(eint_num);
	else {
		/* step 2.1: set hw debounce flag */
		EINT_FUNC.is_deb_en[eint_num] = 1;

		/* step 2.2: disable hw debounce */
		clr_bit = 0xFF << ((eint_num % 4) * 8);
		mt_reg_sync_writel(clr_bit, clr_base);

		/* step 2.3: set new debounce value */
		bit =
		    ((dbnc << EINT_DBNC_SET_DBNC_BITS) |
		     (EINT_DBNC_SET_EN << EINT_DBNC_SET_EN_BITS)) <<
		    ((eint_num % 4) * 8);
		mt_reg_sync_writel(bit, base);

		/* step 2.4: Delay a while (more than 2T) to wait for hw debounce enable work correctly */
		udelay(500);

		/* step 2.5: Reset hw debounce counter to avoid unexpected interrupt */
		rst = (EINT_DBNC_RST_BIT << EINT_DBNC_SET_RST_BITS) <<
		    ((eint_num % 4) * 8);
		mt_reg_sync_writel(rst, base);

		/* step 2.6: Delay a while (more than 2T) to wait for hw debounce counter reset work correctly */
		udelay(500);
	}
	/* step 3: unmask the EINT */
	if (unmask == 1)
		mt_eint_unmask(eint_num);
}
EXPORT_SYMBOL(mt_eint_set_hw_debounce);

/*
 * eint_do_tasklet: EINT tasklet function.
 * @unused: not use.
 */
static void eint_do_tasklet(unsigned long unused)
{
	wake_lock_timeout(&EINT_suspend_lock, HZ / 2);
}

DECLARE_TASKLET(eint_tasklet, eint_do_tasklet, 0);

/*
 * mt_eint_timer_event_handler: EINT sw debounce handler
 * @eint_num: the EINT number and use unsigned long to prevent
 *            compile warning of timer usage.
 */
static void mt_eint_timer_event_handler(unsigned long eint_num)
{
	unsigned int status;
	unsigned long flags;

	if (eint_num >= EINT_MAX_CHANNEL)
		return;

	/* disable interrupt for core 0 and it will run on core 0 only */
	local_irq_save(flags);
	mt_eint_unmask(eint_num);
	status = mt_eint_read_status(eint_num);
	dbgmsg("EINT Module - EINT_STA = 0x%x, in %s\n", status, __func__);
	if (!EINT_FUNC.eint_func[eint_num]) {
		dbgmsg("EINT Module_IRQ - EINT_STA = 0x%x, in %s\n", status,
		       __func__);
		if (status)
			generic_handle_irq(EINT_IRQ(eint_num));

		local_irq_restore(flags);
	} else {
		if (status) {
			mt_eint_mask(eint_num);
			if (EINT_FUNC.eint_func[eint_num])
				EINT_FUNC.eint_func[eint_num] ();
			mt_eint_ack(eint_num);
		}
		local_irq_restore(flags);
		if (EINT_FUNC.eint_auto_umask[eint_num])
			mt_eint_unmask(eint_num);
	}

	return;
}

/*
 * mt_eint_set_timer_event: To set a timer event for sw debounce.
 * @eint_num: the EINT number to set
 */
static void mt_eint_set_timer_event(unsigned int eint_num)
{
	struct timer_list *eint_timer = &EINT_FUNC.eint_sw_deb_timer[eint_num];
	/* assign this handler to execute on core 0 */
	int cpu = 0;

	/* register timer for this sw debounce eint */
	eint_timer->expires =
	    jiffies + msecs_to_jiffies(EINT_FUNC.deb_time[eint_num]);
	dbgmsg("EINT Module - expires:%lu, jiffies:%lu, deb_in_jiffies:%lu, ",
	       eint_timer->expires, jiffies,
	       msecs_to_jiffies(EINT_FUNC.deb_time[eint_num]));
	dbgmsg("deb:%d, in %s\n", EINT_FUNC.deb_time[eint_num], __func__);
	eint_timer->data = eint_num;
	eint_timer->function = &mt_eint_timer_event_handler;
	if (!timer_pending(eint_timer)) {
		init_timer(eint_timer);
		add_timer_on(eint_timer, cpu);
	}
}

/*
 * mt_eint_isr: EINT interrupt service routine.
 * @irq: EINT IRQ number
 * @desc: EINT IRQ descriptor
 * Return IRQ returned code.
 */
static irqreturn_t mt_eint_demux(unsigned irq, struct irq_desc *desc)
{
	unsigned int index, rst;
	unsigned long base;
	unsigned int status = 0;
	unsigned int status_check;
	unsigned int reg_base, offset;
	unsigned long long t1, t2;
	int mask_status = 0;
	struct irq_chip *chip = irq_get_chip(irq);
	chained_irq_enter(chip, desc);

	/*
	 * NoteXXX: Need to get the wake up for 0.5 seconds when an EINT intr tirggers.
	 *          This is used to prevent system from suspend such that other drivers
	 *          or applications can have enough time to obtain their own wake lock.
	 *          (This information is gotten from the power management owner.)
	 */

	tasklet_schedule(&eint_tasklet);
	dbgmsg("EINT Module - %s ISR Start\n", __func__);

	for (reg_base = 0; reg_base < EINT_MAX_CHANNEL; reg_base += 32) {
		/* read status register every 32 interrupts */
		status = mt_eint_get_status(reg_base);
		if (status) {
			dbgmsg("EINT Module - index:%d,EINT_STA = 0x%x\n",
			       reg_base, status);
		} else {
			continue;
		}
		for (offset = 0; offset < 32; offset++) {
			index = reg_base + offset;
			if (index >= EINT_MAX_CHANNEL)
				break;

			status_check = status & (1 << (index % 32));
			if (status_check) {
				dbgmsg("Got eint:%d\n", index);

				EINT_FUNC.count[index]++;

				/* deal with EINT from request_irq() */
				if (!EINT_FUNC.eint_func[index]) {
					dbgmsg("EINT %d: go with new mt_eint\n",
					       index);
					if ((EINT_FUNC.is_deb_en[index] == 1)
					    && (index >= MAX_HW_DEBOUNCE_CNT)) {
						/* if its debounce is enable and it is a sw debounce */
						mt_eint_mask(index);
						dbgmsg("got sw index %d\n",
						       index);
						mt_eint_set_timer_event(index);
					} else {
						dbgmsg("got hw index %d\n",
						       index);
						t1 = sched_clock();
						generic_handle_irq(index +
								   EINT_IRQ_BASE);
						t2 = sched_clock();
						if ((EINT_FUNC.is_deb_en[index] == 1)
						    && (index < MAX_HW_DEBOUNCE_CNT)) {
							if (mt_eint_get_mask(index) == 1)
								mask_status = 1;
							else
								mask_status = 0;

							mt_eint_mask(index);

							/* Don't need to use reset ? */
							/* reset debounce counter */
							base = (index / 4) * 4 + EINT_DBNC_SET_BASE;
							rst = (EINT_DBNC_RST_BIT<<EINT_DBNC_SET_RST_BITS)<<((index%4)*8);
							mt_reg_sync_writel(rst, base);

							if (mask_status == 0)
								mt_eint_unmask(index);
						}
#if (EINT_DEBUG == 1)
						dbgmsg("EINT Module - EINT_STA after ack = 0x%x\n",
						     mt_eint_get_status(index));
#endif

						if ((t2 - t1) > EINT_DELAY_WARNING)
							pr_warn("[EINT]Warn!EINT:%d run too long,s:%llu,e:%llu,total:%llu\n",
							     index, t1, t2, (t2 - t1));
					}
				}
				/* deal with EINT from mt_eint_registration() */
				else {
					mt_eint_mask(index);
					dbgmsg("EINT %d: go with original mt_eint\n", index);
					if ((EINT_FUNC.is_deb_en[index] == 1)
					    && (index >= MAX_HW_DEBOUNCE_CNT)) {
						/* if its debounce is enable and it is a sw debounce */
						mt_eint_set_timer_event(index);
					} else {
						/* HW debounce or no use debounce */
						t1 = sched_clock();
						if (EINT_FUNC.eint_func[index])
							EINT_FUNC.eint_func[index] ();

						t2 = sched_clock();
						mt_eint_ack(index);
#if 1				/* Don't need to use reset ? */
						/* reset debounce counter */
						base =
						    (index / 4) * 4 +
						    EINT_DBNC_SET_BASE;
						rst =
						    (EINT_DBNC_RST_BIT <<
						     EINT_DBNC_SET_RST_BITS) <<
						    ((index % 4) * 8);
						mt_reg_sync_writel(rst, base);
#endif
#if (EINT_DEBUG == 1)
						status = mt_eint_get_status(index);
						dbgmsg("EINT Module - EINT_STA after ack = 0x%x\n", status);
#endif
						if (EINT_FUNC.eint_auto_umask[index])
							mt_eint_unmask(index);

						if ((t2 - t1) > EINT_DELAY_WARNING)
							pr_warn("[EINT]Warn!EINT:%d run too long,s:%llu,e:%llu,total:%llu\n",
							     index, t1, t2, (t2 - t1));
					}
				}
			}
		}
	}

	dbgmsg("EINT Module - %s ISR END\n", __func__);
	chained_irq_exit(chip, desc);
	return IRQ_HANDLED;
}

#if (EINT_DEBUG == 1)
static int mt_eint_max_channel(void)
{
	return EINT_MAX_CHANNEL;
}
#endif

/*
 * mt_eint_dis_debounce: To disable debounce.
 * @eint_num: the EINT number to disable
 */
void mt_eint_dis_debounce(unsigned int eint_num)
{
	/* This function is used to disable debounce whether hw or sw */
	if (eint_num < MAX_HW_DEBOUNCE_CNT)
		mt_eint_dis_hw_debounce(eint_num);
	else
		mt_eint_dis_sw_debounce(eint_num);
}
EXPORT_SYMBOL(mt_eint_dis_debounce);

/*
 * mt_eint_registration: register a EINT.
 * @eint_num: the EINT number to register
 * @flag: the interrupt line behaviour to select
 * @EINT_FUNC_PTR: the ISR callback function
 * @is_auto_unmask: the indication flag of auto unmasking after ISR callback is processed
 */
#if 1
void mt_eint_registration(unsigned int eint_num, unsigned int flag,
			  void (EINT_FUNC_PTR) (void),
			  unsigned int is_auto_umask)
{
	if (eint_num < EINT_MAX_CHANNEL) {
		pr_notice("eint register for %d\n", eint_num);
		spin_lock(&eint_lock);
		mt_eint_mask(eint_num);

		if (flag & (EINTF_TRIGGER_RISING | EINTF_TRIGGER_FALLING)) {
			mt_eint_set_polarity(eint_num,
					     (flag & EINTF_TRIGGER_FALLING) ?
					     MT_EINT_POL_NEG : MT_EINT_POL_POS);
			mt_eint_set_sens(eint_num, MT_EDGE_SENSITIVE);
		} else if (flag & (EINTF_TRIGGER_HIGH | EINTF_TRIGGER_LOW)) {
			mt_eint_set_polarity(eint_num,
					     (flag & EINTF_TRIGGER_LOW) ?
					     MT_EINT_POL_NEG : MT_EINT_POL_POS);
			mt_eint_set_sens(eint_num, MT_LEVEL_SENSITIVE);
		} else {
			pr_err("[EINT]: Wrong EINT Pol/Sens Setting 0x%x\n",
			       flag);
			spin_unlock(&eint_lock);
			return;
		}

		EINT_FUNC.eint_func[eint_num] = EINT_FUNC_PTR;
		spin_unlock(&eint_lock);
		EINT_FUNC.eint_auto_umask[eint_num] = is_auto_umask;
		mt_eint_ack(eint_num);
		mt_eint_unmask(eint_num);
	} else {
		pr_err("[EINT]: Wrong EINT Number %d\n", eint_num);
	}
}
EXPORT_SYMBOL(mt_eint_registration);
#endif

#ifdef EINT_TEST_V2
static int mt_eint_is_debounce_en(unsigned int cur_eint_num)
{
	unsigned long base, val, en;
	if (cur_eint_num < MAX_HW_DEBOUNCE_CNT) {
		base = (cur_eint_num / 4) * 4 + EINT_DBNC_BASE;
		val = readl(IOMEM(base));
		val = val >> ((cur_eint_num % 4) * 8);
		if (val & EINT_DBNC_EN_BIT)
			en = 1;
		else
			en = 0;
	} else {
		en = EINT_FUNC.is_deb_en[cur_eint_num];
	}

	return en;
}

static void mt_eint_enable_debounce(unsigned int cur_eint_num)
{
	mt_eint_mask(cur_eint_num);
	if (cur_eint_num < MAX_HW_DEBOUNCE_CNT) {
		/* HW debounce */
		mt_eint_en_hw_debounce(cur_eint_num);
	} else {
		/* SW debounce */
		mt_eint_en_sw_debounce(cur_eint_num);
	}
	mt_eint_unmask(cur_eint_num);
}

static void mt_eint_disable_debounce(unsigned int cur_eint_num)
{
	mt_eint_mask(cur_eint_num);
	if (cur_eint_num < MAX_HW_DEBOUNCE_CNT) {
		/* HW debounce */
		mt_eint_dis_hw_debounce(cur_eint_num);
	} else {
		/* SW debounce */
		mt_eint_dis_sw_debounce(cur_eint_num);
	}
	mt_eint_unmask(cur_eint_num);
}
#endif /* end of EINT_TEST_V2 */

/*
 * mt_eint_setdomain0: set all eint_num to domain 0.
 */
static void mt_eint_setdomain0(void)
{
	unsigned long base;
	unsigned int val = 0xFFFFFFFF, ap_cnt = (EINT_MAX_CHANNEL / 32), i;
	if (EINT_MAX_CHANNEL % 32)
		ap_cnt++;
	dbgmsg("[EINT] cnt:%d\n", ap_cnt);

	base = EINT_D0_EN_BASE;
	for (i = 0; i < ap_cnt; i++) {
		mt_reg_sync_writel(val, base + (i * 4));
		dbgmsg("[EINT] domain addr:%lx = %x\n", base,
		       readl(IOMEM(base)));
	}
}

#ifdef MD_EINT
struct MD_SIM_HOTPLUG_INFO {
	char name[24];
	int eint_num;
	int eint_deb;
	int eint_pol;
	int eint_sens;
	int socket_type;
	int dedicatedEn;
	int srcPin;
};

#define MD_SIM_MAX 16
struct MD_SIM_HOTPLUG_INFO md_sim_info[MD_SIM_MAX];
unsigned int md_sim_counter = 0;

int get_eint_attribute(char *name, unsigned int name_len, unsigned int type,
		       char *result, unsigned int *len)
{
	int i;
	int ret = 0;
	int *sim_info = (int *)result;

	pr_notice("query info: name:%s, type:%d, len:%d\n", name, type, name_len);
	if (len == NULL || name == NULL || result == NULL)
		return ERR_SIM_HOT_PLUG_NULL_POINTER;

	for (i = 0; i < md_sim_counter; i++) {
		pr_notice("compare string:%s\n", md_sim_info[i].name);
		if (!strncmp(name, md_sim_info[i].name, name_len)) {
			switch (type) {
			case SIM_HOT_PLUG_EINT_NUMBER:
				*len = sizeof(md_sim_info[i].eint_num);
				memcpy(sim_info, &md_sim_info[i].eint_num,
				       *len);
				pr_notice("[EINT]eint_num:%d\n",
				       md_sim_info[i].eint_num);
				break;

			case SIM_HOT_PLUG_EINT_DEBOUNCETIME:
				*len = sizeof(md_sim_info[i].eint_deb);
				memcpy(sim_info, &md_sim_info[i].eint_deb,
				       *len);
				pr_notice("[EINT]eint_deb:%d\n",
				       md_sim_info[i].eint_deb);
				break;

			case SIM_HOT_PLUG_EINT_POLARITY:
				*len = sizeof(md_sim_info[i].eint_pol);
				memcpy(sim_info, &md_sim_info[i].eint_pol,
				       *len);
				pr_notice("[EINT]eint_pol:%d\n",
				       md_sim_info[i].eint_pol);
				break;

			case SIM_HOT_PLUG_EINT_SENSITIVITY:
				*len = sizeof(md_sim_info[i].eint_sens);
				memcpy(sim_info, &md_sim_info[i].eint_sens,
				       *len);
				pr_notice("[EINT]eint_sens:%d\n",
				       md_sim_info[i].eint_sens);
				break;

			case SIM_HOT_PLUG_EINT_SOCKETTYPE:
				*len = sizeof(md_sim_info[i].socket_type);
				memcpy(sim_info, &md_sim_info[i].socket_type,
				       *len);
				pr_notice("[EINT]socket_type:%d\n",
				       md_sim_info[i].socket_type);
				break;

			case SIM_HOT_PLUG_EINT_DEDICATEDEN:
				*len = sizeof(md_sim_info[i].dedicatedEn);
				memcpy(sim_info, &md_sim_info[i].dedicatedEn,
				       *len);
				pr_notice("[EINT]dedicatedEn:%d\n",
				       md_sim_info[i].dedicatedEn);
				break;

			case SIM_HOT_PLUG_EINT_SRCPIN:
				*len = sizeof(md_sim_info[i].srcPin);
				memcpy(sim_info, &md_sim_info[i].srcPin, *len);
				pr_notice("[EINT]srcPin:%d\n",
				       md_sim_info[i].srcPin);
				break;

			default:
				ret = ERR_SIM_HOT_PLUG_QUERY_TYPE;
				*len = sizeof(int);
				memset(sim_info, 0xff, *len);
				break;
			}
			return ret;
		}
	}

	*len = sizeof(int);
	memset(sim_info, 0xff, *len);

	return ERR_SIM_HOT_PLUG_QUERY_STRING;
}

int get_type(char *name)
{

	int type1 = 0x0;
	int type2 = 0x0;
#if defined(CONFIG_MTK_SIM1_SOCKET_TYPE) || defined(CONFIG_MTK_SIM2_SOCKET_TYPE)
	char *p;
#endif

#ifdef CONFIG_MTK_SIM1_SOCKET_TYPE
	p = (char *)CONFIG_MTK_SIM1_SOCKET_TYPE;
	type1 = simple_strtoul(p, &p, 10);
#endif
#ifdef CONFIG_MTK_SIM2_SOCKET_TYPE
	p = (char *)CONFIG_MTK_SIM2_SOCKET_TYPE;
	type2 = simple_strtoul(p, &p, 10);
#endif
	if (!strncmp
	    (name, "MD1_SIM1_HOT_PLUG_EINT", strlen("MD1_SIM1_HOT_PLUG_EINT")))
		return type1;
	else if (!strncmp
		 (name, "MD1_SIM1_HOT_PLUG_EINT",
		  strlen("MD1_SIM1_HOT_PLUG_EINT")))
		return type1;
	else if (!strncmp
		 (name, "MD2_SIM2_HOT_PLUG_EINT",
		  strlen("MD2_SIM2_HOT_PLUG_EINT")))
		return type2;
	else if (!strncmp
		 (name, "MD2_SIM2_HOT_PLUG_EINT",
		  strlen("MD2_SIM2_HOT_PLUG_EINT")))
		return type2;
	else
		return 0;
}
#endif

static void setup_MD_eint(void)
{
#ifdef MD_EINT

#if defined(CUST_EINT_MD1_0_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD1_0_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD1_0_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD1_0_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD1_0_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD1_0_DEBOUNCE_CN;
	md_sim_info[md_sim_counter].dedicatedEn = CUST_EINT_MD1_0_DEDICATED_EN;
	md_sim_info[md_sim_counter].srcPin = CUST_EINT_MD1_0_SRCPIN;
	pr_notice("[EINT] MD1 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD1 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD1_1_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD1_1_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD1_1_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD1_1_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD1_1_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD1_1_DEBOUNCE_CN;
	md_sim_info[md_sim_counter].dedicatedEn = CUST_EINT_MD1_1_DEDICATED_EN;
	md_sim_info[md_sim_counter].srcPin = CUST_EINT_MD1_1_SRCPIN;
	pr_notice("[EINT] MD1 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD1 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD1_2_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD1_2_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD1_2_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD1_2_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD1_2_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD1_2_DEBOUNCE_CN;
	md_sim_info[md_sim_counter].dedicatedEn = CUST_EINT_MD1_2_DEDICATED_EN;
	md_sim_info[md_sim_counter].srcPin = CUST_EINT_MD1_2_SRCPIN;
	pr_notice("[EINT] MD1 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD1 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD1_3_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD1_3_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD1_3_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD1_3_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD1_3_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD1_3_DEBOUNCE_CN;
	md_sim_info[md_sim_counter].dedicatedEn = CUST_EINT_MD1_3_DEDICATED_EN;
	md_sim_info[md_sim_counter].srcPin = CUST_EINT_MD1_3_SRCPIN;
	pr_notice("[EINT] MD1 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD1 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD1_4_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD1_4_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD1_4_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD1_4_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD1_4_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD1_4_DEBOUNCE_CN;
	pr_notice("[EINT] MD1 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD1 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif

#if defined(CUST_EINT_MD2_0_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD2_0_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD2_0_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD2_0_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD2_0_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD2_0_DEBOUNCE_CN;
	pr_notice("[EINT] MD2 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD2 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD2_1_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD2_1_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD2_1_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD2_1_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD2_1_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD2_1_DEBOUNCE_CN;
	pr_notice("[EINT] MD2 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD2 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD2_2_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD2_2_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD2_2_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD2_2_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD2_2_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD2_2_DEBOUNCE_CN;
	pr_notice("[EINT] MD2 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD2 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD2_3_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD2_3_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD2_3_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD2_3_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD2_3_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD2_3_DEBOUNCE_CN;
	pr_notice("[EINT] MD2 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD2 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#if defined(CUST_EINT_MD2_4_NAME)
	sprintf(md_sim_info[md_sim_counter].name, CUST_EINT_MD2_4_NAME);
	md_sim_info[md_sim_counter].eint_num = CUST_EINT_MD2_4_NUM;
	md_sim_info[md_sim_counter].eint_pol = CUST_EINT_MD2_4_POLARITY;
	md_sim_info[md_sim_counter].eint_sens = CUST_EINT_MD2_4_SENSITIVE;
	md_sim_info[md_sim_counter].socket_type =
	    get_type(md_sim_info[md_sim_counter].name);
	md_sim_info[md_sim_counter].eint_deb = CUST_EINT_MD2_4_DEBOUNCE_CN;
	pr_notice("[EINT] MD2 name = %s\n", md_sim_info[md_sim_counter].name);
	pr_notice("[EINT] MD2 type = %d\n",
	       md_sim_info[md_sim_counter].socket_type);
	md_sim_counter++;
#endif
#endif /* end of MD_EINT */
}

#ifdef EINT_TEST_V2
unsigned long long hw_debounce_start;
unsigned long long hw_debounce_end;
volatile int EINT_waiting;
static struct platform_driver eint_driver = {
	.driver = {
		   .name = "eint_test",
		   .bus = &platform_bus_type,
		   .owner = THIS_MODULE,
		   }
};

static void gpio_set_debounce(unsigned gpio, unsigned debounce);

static void mt_eint_soft_isr(void)
{
	dbgmsg("in mt_eint_soft_isr\n");
	mt_eint_soft_clr(cur_eint_num);
	return;
}

static irqreturn_t mt_eint_soft_revert_isr(unsigned irq, struct irq_desc *desc)
{
	int eint_num = 0;
	eint_num = DEMUX_EINT_IRQ(irq);
	pr_notice("======EINT_SOFT_REVERT_ISR======\n");
	pr_notice("EINT %d, in %s\n", eint_num, __func__);
	mt_eint_revert_polarity(eint_num);
	pr_notice("======EINT_SOFT_REVERT_ISR_END======\n");
	return IRQ_HANDLED;
}

volatile unsigned int sw_debounce = 0;

static irqreturn_t mt_eint_soft_debounce_isr(unsigned irq,
					     struct irq_desc *desc)
{
	int eint_num = 0;
	eint_num = DEMUX_EINT_IRQ(irq);
	pr_notice("======EINT_SOFT_DEBOUNCE_ISR======\n");
	pr_notice("EINT %d, in %s\n", eint_num, __func__);
	mt_eint_revert_polarity(eint_num);
	pr_notice("======EINT_SOFT_DEBOUNCE_ISR_END======\n");
	sw_debounce = 1;
	return IRQ_HANDLED;
}

static irqreturn_t mt_eint_soft_nonauto_unmask_isr(unsigned irq,
						   struct irq_desc *desc)
{
	int eint_num = 0;
	eint_num = DEMUX_EINT_IRQ(irq);
	pr_notice("======EINT_NONAUTO_SOFT_ISR======\n");
	pr_notice("EINT %d, in %s\n", eint_num, __func__);
	mt_eint_revert_polarity(eint_num);
	disable_irq_nosync(irq);
	pr_notice("======EINT_NONAUTO_SOFT_ISR_END======\n");
	return IRQ_HANDLED;
}

#define EINT_IN     0
#define EINT_OUT    1
/* these 2 arrays are for EINT HW debounce test
refer to designer / SA for detail */
int eint_debounce[] = { 30, 31 };
int gpio_debounce[] = { 58, 59 };

int eint_num = 3;

static irqreturn_t mt_eint_hw_debounce_isr(unsigned irq, struct irq_desc *desc)
{
	int eint_num;
	eint_num = DEMUX_EINT_IRQ(irq);
	hw_debounce_end = sched_clock();
	EINT_waiting = 0;
	pr_notice("======EINT_HW_DEBOUNCE_ISR======\n");
	pr_notice("EINT %d, in %s\n", eint_num, __func__);
	pr_notice("======EINT_HW_DEBOUNCE_ISR_END======\n");
	mt_set_gpio_out(gpio_debounce[EINT_OUT], 0);
	return IRQ_HANDLED;
}

typedef void (*EINT_FUNC_PTR) (void);

static void mt_eint_test(void)
{
	int eint_num = 1;
	int is_auto_umask = 0;
	int ret = 0;
	cur_eint_num = 1;
	mt_eint_mask(eint_num);
	mt_eint_set_polarity(eint_num, MT_LEVEL_SENSITIVE);
	mt_eint_set_sens(eint_num, MT_EINT_POL_POS);
	mt_eint_registration(eint_num, IRQF_TRIGGER_HIGH, mt_eint_soft_isr,
			     is_auto_umask);
	if (ret > 0)
		pr_err("EINT IRQ LINE NOT AVAILABLE!!\n");

	mt_eint_unmask(eint_num);
	mt_eint_soft_set(eint_num);
}

void setup_level_trigger_env(unsigned int eint)
{
	unsigned long base, pol_base, raw_base;
	if (eint >= EINT_MAX_CHANNEL) {
		pr_err("%s: eint %d >= %d\n", __func__, eint, EINT_MAX_CHANNEL);
		return;
	}

	base = eint / 32;
	raw_base = EINT_RAW_STA_BASE + base * 4;
	pol_base = EINT_POL_SET_BASE + base * 4;

	mt_eint_mask(eint);
	mt_eint_set_sens(eint, MT_LEVEL_SENSITIVE);
	mt_eint_set_polarity(eint, MT_EINT_POL_NEG);
	writel(readl(IOMEM(raw_base)) & 0xFFFFFFFF, IOMEM(pol_base));
}

static void mt_eint_autounmask_test(void)
{
	int ret = 0;
	struct irq_desc *desc;
	pr_notice("%s for EINT %d\n", __func__, eint_num);
	setup_level_trigger_env(eint_num);
	ret =
	    request_irq(EINT_IRQ(eint_num),
			(irq_handler_t) mt_eint_soft_revert_isr,
			mt_eint_get_polarity(eint_num), "EINT-AUTOUNMASK",
			NULL);
	if (ret > 0) {
		pr_err("EINT IRQ LINE NOT AVAILABLE!!\n");
		return;
	}
	desc = irq_to_desc(EINT_IRQ(eint_num));
	pr_notice("EINT %d request_irq done\n", eint_num);
	mt_eint_revert_polarity(eint_num);
	pr_notice("trigger EINT %d done\n", eint_num);
	pr_notice("EINT %d, MASK 0x%d\n", eint_num, mt_eint_get_mask(eint_num));
	free_irq(EINT_IRQ(eint_num), NULL);
}

static irqreturn_t mt_deint_soft_revert_isr(unsigned irq, struct irq_desc *desc)
{
	pr_notice("======DEINT_SOFT_REVERT_ISR======\n");
	pr_notice("irq = %d\n", irq);
	irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	pr_notice("======DEINT_SOFT_REVERT_ISR_END======\n");
	return IRQ_HANDLED;
}

static void mt_deint_test(void)
{
	int ret = 0;

	pr_notice("%s for EINT %d\n", __func__, eint_num);

	ret = mt_eint_set_deint(eint_num, 189);
	if (ret == 0) {
		pr_notice("mt_eint_set_deint done\n");
	} else {
		pr_err("mt_eint_set_deint fail\n");
		return;
	}

	ret =
	    request_irq(189, (irq_handler_t) mt_deint_soft_revert_isr,
			IRQF_TRIGGER_HIGH, "EINT-AUTOUNMASK", NULL);
	if (ret > 0) {
		pr_err("EINT IRQ LINE NOT AVAILABLE!!\n");
		return;
	}
	pr_notice("EINT %d request_irq done\n", eint_num);
	irq_set_irq_type(189, IRQF_TRIGGER_LOW);

	pr_notice("trigger EINT %d done\n", eint_num);
	pr_notice("EINT %d, MASK 0x%d\n", eint_num, mt_eint_get_mask(eint_num));

	free_irq(189, NULL);
	mt_eint_clr_deint(eint_num);
}

static int mt_eint_non_autounmask_test(void)
{
	int ret = 0;

	setup_level_trigger_env(eint_num);
	EINT_FUNC.eint_func[eint_num] = NULL;
	pr_notice("%s for EINT %d\n", __func__, eint_num);
	ret =
	    request_irq(EINT_IRQ(eint_num),
			(irq_handler_t) mt_eint_soft_nonauto_unmask_isr,
			mt_eint_get_polarity(eint_num), "EINT-NONAUTOUNMASK",
			NULL);
	if (ret > 0) {
		pr_err("EINT IRQ LINE NOT AVAILABLE!!\n");
		return -1;
	}
	pr_notice("EINT %d request_irq done\n", eint_num);
	mt_eint_revert_polarity(eint_num);
	pr_notice("trigger EINT %d done\n", eint_num);
	pr_notice("EINT %d, MASK 0x%d\n", eint_num, mt_eint_get_mask(eint_num));
	free_irq(EINT_IRQ(eint_num), NULL);
	return 1;
}

void mt_eint_normal_test_based_on_sw_debounce(void)
{
	int eint_num = 36;
	int ret = 0;
	unsigned int debounce_time = 30;

	setup_level_trigger_env(eint_num);
	pr_notice("%s for EINT %d\n", __func__, eint_num);
	gpio_set_debounce(EINT_GPIO(eint_num), debounce_time);
	pr_notice("EINT %d debounce enable %d\n", eint_num,
	       mt_eint_is_debounce_en(eint_num));
	ret =
	    request_irq(EINT_IRQ(eint_num),
			(irq_handler_t) mt_eint_soft_debounce_isr,
			mt_eint_get_polarity(eint_num), "EINT-SWDEBOUNCE",
			NULL);
	if (ret > 0)
		pr_err("EINT IRQ LINE NOT AVAILABLE!!\n");
	pr_notice("EINT %d request_irq done\n", eint_num);
	mt_eint_revert_polarity(eint_num);
	pr_notice("trigger EINT %d done\n", eint_num);
	pr_notice("start waiting sw_debounce\n");
	while (!sw_debounce)
		;
	sw_debounce = 0;
	pr_notice("EINT %d, MASK 0x%d\n", eint_num, mt_eint_get_mask(eint_num));
	free_irq(EINT_IRQ(eint_num), NULL);
}

void mt_eint_normal_test_based_on_hw_debounce(void)
{
	int ret = 0;
	unsigned int debounce_time = 16;
	unsigned long long delay_start;

	pr_notice("%s for EINT %d\n", __func__, eint_debounce[EINT_IN]);
	gpio_set_debounce(EINT_GPIO(eint_debounce[EINT_IN]), debounce_time);

	/* GPIO setting */
	mt_set_gpio_mode(gpio_debounce[EINT_OUT], GPIO_MODE_00);
	mt_set_gpio_dir(gpio_debounce[EINT_OUT], GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(gpio_debounce[EINT_OUT], 1);
	mt_set_gpio_out(gpio_debounce[EINT_OUT], 0);
	mt_set_gpio_mode(gpio_debounce[EINT_IN], GPIO_MODE_04);
	mt_set_gpio_dir(gpio_debounce[EINT_IN], GPIO_DIR_IN);

	pr_notice("before request_irq for EINT %d\n", eint_debounce[EINT_IN]);
	EINT_waiting = 1;
	ret =
	    request_irq(EINT_IRQ(eint_debounce[EINT_IN]),
			(irq_handler_t) mt_eint_hw_debounce_isr,
			IRQF_TRIGGER_HIGH, "EINT-HWDEBOUNCE", NULL);
	if (ret > 0)
		pr_err("EINT IRQ LINE NOT AVAILABLE!!\n");
	pr_notice("EINT %d request_irq done\n", eint_debounce[EINT_IN]);
	mt_set_gpio_out(gpio_debounce[EINT_OUT], 1);
	pr_notice("trigger EINT %d done\n", eint_debounce[EINT_IN]);
	hw_debounce_start = delay_start = sched_clock();
	pr_notice("waiting for EINT %d\n", eint_debounce[EINT_IN]);

	while (EINT_waiting) {
		delay_start = sched_clock();
		if (!EINT_waiting)
			break;
	}
	pr_notice("hw_debounce time t1=%llu ,t2=%llu, %llu ns\n",
	       hw_debounce_start, hw_debounce_end,
	       (hw_debounce_end - hw_debounce_start));
	pr_notice("duration time t1=%llu ,t2=%llu, %llu ns\n", hw_debounce_start,
	       delay_start, (delay_start - hw_debounce_start));

	/* divide by 1m => ns to ms */
	if (!EINT_waiting
	    && (hw_debounce_start - hw_debounce_end) / 1000000 >
	    debounce_time) {
		pr_notice("...pass\n");
	} else {
		pr_notice("...failed\n");
	}
	pr_notice("EINT %d, MASK 0x%d\n", eint_debounce[EINT_IN],
	       mt_eint_get_mask(eint_debounce[EINT_IN]));
	free_irq(EINT_IRQ(eint_debounce[EINT_IN]), NULL);
}

static ssize_t test_show(struct device_driver *driver, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "==EINT test==\n"
			"1.EINT original registration test\n"
			"2.EINT new design test\n"
			"3.EINT new design hw debounce test\n"
			"4.EINT new design sw debounce test\n"
			"5.EINT new design without auto-unmask\n"
			"6.EINT new design deint test\n");
}

static ssize_t test_store(struct device_driver *driver, const char *buf,
			  size_t count)
{
	char *p = (char *)buf;
	unsigned long num;

	if (kstrtoul(p, 10, &num) != 0) {
		pr_err("[EIC] can not kstrtoul for %s\n", p);
		return -1;
	}

	switch (num) {
	case 1:
		mt_eint_test();
		break;
	case 2:
		mt_eint_autounmask_test();
		break;
	case 3:
		mt_eint_normal_test_based_on_hw_debounce();
		break;
	case 4:
		mt_eint_normal_test_based_on_sw_debounce();
		break;
	case 5:
		mt_eint_non_autounmask_test();
		break;
	case 6:
		mt_deint_test();
		break;
	default:
		break;
	}

	return count;
}

DRIVER_ATTR(eint_test_suit, 0644, test_show, test_store);

#endif

#ifdef EINT_TEST_V2
static void gpio_set_debounce(unsigned gpio, unsigned debounce)
{
	mt_eint_set_hw_debounce(gpio, debounce);
}
#endif

int get_supported_irq_num(void)
{
	struct device_node *node;
	void __iomem *dist_base;
	int ret = 0;

	/* usually, common eic goes with common gic.
	   However, common eic can also go with not common gic as long as we provide an alias-gic node. */
	node = of_find_compatible_node(NULL, NULL, "mtk,mt-gic");
	if (!node) {
		node = of_find_compatible_node(NULL, NULL, "mtk,alias-gic");
		if (!node) {
			pr_err("%s can't find node for gic\n", __func__);
			return ret;
		}
	}

	dist_base = of_iomap(node, 0);
#define GIC_DIST_CTR 0x4
	ret = readl_relaxed(dist_base + GIC_DIST_CTR) & 0x1f;
	ret = (ret + 1) * 32;
	pr_notice("gic supported max = %d\n", ret);
	return ret;
}

int mt_gpio_set_debounce(unsigned gpio, unsigned debounce)
{
	if (gpio >= EINT_MAX_CHANNEL)
		return -EINVAL;

	debounce /= 1000;
	mt_eint_set_hw_debounce(gpio, debounce);

	return 0;
}
EXPORT_SYMBOL(mt_gpio_set_debounce);

#define GPIO_MAX 999
static struct rb_root root = RB_ROOT;
struct pin_node *pins;

static struct pin_node *pin_search(u32 gpio)
{
	struct rb_node *node = root.rb_node;
	struct pin_node *pin = NULL;

	while (node) {
		pin = rb_entry(node, struct pin_node, node);
		if (gpio < pin->gpio_pin)
			node = node->rb_left;
		else if (gpio > pin->gpio_pin)
			node = node->rb_right;
		else
			return pin;
	}

	return NULL;
}

static int pin_insert(struct pin_node *pin)
{
	struct rb_node **new = &(root.rb_node), *parent = NULL;
	struct pin_node *node;

	while (*new) {
		parent = *new;
		node = rb_entry(parent, struct pin_node, node);

		if (pin->gpio_pin < node->gpio_pin)
			new = &(*new)->rb_left;
		else if (pin->gpio_pin > node->gpio_pin)
			new = &(*new)->rb_right;
		else
			return 0;
	}

	rb_link_node(&pin->node, parent, new);
	rb_insert_color(&pin->node, &root);
	return 1;
}

static void pin_init(void)
{
	u32 i;
	for (i = 0; i < mapping_table_entry; i++) {
		if (pins[i].gpio_pin == GPIO_MAX)
			break;
		if (!pin_insert(&pins[i])) {
			pr_warn("duplicate record? i = %d, gpio = %d, eint = %d\n",
			     i, pins[i].gpio_pin, pins[i].eint_pin);
		}
	}
}

#ifdef EINT_TEST_V2
static void pin_traverse(void)
{
	struct pin_node *p;
	struct rb_node *r;

	for (r = rb_first(&root); r; r = rb_next(r)) {
		p = rb_entry(r, struct pin_node, node);
		pr_notice("gpio = %d, eint = %d\n", p->gpio_pin, p->eint_pin);
	}
}
#endif

unsigned int mt_gpio_to_irq(unsigned int gpio)
{
	struct pin_node *p;

	if (mapping_table_entry > 0) {
		p = pin_search(gpio);
		if (p == NULL)
			return -EINVAL;
		else
			return p->eint_pin + EINT_IRQ_BASE;
	} else {
		return gpio + EINT_IRQ_BASE;
	}
}
EXPORT_SYMBOL(mt_gpio_to_irq);

static void mt_eint_irq_mask(struct irq_data *data)
{
	mt_eint_mask(data->hwirq);
}

static void mt_eint_irq_unmask(struct irq_data *data)
{
	mt_eint_unmask(data->hwirq);
}

static void mt_eint_irq_ack(struct irq_data *data)
{
	mt_eint_ack(data->hwirq);
}

static int mt_eint_irq_set_type(struct irq_data *data, unsigned int type)
{
	int eint_num = data->hwirq;

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_EDGE_FALLING))
		mt_eint_set_polarity(eint_num, MT_EINT_POL_NEG);
	else
		mt_eint_set_polarity(eint_num, MT_EINT_POL_POS);
	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING))
		mt_eint_set_sens(eint_num, MT_EDGE_SENSITIVE);
	else
		mt_eint_set_sens(eint_num, MT_LEVEL_SENSITIVE);

	return IRQ_SET_MASK_OK;
}

static struct irq_chip mt_irq_eint = {
	.name = "mt-eint",
	.irq_mask = mt_eint_irq_mask,
	.irq_unmask = mt_eint_irq_unmask,
	.irq_ack = mt_eint_irq_ack,
	.irq_set_type = mt_eint_irq_set_type,
};

int mt_eint_domain_xlate_onetwocell(struct irq_domain *d,
				    struct device_node *ctrlr,
				    const u32 *intspec, unsigned int intsize,
				    unsigned long *out_hwirq,
				    unsigned int *out_type)
{
	if (WARN_ON(intsize < 1))
		return -EINVAL;
	*out_hwirq = mt_gpio_to_irq(intspec[0]) - EINT_IRQ_BASE;
	*out_type = (intsize > 1) ? intspec[1] : IRQ_TYPE_NONE;
	return 0;
}

const struct irq_domain_ops mt_eint_domain_simple_ops = {
	.xlate = mt_eint_domain_xlate_onetwocell,
};

static int __init mt_eint_init(void)
{
	unsigned int i, irq;
	int irq_base;
	struct irq_domain *domain;
	struct device_node *node;
#if defined(EINT_TEST_V2)
	struct pin_node *p;
#endif
	const __be32 *spec;
	u32 len;

#if defined(EINT_TEST_V2)
	int ret;
#endif

	/* DTS version */
	node = of_find_compatible_node(NULL, NULL, "mtk,mt-eic");
	if (node) {
		EINT_BASE = of_iomap(node, 0);
		pr_notice("get EINT_BASE @ %p\n", EINT_BASE);
	} else {
		pr_err("can't find compatible node\n");
		return -1;
	}

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0) {
		pr_err("irq # for eint %d\n", irq);
		return -1;
	}

	if (of_property_read_u32(node, "max_eint_num", &EINT_MAX_CHANNEL)) {
		return -1;
	} else {
		pr_debug("[EIC] max_eint_num = %d\n", EINT_MAX_CHANNEL);

		EINT_FUNC.eint_func = kmalloc(sizeof(eint_funct)*EINT_MAX_CHANNEL, GFP_KERNEL);
		EINT_FUNC.eint_auto_umask = kmalloc(sizeof(unsigned int)*EINT_MAX_CHANNEL, GFP_KERNEL);
		EINT_FUNC.is_deb_en = kmalloc(sizeof(unsigned int)*EINT_MAX_CHANNEL, GFP_KERNEL);
		EINT_FUNC.deb_time = kmalloc(sizeof(unsigned int)*EINT_MAX_CHANNEL, GFP_KERNEL);
		EINT_FUNC.eint_sw_deb_timer = kmalloc(sizeof(struct timer_list)*EINT_MAX_CHANNEL, GFP_KERNEL);
		EINT_FUNC.count = kmalloc(sizeof(unsigned int)*EINT_MAX_CHANNEL, GFP_KERNEL);

		if (of_property_read_u32(node, "mapping_table_entry", &mapping_table_entry)) {
			return -1;
		} else {
			pr_debug("[EIC] mapping_table_entry = %d\n",
				 mapping_table_entry);

			if (mapping_table_entry > 0) {
				spec =
				    of_get_property(node, "mapping_table",
						    &len);
				if (spec == NULL)
					return -EINVAL;
				len /= sizeof(*spec);

				pr_debug
				    ("[EIC] mapping_table: spec=%d len=%d\n",
				     be32_to_cpup(spec), len);

				pins =
				    (struct pin_node *)
				    kmalloc(sizeof(struct pin_node) *
					    (mapping_table_entry + 1),
					    GFP_KERNEL);
				for (i = 0; i < mapping_table_entry; i++) {
					pr_debug
					    ("[EIC] index=%d: gpio_pin=%d, eint_pin=%d\n",
					     i, be32_to_cpup(spec + (i << 1)),
					     be32_to_cpup(spec + (i << 1) + 1));
					pins[i].gpio_pin =
					    be32_to_cpup(spec + (i << 1));
					pins[i].eint_pin =
					    be32_to_cpup(spec + (i << 1) + 1);
				}
				pins[i].gpio_pin = GPIO_MAX;
			}
		}
	}

	if (of_property_read_u32(node, "max_deint_cnt", &MAX_DEINT_CNT)) {
		pr_warn("[EIC] no max_deint_cnt specified\n");
	} else {
		deint_possible_irq =
		    kzalloc(sizeof(u32) * MAX_DEINT_CNT, GFP_KERNEL);
		if (!deint_possible_irq) {
			pr_warn("[EIC] malloc fail for deint_possible_irq\n");
			return -1;
		}

		deint_descriptors =
		    kzalloc(sizeof(struct deint_des) * MAX_DEINT_CNT,
			    GFP_KERNEL);
		if (!deint_descriptors) {
			pr_warn("[EIC] malloc fail for deint_descriptors\n");
			return -1;
		}

		if (of_property_read_u32_array(node, "deint_possible_irq", deint_possible_irq, MAX_DEINT_CNT))
			pr_warn("[EINT] deint function would fail...since there is no <deint_possible_irq> in DTS\n");
	}

	/* assign to domain 0 for AP */
	mt_eint_setdomain0();

	wake_lock_init(&EINT_suspend_lock, WAKE_LOCK_SUSPEND, "EINT wakelock");

	setup_MD_eint();
	for (i = 0; i < EINT_MAX_CHANNEL; i++) {
		EINT_FUNC.eint_func[i] = NULL;
		EINT_FUNC.is_deb_en[i] = 0;
		EINT_FUNC.deb_time[i] = 0;
		EINT_FUNC.eint_sw_deb_timer[i].expires = 0;
		EINT_FUNC.eint_sw_deb_timer[i].data = 0;
		EINT_FUNC.eint_sw_deb_timer[i].function = NULL;

		init_timer(&EINT_FUNC.eint_sw_deb_timer[i]);
	}

	/* gpio to eint structure init */
	if (mapping_table_entry > 0) {
		pin_init();

#if defined(EINT_TEST_V2)
		pin_traverse();
		p = pin_search(31);
		if (p)
			pr_notice("gpio 31 = eint %d\n", p->eint_pin);

		p = pin_search(37);
		if (p)
			pr_notice("gpio 37 = eint %d\n", p->eint_pin);

		p = pin_search(GPIO_MAX);
		if (p)
			pr_notice("gpio GPIO_MAX = eint %d\n", p->eint_pin);
		else
			pr_notice("can't find for GPIO %d\n", GPIO_MAX);
#endif
	}

	/* Register Linux IRQ interface */
	EINT_IRQ_BASE = get_supported_irq_num();
	if (!EINT_IRQ_BASE) {
		pr_err("get_supported_irq_num returns %d\n", EINT_IRQ_BASE);
		return -1;
	}
	pr_notice("EINT_IRQ_BASE = %d\n", EINT_IRQ_BASE);
	irq_base =
	    irq_alloc_descs(EINT_IRQ_BASE, EINT_IRQ_BASE, EINT_MAX_CHANNEL,
			    numa_node_id());
	if (irq_base != EINT_IRQ_BASE) {
		pr_err("EINT alloc desc error %d\n", irq_base);
		return -1;
	}

	for (i = 0; i < EINT_MAX_CHANNEL; i++) {
		irq_set_chip_and_handler(i + EINT_IRQ_BASE, &mt_irq_eint,
					 handle_level_irq);
		set_irq_flags(i + EINT_IRQ_BASE, IRQF_VALID);
	}

	domain = irq_domain_add_legacy(node, EINT_MAX_CHANNEL, EINT_IRQ_BASE, 0,
				       &mt_eint_domain_simple_ops, NULL);
	if (!domain)
		pr_err("EINT domain add error\n");

	irq_set_chained_handler(irq, (irq_flow_handler_t) mt_eint_demux);

#if defined(EINT_TEST_V2)
	ret = platform_driver_register(&eint_driver);
	if (ret)
		pr_err("Fail to register eint_driver");

	ret |=
	    driver_create_file(&eint_driver.driver,
			       &driver_attr_eint_test_suit);
	if (ret)
		pr_err("Fail to create eint_driver sysfs files");

	node = of_find_compatible_node(NULL, NULL, "mediatek, ACCDET-eint");
	if (node) {
		i = irq_of_parse_and_map(node, 0);
		pr_notice("virq for ACCDET = %d\n", i);
	} else {
		pr_err("can't find compatible node for ACCDET\n");
	}

	node =
	    of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if (node) {
		i = irq_of_parse_and_map(node, 0);
		pr_notice("virq for TOUCH = %d\n", i);
	} else {
		pr_err("can't find compatible node for TOUCH\n");
	}
#endif

	return 0;
}

#if (EINT_DEBUG == 1)
static unsigned int mt_eint_get_debounce_cnt(unsigned int cur_eint_num)
{
	unsigned int dbnc, deb;
	unsigned long base;
	base = (cur_eint_num / 4) * 4 + EINT_DBNC_BASE;

	if (cur_eint_num >= EINT_MAX_CHANNEL)
		return 0;

	if (cur_eint_num >= MAX_HW_DEBOUNCE_CNT)
		deb = EINT_FUNC.deb_time[cur_eint_num];
	else {
		dbnc = readl(IOMEM(base));
		dbnc =
		    ((dbnc >> EINT_DBNC_SET_DBNC_BITS) >>
		     ((cur_eint_num % 4) * 8) & EINT_DBNC);

		switch (dbnc) {
		case 0:
			deb = 0;	/* 0.5 actually, but we don't allow user to set. */
			dbgmsg(KERN_CRIT
			       "ms should not be 0. eint_num:%d in %s\n",
			       cur_eint_num, __func__);
			break;
		case 1:
			deb = 1;
			break;
		case 2:
			deb = 16;
			break;
		case 3:
			deb = 32;
			break;
		case 4:
			deb = 64;
			break;
		case 5:
			deb = 128;
			break;
		case 6:
			deb = 256;
			break;
		case 7:
			deb = 512;
			break;
		default:
			deb = 0;
			pr_err("invalid deb time in the EIN_CON register, dbnc:%d, deb:%d\n", dbnc, deb);
			break;
		}
	}

	return deb;
}

void mt_eint_dump_status(unsigned int eint)
{
	if (eint >= EINT_MAX_CHANNEL)
		return;
	pr_notice("[EINT] eint:%d,mask:%x,pol:%x,deb:%x,sens:%x\n", eint,
	       mt_eint_get_mask(eint), mt_eint_get_polarity(eint),
	       mt_eint_get_debounce_cnt(eint), mt_eint_get_sens(eint));
}
#endif

/*
 * mt_eint_print_status: Print the EINT status register.
 */
void mt_eint_print_status(void)
{
	unsigned int status, index;
	unsigned int offset, reg_base, status_check;
	pr_notice("EINT_STA:");
	for (reg_base = 0; reg_base < EINT_MAX_CHANNEL; reg_base += 32) {
		/* read status register every 32 interrupts */
		status = mt_eint_get_status(reg_base);
		if (status)
			pr_notice("EINT Module - index:%d,EINT_STA = 0x%x\n", reg_base, status);
		else
			continue;

		for (offset = 0; offset < 32; offset++) {
			index = reg_base + offset;
			if (index >= EINT_MAX_CHANNEL)
				break;

			status_check = status & (1 << offset);
			if (status_check) {
				pr_notice("EINT %d is pending\n", index);
#if (EINT_DEBUG == 1)
				mt_eint_dump_status(index);
#endif
			}
		}
	}
	pr_notice("\n");
}
EXPORT_SYMBOL(mt_eint_print_status);

arch_initcall(mt_eint_init);
