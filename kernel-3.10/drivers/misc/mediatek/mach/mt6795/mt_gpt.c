#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/proc_fs.h>
#include <linux/syscore_ops.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>
#include <asm/errno.h>
#include <asm/uaccess.h>
#include <linux/version.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_gpt.h>
#include <mach/mt_timer.h>
#include <mach/irqs.h>
#include <mach/mt_cpuxgpt.h>
/* #include <mach/mt_boot.h> */

/* #define CONFIG_CLKSRC_64_BIT */


#ifdef CONFIG_MTK_FPGA
#define SYS_CLK_RATE        (6000000)	/* FPGA clock source is 6M */
#else
#define SYS_CLK_RATE        (13000000)
#endif

/* MT6592 GPT Usage Allocation */

/* #define CONFIG_HAVE_SYSCNT */


#define GPT_CLKEVT_ID		(GPT1)

#ifdef CONFIG_CLKSRC_64_BIT
#define GPT_CLKSRC_ID       (GPT6)
#else
#define GPT_CLKSRC_ID       (GPT2)
#endif

#define GPT4_1MS_TICK       ((U32)13000)	/* 1000000 / 76.92ns = 13000.520 */

#define GPT_IRQEN           (APXGPT_BASE + 0x0000)
#define GPT_IRQSTA          (APXGPT_BASE + 0x0004)
#define GPT_IRQACK          (APXGPT_BASE + 0x0008)
#define GPT1_BASE           (APXGPT_BASE + 0x0010)
#define GPT4_BASE           (APXGPT_BASE + 0x0040)

#define GPT_CON             (0x00)
#define GPT_CLK             (0x04)
#define GPT_CNT             (0x08)
#define GPT_CMP             (0x0C)
#define GPT_CNTH            (0x18)
#define GPT_CMPH            (0x1C)

#define GPT_CON_ENABLE      (0x1 << 0)
#define GPT_CON_CLRCNT      (0x1 << 1)
#define GPT_CON_OPMODE      (0x3 << 4)

#define GPT_CLK_CLKDIV      (0xf << 0)
#define GPT_CLK_CLKSRC      (0x1 << 4)

#define GPT_OPMODE_MASK     (0x3)
#define GPT_CLKDIV_MASK     (0xf)
#define GPT_CLKSRC_MASK     (0x1)

#define GPT_OPMODE_OFFSET   (4)
#define GPT_CLKSRC_OFFSET   (4)


#define GPT_ISR             (0x0010)
#define GPT_IN_USE          (0x0100)

#define GPT_FEAT_64_BIT     (0x0001)
/************define this for 32/64 compatible**************/
#define GPT_BIT_MASK 0x00000000FFFFFFFF
/****************************************************/

int g_debug_flag = 0;


struct gpt_device {
	unsigned int id;
	unsigned int mode;
	unsigned int clksrc;
	unsigned int clkdiv;
	unsigned int cmp[2];
	void (*func) (unsigned long);
	int flags;
	int features;
	unsigned int base_addr;
};
static struct gpt_device gpt_devs[NR_GPTS];

/************************return GPT4 count(before init clear) to record kernel start time between LK and kernel****************************/
static unsigned int boot_time_value;

static unsigned int xgpt_boot_up_time(void)
{
	unsigned int tick;
	tick = DRV_Reg32(GPT4_BASE + GPT_CNT);
	return ((tick + (GPT4_1MS_TICK - 1)) / GPT4_1MS_TICK);
}

/**************************************************************************************************************************/
static struct gpt_device *id_to_dev(unsigned int id)
{
	if ((id == GPT3) || (id == GPT6))	/* MD32 used GPT3 and GPT6 */
		return NULL;
	return id < NR_GPTS ? gpt_devs + id : NULL;
}


static DEFINE_SPINLOCK(gpt_lock);

#define gpt_update_lock(flags)  \
do {    \
    spin_lock_irqsave(&gpt_lock, flags);    \
} while (0)

#define gpt_update_unlock(flags)  \
do {    \
    spin_unlock_irqrestore(&gpt_lock, flags);   \
} while (0)


static inline void noop(unsigned long data)
{
}

static void (*handlers[]) (unsigned long) = {
noop, noop, noop, noop, noop, noop, noop,};


static struct tasklet_struct task[NR_GPTS];
static void task_sched(unsigned long data)
{
	unsigned int id = (unsigned int)data;
	tasklet_schedule(&task[id]);
}

static void __gpt_set_handler(struct gpt_device *dev, void (*func) (unsigned long))
{
	if (func) {
		if (dev->flags & GPT_ISR)
			handlers[dev->id] = func;
		else {
			tasklet_init(&task[dev->id], func, 0);
			handlers[dev->id] = task_sched;
		}
	}
	dev->func = func;
}

static inline unsigned int gpt_get_and_ack_irq(void)
{
	unsigned int id;
	unsigned int mask;
	unsigned int status = DRV_Reg32(GPT_IRQSTA);

	for (id = GPT1; id < NR_GPTS; id++) {
		mask = 0x1 << id;
		if (status & mask) {
			DRV_WriteReg32(GPT_IRQACK, mask);
			break;
		}
	}

	return id;
}

static irqreturn_t gpt_handler(int irq, void *dev_id)
{
	unsigned int id = gpt_get_and_ack_irq();
	struct gpt_device *dev = id_to_dev(id);

	if (likely(dev)) {
		if (!(dev->flags & GPT_ISR)) {
			handlers[id] (id);
		} else {
			handlers[id] ((unsigned long)dev_id);
		}
	} else {
		printk(KERN_WARNING "GPT id is %d\n", id);
	}

	return IRQ_HANDLED;
}


static void __gpt_enable_irq(struct gpt_device *dev)
{
	DRV_SetReg32(GPT_IRQEN, 0x1 << (dev->id));
}

static void __gpt_disable_irq(struct gpt_device *dev)
{
	DRV_ClrReg32(GPT_IRQEN, 0x1 << (dev->id));
}

static void __gpt_ack_irq(struct gpt_device *dev)
{
	DRV_WriteReg32(GPT_IRQACK, 0x1 << (dev->id));
}

static void __gpt_reset(struct gpt_device *dev)
{
	DRV_WriteReg32(dev->base_addr + GPT_CON, 0x0);
	__gpt_disable_irq(dev);
	__gpt_ack_irq(dev);
	DRV_WriteReg32(dev->base_addr + GPT_CLK, 0x0);
	DRV_WriteReg32(dev->base_addr + GPT_CON, 0x2);
	DRV_WriteReg32(dev->base_addr + GPT_CMP, 0x0);
	if (dev->features & GPT_FEAT_64_BIT) {
		DRV_WriteReg32(dev->base_addr + GPT_CMPH, 0);
	}
}

static void __gpt_clrcnt(struct gpt_device *dev)
{
	DRV_SetReg32(dev->base_addr + GPT_CON, GPT_CON_CLRCNT);
	while (DRV_Reg32(dev->base_addr + GPT_CNT)) {
	}
}

static void __gpt_start(struct gpt_device *dev)
{
	DRV_SetReg32(dev->base_addr + GPT_CON, GPT_CON_ENABLE);
}

static void __gpt_start_from_zero(struct gpt_device *dev)
{
	/* DRV_SetReg32(dev->base_addr + GPT_CON, GPT_CON_ENABLE | GPT_CON_CLRCNT); */
	__gpt_clrcnt(dev);
	__gpt_start(dev);
}

/* gpt is counting or not */
static int __gpt_get_status(struct gpt_device *dev)
{
	return !!(DRV_Reg32(dev->base_addr + GPT_CON) & GPT_CON_ENABLE);
}

static void __gpt_stop(struct gpt_device *dev)
{
	DRV_ClrReg32(dev->base_addr + GPT_CON, GPT_CON_ENABLE);
}

static void __gpt_set_mode(struct gpt_device *dev, unsigned int mode)
{
	unsigned int ctl = DRV_Reg32(dev->base_addr + GPT_CON);
	mode <<= GPT_OPMODE_OFFSET;

	ctl &= ~GPT_CON_OPMODE;
	ctl |= mode;

	DRV_WriteReg32(dev->base_addr + GPT_CON, ctl);

	dev->mode = mode;
}

static void __gpt_set_clk(struct gpt_device *dev, unsigned int clksrc, unsigned int clkdiv)
{
	unsigned int clk = (clksrc << GPT_CLKSRC_OFFSET) | clkdiv;
	DRV_WriteReg32(dev->base_addr + GPT_CLK, clk);

	dev->clksrc = clksrc;
	dev->clkdiv = clkdiv;
}

static void __gpt_set_cmp(struct gpt_device *dev, unsigned int cmpl, unsigned int cmph)
{
	DRV_WriteReg32(dev->base_addr + GPT_CMP, cmpl);
	dev->cmp[0] = cmpl;

	if (dev->features & GPT_FEAT_64_BIT) {
		DRV_WriteReg32(dev->base_addr + GPT_CMPH, cmph);
		dev->cmp[1] = cmpl;
	}
}

static void __gpt_get_cmp(struct gpt_device *dev, unsigned int *ptr)
{
	*ptr = DRV_Reg32(dev->base_addr + GPT_CMP);
	if (dev->features & GPT_FEAT_64_BIT) {
		*(++ptr) = DRV_Reg32(dev->base_addr + GPT_CMPH);
	}
}

static void __gpt_get_cnt(struct gpt_device *dev, unsigned int *ptr)
{
	*ptr = DRV_Reg32(dev->base_addr + GPT_CNT);
	if (dev->features & GPT_FEAT_64_BIT) {
		*(++ptr) = DRV_Reg32(dev->base_addr + GPT_CNTH);
	}
}

static void __gpt_set_flags(struct gpt_device *dev, unsigned int flags)
{
	dev->flags |= flags;
}

static void gpt_devs_init(void)
{
	int i;

	for (i = 0; i < NR_GPTS; i++) {
		gpt_devs[i].id = i;
		gpt_devs[i].base_addr = GPT1_BASE + 0x10 * i;
	}

	gpt_devs[GPT6].features |= GPT_FEAT_64_BIT;
}

static void setup_gpt_dev_locked(struct gpt_device *dev, unsigned int mode,
				 unsigned int clksrc, unsigned int clkdiv, unsigned int cmp,
				 void (*func) (unsigned long), unsigned int flags)
{
	__gpt_set_flags(dev, flags | GPT_IN_USE);

	__gpt_set_mode(dev, mode & GPT_OPMODE_MASK);
	__gpt_set_clk(dev, clksrc & GPT_CLKSRC_MASK, clkdiv & GPT_CLKDIV_MASK);

	if (func)
		__gpt_set_handler(dev, func);

	if (dev->mode != GPT_FREE_RUN) {
		__gpt_set_cmp(dev, cmp, 0);
		if (!(dev->flags & GPT_NOIRQEN)) {
			__gpt_enable_irq(dev);
		}
	}

	if (!(dev->flags & GPT_NOAUTOEN))
		__gpt_start(dev);
}


int request_gpt(unsigned int id, unsigned int mode, unsigned int clksrc,
		unsigned int clkdiv, unsigned int cmp,
		void (*func) (unsigned long), unsigned int flags)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);
	if (!dev)
		return -EINVAL;

	if (dev->flags & GPT_IN_USE) {
		printk(KERN_ERR "%s: GPT%d is in use!\n", __func__, (id + 1));
		return -EBUSY;
	}

	gpt_update_lock(save_flags);
	setup_gpt_dev_locked(dev, mode, clksrc, clkdiv, cmp, func, flags);
	gpt_update_unlock(save_flags);

	return 0;
}
EXPORT_SYMBOL(request_gpt);

static void release_gpt_dev_locked(struct gpt_device *dev)
{
	__gpt_reset(dev);

	handlers[dev->id] = noop;
	dev->func = NULL;

	dev->flags = 0;
}

int free_gpt(unsigned int id)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);
	if (!dev)
		return -EINVAL;

	if (!(dev->flags & GPT_IN_USE))
		return 0;

	gpt_update_lock(save_flags);
	release_gpt_dev_locked(dev);
	gpt_update_unlock(save_flags);

	return 0;
}
EXPORT_SYMBOL(free_gpt);

int start_gpt(unsigned int id)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);

	if (!dev)
		return -EINVAL;

	if (!(dev->flags & GPT_IN_USE)) {
		printk(KERN_ERR "%s: GPT%d is not in use!\n", __func__, id);
		return -EBUSY;
	}

	gpt_update_lock(save_flags);
	__gpt_clrcnt(dev);
	__gpt_start(dev);
	gpt_update_unlock(save_flags);

	return 0;
}
EXPORT_SYMBOL(start_gpt);

int stop_gpt(unsigned int id)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);
	if (!dev)
		return -EINVAL;

	if (!(dev->flags & GPT_IN_USE)) {
		printk(KERN_ERR "%s: GPT%d is not in use!\n", __func__, id);
		return -EBUSY;
	}

	gpt_update_lock(save_flags);
	__gpt_stop(dev);
	gpt_update_unlock(save_flags);

	return 0;
}
EXPORT_SYMBOL(stop_gpt);

int restart_gpt(unsigned int id)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);

	if (!dev)
		return -EINVAL;

	if (!(dev->flags & GPT_IN_USE)) {
		printk(KERN_ERR "%s: GPT%d is not in use!\n", __func__, id);
		return -EBUSY;
	}

	gpt_update_lock(save_flags);
	__gpt_start(dev);
	gpt_update_unlock(save_flags);

	return 0;
}
EXPORT_SYMBOL(restart_gpt);


int gpt_is_counting(unsigned int id)
{
	unsigned long save_flags;
	int is_counting;
	struct gpt_device *dev = id_to_dev(id);

	if (!dev)
		return -EINVAL;

	if (!(dev->flags & GPT_IN_USE)) {
		printk(KERN_ERR "%s: GPT%d is not in use!\n", __func__, id);
		return -EBUSY;
	}

	gpt_update_lock(save_flags);
	is_counting = __gpt_get_status(dev);
	gpt_update_unlock(save_flags);

	return is_counting;
}
EXPORT_SYMBOL(gpt_is_counting);


int gpt_set_cmp(unsigned int id, unsigned int val)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);

	if (!dev)
		return -EINVAL;

	if (dev->mode == GPT_FREE_RUN)
		return -EINVAL;

	gpt_update_lock(save_flags);
	__gpt_set_cmp(dev, val, 0);
	gpt_update_unlock(save_flags);

	return 0;
}
EXPORT_SYMBOL(gpt_set_cmp);

int gpt_get_cmp(unsigned int id, unsigned int *ptr)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);
	if (!dev || !ptr)
		return -EINVAL;

	gpt_update_lock(save_flags);
	__gpt_get_cmp(dev, ptr);
	gpt_update_unlock(save_flags);

	return 0;
}
EXPORT_SYMBOL(gpt_get_cmp);

int gpt_get_cnt(unsigned int id, unsigned int *ptr)
{
	unsigned long save_flags;
	struct gpt_device *dev = id_to_dev(id);
	if (!dev || !ptr)
		return -EINVAL;

	if (!(dev->features & GPT_FEAT_64_BIT)) {
		__gpt_get_cnt(dev, ptr);
	} else {
		gpt_update_lock(save_flags);
		__gpt_get_cnt(dev, ptr);
		gpt_update_unlock(save_flags);
	}

	return 0;
}
EXPORT_SYMBOL(gpt_get_cnt);


int gpt_check_irq(unsigned int id)
{
	unsigned int mask = 0x1 << id;
	unsigned int status = DRV_Reg32(GPT_IRQSTA);

	return (status & mask) ? 1 : 0;
}
EXPORT_SYMBOL(gpt_check_irq);


int gpt_check_and_ack_irq(unsigned int id)
{
	unsigned int mask = 0x1 << id;
	unsigned int status = DRV_Reg32(GPT_IRQSTA);

	if (status & mask) {
		DRV_WriteReg32(GPT_IRQACK, mask);
		return 1;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(gpt_check_and_ack_irq);

unsigned int gpt_boot_time(void)
{
	return boot_time_value;
}
EXPORT_SYMBOL(gpt_boot_time);


static int mt_gpt_set_next_event(unsigned long cycles, struct clock_event_device *evt)
{
	struct gpt_device *dev = id_to_dev(GPT_CLKEVT_ID);

	/* printk("[%s]entry, evt=%lu\n", __func__, cycles); */

	__gpt_stop(dev);
	__gpt_set_cmp(dev, cycles, 0);
	__gpt_start_from_zero(dev);

	return 0;
}

static void mt_gpt_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	struct gpt_device *dev = id_to_dev(GPT_CLKEVT_ID);

	/* printk("[%s]entry, mode=%d\n", __func__, mode); */
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		__gpt_stop(dev);
		__gpt_set_mode(dev, GPT_REPEAT);
		__gpt_enable_irq(dev);
		__gpt_start_from_zero(dev);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		__gpt_stop(dev);
		__gpt_set_mode(dev, GPT_ONE_SHOT);
		__gpt_enable_irq(dev);
		__gpt_start_from_zero(dev);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		__gpt_stop(dev);
		__gpt_disable_irq(dev);
		__gpt_ack_irq(dev);
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static cycle_t mt_gpt_read(struct clocksource *cs)
{
	cycle_t cycles;
	unsigned int cnt[2] = { 0, 0 };
	struct gpt_device *dev = id_to_dev(GPT_CLKSRC_ID);
	__gpt_get_cnt(dev, cnt);
	
    if(GPT_CLKSRC_ID != GPT6) {
		/*
              * force do mask for high 32-bit to avoid unpredicted alignment
		*/
		cycles = (GPT_BIT_MASK & (cycle_t) (cnt[0]));
    }else {
		cycles = ((cycle_t) (cnt[1])) << 32 | (cycle_t) (cnt[0]);
    }

	return cycles;
}

static u64 notrace mt_read_sched_clock(void)
{
	return mt_gpt_read(NULL);
}

static void mt_gpt_init(void);
struct mt_clock mt_gpt = {
	.clockevent = {
		       .name = "mt-gpt",
		       .features = CLOCK_EVT_FEAT_ONESHOT,
		       .shift = 32,
		       .rating = 300,
		       .set_next_event = mt_gpt_set_next_event,
		       .set_mode = mt_gpt_set_mode,
		       },
	.clocksource = {
			.name = "mt-gpt",
			.rating = 300,
			.read = mt_gpt_read,
			.mask = CLOCKSOURCE_MASK(32),
			.shift = 25,
			.flags = CLOCK_SOURCE_IS_CONTINUOUS,
			},
	.irq = {
		.name = "mt-gpt",
		.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL | IRQF_TRIGGER_LOW,
		.handler = gpt_handler,
		.dev_id = &mt_gpt.clockevent,
		.irq = APXGPT_IRQ_BIT_ID,
		},
	.init_func = mt_gpt_init,
};

static void clkevt_handler(unsigned long data)
{
	struct clock_event_device *evt = (struct clock_event_device *)data;
	evt->event_handler(evt);
}

static inline void setup_clkevt(void)
{
	unsigned int cmp;
	struct clock_event_device *evt = &mt_gpt.clockevent;
	struct gpt_device *dev = id_to_dev(GPT_CLKEVT_ID);

	evt->mult = div_sc(SYS_CLK_RATE, NSEC_PER_SEC, evt->shift);
	evt->max_delta_ns = clockevent_delta2ns(0xffffffff, evt);
	evt->min_delta_ns = clockevent_delta2ns(3, evt);
	evt->cpumask = cpumask_of(0);
#ifndef CONFIG_MTK_FPGA

	setup_gpt_dev_locked(dev, GPT_REPEAT, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1,
			     SYS_CLK_RATE / HZ, clkevt_handler, GPT_ISR);
#else
	setup_gpt_dev_locked(dev, GPT_REPEAT, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1,
			     SYS_CLK_RATE / HZ, clkevt_handler, GPT_ISR);
#endif

	__gpt_get_cmp(dev, &cmp);
	printk("GPT1_CMP = %d, HZ = %d\n", cmp, HZ);
}

#ifndef CONFIG_CLKSRC_64_BIT
static inline void setup_clksrc(void)
{
	struct clocksource *cs = &mt_gpt.clocksource;
	struct gpt_device *dev = id_to_dev(GPT_CLKSRC_ID);

	cs->mult = clocksource_hz2mult(SYS_CLK_RATE, cs->shift);

	setup_gpt_dev_locked(dev, GPT_FREE_RUN, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1, 0, NULL, 0);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0))
	setup_sched_clock_needs_suspend((void *)mt_read_sched_clock, 32, SYS_CLK_RATE);
#else
    sched_clock_register(mt_read_sched_clock, 32, SYS_CLK_RATE);
#endif
}
#else

static u32 notrace jiffy_sched_clock_read(void)
{
	return (u32) (jiffies - INITIAL_JIFFIES);
}

static inline u64 cyc_to_ns(u64 cyc, u32 mult, u32 shift)
{
	return (cyc * mult) >> shift;
}

static u32 g_clksrc_init;

unsigned long long notrace sched_clock(void)
{
	struct clocksource *cs = &mt_gpt.clocksource;
	cycle_t cycles;

	if (0 == g_clksrc_init) {
		return jiffy_sched_clock_read();
	} else {
		cycles = mt_gpt_read(cs);
		cycles *= 1000 << 1;
#ifndef CONFIG_MTK_FPGA
		do_div(cycles, 13 << 1);
#else
		do_div(cycles, 6 << 1);
#endif
		return cycles;
		/* return cyc_to_ns(cycles,cs->mult, cs->shift); */
	}

}

static inline void setup_clksrc(void)
{
	struct clocksource *cs = &mt_gpt.clocksource;
	struct gpt_device *dev = id_to_dev(GPT_CLKSRC_ID);

	cs->mult = clocksource_hz2mult(SYS_CLK_RATE, cs->shift);
#ifndef CONFIG_MTK_FPGA

	setup_gpt_dev_locked(dev, GPT_FREE_RUN, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1, 0, NULL, 0);
#else

	setup_gpt_dev_locked(dev, GPT_FREE_RUN, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1, 0, NULL, 0);

#endif
	g_clksrc_init = 1;
}
#endif


static void setup_syscnt(void)
{
	/* set cpuxgpt free run,cpuxgpt always free run & oneshot no need to set */
	/* set cpuxgpt 13Mhz clock */
	set_cpuxgpt_clk(CLK_DIV2);
	/* enable cpuxgpt */
	enable_cpuxgpt();
}

static void mt_gpt_init(void)
{
	int i;
	unsigned long save_flags;

	boot_time_value = xgpt_boot_up_time();	/*record the time when init GPT */

	gpt_update_lock(save_flags);

	gpt_devs_init();

	for (i = 0; i < NR_GPTS; i++) {
		__gpt_reset(&gpt_devs[i]);
	}

	setup_clkevt();
	setup_clksrc();
	/* use cpuxgpt as syscnt */
	setup_syscnt();

	gpt_update_unlock(save_flags);
}

static ssize_t gpt_stat_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	char *p = NULL;
	char *page = NULL;
	int len = 0;
	int i = 0;
	int in_use;
	int is_counting;
	int err = 0;

	page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!page) {
		kfree(page);
		return -ENOMEM;
	}
	p = page;
	p += sprintf(p, "\n(HW Timer) GPT Status :\n");
	p += sprintf(p, "=========================================\n");

	for (i = 0; i < NR_GPTS; i++) {
		in_use = gpt_devs[i].flags & GPT_IN_USE;
		is_counting = gpt_is_counting(i);
		p += sprintf(p, "[GPT%d]in_use:%s, is_counting:%s\n", i + 1,
			     in_use ? "Y" : "N", is_counting ? "Y" : "N");
	}
	len = p - page;

	if (*ppos >= len) {
		kfree(page);
		return 0;
	}

	err = copy_to_user(buf, (char *)page, len);
	*ppos += len;
	if (err) {
		kfree(page);
		return err;
	}
	kfree(page);
	return len;
}

static const struct file_operations xgpt_cmd_proc_fops = {
	.read = gpt_stat_read,
};

static int __init gpt_mod_init(void)
{
	struct proc_dir_entry *xgpt_dir = NULL;
	xgpt_dir = proc_mkdir("mt_xgpt", NULL);
	proc_create("xgpt_stat", S_IRUGO, xgpt_dir, &xgpt_cmd_proc_fops);
#if 0
#ifndef CONFIG_MT6592_FPGA

	printk("GPT: chipver=%d\n", mt_get_chip_sw_ver());
#else
	printk("GPT: FPGA 6592\n");
#endif
#endif
	printk("GPT: init\n");

	return 0;
}
module_init(gpt_mod_init);

MODULE_DESCRIPTION("MT GPT Driver v0.1");
MODULE_LICENSE("GPL");
