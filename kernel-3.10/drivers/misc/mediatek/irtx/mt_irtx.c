#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#if !defined(CONFIG_MTK_LEGACY)
#include <linux/clk.h>
#endif /* !defined(CONFIG_MTK_LEGACY) */

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <mach/mt_pwm.h>
#include <mach/mt_pwm_hal.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>

#include "mt_irtx.h"

#ifdef GPIO_IRTX_OUT_PIN
#define IRTX_GPIO GPIO_IRTX_OUT_PIN
#else 
#ifdef GPIO_ANT_SEL0
#define IRTX_GPIO GPIO_ANT_SEL0
#else
#define IRTX_GPIO GPIO19
#endif
#endif

struct mt_irtx mt_irtx_dev;
void __iomem *irtx_reg_base;
unsigned int irtx_irq;

struct pwm_spec_config irtx_pwm_config = {
	.pwm_no = 0,
	.mode = PWM_MODE_MEMORY,
	.clk_div = CLK_DIV1,
	.clk_src = PWM_CLK_NEW_MODE_BLOCK,
	.pmic_pad = 0,
	.PWM_MODE_MEMORY_REGS.IDLE_VALUE = IDLE_FALSE,
	.PWM_MODE_MEMORY_REGS.GUARD_VALUE = GUARD_FALSE,
	.PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31,
	.PWM_MODE_MEMORY_REGS.HDURATION = 25, /* 1 microseconds, assume clock source is 26M */
	.PWM_MODE_MEMORY_REGS.LDURATION = 25,
	.PWM_MODE_MEMORY_REGS.GDURATION = 0,
	.PWM_MODE_MEMORY_REGS.WAVE_NUM = 1,
};

static int dev_char_open(struct inode *inode, struct file *file)
{
#if !defined(CONFIG_MTK_LEGACY)
	int clk_en_ret = 0;
#endif /* !defined(CONFIG_MTK_LEGACY) */

#ifdef GPIO_IRTX_OUT_PIN
pr_warn("[IRTX] open GPIO_IRTX_OUT_PIN defined\n");
#else 
	#ifdef GPIO_ANT_SEL0
		pr_warn("[IRTX] open GPIO_ANT_SEL0 defined\n");
	#else
		pr_warn("[IRTX] open GPIO19 defined\n");
	#endif
#endif

	if (atomic_read(&mt_irtx_dev.usage_cnt))
		return -EBUSY;

#if !defined(CONFIG_MTK_LEGACY)
	pr_notice("[IRTX][CCF]enable clk_irtx_main:%p\n", mt_irtx_dev.clk_irtx_main);
	clk_en_ret = clk_prepare_enable(mt_irtx_dev.clk_irtx_main);
	if (clk_en_ret) {
		pr_err("[IRTX][CCF]enable clk_irtx_main failed. ret:%d, clk_irtx_main:%p\n"
				, clk_en_ret, mt_irtx_dev.clk_irtx_main);
	}
#else /* !defined(CONFIG_MTK_LEGACY) */
	enable_clock(MT_CG_PERI_IRTX, "IRTX");
#endif /* !defined(CONFIG_MTK_LEGACY) */

	pr_warn("[IRTX] open by %s\n", current->comm);
	/* set IRTX_IRINV */
	irtx_write32(mt_irtx_dev.reg_base, IRTXCFG, irtx_read32(mt_irtx_dev.reg_base, IRTXCFG) | 0x80);
	nonseekable_open(inode, file);
	atomic_inc(&mt_irtx_dev.usage_cnt);
	return 0;
}

static int dev_char_close(struct inode *inode, struct file *file)
{
	pr_warn("[IRTX] close by %s\n", current->comm);
	atomic_dec(&mt_irtx_dev.usage_cnt);
#if !defined(CONFIG_MTK_LEGACY)
	pr_notice("[IRTX][CCF]disable clk_irtx_main:%p\n", mt_irtx_dev.clk_irtx_main);
	clk_disable_unprepare(mt_irtx_dev.clk_irtx_main);
#else /* !defined(CONFIG_MTK_LEGACY) */
	disable_clock(MT_CG_PERI_IRTX, "IRTX");
#endif /* !defined(CONFIG_MTK_LEGACY) */

	return 0;
}

static ssize_t dev_char_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static long dev_char_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned int para = 0, gpio_id = -1, en = 0;
	unsigned long mode = 0, dir = 0, outp = 0;

	switch (cmd) {
	case IRTX_IOC_SET_CARRIER_FREQ:
	    if (copy_from_user(&mt_irtx_dev.carrier_freq, (void __user *)arg, sizeof(unsigned int))) {
		    pr_err("[IRTX] IRTX_IOC_SET_CARRIER_FREQ: copy_from_user fail!\n");
		    ret = -EFAULT;
	    } else {
		    pr_warn("[IRTX] IRTX_IOC_SET_CARRIER_FREQ: %d\n", mt_irtx_dev.carrier_freq);
		    if (!mt_irtx_dev.carrier_freq) {
			    ret = -EINVAL;
			    mt_irtx_dev.carrier_freq = 38000;
		    }
	    }
	break;

	case IRTX_IOC_SET_IRTX_LED_EN:
	    if (copy_from_user(&para, (void __user *)arg, sizeof(unsigned int))) {
		    pr_err("[IRTX] IRTX_IOC_SET_IRTX_LED_EN: copy_from_user fail!\n");
		    ret = -EFAULT;
	    } else {
		    /* en: bit 12; */
		    /* gpio: bit 0-11 */
		    gpio_id = (unsigned long)((para & 0x0FFF0000) > 16);
		    en = (para & 0xF);
		    pr_warn("[IRTX] IRTX_IOC_SET_IRTX_LED_EN: 0x%x, gpio_id:%ul, en:%ul\n", para, gpio_id, en);
		    if (en) {
			    mode = GPIO_MODE_02;
			    dir = GPIO_DIR_OUT;
			    outp = GPIO_OUT_ZERO; /* Low means enable LED */
		    } else {
			    mode = GPIO_MODE_00;
			    dir = GPIO_DIR_OUT;
			    outp = GPIO_OUT_ONE;  /* High means disable LED */
		    }
		    gpio_id = IRTX_GPIO;

		    mt_set_gpio_mode(gpio_id, mode);
		    mt_set_gpio_dir(gpio_id, dir);
		    mt_set_gpio_out(gpio_id, outp);

		    pr_warn("[IRTX] IOC_SET_IRTX_LED_EN: gpio:0x%xl, mode:%d, dir:%d, out:%d\n", gpio_id
			    , mt_get_gpio_mode(gpio_id), mt_get_gpio_dir(gpio_id), mt_get_gpio_out(gpio_id));
	    }
	break;

	default:
	    pr_err("[IRTX] unknown ioctl cmd 0x%x\n", cmd);
	    ret = -ENOTTY;
    break;
	}
	return ret;
}

static void set_irtx_sw_mode(void)
{
	unsigned int ir_conf_wr;
	unsigned int L0H, L0L, L1H, L1L;
	unsigned int sync_h, sync_l;
	unsigned int cdt, cwt;
	struct irtx_config ir_conf;

	pr_warn("[IRTX] configure IrTx software mode\n");

	ir_conf.mode = 3;
	ir_conf.start = 0;
	ir_conf.sw_o = 0;
	ir_conf.b_ord = 1; /* LSB first */
	ir_conf.r_ord = 0; /* R0 first */
	ir_conf.ir_os = 1; /* modulated signal */
	ir_conf.ir_inv = 1;
	ir_conf.bit_num = 0;
	ir_conf.data_inv = 0;
	L0H = 0;
	L0L = (mt_irtx_dev.pwm_ch+1) & 0x7; /* FIXME, workaround for Denali, HW will fix on Jade */
	L1H = 0;
	L1L = 0;
	sync_h = 0;
	sync_l = 0;
	cwt = (CLOCK_SRC*1000*1000)/(mt_irtx_dev.carrier_freq); /* carrier freq. */
	cdt = cwt/3; /* duty=1/3 */

	memcpy(&ir_conf_wr, &ir_conf, sizeof(ir_conf));
	irtx_write32(mt_irtx_dev.reg_base, IRTXCFG, ir_conf_wr);
	irtx_write32(mt_irtx_dev.reg_base, IRTX_L0H, L0H);
	irtx_write32(mt_irtx_dev.reg_base, IRTX_L0L, L0L);
	irtx_write32(mt_irtx_dev.reg_base, IRTX_L1H, L1H);
	irtx_write32(mt_irtx_dev.reg_base, IRTX_L1L, L1L);
	irtx_write32(mt_irtx_dev.reg_base, IRTXSYNCH, sync_h);
	irtx_write32(mt_irtx_dev.reg_base, IRTXSYNCL, sync_l);
	irtx_write32(mt_irtx_dev.reg_base, IRTXMT, (cdt<<16)|(cwt&0xFFFF));

	pr_warn("[IRTX] configured IrTx: cfg=%x L0=%x/%x L1=%x/%x sync=%x/%x mt=%x/%x\n"
		    , ir_conf_wr, L0H, L0L, L1H, L1L, sync_h, sync_l, cdt, cwt);
	pr_warn("[IRTX] configured cfg=0x%x", (unsigned int)irtx_read32(mt_irtx_dev.reg_base, IRTXCFG));
}

static ssize_t dev_char_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	dma_addr_t wave_phy;
	void *wave_vir;
	int ret;
	int buf_size = (count + 3) / 4;  /* when count is 5... */

	pr_warn("[IRTX] irtx write len=0x%x, pwm=%d\n", (unsigned int)count, (unsigned int)irtx_pwm_config.pwm_no);
	wave_vir = dma_alloc_coherent(&mt_irtx_dev.plat_dev->dev, count, &wave_phy, GFP_KERNEL);
	if (!wave_vir) {
		pr_err("[IRTX] alloc memory fail\n");
		return -ENOMEM;
	}
	ret = copy_from_user(wave_vir, buf, count);
	if (ret) {
		pr_err("[IRTX] write, copy from user fail %d\n", ret);
		goto exit;
	}

	mt_set_intr_enable(0);
	mt_set_intr_enable(1);
	mt_pwm_26M_clk_enable_hal(1);
	pr_warn("[IRTX] irtx before read IRTXCFG:0x%x\n", (irtx_read32(mt_irtx_dev.reg_base, IRTXCFG)));
	irtx_pwm_config.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = (U32 *)wave_phy;
	irtx_pwm_config.PWM_MODE_MEMORY_REGS.BUF0_SIZE = (buf_size ? (buf_size - 1) : 0);

	set_irtx_sw_mode();
	mt_set_gpio_mode(IRTX_GPIO, GPIO_MODE_02);

	irtx_write32(mt_irtx_dev.reg_base, IRTXCFG, irtx_read32(mt_irtx_dev.reg_base, IRTXCFG)|0x1); /* STRT=1 */

	mt_set_intr_ack(0);
	mt_set_intr_ack(1);
	ret = pwm_set_spec_config(&irtx_pwm_config);
	pr_warn("[IRTX] pwm is triggered, %d\n", ret);

	msleep(count * 8 / 1000);
	msleep(100);
	ret = count;

exit:
	pr_warn("[IRTX] done, clean up\n");
	dma_free_coherent(&mt_irtx_dev.plat_dev->dev, count, wave_vir, wave_phy);
	irtx_write32(mt_irtx_dev.reg_base, IRTXCFG, irtx_read32(mt_irtx_dev.reg_base, IRTXCFG)&0xFFFFFFF7); /* SWO=0 */
	irtx_write32(mt_irtx_dev.reg_base, IRTXCFG, irtx_read32(mt_irtx_dev.reg_base, IRTXCFG)&0xFFFFFFFE); /* STRT=0 */
	mt_pwm_disable(irtx_pwm_config.pwm_no, irtx_pwm_config.pmic_pad);

	mt_set_gpio_mode(IRTX_GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(IRTX_GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(IRTX_GPIO, GPIO_OUT_ONE);

	return ret;
}

static irqreturn_t irtx_isr(int irq, void *data)
{
	return IRQ_HANDLED;
}

#define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
static u64 irtx_dma_mask = DMA_BIT_MASK((sizeof(unsigned long)<<3)); /* TODO: 3? */

static struct file_operations char_dev_fops = {
	.owner = THIS_MODULE,
	.open = &dev_char_open,
	.read = &dev_char_read,
	.write = &dev_char_write,
	.release = &dev_char_close,
	.unlocked_ioctl = &dev_char_ioctl,
};

#define irtx_driver_name "mt_irtx"
static int irtx_probe(struct platform_device *plat_dev)
{
	struct cdev *c_dev;
	dev_t dev_t_irtx;
	struct device *dev = NULL;
	static void *dev_class;
	u32 major = 0, minor = 0;
	int ret = 0;
	unsigned int gpio_id = -1;
	int error = 0;

	gpio_id = IRTX_GPIO;

	/* mt_set_gpio_mode(gpio_id, GPIO_MODE_02); */

#ifdef CONFIG_OF
	if (plat_dev->dev.of_node == NULL) {
		pr_err("[IRTX] irtx OF node is NULL\n");
		return -1;
	}

	mt_set_gpio_mode(gpio_id, GPIO_MODE_00);
	mt_set_gpio_dir(gpio_id, GPIO_DIR_OUT);
	mt_set_gpio_out(gpio_id, GPIO_OUT_ONE);

	of_property_read_u32(plat_dev->dev.of_node, "major", &major);
	mt_irtx_dev.reg_base = of_iomap(plat_dev->dev.of_node, 0);
	mt_irtx_dev.irq = irq_of_parse_and_map(plat_dev->dev.of_node, 0);
	of_property_read_u32(plat_dev->dev.of_node, "pwm_ch", &mt_irtx_dev.pwm_ch);
	pr_warn("[IRTX] device tree info: major=%d base=0x%p irq=%d pwm=%d\n"
		    , major, mt_irtx_dev.reg_base, mt_irtx_dev.irq, mt_irtx_dev.pwm_ch);
#endif

#if !defined(CONFIG_MTK_LEGACY)
	mt_irtx_dev.clk_irtx_main = devm_clk_get(&plat_dev->dev, "clk-irtx-main");
	if (IS_ERR(mt_irtx_dev.clk_irtx_main)) {
		pr_err("[IRTX][CCF]cannot get irtx clock. ptr_err:%ld\n", PTR_ERR(mt_irtx_dev.clk_irtx_main));
		return PTR_ERR(mt_irtx_dev.clk_irtx_main);
	}
	pr_notice("[IRTX][CCF]clk_irtx_main:%p\n", mt_irtx_dev.clk_irtx_main);
#endif /* !defined(CONFIG_MTK_LEGACY) */

	if (!major) {
		error = alloc_chrdev_region(&dev_t_irtx, 0, 1, irtx_driver_name);
		if (!error) {
			major = MAJOR(dev_t_irtx);
			minor = MINOR(dev_t_irtx);
		}
	} else {
		dev_t_irtx = MKDEV(major, minor);
	}

	ret = request_irq(mt_irtx_dev.irq, irtx_isr, IRQF_TRIGGER_FALLING, "IRTX", NULL); /* TODO: trigger */
	if (ret) {
		pr_err("[IRTX] request IRQ(%d) fail ret=%d\n", mt_irtx_dev.irq, ret);
		goto exit;
	}
	irtx_pwm_config.pwm_no = mt_irtx_dev.pwm_ch;

	mt_irtx_dev.plat_dev = plat_dev;
	mt_irtx_dev.plat_dev->dev.dma_mask = &irtx_dma_mask;
	mt_irtx_dev.plat_dev->dev.coherent_dma_mask = irtx_dma_mask;
	atomic_set(&mt_irtx_dev.usage_cnt, 0);
	mt_irtx_dev.carrier_freq = 38000; /* NEC as default */

	ret = register_chrdev_region(dev_t_irtx, 1, irtx_driver_name);
	if (ret) {
		pr_err("[IRTX] register_chrdev_region fail ret=%d\n", ret);
		goto exit;
	}
	c_dev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
	if (!c_dev) {
		pr_err("[IRTX] kmalloc cdev fail\n");
		goto exit;
	}
	cdev_init(c_dev, &char_dev_fops);
	c_dev->owner = THIS_MODULE;
	ret = cdev_add(c_dev, dev_t_irtx, 1);
	if (ret) {
		pr_err("[IRTX] cdev_add fail ret=%d\n", ret);
		goto exit;
	}
	dev_class = class_create(THIS_MODULE, irtx_driver_name);
	dev = device_create(dev_class, NULL, dev_t_irtx, NULL, "irtx");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("[IRTX] device_create fail ret=%d\n", ret);
		goto exit;
	}

exit:
	pr_warn("[IRTX] irtx probe ret=%d\n", ret);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id irtx_of_ids[] = {
	{.compatible = "mediatek,IRTX", },
	{}
};
#endif

static struct platform_driver irtx_driver =
{
	.driver = {
		.name = "mt_irtx",
	},
	.probe = irtx_probe,
};

static int __init irtx_init(void)
{
	int ret = 0;

	pr_warn("[IRTX] irtx init\n");
#ifdef CONFIG_OF
	irtx_driver.driver.of_match_table = irtx_of_ids;
#else
	pr_err("[IRTX] irtx needs device tree!\n");
	BUG_ON(1);
#endif

	ret = platform_driver_register(&irtx_driver);
	if (ret) {
		pr_err("[IRTX] irtx platform driver register fail %d\n", ret);
		goto exit;
	}

exit:
	return ret;
}


module_init(irtx_init);

MODULE_AUTHOR("Xiao Wang <xiao.wang@mediatek.com>");
MODULE_DESCRIPTION("Consumer IR transmitter driver v0.1");
