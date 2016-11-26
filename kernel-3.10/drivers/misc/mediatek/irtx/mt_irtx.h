struct mt_irtx {
	unsigned int pwm_ch;
	void __iomem *reg_base;
	unsigned int irq;
	struct platform_device *plat_dev;
	unsigned int carrier_freq;
	atomic_t usage_cnt;

#if !defined(CONFIG_MTK_LEGACY)
	/* struct clk *clk_irtx_main; */
	struct clk *clk_irtx_main;
#endif /* !defined(CONFIG_MTK_LEGACY) */
};

struct irtx_config {
	unsigned int start : 1;
	unsigned int mode : 2;
	unsigned int sw_o : 1;
	unsigned int b_ord : 1;
	unsigned int r_ord : 1;
	unsigned int ir_os : 1;
	unsigned int ir_inv : 1;
	unsigned int bit_num : 7;
	unsigned int data_inv : 1;
};

#define IRTX_IOC_SET_CARRIER_FREQ _IOW('R', 0, unsigned int)

#define IRTX_IOC_SET_IRTX_LED_EN	_IOW('R', 10, unsigned int)

#define irtx_write32(b, a, v) mt_reg_sync_writel(v, (b)+(a))
#define irtx_read32(b, a) ioread32((void __iomem *)((b)+(a)))

#define CLOCK_SRC 26 /* MHz */

#define IRTXCFG 0x0
#define IRTXD0 0x4
#define IRTXD1 0x8
#define IRTXD2 0xC
#define IRTX_L0H 0x10
#define IRTX_L0L 0x14
#define IRTX_L1H 0x18
#define IRTX_L1L 0x1C
#define IRTXSYNCH 0x20
#define IRTXSYNCL 0x24
#define IRTXMT 0x28

