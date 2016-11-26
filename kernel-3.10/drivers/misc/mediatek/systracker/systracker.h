#ifndef _SYSTRAKCER_H
#define _SYSTRAKCER_H

#include <linux/platform_device.h>

#define BUS_DBG_CON                 (BUS_DBG_BASE + 0x0000)
#define BUS_DBG_TIMER_CON           (BUS_DBG_BASE + 0x0004)
#define BUS_DBG_TIMER               (BUS_DBG_BASE + 0x0008)
#define BUS_DBG_WP                  (BUS_DBG_BASE + 0x000C)
#define BUS_DBG_WP_MASK             (BUS_DBG_BASE + 0x0010)
#define BUS_DBG_MON                 (BUS_DBG_BASE + 0x0014)
#define BUS_DBG_AR_TRACK_L(__n)     (BUS_DBG_BASE + 0x0100 + 8 * (__n))
#define BUS_DBG_AR_TRACK_H(__n)     (BUS_DBG_BASE + 0x0104 + 8 * (__n))
#define BUS_DBG_AR_TRANS_TID(__n)   (BUS_DBG_BASE + 0x0180 + 4 * (__n))
#define BUS_DBG_AW_TRACK_L(__n)     (BUS_DBG_BASE + 0x0200 + 8 * (__n))
#define BUS_DBG_AW_TRACK_H(__n)     (BUS_DBG_BASE + 0x0204 + 8 * (__n))
#define BUS_DBG_AW_TRANS_TID(__n)   (BUS_DBG_BASE + 0x0280 + 4 * (__n))


#define BUS_DBG_BUS_MHZ             (266)
#define BUS_DBG_NUM_TRACKER         (8)
#define BUS_DBG_CON_DEFAULT_VAL     (0x00006036)

#define BUS_DBG_CON_BUS_DBG_EN      (0x00000001)
#define BUS_DBG_CON_TIMEOUT_EN      (0x00000002)
#define BUS_DBG_CON_SLV_ERR_EN      (0x00000004)
#define BUS_DBG_CON_WP_EN           (0x00000008)
#define BUS_DBG_CON_IRQ_AR_EN       (0x00000010)
#define BUS_DBG_CON_IRQ_AW_EN       (0x00000020)
#define BUS_DBG_CON_SW_RST_DN       (0x00000040)
/* DE uses exotic name for ESO items... we use another human-readable name */
#define BUS_DBG_CON_IRQ_WP_EN       (0x00000040)
#define BUS_DBG_CON_IRQ_CLR         (0x00000080)
#define BUS_DBG_CON_IRQ_AR_STA      (0x00000100)
#define BUS_DBG_CON_IRQ_AW_STA      (0x00000200)
#define BUS_DBG_CON_IRQ_WP_STA      (0x00000400)
#define BUS_DBG_CON_WDT_RST_EN      (0x00001000)
#define BUS_DBG_CON_HALT_ON_EN      (0x00002000)
#define BUS_DBG_CON_BUS_OT_EN       (0x00004000)
#define BUS_DBG_CON_SW_RST          (0x00010000)

#define BUS_DBG_CON_IRQ_EN          (BUS_DBG_CON_IRQ_AR_EN | BUS_DBG_CON_IRQ_AW_EN | BUS_DBG_CON_IRQ_WP_EN)

static inline unsigned int extract_n2mbits(unsigned int input, int n, int m)
{
/*
 * 1. ~0 = 1111 1111 1111 1111 1111 1111 1111 1111
 * 2. ~0 << (m - n + 1) = 1111 1111 1111 1111 1100 0000 0000 0000
 * // assuming we are extracting 14 bits, the +1 is added for inclusive selection
 * 3. ~(~0 << (m - n + 1)) = 0000 0000 0000 0000 0011 1111 1111 1111
 */
        int mask;
        if (n > m) {
            n = n + m;
            m = n - m;
            n = n - m;
        }
        mask = ~(~0 << (m - n + 1));
        return ((input >> n) & mask);
}

struct mt_systracker_driver {
	struct	platform_driver driver;
	struct	platform_device device;
	void	(*reset_systracker)(void);
	int		(*enable_watchpoint)(void);
	int		(*disable_watchpoint)(void);
	int		(*set_watchpoint_address)(unsigned int wp_phy_address);
	void	(*enable_systracker)(void);
	void	(*disable_systracker)(void);
	int		(*test_systracker)(void);
	int         (*systracker_probe)(struct platform_device *pdev);
	unsigned int (*systracker_timeout_value)(void);
	int 	(*systracker_remove)(struct platform_device *pdev);
	int 	(*systracker_suspend)(struct platform_device *pdev, pm_message_t state);
	int 	(*systracker_resume)(struct platform_device *pdev);
	int	(*systracker_hook_fault)(void);
	int	(*systracker_test_init)(void);
	void	(*systracker_test_cleanup)(void);
	void	(*systracker_wp_test)(void);
	void	(*systracker_read_timeout_test)(void);
	void	(*systracker_write_timeout_test)(void);
	void	(*systracker_withrecord_test)(void);
	void	(*systracker_notimeout_test)(void);
};

struct systracker_entry_t
{
    unsigned int dbg_con;
    unsigned int ar_track_l[BUS_DBG_NUM_TRACKER];
    unsigned int ar_track_h[BUS_DBG_NUM_TRACKER];
    unsigned int ar_trans_tid[BUS_DBG_NUM_TRACKER];
    unsigned int aw_track_l[BUS_DBG_NUM_TRACKER];
    unsigned int aw_track_h[BUS_DBG_NUM_TRACKER];
    unsigned int aw_trans_tid[BUS_DBG_NUM_TRACKER];
};
struct systracker_config_t
{
        int state;
        int enable_timeout;
        int enable_slave_err;
        int enable_wp;
        int enable_irq;
        int timeout_ms;
        int wp_phy_address;
};

extern int tracker_dump(char *buf);
extern void dump_backtrace_entry_ramconsole_print(unsigned long where, unsigned long from, unsigned long frame);
extern void dump_regs(const char *fmt, const char v1, const unsigned int reg, const unsigned int reg_val);
extern struct mt_systracker_driver *get_mt_systracker_drv(void);

extern void __iomem *BUS_DBG_BASE;
extern int systracker_irq;
extern struct systracker_config_t track_config;
extern struct systracker_entry_t track_entry;

extern unsigned int is_systracker_irq_registered;
extern void save_entry(void);
extern void aee_dump_backtrace(struct pt_regs *regs, struct task_struct *tsk);
extern irqreturn_t systracker_isr(void);
extern int systracker_watchpoint_enable(void);
extern int systracker_set_watchpoint_addr(unsigned phy_addr);
extern void systracker_reset(void);
extern void systracker_enable(void);
extern void systracker_disable(void);
extern int systracker_handler(unsigned long addr, unsigned int fsr, struct pt_regs *regs);
extern int systracker_probe(struct platform_device *pdev);
extern int systracker_remove(struct platform_device *pdev);
extern int systracker_suspend(struct platform_device *pdev, pm_message_t state);
extern int systracker_resume(struct platform_device *pdev);
extern void systracker_reset(void);
extern void systracker_enable(void);
extern void systracker_test_cleanup(void);


//#define SYSTRACKER_TEST_SUIT  //enable for driver poring test suit
//#define TRACKER_DEBUG 0
#endif
