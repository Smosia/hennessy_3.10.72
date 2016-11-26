#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/pid.h>

#include <linux/irq.h>
#include <linux/interrupt.h>

#include <linux/aee.h>
#include <linux/stacktrace.h>

#include <linux/printk.h>
#include "internal.h"

/* //////////////////////////////////////////////////////// */
/* --------------------------------------------------- */
/* Real work */
/* --------------------------------------------------- */
/*                     Define Proc entry               */
/* --------------------------------------------------- */
MT_DEBUG_ENTRY(printk_ctrl);
int mt_need_uart_console;
void mt_enable_uart(void);	/* printk.c */
void mt_disable_uart(void);	/* printk.c */
static int mt_printk_ctrl_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "=== mt printk controller ===\n");
	SEQ_printf(m, "mt_need_uart_console:%d, printk_disable_uart:%d, printk too much eandble:%d,detect line count: %d.\n",
		mt_need_uart_console, printk_disable_uart,
		get_logtoomuch_enable(), get_detect_count());

	return 0;
}

static ssize_t mt_printk_ctrl_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[64];
	int val;
	int ret;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt] = 0;

	ret = kstrtoul(buf, 10, (unsigned long *)&val);
	if (val == 0)
		mt_disable_uart();
	else if (val == 1) {
		mt_need_uart_console = 1;
		mt_enable_uart();
		pr_err("need uart log\n");
	} else if (val == 2)
		set_logtoomuch_enable(1);
	else if (val == 3)
		set_logtoomuch_enable(0);
	else if (val == 4)
		set_detect_count(100);
	else if (val == 5)
		set_detect_count(200);
	if (ret < 0)
		return ret;
	pr_err(" %d\n", val);
	return cnt;
}

static int __init init_mt_printk_ctrl(void)
{
	struct proc_dir_entry *pe;

	mt_need_uart_console = 0;	/* defualt, no uart */
	pe = proc_create("mtprintk", 0664, NULL, &mt_printk_ctrl_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

device_initcall(init_mt_printk_ctrl);
