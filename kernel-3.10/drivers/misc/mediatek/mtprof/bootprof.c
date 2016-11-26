#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>
#include <linux/printk.h>

#include "internal.h"

#define BOOT_STR_SIZE 128
#ifdef CONFIG_MT_ENG_BUILD
#define BOOT_LOG_NUM 64
#else
#define BOOT_LOG_NUM 48
#endif

struct boot_log_struct {
	u64 timestamp;
	char event[BOOT_STR_SIZE];
} mt_bootprof[BOOT_LOG_NUM];
int boot_log_count = 0;

static DEFINE_MUTEX(mt_bootprof_lock);
static int mt_bootprof_enabled;
static int bootprof_lk_t, bootprof_pl_t;
int boot_finish = 0;


module_param_named(pl_t, bootprof_pl_t, int, S_IRUGO | S_IWUSR);
module_param_named(lk_t, bootprof_lk_t, int, S_IRUGO | S_IWUSR);

void log_boot(char *str)
{
	unsigned long long ts;

	if (0 == mt_bootprof_enabled)
		return;
	ts = sched_clock();
	pr_err("BOOTPROF:%10Ld.%06ld:%s\n", nsec_high(ts), nsec_low(ts), str);
	if (boot_log_count >= BOOT_LOG_NUM) {
		pr_err("[BOOTPROF] not enuough bootprof buffer\n");
		return;
	}
	mutex_lock(&mt_bootprof_lock);
	mt_bootprof[boot_log_count].timestamp = ts;
	strcpy((char *)&mt_bootprof[boot_log_count].event, str);
	boot_log_count++;
	mutex_unlock(&mt_bootprof_lock);
}

#ifdef CONFIG_MT_PRINTK_UART_CONSOLE
static void bootup_finish(void)
{
	mt_disable_uart();
	set_logtoomuch_enable(1);
}
#else
static void bootup_finish(void)
{
	set_logtoomuch_enable(1);
}
#endif
/* extern void (*set_intact_mode)(void); */
static void mt_bootprof_switch(int on)
{
	mutex_lock(&mt_bootprof_lock);
	if (mt_bootprof_enabled ^ on) {
		if (on) {
			mt_bootprof_enabled = 1;
		} else {	/* boot up complete */
			mt_bootprof_enabled = 0;
			boot_finish = 1;
			bootup_finish();
		}
	}
	mutex_unlock(&mt_bootprof_lock);
}

static ssize_t mt_bootprof_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[BOOT_STR_SIZE];
	size_t copy_size = cnt;

	if (cnt >= sizeof(buf))
		copy_size = BOOT_STR_SIZE - 1;

	if (copy_from_user(&buf, ubuf, copy_size))
		return -EFAULT;

	if (cnt == 1) {
		if (buf[0] == '0')
			mt_bootprof_switch(0);
		else if (buf[0] == '1')
			mt_bootprof_switch(1);
	return 1;
	}

	buf[copy_size] = 0;
	log_boot(buf);

	return cnt;

}

static int mt_bootprof_show(struct seq_file *m, void *v)
{
	int i;

	SEQ_printf(m, "----------------------------------------\n");
	SEQ_printf(m, "%d	    BOOT PROF (unit:msec)\n", mt_bootprof_enabled);
	SEQ_printf(m, "----------------------------------------\n");

	if (bootprof_pl_t > 0 && bootprof_lk_t > 0) {
		SEQ_printf(m, "%10d        : %s\n", bootprof_pl_t, "preloader");
		SEQ_printf(m, "%10d        : %s\n", bootprof_lk_t, "lk");
		SEQ_printf(m, "%10d        : %s\n",
			   gpt_boot_time() - bootprof_pl_t - bootprof_lk_t, "lk->Kernel");
		SEQ_printf(m, "----------------------------------------\n");
	}

	for (i = 0; i < boot_log_count; i++) {
		SEQ_printf(m, "%10Ld.%06ld : %s\n",
			   nsec_high(mt_bootprof[i].timestamp), nsec_low(mt_bootprof[i].timestamp),
			   mt_bootprof[i].event);
	}
	SEQ_printf(m, "----------------------------------------\n");
	return 0;
}

/*** Seq operation of mtprof ****/
static int mt_bootprof_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_bootprof_show, inode->i_private);
}

static const struct file_operations mt_bootprof_fops = {
	.open = mt_bootprof_open,
	.write = mt_bootprof_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init init_boot_prof(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("bootprof", 0664, NULL, &mt_bootprof_fops);
	if (!pe)
		return -ENOMEM;
	/* set_intact_mode = NULL; */
	mt_bootprof_switch(1);
	return 0;
}

device_initcall(init_boot_prof);
