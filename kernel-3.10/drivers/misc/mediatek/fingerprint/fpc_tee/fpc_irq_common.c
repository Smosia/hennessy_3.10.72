/* Fingerprint Cards, Hybrid Touch sensor driver
 *
 * Copyright (c) 2014,2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 *
 * Software license : "Dual BSD/GPL"
 * see <linux/module.h> and ./Documentation
 * for  details.
 *
*/

#include "fpc_irq_common.h"

#define DEBUG

/* -------------------------------------------------------------------------- */
int fpc_irq_check_instance(const char *str_name)
{
	const char str_ref[] = FPC_IRQ_DEV_NAME;
	size_t index;

	printk(KERN_INFO "%s => %s \n", __func__, str_name);

	index = sizeof(str_ref);
	while (index) {
		--index;
		if (str_ref[index] != str_name[index])
			return 0;
	}
	return -EINVAL;
}


/* -------------------------------------------------------------------------- */
int fpc_irq_send_signal(struct device *dev,
			pid_t dst_pid,
			int signo,
			int payload)
{
	int ret = 0;

	struct siginfo		info;
	struct task_struct	*dst_task;

	dev_dbg(dev, "%s\n", __func__);

	if (dst_pid == 0) {
		dev_err(dev, "%s : destination PID not set!\n", __func__);
		return -ENODEV;
	}

	memset(&info, 0, sizeof(struct siginfo));

	info.si_signo = signo;
	info.si_code  = SI_QUEUE;
	info.si_int   = payload;

	rcu_read_lock();

	dst_task = pid_task(find_pid_ns(dst_pid, &init_pid_ns), PIDTYPE_PID);

	if (dst_task == NULL)
		ret = -ENODEV;

	rcu_read_unlock();

	if (ret < 0) {
		dev_err(dev, "%s : no such PID %d\n", __func__, dst_pid);
		return ret;
	}

	ret = send_sig_info(signo, &info, dst_task);
	if (ret < 0)
		dev_err(dev, "%s : unable to send signal\n", __func__);
	else
		dev_dbg(dev, "%s sent signal %d, with payload %d to PID:%d\n", __func__, signo, payload, dst_pid);

	return ret;
}


/* -------------------------------------------------------------------------- */
void fpc_irq_wake_unlock(fpc_irq_data_t *data)
{
    pr_debug("%s: enter\n", __func__);

    if (data->wake_lock_acquired) {
        wake_unlock(&data->wake_lock);
        data->wake_lock_acquired = 0;
    }
}


/* -------------------------------------------------------------------------- */
void fpc_irq_wake_unlock_timer_handler(unsigned long ptr)
{
    fpc_irq_data_t *data = (fpc_irq_data_t*)ptr;

    pr_debug("%s: enter\n", __func__);
    fpc_irq_wake_unlock(data);
}


/* -------------------------------------------------------------------------- */
void fpc_irq_wake_lock_delayed_unlock(fpc_irq_data_t *data)
{
    pr_debug("%s: enter\n", __func__);

    if (!data->wake_lock_acquired) {
        wake_lock(&data->wake_lock);
        data->wake_lock_acquired = 1;
    }
    mod_timer(&data->wake_unlock_timer, jiffies + FPC_IRQ_WAKE_TIME);
}


/* -------------------------------------------------------------------------- */
