/*
 * mtk_governor.c - the MediaTek idle governor
 *
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/cpu.h>

struct mtk_idle_device {
	unsigned int        cpu;
	int                 last_state_idx;
};

#define LOAD_INT(x) ((x) >> FSHIFT)
#define LOAD_FRAC(x) LOAD_INT(((x) & (FIXED_1-1)) * 100)

#define IDLE_TAG     "[Power/swap]"
#define idle_dbg(fmt, args...)		pr_debug(IDLE_TAG fmt, ##args)

static DEFINE_PER_CPU(struct mtk_idle_device, mtk_idle_devices);

int __attribute__((weak)) mt_idle_select(int cpu)
{
	return 0;
}

void __attribute__((weak)) mt_cpuidle_framework_init(void)
{

}

/**
 * mtk_governor_select - selects the next idle state to enter
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 */
static int mtk_governor_select(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct mtk_idle_device *data = &__get_cpu_var(mtk_idle_devices);
	int state;

	state = mt_idle_select(data->cpu);
	state = (state >= drv->state_count) ? 0 : state;

	return state;
}

/**
 * mtk_governor_reflect - records that data structures need update
 * @dev: the CPU
 * @index: the index of actual entered state
 *
 * NOTE: it's important to be fast here because this operation will add to
 *       the overall exit latency.
 */
static void mtk_governor_reflect(struct cpuidle_device *dev, int index)
{
    /* TODO: implement actions for MTK governor & replace it */
}

/**
 * mtk_governor_enable_device - scans a CPU's states and does setup
 * @drv: cpuidle driver
 * @dev: the CPU
 */
static int mtk_governor_enable_device(struct cpuidle_driver *drv,
				struct cpuidle_device *dev)
{
	/* TODO: implement actions for MTK governor & replace it */
	struct mtk_idle_device *data = &per_cpu(mtk_idle_devices, dev->cpu);

	memset(data, 0, sizeof(struct mtk_idle_device));
	data->cpu = dev->cpu;

	return 0;
}

static struct cpuidle_governor mtk_governor = {
	.name =		"mtk_governor",
	.rating =	100,
	.enable =	mtk_governor_enable_device,
	.select =	mtk_governor_select,
	.reflect =	mtk_governor_reflect,
	.owner =	THIS_MODULE,
};

/**
 * init_mtk_governor - initializes the governor
 */
static int __init init_mtk_governor(void)
{
    /* TODO: check if debugfs_create_file() failed */
	mt_cpuidle_framework_init();
	return cpuidle_register_governor(&mtk_governor);
}

/**
 * exit_mtk_governor - exits the governor
 */
static void __exit exit_mtk_governor(void)
{
	cpuidle_unregister_governor(&mtk_governor);
}

MODULE_LICENSE("GPL");
module_init(init_mtk_governor);
module_exit(exit_mtk_governor);
