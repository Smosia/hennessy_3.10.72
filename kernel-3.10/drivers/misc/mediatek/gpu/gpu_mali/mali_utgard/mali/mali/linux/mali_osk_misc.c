/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2015 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_osk_misc.c
 * Implementation of the OS abstraction layer for the kernel device driver
 */
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include "mali_osk.h"

extern void smi_dumpDebugMsg(void);
extern int m4u_dump_debug_registers(void);;

#if !defined(CONFIG_MALI_QUIET)
void _mali_osk_dbgmsg(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vprintk(fmt, args);
	va_end(args);
}
#endif /* !defined(CONFIG_MALI_QUIET) */

u32 _mali_osk_snprintf(char *buf, u32 size, const char *fmt, ...)
{
	int res;
	va_list args;
	va_start(args, fmt);

	res = vscnprintf(buf, (size_t)size, fmt, args);

	va_end(args);
	return res;
}

void _mali_osk_ctxprintf(_mali_osk_print_ctx *print_ctx, const char *fmt, ...)
{
	va_list args;
	char buf[512];

	va_start(args, fmt);
	vscnprintf(buf, 512, fmt, args);
	seq_printf(print_ctx, buf);
	va_end(args);
}

void _mali_osk_abort(void)
{
	/* make a simple fault by dereferencing a NULL pointer */
	dump_stack();
	*(int *)0 = 0;
}

void _mali_osk_break(void)
{
	_mali_osk_abort();
}

u32 _mali_osk_get_pid(void)
{
	/* Thread group ID is the process ID on Linux */
	return (u32)current->tgid;
}

char *_mali_osk_get_comm(void)
{
	return (char *)current->comm;
}


u32 _mali_osk_get_tid(void)
{
	/* pid is actually identifying the thread on Linux */
	u32 tid = current->pid;

	/* If the pid is 0 the core was idle.  Instead of returning 0 we return a special number
	 * identifying which core we are on. */
	if (0 == tid) {
		tid = -(1 + raw_smp_processor_id());
	}

	return tid;
}
