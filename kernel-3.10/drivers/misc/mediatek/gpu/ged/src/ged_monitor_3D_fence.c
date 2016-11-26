#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <asm/atomic.h>
#include <linux/module.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))
#include <linux/sync.h>
#else
#include <../drivers/staging/android/sync.h>
#endif

#include <linux/mtk_gpu_utility.h>
#include <trace/events/gpu.h>
#ifdef GED_DVFS_ENABLE
#include <mach/mt_gpufreq.h>
#endif

#include "ged_monitor_3D_fence.h"

#include "ged_log.h"
#include "ged_base.h"
#include "ged_type.h"
#include "ged_dvfs.h"

#include <asm/div64.h>

static atomic_t g_i32Count = ATOMIC_INIT(0);
static unsigned int ged_monitor_3D_fence_debug = 0;
static unsigned int ged_monitor_3D_fence_disable = 0;
static unsigned int ged_monitor_3D_fence_systrace = 0;
static unsigned long g_ul3DFenceDoneTime = 0;


extern bool mtk_get_bottom_gpu_freq(unsigned int *pui32FreqLevel);

#ifdef GED_DEBUG_MONITOR_3D_FENCE
extern GED_LOG_BUF_HANDLE ghLogBuf_GED;
#endif
extern GED_LOG_BUF_HANDLE ghLogBuf_DVFS;

typedef struct GED_MONITOR_3D_FENCE_TAG
{
	struct sync_fence_waiter    sSyncWaiter;
	struct work_struct          sWork;
	struct sync_fence*          psSyncFence;
} GED_MONITOR_3D_FENCE;

static void ged_sync_cb(struct sync_fence *fence, struct sync_fence_waiter *waiter)
{
	GED_MONITOR_3D_FENCE *psMonitor;
	unsigned long long t;

	t = ged_get_time();


	do_div(t,1000);

	ged_monitor_3D_fence_notify();
	ged_dvfs_cal_gpu_utilization_force();
	psMonitor = GED_CONTAINER_OF(waiter, GED_MONITOR_3D_FENCE, sSyncWaiter);

	ged_log_buf_print(ghLogBuf_DVFS, "[-] ged_monitor_3D_fence_done (ts=%llu) %p", t, psMonitor->psSyncFence);

	schedule_work(&psMonitor->sWork);
}

static void ged_monitor_3D_fence_work_cb(struct work_struct *psWork)
{
	GED_MONITOR_3D_FENCE *psMonitor;


#ifdef GED_DEBUG_MONITOR_3D_FENCE
	ged_log_buf_print(ghLogBuf_GED, "ged_monitor_3D_fence_work_cb");
#endif

	if (atomic_sub_return(1, &g_i32Count) < 1)
	{
		if (0 == ged_monitor_3D_fence_disable)
		{
			unsigned int uiFreqLevelID;
			if (mtk_get_bottom_gpu_freq(&uiFreqLevelID))
			{
				if (uiFreqLevelID > 0)
				{
#ifdef GED_DEBUG_MONITOR_3D_FENCE
					ged_log_buf_print(ghLogBuf_GED, "mtk_set_bottom_gpu_freq(0)");
#endif
					mtk_set_bottom_gpu_freq(0);
#if 0
#ifdef CONFIG_MTK_SCHED_TRACERS
					if (ged_monitor_3D_fence_systrace)
					{
						unsigned long long t = cpu_clock(smp_processor_id());
						trace_gpu_sched_switch("Smart Boost", t, 0, 0, 1);
					}
#endif
#endif
				}
			}
		}
	}

	if (ged_monitor_3D_fence_debug > 0)
	{
		GED_LOGI("[-]3D fences count = %d\n", atomic_read(&g_i32Count));
	}

	psMonitor = GED_CONTAINER_OF(psWork, GED_MONITOR_3D_FENCE, sWork);
	sync_fence_put(psMonitor->psSyncFence);
	ged_free(psMonitor, sizeof(GED_MONITOR_3D_FENCE));
}

unsigned long ged_monitor_3D_fence_done_time()
{
	return g_ul3DFenceDoneTime;
}

GED_ERROR ged_monitor_3D_fence_add(int fence_fd)
{
	int err;

	unsigned long long t;

	GED_MONITOR_3D_FENCE* psMonitor;

	t = ged_get_time();


	do_div(t,1000);


	psMonitor = (GED_MONITOR_3D_FENCE*)ged_alloc(sizeof(GED_MONITOR_3D_FENCE));

#ifdef GED_DEBUG_MONITOR_3D_FENCE
	ged_log_buf_print(ghLogBuf_GED, "[+]ged_monitor_3D_fence_add");
#endif

	if (!psMonitor)
	{
		return GED_ERROR_OOM;
	}

	sync_fence_waiter_init(&psMonitor->sSyncWaiter, ged_sync_cb);
	INIT_WORK(&psMonitor->sWork, ged_monitor_3D_fence_work_cb);
	psMonitor->psSyncFence = sync_fence_fdget(fence_fd);
	if (NULL == psMonitor->psSyncFence)
	{
		ged_free(psMonitor, sizeof(GED_MONITOR_3D_FENCE));
		return GED_ERROR_INVALID_PARAMS;
	}

	ged_log_buf_print(ghLogBuf_DVFS, "[+] ged_monitor_3D_fence_add (ts=%llu) %p", t, psMonitor->psSyncFence);

#ifdef GED_DEBUG_MONITOR_3D_FENCE
	ged_log_buf_print(ghLogBuf_GED, "[+]sync_fence_wait_async");
#endif

	err = sync_fence_wait_async(psMonitor->psSyncFence, &psMonitor->sSyncWaiter);

#ifdef GED_DEBUG_MONITOR_3D_FENCE
	ged_log_buf_print(ghLogBuf_GED, "[-]sync_fence_wait_async, err = %d", err);
#endif

	if ((1 == err) || (0 > err))
	{
		sync_fence_put(psMonitor->psSyncFence);
		ged_free(psMonitor, sizeof(GED_MONITOR_3D_FENCE));
	}
	else if (0 == err)
	{
		int iCount = atomic_add_return (1, &g_i32Count);
		if (iCount > 1)
		{
			if (0 == ged_monitor_3D_fence_disable)
			{
				unsigned int uiFreqLevelID;
				if (mtk_get_bottom_gpu_freq(&uiFreqLevelID))
				{
#ifdef GED_DVFS_ENABLE
					if (uiFreqLevelID != mt_gpufreq_get_dvfs_table_num() - 1)
#else
						if (uiFreqLevelID != 9999) // NEVER TRUE
#endif
						{
#if 0
#ifdef CONFIG_MTK_SCHED_TRACERS
							if (ged_monitor_3D_fence_systrace)
							{
								unsigned long long t = cpu_clock(smp_processor_id());
								trace_gpu_sched_switch("Smart Boost", t, 1, 0, 1);
							}
#endif
#endif

#ifdef GED_DVFS_ENABLE
							mtk_set_bottom_gpu_freq(mt_gpufreq_get_dvfs_table_num() - 1);
#endif
						}
				}
			}
		}
	}

	if (ged_monitor_3D_fence_debug > 0)
	{
		GED_LOGI("[+]3D fences count = %d\n", atomic_read(&g_i32Count));
	}
#ifdef GED_DEBUG_MONITOR_3D_FENCE
	ged_log_buf_print(ghLogBuf_GED, "[-]ged_monitor_3D_fence_add, count = %d", atomic_read(&g_i32Count));
#endif

	return GED_OK;
}

void ged_monitor_3D_fence_set_disable(GED_BOOL bFlag)
{
	if(bFlag!=ged_monitor_3D_fence_disable)
	{
		ged_monitor_3D_fence_disable = bFlag;
	}
}

void ged_monitor_3D_fence_notify(void)
{
	unsigned long long t;

	t = ged_get_time();

	do_div(t,1000);

	g_ul3DFenceDoneTime = (unsigned long)t;
}


module_param(ged_monitor_3D_fence_debug, uint, 0644);
module_param(ged_monitor_3D_fence_disable, uint, 0644);
module_param(ged_monitor_3D_fence_systrace, uint, 0644);
