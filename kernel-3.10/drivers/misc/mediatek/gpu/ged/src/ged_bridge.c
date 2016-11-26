#include <linux/kernel.h>
#include <linux/mtk_gpu_utility.h>

#include "ged_base.h"
#include "ged_bridge.h"
#include "ged_log.h"
#include "ged_profile_dvfs.h"
#include "ged_monitor_3D_fence.h"
#include "ged_notify_sw_vsync.h"
#include "ged_dvfs.h"

//-----------------------------------------------------------------------------
int ged_bridge_log_buf_get(
		GED_BRIDGE_IN_LOGBUFGET *psLogBufGetIN,
		GED_BRIDGE_OUT_LOGBUFGET *psLogBufGetOUT)
{
	psLogBufGetOUT->hLogBuf = ged_log_buf_get(psLogBufGetIN->acName);
	psLogBufGetOUT->eError = psLogBufGetOUT->hLogBuf ? GED_OK : GED_ERROR_FAIL;
	return 0;
}
//-----------------------------------------------------------------------------
int ged_bridge_log_buf_write(
		GED_BRIDGE_IN_LOGBUFWRITE *psLogBufWriteIN,
		GED_BRIDGE_OUT_LOGBUFWRITE *psLogBufWriteOUT)
{
	psLogBufWriteOUT->eError = 
		ged_log_buf_print2(psLogBufWriteIN->hLogBuf, psLogBufWriteIN->attrs, psLogBufWriteIN->acLogBuf);

#if 0
	if (ged_log_buf_write(
				psLogBufWriteIN->hLogBuf, 
				/*from user*/psLogBufWriteIN->acLogBuf,
				GED_BRIDGE_IN_LOGBUF_SIZE) > 0)
	{
		psLogBufWriteOUT->eError = GED_OK;
	}
	else
	{
		psLogBufWriteOUT->eError = GED_ERROR_FAIL;
	}
#endif
	return 0;
}
//-----------------------------------------------------------------------------
int ged_bridge_log_buf_reset(
		GED_BRIDGE_IN_LOGBUFRESET *psLogBufResetIn,
		GED_BRIDGE_OUT_LOGBUFRESET *psLogBufResetOUT)
{
	psLogBufResetOUT->eError = ged_log_buf_reset(psLogBufResetIn->hLogBuf);
	return 0;
}
//-----------------------------------------------------------------------------
int ged_bridge_boost_gpu_freq(
		GED_BRIDGE_IN_BOOSTGPUFREQ *psBoostGpuFreqIN,
		GED_BRIDGE_OUT_BOOSTGPUFREQ *psBoostGpuFreqOUT)
{
#if 1
	psBoostGpuFreqOUT->eError = mtk_boost_gpu_freq() ? GED_OK : GED_ERROR_FAIL;
#else
	unsigned int ui32Count;
	if (mtk_custom_get_gpu_freq_level_count(&ui32Count))
	{
		int i32Level = (ui32Count - 1) - GED_BOOST_GPU_FREQ_LEVEL_MAX - psBoostGpuFreqIN->eGPUFreqLevel;
		mtk_boost_gpu_freq(i32Level);
		psBoostGpuFreqOUT->eError = GED_OK;
	}
	else
	{
		psBoostGpuFreqOUT->eError = GED_ERROR_FAIL;
	}
#endif
	return 0;
}
//-----------------------------------------------------------------------------
int ged_bridge_monitor_3D_fence(
		GED_BRIDGE_IN_MONITOR3DFENCE *psMonitor3DFenceINT,
		GED_BRIDGE_OUT_MONITOR3DFENCE *psMonitor3DFenceOUT)
{
	psMonitor3DFenceOUT->eError = ged_monitor_3D_fence_add(psMonitor3DFenceINT->fd);
	return 0;
}
//-----------------------------------------------------------------------------
int ged_bridge_query_info(
		GED_BRIDGE_IN_QUERY_INFO *psQueryInfoINT,
		GED_BRIDGE_OUT_QUERY_INFO *psQueryInfoOUT)
{
	psQueryInfoOUT->retrieve = ged_query_info( psQueryInfoINT->eType);
	return 0;
}
//-----------------------------------------------------------------------------
int ged_bridge_notify_vsync(
		GED_BRIDGE_IN_NOTIFY_VSYNC *psNotifyVsyncINT,
		GED_BRIDGE_OUT_NOTIFY_VSYNC *psNotifyVsyncOUT)
{
	psNotifyVsyncOUT->eError = 
		//ged_notify_vsync(psNotifyVsyncINT->eType, &psNotifyVsyncOUT->t);
		ged_notify_sw_vsync(psNotifyVsyncINT->eType, &psNotifyVsyncOUT->sQueryData);

	return 0;
}
//-----------------------------------------------------------------------------
int ged_bridge_dvfs_probe(
		GED_BRIDGE_IN_DVFS_PROBE *psDVFSProbeINT, 
		GED_BRIDGE_OUT_DVFS_PROBE *psDVFSProbeOUT)
{
	psDVFSProbeOUT->eError = ged_dvfs_probe(psDVFSProbeINT->pid);
	return 0;
}

//-----------------------------------------------------------------------------
int ged_bridge_dvfs_um_retrun(
		GED_BRIDGE_IN_DVFS_UM_RETURN *psDVFS_UM_returnINT, 
		GED_BRIDGE_OUT_DVFS_UM_RETURN *psDVFS_UM_returnOUT)
{
	psDVFS_UM_returnOUT->eError = 
		ged_dvfs_um_commit( psDVFS_UM_returnINT->gpu_tar_freq,
				psDVFS_UM_returnINT->bFallback);
	return 0;
}


