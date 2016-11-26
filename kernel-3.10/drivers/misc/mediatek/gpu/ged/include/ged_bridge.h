#ifndef __GED_BRIDGE_H__
#define __GED_BRIDGE_H__

#include "ged_base.h"
#include "ged_log.h"
#include "ged_type.h"

typedef struct _GED_BRIDGE_PACKAGE
{
    unsigned int    ui32FunctionID;
    int             i32Size;
    void            *pvParamIn;
    int             i32InBufferSize;
    void            *pvParamOut;
    int             i32OutBufferSize;
} GED_BRIDGE_PACKAGE;

/*****************************************************************************
 *  IOCTL values.
 *****************************************************************************/

#define GED_MAGIC 'g'

#define GED_IO(INDEX)    _IO(GED_MAGIC, INDEX, GED_BRIDGE_PACKAGE)
#define GED_IOW(INDEX)   _IOW(GED_MAGIC, INDEX, GED_BRIDGE_PACKAGE)
#define GED_IOR(INDEX)   _IOR(GED_MAGIC, INDEX, GED_BRIDGE_PACKAGE)
#define GED_IOWR(INDEX)  _IOWR(GED_MAGIC, INDEX, GED_BRIDGE_PACKAGE)
#define GED_GET_BRIDGE_ID(X)	_IOC_NR(X)

/******************************************************************************
 *  IOCTL Commands
 ******************************************************************************/
typedef enum
{
    GED_BRIDGE_COMMAND_LOG_BUF_GET,
    GED_BRIDGE_COMMAND_LOG_BUF_WRITE,
    GED_BRIDGE_COMMAND_LOG_BUF_RESET,
    GED_BRIDGE_COMMAND_BOOST_GPU_FREQ,
    GED_BRIDGE_COMMAND_MONITOR_3D_FENCE,
    GED_BRIDGE_COMMAND_QUERY_INFO,
    GED_BRIDGE_COMMAND_NOTIFY_VSYNC,
    GED_BRIDGE_COMMAND_DVFS_PROBE,
    GED_BRIDGE_COMMAND_DVFS_UM_RETURN,
} GED_BRIDGE_COMMAND_ID;

#define GED_BRIDGE_IO_LOG_BUF_GET			GED_IOWR(GED_BRIDGE_COMMAND_LOG_BUF_GET)
#define GED_BRIDGE_IO_LOG_BUF_WRITE			GED_IOWR(GED_BRIDGE_COMMAND_LOG_BUF_WRITE)
#define GED_BRIDGE_IO_LOG_BUF_RESET			GED_IOWR(GED_BRIDGE_COMMAND_LOG_BUF_RESET)
#define GED_BRIDGE_IO_BOOST_GPU_FREQ		GED_IOWR(GED_BRIDGE_COMMAND_BOOST_GPU_FREQ)
#define GED_BRIDGE_IO_MONITOR_3D_FENCE      GED_IOWR(GED_BRIDGE_COMMAND_MONITOR_3D_FENCE)
#define GED_BRIDGE_IO_QUERY_INFO      GED_IOWR(GED_BRIDGE_COMMAND_QUERY_INFO)
#define GED_BRIDGE_IO_NOTIFY_VSYNC       GED_IOWR(GED_BRIDGE_COMMAND_NOTIFY_VSYNC)
#define GED_BRIDGE_IO_DVFS_PROBE       GED_IOWR(GED_BRIDGE_COMMAND_DVFS_PROBE)
#define GED_BRIDGE_IO_DVFS_UM_RETURN GED_IOWR(GED_BRIDGE_COMMAND_DVFS_UM_RETURN)

/*****************************************************************************
 *  LOG_BUF_GET
 *****************************************************************************/
    
/* Bridge in structure for LOG_BUF_GET */
typedef struct GED_BRIDGE_IN_LOGBUFGET_TAG
{
    char acName[GED_LOG_BUF_NAME_LENGTH];
} GED_BRIDGE_IN_LOGBUFGET;


/* Bridge out structure for LOG_BUF_GETC */
typedef struct GED_BRIDGE_OUT_LOGBUFGET_TAG
{
    GED_ERROR eError;
    GED_LOG_BUF_HANDLE hLogBuf;
} GED_BRIDGE_OUT_LOGBUFGET;

/*****************************************************************************
 *  LOG_BUF_WRITE
 *****************************************************************************/

#define GED_BRIDGE_IN_LOGBUF_SIZE 320

/* Bridge in structure for LOG_BUF_WRITE */
typedef struct GED_BRIDGE_IN_LOGBUFWRITE_TAG
{
    GED_LOG_BUF_HANDLE hLogBuf;
    int attrs;
    char acLogBuf[GED_BRIDGE_IN_LOGBUF_SIZE];
} GED_BRIDGE_IN_LOGBUFWRITE;

/* Bridge out structure for LOG_BUF_WRITE */
typedef struct GED_BRIDGE_OUT_LOGBUFWRITE_TAG
{
    GED_ERROR eError;
} GED_BRIDGE_OUT_LOGBUFWRITE;

/******************************************************************************
 *  LOG_BUF_RESET
 ******************************************************************************/

/* Bridge in structure for LOG_BUF_RESET */
typedef struct GED_BRIDGE_IN_LOGBUFRESET_TAG
{
    GED_LOG_BUF_HANDLE hLogBuf;
} GED_BRIDGE_IN_LOGBUFRESET;

/* Bridge out structure for LOG_BUF_RESET */
typedef struct GED_BRIDGE_OUT_LOGBUFRESET_TAG
{
    GED_ERROR eError;
} GED_BRIDGE_OUT_LOGBUFRESET;

/*****************************************************************************
 *  BOOST GPU FREQ
 *****************************************************************************/

typedef enum
{
    GED_BOOST_GPU_FREQ_LEVEL_MAX = 100
} GED_BOOST_GPU_FREQ_LEVEL;

/* Bridge in structure for LOG_BUF_WRITE */
typedef struct GED_BRIDGE_IN_BOOSTGPUFREQ_TAG
{
    GED_BOOST_GPU_FREQ_LEVEL eGPUFreqLevel;
} GED_BRIDGE_IN_BOOSTGPUFREQ;

/* Bridge out structure for LOG_BUF_WRITE */
typedef struct GED_BRIDGE_OUT_BOOSTGPUFREQ_TAG
{
    GED_ERROR eError;
} GED_BRIDGE_OUT_BOOSTGPUFREQ;

/*****************************************************************************
 *  MONITOR 3D FENCE
 *****************************************************************************/

/* Bridge in structure for MONITOR3DFENCE */
typedef struct GED_BRIDGE_IN_MONITOR3DFENCE_TAG
{
    int fd;
} GED_BRIDGE_IN_MONITOR3DFENCE;

/* Bridge out structure for MONITOR3DFENCE */
typedef struct GED_BRIDGE_OUT_MONITOR3DFENCE_TAG
{
    GED_ERROR eError;
} GED_BRIDGE_OUT_MONITOR3DFENCE;

/*****************************************************************************
 *  QUERY INFO
 *****************************************************************************/

/* Bridge in structure for QUERY INFO*/
typedef struct GED_BRIDGE_IN_QUERY_INFO_TAG
{
    GED_INFO eType;
} GED_BRIDGE_IN_QUERY_INFO;


/* Bridge out structure for QUERY INFO*/
typedef struct GED_BRIDGE_OUT_QUERY_INFO_TAG
{
    unsigned long   retrieve;
} GED_BRIDGE_OUT_QUERY_INFO;
/*****************************************************************************
 *  NOTIFY VSYNC
 *****************************************************************************/

/* Bridge in structure for VSYNCEVENT */
typedef struct GED_BRIDGE_IN_NOTIFY_VSYNC_TAG
{
    GED_VSYNC_TYPE eType;
} GED_BRIDGE_IN_NOTIFY_VSYNC;

/* Bridge out structure for VSYNCEVENT */
typedef struct GED_BRIDGE_OUT_NOTIFY_VSYNC_TAG
{
    GED_DVFS_UM_QUERY_PACK sQueryData;
    GED_ERROR eError;
} GED_BRIDGE_OUT_NOTIFY_VSYNC;

/*****************************************************************************
 *  DVFS PROBE
 *****************************************************************************/

/* Bridge in structure for SWVSYNCEVENT */
typedef struct GED_BRIDGE_IN_DVFS_PROBE_TAG
{
    int          pid;
} GED_BRIDGE_IN_DVFS_PROBE;

/* Bridge out structure for RECORDSWAPBUFFERS */
typedef struct GED_BRIDGE_OUT_DVFS_PROBE_TAG
{
    GED_ERROR eError;
} GED_BRIDGE_OUT_DVFS_PROBE;

/*****************************************************************************
 *  DVFS UM RETURN
 *****************************************************************************/

/* Bridge in structure for DVFS_UM_RETURN */
typedef struct GED_BRIDGE_IN_DVFS_UM_RETURN_TAG
{
   unsigned long gpu_tar_freq; 
   bool bFallback;
} GED_BRIDGE_IN_DVFS_UM_RETURN;

/* Bridge out structure for DVFS_UM_RETURN */
typedef struct GED_BRIDGE_OUT_DVFS_UM_RETURN_TAG
{
    GED_ERROR eError;
} GED_BRIDGE_OUT_DVFS_UM_RETURN;

/*****************************************************************************
 *  BRIDGE FUNCTIONS
 *****************************************************************************/

int ged_bridge_log_buf_get(
    GED_BRIDGE_IN_LOGBUFGET *psLogBufGetIN,
    GED_BRIDGE_OUT_LOGBUFGET *psLogBufGetOUT);

int ged_bridge_log_buf_write(
    GED_BRIDGE_IN_LOGBUFWRITE *psLogBufWriteIN,
    GED_BRIDGE_OUT_LOGBUFWRITE *psLogBufWriteOUT);

int ged_bridge_log_buf_reset(
    GED_BRIDGE_IN_LOGBUFRESET *psLogBufResetIn,
    GED_BRIDGE_OUT_LOGBUFRESET *psLogBufResetOUT);

int ged_bridge_boost_gpu_freq(
    GED_BRIDGE_IN_BOOSTGPUFREQ *psBoostGpuFreqIN,
    GED_BRIDGE_OUT_BOOSTGPUFREQ *psBoostGpuFreqOUT);

int ged_bridge_monitor_3D_fence(
    GED_BRIDGE_IN_MONITOR3DFENCE *psMonitor3DFenceINT,
    GED_BRIDGE_OUT_MONITOR3DFENCE *psMonitor3DFenceOUT);

int ged_bridge_query_info(
    GED_BRIDGE_IN_QUERY_INFO *psQueryInfoINT,
    GED_BRIDGE_OUT_QUERY_INFO *psQueryInfoOUT);

int ged_bridge_notify_vsync(
    GED_BRIDGE_IN_NOTIFY_VSYNC *psNotifyVsyncINT,
    GED_BRIDGE_OUT_NOTIFY_VSYNC *psNotifyVsyncOUT);

int ged_bridge_dvfs_probe(
    GED_BRIDGE_IN_DVFS_PROBE *psDVFSProbeINT, 
    GED_BRIDGE_OUT_DVFS_PROBE *psDVFSProbeOUT);

int ged_bridge_dvfs_um_retrun(
    GED_BRIDGE_IN_DVFS_UM_RETURN *psDVFS_UM_returnINT, 
    GED_BRIDGE_OUT_DVFS_UM_RETURN *psDVFS_UM_returnOUT);

#endif
