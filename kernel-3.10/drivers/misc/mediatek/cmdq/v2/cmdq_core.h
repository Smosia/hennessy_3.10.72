#ifndef __CMDQ_CORE_H__
#define __CMDQ_CORE_H__

#include <linux/list.h>
#include <linux/time.h>
#include <linux/xlog.h>
#include <linux/aee.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include "cmdq_def.h"

/*  */
/* address conversion for 4GB ram support: */
/* .address register: 32 bit */
/* .physical address: 32 bit, or 64 bit for CONFIG_ARCH_DMA_ADDR_T_64BIT enabled */
/*  */
/* when 33 bit enabled(4GB ram), 0x_0_xxxx_xxxx and 0x_1_xxxx_xxxx access is same for CPU */
/*  */
/*  */
/* 0x0            0x0_4000_0000        0x1_0000_0000        0x1_4000_0000 */
/* |---1GB HW addr---|------3GB DRAM------|----1GB DRAM(new)----|-----3GB DRAM(same)------| */
/* |                                               | */
/* |<--------------4GB RAM support---------------->| */
/*  */
#define CMDQ_PHYS_TO_AREG(addr) ((addr) & 0xFFFFFFFF)	/* truncate directly */
#define CMDQ_AREG_TO_PHYS(addr) ((addr) | 0L)

#define CMDQ_LONGSTRING_MAX (180)
#define CMDQ_DELAY_RELEASE_RESOURCE_MS (1000)

#define CMDQ_ENG_ISP_GROUP_BITS                 ((1LL << CMDQ_ENG_ISP_IMGI) |       \
						 (1LL << CMDQ_ENG_ISP_IMGO) |       \
						 (1LL << CMDQ_ENG_ISP_IMG2O))

#define CMDQ_ENG_MDP_GROUP_BITS                 ((1LL << CMDQ_ENG_MDP_CAMIN) |      \
						 (1LL << CMDQ_ENG_MDP_RDMA0) |      \
						 (1LL << CMDQ_ENG_MDP_RDMA1) |      \
						 (1LL << CMDQ_ENG_MDP_RSZ0) |       \
						 (1LL << CMDQ_ENG_MDP_RSZ1) |       \
						 (1LL << CMDQ_ENG_MDP_RSZ2) |       \
						 (1LL << CMDQ_ENG_MDP_TDSHP0) |     \
						 (1LL << CMDQ_ENG_MDP_TDSHP1) |     \
						 (1LL << CMDQ_ENG_MDP_COLOR0) |     \
						 (1LL << CMDQ_ENG_MDP_WROT0) |      \
						 (1LL << CMDQ_ENG_MDP_WROT1) |      \
						 (1LL << CMDQ_ENG_MDP_WDMA))

#define CMDQ_ENG_DISP_GROUP_BITS                ((1LL << CMDQ_ENG_DISP_UFOE) |      \
						 (1LL << CMDQ_ENG_DISP_AAL) |       \
						 (1LL << CMDQ_ENG_DISP_COLOR0) |    \
						 (1LL << CMDQ_ENG_DISP_COLOR1) |    \
						 (1LL << CMDQ_ENG_DISP_RDMA0) |     \
						 (1LL << CMDQ_ENG_DISP_RDMA1) |     \
						 (1LL << CMDQ_ENG_DISP_RDMA2) |     \
						 (1LL << CMDQ_ENG_DISP_WDMA0) |     \
						 (1LL << CMDQ_ENG_DISP_WDMA1) |     \
						 (1LL << CMDQ_ENG_DISP_OVL0) |      \
						 (1LL << CMDQ_ENG_DISP_OVL1) |      \
						 (1LL << CMDQ_ENG_DISP_OVL2) |      \
						 (1LL << CMDQ_ENG_DISP_2L_OVL0) |      \
						 (1LL << CMDQ_ENG_DISP_2L_OVL1) |      \
						 (1LL << CMDQ_ENG_DISP_2L_OVL2) |      \
						 (1LL << CMDQ_ENG_DISP_GAMMA) |     \
						 (1LL << CMDQ_ENG_DISP_MERGE) |     \
						 (1LL << CMDQ_ENG_DISP_SPLIT0) |    \
						 (1LL << CMDQ_ENG_DISP_SPLIT1) |    \
						 (1LL << CMDQ_ENG_DISP_DSI0_VDO) |  \
						 (1LL << CMDQ_ENG_DISP_DSI1_VDO) |  \
						 (1LL << CMDQ_ENG_DISP_DSI0_CMD) |  \
						 (1LL << CMDQ_ENG_DISP_DSI1_CMD) |  \
						 (1LL << CMDQ_ENG_DISP_DSI0) |      \
						 (1LL << CMDQ_ENG_DISP_DSI1) |      \
						 (1LL << CMDQ_ENG_DISP_DPI))

#define CMDQ_ENG_VENC_GROUP_BITS                ((1LL << CMDQ_ENG_VIDEO_ENC))

#define CMDQ_ENG_JPEG_GROUP_BITS                ((1LL << CMDQ_ENG_JPEG_ENC) | \
						 (1LL << CMDQ_ENG_JPEG_REMDC) | \
						 (1LL << CMDQ_ENG_JPEG_DEC))

#define CMDQ_ENG_DPE_GROUP_BITS					(1LL << CMDQ_ENG_DPE)

#ifdef CMDQ_DUMP_FIRSTERROR
#define CMDQ_MAX_FIRSTERROR	(32*1024)
typedef struct DumpFirstErrorStruct {
	pid_t callerPid;
	char callerName[TASK_COMM_LEN];
	unsigned long long savetime;	/* epoch time of first error occur */
	char cmdqString[CMDQ_MAX_FIRSTERROR];
	uint32_t cmdqCount;
	int32_t cmdqMaxSize;
	bool flag;
	struct timeval savetv;
} DumpFirstErrorStruct;
#endif

#define CMDQ_LOG(string, args...) \
{			\
if (1) {	\
	pr_err("[CMDQ]"string, ##args); \
	cmdq_core_save_first_dump("[CMDQ]"string, ##args); \
}			\
}

#define CMDQ_MSG(string, args...) \
{			\
if (cmdq_core_should_print_msg()) { \
	pr_warn("[CMDQ]"string, ##args); \
}			\
}

#define CMDQ_VERBOSE(string, args...) \
{			\
if (cmdq_core_should_print_msg()) { \
	pr_debug("[CMDQ]"string, ##args); \
}			\
}


#define CMDQ_ERR(string, args...) \
{			\
if (1) {	\
	pr_err("[CMDQ][ERR]"string, ##args); \
	cmdq_core_save_first_dump("[CMDQ][ERR]"string, ##args); \
}			\
}

#define CMDQ_AEE(tag, string, args...) \
{		\
do {			\
	char dispatchedTag[50]; \
	snprintf(dispatchedTag, 50, "CRDISPATCH_KEY:%s", tag); \
	pr_err("[CMDQ][AEE]"string, ##args); \
	cmdq_core_save_first_dump("[CMDQ][AEE]"string, ##args); \
	cmdq_core_turnoff_first_dump(); \
	aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT | DB_OPT_PROC_CMDQ_INFO | DB_OPT_MMPROFILE_BUFFER, \
		dispatchedTag, "error: "string, ##args); \
} while (0);	\
}

/*#define CMDQ_PROFILE*/

typedef unsigned long long CMDQ_TIME;

#ifdef CMDQ_PROFILE
#define CMDQ_PROF_INIT()	\
{		\
do {if (0 < cmdq_core_profile_enabled()) met_tag_init(); } while (0);	\
}

#define CMDQ_PROF_START(args...)	\
{		\
do {if (0 < cmdq_core_profile_enabled()) met_tag_start(args); } while (0);	\
}

#define CMDQ_PROF_END(args...)	\
{		\
do {if (0 < cmdq_core_profile_enabled()) met_tag_end(args); } while (0);	\
}
#define CMDQ_PROF_ONESHOT(args...)	\
{		\
do {if (0 < cmdq_core_profile_enabled()) met_tag_oneshot(args); } while (0);	\
}
#else
#define CMDQ_PROF_INIT()
#define CMDQ_PROF_START(args...)
#define CMDQ_PROF_END(args...)
#define CMDQ_PROF_ONESHOT(args...)
#endif

#ifdef CMDQ_PROFILE_MMP
#define CMDQ_PROF_MMP(args...)\
{\
do {if (1) MMProfileLogEx(args); } while (0);	\
}
#else
#define CMDQ_PROF_MMP(args...)
#endif

#define CMDQ_GET_TIME_IN_MS(start, end, duration)			\
{		\
CMDQ_TIME _duration = end - start;		\
do_div(_duration, 1000000);				\
duration = (int32_t)_duration;			\
}

#define CMDQ_GET_TIME_IN_US_PART(start, end, duration)		\
{		\
CMDQ_TIME _duration = end - start;		\
do_div(_duration, 1000);				\
duration = (int32_t)_duration;			\
}

#define CMDQ_ENG_ISP_GROUP_FLAG(flag)   ((flag) & (CMDQ_ENG_ISP_GROUP_BITS))

#define CMDQ_ENG_MDP_GROUP_FLAG(flag)   ((flag) & (CMDQ_ENG_MDP_GROUP_BITS))

#define CMDQ_ENG_DISP_GROUP_FLAG(flag)  ((flag) & (CMDQ_ENG_DISP_GROUP_BITS))

#define CMDQ_ENG_JPEG_GROUP_FLAG(flag)  ((flag) & (CMDQ_ENG_JPEG_GROUP_BITS))

#define CMDQ_ENG_VENC_GROUP_FLAG(flag)	((flag) & (CMDQ_ENG_VENC_GROUP_BITS))

#define CMDQ_ENG_DPE_GROUP_FLAG(flag)	((flag) & (CMDQ_ENG_DPE_GROUP_BITS))

#define GENERATE_ENUM(_enum, _string) _enum,
#define GENERATE_STRING(_enum, _string) (#_string),

#define CMDQ_FOREACH_GROUP(ACTION_struct)\
ACTION_struct(CMDQ_GROUP_ISP, ISP)		\
ACTION_struct(CMDQ_GROUP_MDP, MDP)		\
ACTION_struct(CMDQ_GROUP_DISP, DISP)	\
ACTION_struct(CMDQ_GROUP_JPEG, JPEG)	\
ACTION_struct(CMDQ_GROUP_VENC, VENC)	\
ACTION_struct(CMDQ_GROUP_DPE, DPE)

typedef enum CMDQ_GROUP_ENUM {
	CMDQ_FOREACH_GROUP(GENERATE_ENUM)
	    CMDQ_MAX_GROUP_COUNT,	/* ALWAYS keep at the end */
} CMDQ_GROUP_ENUM;

/* engineFlag are bit fields defined in CMDQ_ENG_ENUM */
typedef int32_t(*CmdqClockOnCB) (uint64_t engineFlag);

/* engineFlag are bit fields defined in CMDQ_ENG_ENUM */
typedef int32_t(*CmdqDumpInfoCB) (uint64_t engineFlag, int level);

/* engineFlag are bit fields defined in CMDQ_ENG_ENUM */
typedef int32_t(*CmdqResetEngCB) (uint64_t engineFlag);

/* engineFlag are bit fields defined in CMDQ_ENG_ENUM */
typedef int32_t(*CmdqClockOffCB) (uint64_t engineFlag);

/* data are user data passed to APIs */
typedef int32_t(*CmdqInterruptCB) (unsigned long data);

/* data are user data passed to APIs */
typedef int32_t(*CmdqAsyncFlushCB) (unsigned long data);

/* resource event can be indicated to resource unit */
typedef int32_t(*CmdqResourceReleaseCB) (CMDQ_EVENT_ENUM resourceEvent);

/* resource event can be indicated to resource unit */
typedef int32_t(*CmdqResourceAvailableCB) (CMDQ_EVENT_ENUM resourceEvent);

/* TaskID is passed down from IOCTL */
/* client should fill "regCount" and "regAddress" */
/* the buffer pointed by (*regAddress) must be valid until */
/* CmdqDebugRegDumpEndCB() is called. */
typedef int32_t(*CmdqDebugRegDumpBeginCB) (uint32_t taskID, uint32_t *regCount,
					   uint32_t **regAddress);
typedef int32_t(*CmdqDebugRegDumpEndCB) (uint32_t taskID, uint32_t regCount, uint32_t *regValues);

typedef struct CmdqCBkStruct {
	CmdqClockOnCB clockOn;
	CmdqDumpInfoCB dumpInfo;
	CmdqResetEngCB resetEng;
	CmdqClockOffCB clockOff;
} CmdqCBkStruct;

typedef struct CmdqDebugCBkStruct {
	/* Debug Register Dump */
	CmdqDebugRegDumpBeginCB beginDebugRegDump;
	CmdqDebugRegDumpEndCB endDebugRegDump;
} CmdqDebugCBkStruct;

typedef enum CMDQ_CODE_ENUM {
	/* these are actual HW op code */
	CMDQ_CODE_READ = 0x01,
	CMDQ_CODE_MOVE = 0x02,
	CMDQ_CODE_WRITE = 0x04,
	CMDQ_CODE_POLL = 0x08,
	CMDQ_CODE_JUMP = 0x10,
	CMDQ_CODE_WFE = 0x20,	/* wait for event and clear */
	CMDQ_CODE_EOC = 0x40,	/* end of command */

	/* these are pseudo op code defined by SW */
	/* for instruction generation */
	CMDQ_CODE_SET_TOKEN = 0x21,	/* set event */
	CMDQ_CODE_WAIT_NO_CLEAR = 0x22,	/* wait event, but don't clear it */
	CMDQ_CODE_CLEAR_TOKEN = 0x23,	/* clear event */
	CMDQ_CODE_RAW = 0x24,	/* allow entirely custom argA/argB */
	CMDQ_CODE_PREFETCH_ENABLE = 0x41,	/* enable prefetch marker */
	CMDQ_CODE_PREFETCH_DISABLE = 0x42,	/* disable prefetch marker */
} CMDQ_CODE_ENUM;

typedef enum CMDQ_LOG_LEVEL_ENUM {
	CMDQ_LOG_LEVEL_NORMAL = 0,
	CMDQ_LOG_LEVEL_MSG = 1,
	CMDQ_LOG_LEVEL_FULL_ERROR = 2,
	CMDQ_LOG_LEVEL_EXTENSION = 3,

	CMDQ_LOG_LEVEL_MAX	/* ALWAYS keep at the end */
} CMDQ_LOG_LEVEL_ENUM;

typedef enum TASK_STATE_ENUM {
	TASK_STATE_IDLE,	/* free task */
	TASK_STATE_BUSY,	/* task running on a thread */
	TASK_STATE_KILLED,	/* task process being killed */
	TASK_STATE_ERROR,	/* task execution error */
	TASK_STATE_ERR_IRQ,	/* task execution invalid instruction */
	TASK_STATE_DONE,	/* task finished */
	TASK_STATE_WAITING,	/* allocated but waiting for available thread */
} TASK_STATE_ENUM;

#ifdef CMDQ_INSTRUCTION_COUNT
/* GCE instructions count information */
typedef enum CMDQ_STAT_ENUM {
	CMDQ_STAT_WRITE = 0,
	CMDQ_STAT_WRITE_W_MASK = 1,
	CMDQ_STAT_READ = 2,
	CMDQ_STAT_POLLING = 3,
	CMDQ_STAT_MOVE = 4,
	CMDQ_STAT_SYNC = 5,
	CMDQ_STAT_PREFETCH_EN = 6,
	CMDQ_STAT_PREFETCH_DIS = 7,
	CMDQ_STAT_EOC = 8,
	CMDQ_STAT_JUMP = 9,

	CMDQ_STAT_MAX		/* ALWAYS keep at the end */
} CMDQ_STAT_ENUM;

typedef enum CMDQ_MODULE_STAT_ENUM {
	CMDQ_MODULE_STAT_MMSYS_CONFIG = 0,
	CMDQ_MODULE_STAT_MDP_RDMA = 1,
	CMDQ_MODULE_STAT_MDP_RSZ0 = 2,
	CMDQ_MODULE_STAT_MDP_RSZ1 = 3,
	CMDQ_MODULE_STAT_MDP_WDMA = 4,
	CMDQ_MODULE_STAT_MDP_WROT = 5,
	CMDQ_MODULE_STAT_MDP_TDSHP = 6,
	CMDQ_MODULE_STAT_MM_MUTEX = 7,
	CMDQ_MODULE_STAT_VENC = 8,
	CMDQ_MODULE_STAT_DISP_OVL0 = 9,
	CMDQ_MODULE_STAT_DISP_OVL1 = 10,
	CMDQ_MODULE_STAT_DISP_RDMA0 = 11,
	CMDQ_MODULE_STAT_DISP_RDMA1 = 12,
	CMDQ_MODULE_STAT_DISP_WDMA0 = 13,
	CMDQ_MODULE_STAT_DISP_COLOR = 14,
	CMDQ_MODULE_STAT_DISP_CCORR = 15,
	CMDQ_MODULE_STAT_DISP_AAL = 16,
	CMDQ_MODULE_STAT_DISP_GAMMA = 17,
	CMDQ_MODULE_STAT_DISP_DITHER = 18,
	CMDQ_MODULE_STAT_DISP_UFOE = 19,
	CMDQ_MODULE_STAT_DISP_PWM = 20,
	CMDQ_MODULE_STAT_DISP_WDMA1 = 21,
	CMDQ_MODULE_STAT_DISP_MUTEX = 22,
	CMDQ_MODULE_STAT_DISP_DSI0 = 23,
	CMDQ_MODULE_STAT_DISP_DPI0 = 24,
	CMDQ_MODULE_STAT_DISP_OD = 25,
	CMDQ_MODULE_STAT_CAM0 = 26,
	CMDQ_MODULE_STAT_CAM1 = 27,
	CMDQ_MODULE_STAT_CAM2 = 28,
	CMDQ_MODULE_STAT_CAM3 = 29,
	CMDQ_MODULE_STAT_SODI = 30,
	CMDQ_MODULE_STAT_GPR = 31,
	CMDQ_MODULE_STAT_OTHERS = 32,

	CMDQ_MODULE_STAT_MAX	/* ALWAYS keep at the end */
} CMDQ_MODULE_STAT_ENUM;

typedef enum CMDQ_EVENT_STAT_ENUM {
	CMDQ_EVENT_STAT_HW = 0,
	CMDQ_EVENT_STAT_SW = 1,

	CMDQ_EVENT_STAT_MAX	/* ALWAYS keep at the end */
} CMDQ_EVENT_STAT_ENUM;

#define CMDQ_MAX_OTHERINSTRUCTION_MAX		(16)

typedef struct CmdqModulePAStatStruct {
	long start[CMDQ_MODULE_STAT_MAX];
	long end[CMDQ_MODULE_STAT_MAX];
} CmdqModulePAStatStruct;
#endif

typedef struct TaskStruct {
	struct list_head listEntry;

	/* For buffer state */
	TASK_STATE_ENUM taskState;	/* task life cycle */
	uint32_t *pVABase;	/* virtual address of command buffer */
	dma_addr_t MVABase;	/* physical address of command buffer */
	uint32_t bufferSize;	/* size of allocated command buffer */
	bool useEmergencyBuf;	/* is the command buffer emergency buffer? */

	/* For execution */
	int32_t scenario;
	int32_t priority;
	uint64_t engineFlag;
	int32_t commandSize;
	uint32_t *pCMDEnd;
	int32_t reorder;
	int32_t thread;		/* ASYNC: CMDQ_INVALID_THREAD if not running */
	int32_t irqFlag;	/* ASYNC: flag of IRQ received */
	CmdqInterruptCB loopCallback;	/* LOOP execution */
	unsigned long loopData;	/* LOOP execution */
	CmdqAsyncFlushCB flushCallback;	/* Callback on AsyncFlush (fire-and-forget) tasks */
	unsigned long flushData;	/* for callbacks & error handling */
	struct work_struct autoReleaseWork;	/* Work item when auto release is used */
	bool useWorkQueue;

	/* Output section for "read from reg to mem" */
	uint32_t regCount;
	uint32_t *regResults;
	dma_addr_t regResultsMVA;

	/* For register backup */
	uint32_t regCountUserSpace;	/* this is to separate backup request from user space and kernel space. */
	uint32_t regUserToken;	/* user data store for callback beginDebugRegDump / endDebugRegDump */

	/* For seucre execution */
	cmdqSecDataStruct secData;

	/* For statistics & debug */
	CMDQ_TIME submit;	/* ASYNC: task submit time (as soon as task acquired) */
	CMDQ_TIME trigger;
	CMDQ_TIME beginWait;
	CMDQ_TIME gotIRQ;
	CMDQ_TIME wakedUp;
	CMDQ_TIME entrySec;	/* time stamp of entry secure world */
	CMDQ_TIME exitSec;	/* time stamp of exit secure world */

	uint32_t *profileData;	/* store GPT counter when it starts and ends */
	dma_addr_t profileDataPA;

	void *privateData;	/* this is used to track associated file handle */

	pid_t callerPid;
	char callerName[TASK_COMM_LEN];

	/* Custom profile marker */
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	cmdqProfileMarkerStruct profileMarker;
#endif
} TaskStruct;

typedef struct EngineStruct {
	int32_t userCount;
	int32_t currOwner;
	int32_t resetCount;
	int32_t failCount;
} EngineStruct;

typedef struct ThreadStruct {
	uint32_t taskCount;
	uint32_t waitCookie;
	uint32_t nextCookie;
	uint64_t engineFlag;	/* keep used engine to look up while dispatch thread */
	CmdqInterruptCB loopCallback;	/* LOOP execution */
	unsigned long loopData;	/* LOOP execution */
	TaskStruct *pCurTask[CMDQ_MAX_TASK_IN_THREAD];

	/* 1 to describe thread is available to dispatch a task. 0: not available */
	/* .note thread's taskCount increase when attatch a task to it. */
	/* used it to prevent 2 tasks, which uses different engines, */
	/* acquire same HW thread when dispatching happened before attaches task to thread */
	/* .note it is align task attachment, so use cmdqExecLock to ensure atomic access */
	uint32_t allowDispatching;
} ThreadStruct;

typedef struct RecordStruct {
	pid_t user;		/* calling SW thread tid */
	int32_t scenario;	/* task scenario */
	int32_t priority;	/* task priority (not thread priority) */
	int32_t thread;		/* allocated thread */
	int32_t reorder;
	int32_t size;
	uint32_t writeTimeNS;	/* if profile enabled, the time of command execution */
	uint64_t engineFlag;	/* task engine flag */

	bool isSecure;		/* true for secure task */

	CMDQ_TIME submit;	/* epoch time of IOCTL/Kernel API call */
	CMDQ_TIME trigger;	/* epoch time of enable HW thread */
	CMDQ_TIME beginWait;	/* epoch time of start waiting for task completion */
	CMDQ_TIME gotIRQ;	/* epoch time of IRQ event */
	CMDQ_TIME wakedUp;	/* epoch time of SW thread leaving wait state */
	CMDQ_TIME done;		/* epoch time of task finish */

	unsigned long long writeTimeNSBegin;
	unsigned long long writeTimeNSEnd;

	/* Custom profile marker */
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	uint32_t profileMarkerCount;
	unsigned long long profileMarkerTimeNS[CMDQ_MAX_PROFILE_MARKER_IN_TASK];
	const char *profileMarkerTag[CMDQ_MAX_PROFILE_MARKER_IN_TASK];
#endif

	/* GCE instructions count information */
#ifdef CMDQ_INSTRUCTION_COUNT
	unsigned short instructionStat[CMDQ_STAT_MAX];
	unsigned short writeModule[CMDQ_MODULE_STAT_MAX];
	unsigned short writewmaskModule[CMDQ_MODULE_STAT_MAX];
	unsigned short readModlule[CMDQ_MODULE_STAT_MAX];
	unsigned short pollModule[CMDQ_MODULE_STAT_MAX];
	unsigned short eventCount[CMDQ_EVENT_STAT_MAX];
	uint32_t otherInstr[CMDQ_MAX_OTHERINSTRUCTION_MAX];
	uint32_t otherInstrNUM;
#endif
} RecordStruct;

typedef struct ErrorStruct {
	RecordStruct errorRec;	/* the record of the error task */
	u64 ts_nsec;		/* kernel time of attach_error_task */
} ErrorStruct;

typedef struct WriteAddrStruct {
	struct list_head list_node;
	uint32_t count;
	void *va;
	dma_addr_t pa;
	pid_t user;
} WriteAddrStruct;

typedef struct EmergencyBufferStruct {
	bool used;
	uint32_t size;
	void *va;
	dma_addr_t pa;
} EmergencyBufferStruct;

/**
 * shared memory between normal and secure world
 */
typedef struct cmdqSecSharedMemoryStruct {
	void *pVABase;		/* virtual address of command buffer */
	dma_addr_t MVABase;	/* physical address of command buffer */
	uint32_t size;		/* buffer size */
} cmdqSecSharedMemoryStruct, *cmdqSecSharedMemoryHandle;

/**
 * resource unit between each module
 */
typedef struct ResourceUnitStruct {
	struct list_head listEntry;
	CMDQ_TIME notify;			/* notify time from module prepare */
	CMDQ_TIME lock;			/* lock time from module lock */
	CMDQ_TIME unlock;			/* unlock time from module unlock*/
	CMDQ_TIME delay;			/* delay start time from module release*/
	CMDQ_TIME acquire;		/* acquire time from module acquire */
	CMDQ_TIME release;		/* release time from module release */
	bool used;				/* indicate resource is in use or not */
	bool delaying;			/* indicate resource is in delay check or not */
	CMDQ_EVENT_ENUM lockEvent;	/* SW token to lock in GCE thread */
	uint64_t engine;			/* which engine is resource */
	CmdqResourceAvailableCB availableCB;
	CmdqResourceReleaseCB releaseCB;
	struct delayed_work delayCheckWork;	/* Delay Work item when delay check is used */
} ResourceUnitStruct;

typedef struct ContextStruct {
	/* Task information */
	struct kmem_cache *taskCache;	/* TaskStruct object cache */
	struct list_head taskFreeList;	/* Unused free tasks */
	struct list_head taskActiveList;	/* Active tasks */
	struct list_head taskWaitList;	/* Tasks waiting for available thread */
	struct work_struct taskConsumeWaitQueueItem;
	struct workqueue_struct *taskAutoReleaseWQ;	/* auto-release workqueue */
	struct workqueue_struct *taskConsumeWQ;	/* task consumption workqueue (for queued tasks) */
	struct workqueue_struct *resourceCheckWQ;	/* delay resource check workqueue */

	/* Write Address management */
	struct list_head writeAddrList;

	/* Basic information */
	EngineStruct engine[CMDQ_MAX_ENGINE_COUNT];
	ThreadStruct thread[CMDQ_MAX_THREAD_COUNT];

	/* auto-release workqueue per thread */
	struct workqueue_struct *taskThreadAutoReleaseWQ[CMDQ_MAX_THREAD_COUNT];

	/* Secure path shared information */
	cmdqSecSharedMemoryHandle hSecSharedMem;
	void *hNotifyLoop;

	/* Profile information */
	int32_t enableProfile;
	int32_t lastID;
	int32_t recNum;
	RecordStruct record[CMDQ_MAX_RECORD_COUNT];

	/* Error information */
	int32_t logLevel;
	int32_t errNum;
	ErrorStruct error[CMDQ_MAX_ERROR_COUNT];

	/* Resource manager information */
	struct list_head resourceList;	/* all resource list */

#ifdef CMDQ_INSTRUCTION_COUNT
	/* GCE instructions count information */
	int32_t instructionCountLevel;
#endif
} ContextStruct;

/* Command dump information */
typedef struct DumpCommandBufferStruct {
	uint64_t scenario;
	uint32_t bufferSize;
	uint32_t count;
	char *cmdqString;
} DumpCommandBufferStruct;

#ifdef __cplusplus
extern "C" {
#endif
	void cmdqCoreInitGroupCB(void);
	void cmdqCoreDeinitGroupCB(void);

	int32_t cmdqCoreRegisterCB(CMDQ_GROUP_ENUM engGroup,
				   CmdqClockOnCB clockOn,
				   CmdqDumpInfoCB dumpInfo,
				   CmdqResetEngCB resetEng, CmdqClockOffCB clockOff);

	int32_t cmdqCoreRegisterDebugRegDumpCB(CmdqDebugRegDumpBeginCB beginCB,
					       CmdqDebugRegDumpEndCB endCB);

	int32_t cmdqCoreSuspend(void);

	int32_t cmdqCoreResume(void);

	int32_t cmdqCoreResumedNotifier(void);

	void cmdqCoreHandleIRQ(int32_t index);

/**
 * Wait for completion of the given CmdQ task
 *
 * Parameter:
 *      scenario: The Scnerio enumeration
 *      priority: Desied task priority
 *      engineFlag: HW engine involved in the CMDs
 *      pCMDBlock: The command buffer
 *      blockSize: Size of the command buffer
 *      ppTaskOut: output pointer to a pTask for the resulting task
 *                 if fail, this gives NULL
 *      loopCB:    Assign this CB if your command loops itself.
 *                 since this disables thread completion handling, do not set this CB
 *                 if it is not intended to be a HW looping thread.
 *      loopData:  The user data passed to loopCB
 *
 * Return:
 *      >=0 for success; else the error code is returned
 */
	int32_t cmdqCoreSubmitTaskAsync(cmdqCommandStruct *pCommandDesc,
					CmdqInterruptCB loopCB,
					unsigned long loopData, TaskStruct **ppTaskOut);

/**
 * Wait for completion of the given CmdQ task
 *
 * Parameter:
 *      pTask: Task returned from successful cmdqCoreSubmitTaskAsync
 *             additional cleanup will be performed.
 *
 *      timeout: SW timeout error will be generated after this threshold
 * Return:
 *      >=0 for success; else the error code is returned
 */
	int32_t cmdqCoreWaitAndReleaseTask(TaskStruct *pTask, long timeout_jiffies);

/**
 * Wait for completion of the given CmdQ task, and retrieve
 * read register result.
 *
 * Parameter:
 *      pTask: Task returned from successful cmdqCoreSubmitTaskAsync
 *             additional cleanup will be performed.
 *
 *      timeout: SW timeout error will be generated after this threshold
 * Return:
 *      >=0 for success; else the error code is returned
 */
	int32_t cmdqCoreWaitResultAndReleaseTask(TaskStruct *pTask, cmdqRegValueStruct *pResult,
						 long timeout_jiffies);

/**
 * Stop task and release it immediately
 *
 * Parameter:
 *      pTask: Task returned from successful cmdqCoreSubmitTaskAsync
 *             additional cleanup will be performed.
 *
 * Return:
 *      >=0 for success; else the error code is returned
 */
	int32_t cmdqCoreReleaseTask(TaskStruct *pTask);

/**
 * Register the task in the auto-release queue. It will be released
 * upon finishing. You MUST NOT perform further operations on this task.
 *
 * Parameter:
 *      pTask: Task returned from successful cmdqCoreSubmitTaskAsync.
 *             additional cleanup will be performed.
 * Return:
 *      >=0 for success; else the error code is returned
 */
	int32_t cmdqCoreAutoReleaseTask(TaskStruct *pTask);

/**
 * Create CMDQ Task and block wait for its completion
 *
 * Return:
 *     >=0 for success; else the error code is returned
 */
	int32_t cmdqCoreSubmitTask(cmdqCommandStruct *pCommandDesc);


/**
 * Helper function checking validity of a task pointer
 *
 * Return:
 *     false if NOT a valid pointer
 *     true if valid
 */
	bool cmdqIsValidTaskPtr(void *pTask);

/**
 * Immediately clear CMDQ event to 0 with CPU
 *
 */

	void cmdqCoreClearEvent(CMDQ_EVENT_ENUM event);

/**
 * Immediately set CMDQ event to 1 with CPU
 *
 */
	void cmdqCoreSetEvent(CMDQ_EVENT_ENUM event);

/**
 * Get event value with CPU. This is for debug purpose only
 * since it does not guarantee atomicity.
 *
 */
	uint32_t cmdqCoreGetEvent(CMDQ_EVENT_ENUM event);

	int32_t cmdqCoreInitialize(void);
#ifdef CMDQ_SECURE_PATH_SUPPORT
	int32_t cmdqCoreLateInitialize(void);
#endif
	void cmdqCoreDeInitialize(void);

/**
 * Allocate/Free HW use buffer, e.g. command buffer forCMDQ HW
 */
	void *cmdq_core_alloc_hw_buffer(struct device *dev, size_t size, dma_addr_t *dma_handle,
					const gfp_t flag);
	void cmdq_core_free_hw_buffer(struct device *dev, size_t size, void *cpu_addr,
				      dma_addr_t dma_handle);

	cmdqSecSharedMemoryHandle cmdq_core_get_secure_shared_memory(void);

/*
 * GCE capability
 */
	uint32_t cmdq_core_subsys_to_reg_addr(uint32_t argA);
	const char *cmdq_core_parse_subsys_from_reg_addr(uint32_t reg_addr);
	int32_t cmdq_core_subsys_from_phys_addr(uint32_t physAddr);
	int32_t cmdq_core_suspend_HW_thread(int32_t thread, uint32_t lineNum);

/**
 * Event
 */
	void cmdq_core_reset_hw_events(void);

/**
 * Get and HW information from device tree
 */
	void cmdq_core_init_DTS_data(void);
	cmdqDTSDataStruct *cmdq_core_get_whole_DTS_Data(void);

/**
 * Get and set HW event form device tree
 */
	void cmdq_core_set_event_table(CMDQ_EVENT_ENUM event, const int32_t value);
	const int32_t cmdq_core_get_event_value(CMDQ_EVENT_ENUM event);
	const char *cmdq_core_get_event_name_ENUM(CMDQ_EVENT_ENUM event);
	const char *cmdq_core_get_event_name(CMDQ_EVENT_ENUM event);

/**
 * Utilities
 */
	void cmdq_core_set_log_level(const int32_t value);
	ssize_t cmdqCorePrintLogLevel(struct device *dev, struct device_attribute *attr, char *buf);
	ssize_t cmdqCoreWriteLogLevel(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t size);

	ssize_t cmdqCorePrintProfileEnable(struct device *dev, struct device_attribute *attr,
					   char *buf);
	ssize_t cmdqCoreWriteProfileEnable(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size);

	void cmdq_core_dump_tasks_info(void);
	void cmdq_core_dump_secure_metadata(cmdqSecDataStruct *pSecData);
	int32_t cmdqCoreDebugRegDumpBegin(uint32_t taskID, uint32_t *regCount,
					  uint32_t **regAddress);
	int32_t cmdqCoreDebugRegDumpEnd(uint32_t taskID, uint32_t regCount, uint32_t *regValues);
	int32_t cmdqCoreDebugDumpCommand(TaskStruct *pTask);
	int32_t cmdqCoreQueryUsage(int32_t *pCount);

	int cmdqCorePrintRecordSeq(struct seq_file *m, void *v);
	int cmdqCorePrintErrorSeq(struct seq_file *m, void *v);
	int cmdqCorePrintStatusSeq(struct seq_file *m, void *v);

	ssize_t cmdqCorePrintRecord(struct device *dev, struct device_attribute *attr, char *buf);
	ssize_t cmdqCorePrintError(struct device *dev, struct device_attribute *attr, char *buf);
	ssize_t cmdqCorePrintStatus(struct device *dev, struct device_attribute *attr, char *buf);

	void cmdq_core_fix_command_scenario_for_user_space(cmdqCommandStruct *pCommand);
	bool cmdq_core_is_request_from_user_space(const CMDQ_SCENARIO_ENUM scenario);

	unsigned long long cmdq_core_get_GPR64(const CMDQ_DATA_REGISTER_ENUM regID);
	void cmdq_core_set_GPR64(const CMDQ_DATA_REGISTER_ENUM regID,
				 const unsigned long long value);

	uint32_t cmdqCoreReadDataRegister(CMDQ_DATA_REGISTER_ENUM regID);

	int cmdqCoreAllocWriteAddress(uint32_t count, dma_addr_t *paStart);
	int cmdqCoreFreeWriteAddress(dma_addr_t paStart);
	uint32_t cmdqCoreReadWriteAddress(dma_addr_t pa);
	uint32_t cmdqCoreWriteWriteAddress(dma_addr_t pa, uint32_t value);

	int32_t cmdq_core_profile_enabled(void);

	bool cmdq_core_should_print_msg(void);
	bool cmdq_core_should_full_error(void);

	int32_t cmdq_core_enable_emergency_buffer_test(const bool enable);

	int32_t cmdq_core_parse_instruction(const uint32_t *pCmd, char *textBuf, int bufLen);

	void cmdq_core_add_consume_task(void);

/* file_node is a pointer to cmdqFileNodeStruct that is */
/* created when opening the device file. */
	void cmdq_core_release_task_by_file_node(void *file_node);

	void cmdq_core_longstring_init(char *buf, uint32_t *offset, int32_t *maxSize);
	void cmdqCoreLongString(bool forceLog, char *buf, uint32_t *offset, int32_t *maxSize,
				const char *string, ...);

	/* Command Buffer Dump */
	void cmdq_core_set_command_buffer_dump(int32_t scenario, int32_t bufferSize);

	/* test case initialization */
	void cmdq_test_init_setting(void);

#ifdef CMDQ_INSTRUCTION_COUNT
	CmdqModulePAStatStruct *cmdq_core_Initial_and_get_module_stat(void);
	ssize_t cmdqCorePrintInstructionCountLevel(struct device *dev,
						   struct device_attribute *attr, char *buf);
	ssize_t cmdqCoreWriteInstructionCountLevel(struct device *dev,
						   struct device_attribute *attr, const char *buf,
						   size_t size);
	void cmdq_core_set_instruction_count_level(const int32_t value);
	int cmdqCorePrintInstructionCountSeq(struct seq_file *m, void *v);
#endif				/* CMDQ_INSTRUCTION_COUNT */

/**
 * Save first error dump
 */
	void cmdq_core_turnon_first_dump(const TaskStruct *pTask);
	void cmdq_core_turnoff_first_dump(void);
/**
 * cmdq_core_save_first_dump - save a CMDQ first error dump to file
 */
	int32_t cmdq_core_save_first_dump(const char *string, ...);
/**
 * cmdq_core_save_hex_first_dump - save a CMDQ first error hex dump to file
 * @prefix_str: string to prefix each line with;
 *	caller supplies trailing spaces for alignment if desired
 * @rowsize: number of bytes to print per line; must be 16 or 32
 * @groupsize: number of bytes to print at a time (1, 2, 4, 8; default = 1)
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 *
 * Given a buffer of u8 data, cmdq_core_save_hex_first_dump() save a CMDQ first error hex dump
 * to the file , with an optional leading prefix.
 *
 * cmdq_core_save_hex_first_dump() works on one "line" of output at a time, i.e.,
 * 16 or 32 bytes of input data converted to hex.
 * cmdq_core_save_hex_first_dump() iterates over the entire input @buf, breaking it into
 * "line size" chunks to format and print.
 *
 * E.g.:
 *	 cmdq_core_save_hex_first_dump("", 16, 4,
 *			       pTask->pVABase, (pTask->commandSize));
 *
 * Example output using 4-byte mode:
 * ed7e4510: ff7fffff 02000000 00800000 0804401d
 * ed7e4520: fffffffe 02000000 00000001 04044001
 * ed7e4530: 80008001 20000043
*/
	void cmdq_core_save_hex_first_dump(const char *prefix_str,
					   int rowsize, int groupsize, const void *buf, size_t len);

	void cmdqCoreLockResource(uint64_t engineFlag, bool fromNotify);
	bool cmdqCoreAcquireResource(CMDQ_EVENT_ENUM resourceEvent);
	void cmdqCoreReleaseResource(CMDQ_EVENT_ENUM resourceEvent);
	void cmdqCoreSetResourceCallback(CMDQ_EVENT_ENUM resourceEvent,
								CmdqResourceAvailableCB resourceAvailable,
								CmdqResourceReleaseCB resourceRelease);


#ifdef __cplusplus
}
#endif
#endif				/* __CMDQ_CORE_H__ */
