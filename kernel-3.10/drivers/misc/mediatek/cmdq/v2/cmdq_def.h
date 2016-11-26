#ifndef __CMDQ_DEF_H__
#define __CMDQ_DEF_H__

#include <linux/kernel.h>

#define CMDQ_DRIVER_DEVICE_NAME         "mtk_cmdq"

/* #define CMDQ_COMMON_ENG_SUPPORT */
#ifdef CMDQ_COMMON_ENG_SUPPORT
#include "cmdq_engine_common.h"
#else
#include "cmdq_engine.h"
#endif

#define CMDQ_SPECIAL_SUBSYS_ADDR (99)

#define CMDQ_GPR_SUPPORT
#define CMDQ_PROFILE_MARKER_SUPPORT

#ifdef CMDQ_PROFILE_MARKER_SUPPORT
#define CMDQ_MAX_PROFILE_MARKER_IN_TASK (5)
#endif

#define CMDQ_INVALID_THREAD             (-1)

#define CMDQ_MAX_THREAD_COUNT           (16)
#define CMDQ_MAX_TASK_IN_THREAD         (16)
#define CMDQ_MAX_READ_SLOT_COUNT        (4)
#define CMDQ_INIT_FREE_TASK_COUNT       (8)

#define CMDQ_MAX_HIGH_PRIORITY_THREAD_COUNT (7)	/* Thread that are high-priority (display threads) */
#define CMDQ_MIN_SECURE_THREAD_ID		(12)
#define CMDQ_MAX_SECURE_THREAD_COUNT	(3)

#define CMDQ_MAX_ERROR_COUNT            (2)
#define CMDQ_MAX_RETRY_COUNT            (1)
/* ram optimization related configuration */
#ifdef CONFIG_MTK_GMO_RAM_OPTIMIZE
#define CMDQ_MAX_RECORD_COUNT           (100)
#else
#define CMDQ_MAX_RECORD_COUNT           (1024)
#endif

#define CMDQ_MAX_PREFETCH_INSTUCTION    (240)	/* Maximum prefetch buffer size, in instructions. */
#define CMDQ_INITIAL_CMD_BLOCK_SIZE     (PAGE_SIZE)
#define CMDQ_EMERGENCY_BLOCK_SIZE       (256 * 1024)	/* 256 KB command buffer */
#define CMDQ_EMERGENCY_BLOCK_COUNT      (4)
#define CMDQ_INST_SIZE                  (2 * sizeof(uint32_t))	/* instruction is 64-bit */

#define CMDQ_MAX_LOOP_COUNT             (1000000)
#define CMDQ_MAX_INST_CYCLE             (27)
#define CMDQ_MIN_AGE_VALUE              (5)
#define CMDQ_MAX_ERROR_SIZE             (8 * 1024)

#define CMDQ_MAX_TASK_IN_SECURE_THREAD	(10)

/* max value of CMDQ_THR_EXEC_CMD_CNT (value starts from 0) */
#ifdef CMDQ_USE_LEGACY
#define CMDQ_MAX_COOKIE_VALUE           (0xFFFF)
#else
#define CMDQ_MAX_COOKIE_VALUE           (0xFFFFFFFF)
#endif
#define CMDQ_ARG_A_SUBSYS_MASK          (0x001F0000)

#ifdef CONFIG_MTK_FPGA
#define CMDQ_DEFAULT_TIMEOUT_MS         (10000)
#else
#define CMDQ_DEFAULT_TIMEOUT_MS         (1000)
#endif

#define CMDQ_ACQUIRE_THREAD_TIMEOUT_MS  (2000)
#define CMDQ_PREDUMP_TIMEOUT_MS         (200)
#define CMDQ_PREDUMP_RETRY_COUNT        (5)

#ifdef CONFIG_OF
#define CMDQ_OF_SUPPORT		/* enable device tree support */
#else
#undef  CMDQ_OF_SUPPORT
#endif

#ifndef CONFIG_MTK_FPGA
#define CMDQ_PWR_AWARE		/* FPGA does not have ClkMgr */
#else
#undef CMDQ_PWR_AWARE
#endif

#ifdef CMDQ_SECURE_PATH_HW_LOCK
#undef CMDQ_SECURE_PATH_NORMAL_IRQ
#endif

#define CMDQ_DUMP_GIC (1)
/* #define CMDQ_PROFILE_MMP (1) */

#define CMDQ_DUMP_FIRSTERROR
/* #define CMDQ_INSTRUCTION_COUNT */

typedef enum CMDQ_HW_THREAD_PRIORITY_ENUM {
	CMDQ_THR_PRIO_SUPERLOW = 0,	/* low priority monitor loop */

	CMDQ_THR_PRIO_NORMAL = 1,	/* nomral priority */
	CMDQ_THR_PRIO_DISPLAY_TRIGGER = 2,	/* trigger loop (enables display mutex) */

	/* display ESD check (every 2 secs) */
#ifdef CMDQ_SPECIAL_ESD_PRIORITY
	CMDQ_THR_PRIO_DISPLAY_ESD = 3,
#else
	CMDQ_THR_PRIO_DISPLAY_ESD = 4,
#endif

	CMDQ_THR_PRIO_DISPLAY_CONFIG = 4,	/* display config (every frame) */

	CMDQ_THR_PRIO_SUPERHIGH = 5,	/* High priority monitor loop */

	CMDQ_THR_PRIO_MAX = 7,	/* maximum possible priority */
} CMDQ_HW_THREAD_PRIORITY_ENUM;

typedef enum CMDQ_SCENARIO_ENUM {
	CMDQ_SCENARIO_JPEG_DEC = 0,
	CMDQ_SCENARIO_PRIMARY_DISP = 1,
	CMDQ_SCENARIO_PRIMARY_MEMOUT = 2,
	CMDQ_SCENARIO_PRIMARY_ALL = 3,
	CMDQ_SCENARIO_SUB_DISP = 4,
	CMDQ_SCENARIO_SUB_MEMOUT = 5,
	CMDQ_SCENARIO_SUB_ALL = 6,
	CMDQ_SCENARIO_MHL_DISP = 7,
	CMDQ_SCENARIO_RDMA0_DISP = 8,
	CMDQ_SCENARIO_RDMA0_COLOR0_DISP = 9,
	CMDQ_SCENARIO_RDMA1_DISP = 10,

	/* Trigger loop scenario does not enable HWs */
	CMDQ_SCENARIO_TRIGGER_LOOP = 11,

	/* client from user space, so the cmd buffer is in user space. */
	CMDQ_SCENARIO_USER_MDP = 12,

	CMDQ_SCENARIO_DEBUG = 13,
	CMDQ_SCENARIO_DEBUG_PREFETCH = 14,

	/* ESD check */
	CMDQ_SCENARIO_DISP_ESD_CHECK = 15,
	/* for screen capture to wait for RDMA-done without blocking config thread */
	CMDQ_SCENARIO_DISP_SCREEN_CAPTURE = 16,

	/* notifiy there are some tasks exec done in secure path */
	CMDQ_SCENARIO_SECURE_NOTIFY_LOOP = 17,

	CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH = 18,
	CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH = 19,

	/* color path request from kernel */
	CMDQ_SCENARIO_DISP_COLOR = 20,
	/* color path request from user sapce */
	CMDQ_SCENARIO_USER_DISP_COLOR = 21,

	/* [phased out]client from user space, so the cmd buffer is in user space. */
	CMDQ_SCENARIO_USER_SPACE = 22,

	CMDQ_SCENARIO_DISP_MIRROR_MODE = 23,

	CMDQ_SCENARIO_DISP_CONFIG_AAL = 24,
	CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_GAMMA = 25,
	CMDQ_SCENARIO_DISP_CONFIG_SUB_GAMMA = 26,
	CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_DITHER = 27,
	CMDQ_SCENARIO_DISP_CONFIG_SUB_DITHER = 28,
	CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PWM = 29,
	CMDQ_SCENARIO_DISP_CONFIG_SUB_PWM = 30,
	CMDQ_SCENARIO_DISP_CONFIG_PRIMARY_PQ = 31,
	CMDQ_SCENARIO_DISP_CONFIG_SUB_PQ = 32,
	CMDQ_SCENARIO_DISP_CONFIG_OD = 33,

	CMDQ_SCENARIO_RDMA2_DISP = 34,

	CMDQ_SCENARIO_HIGHP_TRIGGER_LOOP = 35,	/* for primary trigger loop enable pre-fetch usage */
	CMDQ_SCENARIO_LOWP_TRIGGER_LOOP = 36,	/* for low priority monitor loop to polling bus status*/

	CMDQ_SCENARIO_KERNEL_CONFIG_GENERAL = 37,

	CMDQ_MAX_SCENARIO_COUNT	/* ALWAYS keep at the end */
} CMDQ_SCENARIO_ENUM;

typedef enum CMDQ_DATA_REGISTER_ENUM {
	/* Value Reg, we use 32-bit */
	/* Address Reg, we use 64-bit */
	/* Note that R0-R15 and P0-P7 actullay share same memory */
	/* and R1 cannot be used. */

	CMDQ_DATA_REG_JPEG = 0x00,	/* R0 */
	CMDQ_DATA_REG_JPEG_DST = 0x11,	/* P1 */

	CMDQ_DATA_REG_PQ_COLOR = 0x04,	/* R4 */
	CMDQ_DATA_REG_PQ_COLOR_DST = 0x13,	/* P3 */

	CMDQ_DATA_REG_2D_SHARPNESS_0 = 0x05,	/* R5 */
	CMDQ_DATA_REG_2D_SHARPNESS_0_DST = 0x14,	/* P4 */

	CMDQ_DATA_REG_2D_SHARPNESS_1 = 0x0a,	/* R10 */
	CMDQ_DATA_REG_2D_SHARPNESS_1_DST = 0x16,	/* P6 */

	CMDQ_DATA_REG_DEBUG = 0x0b,	/* R11 */
	CMDQ_DATA_REG_DEBUG_DST = 0x17,	/* P7 */

	/* sentinel value for invalid register ID */
	CMDQ_DATA_REG_INVALID = -1,
} CMDQ_DATA_REGISTER_ENUM;

typedef enum CMDQ_MDP_PA_BASE_ENUM {
	CMDQ_MDP_PA_BASE_MM_MUTEX,
	CMDQ_MAX_MDP_PA_BASE_COUNT,		/* ALWAYS keep at the end */
} CMDQ_MDP_PA_BASE_ENUM;

/* CMDQ Events */
#undef DECLARE_CMDQ_EVENT
#define DECLARE_CMDQ_EVENT(name_struct, val, dts_name) name_struct = val,
typedef enum CMDQ_EVENT_ENUM {
#include "cmdq_event_common.h"
} CMDQ_EVENT_ENUM;
#undef DECLARE_CMDQ_EVENT

/* CMDQ subsys */
#undef DECLARE_CMDQ_SUBSYS
#define DECLARE_CMDQ_SUBSYS(name_struct, val, grp, dts_name) name_struct = val,
typedef enum CMDQ_SUBSYS_ENUM {
#include "cmdq_subsys_common.h"

	/* ALWAYS keep at the end */
	CMDQ_SUBSYS_MAX_COUNT
} CMDQ_SUBSYS_ENUM;
#undef DECLARE_CMDQ_SUBSYS

#define CMDQ_SUBSYS_GRPNAME_MAX		(30)
/* GCE subsys information */
typedef struct SubsysStruct {
	uint32_t msb;
	int32_t subsysID;
	uint32_t mask;
	char grpName[CMDQ_SUBSYS_GRPNAME_MAX];
} SubsysStruct;

typedef struct cmdqDTSDataStruct {
	/* [Out] GCE event table */
	int32_t eventTable[CMDQ_SYNC_TOKEN_MAX];
	/* [Out] GCE subsys ID table */
	SubsysStruct subsys[CMDQ_SUBSYS_MAX_COUNT];
	/* [Out] MDP Base address */
	uint32_t MDPBaseAddress[CMDQ_MAX_MDP_PA_BASE_COUNT];
} cmdqDTSDataStruct;

/* Custom "wide" pointer type for 64-bit job handle (pointer to VA) */
typedef unsigned long long cmdqJobHandle_t;
/* Custom "wide" pointer type for 64-bit compatibility. Always cast from uint32_t*. */
typedef unsigned long long cmdqU32Ptr_t;

#define CMDQ_U32_PTR(x) ((uint32_t *)(unsigned long)x)

typedef struct cmdqReadRegStruct {
	uint32_t count;		/* number of entries in regAddresses */
	cmdqU32Ptr_t regAddresses;	/* an array of 32-bit register addresses (uint32_t) */
} cmdqReadRegStruct;

typedef struct cmdqRegValueStruct {
	/* number of entries in result */
	uint32_t count;

	/* array of 32-bit register values (uint32_t). */
	/* in the same order as cmdqReadRegStruct */
	cmdqU32Ptr_t regValues;
} cmdqRegValueStruct;

typedef struct cmdqReadAddressStruct {
	uint32_t count;		/* [IN] number of entries in result. */

	/* [IN] array of physical addresses to read. */
	/* these value must allocated by CMDQ_IOCTL_ALLOC_WRITE_ADDRESS ioctl */
	/*  */
	/* indeed param dmaAddresses should be UNSIGNED LONG type for 64 bit kernel. */
	/* Considering our plartform supports max 4GB RAM(upper-32bit don't care for SW) */
	/* and consistent common code interface, remain uint32_t type. */
	cmdqU32Ptr_t dmaAddresses;

	cmdqU32Ptr_t values;	/* [OUT] uint32_t values that dmaAddresses point into */
} cmdqReadAddressStruct;

/*
 * Secure address metadata:
 * According to handle type, translate handle and replace (_d)th instruciton to
 *     1. sec_addr = hadnle_sec_base_addr(baseHandle) + offset(_b)
 *     2. sec_mva = mva( hadnle_sec_base_addr(baseHandle) + offset(_b) )
 *     3. secure world normal mva = map(baseHandle)
 *        . pass normal mva to parameter baseHandle
 *        . use case: OVL reads from secure and normal buffers at the same time)
 */
typedef enum CMDQ_SEC_ADDR_METADATA_TYPE {
	CMDQ_SAM_H_2_PA = 0,	/* sec handle to sec PA */
	CMDQ_SAM_H_2_MVA = 1,	/* sec handle to sec MVA */
	CMDQ_SAM_NMVA_2_MVA = 2,	/* map normal MVA to secure world */
} CMDQ_SEC_ADDR_METADATA_TYPE;

typedef struct cmdqSecAddrMetadataStruct {
	/* [IN]_d, index of instruction. Update its argB value to real PA/MVA in secure world */
	uint32_t instrIndex;

	CMDQ_SEC_ADDR_METADATA_TYPE type;	/* [IN] addr handle type */
	uint32_t baseHandle;	/* [IN]_h, secure address handle */
	uint32_t offset;	/* [IN]_b, buffser offset to secure handle */
	uint32_t size;		/* buffer size */
	uint32_t port;		/* hw port id (i.e. M4U port id) */
} cmdqSecAddrMetadataStruct;

typedef struct cmdqSecDataStruct {
	bool isSecure;		/* [IN]true for secure command */

	/* address metadata, used to translate secure buffer PA related instruction in secure world */
	uint32_t addrMetadataCount;	/* [IN] count of element in addrList */
	cmdqU32Ptr_t addrMetadatas;	/* [IN] array of cmdqSecAddrMetadataStruct */
	uint32_t addrMetadataMaxCount;	/*[Reserved] */

	uint64_t enginesNeedDAPC;
	uint64_t enginesNeedPortSecurity;

	/* [Reserved] This is for CMDQ driver usage itself. Not for client. */
	int32_t waitCookie;	/* task index in thread's tasklist. -1 for not in tasklist. */
	bool resetExecCnt;	/* reset HW thread in SWd */
} cmdqSecDataStruct;

#ifdef CMDQ_PROFILE_MARKER_SUPPORT
typedef struct cmdqProfileMarkerStruct {
	uint32_t count;
	long long hSlot;	/* i.e. cmdqBackupSlotHandle, physical start address of backup slot */
	cmdqU32Ptr_t tag[CMDQ_MAX_PROFILE_MARKER_IN_TASK];
} cmdqProfileMarkerStruct;
#endif

typedef struct cmdqCommandStruct {
	/* [IN] deprecated. will remove in the future. */
	uint32_t scenario;
	/* [IN] task schedule priority. this is NOT HW thread priority. */
	uint32_t priority;
	/* [IN] bit flag of engines used. */
	uint64_t engineFlag;
	/* [IN] pointer to instruction buffer. Use 64-bit for compatibility. */
	/* This must point to an 64-bit aligned uint32_t array */
	cmdqU32Ptr_t pVABase;
	/* [IN] size of instruction buffer, in bytes. */
	uint32_t blockSize;
	/* [IN] request to read register values at the end of command */
	cmdqReadRegStruct regRequest;
	/* [OUT] register values of regRequest */
	cmdqRegValueStruct regValue;
	/* [IN/OUT] physical addresses to read value */
	cmdqReadAddressStruct readAddress;
	/*[IN] secure execution data */
	cmdqSecDataStruct secData;
	/* [IN] set to non-zero to enable register debug dump. */
	uint32_t debugRegDump;
	/* [Reserved] This is for CMDQ driver usage itself. Not for client. Do not access this field from User Space */
	cmdqU32Ptr_t privateData;
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	cmdqProfileMarkerStruct profileMarker;
#endif
} cmdqCommandStruct;

typedef enum CMDQ_CAP_BITS {
	/* bit 0: TRUE if WFE instruction support is ready. FALSE if we need to POLL instead. */
	CMDQ_CAP_WFE = 0,
} CMDQ_CAP_BITS;


/**
 * reply struct for cmdq_sec_cancel_error_task
 */

typedef struct {
	/* [OUT] */
	bool throwAEE;
	bool hasReset;
	int32_t irqFlag;
	uint32_t errInstr[2];
	uint32_t regValue;
	uint32_t pc;
} cmdqSecCancelTaskResultStruct;

#endif				/* __CMDQ_DEF_H__ */
