#ifndef __CMDQ_SEC_IWC_COMMON_H__
#define __CMDQ_SEC_IWC_COMMON_H__

/* shared DRAM */
#define CMDQ_SEC_SHARED_IRQ_RAISED_OFFSET    (0x0) /* bit x = 1 means thread x raise IRQ */
#define CMDQ_SEC_SHARED_THR_CNT_OFFSET (0x100)

/* commanad buffer & metadata */
#define CMDQ_TZ_CMD_BLOCK_SIZE	 (32 * 1024)

#define CMDQ_IWC_MAX_CMD_LENGTH (32 * 1024 / 4)

#define CMDQ_IWC_MAX_ADDR_LIST_LENGTH (20)

#define CMDQ_IWC_CLIENT_NAME (16)

typedef enum CMDQ_IWC_ADDR_METADATA_TYPE {
	CMDQ_IWC_H_2_PA = 0, /* sec handle to sec PA */
	CMDQ_IWC_H_2_MVA = 1, /* sec handle to sec MVA */
	CMDQ_IWC_NMVA_2_MVA = 2, /* map normal MVA to secure world */
} CMDQ_IWC_ADDR_METADATA_TYPE;

/*  */
/* IWC message */
/*  */
typedef struct{
	/* [IN]_d, index of instruction. Update its argB value to real PA/MVA in secure world */
	uint32_t instrIndex;

	uint32_t type; /* [IN] addr handle type*/
	uint32_t baseHandle; /* [IN]_h, secure address handle */
	uint32_t offset;     /* [IN]_b, buffser offset to secure handle */
	uint32_t size;       /* buffer size */
	uint32_t port;       /* hw port id (i.e. M4U port id)*/
} iwcCmdqAddrMetadata_t;

typedef struct {
	int32_t logLevel;
	int32_t enableProfile;
} iwcCmdqDebugConfig_t;

typedef struct {
	uint64_t startTime;	/* start timestamp */
	uint64_t endTime;	/* end timestamp */
} iwcCmdqSystraceLog_t;

typedef struct {
	uint32_t addrListLength;
	iwcCmdqAddrMetadata_t addrList[CMDQ_IWC_MAX_ADDR_LIST_LENGTH];

	uint64_t enginesNeedDAPC;
	uint64_t enginesNeedPortSecurity;
} iwcCmdqMetadata_t;

typedef struct {
	uint32_t addr; /* pass VA for TCI cases, and pass PA for DCI case */
	uint32_t size;
} iwcCmdqSectraceBuffer_t;

typedef struct {
	long long shareMemoyPA; /* use long long for 64 bit compatible support */
	uint32_t size;
	bool useNormalIRQ;		/* use normal IRQ in SWd */
} iwcCmdqPathResource_t;

typedef struct {
	/* [IN] */
	int32_t thread;
	uint32_t waitCookie;

	/* [OUT] */
	bool throwAEE;
	bool hasReset;
	int32_t irqStatus; /* global secure IRQ flag */
	int32_t irqFlag; /* thread IRQ flag */
	uint32_t errInstr[2]; /* errInstr[0] = instB, errInstr[1] = instA */
	uint32_t regValue;
	uint32_t pc;
} iwcCmdqCancelTask_t;

typedef struct {
	/* basic execution data */
	uint32_t thread;
	uint32_t scenario;
	uint32_t priority;
	uint32_t commandSize;
	uint64_t engineFlag;
	uint32_t pVABase[CMDQ_IWC_MAX_CMD_LENGTH];

	/* exec order data */
	uint32_t waitCookie; /* [IN] index in thread's task list, it should be (nextCookie - 1) */
	bool resetExecCnt;   /* [IN] reset HW thread */

	/* client info */
	int32_t callerPid;
	char callerName[CMDQ_IWC_CLIENT_NAME];

	/* metadata */
	iwcCmdqMetadata_t metadata;

	/* debug */
	uint64_t hNormalTask; /* handle to reference task in normal world*/
} iwcCmdqCommand_t;

/*  */
/* linex kernel and mobicore has their own MMU tables, */
/* the latter's is used to map world shared memory and physical address */
/* so mobicore dose not understand linux virtual address mapping. */
/*  */
/* if we want to transact a large buffer in TCI/DCI, there are 2 method (both need 1 copy): */
/* 1. use mc_map, to map normal world buffer to WSM, and pass secure_virt_addr in TCI/DCI buffer */
/* note mc_map implies a memcopy to copy content from normal world to WSM */
/* 2. declare a fixed lenth array in TCI/DCI struct, and its size must be < 1M */
/*  */
typedef struct {
	union {
		uint32_t cmd;	/* [IN] command id */
		int32_t rsp;	/* [OUT] 0 for success, < 0 for error */
	};

	union {
		iwcCmdqCommand_t command;
		iwcCmdqCancelTask_t cancelTask;
		iwcCmdqPathResource_t pathResource;
		iwcCmdqSectraceBuffer_t sectracBuffer;
	};

	iwcCmdqDebugConfig_t debug;
} iwcCmdqMessage_t, *iwcCmdqMessage_ptr;

/*  */
/* ERROR code number (ERRNO) */
/* note the error result returns negative value, i.e, -(ERRNO) */
/*  */
#define	CMDQ_ERR_NOMEM		(12)	/* out of memory */
#define	CMDQ_ERR_FAULT		(14)	/* bad address */

#define CMDQ_ERR_ADDR_CONVERT_HANDLE_2_PA (1000)
#define CMDQ_ERR_ADDR_CONVERT_ALLOC_MVA   (1100)
#define CMDQ_ERR_ADDR_CONVERT_FREE_MVA	  (1200)
#define CMDQ_ERR_PORT_CONFIG			  (1300)

/* param check */
#define CMDQ_ERR_UNKNOWN_ADDR_METADATA_TYPE (1400)
#define CMDQ_ERR_TOO_MANY_SEC_HANDLE (1401)
/* security check */
#define CMDQ_ERR_SECURITY_INVALID_INSTR	  (1500)
#define CMDQ_ERR_SECURITY_INVALID_SEC_HANDLE (1501)
#define CMDQ_ERR_SECURITY_INVALID_DAPC_FALG (1502)
#define CMDQ_ERR_INSERT_DAPC_INSTR_FAILED (1503)
#define CMDQ_ERR_INSERT_PORT_SECURITY_INSTR_FAILED (1504)

#define CMDQ_TL_ERR_UNKNOWN_IWC_CMD	   (5000)

#define CMDQ_ERR_DR_IPC_EXECUTE_SESSION   (5001)
#define CMDQ_ERR_DR_IPC_CLOSE_SESSION	 (5002)
#define CMDQ_ERR_DR_EXEC_FAILED		   (5003)

#endif				/* __CMDQ_SEC_TLAPI_H__ */
