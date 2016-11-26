#ifndef __CMDQ_CORE_VIRTUAL_H__
#define __CMDQ_CORE_VIRTUAL_H__

#include "cmdq_def.h"

/* get subsys LSB in argA */
typedef const uint32_t(*CmdqGetSubsysLSBArgA) (void);

/* is a secure thread */
typedef const bool(*CmdqIsSecureThread) (const int32_t thread);

/* is valid notify thread for secure path */
typedef const bool(*CmdqIsValidNotifyThread) (const int32_t thread);

/* is display scenario */
typedef bool(*CmdqIsDispScenario) (const CMDQ_SCENARIO_ENUM scenario);

/* should enable prefetch */
typedef bool(*CmdqShouldEnablePrefetch) (const CMDQ_SCENARIO_ENUM scenario);

/* should profile */
typedef bool(*CmdqShouldProfile) (const CMDQ_SCENARIO_ENUM scenario);

/* display thread index from scenario */
typedef int (*CmdqDispThread) (CMDQ_SCENARIO_ENUM scenario);

/* get thread index from scenario and secure */
typedef int (*CmdqGetThreadID) (CMDQ_SCENARIO_ENUM scenario, const bool secure);

/*  priority from scenario */
typedef CMDQ_HW_THREAD_PRIORITY_ENUM(*CmdqPriority) (CMDQ_SCENARIO_ENUM scenario);

/*  force loop IRQ from scenario */
typedef bool(*CmdqForceLoopIRQ) (CMDQ_SCENARIO_ENUM scenario);

/*  is loop scenario */
typedef bool(*CmdqIsLoopScenario) (CMDQ_SCENARIO_ENUM scenario, bool displayOnly);

/* get register index from hwflag */
typedef void(*CmdqGetRegID) (uint64_t hwflag,
			     CMDQ_DATA_REGISTER_ENUM *valueRegId,
			     CMDQ_DATA_REGISTER_ENUM *destRegId, CMDQ_EVENT_ENUM *regAccessToken);

/*  module from event index */
typedef const char *(*CmdqModuleFromEvent) (const int32_t event);

/* parse module from register addr */
typedef const char *(*CmdqParseModule) (uint32_t reg_addr);

/* can module entry suspend */
typedef const int32_t(*CmdqModuleEntrySuspend) (EngineStruct *engineList);

/* print status clock */
typedef ssize_t(*CmdqPrintStatusClock) (char *buf);

/* print seq status clock */
typedef void (*CmdqPrintStatusSeqClock) (struct seq_file *m);

/* enable common clock locked */
typedef void (*CmdqEnableCommonClockLocked) (bool enable);

/* enable GCE clock locked */
typedef void (*CmdqEnableGCEClockLocked) (bool enable);

/* parse error module by hwflag */
typedef const char *(*CmdqParseErrorModule) (const TaskStruct *pTask);

/* dump mmsys config */
typedef void (*CmdqDumpMMSYSConfig) (void);

/* dump clock gating */
typedef void (*CmdqDumpClockGating) (void);

/* dump SMI */
typedef int (*CmdqDumpSMI) (const int showSmiDump);

/* dump GPR */
typedef void (*CmdqDumpGPR) (void);

/* flag from scenario */
typedef uint64_t(*CmdqFlagFromScenario) (CMDQ_SCENARIO_ENUM scenario);

/* initial backup evet setting */
typedef void (*CmdqInitialBackupEvent) (void);

/* evet backup */
typedef void (*CmdqEventBackup) (void);

/* evet restore */
typedef void (*CmdqEventRestore) (void);

/* test setup */
typedef void (*CmdqTestSetup) (void);

/* test cleanup */
typedef void (*CmdqTestCleanup) (void);

/* test for instruction statistic */
typedef void (*CmdqInitModulePAStat) (void);

typedef struct cmdqCoreFuncStruct {
	CmdqGetSubsysLSBArgA getSubsysLSBArgA;
	CmdqIsSecureThread isSecureThread;
	CmdqIsValidNotifyThread isValidNotifyThread;
	CmdqIsDispScenario isDispScenario;
	CmdqShouldEnablePrefetch shouldEnablePrefetch;
	CmdqShouldProfile shouldProfile;
	CmdqDispThread dispThread;
	CmdqGetThreadID getThreadID;
	CmdqPriority priority;
	CmdqForceLoopIRQ forceLoopIRQ;
	CmdqIsLoopScenario isLoopScenario;
	CmdqGetRegID getRegID;
	CmdqModuleFromEvent moduleFromEvent;
	CmdqParseModule parseModule;
	CmdqModuleEntrySuspend moduleEntrySuspend;
	CmdqPrintStatusClock printStatusClock;
	CmdqPrintStatusSeqClock printStatusSeqClock;
	CmdqEnableCommonClockLocked enableCommonClockLocked;
	CmdqEnableGCEClockLocked enableGCEClockLocked;
	CmdqParseErrorModule parseErrorModule;
	CmdqDumpMMSYSConfig dumpMMSYSConfig;
	CmdqDumpClockGating dumpClockGating;
	CmdqDumpSMI dumpSMI;
	CmdqDumpGPR dumpGPR;
	CmdqFlagFromScenario flagFromScenario;
	CmdqInitialBackupEvent initialBackupEvent;
	CmdqEventBackup eventBackup;
	CmdqEventRestore eventRestore;
	CmdqTestSetup testSetup;
	CmdqTestCleanup testCleanup;
	CmdqInitModulePAStat initModulePAStat;
} cmdqCoreFuncStruct;

#ifdef __cplusplus
extern "C" {
#endif
	void cmdq_virtual_function_setting(void);
	cmdqCoreFuncStruct *cmdq_get_func(void);

#ifdef __cplusplus
}
#endif
#endif				/* __CMDQ_CORE_VIRTUAL_H__ */
