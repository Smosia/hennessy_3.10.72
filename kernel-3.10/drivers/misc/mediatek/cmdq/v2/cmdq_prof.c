#include "cmdq_prof.h"
#include "cmdq_core.h"

/* expect no EMI latency, GCE spends 80 ns per 4 cycle*/
#define CMDQ_HW_EXEC_NS(hw_cycle) (hw_cycle * (80 / 4))

typedef enum CMDQ_OP_STATISTIC_ENUM {
	CMDQ_STA_READ = 0,
	CMDQ_STA_MOVE,
	CMDQ_STA_WRI,
	CMDQ_STA_WRI_WITH_MASK,
	CMDQ_STA_POLL,
	CMDQ_STA_JUMP,
	CMDQ_STA_WFE,		/* wait for event */
	CMDQ_STA_SYNC,		/* sync op and no wait for event */
	CMDQ_STA_EOC,
	CMDQ_STA_MAX_COUNT,	/* always keep at the end */
} CMDQ_OP_STATISTIC_ENUM;

const char *cmdq_prof_get_statistic_id_name(const CMDQ_OP_STATISTIC_ENUM statisticId)
{
	const char *IDName = "UNKNOWN";

#undef DECLARE_CMDQ_INSTR_CYCLE
#define DECLARE_CMDQ_INSTR_CYCLE(id, hw_op, cycle, name) { if (id == statisticId) {IDName = #name; break; } }
	do {
#include "cmdq_instr_cycle.h"
	} while (0);
#undef DECLARE_CMDQ_INSTR_CYCLE

	return IDName;
}

uint32_t cmdq_prof_get_statistic_id(const uint32_t *pCmd)
{
	const uint32_t argB = pCmd[0];
	const uint32_t argA = pCmd[1];
	const uint32_t op = argA >> 24;
	uint32_t addr;
	uint32_t maskEn = 0;

	switch (op) {
	case CMDQ_CODE_WRITE:
		/* check address */
		if (argA & (1 << 23)) {
			/* address is GPR */
			return CMDQ_STA_WRI;
		}
		addr = cmdq_core_subsys_to_reg_addr(argA);
		maskEn = (addr & 0x1);
		if (maskEn)
			return CMDQ_STA_WRI_WITH_MASK;
		else
			return CMDQ_STA_WRI;

	case CMDQ_CODE_WFE:
		/* check waiting flag */
		if ((argB >> 15) & 0x1)
			return CMDQ_STA_WFE;
		else
			return CMDQ_STA_SYNC;

	case CMDQ_CODE_READ:
		return CMDQ_STA_READ;

	case CMDQ_CODE_MOVE:
		return CMDQ_STA_MOVE;

	case CMDQ_CODE_POLL:
		return CMDQ_STA_POLL;

	case CMDQ_CODE_JUMP:
		return CMDQ_STA_JUMP;

	case CMDQ_CODE_EOC:
		return CMDQ_STA_EOC;
	default:
		CMDQ_ERR("unknown op, argA: 0x%08x\n", argA);
		return 0;
	}

	CMDQ_ERR("unknown op, argA: 0x%08x\n", argA);
	return 0;
}

uint32_t cmdq_prof_calculate_HW_cycle(const CMDQ_OP_STATISTIC_ENUM statisticId,
				      const uint32_t count)
{
	uint32_t hwCycle = -1;

#undef DECLARE_CMDQ_INSTR_CYCLE
#define DECLARE_CMDQ_INSTR_CYCLE(id, hw_op, cycle, name) { if (id == statisticId) {hwCycle = (count * cycle); break; } }
	do {
#include "cmdq_instr_cycle.h"
	} while (0);
#undef DECLARE_CMDQ_INSTR_CYCLE

	if (-1 == hwCycle) {
		/* Error message dump */
		CMDQ_ERR("unknown statisticId: %d\n", statisticId);
	}

	return hwCycle;
}

int32_t cmdq_prof_estimate_command_exe_time(const uint32_t *pCmd, uint32_t commandSize)
{
	uint32_t statistic[CMDQ_STA_MAX_COUNT] = { 0 };
	int i;
	uint32_t statisticId = 0;
	uint32_t cycle = 0;
	uint32_t totalCycle = 0;
	uint32_t totalNS = 0;

	if (NULL == pCmd)
		return -EFAULT;

	/* gather statistic */
	for (i = 0; i < commandSize; i += CMDQ_INST_SIZE, pCmd += 2) {
		statisticId = cmdq_prof_get_statistic_id(pCmd);
		statistic[statisticId] += 1;
	}

	CMDQ_LOG("NAME, COUNT, least HW exec cycle\n");
	/* calculate execution time */
	for (i = 0; i < CMDQ_STA_MAX_COUNT; i++) {
		cycle = cmdq_prof_calculate_HW_cycle(i, statistic[i]);
		totalCycle += cycle;
		CMDQ_LOG("%d:%11s, %d, %3d\n",
			 i, cmdq_prof_get_statistic_id_name(i), statistic[i], cycle);
	}

	totalNS = CMDQ_HW_EXEC_NS(totalCycle);
	CMDQ_LOG("=====================================\n");
	CMDQ_LOG("estimated least HW exec time(ns): %6d\n", totalNS);
	CMDQ_LOG("***each HW cycle spends time(ns): %6d\n", CMDQ_HW_EXEC_NS(1));
	CMDQ_LOG
	    ("***Real exec time will be longer when POLL/WAIT instr cannot pass their condition immediately.\n");

	return totalNS;
}
