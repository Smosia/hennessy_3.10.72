#include "cmdq_mdp_common.h"
#include "cmdq_core.h"
#include "cmdq_device.h"
#include "cmdq_reg.h"
#ifdef CMDQ_PROFILE_MMP
#include "cmdq_mmp.h"
#endif

/**************************************************************************************/
/*******************                    Platform dependent function                    ********************/
/**************************************************************************************/

typedef struct RegDef {
	int offset;
	const char *name;
} RegDef;

void cmdq_mdp_dump_mmsys_config_virtual(void)
{
	int i = 0;
	uint32_t value = 0;

	static const struct RegDef configRegisters[] = {
		{0x01c, "ISP_MOUT_EN"},
		{0x020, "MDP_RDMA_MOUT_EN"},
		{0x024, "MDP_PRZ0_MOUT_EN"},
		{0x028, "MDP_PRZ1_MOUT_EN"},
		{0x02C, "MDP_TDSHP_MOUT_EN"},
		{0x030, "DISP_OVL0_MOUT_EN"},
		{0x034, "DISP_OVL1_MOUT_EN"},
		{0x038, "DISP_DITHER_MOUT_EN"},
		{0x03C, "DISP_UFOE_MOUT_EN"},
		/* {0x040, "MMSYS_MOUT_RST"}, */
		{0x044, "MDP_PRZ0_SEL_IN"},
		{0x048, "MDP_PRZ1_SEL_IN"},
		{0x04C, "MDP_TDSHP_SEL_IN"},
		{0x050, "MDP_WDMA_SEL_IN"},
		{0x054, "MDP_WROT_SEL_IN"},
		{0x058, "DISP_COLOR_SEL_IN"},
		{0x05C, "DISP_WDMA_SEL_IN"},
		{0x060, "DISP_UFOE_SEL_IN"},
		{0x064, "DSI0_SEL_IN"},
		{0x068, "DPI0_SEL_IN"},
		{0x06C, "DISP_RDMA0_SOUT_SEL_IN"},
		{0x070, "DISP_RDMA1_SOUT_SEL_IN"},
		{0x0F0, "MMSYS_MISC"},
		/* ACK and REQ related */
		{0x8a0, "DISP_DL_VALID_0"},
		{0x8a4, "DISP_DL_VALID_1"},
		{0x8a8, "DISP_DL_READY_0"},
		{0x8ac, "DISP_DL_READY_1"},
		{0x8b0, "MDP_DL_VALID_0"},
		{0x8b4, "MDP_DL_READY_0"}
	};

	for (i = 0; i < sizeof(configRegisters) / sizeof(configRegisters[0]); ++i) {
		value = CMDQ_REG_GET16(MMSYS_CONFIG_BASE + configRegisters[i].offset);
		CMDQ_ERR("%s: 0x%08x\n", configRegisters[i].name, value);
	}
}

/* VENC callback function */
int32_t cmdqVEncDumpInfo_virtual(uint64_t engineFlag, int level)
{
	return 0;
}

/* Initialization & de-initialization MDP base VA */
void cmdq_mdp_init_module_base_VA_virtual(void)
{
	/* Do Nothing */
}

void cmdq_mdp_deinit_module_base_VA_virtual(void)
{
	/* Do Nothing */
}

/* query MDP clock is on  */
bool cmdq_mdp_clock_is_on_virtual(CMDQ_ENG_ENUM engine)
{
	return false;
}

/* enable MDP clock  */
void cmdq_mdp_enable_clock_virtual(bool enable, CMDQ_ENG_ENUM engine)
{
	/* Do Nothing */
}

/* Common Clock Framework */
void cmdq_mdp_init_module_clk_virtual(void)
{
	/* Do Nothing */
}

/* MDP engine dump */
void cmdq_mdp_dump_rsz_virtual(const unsigned long base, const char *label)
{
	uint32_t value[8] = { 0 };
	uint32_t request[4] = { 0 };
	uint32_t state = 0;

	value[0] = CMDQ_REG_GET32(base + 0x004);
	value[1] = CMDQ_REG_GET32(base + 0x00C);
	value[2] = CMDQ_REG_GET32(base + 0x010);
	value[3] = CMDQ_REG_GET32(base + 0x014);
	value[4] = CMDQ_REG_GET32(base + 0x018);
	CMDQ_REG_SET32(base + 0x040, 0x00000001);
	value[5] = CMDQ_REG_GET32(base + 0x044);
	CMDQ_REG_SET32(base + 0x040, 0x00000002);
	value[6] = CMDQ_REG_GET32(base + 0x044);
	CMDQ_REG_SET32(base + 0x040, 0x00000003);
	value[7] = CMDQ_REG_GET32(base + 0x044);

	CMDQ_ERR("=============== [CMDQ] %s Status ====================================\n", label);
	CMDQ_ERR("RSZ_CONTROL: 0x%08x, RSZ_INPUT_IMAGE: 0x%08x RSZ_OUTPUT_IMAGE: 0x%08x\n",
		 value[0], value[1], value[2]);
	CMDQ_ERR("RSZ_HORIZONTAL_COEFF_STEP: 0x%08x, RSZ_VERTICAL_COEFF_STEP: 0x%08x\n",
		 value[3], value[4]);
	CMDQ_ERR("RSZ_DEBUG_1: 0x%08x, RSZ_DEBUG_2: 0x%08x, RSZ_DEBUG_3: 0x%08x\n",
		 value[5], value[6], value[7]);

	/* parse state */
	/* .valid=1/request=1: upstream module sends data */
	/* .ready=1: downstream module receives data */
	state = value[6] & 0xF;
	request[0] = state & (0x1);	/* out valid */
	request[1] = (state & (0x1 << 1)) >> 1;	/* out ready */
	request[2] = (state & (0x1 << 2)) >> 2;	/* in valid */
	request[3] = (state & (0x1 << 3)) >> 3;	/* in ready */

	CMDQ_ERR("RSZ inRdy,inRsq,outRdy,outRsq: %d,%d,%d,%d (%s)\n",
		 request[3], request[2], request[1], request[0], cmdq_mdp_get_rsz_state(state));
}

void cmdq_mdp_dump_tdshp_virtual(const unsigned long base, const char *label)
{
	uint32_t value[8] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x114);
	value[1] = CMDQ_REG_GET32(base + 0x11C);
	value[2] = CMDQ_REG_GET32(base + 0x104);
	value[3] = CMDQ_REG_GET32(base + 0x108);
	value[4] = CMDQ_REG_GET32(base + 0x10C);
	value[5] = CMDQ_REG_GET32(base + 0x120);
	value[6] = CMDQ_REG_GET32(base + 0x128);
	value[7] = CMDQ_REG_GET32(base + 0x110);

	CMDQ_ERR("=============== [CMDQ] %s Status ====================================\n", label);
	CMDQ_ERR("TDSHP INPUT_CNT: 0x%08x, OUTPUT_CNT: 0x%08x\n", value[0], value[1]);
	CMDQ_ERR("TDSHP INTEN: 0x%08x, INTSTA: 0x%08x, 0x10C: 0x%08x\n", value[2], value[3],
		 value[4]);
	CMDQ_ERR("TDSHP CFG: 0x%08x, IN_SIZE: 0x%08x, OUT_SIZE: 0x%08x\n", value[7], value[5],
		 value[6]);
}

/* MDP callback function */
int32_t cmdqMdpClockOn_virtual(uint64_t engineFlag)
{
	return 0;
}

int32_t cmdqMdpDumpInfo_virtual(uint64_t engineFlag, int level)
{
	return 0;
}

int32_t cmdqMdpResetEng_virtual(uint64_t engineFlag)
{
	return 0;
}

int32_t cmdqMdpClockOff_virtual(uint64_t engineFlag)
{
	return 0;
}

/* MDP Initialization setting */
void cmdqMdpInitialSetting_virtual(void)
{
	/* Do Nothing */
}

/* test MDP clock function */
const uint32_t cmdq_mdp_rdma_get_reg_offset_src_addr_virtual(void)
{
	return 0;
}

const uint32_t cmdq_mdp_wrot_get_reg_offset_dst_addr_virtual(void)
{
	return 0;
}

const uint32_t cmdq_mdp_wdma_get_reg_offset_dst_addr_virtual(void)
{
	return 0;
}

void testcase_clkmgr_mdp_virtual(void)
{
}

/**************************************************************************************/
/************************                      Common Code                      ************************/
/**************************************************************************************/
static cmdqMDPFuncStruct gMDPFunctionPointer;

void cmdq_mdp_virtual_function_setting(void)
{
	cmdqMDPFuncStruct *pFunc;

	pFunc = &(gMDPFunctionPointer);

	pFunc->dumpMMSYSConfig = cmdq_mdp_dump_mmsys_config_virtual;

	pFunc->vEncDumpInfo = cmdqVEncDumpInfo_virtual;

	pFunc->initModuleBaseVA = cmdq_mdp_init_module_base_VA_virtual;
	pFunc->deinitModuleBaseVA = cmdq_mdp_deinit_module_base_VA_virtual;

	pFunc->mdpClockIsOn = cmdq_mdp_clock_is_on_virtual;
	pFunc->enableMdpClock = cmdq_mdp_enable_clock_virtual;
	pFunc->initModuleCLK = cmdq_mdp_init_module_clk_virtual;

	pFunc->mdpDumpRsz = cmdq_mdp_dump_rsz_virtual;
	pFunc->mdpDumpTdshp = cmdq_mdp_dump_tdshp_virtual;

	pFunc->mdpClockOn = cmdqMdpClockOn_virtual;
	pFunc->mdpDumpInfo = cmdqMdpDumpInfo_virtual;
	pFunc->mdpResetEng = cmdqMdpResetEng_virtual;
	pFunc->mdpClockOff = cmdqMdpClockOff_virtual;

	pFunc->mdpInitialSet = cmdqMdpInitialSetting_virtual;

	pFunc->rdmaGetRegOffsetSrcAddr = cmdq_mdp_rdma_get_reg_offset_src_addr_virtual;
	pFunc->wrotGetRegOffsetDstAddr = cmdq_mdp_wrot_get_reg_offset_dst_addr_virtual;
	pFunc->wdmaGetRegOffsetDstAddr = cmdq_mdp_wdma_get_reg_offset_dst_addr_virtual;
	pFunc->testcaseClkmgrMdp = testcase_clkmgr_mdp_virtual;
}

cmdqMDPFuncStruct *cmdq_mdp_get_func(void)
{
	return &gMDPFunctionPointer;
}

void cmdq_mdp_enable(uint64_t engineFlag, CMDQ_ENG_ENUM engine)
{
#ifdef CMDQ_PWR_AWARE
	CMDQ_VERBOSE("Test for ENG %d\n", engine);
	if (engineFlag & (1LL << engine))
		cmdq_mdp_get_func()->enableMdpClock(true, engine);
#endif
}

int cmdq_mdp_loop_reset_impl(const unsigned long resetReg,
			     const uint32_t resetWriteValue,
			     const unsigned long resetStateReg,
			     const uint32_t resetMask,
			     const uint32_t resetPollingValue, const int32_t maxLoopCount)
{
	int loop = 0;

	CMDQ_REG_SET32(resetReg, resetWriteValue);
	while (loop < maxLoopCount) {
		if (resetPollingValue == (CMDQ_REG_GET32(resetStateReg) & resetMask))
			break;

		loop++;
	}

	/* return polling result */
	if (loop >= maxLoopCount) {
		CMDQ_ERR
		    ("%s failed, Reg:0x%lx, writeValue:0x%08x, stateReg:0x%lx, mask:0x%08x, pollingValue:0x%08x\n",
		     __func__, resetReg, resetWriteValue, resetStateReg, resetMask,
		     resetPollingValue);
		return -EFAULT;
	}

	return 0;
}

int cmdq_mdp_loop_reset(CMDQ_ENG_ENUM engine,
			const unsigned long resetReg,
			const unsigned long resetStateReg,
			const uint32_t resetMask,
			const uint32_t resetValue, const bool pollInitResult)
{
#ifdef CMDQ_PWR_AWARE
	int resetStatus = 0;
	int initStatus = 0;

	if (cmdq_mdp_get_func()->mdpClockIsOn(engine)) {
		CMDQ_PROF_START(current->pid, __func__);
		CMDQ_PROF_MMP(cmdq_mmp_get_event()->MDP_reset,
			      MMProfileFlagStart, resetReg, resetStateReg);


		/* loop reset */
		resetStatus = cmdq_mdp_loop_reset_impl(resetReg, 0x1,
						       resetStateReg, resetMask, resetValue,
						       CMDQ_MAX_LOOP_COUNT);

		if (pollInitResult) {
			/* loop  init */
			initStatus = cmdq_mdp_loop_reset_impl(resetReg, 0x0,
							      resetStateReg, resetMask, 0x0,
							      CMDQ_MAX_LOOP_COUNT);
		} else {
			/* always clear to init state no matter what polling result */
			CMDQ_REG_SET32(resetReg, 0x0);
		}

		CMDQ_PROF_MMP(cmdq_mmp_get_event()->MDP_reset,
			      MMProfileFlagEnd, resetReg, resetStateReg);
		CMDQ_PROF_END(current->pid, __func__);

		/* retrun failed if loop failed */
		if ((0 > resetStatus) || (0 > initStatus)) {
			CMDQ_ERR("Reset MDP %ld failed, resetStatus:%d, initStatus:%d\n",
				 resetReg, resetStatus, initStatus);
			return -EFAULT;
		}
	}
#endif

	return 0;
};

void cmdq_mdp_loop_off(CMDQ_ENG_ENUM engine,
		       const unsigned long resetReg,
		       const unsigned long resetStateReg,
		       const uint32_t resetMask,
		       const uint32_t resetValue, const bool pollInitResult)
{
#ifdef CMDQ_PWR_AWARE
	int resetStatus = 0;
	int initStatus = 0;

	if (cmdq_mdp_get_func()->mdpClockIsOn(engine)) {

		/* loop reset */
		resetStatus = cmdq_mdp_loop_reset_impl(resetReg, 0x1,
						       resetStateReg, resetMask, resetValue,
						       CMDQ_MAX_LOOP_COUNT);

		if (pollInitResult) {
			/* loop init */
			initStatus = cmdq_mdp_loop_reset_impl(resetReg, 0x0,
							      resetStateReg, resetMask, 0x0,
							      CMDQ_MAX_LOOP_COUNT);
		} else {
			/* always clear to init state no matter what polling result */
			CMDQ_REG_SET32(resetReg, 0x0);
		}

		/* retrun failed if loop failed */
		if ((0 > resetStatus) || (0 > initStatus)) {
			CMDQ_AEE("MDP",
				 "Disable %ld engine failed, resetStatus:%d, initStatus:%d\n",
				 resetReg, resetStatus, initStatus);
			return;
		}

		cmdq_mdp_get_func()->enableMdpClock(false, engine);
	}
#endif
}

void cmdq_mdp_dump_venc(const unsigned long base, const char *label)
{
	CMDQ_ERR("======== cmdq_mdp_dump_venc + ========\n");
	CMDQ_ERR("[0x%lx] to [0x%lx]\n", base, base + 0x1000 * 4);

	print_hex_dump(KERN_ERR, "[CMDQ][ERR][VENC]", DUMP_PREFIX_ADDRESS, 16, 4,
		       (void *)base, 0x1000, false);
	CMDQ_ERR("======== cmdq_mdp_dump_venc - ========\n");
}

const char *cmdq_mdp_get_rdma_state(uint32_t state)
{
	switch (state) {
	case 0x1:
		return "idle";
	case 0x2:
		return "wait sof";
	case 0x4:
		return "reg update";
	case 0x8:
		return "clear0";
	case 0x10:
		return "clear1";
	case 0x20:
		return "int0";
	case 0x40:
		return "int1";
	case 0x80:
		return "data running";
	case 0x100:
		return "wait done";
	case 0x200:
		return "warm reset";
	case 0x400:
		return "wait reset";
	default:
		return "";
	}
}

void cmdq_mdp_dump_rdma(const unsigned long base, const char *label)
{
	uint32_t value[15] = { 0 };
	uint32_t state = 0;
	uint32_t grep = 0;

	value[0] = CMDQ_REG_GET32(base + 0x030);
	value[1] = CMDQ_REG_GET32(base + cmdq_mdp_get_func()->rdmaGetRegOffsetSrcAddr());
	value[2] = CMDQ_REG_GET32(base + 0x060);
	value[3] = CMDQ_REG_GET32(base + 0x070);
	value[4] = CMDQ_REG_GET32(base + 0x078);
	value[5] = CMDQ_REG_GET32(base + 0x080);
	value[6] = CMDQ_REG_GET32(base + 0x100);
	value[7] = CMDQ_REG_GET32(base + 0x118);
	value[8] = CMDQ_REG_GET32(base + 0x130);
	value[9] = CMDQ_REG_GET32(base + 0x400);
	value[10] = CMDQ_REG_GET32(base + 0x408);
	value[11] = CMDQ_REG_GET32(base + 0x410);
	value[12] = CMDQ_REG_GET32(base + 0x420);
	value[13] = CMDQ_REG_GET32(base + 0x430);
	value[14] = CMDQ_REG_GET32(base + 0x4D0);

	CMDQ_ERR("=============== [CMDQ] %s Status ====================================\n", label);
	CMDQ_ERR
	    ("RDMA_SRC_CON: 0x%08x, RDMA_SRC_BASE_0: 0x%08x, RDMA_MF_BKGD_SIZE_IN_BYTE: 0x%08x\n",
	     value[0], value[1], value[2]);
	CMDQ_ERR("RDMA_MF_SRC_SIZE: 0x%08x, RDMA_MF_CLIP_SIZE: 0x%08x, RDMA_MF_OFFSET_1: 0x%08x\n",
		 value[3], value[4], value[5]);
	CMDQ_ERR("RDMA_SRC_END_0: 0x%08x, RDMA_SRC_OFFSET_0: 0x%08x, RDMA_SRC_OFFSET_W_0: 0x%08x\n",
		 value[6], value[7], value[8]);
	CMDQ_ERR("RDMA_MON_STA_0: 0x%08x, RDMA_MON_STA_1: 0x%08x, RDMA_MON_STA_2: 0x%08x\n",
		 value[9], value[10], value[11]);
	CMDQ_ERR("RDMA_MON_STA_4: 0x%08x, RDMA_MON_STA_6: 0x%08x, RDMA_MON_STA_26: 0x%08x\n",
		 value[12], value[13], value[14]);

	/* parse state */
	CMDQ_ERR("RDMA ack:%d req:%d\n", (value[9] & (1 << 11)) >> 11,
		 (value[9] & (1 << 10)) >> 10);
	state = (value[10] >> 8) & 0x7FF;
	grep = (value[10] >> 20) & 0x1;
	CMDQ_ERR("RDMA state: 0x%x (%s)\n", state, cmdq_mdp_get_rdma_state(state));
	CMDQ_ERR("RDMA horz_cnt: %d vert_cnt:%d\n", value[14] & 0xFFF, (value[14] >> 16) & 0xFFF);

	CMDQ_ERR("RDMA grep:%d => suggest to ask SMI help:%d\n", grep, grep);
}

const char *cmdq_mdp_get_rsz_state(const uint32_t state)
{
	switch (state) {
	case 0x5:
		return "downstream hang";	/* 0,1,0,1 */
	case 0xa:
		return "upstream hang";	/* 1,0,1,0 */
	default:
		return "";
	}
}

void cmdq_mdp_dump_rot(const unsigned long base, const char *label)
{
	uint32_t value[32] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x000);
	value[1] = CMDQ_REG_GET32(base + 0x008);
	value[2] = CMDQ_REG_GET32(base + 0x00C);
	value[3] = CMDQ_REG_GET32(base + 0x024);
	value[4] = CMDQ_REG_GET32(base + cmdq_mdp_get_func()->wrotGetRegOffsetDstAddr());
	value[5] = CMDQ_REG_GET32(base + 0x02C);
	value[6] = CMDQ_REG_GET32(base + 0x004);
	value[7] = CMDQ_REG_GET32(base + 0x030);
	value[8] = CMDQ_REG_GET32(base + 0x078);
	value[9] = CMDQ_REG_GET32(base + 0x070);
	CMDQ_REG_SET32(base + 0x018, 0x00000100);
	value[10] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000200);
	value[11] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000300);
	value[12] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000400);
	value[13] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000500);
	value[14] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000600);
	value[15] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000700);
	value[16] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000800);
	value[17] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000900);
	value[18] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000A00);
	value[19] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000B00);
	value[20] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000C00);
	value[21] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000D00);
	value[22] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000E00);
	value[23] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00000F00);
	value[24] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00001000);
	value[25] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00001100);
	value[26] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00001200);
	value[27] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00001300);
	value[28] = CMDQ_REG_GET32(base + 0x0D0);
	CMDQ_REG_SET32(base + 0x018, 0x00001400);
	value[29] = CMDQ_REG_GET32(base + 0x0D0);
	value[30] = CMDQ_REG_GET32(base + 0x01C);

	CMDQ_ERR("=============== [CMDQ] %s Status ====================================\n", label);
	CMDQ_ERR("ROT_CTRL: 0x%08x, ROT_MAIN_BUF_SIZE: 0x%08x, ROT_SUB_BUF_SIZE: 0x%08x\n",
		 value[0], value[1], value[2]);
	CMDQ_ERR("ROT_TAR_SIZE: 0x%08x, ROT_BASE_ADDR: 0x%08x, ROT_OFST_ADDR: 0x%08x\n",
		 value[3], value[4], value[5]);
	CMDQ_ERR("ROT_DMA_PERF: 0x%08x, ROT_STRIDE: 0x%08x, ROT_IN_SIZE: 0x%08x\n",
		 value[6], value[7], value[8]);
	CMDQ_ERR("ROT_EOL: 0x%08x, ROT_DBUGG_1: 0x%08x, ROT_DEBUBG_2: 0x%08x\n",
		 value[9], value[10], value[11]);
	CMDQ_ERR("ROT_DBUGG_3: 0x%08x, ROT_DBUGG_4: 0x%08x, ROT_DEBUBG_5: 0x%08x\n",
		 value[12], value[13], value[14]);
	CMDQ_ERR("ROT_DBUGG_6: 0x%08x, ROT_DBUGG_7: 0x%08x, ROT_DEBUBG_8: 0x%08x\n",
		 value[15], value[16], value[17]);
	CMDQ_ERR("ROT_DBUGG_9: 0x%08x, ROT_DBUGG_A: 0x%08x, ROT_DEBUBG_B: 0x%08x\n",
		 value[18], value[19], value[20]);
	CMDQ_ERR("ROT_DBUGG_C: 0x%08x, ROT_DBUGG_D: 0x%08x, ROT_DEBUBG_E: 0x%08x\n",
		 value[21], value[22], value[23]);
	CMDQ_ERR("ROT_DBUGG_F: 0x%08x, ROT_DBUGG_10: 0x%08x, ROT_DEBUBG_11: 0x%08x\n",
		 value[24], value[25], value[26]);
	CMDQ_ERR("ROT_DEBUG_12: 0x%08x, ROT_DBUGG_13: 0x%08x, ROT_DBUGG_14: 0x%08x\n",
		 value[27], value[28], value[29]);
	CMDQ_ERR("VIDO_INT: 0x%08x\n", value[30]);
}

void cmdq_mdp_dump_color(const unsigned long base, const char *label)
{
	uint32_t value[13] = { 0 };

	value[0] = CMDQ_REG_GET32(base + 0x400);
	value[1] = CMDQ_REG_GET32(base + 0x404);
	value[2] = CMDQ_REG_GET32(base + 0x408);
	value[3] = CMDQ_REG_GET32(base + 0x40C);
	value[4] = CMDQ_REG_GET32(base + 0x410);
	value[5] = CMDQ_REG_GET32(base + 0x420);
	value[6] = CMDQ_REG_GET32(base + 0xC00);
	value[7] = CMDQ_REG_GET32(base + 0xC04);
	value[8] = CMDQ_REG_GET32(base + 0xC08);
	value[9] = CMDQ_REG_GET32(base + 0xC0C);
	value[10] = CMDQ_REG_GET32(base + 0xC10);
	value[11] = CMDQ_REG_GET32(base + 0xC50);
	value[12] = CMDQ_REG_GET32(base + 0xC54);

	CMDQ_ERR("=============== [CMDQ] %s Status ====================================\n", label);
	CMDQ_ERR("COLOR CFG_MAIN: 0x%08x\n", value[0]);
	CMDQ_ERR("COLOR PXL_CNT_MAIN: 0x%08x, LINE_CNT_MAIN: 0x%08x\n", value[1], value[2]);
	CMDQ_ERR("COLOR WIN_X_MAIN: 0x%08x, WIN_Y_MAIN: 0x%08x, DBG_CFG_MAIN: 0x%08x\n", value[3], value[4],
		 value[5]);
	CMDQ_ERR("COLOR START: 0x%08x, INTEN: 0x%08x, INTSTA: 0x%08x\n", value[6], value[7],
		 value[8]);
	CMDQ_ERR("COLOR OUT_SEL: 0x%08x, FRAME_DONE_DEL: 0x%08x\n", value[9], value[10]);
	CMDQ_ERR("COLOR INTERNAL_IP_WIDTH: 0x%08x, INTERNAL_IP_HEIGHT: 0x%08x\n", value[11], value[12]);
}

const char *cmdq_mdp_get_wdma_state(uint32_t state)
{
	switch (state) {
	case 0x1:
		return "idle";
	case 0x2:
		return "clear";
	case 0x4:
		return "prepare";
	case 0x8:
		return "prepare";
	case 0x10:
		return "data running";
	case 0x20:
		return "eof wait";
	case 0x40:
		return "soft reset wait";
	case 0x80:
		return "eof done";
	case 0x100:
		return "sof reset done";
	case 0x200:
		return "frame complete";
	default:
		return "";
	}
}

void cmdq_mdp_dump_wdma(const unsigned long base, const char *label)
{
	uint32_t value[40] = { 0 };
	uint32_t state = 0;
	uint32_t grep = 0;	/* grep bit = 1, WDMA has sent request to SMI, and not receive done yet */
	uint32_t isFIFOFull = 0;	/* 1 for WDMA FIFO full */

	value[0] = CMDQ_REG_GET32(base + 0x014);
	value[1] = CMDQ_REG_GET32(base + 0x018);
	value[2] = CMDQ_REG_GET32(base + 0x028);
	value[3] = CMDQ_REG_GET32(base + cmdq_mdp_get_func()->wdmaGetRegOffsetDstAddr());
	value[4] = CMDQ_REG_GET32(base + 0x078);
	value[5] = CMDQ_REG_GET32(base + 0x080);
	value[6] = CMDQ_REG_GET32(base + 0x0A0);
	value[7] = CMDQ_REG_GET32(base + 0x0A8);

	CMDQ_REG_SET32(base + 0x014, (value[0] & (0x0FFFFFFF)));
	value[8] = CMDQ_REG_GET32(base + 0x014);
	value[9] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x10000000 | (value[0] & (0x0FFFFFFF)));
	value[10] = CMDQ_REG_GET32(base + 0x014);
	value[11] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x20000000 | (value[0] & (0x0FFFFFFF)));
	value[12] = CMDQ_REG_GET32(base + 0x014);
	value[13] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x30000000 | (value[0] & (0x0FFFFFFF)));
	value[14] = CMDQ_REG_GET32(base + 0x014);
	value[15] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x40000000 | (value[0] & (0x0FFFFFFF)));
	value[16] = CMDQ_REG_GET32(base + 0x014);
	value[17] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x50000000 | (value[0] & (0x0FFFFFFF)));
	value[18] = CMDQ_REG_GET32(base + 0x014);
	value[19] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x60000000 | (value[0] & (0x0FFFFFFF)));
	value[20] = CMDQ_REG_GET32(base + 0x014);
	value[21] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x70000000 | (value[0] & (0x0FFFFFFF)));
	value[22] = CMDQ_REG_GET32(base + 0x014);
	value[23] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x80000000 | (value[0] & (0x0FFFFFFF)));
	value[24] = CMDQ_REG_GET32(base + 0x014);
	value[25] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0x90000000 | (value[0] & (0x0FFFFFFF)));
	value[26] = CMDQ_REG_GET32(base + 0x014);
	value[27] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0xA0000000 | (value[0] & (0x0FFFFFFF)));
	value[28] = CMDQ_REG_GET32(base + 0x014);
	value[29] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0xB0000000 | (value[0] & (0x0FFFFFFF)));
	value[30] = CMDQ_REG_GET32(base + 0x014);
	value[31] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0xC0000000 | (value[0] & (0x0FFFFFFF)));
	value[32] = CMDQ_REG_GET32(base + 0x014);
	value[33] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0xD0000000 | (value[0] & (0x0FFFFFFF)));
	value[34] = CMDQ_REG_GET32(base + 0x014);
	value[35] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0xE0000000 | (value[0] & (0x0FFFFFFF)));
	value[36] = CMDQ_REG_GET32(base + 0x014);
	value[37] = CMDQ_REG_GET32(base + 0x0AC);
	CMDQ_REG_SET32(base + 0x014, 0xF0000000 | (value[0] & (0x0FFFFFFF)));
	value[38] = CMDQ_REG_GET32(base + 0x014);
	value[39] = CMDQ_REG_GET32(base + 0x0AC);

	CMDQ_ERR("=============== [CMDQ] %s Status ====================================\n", label);
	CMDQ_ERR("[CMDQ]WDMA_CFG: 0x%08x, WDMA_SRC_SIZE: 0x%08x, WDMA_DST_W_IN_BYTE = 0x%08x\n",
		 value[0], value[1], value[2]);
	CMDQ_ERR
	    ("[CMDQ]WDMA_DST_ADDR0: 0x%08x, WDMA_DST_UV_PITCH: 0x%08x, WDMA_DST_ADDR_OFFSET0 = 0x%08x\n",
	     value[3], value[4], value[5]);
	CMDQ_ERR("[CMDQ]WDMA_STATUS: 0x%08x, WDMA_INPUT_CNT: 0x%08x\n", value[6], value[7]);

	/* Dump Addtional WDMA debug info */
	CMDQ_ERR("WDMA_DEBUG_0 +014: 0x%08x , +0ac: 0x%08x\n", value[8], value[9]);
	CMDQ_ERR("WDMA_DEBUG_1 +014: 0x%08x , +0ac: 0x%08x\n", value[10], value[11]);
	CMDQ_ERR("WDMA_DEBUG_2 +014: 0x%08x , +0ac: 0x%08x\n", value[12], value[13]);
	CMDQ_ERR("WDMA_DEBUG_3 +014: 0x%08x , +0ac: 0x%08x\n", value[14], value[15]);
	CMDQ_ERR("WDMA_DEBUG_4 +014: 0x%08x , +0ac: 0x%08x\n", value[16], value[17]);
	CMDQ_ERR("WDMA_DEBUG_5 +014: 0x%08x , +0ac: 0x%08x\n", value[18], value[19]);
	CMDQ_ERR("WDMA_DEBUG_6 +014: 0x%08x , +0ac: 0x%08x\n", value[20], value[21]);
	CMDQ_ERR("WDMA_DEBUG_7 +014: 0x%08x , +0ac: 0x%08x\n", value[22], value[23]);
	CMDQ_ERR("WDMA_DEBUG_8 +014: 0x%08x , +0ac: 0x%08x\n", value[24], value[25]);
	CMDQ_ERR("WDMA_DEBUG_9 +014: 0x%08x , +0ac: 0x%08x\n", value[26], value[27]);
	CMDQ_ERR("WDMA_DEBUG_A +014: 0x%08x , +0ac: 0x%08x\n", value[28], value[29]);
	CMDQ_ERR("WDMA_DEBUG_B +014: 0x%08x , +0ac: 0x%08x\n", value[30], value[31]);
	CMDQ_ERR("WDMA_DEBUG_C +014: 0x%08x , +0ac: 0x%08x\n", value[32], value[33]);
	CMDQ_ERR("WDMA_DEBUG_D +014: 0x%08x , +0ac: 0x%08x\n", value[34], value[35]);
	CMDQ_ERR("WDMA_DEBUG_E +014: 0x%08x , +0ac: 0x%08x\n", value[36], value[37]);
	CMDQ_ERR("WDMA_DEBUG_F +014: 0x%08x , +0ac: 0x%08x\n", value[38], value[39]);

	/* parse WDMA state */
	state = value[6] & 0x3FF;
	grep = (value[6] >> 13) & 0x1;
	isFIFOFull = (value[6] >> 12) & 0x1;

	CMDQ_ERR("WDMA state:0x%x (%s)\n", state, cmdq_mdp_get_wdma_state(state));
	CMDQ_ERR("WDMA in_req:%d in_ack:%d\n", (value[6] >> 15) & 0x1, (value[6] >> 14) & 0x1);

	/* note WDMA send request(i.e command) to SMI first, then SMI takes request data from WDMA FIFO */
	/* if SMI dose not process request and upstream HWs */
	/* such as MDP_RSZ send data to WDMA, WDMA FIFO will full finally */
	CMDQ_ERR("WDMA grep:%d, FIFO full:%d\n", grep, isFIFOFull);
	CMDQ_ERR("WDMA suggest: Need SMI help:%d, Need check WDMA config:%d\n", (grep),
		 ((0 == grep) && (1 == isFIFOFull)));
}
