#include "cmdq_core.h"
#include "cmdq_reg.h"
#include "cmdq_mdp_common.h"

#include <linux/met_drv.h>
#ifndef CMDQ_USE_CCF
#include <mach/mt_clkmgr.h>
#endif				/* !defined(CMDQ_USE_CCF) */

#include <mach/m4u.h>

#include "cmdq_device.h"

typedef struct CmdqMdpModuleBaseVA {
	long MDP_RDMA0;
	long MDP_RDMA1;
	long MDP_RSZ0;
	long MDP_RSZ1;
	long MDP_RSZ2;
	long MDP_TDSHP0;
	long MDP_TDSHP1;
	long MDP_MOUT0;
	long MDP_MOUT1;
	long MDP_WROT0;
	long MDP_WROT1;
	long MDP_WDMA;
	long VENC;
} CmdqMdpModuleBaseVA;
static CmdqMdpModuleBaseVA gCmdqMdpModuleBaseVA;

#define IMP_ENABLE_MDP_HW_CLOCK(FN_NAME, HW_NAME)	\
uint32_t cmdq_mdp_enable_clock_##FN_NAME(bool enable)	\
{		\
	return cmdq_dev_enable_mtk_clock(enable, MT_CG_DISP0_##HW_NAME, "CMDQ_MDP_"#HW_NAME);	\
}
#define IMP_MDP_HW_CLOCK_IS_ENABLE(FN_NAME, HW_NAME)	\
bool cmdq_mdp_clock_is_enable_##FN_NAME(void)	\
{		\
	return cmdq_dev_mtk_clock_is_enable(MT_CG_DISP0_##HW_NAME);	\
}

IMP_ENABLE_MDP_HW_CLOCK(CAM_MDP, CAM_MDP);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RDMA0, MDP_RDMA0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RDMA1, MDP_RDMA1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RSZ0, MDP_RSZ0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RSZ1, MDP_RSZ1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_RSZ2, MDP_RSZ2);
IMP_ENABLE_MDP_HW_CLOCK(MDP_WDMA, MDP_WDMA);
IMP_ENABLE_MDP_HW_CLOCK(MDP_WROT0, MDP_WROT0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_WROT1, MDP_WROT1);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TDSHP0, MDP_TDSHP0);
IMP_ENABLE_MDP_HW_CLOCK(MDP_TDSHP1, MDP_TDSHP1);
IMP_MDP_HW_CLOCK_IS_ENABLE(CAM_MDP, CAM_MDP);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RDMA0, MDP_RDMA0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RDMA1, MDP_RDMA1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RSZ0, MDP_RSZ0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RSZ1, MDP_RSZ1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_RSZ2, MDP_RSZ2);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_WDMA, MDP_WDMA);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_WROT0, MDP_WROT0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_WROT1, MDP_WROT1);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TDSHP0, MDP_TDSHP0);
IMP_MDP_HW_CLOCK_IS_ENABLE(MDP_TDSHP1, MDP_TDSHP1);
#undef IMP_ENABLE_MDP_HW_CLOCK
#undef IMP_MDP_HW_CLOCK_IS_ENABLE

#define IMP_MDP_MODULE_BASE_VA(BASE_SYMBOL) \
const long cmdq_mdp_get_module_base_VA_##BASE_SYMBOL(void)	\
{	\
return gCmdqMdpModuleBaseVA.BASE_SYMBOL; \
}
IMP_MDP_MODULE_BASE_VA(MDP_RDMA0);
IMP_MDP_MODULE_BASE_VA(MDP_RDMA1);
IMP_MDP_MODULE_BASE_VA(MDP_RSZ0);
IMP_MDP_MODULE_BASE_VA(MDP_RSZ1);
IMP_MDP_MODULE_BASE_VA(MDP_RSZ2);
IMP_MDP_MODULE_BASE_VA(MDP_TDSHP0);
IMP_MDP_MODULE_BASE_VA(MDP_TDSHP1);
IMP_MDP_MODULE_BASE_VA(MDP_MOUT0);
IMP_MDP_MODULE_BASE_VA(MDP_MOUT1);
IMP_MDP_MODULE_BASE_VA(MDP_WROT0);
IMP_MDP_MODULE_BASE_VA(MDP_WROT1);
IMP_MDP_MODULE_BASE_VA(MDP_WDMA);
IMP_MDP_MODULE_BASE_VA(VENC);
#undef IMP_MDP_MODULE_BASE_VA

#ifdef CMDQ_OF_SUPPORT
#define MDP_RDMA0_BASE cmdq_mdp_get_module_base_VA_MDP_RDMA0()
#define MDP_RDMA1_BASE cmdq_mdp_get_module_base_VA_MDP_RDMA1()
#define MDP_RSZ0_BASE cmdq_mdp_get_module_base_VA_MDP_RSZ0()
#define MDP_RSZ1_BASE cmdq_mdp_get_module_base_VA_MDP_RSZ1()
#define MDP_RSZ2_BASE cmdq_mdp_get_module_base_VA_MDP_RSZ2()
#define MDP_TDSHP0_BASE cmdq_mdp_get_module_base_VA_MDP_TDSHP0()
#define MDP_TDSHP1_BASE cmdq_mdp_get_module_base_VA_MDP_TDSHP1()
#define MDP_MOUT0_BASE cmdq_mdp_get_module_base_VA_MDP_MOUT0()
#define MDP_MOUT1_BASE cmdq_mdp_get_module_base_VA_MDP_MOUT1()
#define MDP_WROT0_BASE cmdq_mdp_get_module_base_VA_MDP_WROT0()
#define MDP_WROT1_BASE cmdq_mdp_get_module_base_VA_MDP_WROT1()
#define MDP_WDMA_BASE cmdq_mdp_get_module_base_VA_MDP_WDMA()
#define VENC_BASE cmdq_mdp_get_module_base_VA_VENC()
#else
#include <mach/mt_reg_base.h>
#endif

typedef struct RegDef {
	int offset;
	const char *name;
} RegDef;

void cmdq_mdp_dump_mmsys_config(void)
{
	int i = 0;
	uint32_t value = 0;

	static const struct RegDef configRegisters[] = {
		{0x01c, "ISP_MOUT_EN"},
		{0x020, "MDP_RDMA_MOUT_EN"},
		{0x024, "MDP_PRZ0_MOUT_EN"},
		{0x028, "MDP_PRZ1_MOUT_EN"},
		{0x02C, "MDP_PRZ2_MOUT_EN"},
		{0x030, "MDP_TDSHP0_MOUT_EN"},
		{0x034, "MDP_TDSHP1_MOUT_EN"},
		{0x038, "MDP0_MOUT_EN"},
		{0x03C, "MDP1_MOUT_EN"},
		{0x040, "DISP_OVL0_MOUT_EN"},
		{0x044, "DISP_OVL1_MOUT_EN"},
		{0x048, "DISP_OD_MOUT_EN"},
		{0x04C, "DISP_GAMMA_MOUT_EN"},
		{0x050, "DISP_UFOE_MOUT_EN"},
		/* {0x054, "MMSYS_MOUT_RST"}, */
		{0x058, "MDP_PRZ0_SEL_IN"},
		{0x05C, "MDP_PRZ1_SEL_IN"},
		{0x060, "MDP_PRZ2_SEL_IN"},
		{0x064, "MDP_TDSHP0_SEL_IN"},
		{0x068, "MDP_TDSHP1_SEL_IN"},
		{0x06C, "MDP0_SEL_IN"},
		{0x070, "MDP1_SEL_IN"},
		{0x074, "MDP_CROP_SEL_IN"},
		{0x078, "MDP_WDMA_SEL_IN"},
		{0x07C, "MDP_WROT0_SEL_IN"},
		{0x080, "MDP_WROT1_SEL_IN"},
		{0x084, "DISP_COLOR0_SEL_IN"},
		{0x088, "DISP_COLOR1_SEL_IN"},
		{0x08C, "DISP_AAL_SEL_IN"},
		{0x090, "DISP_PATH0_SEL_IN"},
		{0x094, "DISP_PATH1_SEL_IN"},
		{0x098, "DISP_WDMA0_SEL_IN"},
		{0x09C, "DISP_WDMA1_SEL_IN"},
		{0x0A0, "DISP_UFOE_SEL_IN"},
		{0x0A4, "DSI0_SEL_IN"},
		{0x0A8, "DSI1_SEL_IN"},
		{0x0AC, "DPI_SEL_IN"},
		{0x0B0, "DISP_RDMA0_SOUT_SEL_IN"},
		{0x0B4, "DISP_RDMA1_SOUT_SEL_IN"},
		{0x0B8, "DISP_RDMA2_SOUT_SEL_IN"},
		{0x0BC, "DISP_COLOR0_SOUT_SEL_IN"},
		{0x0C0, "DISP_COLOR1_SOUT_SEL_IN"},
		{0x0C4, "DISP_PATH0_SOUT_SEL_IN"},
		{0x0C8, "DISP_PATH1_SOUT_SEL_IN"},
		{0x0F0, "MMSYS_MISC"},
		/* ACK and REQ related */
		{0x8b0, "DISP_DL_VALID_0"},
		{0x8b4, "DISP_DL_VALID_1"},
		{0x8b8, "DISP_DL_READY_0"},
		{0x8bc, "DISP_DL_READY_1"},
		{0x8c0, "MDP_DL_VALID_0"},
		{0x8c4, "MDP_DL_VALID_1"},
		{0x8c8, "MDP_DL_READY_0"},
		{0x8cc, "MDP_DL_READY_1"}
	};

	for (i = 0; i < sizeof(configRegisters) / sizeof(configRegisters[0]); ++i) {
		value = CMDQ_REG_GET16(MMSYS_CONFIG_BASE + configRegisters[i].offset);
		CMDQ_ERR("%s: 0x%08x\n", configRegisters[i].name, value);
	}
}

int32_t cmdq_mdp_reset_with_mmsys(const uint64_t engineToResetAgain)
{
	const long MMSYS_SW0_RST_B_REG = MMSYS_CONFIG_BASE + (0x140);
	int i = 0;
	uint32_t reset_bits = 0L;
	static const int engineResetBit[32] = {
		-1,		/* bit  0 : SMI COMMON */
		-1,		/* bit  1 : SMI LARB0 */
		CMDQ_ENG_MDP_CAMIN,	/* bit  2 : CAM_MDP */
		CMDQ_ENG_MDP_RDMA0,	/* bit  3 : MDP_RDMA0 */
		CMDQ_ENG_MDP_RDMA1,	/* bit  4 : MDP_RDMA1 */
		CMDQ_ENG_MDP_RSZ0,	/* bit  5 : MDP_RSZ0 */
		CMDQ_ENG_MDP_RSZ1,	/* bit  6 : MDP_RSZ1 */
		CMDQ_ENG_MDP_RSZ2,	/* bit  7 : MDP_RSZ2 */
		CMDQ_ENG_MDP_TDSHP0,	/* bit  8 : MDP_TDSHP0 */
		CMDQ_ENG_MDP_TDSHP1,	/* bit  9 : MDP_TDSHP1 */
		CMDQ_ENG_MDP_WDMA,	/* bit 10 : MDP_WDMA */
		CMDQ_ENG_MDP_WROT0,	/* bit 11 : MDP_WROT0 */
		CMDQ_ENG_MDP_WROT1,	/* bit 12 : MDP_WROT1 */
		CMDQ_ENG_JPEG_ENC,	/* bit 13 : MDP_CROP, note that we use JPEG_ENC */
		[14 ... 31] = -1
	};

	for (i = 0; i < 32; ++i) {
		if (0 > engineResetBit[i])
			continue;

		if (engineToResetAgain & (1LL << engineResetBit[i]))
			reset_bits |= (1 << i);
	}

	if (0 != reset_bits) {
		/* 0: reset */
		/* 1: not reset */
		/* so we need to reverse the bits */
		reset_bits = ~reset_bits;

		CMDQ_REG_SET32(MMSYS_SW0_RST_B_REG, reset_bits);
		CMDQ_REG_SET32(MMSYS_SW0_RST_B_REG, ~0);
		/* This takes effect immediately, no need to poll state */
	}

	return 0;
}

int32_t cmdqVEncDumpInfo(uint64_t engineFlag, int logLevel)
{
	if (engineFlag & (1LL << CMDQ_ENG_VIDEO_ENC))
		cmdq_mdp_dump_venc(VENC_BASE, "VENC");

	return 0;
}

void cmdq_mdp_init_module_base_VA(void)
{
	memset(&gCmdqMdpModuleBaseVA, 0, sizeof(CmdqMdpModuleBaseVA));

#ifdef CMDQ_OF_SUPPORT
	gCmdqMdpModuleBaseVA.MDP_RDMA0 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_rdma0");
	gCmdqMdpModuleBaseVA.MDP_RDMA1 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_rdma1");
	gCmdqMdpModuleBaseVA.MDP_RSZ0 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_rsz0");
	gCmdqMdpModuleBaseVA.MDP_RSZ1 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_rsz1");
	gCmdqMdpModuleBaseVA.MDP_RSZ2 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_rsz2");
	gCmdqMdpModuleBaseVA.MDP_WDMA = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_wdma");
	gCmdqMdpModuleBaseVA.MDP_WROT0 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_wrot0");
	gCmdqMdpModuleBaseVA.MDP_WROT1 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_wrot1");
	gCmdqMdpModuleBaseVA.MDP_TDSHP0 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_tdshp0");
	gCmdqMdpModuleBaseVA.MDP_TDSHP1 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mdp_tdshp1");
	gCmdqMdpModuleBaseVA.MDP_MOUT0 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_MOUT0");
	gCmdqMdpModuleBaseVA.MDP_MOUT1 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_MOUT1");
	gCmdqMdpModuleBaseVA.VENC = cmdq_dev_alloc_module_base_VA_by_name("mediatek,VENC");
#endif
}

void cmdq_mdp_deinit_module_base_VA(void)
{
#ifdef CMDQ_OF_SUPPORT
#define FREE_MODULE_BASE(SYMBOL) cmdq_dev_free_module_base_VA((cmdq_mdp_get_module_base_VA_##SYMBOL()))
	FREE_MODULE_BASE(MDP_RDMA0);
	FREE_MODULE_BASE(MDP_RDMA1);
	FREE_MODULE_BASE(MDP_RSZ0);
	FREE_MODULE_BASE(MDP_RSZ1);
	FREE_MODULE_BASE(MDP_RSZ2);
	FREE_MODULE_BASE(MDP_TDSHP0);
	FREE_MODULE_BASE(MDP_TDSHP1);
	FREE_MODULE_BASE(MDP_MOUT0);
	FREE_MODULE_BASE(MDP_MOUT1);
	FREE_MODULE_BASE(MDP_WROT0);
	FREE_MODULE_BASE(MDP_WROT1);
	FREE_MODULE_BASE(MDP_WDMA);
	FREE_MODULE_BASE(VENC);

	memset(&gCmdqMdpModuleBaseVA, 0, sizeof(CmdqMdpModuleBaseVA));
#else
	/* do nothing, registers' IOMAP will be destroyed by platform */
#endif
}

bool cmdq_mdp_clock_is_on(CMDQ_ENG_ENUM engine)
{
	switch (engine) {
	case CMDQ_ENG_MDP_CAMIN:
		return cmdq_mdp_clock_is_enable_CAM_MDP();
	case CMDQ_ENG_MDP_RDMA0:
		return cmdq_mdp_clock_is_enable_MDP_RDMA0();
	case CMDQ_ENG_MDP_RDMA1:
		return cmdq_mdp_clock_is_enable_MDP_RDMA1();
	case CMDQ_ENG_MDP_RSZ0:
		return cmdq_mdp_clock_is_enable_MDP_RSZ0();
	case CMDQ_ENG_MDP_RSZ1:
		return cmdq_mdp_clock_is_enable_MDP_RSZ1();
	case CMDQ_ENG_MDP_RSZ2:
		return cmdq_mdp_clock_is_enable_MDP_RSZ2();
	case CMDQ_ENG_MDP_WDMA:
		return cmdq_mdp_clock_is_enable_MDP_WDMA();
	case CMDQ_ENG_MDP_WROT0:
		return cmdq_mdp_clock_is_enable_MDP_WROT0();
	case CMDQ_ENG_MDP_WROT1:
		return cmdq_mdp_clock_is_enable_MDP_WROT1();
	case CMDQ_ENG_MDP_TDSHP0:
		return cmdq_mdp_clock_is_enable_MDP_TDSHP0();
	case CMDQ_ENG_MDP_TDSHP1:
		return cmdq_mdp_clock_is_enable_MDP_TDSHP1();
	default:
		CMDQ_ERR("try to query unknown mdp clock");
		return false;
	}
}

void cmdq_mdp_enable_clock(bool enable, CMDQ_ENG_ENUM engine)
{
	switch (engine) {
	case CMDQ_ENG_MDP_CAMIN:
		cmdq_mdp_enable_clock_CAM_MDP(enable);
		break;
	case CMDQ_ENG_MDP_RDMA0:
		cmdq_mdp_enable_clock_MDP_RDMA0(enable);
		break;
	case CMDQ_ENG_MDP_RDMA1:
		cmdq_mdp_enable_clock_MDP_RDMA1(enable);
		break;
	case CMDQ_ENG_MDP_RSZ0:
		cmdq_mdp_enable_clock_MDP_RSZ0(enable);
		break;
	case CMDQ_ENG_MDP_RSZ1:
		cmdq_mdp_enable_clock_MDP_RSZ1(enable);
		break;
	case CMDQ_ENG_MDP_RSZ2:
		cmdq_mdp_enable_clock_MDP_RSZ2(enable);
		break;
	case CMDQ_ENG_MDP_WDMA:
		cmdq_mdp_enable_clock_MDP_WDMA(enable);
		break;
	case CMDQ_ENG_MDP_WROT0:
		cmdq_mdp_enable_clock_MDP_WROT0(enable);
		break;
	case CMDQ_ENG_MDP_WROT1:
		cmdq_mdp_enable_clock_MDP_WROT1(enable);
		break;
	case CMDQ_ENG_MDP_TDSHP0:
		cmdq_mdp_enable_clock_MDP_TDSHP0(enable);
		break;
	case CMDQ_ENG_MDP_TDSHP1:
		cmdq_mdp_enable_clock_MDP_TDSHP1(enable);
		break;
	default:
		CMDQ_ERR("try to enable unknown mdp clock");
		break;
	}
}

int32_t cmdqMdpClockOn(uint64_t engineFlag)
{
	CMDQ_MSG("Enable MDP(0x%llx) clock begin\n", engineFlag);

#ifdef CMDQ_PWR_AWARE
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_CAMIN);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RDMA0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RDMA1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RSZ0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RSZ1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_RSZ2);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_TDSHP0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_TDSHP1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_WROT0);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_WROT1);
	cmdq_mdp_enable(engineFlag, CMDQ_ENG_MDP_WDMA);
#else
	CMDQ_MSG("Force MDP clock all on\n");

	/* enable all bits in MMSYS_CG_CLR0 and MMSYS_CG_CLR1 */
	CMDQ_REG_SET32(MMSYS_CONFIG_BASE + 0x108, 0xFFFFFFFF);
	CMDQ_REG_SET32(MMSYS_CONFIG_BASE + 0x118, 0xFFFFFFFF);

#endif				/* #ifdef CMDQ_PWR_AWARE */

	CMDQ_MSG("Enable MDP(0x%llx) clock end\n", engineFlag);

	return 0;
}

m4u_callback_ret_t cmdq_M4U_TranslationFault_callback(int port, unsigned	int	mva, void *data)
{
	CMDQ_ERR("================= [MDP M4U] Dump Begin ================\n");
	CMDQ_ERR("[MDP M4U]fault call port=%d, mva=0x%x", port, mva);

	cmdq_core_dump_tasks_info();


	switch (port) {
	case M4U_PORT_MDP_RDMA0:
		cmdq_mdp_dump_rdma(MDP_RDMA0_BASE, "RDMA0");
		break;
	case M4U_PORT_MDP_RDMA1:
		cmdq_mdp_dump_rdma(MDP_RDMA1_BASE, "RDMA1");
		break;
	case M4U_PORT_MDP_WDMA:
		cmdq_mdp_dump_wdma(MDP_WDMA_BASE, "WDMA");
		break;
	case M4U_PORT_MDP_WROT0:
		cmdq_mdp_dump_rot(MDP_WROT0_BASE, "WROT0");
		break;
	case M4U_PORT_MDP_WROT1:
		cmdq_mdp_dump_rot(MDP_WROT1_BASE, "WROT1");
		break;
	default:
		CMDQ_ERR("[MDP M4U]fault callback function");
		break;
	}

	CMDQ_ERR("================= [MDP M4U] Dump End ================\n");

	return M4U_CALLBACK_HANDLED;
}

typedef struct MODULE_BASE {
	uint64_t engine;
	long base;		/* considering 64 bit kernel, use long type to store base addr */
	const char *name;
} MODULE_BASE;

#define DEFINE_MODULE(eng, base) {eng, base, #eng}

int32_t cmdqMdpDumpInfo(uint64_t engineFlag, int logLevel)
{
	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA0))
		cmdq_mdp_dump_rdma(MDP_RDMA0_BASE, "RDMA0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA1))
		cmdq_mdp_dump_rdma(MDP_RDMA1_BASE, "RDMA1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ0))
		cmdq_mdp_get_func()->mdpDumpRsz(MDP_RSZ0_BASE, "RSZ0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ1))
		cmdq_mdp_get_func()->mdpDumpRsz(MDP_RSZ1_BASE, "RSZ1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ2))
		cmdq_mdp_get_func()->mdpDumpRsz(MDP_RSZ2_BASE, "RSZ2");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP0))
		cmdq_mdp_get_func()->mdpDumpTdshp(MDP_TDSHP0_BASE, "TDSHP0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP1))
		cmdq_mdp_get_func()->mdpDumpTdshp(MDP_TDSHP1_BASE, "TDSHP1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT0))
		cmdq_mdp_dump_rot(MDP_WROT0_BASE, "WROT0");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT1))
		cmdq_mdp_dump_rot(MDP_WROT1_BASE, "WROT1");

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WDMA))
		cmdq_mdp_dump_wdma(MDP_WDMA_BASE, "WDMA");

	/* verbose case, dump entire 1KB HW register region */
	/* for each enabled HW module. */
	if (logLevel >= 1) {
		int inner = 0;

		const MODULE_BASE bases[] = {
			DEFINE_MODULE(CMDQ_ENG_MDP_RDMA0, MDP_RDMA0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_RDMA1, MDP_RDMA1_BASE),

			DEFINE_MODULE(CMDQ_ENG_MDP_RSZ0, MDP_RSZ0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_RSZ1, MDP_RSZ1_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_RSZ2, MDP_RSZ2_BASE),

			DEFINE_MODULE(CMDQ_ENG_MDP_TDSHP0, MDP_TDSHP0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_TDSHP1, MDP_TDSHP1_BASE),

			DEFINE_MODULE(CMDQ_ENG_MDP_WROT0, MDP_WROT0_BASE),
			DEFINE_MODULE(CMDQ_ENG_MDP_WROT1, MDP_WROT1_BASE),

			DEFINE_MODULE(CMDQ_ENG_MDP_WDMA, MDP_WDMA_BASE),
		};

		for (inner = 0; inner < (sizeof(bases) / sizeof(bases[0])); ++inner) {
			if (engineFlag & (1LL << bases[inner].engine)) {
				CMDQ_ERR("========= [CMDQ] %s dump base 0x%lx ========\n",
					 bases[inner].name, bases[inner].base);
				print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
					       (void *)bases[inner].base, 1024, false);
			}
		}
	}

	return 0;
}

typedef enum MOUT_BITS {
	MOUT_BITS_ISP_MDP = 0,	/* bit  0: ISP_MDP multiple outupt reset */
	MOUT_BITS_MDP_RDMA0 = 1,	/* bit  1: MDP_RDMA0 multiple outupt reset */
	MOUT_BITS_MDP_PRZ0 = 2,	/* bit  2: MDP_PRZ0 multiple outupt reset */
	MOUT_BITS_MDP_PRZ1 = 3,	/* bit  3: MDP_PRZ1 multiple outupt reset */
	MOUT_BITS_MDP_PRZ2 = 4,	/* bit  4: MDP_PRZ2 multiple outupt reset */
	MOUT_BITS_MDP_TDSHP0 = 5,	/* bit  5: MDP_TDSHP0 multiple outupt reset */
	MOUT_BITS_MDP_TDSHP1 = 6,	/* bit  6: MDP_TDSHP1 multiple outupt reset */
	MOUT_BITS_MDP_MOUT0 = 7,	/* bit  7: MDP path 0 multiple outupt reset */
	MOUT_BITS_MDP_MOUT1 = 8,	/* bit  8: MDP path 1 multiple outupt reset */
} MOUT_BITS;

int32_t cmdqMdpResetEng(uint64_t engineFlag)
{
#ifndef CMDQ_PWR_AWARE
	return 0;
#else
	int status = 0;
	int64_t engineToResetAgain = 0LL;
	uint32_t mout_bits_old = 0L;
	uint32_t mout_bits = 0L;
	const long MMSYS_MOUT_RST_REG = MMSYS_CONFIG_BASE + (0x054);

	CMDQ_PROF_START(0, "MDP_Rst");
	CMDQ_VERBOSE("Reset MDP(0x%llx) begin\n", engineFlag);

	/* After resetting each component, */
	/* we need also reset corresponding MOUT config. */
	mout_bits_old = CMDQ_REG_GET32(MMSYS_MOUT_RST_REG);
	mout_bits = 0;

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA0)) {
		mout_bits |= (1 << MOUT_BITS_MDP_RDMA0);

		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_RDMA0,
					     MDP_RDMA0_BASE + 0x8,
					     MDP_RDMA0_BASE + 0x408, 0x7FF00, 0x100, false);
		if (0 != status)
			engineToResetAgain |= (1 << CMDQ_ENG_MDP_RDMA0);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA1)) {
		/* RDMA1 does not have MOUT configuration. */
		/* mout_bits |= (1 << MOUT_BITS_MDP_RDMA1); */
		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_RDMA1,
					     MDP_RDMA1_BASE + 0x8,
					     MDP_RDMA1_BASE + 0x408, 0x7FF00, 0x100, false);
		if (0 != status)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_RDMA1);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ0)) {
		mout_bits |= (1 << MOUT_BITS_MDP_PRZ0);
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ0)) {
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ1)) {
		mout_bits |= (1 << MOUT_BITS_MDP_PRZ1);
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ1)) {
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ2)) {
		mout_bits |= (1 << MOUT_BITS_MDP_PRZ2);
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ2)) {
			CMDQ_REG_SET32(MDP_RSZ2_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ2_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ2_BASE, 0x0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP0)) {
		mout_bits |= (1 << MOUT_BITS_MDP_TDSHP0);
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_TDSHP0)) {
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x2);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP1)) {
		mout_bits |= (1 << MOUT_BITS_MDP_TDSHP1);
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_TDSHP1)) {
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x0);
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x2);
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT0)) {
		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_WROT0,
					     MDP_WROT0_BASE + 0x010,
					     MDP_WROT0_BASE + 0x014, 0x1, 0x0, false);
		if (0 != status)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_WROT0);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT1)) {
		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_WROT1,
					     MDP_WROT1_BASE + 0x010,
					     MDP_WROT1_BASE + 0x014, 0x1, 0x0, false);
		if (0 != status)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_WROT1);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WDMA)) {
		status = cmdq_mdp_loop_reset(CMDQ_ENG_MDP_WDMA,
					     MDP_WDMA_BASE + 0x00C,
					     MDP_WDMA_BASE + 0x0A0, 0x3FF, 0x1, false);
		if (0 != status)
			engineToResetAgain |= (1LL << CMDQ_ENG_MDP_WDMA);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_CAMIN)) {
		/* MDP_CAMIN can only reset by mmsys, */
		/* so this is not a "error" */
		cmdq_mdp_reset_with_mmsys((1LL << CMDQ_ENG_MDP_CAMIN));
	}

	/*
	   when MDP engines fail to reset,
	   1. print SMI debug log
	   2. try resetting from MMSYS to restore system state
	   3. report to QA by raising AEE warning
	   this reset will reset all registers to power on state.
	   but DpFramework always reconfigures register values,
	   so there is no need to backup registers.
	 */
	if (0 != engineToResetAgain) {
		/* check SMI state immediately */
		/* if (1 == is_smi_larb_busy(0)) */
		/* { */
		/* smi_hanging_debug(5); */
		/* } */

		CMDQ_ERR("Reset failed MDP engines(0x%llx), reset again with MMSYS_SW0_RST_B\n",
			 engineToResetAgain);

		cmdq_mdp_reset_with_mmsys(engineToResetAgain);

		/* finally, raise AEE warning to report normal reset fail. */
		/* we hope that reset MMSYS. */
		CMDQ_AEE("MDP", "Disable 0x%llx engine failed\n", engineToResetAgain);

		status = -EFAULT;
	}

	/* Reset MOUT bits */
	if (engineFlag & (1LL << CMDQ_ENG_MDP_MOUT0))
		mout_bits |= (1 << MOUT_BITS_MDP_MOUT0);

	if (engineFlag & (1LL << CMDQ_ENG_MDP_MOUT1))
		mout_bits |= (1 << MOUT_BITS_MDP_MOUT1);

	/* MOUT configuration reset */
	CMDQ_REG_SET32(MMSYS_MOUT_RST_REG, (mout_bits_old & (~mout_bits)));
	CMDQ_REG_SET32(MMSYS_MOUT_RST_REG, (mout_bits_old | mout_bits));
	CMDQ_REG_SET32(MMSYS_MOUT_RST_REG, (mout_bits_old & (~mout_bits)));

	CMDQ_MSG("Reset MDP(0x%llx) end\n", engineFlag);
	CMDQ_PROF_END(0, "MDP_Rst");

	return status;

#endif				/* #ifdef CMDQ_PWR_AWARE */

}

int32_t cmdqMdpClockOff(uint64_t engineFlag)
{
#ifdef CMDQ_PWR_AWARE
	CMDQ_MSG("Disable MDP(0x%llx) clock begin\n", engineFlag);

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WDMA)) {
		cmdq_mdp_loop_off(CMDQ_ENG_MDP_WDMA,
				  MDP_WDMA_BASE + 0x00C, MDP_WDMA_BASE + 0X0A0, 0x3FF, 0x1, false);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT1)) {
		cmdq_mdp_loop_off(CMDQ_ENG_MDP_WROT1,
				  MDP_WROT1_BASE + 0X010, MDP_WROT1_BASE + 0X014, 0X1, 0X0, false);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_WROT0)) {
		cmdq_mdp_loop_off(CMDQ_ENG_MDP_WROT0,
				  MDP_WROT0_BASE + 0X010, MDP_WROT0_BASE + 0X014, 0X1, 0X0, false);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP1)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_TDSHP1)) {
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x0);
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x2);
			CMDQ_REG_SET32(MDP_TDSHP1_BASE + 0x100, 0x0);
			CMDQ_MSG("Disable MDP_TDSHP1 clock\n");
			cmdq_mdp_get_func()->enableMdpClock(false, CMDQ_ENG_MDP_TDSHP1);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_TDSHP0)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_TDSHP0)) {
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x2);
			CMDQ_REG_SET32(MDP_TDSHP0_BASE + 0x100, 0x0);
			CMDQ_MSG("Disable MDP_TDSHP0 clock\n");
			cmdq_mdp_get_func()->enableMdpClock(false, CMDQ_ENG_MDP_TDSHP0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ2)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ2)) {
			CMDQ_REG_SET32(MDP_RSZ2_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ2_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ2_BASE, 0x0);

			CMDQ_MSG("Disable MDP_RSZ2 clock\n");

			cmdq_mdp_get_func()->enableMdpClock(false, CMDQ_ENG_MDP_RSZ2);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ1)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ1)) {
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ1_BASE, 0x0);

			CMDQ_MSG("Disable MDP_RSZ1 clock\n");

			cmdq_mdp_get_func()->enableMdpClock(false, CMDQ_ENG_MDP_RSZ1);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RSZ0)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_RSZ0)) {
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x0);
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x10000);
			CMDQ_REG_SET32(MDP_RSZ0_BASE, 0x0);

			CMDQ_MSG("Disable MDP_RSZ0 clock\n");

			cmdq_mdp_get_func()->enableMdpClock(false, CMDQ_ENG_MDP_RSZ0);
		}
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA1)) {
		cmdq_mdp_loop_off(CMDQ_ENG_MDP_RDMA1,
				  MDP_RDMA1_BASE + 0x008,
				  MDP_RDMA1_BASE + 0x408, 0x7FF00, 0x100, false);
	}

	if (engineFlag & (1LL << CMDQ_ENG_MDP_RDMA0)) {
		cmdq_mdp_loop_off(CMDQ_ENG_MDP_RDMA0,
				  MDP_RDMA0_BASE + 0x008,
				  MDP_RDMA0_BASE + 0x408, 0x7FF00, 0x100, false);
	}


	if (engineFlag & (1LL << CMDQ_ENG_MDP_CAMIN)) {
		if (cmdq_mdp_get_func()->mdpClockIsOn(CMDQ_ENG_MDP_CAMIN)) {
			cmdq_mdp_reset_with_mmsys((1LL << CMDQ_ENG_MDP_CAMIN));

			CMDQ_MSG("Disable MDP_CAMIN clock\n");
			cmdq_mdp_get_func()->enableMdpClock(false, CMDQ_ENG_MDP_CAMIN);
		}
	}


	CMDQ_MSG("Disable MDP(0x%llx) clock end\n", engineFlag);
#endif				/* #ifdef CMDQ_PWR_AWARE */

	return 0;
}

void cmdqMdpInitialSetting(void)
{
	/* Register M4U Translation Fault function */
	m4u_register_fault_callback(M4U_PORT_MDP_RDMA0, cmdq_M4U_TranslationFault_callback, NULL);
	m4u_register_fault_callback(M4U_PORT_MDP_RDMA1, cmdq_M4U_TranslationFault_callback, NULL);
	m4u_register_fault_callback(M4U_PORT_MDP_WDMA, cmdq_M4U_TranslationFault_callback, NULL);
	m4u_register_fault_callback(M4U_PORT_MDP_WROT0, cmdq_M4U_TranslationFault_callback, NULL);
	m4u_register_fault_callback(M4U_PORT_MDP_WROT1, cmdq_M4U_TranslationFault_callback, NULL);
}


const uint32_t cmdq_mdp_rdma_get_reg_offset_src_addr(void)
{
	return 0xF00;
}

const uint32_t cmdq_mdp_wrot_get_reg_offset_dst_addr(void)
{
	return 0xF00;
}

const uint32_t cmdq_mdp_wdma_get_reg_offset_dst_addr(void)
{
	return 0xF00;
}

void testcase_clkmgr_mdp(void)
{
#ifndef CMDQ_USE_CCF
	/* RDMA clk test with src buffer addr */
	testcase_clkmgr_impl(MT_CG_DISP0_MDP_RDMA0,
			     "CMDQ_TEST_MDP_RDMA0",
			     MDP_RDMA0_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(),
			     0xAACCBBDD,
			     MDP_RDMA0_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(), true);

	testcase_clkmgr_impl(MT_CG_DISP0_MDP_RDMA1,
			     "CMDQ_TEST_MDP_RDMA1",
			     MDP_RDMA1_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(),
			     0xAACCBBDD,
			     MDP_RDMA1_BASE + cmdq_mdp_rdma_get_reg_offset_src_addr(), true);

	/* WDMA clk test with dst buffer addr */
	testcase_clkmgr_impl(MT_CG_DISP0_MDP_WDMA,
			     "CMDQ_TEST_MDP_WDMA",
			     MDP_WDMA_BASE + cmdq_mdp_wdma_get_reg_offset_dst_addr(),
			     0xAACCBBDD,
			     MDP_WDMA_BASE + cmdq_mdp_wdma_get_reg_offset_dst_addr(), true);

	/* WROT clk test with dst buffer addr */
	testcase_clkmgr_impl(MT_CG_DISP0_MDP_WROT0,
			     "CMDQ_TEST_MDP_WROT0",
			     MDP_WROT0_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(),
			     0xAACCBBDD,
			     MDP_WROT0_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(), true);

	testcase_clkmgr_impl(MT_CG_DISP0_MDP_WROT1,
			     "CMDQ_TEST_MDP_WROT1",
			     MDP_WROT1_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(),
			     0xAACCBBDD,
			     MDP_WROT1_BASE + cmdq_mdp_wrot_get_reg_offset_dst_addr(), true);

	/* TDSHP clk test with input size */
	testcase_clkmgr_impl(MT_CG_DISP0_MDP_TDSHP0,
			     "CMDQ_TEST_MDP_TDSHP0",
			     MDP_TDSHP0_BASE + 0x244, 0xAACCBBDD, MDP_TDSHP0_BASE + 0x244, true);

	testcase_clkmgr_impl(MT_CG_DISP0_MDP_TDSHP1,
			     "CMDQ_TEST_MDP_TDSHP1",
			     MDP_TDSHP1_BASE + 0x244, 0xAACCBBDD, MDP_TDSHP1_BASE + 0x244, true);

	/* RSZ clk test with debug port */
	testcase_clkmgr_impl(MT_CG_DISP0_MDP_RSZ0,
			     "CMDQ_TEST_MDP_RSZ0",
			     MDP_RSZ0_BASE + 0x040, 0x00000001, MDP_RSZ0_BASE + 0x044, false);

	testcase_clkmgr_impl(MT_CG_DISP0_MDP_RSZ1,
			     "CMDQ_TEST_MDP_RSZ1",
			     MDP_RSZ1_BASE + 0x040, 0x00000001, MDP_RSZ1_BASE + 0x044, false);

	testcase_clkmgr_impl(MT_CG_DISP0_MDP_RSZ2,
			     "CMDQ_TEST_MDP_RSZ2",
			     MDP_RSZ2_BASE + 0x040, 0x00000001, MDP_RSZ2_BASE + 0x044, false);
#endif				/* !defined(CMDQ_USE_CCF) */
}

void cmdq_mdp_platform_function_setting(void)
{
	cmdqMDPFuncStruct *pFunc;

	pFunc = cmdq_mdp_get_func();

	pFunc->dumpMMSYSConfig = cmdq_mdp_dump_mmsys_config;

	pFunc->vEncDumpInfo = cmdqVEncDumpInfo;

	pFunc->initModuleBaseVA = cmdq_mdp_init_module_base_VA;
	pFunc->deinitModuleBaseVA = cmdq_mdp_deinit_module_base_VA;

	pFunc->mdpClockIsOn = cmdq_mdp_clock_is_on;
	pFunc->enableMdpClock = cmdq_mdp_enable_clock;

	pFunc->mdpClockOn = cmdqMdpClockOn;
	pFunc->mdpDumpInfo = cmdqMdpDumpInfo;
	pFunc->mdpResetEng = cmdqMdpResetEng;
	pFunc->mdpClockOff = cmdqMdpClockOff;

	pFunc->mdpInitialSet = cmdqMdpInitialSetting;

	pFunc->rdmaGetRegOffsetSrcAddr = cmdq_mdp_rdma_get_reg_offset_src_addr;
	pFunc->wrotGetRegOffsetDstAddr = cmdq_mdp_wrot_get_reg_offset_dst_addr;
	pFunc->wdmaGetRegOffsetDstAddr = cmdq_mdp_wdma_get_reg_offset_dst_addr;
	pFunc->testcaseClkmgrMdp = testcase_clkmgr_mdp;
}
