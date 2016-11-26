#define LOG_TAG "DSI"

#include <linux/delay.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <debug.h>
#include "disp_drv_log.h"
#include "disp_drv_platform.h"

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
/* #include <mach/irqs.h> */
#include "mtkfb.h"
#include "ddp_drv.h"
#include "ddp_manager.h"
#include "ddp_dump.h"
#include "ddp_irq.h"
#include "ddp_dsi.h"
#include "ddp_log.h"
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include "ddp_mmp.h"

/* static unsigned int _dsi_reg_update_wq_flag = 0; */
static DECLARE_WAIT_QUEUE_HEAD(_dsi_reg_update_wq);
atomic_t PMaster_enable = ATOMIC_INIT(0);
#include "debug.h"

#include "ddp_reg.h"
#include "ddp_dsi.h"

/*...below is new dsi driver...*/
#define IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII 0
static int dsi_reg_op_debug;
#include <mach/sync_write.h>
#include "disp_helper.h"

extern bool is_ipoh_bootup;

#define DSI_OUTREG32(cmdq, addr, val) DISP_REG_SET(cmdq, addr, val)
/* #define DSI_OUTREG32(cmdq, addr, val) \ */
/* {\ */
/* DISPMSG("[dsi/reg]0x%08x=0x%08x\n", (int)disp_addr_convert(addr), val);\ */
/* DISP_REG_SET(cmdq, addr,val);} */

#define BIT_TO_VALUE(TYPE, bit) \
do { \
	TYPE r; \
	*(unsigned int *)(&r) = ((unsigned int)0x00000000); \
	r.bit = ~(r.bit); \
	r; \
} while (0)

#define DSI_MASKREG32(cmdq, REG, MASK, VALUE)	DISP_REG_MASK((cmdq), (REG), (VALUE), (MASK));

#define DSI_OUTREGBIT(cmdq, TYPE, REG, bit, value) \
do { \
	TYPE r; \
	TYPE v; \
	if (cmdq) { \
		*(unsigned int *)(&r) = ((unsigned int)0x00000000); \
		r.bit = ~(r.bit); \
		*(unsigned int *)(&v) = ((unsigned int)0x00000000); \
		v.bit = value; \
		DISP_REG_MASK(cmdq, &REG, AS_UINT32(&v), AS_UINT32(&r)); \
	} else { \
		r = *((TYPE *)&INREG32(&REG)); \
		r.bit = (value); \
		DISP_REG_SET(cmdq, &REG, AS_UINT32(&r)); \
	} \
} while (0)

#ifdef CONFIG_MTK_FPGA
#define MIPITX_INREG32(addr) \
({ \
	unsigned int val = 0; \
	if (0) \
		val = INREG32(addr); \
	if (dsi_reg_op_debug) { \
		pr_info("[mipitx/inreg]0x%08x=0x%08x\n", addr, val); \
	} \
	val; \
})

#define MIPITX_OUTREG32(addr, val) \
{\
	if (dsi_reg_op_debug) \
		; \
	if (0) \
		mt65xx_reg_sync_writel(val, addr); \
}

#define MIPITX_OUTREGBIT(TYPE, REG, bit, value) \
{\
	do { \
		TYPE r;\
		if (0) \
			r = *((TYPE *)&INREG32(&REG)); \
		*(unsigned int *)(&r) = ((unsigned int)0x00000000); \
		r.bit = value; \
		MIPITX_OUTREG32(&REG, AS_UINT32(&r)); \
	} while (0); \
}

#define MIPITX_MASKREG32(x, y, z)  MIPITX_OUTREG32(x, (MIPITX_INREG32(x)&~(y))|(z))
#else
#define MIPITX_INREG32(addr) \
({ \
	unsigned int val = 0; \
	val = INREG32(addr); \
	if (dsi_reg_op_debug) { \
		pr_info("[mipitx/inreg]0x%08x=0x%08x\n", addr, val); \
	} \
	val; \
})

#define MIPITX_OUTREG32(addr, val) DSI_OUTREG32(NULL, addr, val)

#define MIPITX_OUTREGBIT(TYPE, REG, bit, value) \
do { \
	TYPE r; \
	r = *((TYPE *)&INREG32(&REG)); \
	r.bit = value; \
	MIPITX_OUTREG32(&REG, AS_UINT32(&r)); \
} while (0)

#define MIPITX_MASKREG32(x, y, z)  MIPITX_OUTREG32(x, (MIPITX_INREG32(x)&~(y))|(z))
#endif



#define DSI_POLLREG32(cmdq, addr, mask, value) \
	cmdqRecPoll(cmdq, disp_addr_convert((unsigned long)addr), value, mask)

#define DSI_INREG32(type, addr) \
({ \
	unsigned int var = 0; \
	union p_regs { \
		type p_reg; \
		unsigned int *p_uint; \
	} p_temp1; \
	p_temp1.p_reg = (type)(addr); \
	var = INREG32(p_temp1.p_uint); \
	var; \
})

#define DSI_READREG32(type, dst, src) \
{ \
	union p_regs { \
		type p_reg; \
		unsigned int *p_uint; \
	} p_temp1, p_temp2; \
	p_temp1.p_reg = (type)(dst); \
	p_temp2.p_reg = (type)(src); \
	OUTREG32(p_temp1.p_uint, INREG32(p_temp2.p_uint)); \
}

typedef struct {
	LCM_INTERFACE_ID lcm_cmd_if;
	unsigned int lcm_width;
	unsigned int lcm_height;
	cmdqRecHandle *handle;
	bool enable;
	DSI_REGS regBackup;
	unsigned int cmdq_size;
	LCM_DSI_PARAMS dsi_params;
	int swap_port;
} t_dsi_context;

t_dsi_context _dsi_context[DSI_INTERFACE_NUM];
#define DSI_MODULE_BEGIN(x)	(x == DISP_MODULE_DSIDUAL?0:DSI_MODULE_to_ID(x))
#define DSI_MODULE_END(x)		(x == DISP_MODULE_DSIDUAL?1:DSI_MODULE_to_ID(x))
#define DSI_MODULE_to_ID(x)	(x == DISP_MODULE_DSI0?0:1)
#define DIFF_CLK_LANE_LP 0X10
static PDSI_REGS DSI_REG[2];
static PDSI_PHY_REGS DSI_PHY_REG[2];
static PDSI_CMDQ_REGS DSI_CMDQ_REG[2];
static PDSI_VM_CMDQ_REGS DSI_VM_CMD_REG[2];

static wait_queue_head_t _dsi_cmd_done_wait_queue[2];
static wait_queue_head_t _dsi_dcs_read_wait_queue[2];
static wait_queue_head_t _dsi_wait_bta_te[2];
static wait_queue_head_t _dsi_wait_ext_te[2];
static wait_queue_head_t _dsi_wait_vm_done_queue[2];
static wait_queue_head_t _dsi_wait_vm_cmd_done_queue[2];
static bool waitRDDone;
static bool wait_vm_cmd_done;

static int s_isDsiPowerOn;

static int dsi_currect_mode;

static void _DSI_INTERNAL_IRQ_Handler(DISP_MODULE_ENUM module, unsigned int param)
{
	int i = 0;
	DSI_INT_STATUS_REG status;
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* status = DSI_REG[i]->DSI_INTSTA; */
		status = *(PDSI_INT_STATUS_REG) & param;
		if (status.RD_RDY) {
			/* /write clear RD_RDY interrupt */

			/* / write clear RD_RDY interrupt must be before DSI_RACK */
			/* / because CMD_DONE will raise after DSI_RACK, */
			/* / so write clear RD_RDY after that will clear CMD_DONE too */
			/* do */
			/* { */
			/* /send read ACK */
			/* DSI_REG->DSI_RACK.DSI_RACK = 1; */
			/* DSI_OUTREGBIT(NULL, DSI_RACK_REG,DSI_REG[i]->DSI_RACK,DSI_RACK,1); */
			/* } while(DSI_REG[i]->DSI_INTSTA.BUSY); */
			waitRDDone = true;
			wake_up_interruptible(&_dsi_dcs_read_wait_queue[i]);
		}

		if (status.CMD_DONE)
			wake_up_interruptible(&_dsi_cmd_done_wait_queue[i]);

		if (status.TE_RDY)
			wake_up_interruptible(&_dsi_wait_bta_te[i]);

		if (status.EXT_TE)
			wake_up_interruptible(&_dsi_wait_ext_te[i]);

		if (status.VM_DONE)
			wake_up_interruptible(&_dsi_wait_vm_done_queue[i]);

		if (status.VM_CMD_DONE) {
			wait_vm_cmd_done = true;
			wake_up_interruptible(&_dsi_wait_vm_cmd_done_queue[i]);
		}
	}
}


static DSI_STATUS DSI_Reset(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_RESET, 1);
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_RESET, 0);
	}

	return DSI_STATUS_OK;
}

static int _dsi_is_video_mode(DISP_MODULE_ENUM module)
{
	int i = 0;
	DSI_MODE_CTRL_REG tmpreg;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (DSI_REG[i]->DSI_MODE_CTRL.MODE == CMD_MODE)
			return 0;
		else
			return 1;
	}
}

static DSI_STATUS DSI_SetMode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, MODE, mode);

	return DSI_STATUS_OK;
}

static DSI_STATUS DSI_SetVdoFrmMode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, FRM_MODE, mode);

	return DSI_STATUS_OK;
}


static DSI_STATUS DSI_SetSwitchMode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (mode == 0) {	/* V2C */
			/* DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG,DSI_REG[i]->DSI_MODE_CTRL,C2V_SWITCH_ON,0); */
			DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, V2C_SWITCH_ON, 1);
		} else		/* C2V */
			/* DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG,DSI_REG[i]->DSI_MODE_CTRL,V2C_SWITCH_ON,0); */
			DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, C2V_SWITCH_ON, 1);

	}

	return DSI_STATUS_OK;
}

static DSI_STATUS DSI_SetBypassRack(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int bypass)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (bypass == 0)
			DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[i]->DSI_RACK, DSI_RACK_BYPASS, 0);
		else
			DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[i]->DSI_RACK, DSI_RACK_BYPASS, 1);

	}

	return DSI_STATUS_OK;
}


DSI_STATUS DSI_DisableClk(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	DISPFUNC();
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_EN, 0);

	return DSI_STATUS_OK;
}

void DSI_lane0_ULP_mode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enter)
{
	int i = 0;

	ASSERT(cmdq == NULL);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enter) {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, L0_HS_TX_EN, 0);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, Lx_ULPM_AS_L0, 1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, L0_ULPM_EN, 0);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, L0_ULPM_EN, 1);
			mdelay(1);
		} else {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, L0_ULPM_EN, 0);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, Lx_ULPM_AS_L0, 0);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, L0_WAKEUP_EN, 1);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON, L0_WAKEUP_EN, 0);
			mdelay(1);
		}
	}
}


void DSI_clk_ULP_mode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enter)
{
	int i = 0;

	ASSERT(cmdq == NULL);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enter) {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON, LC_HS_TX_EN, 0);
			mdelay(1);

			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON, LC_ULPM_EN, 1);

			mdelay(1);
		} else {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON, LC_ULPM_EN, 0);
			mdelay(1);

			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON, LC_WAKEUP_EN, 1);
			mdelay(1);

			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON, LC_WAKEUP_EN, 0);
			mdelay(1);
		}
	}
}

bool DSI_clk_HS_state(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	DSI_PHY_LCCON_REG tmpreg;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_READREG32(PDSI_PHY_LCCON_REG, &tmpreg, &DSI_REG[i]->DSI_PHY_LCCON);
		return tmpreg.LC_HS_TX_EN ? TRUE : FALSE;
	}
}

void DSI_clk_HS_mode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enter)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enter) {	/* && !DSI_clk_HS_state(i, cmdq)) */
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON, LC_HS_TX_EN, 1);
		} else if (!enter) {	/* && DSI_clk_HS_state(i, cmdq)) */
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON, LC_HS_TX_EN, 0);
		}
	}
}

const char *_dsi_cmd_mode_parse_state(unsigned int state)
{
	switch (state) {
	case 0x0001:
		return "idle";
	case 0x0002:
		return "Reading command queue for header";
	case 0x0004:
		return "Sending type-0 command";
	case 0x0008:
		return "Waiting frame data from RDMA for type-1 command";
	case 0x0010:
		return "Sending type-1 command";
	case 0x0020:
		return "Sending type-2 command";
	case 0x0040:
		return "Reading command queue for data";
	case 0x0080:
		return "Sending type-3 command";
	case 0x0100:
		return "Sending BTA";
	case 0x0200:
		return "Waiting RX-read data ";
	case 0x0400:
		return "Waiting SW RACK for RX-read data";
	case 0x0800:
		return "Waiting TE";
	case 0x1000:
		return "Get TE ";
	case 0x2000:
		return "Waiting SW RACK for TE";
	case 0x4000:
		return "Waiting external TE";
	default:
		return "unknown";
	}
}

DSI_STATUS DSI_DumpRegisters(DISP_MODULE_ENUM module, int level)
{
	UINT32 i;

	if (level >= 0) {
		if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
			unsigned int DSI_DBG6_Status = (INREG32(DDP_REG_BASE_DSI0 + 0x160)) & 0xffff;
			DDPDUMP("DSI0 state:%s\n", _dsi_cmd_mode_parse_state(DSI_DBG6_Status));
		}

		if (module == DISP_MODULE_DSI1 || module == DISP_MODULE_DSIDUAL) {
			unsigned int DSI_DBG6_Status = (INREG32(DDP_REG_BASE_DSI1 + 0x160)) & 0xffff;
			DDPDUMP("DSI1 state:%s\n", _dsi_cmd_mode_parse_state(DSI_DBG6_Status));
		}
	}
	if (level >= 1) {
		if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
			unsigned int DSI_DBG6_Status = (INREG32(DDP_REG_BASE_DSI0 + 0x160)) & 0xffff;

			DDPDUMP("---------- Start dump DSI0 registers ----------\n");

			for (i = 0; i < sizeof(DSI_REGS); i += 16) {
				DDPDUMP("DSI+%04x : 0x%08x  0x%08x  0x%08x  0x%08x\n", i,
					INREG32(DDP_REG_BASE_DSI0 + i), INREG32(DDP_REG_BASE_DSI0 + i + 0x4),
					INREG32(DDP_REG_BASE_DSI0 + i + 0x8), INREG32(DDP_REG_BASE_DSI0 + i + 0xc));
			}

			for (i = 0; i < sizeof(DSI_CMDQ_REGS); i += 16) {
				DDPDUMP("DSI_CMD+%04x : 0x%08x  0x%08x  0x%08x  0x%08x\n", i,
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i)),
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i + 0x4)),
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i + 0x8)),
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i + 0xc)));
			}
#if 0
			if (disp_helper_get_stage() != DISP_HELPER_STAGE_EARLY_PORTING) {
				for (i = 0; i < sizeof(DSI_PHY_REGS); i += 16) {
					DDPDUMP("DSI_PHY+%04x : 0x%08x    0x%08x  0x%08x  0x%08x\n", i,
						INREG32((DDP_REG_BASE_MIPITX0 + i)),
						INREG32((DDP_REG_BASE_MIPITX0 + i + 0x4)),
						INREG32((DDP_REG_BASE_MIPITX0 + i + 0x8)),
						INREG32((DDP_REG_BASE_MIPITX0 + i + 0xc)));
				}
			}
#endif
		}

		if (module == DISP_MODULE_DSI1 || module == DISP_MODULE_DSIDUAL) {
			unsigned int DSI_DBG6_Status = (INREG32(DDP_REG_BASE_DSI1 + 0x160)) & 0xffff;

			DDPDUMP("---------- Start dump DSI1 registers ----------\n");

			for (i = 0; i < sizeof(DSI_REGS); i += 16) {
				DDPDUMP("DSI+%04x : 0x%08x  0x%08x  0x%08x  0x%08x\n", i,
					INREG32(DDP_REG_BASE_DSI1 + i), INREG32(DDP_REG_BASE_DSI1 + i + 0x4),
					INREG32(DDP_REG_BASE_DSI1 + i + 0x8), INREG32(DDP_REG_BASE_DSI1 + i + 0xc));
			}

			for (i = 0; i < sizeof(DSI_CMDQ_REGS); i += 16) {
				DDPDUMP("DSI_CMD+%04x : 0x%08x  0x%08x  0x%08x  0x%08x\n", i,
					INREG32((DDP_REG_BASE_DSI1 + 0x200 + i)),
					INREG32((DDP_REG_BASE_DSI1 + 0x200 + i + 0x4)),
					INREG32((DDP_REG_BASE_DSI1 + 0x200 + i + 0x8)),
					INREG32((DDP_REG_BASE_DSI1 + 0x200 + i + 0xc)));
			}
#if 0
			if (disp_helper_get_stage() != DISP_HELPER_STAGE_EARLY_PORTING) {
				for (i = 0; i < sizeof(DSI_PHY_REGS); i += 16) {
					DDPDUMP("DSI_PHY+%04x : 0x%08x    0x%08x  0x%08x  0x%08x\n", i,
						INREG32((DDP_REG_BASE_MIPITX1 + i)),
						INREG32((DDP_REG_BASE_MIPITX1 + i + 0x4)),
						INREG32((DDP_REG_BASE_MIPITX1 + i + 0x8)),
						INREG32((DDP_REG_BASE_MIPITX1 + i + 0xc)));
				}
			}
#endif
		}
	}

	return DSI_STATUS_OK;
}

int DSI_WaitVMDone(DISP_MODULE_ENUM module)
{
	int i = 0;
	static const long WAIT_TIMEOUT = 2 * HZ;	/* 2 sec */
	int ret = 0;

	/*...dsi video is always in busy state... */
	if (_dsi_is_video_mode(module)) {
		pr_info("DSI_WaitVMDone error: should set DSI to CMD mode firstly\n");
		return;
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		ret =
		    wait_event_interruptible_timeout(_dsi_wait_vm_done_queue[i], !(DSI_REG[i]->DSI_INTSTA.BUSY),
						     WAIT_TIMEOUT);
		if (0 == ret) {
			DISPERR("dsi wait VM done  timeout\n");
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
			return -1;
		}
	}
	return 0;
}

static void DSI_WaitForNotBusy(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	unsigned int count = 0;
	unsigned int tmp = 0;
	static const long WAIT_TIMEOUT = 2 * HZ;	/* 2 sec */
	int ret = 0;
#if 0
	if (module == DISP_MODULE_DSIDUAL) {
		if (_dsi_context[0].swap_port || _dsi_context[1].swap_port)
			module = DISP_MODULE_DSI1;
		else
			module = DISP_MODULE_DSI0;
	}
#endif
	if (cmdq) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
			DSI_POLLREG32(cmdq, &DSI_REG[i]->DSI_INTSTA, 0x80000000, 0x0);

		return;
	}


	/*...dsi video is always in busy state... */
	if (_dsi_is_video_mode(module))
		return;

	/* TODO: */
	/*
	   i = DSI_MODULE_BEGIN(module);
	   while(1)
	   {
	   tmp = INREG32(&DSI_REG[i]->DSI_INTSTA);
	   if(!(tmp &0x80000000))
	   break;

	   //if(count %1000)
	   //   DISPMSG("dsi state:0x%08x, 0x%08x\n", tmp, INREG32(&DSI_REG[i]->DSI_STATE_DBG6));

	   //msleep(1);

	   if(count++>1000000000)
	   {
	   DISPERR("dsi wait not busy timeout\n");
	   DSI_DumpRegisters(module,1);
	   DSI_Reset(module, NULL);
	   break;
	   }
	   }
	 */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		ret =
		    wait_event_interruptible_timeout(_dsi_cmd_done_wait_queue[i], !(DSI_REG[i]->DSI_INTSTA.BUSY),
						     WAIT_TIMEOUT);
		if (0 == ret) {
			DISPERR("dsi wait not busy timeout\n");
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
		}
	}
}

DSI_STATUS DSI_SleepOut(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;

	/* TODO: can we just start dsi0 for dsi dual? */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, SLEEP_MODE, 1);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON4_REG, DSI_REG[i]->DSI_PHY_TIMECON4, ULPS_WAKEUP, 0x22E09);
	}

	return DSI_STATUS_OK;
}


DSI_STATUS DSI_Wakeup(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;

	/* TODO: can we just start dsi0 for dsi dual? */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, SLEEPOUT_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, SLEEPOUT_START, 1);
		mdelay(1);

		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, SLEEPOUT_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, SLEEP_MODE, 0);
	}

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_BackupRegisters(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	DSI_REGS *regs = NULL;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		regs = &(_dsi_context[i].regBackup);

		DSI_OUTREG32(cmdq, &regs->DSI_INTEN, AS_UINT32(&DSI_REG[i]->DSI_INTEN));
		DSI_OUTREG32(cmdq, &regs->DSI_MODE_CTRL, AS_UINT32(&DSI_REG[i]->DSI_MODE_CTRL));
		DSI_OUTREG32(cmdq, &regs->DSI_TXRX_CTRL, AS_UINT32(&DSI_REG[i]->DSI_TXRX_CTRL));
		DSI_OUTREG32(cmdq, &regs->DSI_PSCTRL, AS_UINT32(&DSI_REG[i]->DSI_PSCTRL));

		DSI_OUTREG32(cmdq, &regs->DSI_VSA_NL, AS_UINT32(&DSI_REG[i]->DSI_VSA_NL));
		DSI_OUTREG32(cmdq, &regs->DSI_VBP_NL, AS_UINT32(&DSI_REG[i]->DSI_VBP_NL));
		DSI_OUTREG32(cmdq, &regs->DSI_VFP_NL, AS_UINT32(&DSI_REG[i]->DSI_VFP_NL));
		DSI_OUTREG32(cmdq, &regs->DSI_VACT_NL, AS_UINT32(&DSI_REG[i]->DSI_VACT_NL));

		DSI_OUTREG32(cmdq, &regs->DSI_HSA_WC, AS_UINT32(&DSI_REG[i]->DSI_HSA_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_HBP_WC, AS_UINT32(&DSI_REG[i]->DSI_HBP_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_HFP_WC, AS_UINT32(&DSI_REG[i]->DSI_HFP_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_BLLP_WC, AS_UINT32(&DSI_REG[i]->DSI_BLLP_WC));

		DSI_OUTREG32(cmdq, &regs->DSI_HSTX_CKL_WC, AS_UINT32(&DSI_REG[i]->DSI_HSTX_CKL_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_MEM_CONTI, AS_UINT32(&DSI_REG[i]->DSI_MEM_CONTI));

		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON0, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON0));
		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON1, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON1));
		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON2, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON2));
		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON3, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON3));
		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON4, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON4));

		DSI_OUTREG32(cmdq, &regs->DSI_VM_CMD_CON, AS_UINT32(&DSI_REG[i]->DSI_VM_CMD_CON));

		DSI_OUTREG32(cmdq, &regs->DSI_SWAP_PORT, AS_UINT32(&DSI_REG[i]->DSI_SWAP_PORT));

		pr_info("DSI_BackupRegisters VM_CMD_EN %d TS_VFP_EN %d\n", DSI_REG[i]->DSI_VM_CMD_CON.VM_CMD_EN,
		       DSI_REG[i]->DSI_VM_CMD_CON.TS_VFP_EN);
	}

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_RestoreRegisters(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	DSI_REGS *regs = NULL;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		regs = &(_dsi_context[i].regBackup);

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_INTEN, AS_UINT32(&regs->DSI_INTEN));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MODE_CTRL, AS_UINT32(&regs->DSI_MODE_CTRL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_TXRX_CTRL, AS_UINT32(&regs->DSI_TXRX_CTRL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PSCTRL, AS_UINT32(&regs->DSI_PSCTRL));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VSA_NL, AS_UINT32(&regs->DSI_VSA_NL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VBP_NL, AS_UINT32(&regs->DSI_VBP_NL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VFP_NL, AS_UINT32(&regs->DSI_VFP_NL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VACT_NL, AS_UINT32(&regs->DSI_VACT_NL));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSA_WC, AS_UINT32(&regs->DSI_HSA_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HBP_WC, AS_UINT32(&regs->DSI_HBP_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, AS_UINT32(&regs->DSI_HFP_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BLLP_WC, AS_UINT32(&regs->DSI_BLLP_WC));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, AS_UINT32(&regs->DSI_HSTX_CKL_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MEM_CONTI, AS_UINT32(&regs->DSI_MEM_CONTI));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, AS_UINT32(&regs->DSI_PHY_TIMECON0));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, AS_UINT32(&regs->DSI_PHY_TIMECON1));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, AS_UINT32(&regs->DSI_PHY_TIMECON2));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, AS_UINT32(&regs->DSI_PHY_TIMECON3));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON4, AS_UINT32(&regs->DSI_PHY_TIMECON4));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VM_CMD_CON, AS_UINT32(&regs->DSI_VM_CMD_CON));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_SWAP_PORT, AS_UINT32(&regs->DSI_SWAP_PORT));

		pr_info("DSI_RestoreRegisters VM_CMD_EN %d TS_VFP_EN %d\n", regs->DSI_VM_CMD_CON.VM_CMD_EN,
		       regs->DSI_VM_CMD_CON.TS_VFP_EN);
	}
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_BIST_Pattern_Test(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enable, unsigned int color)
{
	int i = 0;
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enable) {
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_PATTERN, color);
			/* DSI_OUTREG32(&DSI_REG->DSI_BIST_CON, AS_UINT32(&temp_reg)); */
			/* DSI_OUTREGBIT(DSI_BIST_CON_REG, DSI_REG->DSI_BIST_CON, SELF_PAT_MODE, 1); */
			DSI_OUTREGBIT(cmdq, DSI_BIST_CON_REG, DSI_REG[i]->DSI_BIST_CON, SELF_PAT_MODE, 1);

			if (!_dsi_is_video_mode(module)) {
				DSI_T0_INS t0;
				t0.CONFG = 0x09;
				t0.Data_ID = 0x39;
				t0.Data0 = 0x2c;
				t0.Data1 = 0;

				DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[i]->data[0], AS_UINT32(&t0));
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_CMDQ_SIZE, 1);

				/* DSI_OUTREGBIT(DSI_START_REG,DSI_REG->DSI_START,DSI_START,0); */
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_START, 0);
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_START, 1);
				/* DSI_OUTREGBIT(DSI_START_REG,DSI_REG->DSI_START,DSI_START,1); */
			}
		} else {
			/* if disable dsi pattern, need enable mutex, can't just start dsi */
			/* so we just disable pattern bit, do not start dsi here */
			/* DSI_WaitForNotBusy(module,cmdq); */
			/* DSI_OUTREGBIT(cmdq, DSI_BIST_CON_REG, DSI_REG[i]->DSI_BIST_CON, SELF_PAT_MODE, 0); */
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_CON, 0x00);
		}

	}
}

void DSI_Config_VDO_Timing(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int line_byte;
	unsigned int horizontal_sync_active_byte;
	unsigned int horizontal_backporch_byte;
	unsigned int horizontal_frontporch_byte;
	unsigned int horizontal_bllp_byte;
	unsigned int dsiTmpBufBpp;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (dsi_params->data_format.format == LCM_DSI_FORMAT_RGB565)
			dsiTmpBufBpp = 2;
		else
			dsiTmpBufBpp = 3;

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VSA_NL, dsi_params->vertical_sync_active);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VBP_NL, dsi_params->vertical_backporch);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VFP_NL, dsi_params->vertical_frontporch);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VACT_NL, dsi_params->vertical_active_line);

		line_byte =
		    (dsi_params->horizontal_sync_active + dsi_params->horizontal_backporch +
		     dsi_params->horizontal_frontporch + dsi_params->horizontal_active_pixel) * dsiTmpBufBpp;

		if (dsi_params->mode == SYNC_EVENT_VDO_MODE || dsi_params->mode == BURST_VDO_MODE
		    || dsi_params->switch_mode == SYNC_EVENT_VDO_MODE || dsi_params->switch_mode == BURST_VDO_MODE) {
			ASSERT((dsi_params->horizontal_backporch + dsi_params->horizontal_sync_active) * dsiTmpBufBpp >
			       9);
			horizontal_backporch_byte =
			    ((dsi_params->horizontal_backporch + dsi_params->horizontal_sync_active) * dsiTmpBufBpp -
			     10);
		} else {
			ASSERT(dsi_params->horizontal_sync_active * dsiTmpBufBpp > 9);
			horizontal_sync_active_byte = (dsi_params->horizontal_sync_active * dsiTmpBufBpp - 10);

			ASSERT(dsi_params->horizontal_backporch * dsiTmpBufBpp > 9);
			horizontal_backporch_byte = (dsi_params->horizontal_backporch * dsiTmpBufBpp - 10);
		}

		ASSERT(dsi_params->horizontal_frontporch * dsiTmpBufBpp > 11);
		horizontal_frontporch_byte = (dsi_params->horizontal_frontporch * dsiTmpBufBpp - 12);
		horizontal_bllp_byte = (dsi_params->horizontal_bllp * dsiTmpBufBpp);

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSA_WC, ALIGN_TO((horizontal_sync_active_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HBP_WC, ALIGN_TO((horizontal_backporch_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, ALIGN_TO((horizontal_frontporch_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BLLP_WC, ALIGN_TO((horizontal_bllp_byte), 4));
	}
}

int _dsi_ps_type_to_bpp(LCM_PS_TYPE ps)
{
	switch (ps) {
	case LCM_PACKED_PS_16BIT_RGB565:
		return 2;
	case LCM_LOOSELY_PS_18BIT_RGB666:
		return 3;
	case LCM_PACKED_PS_24BIT_RGB888:
		return 3;
	case LCM_PACKED_PS_18BIT_RGB666:
		return 3;
	}
}

DSI_STATUS DSI_PS_Control(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params, int w, int h)
{
	int i = 0;
	unsigned int ps_sel_bitvalue = 0;
	unsigned int ps_wc_adjust = 0;
	unsigned int ps_wc = 0;
	/* /TODO: parameter checking */
	ASSERT(dsi_params->PS <= PACKED_PS_18BIT_RGB666);

	if (dsi_params->PS > LOOSELY_PS_18BIT_RGB666)
		ps_sel_bitvalue = (5 - dsi_params->PS);
	else
		ps_sel_bitvalue = dsi_params->PS;


	if (module == DISP_MODULE_DSIDUAL)
		w = w / 2;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* vact setting */
		if (dsi_params->packet_size_mult && (h % dsi_params->packet_size_mult) == 0) {
			DSI_OUTREGBIT(cmdq, DSI_VACT_NL_REG, DSI_REG[i]->DSI_VACT_NL, VACT_NL,
				      h / dsi_params->packet_size_mult);
			ps_wc_adjust = 1;
		} else {
			DSI_OUTREGBIT(cmdq, DSI_VACT_NL_REG, DSI_REG[i]->DSI_VACT_NL, VACT_NL, h);
		}
		/* ps_wc setting */
		if (dsi_params->ufoe_enable && dsi_params->ufoe_params.lr_mode_en != 1) {
			if (dsi_params->ufoe_params.compress_ratio == 3) {	/* 1/3 */
				unsigned int ufoe_internal_width = w + w % 4;
				if (ufoe_internal_width % 3 == 0) {
					ps_wc = (ufoe_internal_width / 3) * _dsi_ps_type_to_bpp(dsi_params->PS);
				} else {
					unsigned int temp_w = ufoe_internal_width / 3 + 1;
					temp_w = ((temp_w % 2) == 1) ? (temp_w + 1) : temp_w;
					ps_wc = temp_w * _dsi_ps_type_to_bpp(dsi_params->PS);
				}
			} else {	/* 1/2 */
				ps_wc = (w + w % 4) / 2 * _dsi_ps_type_to_bpp(dsi_params->PS);
			}
		} else {
			ps_wc = w * _dsi_ps_type_to_bpp(dsi_params->PS);
		}

		if (ps_wc_adjust)
			ps_wc *= dsi_params->packet_size_mult;

		DSI_OUTREGBIT(cmdq, DSI_PSCTRL_REG, DSI_REG[i]->DSI_PSCTRL, DSI_PS_WC, ps_wc);

		DSI_OUTREGBIT(cmdq, DSI_PSCTRL_REG, DSI_REG[i]->DSI_PSCTRL, DSI_PS_SEL, ps_sel_bitvalue);
	}

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_TXRX_Control(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int lane_num_bitvalue = 0;
	bool cksm_en = TRUE;
	bool ecc_en = TRUE;
	int lane_num = dsi_params->LANE_NUM;
	int vc_num = 0;
	bool null_packet_en = FALSE;
	bool err_correction_en = FALSE;
	bool dis_eotp_en = FALSE;
	bool hstx_cklp_en = dsi_params->cont_clock ? FALSE : TRUE;
	int max_return_size = 0;

	switch (lane_num) {
	case LCM_ONE_LANE:
		lane_num_bitvalue = 0x1;
		break;
	case LCM_TWO_LANE:
		lane_num_bitvalue = 0x3;
		break;
	case LCM_THREE_LANE:
		lane_num_bitvalue = 0x7;
		break;
	case LCM_FOUR_LANE:
		lane_num_bitvalue = 0xF;
		break;
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, VC_NUM, vc_num);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, DIS_EOT, dis_eotp_en);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, BLLP_EN, null_packet_en);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, MAX_RTN_SIZE, max_return_size);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, HSTX_CKLP_EN, hstx_cklp_en);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, LANE_NUM, lane_num_bitvalue);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MEM_CONTI, DSI_WMEM_CONTI);

		if (CMD_MODE == dsi_params->mode)
			DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 1);
		else
			DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 0);

	}

	return DSI_STATUS_OK;
}

int MIPITX_IsEnabled(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	int ret = 0;
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_PLL_EN)
			ret++;
	}

	DISPMSG("MIPITX for %s is %s\n", ddp_get_module_name(module), ret ? "on" : "off");
	return ret;
}

int DSI_ClkIsEnabled(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	int ret = 0;
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (DSI_REG[i]->DSI_COM_CTRL.DSI_EN)
			ret++;
	}

	DISPMSG("%s is %s\n", ddp_get_module_name(module), ret ? "on" : "off");
	return ret;
}

unsigned int dsi_phy_get_clk(DISP_MODULE_ENUM module)
{
	int i = 0;
	int j = 0;
	unsigned int pcw = DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2.RG_DSI0_MPPLL_SDM_PCW_H;
	unsigned int prediv = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_PREDIV));
	unsigned int posdiv = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_POSDIV));
	unsigned int txdiv0 = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_TXDIV0));
	unsigned int txdiv1 = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_TXDIV1));

	DISPMSG("%s, pcw: %d, prediv: %d, posdiv: %d, txdiv0: %d, txdiv1: %d\n", __func__, pcw, prediv, posdiv, txdiv0,
		txdiv1);
	j = prediv * 4 * txdiv0 * txdiv1;
	if (j > 0)
		return 26 * pcw / j;

	return 0;
}

void DSI_PHY_clk_setting(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	DISPFUNC();
/* #ifdef MTKFB_FPGA_ONLY */
#if 0
	MIPITX_OUTREG32(0x10215044, 0x88492483);
	MIPITX_OUTREG32(0x10215040, 0x00000002);
	mdelay(10);
	MIPITX_OUTREG32(0x10215000, 0x00000403);
	MIPITX_OUTREG32(0x10215068, 0x00000003);
	MIPITX_OUTREG32(0x10215068, 0x00000001);

	mdelay(10);
	MIPITX_OUTREG32(0x10215050, 0x00000000);
	mdelay(10);
	MIPITX_OUTREG32(0x10215054, 0x00000003);
	MIPITX_OUTREG32(0x10215058, 0x60000000);
	MIPITX_OUTREG32(0x1021505c, 0x00000000);

	MIPITX_OUTREG32(0x10215004, 0x00000803);
	MIPITX_OUTREG32(0x10215008, 0x00000801);
	MIPITX_OUTREG32(0x1021500c, 0x00000801);
	MIPITX_OUTREG32(0x10215010, 0x00000801);
	MIPITX_OUTREG32(0x10215014, 0x00000801);

	MIPITX_OUTREG32(0x10215050, 0x00000001);

	mdelay(10);


	MIPITX_OUTREG32(0x10215064, 0x00000020);
	return 0;
	/* mipitx1 */

	MIPITX_OUTREG32(0x10216044, 0x88492483);
	MIPITX_OUTREG32(0x10216040, 0x00000002);
	mdelay(10);
	MIPITX_OUTREG32(0x10216000, 0x00000403);
	MIPITX_OUTREG32(0x10216068, 0x00000003);
	MIPITX_OUTREG32(0x10216068, 0x00000001);

	mdelay(10);
	MIPITX_OUTREG32(0x10216050, 0x00000000);
	mdelay(10);
	MIPITX_OUTREG32(0x10216054, 0x00000003);
	MIPITX_OUTREG32(0x10216058, 0x40000000);
	MIPITX_OUTREG32(0x1021605c, 0x00000000);

	MIPITX_OUTREG32(0x10216004, 0x00000803);
	MIPITX_OUTREG32(0x10216008, 0x00000801);
	MIPITX_OUTREG32(0x1021600c, 0x00000801);
	MIPITX_OUTREG32(0x10216010, 0x00000801);
	MIPITX_OUTREG32(0x10216014, 0x00000801);

	MIPITX_OUTREG32(0x10216050, 0x00000001);

	mdelay(10);


	MIPITX_OUTREG32(0x10216064, 0x00000020);
	return 0;
#endif

	int i = 0;
	unsigned int data_Rate = dsi_params->PLL_CLOCK * 2;
	unsigned int txdiv = 0;
	unsigned int txdiv0 = 0;
	unsigned int txdiv1 = 0;
	unsigned int pcw = 0;
/* unsigned int fmod = 30;//Fmod = 30KHz by default */
	unsigned int delta1 = 5;	/* Delta1 is SSC range, default is 0%~-5% */
	unsigned int pdelta1 = 0;
	u32 m_hw_res3 = 0;
	u32 temp1 = 0;
	u32 temp2 = 0;
	u32 temp3 = 0;
	u32 temp4 = 0;
	u32 temp5 = 0;

	/* temp1~5 is used for impedence calibration, not enable now */
#if 0
	m_hw_res3 = INREG32(0xF0206180);
	temp1 = (m_hw_res3 >> 28) & 0xF;
	temp2 = (m_hw_res3 >> 24) & 0xF;
	temp3 = (m_hw_res3 >> 20) & 0xF;
	temp4 = (m_hw_res3 >> 16) & 0xF;
	temp5 = (m_hw_res3 >> 12) & 0xF;
#endif

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* step 0 MIPITX lane swap setting */
		if (dsi_params->lane_swap_en) {
			DISPMSG("MIPITX Lane Swap Enabled for DSI Port %d\n", i);
			DISPMSG("MIPITX Lane Swap mapping: %d|%d|%d|%d|%d|%d\n",
				dsi_params->lane_swap[i][MIPITX_PHY_LANE_0],
				dsi_params->lane_swap[i][MIPITX_PHY_LANE_1],
				dsi_params->lane_swap[i][MIPITX_PHY_LANE_2],
				dsi_params->lane_swap[i][MIPITX_PHY_LANE_3],
				dsi_params->lane_swap[i][MIPITX_PHY_LANE_CK],
				dsi_params->lane_swap[i][MIPITX_PHY_LANE_RX]);
			MIPITX_OUTREGBIT(MIPITX_DSI_PHY_SEL_REG, DSI_PHY_REG[i]->MIPITX_DSI_PHY_SEL, MIPI_TX_PHY0_SEL,
					 dsi_params->lane_swap[i][MIPITX_PHY_LANE_0]);
			MIPITX_OUTREGBIT(MIPITX_DSI_PHY_SEL_REG, DSI_PHY_REG[i]->MIPITX_DSI_PHY_SEL, MIPI_TX_PHY1_SEL,
					 dsi_params->lane_swap[i][MIPITX_PHY_LANE_1]);
			MIPITX_OUTREGBIT(MIPITX_DSI_PHY_SEL_REG, DSI_PHY_REG[i]->MIPITX_DSI_PHY_SEL, MIPI_TX_PHY2_SEL,
					 dsi_params->lane_swap[i][MIPITX_PHY_LANE_2]);
			MIPITX_OUTREGBIT(MIPITX_DSI_PHY_SEL_REG, DSI_PHY_REG[i]->MIPITX_DSI_PHY_SEL, MIPI_TX_PHY3_SEL,
					 dsi_params->lane_swap[i][MIPITX_PHY_LANE_3]);
			MIPITX_OUTREGBIT(MIPITX_DSI_PHY_SEL_REG, DSI_PHY_REG[i]->MIPITX_DSI_PHY_SEL, MIPI_TX_PHYC_SEL,
					 dsi_params->lane_swap[i][MIPITX_PHY_LANE_CK]);
			MIPITX_OUTREGBIT(MIPITX_DSI_PHY_SEL_REG, DSI_PHY_REG[i]->MIPITX_DSI_PHY_SEL, MIPI_TX_LPRX_SEL,
					 dsi_params->lane_swap[i][MIPITX_PHY_LANE_RX]);
		}

		/* step 1 */
		/* MIPITX_MASKREG32(APMIXED_BASE+0x00, (0x1<<6), 1); */

		/* step 2 */
		MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON, RG_DSI_BG_CORE_EN, 1);
		MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON, RG_DSI_BG_CKEN, 1);

		/* step 3 */
		mdelay(1);

		/* step 4 */
		MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON, RG_DSI_LNT_HS_BIAS_EN, 1);

		/* step 5 */
		MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON, RG_DSI_CKG_LDOOUT_EN, 1);
		MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON, RG_DSI_LDOCORE_EN, 1);

		/* step 6 */
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
				 DA_DSI_MPPLL_SDM_PWR_ON, 1);

		/* step 7 */
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
				 DA_DSI_MPPLL_SDM_ISO_EN, 0);
		mdelay(1);

		if (0 != data_Rate) {
			if (data_Rate > 1250) {
				DISPCHECK("mipitx Data Rate exceed limitation(%d)\n", data_Rate);
				ASSERT(0);
			} else if (data_Rate >= 500) {
				txdiv = 1;
				txdiv0 = 0;
				txdiv1 = 0;
			} else if (data_Rate >= 250) {
				txdiv = 2;
				txdiv0 = 1;
				txdiv1 = 0;
			} else if (data_Rate >= 125) {
				txdiv = 4;
				txdiv0 = 2;
				txdiv1 = 0;
			} else if (data_Rate > 62) {
				txdiv = 8;
				txdiv0 = 2;
				txdiv1 = 1;
			} else if (data_Rate >= 50) {
				txdiv = 16;
				txdiv0 = 2;
				txdiv1 = 2;
			} else {
				DISPCHECK("dataRate is too low(%d)\n", data_Rate);
				ASSERT(0);
			}

			/* step 8 */
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_TXDIV0, txdiv0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_TXDIV1, txdiv1);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_PREDIV, 0);


			/* step 10 */
			/* PLL PCW config */
			/*
			   PCW bit 24~30 = floor(pcw)
			   PCW bit 16~23 = (pcw - floor(pcw))*256
			   PCW bit 8~15 = (pcw*256 - floor(pcw)*256)*256
			   PCW bit 8~15 = (pcw*256*256 - floor(pcw)*256*256)*256
			 */
			/* pcw = data_Rate*4*txdiv/(26*2);//Post DIV =4, so need data_Rate*4 */
			pcw = data_Rate * txdiv / 13;

			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_H, (pcw & 0x7F));
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_16_23, ((256 * (data_Rate * txdiv % 13) / 13) & 0xFF));
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_8_15,
					 ((256 * (256 * (data_Rate * txdiv % 13) % 13) / 13) & 0xFF));
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_0_7,
					 ((256 * (256 * (256 * (data_Rate * txdiv % 13) % 13) % 13) / 13) & 0xFF));

			if (1 != dsi_params->ssc_disable) {
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
						 RG_DSI0_MPPLL_SDM_SSC_PH_INIT, 1);
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
						 RG_DSI0_MPPLL_SDM_SSC_PRD, 0x1B1);
				if (0 != dsi_params->ssc_range)
					delta1 = dsi_params->ssc_range;

				ASSERT(delta1 <= 8);
				pdelta1 = (delta1 * data_Rate * txdiv * 262144 + 281664) / 563329;
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON3_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
						 RG_DSI0_MPPLL_SDM_SSC_DELTA, pdelta1);
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON3_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
						 RG_DSI0_MPPLL_SDM_SSC_DELTA1, pdelta1);
				DISPMSG("[dsi_drv.c] PLL config:data_rate=%d,txdiv=%d,pcw=%d,delta1=%d,pdelta1=0x%x\n",
					data_Rate, txdiv, DSI_INREG32(PMIPITX_DSI_PLL_CON2_REG,
								      &DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2), delta1,
					pdelta1);
			}
		} else {
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_TXDIV0, dsi_params->pll_div1);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_TXDIV1, dsi_params->pll_div2);

			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_H, ((dsi_params->fbk_div) << 2));
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_16_23, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_8_15, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_0_7, 0);

		}

		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1, RG_DSI0_MPPLL_SDM_FRA_EN,
				 1);


		/* step 11 */
		MIPITX_OUTREGBIT(MIPITX_DSI_CLOCK_LANE_REG, DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE,
				 RG_DSI_LNTC_LDOOUT_EN, 1);

		if (dsi_params->lane_swap_en) {
			if (dsi_params->lane_swap[i][MIPITX_PHY_LANE_CK] == MIPITX_PHY_LANE_CK) {
				MIPITX_OUTREGBIT(MIPITX_DSI_CLOCK_LANE_REG, DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE,
						 RG_DSI_LNTC_CKLANE_EN, 1);
			} else {
				MIPITX_OUTREGBIT(MIPITX_DSI_CLOCK_LANE_REG, DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE,
						 RG_DSI_LNTC_CKLANE_EN, 0);
			}
		}

		/* step 12 */
		if (dsi_params->LANE_NUM > 0) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE0_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0,
					 RG_DSI_LNT0_LDOOUT_EN, 1);
			if (dsi_params->lane_swap_en) {
				if (dsi_params->lane_swap[i][MIPITX_PHY_LANE_0] == MIPITX_PHY_LANE_CK) {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE0_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0, RG_DSI_LNT0_CKLANE_EN,
							 1);
				} else {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE0_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0, RG_DSI_LNT0_CKLANE_EN,
							 0);
				}
			}
		}
		/* step 13 */
		if (dsi_params->LANE_NUM > 1) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE1_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1,
					 RG_DSI_LNT1_LDOOUT_EN, 1);
			if (dsi_params->lane_swap_en) {
				if (dsi_params->lane_swap[i][MIPITX_PHY_LANE_1] == MIPITX_PHY_LANE_CK) {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE1_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1, RG_DSI_LNT1_CKLANE_EN,
							 1);
				} else {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE1_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1, RG_DSI_LNT1_CKLANE_EN,
							 0);
				}
			}
		}
		/* step 14 */
		if (dsi_params->LANE_NUM > 2) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE2_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2,
					 RG_DSI_LNT2_LDOOUT_EN, 1);
			if (dsi_params->lane_swap_en) {
				if (dsi_params->lane_swap[i][MIPITX_PHY_LANE_2] == MIPITX_PHY_LANE_CK) {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE2_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2, RG_DSI_LNT2_CKLANE_EN,
							 1);
				} else {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE2_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2, RG_DSI_LNT2_CKLANE_EN,
							 0);
				}
			}
		}
		/* step 15 */
		if (dsi_params->LANE_NUM > 3) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE3_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3,
					 RG_DSI_LNT3_LDOOUT_EN, 1);
			if (dsi_params->lane_swap_en) {
				if (dsi_params->lane_swap[i][MIPITX_PHY_LANE_3] == MIPITX_PHY_LANE_CK) {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE3_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3, RG_DSI_LNT3_CKLANE_EN,
							 1);
				} else {
					MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE3_REG,
							 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3, RG_DSI_LNT3_CKLANE_EN,
							 0);
				}
			}
		}
		/* step 16 */
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_PLL_EN, 1);

		/* step 17 */
		mdelay(1);

		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CHG_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG, RG_DSI0_MPPLL_SDM_PCW_CHG,
				 0);
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CHG_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG, RG_DSI0_MPPLL_SDM_PCW_CHG,
				 1);

		if ((0 != data_Rate) && (1 != dsi_params->ssc_disable))
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1, RG_DSI0_MPPLL_SDM_SSC_EN, 1);
		else
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1, RG_DSI0_MPPLL_SDM_SSC_EN, 0);

		/* step 18 */
		MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON, RG_DSI_PAD_TIE_LOW_EN, 0);

		mdelay(1);
	}
}



void DSI_PHY_clk_change(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	DISPFUNC();

	int i = 0;
	unsigned int j = 0;
	unsigned int data_Rate = dsi_params->PLL_CLOCK * 2;
	unsigned int txdiv = 0;
	unsigned int txdiv0 = 0;
	unsigned int txdiv1 = 0;
	unsigned int pcw = 0;
	unsigned int delta1 = 5;	/* Delta1 is SSC range, default is 0%~-5% */
	unsigned int pdelta1 = 0;

	DISPCHECK("New mipitx Data Rate=%d\n", data_Rate);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (0 != data_Rate) {
			if (data_Rate > 1250) {
				DISPCHECK("mipitx Data Rate exceed limitation(%d)\n", data_Rate);
				ASSERT(0);
			} else if (data_Rate >= 500) {
				txdiv = 1;
				txdiv0 = 0;
				txdiv1 = 0;
			} else if (data_Rate >= 250) {
				txdiv = 2;
				txdiv0 = 1;
				txdiv1 = 0;
			} else if (data_Rate >= 125) {
				txdiv = 4;
				txdiv0 = 2;
				txdiv1 = 0;
			} else if (data_Rate > 62) {
				txdiv = 8;
				txdiv0 = 2;
				txdiv1 = 1;
			} else if (data_Rate >= 50) {
				txdiv = 16;
				txdiv0 = 2;
				txdiv1 = 2;
			} else {
				DISPCHECK("dataRate is too low(%d)\n", data_Rate);
				ASSERT(0);
			}

			/* 1. PLL TXDIV Config */
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
				      RG_DSI0_MPPLL_TXDIV0, txdiv0);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
				      RG_DSI0_MPPLL_TXDIV1, txdiv1);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
				      RG_DSI0_MPPLL_PREDIV, 0);


			/* 2. PLL PCW Config */
			/*
			   PCW bit 24~30 = floor(pcw)
			   PCW bit 16~23 = (pcw - floor(pcw))*256
			   PCW bit 8~15 = (pcw*256 - floor(pcw)*256)*256
			   PCW bit 8~15 = (pcw*256*256 - floor(pcw)*256*256)*256
			 */
			/* pcw = data_Rate*4*txdiv/(26*2);//Post DIV =4, so need data_Rate*4 */
			pcw = data_Rate * txdiv / 13;

			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_H, (pcw & 0x7F));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_16_23, ((256 * (data_Rate * txdiv % 13) / 13) & 0xFF));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_8_15,
				      ((256 * (256 * (data_Rate * txdiv % 13) % 13) / 13) & 0xFF));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_0_7,
				      ((256 * (256 * (256 * (data_Rate * txdiv % 13) % 13) % 13) / 13) & 0xFF));

			/* 3. SSC Config */
			if (1 != dsi_params->ssc_disable) {
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
					      RG_DSI0_MPPLL_SDM_SSC_PH_INIT, 1);
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
					      RG_DSI0_MPPLL_SDM_SSC_PRD, 0x1B1);
				if (0 != dsi_params->ssc_range)
					delta1 = dsi_params->ssc_range;

				ASSERT(delta1 <= 8);
				pdelta1 = (delta1 * data_Rate * txdiv * 262144 + 281664) / 563329;
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON3_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
					      RG_DSI0_MPPLL_SDM_SSC_DELTA, pdelta1);
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON3_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
					      RG_DSI0_MPPLL_SDM_SSC_DELTA1, pdelta1);
				DISPMSG("[dsi_drv.c] PLL config:data_rate=%d,txdiv=%d,pcw=%d,delta1=%d,pdelta1=0x%x\n",
					data_Rate, txdiv, DSI_INREG32(PMIPITX_DSI_PLL_CON2_REG,
								      &DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2), delta1,
					pdelta1);
			}

		} else {	/* not use */

			/* 1. PLL TXDIV Config */
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
				      RG_DSI0_MPPLL_TXDIV0, dsi_params->pll_div1);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
				      RG_DSI0_MPPLL_TXDIV1, dsi_params->pll_div2);

			/* 2. PLL PCW Config */
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_H, ((dsi_params->fbk_div) << 2));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_16_23, 0);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_8_15, 0);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				      RG_DSI0_MPPLL_SDM_PCW_0_7, 0);

			/* 3. SSC Config */
			/* why no ssc config ??? */

		}

		DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_FRA_EN, 1);

		/* 4. delay >30ns, need check, cmdq writewithmask 80ns *  9 */
		/*for (j = 0; j < 10; j++)*/
			/* DSI_OUTREG32(cmdq,&DSI_CMDQ_REG[0]->data[127],1); */

		/* 5. DSI_PLL_CHG = 0 */
		DSI_OUTREG32(cmdq, &DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG, 0);


		/* 6. delay >20us , need check */
		for (j = 0; j < 250; j++)
			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[0]->data[126], 1);

		/* 7. DSI_PLL_CHG = 1 */
		DSI_OUTREG32(cmdq, &DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG, 1);


	}
}

void DSI_PHY_TIMCONFIG(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_EARLY_PORTING) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, 0x0B010102);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, 0x120A0308);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, 0x0A020000);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, 0x00041201);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON4, 0x00023000);
		}
		return;
	}

	DSI_PHY_TIMCON0_REG timcon0;
	DSI_PHY_TIMCON1_REG timcon1;
	DSI_PHY_TIMCON2_REG timcon2;
	DSI_PHY_TIMCON3_REG timcon3;
	unsigned int div1 = 0;
	unsigned int div2 = 0;
	unsigned int pre_div = 0;
	unsigned int post_div = 0;
	unsigned int fbk_sel = 0;
	unsigned int fbk_div = 0;
	unsigned int lane_no = dsi_params->LANE_NUM;

	/* unsigned int div2_real; */
	unsigned int cycle_time;
	unsigned int ui;
	unsigned int hs_trail_m, hs_trail_n;

	if (0 != dsi_params->PLL_CLOCK) {
		ui = 1000 / (dsi_params->PLL_CLOCK * 2) + 0x01;
		cycle_time = 8000 / (dsi_params->PLL_CLOCK * 2) + 0x01;
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
		"[DISP] - kernel - DSI_PHY_TIMCONFIG, Cycle Time = %d(ns), Unit Interval = %d(ns). , lane# = %d\n",
			cycle_time, ui, lane_no);
	} else {
		div1 = dsi_params->pll_div1;
		div2 = dsi_params->pll_div2;
		fbk_div = dsi_params->fbk_div;
		switch (div1) {
		case 0:
			div1 = 1;
			break;

		case 1:
			div1 = 2;
			break;

		case 2:
		case 3:
			div1 = 4;
			break;

		default:
			pr_info("div1 should be less than 4!!\n");
			div1 = 4;
			break;
		}

		switch (div2) {
		case 0:
			div2 = 1;
			break;
		case 1:
			div2 = 2;
			break;
		case 2:
		case 3:
			div2 = 4;
			break;
		default:
			pr_info("div2 should be less than 4!!\n");
			div2 = 4;
			break;
		}

		switch (pre_div) {
		case 0:
			pre_div = 1;
			break;

		case 1:
			pre_div = 2;
			break;

		case 2:
		case 3:
			pre_div = 4;
			break;

		default:
			pr_info("pre_div should be less than 4!!\n");
			pre_div = 4;
			break;
		}

		switch (post_div) {
		case 0:
			post_div = 1;
			break;

		case 1:
			post_div = 2;
			break;

		case 2:
		case 3:
			post_div = 4;
			break;

		default:
			pr_info("post_div should be less than 4!!\n");
			post_div = 4;
			break;
		}

		switch (fbk_sel) {
		case 0:
			fbk_sel = 1;
			break;

		case 1:
			fbk_sel = 2;
			break;

		case 2:
		case 3:
			fbk_sel = 4;
			break;

		default:
			pr_info("fbk_sel should be less than 4!!\n");
			fbk_sel = 4;
			break;
		}
		cycle_time = (1000 * 4 * div2 * div1) / (fbk_div * 26) + 0x01;

		ui = (1000 * div2 * div1) / (fbk_div * 26 * 0x2) + 0x01;
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
		"[DISP] - kernel - DSI_PHY_TIMCONFIG, Cycle Time = %d(ns), Unit Interval = %d(ns).", cycle_time, ui);
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
		" div1 = %d, div2 = %d, fbk_div = %d, lane# = %d\n", div1, div2, fbk_div, lane_no);
	}

	/* div2_real=div2 ? div2*0x02 : 0x1; */
	/* cycle_time = (1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26) + 1; */
	/* ui = (1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26 * 2) + 1; */
#define NS_TO_CYCLE(n, c)	((n) / (c))

	hs_trail_m = 1;
	hs_trail_n =
	    (dsi_params->HS_TRAIL == 0) ? NS_TO_CYCLE(((hs_trail_m * 0x4) + 60), cycle_time) : dsi_params->HS_TRAIL;
	/* +3 is recommended from designer becauase of HW latency */
	timcon0.HS_TRAIL = ((hs_trail_m > hs_trail_n) ? hs_trail_m : hs_trail_n) + 0x0a;

	timcon0.HS_PRPR = (dsi_params->HS_PRPR == 0) ? NS_TO_CYCLE((0x40 + 0x5 * ui), cycle_time) : dsi_params->HS_PRPR;
	/* HS_PRPR can't be 1. */
	if (timcon0.HS_PRPR == 0)
		timcon0.HS_PRPR = 1;

	timcon0.HS_ZERO =
	    (dsi_params->HS_ZERO == 0) ? NS_TO_CYCLE((0xC8 + 0x0a * ui), cycle_time) : dsi_params->HS_ZERO;
	if (timcon0.HS_ZERO > timcon0.HS_PRPR)
		timcon0.HS_ZERO -= timcon0.HS_PRPR;

	timcon0.LPX = (dsi_params->LPX == 0) ? NS_TO_CYCLE(0x50, cycle_time) : dsi_params->LPX;
	if (timcon0.LPX == 0)
		timcon0.LPX = 1;

	/* timcon1.TA_SACK      = (dsi_params->TA_SACK == 0) ? 1 : dsi_params->TA_SACK; */
	timcon1.TA_GET = (dsi_params->TA_GET == 0) ? (0x5 * timcon0.LPX) : dsi_params->TA_GET;
	timcon1.TA_SURE = (dsi_params->TA_SURE == 0) ? (0x3 * timcon0.LPX / 0x2) : dsi_params->TA_SURE;
	timcon1.TA_GO = (dsi_params->TA_GO == 0) ? (0x4 * timcon0.LPX) : dsi_params->TA_GO;
	/* -------------------------------------------------------------- */
	/* NT35510 need fine tune timing */
	/* Data_hs_exit = 60 ns + 128UI */
	/* Clk_post = 60 ns + 128 UI. */
	/* -------------------------------------------------------------- */
	timcon1.DA_HS_EXIT =
	    (dsi_params->DA_HS_EXIT == 0) ? NS_TO_CYCLE((0x3c + 0x80 * ui), cycle_time) : dsi_params->DA_HS_EXIT;

	timcon2.CLK_TRAIL =
	    ((dsi_params->CLK_TRAIL == 0) ? NS_TO_CYCLE(0x64, cycle_time) : dsi_params->CLK_TRAIL) + 0x0a;
	/* CLK_TRAIL can't be 1. */
	if (timcon2.CLK_TRAIL < 2)
		timcon2.CLK_TRAIL = 2;

	/* timcon2.LPX_WAIT     = (dsi_params->LPX_WAIT == 0) ? 1 : dsi_params->LPX_WAIT; */
	timcon2.CONT_DET = dsi_params->CONT_DET;
	timcon2.CLK_ZERO = (dsi_params->CLK_ZERO == 0) ? NS_TO_CYCLE(0x190, cycle_time) : dsi_params->CLK_ZERO;

	timcon3.CLK_HS_PRPR = (dsi_params->CLK_HS_PRPR == 0) ? NS_TO_CYCLE(0x40, cycle_time) : dsi_params->CLK_HS_PRPR;
	if (timcon3.CLK_HS_PRPR == 0)
		timcon3.CLK_HS_PRPR = 1;
	timcon3.CLK_HS_EXIT = (dsi_params->CLK_HS_EXIT == 0) ? (2 * timcon0.LPX) : dsi_params->CLK_HS_EXIT;
	timcon3.CLK_HS_POST =
	    (dsi_params->CLK_HS_POST == 0) ? NS_TO_CYCLE((0x3c + 0x80 * ui), cycle_time) : dsi_params->CLK_HS_POST;

	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
		"[DISP] - kernel - DSI_PHY_TIMCONFIG, HS_TRAIL = %d, HS_ZERO = %d, HS_PRPR = %d, LPX = %d,",
		timcon0.HS_TRAIL, timcon0.HS_ZERO, timcon0.HS_PRPR, timcon0.LPX);
	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
		" TA_GET = %d, TA_SURE = %d, TA_GO = %d, CLK_TRAIL = %d, CLK_ZERO = %d, CLK_HS_PRPR = %d\n",
		timcon1.TA_GET, timcon1.TA_SURE, timcon1.TA_GO,
		timcon2.CLK_TRAIL, timcon2.CLK_ZERO, timcon3.CLK_HS_PRPR);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, LPX, timcon0.LPX);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, HS_PRPR, timcon0.HS_PRPR);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, HS_ZERO, timcon0.HS_ZERO);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, HS_TRAIL, timcon0.HS_TRAIL);

		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, TA_GO, timcon1.TA_GO);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, TA_SURE, timcon1.TA_SURE);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, TA_GET, timcon1.TA_GET);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, DA_HS_EXIT, timcon1.DA_HS_EXIT);

		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON2_REG, DSI_REG[i]->DSI_PHY_TIMECON2, CONT_DET, timcon2.CONT_DET);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON2_REG, DSI_REG[i]->DSI_PHY_TIMECON2, CLK_ZERO, timcon2.CLK_ZERO);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON2_REG, DSI_REG[i]->DSI_PHY_TIMECON2, CLK_TRAIL, timcon2.CLK_TRAIL);

		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON3_REG, DSI_REG[i]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
			      timcon3.CLK_HS_PRPR);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON3_REG, DSI_REG[i]->DSI_PHY_TIMECON3, CLK_HS_POST,
			      timcon3.CLK_HS_POST);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON3_REG, DSI_REG[i]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
			      timcon3.CLK_HS_EXIT);
		DISPCHECK("%s, 0x%08x,0x%08x,0x%08x,0x%08x\n", __func__, INREG32(&DSI_REG[i]->DSI_PHY_TIMECON0),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON1), INREG32(&DSI_REG[i]->DSI_PHY_TIMECON2),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON3));
	}
}


void DSI_PHY_clk_switch(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, int on)
{
	int i = 0;

	/* can't use cmdq for this */
	ASSERT(cmdq == NULL);

	if (on) {
		DSI_PHY_clk_setting(module, cmdq, &(_dsi_context[i].dsi_params));
	} else {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			/* disable mipi clock */
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_PLL_EN, 0);
			mdelay(1);
			MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON,
					 RG_DSI_PAD_TIE_LOW_EN, 1);


			MIPITX_OUTREGBIT(MIPITX_DSI_CLOCK_LANE_REG, DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE,
					 RG_DSI_LNTC_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE0_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0,
					 RG_DSI_LNT0_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE1_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1,
					 RG_DSI_LNT1_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE2_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2,
					 RG_DSI_LNT2_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE3_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3,
					 RG_DSI_LNT3_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
					 DA_DSI_MPPLL_SDM_ISO_EN, 1);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
					 DA_DSI_MPPLL_SDM_PWR_ON, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON,
					 RG_DSI_LNT_HS_BIAS_EN, 0);

			MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON, RG_DSI_CKG_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON, RG_DSI_LDOCORE_EN, 0);

			MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON, RG_DSI_BG_CKEN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON, RG_DSI_BG_CORE_EN,
					 0);

			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_PREDIV, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_TXDIV0, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_TXDIV1, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					 RG_DSI0_MPPLL_POSDIV, 0);


			MIPITX_OUTREG32(&DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1, 0x00000000);
			MIPITX_OUTREG32(&DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2, 0x50000000);
			mdelay(1);
		}
	}
}

DSI_STATUS DSI_EnableClk(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	DISPFUNC();
	int i = 0;
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_EN, 1);

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_Start(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	if (module != DISP_MODULE_DSIDUAL) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, DSI_START, 0);
			DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, DSI_START, 1);
		}
	} else {
		/* TODO: do we need this? */
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 1);
	}
	return DSI_STATUS_OK;
}

void DSI_Set_VM_CMD(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{

	int i = 0;
	if (module != DISP_MODULE_DSIDUAL) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON, TS_VFP_EN, 1);
			DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON, VM_CMD_EN, 1);
			pr_info("DSI_Set_VM_CMD");
		}
	} else {
		DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON, TS_VFP_EN, 1);
		DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON, VM_CMD_EN, 1);
	}

	return;
}

DSI_STATUS DSI_EnableVM_CMD(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	if (module != DISP_MODULE_DSIDUAL) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
			DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		}
	} else {
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, VM_CMD_START, 1);
	}
	return DSI_STATUS_OK;
}


/* / return value: the data length we got */
UINT32 DSI_dcs_read_lcm_reg_v2(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, UINT8 cmd, UINT8 *buffer,
			       UINT8 buffer_size)
{
	int d = 0;
	UINT32 max_try_count = 5;
	UINT32 recv_data_cnt;	/* return value */
	unsigned char packet_type;
	DSI_RX_DATA_REG read_data0;
	DSI_RX_DATA_REG read_data1;
	DSI_RX_DATA_REG read_data2;
	DSI_RX_DATA_REG read_data3;
	DSI_T0_INS t0;
	DSI_T0_INS t1;
	static const long WAIT_TIMEOUT = 2 * HZ;	/* 2 sec */
	long ret;
	int timeout = 0;
	if (_dsi_context[0].swap_port || _dsi_context[1].swap_port)
		module = DISP_MODULE_DSI1;

	for (d = DSI_MODULE_BEGIN(module); d <= DSI_MODULE_END(module); d++) {
		DISPCHECK("DSI Read Module_Num:%d\n", d);
		if (DSI_REG[d]->DSI_MODE_CTRL.MODE) {
			/* only support cmd mode read */
			DISPCHECK("DSI Read Fail: DSI Mode is %d\n", DSI_REG[d]->DSI_MODE_CTRL.MODE);
			return 0;
		}

		if (buffer == NULL || buffer_size == 0) {
			/* illegal parameters */
			DISPCHECK("DSI Read Fail: buffer=0x%lx and buffer_size=%d\n", (unsigned long)buffer,
				  (unsigned int)buffer_size);
			return 0;
		}

		do {
			if (max_try_count == 0) {
				DISPCHECK("DSI Read Fail: try 5 times\n");
				return 0;
			}

			max_try_count--;
			recv_data_cnt = 0;

			/* 1. wait dsi not busy => can't read if dsi busy */
			DSI_WaitForNotBusy(module, cmdq);

			/* 2. Check rd_rdy & cmd_done irq */
			if (DSI_REG[d]->DSI_INTEN.RD_RDY == 0)
				DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTEN, RD_RDY, 1);

			if (DSI_REG[d]->DSI_INTEN.CMD_DONE == 0)
				DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTEN, CMD_DONE, 1);

			if (DSI_REG[d]->DSI_INTSTA.RD_RDY != 0 || DSI_REG[d]->DSI_INTSTA.CMD_DONE != 0) {
				/* dump cmdq & rxdata */
				{
					unsigned int i;
					DISPCHECK("Last DSI Read Why not clear irq???\n");
					DISPCHECK("DSI_CMDQ_SIZE  : %d\n", AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
					for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE; i++) {
						DISPCHECK("DSI_CMDQ_DATA%d : 0x%08x\n", i,
							  AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
					}
					DISPCHECK("DSI_RX_DATA0   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
					DISPCHECK("DSI_RX_DATA1   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
					DISPCHECK("DSI_RX_DATA2   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
					DISPCHECK("DSI_RX_DATA3   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));
				}
				/* clear irq */
				DSI_OUTREGBIT(cmdq, DSI_INT_STATUS_REG, DSI_REG[d]->DSI_INTSTA, RD_RDY, 0);
				DSI_OUTREGBIT(cmdq, DSI_INT_STATUS_REG, DSI_REG[d]->DSI_INTSTA, CMD_DONE, 0);
			}


			/* 3. Send cmd */
			t0.CONFG = 0x04;	/* /BTA */
			/* / 0xB0 is used to distinguish DCS cmd or Gerneric cmd, is that Right??? */
			t0.Data_ID = (cmd < 0xB0) ? DSI_DCS_READ_PACKET_ID : DSI_GERNERIC_READ_LONG_PACKET_ID;
			t0.Data0 = cmd;
			t0.Data1 = 0;

			/* set max return size */
			t1.CONFG = 0x00;
			t1.Data_ID = 0x37;
			t1.Data0 = buffer_size <= 10 ? buffer_size : 10;
			t1.Data1 = 0;

			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0], AS_UINT32(&t1));
			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[1], AS_UINT32(&t0));
			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 2);

			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 0);
			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 1);

			/* / the following code is to */
			/* / 1: wait read ready */
			/* / 2: ack read ready */
			/* / 3: wait for CMDQ_DONE(interrupt handler do this op) */
			/* / 4: read data */
			ret = wait_event_interruptible_timeout(_dsi_dcs_read_wait_queue[d], waitRDDone, WAIT_TIMEOUT);
			waitRDDone = false;
			if (ret > 0) {
				do {
					timeout++;
					udelay(1);
					DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
				} while (DSI_REG[d]->DSI_INTSTA.BUSY && (timeout < 1000));

				if (timeout == 1000) {
					/* wait cmd done timeout */
					DISPERR("DSI Read Fail: dsi wait cmd done timeout\n");
					DSI_DumpRegisters(module, 2);

					/* /do necessary reset here */
					DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
					DSI_Reset(module, NULL);

					/* clear rd rdy interrupt */
					DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTSTA, RD_RDY, 0);
					return 0;
				}
			} else if (ret == 0) {
				/* wait read ready timeout */
				DISPERR("DSI Read Fail: dsi wait read ready timeout\n");
				DSI_DumpRegisters(module, 2);

				/* /do necessary reset here */
				DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
				DSI_Reset(module, NULL);
				return 0;
			} else {
				/* wake up by other interrupt, need try again??? */
				DISPERR("DSI Read Fail: dsi wait read ready wake up by other interrupt\n");

				/* /do necessary reset here */
				DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
				DSI_Reset(module, NULL);
				return 0;
			}


			/* clear interrupt */
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTSTA, RD_RDY, 0);
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTSTA, CMD_DONE, 0);

			/* read data */
			DSI_OUTREG32(cmdq, &read_data0, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
			DSI_OUTREG32(cmdq, &read_data1, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
			DSI_OUTREG32(cmdq, &read_data2, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
			DSI_OUTREG32(cmdq, &read_data3, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));


			{
				unsigned int i;
				DISPCHECK("DSI read begin i = %d --------------------\n", 5 - max_try_count);
				DISPCHECK("DSI_RX_STA     : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_TRIG_STA));
				DISPCHECK("DSI_CMDQ_SIZE  : %d\n", AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
				for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE; i++) {
					DISPCHECK("DSI_CMDQ_DATA%d : 0x%08x\n", i,
						  AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
				}
				DISPCHECK("DSI_RX_DATA0   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
				DISPCHECK("DSI_RX_DATA1   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
				DISPCHECK("DSI_RX_DATA2   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
				DISPCHECK("DSI_RX_DATA3   : 0x%08x\n", AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));
				DISPCHECK("DSI read end ----------------------------\n");
			}

			packet_type = read_data0.byte0;

			DISPCHECK("DSI read packet_type is 0x%x\n", packet_type);

			/* 0x02: acknowledge & error report */
			/* 0x11: generic short read response(1 byte return) */
			/* 0x12: generic short read response(2 byte return) */
			/* 0x1a: generic long read response */
			/* 0x1c: dcs long read response */
			/* 0x21: dcs short read response(1 byte return) */
			/* 0x22: dcs short read response(2 byte return) */
			if (packet_type == 0x1A || packet_type == 0x1C) {	/* long read response */
				recv_data_cnt = read_data0.byte1 + read_data0.byte2 * 16;
				if (recv_data_cnt > 10) {
					DISPCHECK("DSI read long packet data exceeds 4 bytes return size: %d\n",
						  recv_data_cnt);
					recv_data_cnt = 10;
				}

				if (recv_data_cnt > buffer_size) {
					DISPCHECK("DSI read long packet data exceeds buffer size return size %d\n",
						  recv_data_cnt);
					recv_data_cnt = buffer_size;
				}
				DISPCHECK("DSI read long packet size: %d\n", recv_data_cnt);

				if (recv_data_cnt <= 4) {
					memcpy((void *)buffer, (void *)&read_data1, recv_data_cnt);
				} else if (recv_data_cnt <= 8) {
					memcpy((void *)buffer, (void *)&read_data1, 4);
					memcpy((void *)buffer + 4, (void *)&read_data2, recv_data_cnt - 4);
				} else {
					memcpy((void *)buffer, (void *)&read_data1, 4);
					memcpy((void *)buffer + 4, (void *)&read_data2, 4);
					memcpy((void *)buffer + 8, (void *)&read_data2, recv_data_cnt - 8);
				}
			} else if (packet_type == 0x11 || packet_type == 0x12 ||
				   packet_type == 0x21 || packet_type == 0x22) { /* short read response */
				if (packet_type == 0x11 || packet_type == 0x21)
					recv_data_cnt = 1;
				else
					recv_data_cnt = 2;

				if (recv_data_cnt > buffer_size) {
					DISPCHECK("DSI read short packet data exceeds buffer size: %d\n", buffer_size);
					recv_data_cnt = buffer_size;
					memcpy((void *)buffer, (void *)&read_data0.byte1, recv_data_cnt);
				} else {
					memcpy((void *)buffer, (void *)&read_data0.byte1, recv_data_cnt);
				}

			} else if (packet_type == 0x02) {
				DISPCHECK("read return type is 0x02, re-read\n");
			} else {
				DISPCHECK("read return type is non-recognite, type = 0x%x\n", packet_type);
				return 0;
			}
		} while (packet_type == 0x02);
		/* / here: we may receive a ACK packet which packet type is 0x02 (incdicates some error happened) */
		/* / therefore we try re-read again until no ACK packet */
		/* / But: if it is a good way to keep re-trying ??? */
	}

	return recv_data_cnt;
}

void DSI_set_cmdq_V2(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned cmd, unsigned char count,
		     unsigned char *para_list, unsigned char force_update)
{
	UINT32 i = 0;
	int d = 0;
	unsigned long goto_addr, mask_para, set_para;
	DSI_T0_INS t0;
	DSI_T2_INS t2;
	/* DISPFUNC(); */

	if (_dsi_context[0].swap_port || _dsi_context[1].swap_port)
		module = DISP_MODULE_DSI1;

	for (d = DSI_MODULE_BEGIN(module); d <= DSI_MODULE_END(module); d++) {
		if (0 != DSI_REG[d]->DSI_MODE_CTRL.MODE) {	/* not in cmd mode */
			DSI_VM_CMD_CON_REG vm_cmdq;
			memset(&vm_cmdq, 0, sizeof(DSI_VM_CMD_CON_REG));
			DSI_READREG32(PDSI_VM_CMD_CON_REG, &vm_cmdq, &DSI_REG[d]->DSI_VM_CMD_CON);
			if (cmd < 0xB0) {
				if (count > 1) {
					vm_cmdq.LONG_PKT = 1;
					vm_cmdq.CM_DATA_ID = DSI_DCS_LONG_PACKET_ID;
					vm_cmdq.CM_DATA_0 = count + 1;
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));

					goto_addr = (&DSI_VM_CMD_REG[d]->data[0].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr = (&DSI_VM_CMD_REG[d]->data[0].byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para = (para_list[i] << ((goto_addr & 0x3) * 8));
						DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);
					}
				} else {
					vm_cmdq.LONG_PKT = 0;
					vm_cmdq.CM_DATA_0 = cmd;
					if (count) {
						vm_cmdq.CM_DATA_ID = DSI_DCS_SHORT_PACKET_ID_1;
						vm_cmdq.CM_DATA_1 = para_list[0];
					} else {
						vm_cmdq.CM_DATA_ID = DSI_DCS_SHORT_PACKET_ID_0;
						vm_cmdq.CM_DATA_1 = 0;
					}
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
				}
			} else {
				if (count > 1) {
					vm_cmdq.LONG_PKT = 1;
					vm_cmdq.CM_DATA_ID = DSI_GERNERIC_LONG_PACKET_ID;
					vm_cmdq.CM_DATA_0 = count + 1;
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));

					goto_addr = (&DSI_VM_CMD_REG[d]->data[0].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr = (&DSI_VM_CMD_REG[d]->data[0].byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para = (para_list[i] << ((goto_addr & 0x3) * 8));
						DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);
					}
				} else {
					vm_cmdq.LONG_PKT = 0;
					vm_cmdq.CM_DATA_0 = cmd;
					if (count) {
						vm_cmdq.CM_DATA_ID = DSI_GERNERIC_SHORT_PACKET_ID_2;
						vm_cmdq.CM_DATA_1 = para_list[0];
					} else {
						vm_cmdq.CM_DATA_ID = DSI_GERNERIC_SHORT_PACKET_ID_1;
						vm_cmdq.CM_DATA_1 = 0;
					}
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
				}
			}
			/* start DSI VM CMDQ */
			if (force_update)
				DSI_EnableVM_CMD(module, cmdq);
		} else {
#ifdef ENABLE_DSI_ERROR_REPORT
			if ((para_list[0] & 1)) {
				memset(_dsi_cmd_queue, 0, sizeof(_dsi_cmd_queue));
				memcpy(_dsi_cmd_queue, para_list, count);
				_dsi_cmd_queue[(count + 3) / 4 * 4] = 0x4;
				count = (count + 3) / 4 * 4 + 4;
				para_list = (unsigned char *)_dsi_cmd_queue;
			} else {
				para_list[0] |= 4;
			}
#endif
			DSI_WaitForNotBusy(module, cmdq);

			if (cmd < 0xB0) {
				if (count > 1) {
					t2.CONFG = 2;
					t2.Data_ID = DSI_DCS_LONG_PACKET_ID;
					t2.WC16 = count + 1;

					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0], AS_UINT32(&t2));

					goto_addr = (&DSI_CMDQ_REG[d]->data[1].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr = (&DSI_CMDQ_REG[d]->data[1].byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para = (para_list[i] << ((goto_addr & 0x3) * 8));
						DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);
					}

					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 2 + (count) / 4);
				} else {
					t0.CONFG = 0;
					t0.Data0 = cmd;
					if (count) {
						t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_1;
						t0.Data1 = para_list[0];
					} else {
						t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_0;
						t0.Data1 = 0;
					}

					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0], AS_UINT32(&t0));
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 1);
				}
			} else {
				if (count > 1) {
					t2.CONFG = 2;
					t2.Data_ID = DSI_GERNERIC_LONG_PACKET_ID;
					t2.WC16 = count + 1;

					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0], AS_UINT32(&t2));

					goto_addr = (&DSI_CMDQ_REG[d]->data[1].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr = (&DSI_CMDQ_REG[d]->data[1].byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para = (para_list[i] << ((goto_addr & 0x3) * 8));
						DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);
					}

					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 2 + (count) / 4);

				} else {
					t0.CONFG = 0;
					t0.Data0 = cmd;
					if (count) {
						t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_2;
						t0.Data1 = para_list[0];
					} else {
						t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_1;
						t0.Data1 = 0;
					}
					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0], AS_UINT32(&t0));
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 1);
				}
			}
			if (force_update) {
				DSI_Start(module, cmdq);
				DSI_WaitForNotBusy(module, cmdq);
			}
		}
	}

}


void DSI_set_cmdq_V3(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_setting_table_V3 *para_tbl, unsigned int size,
		     unsigned char force_update)
{
#if 0
	UINT32 i;
	/* UINT32 layer, layer_state, lane_num; */
	UINT32 goto_addr, mask_para, set_para;
	/* UINT32 fbPhysAddr, fbVirAddr; */
	DSI_T0_INS t0;
	/* DSI_T1_INS t1; */
	DSI_T2_INS t2;

	UINT32 index = 0;

	unsigned char data_id, cmd, count;
	unsigned char *para_list;

	do {
		data_id = para_tbl[index].id;
		cmd = para_tbl[index].cmd;
		count = para_tbl[index].count;
		para_list = para_tbl[index].para_list;

		if (data_id == REGFLAG_ESCAPE_ID && cmd == REGFLAG_DELAY_MS_V3) {
			udelay(1000 * count);
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI", "DSI_set_cmdq_V3[%d]. Delay %d (ms)\n", index, count);

			continue;
		}

		if (0 != DSI_REG->DSI_MODE_CTRL.MODE) {	/* not in cmd mode */
			DSI_VM_CMD_CON_REG vm_cmdq;
			OUTREG32(&vm_cmdq, AS_UINT32(&DSI_REG->DSI_VM_CMD_CON));
			pr_info("set cmdq in VDO mode\n");
			if (count > 1) {
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID = data_id;
				vm_cmdq.CM_DATA_0 = count + 1;
				OUTREG32(&DSI_REG->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));

				goto_addr = (UINT32) (&DSI_VM_CMD_REG->data[0].byte0);
				mask_para = (0xFF << ((goto_addr & 0x3) * 8));
				set_para = (cmd << ((goto_addr & 0x3) * 8));
				MASKREG32(goto_addr & (~0x3), mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr = (UINT32) (&DSI_VM_CMD_REG->data[0].byte1) + i;
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (para_list[i] << ((goto_addr & 0x3) * 8));
					MASKREG32(goto_addr & (~0x3), mask_para, set_para);
				}
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_0 = cmd;
				if (count) {
					vm_cmdq.CM_DATA_ID = data_id;
					vm_cmdq.CM_DATA_1 = para_list[0];
				} else {
					vm_cmdq.CM_DATA_ID = data_id;
					vm_cmdq.CM_DATA_1 = 0;
				}
				OUTREG32(&DSI_REG->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
			}
			/* start DSI VM CMDQ */
			if (force_update) {
				MMProfileLogEx(MTKFB_MMP_Events.DSICmd, MMProfileFlagStart,
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[0]),
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[1]));
				DSI_EnableVM_CMD();

				/* must wait VM CMD done? */
				MMProfileLogEx(MTKFB_MMP_Events.DSICmd, MMProfileFlagEnd,
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[2]),
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[3]));
			}
		} else {
			_WaitForEngineNotBusy();
			{
				/* for(i = 0; i < sizeof(DSI_CMDQ_REG->data0) / sizeof(DSI_CMDQ); i++) */
				/* OUTREG32(&DSI_CMDQ_REG->data0[i], 0); */
				/* memset(&DSI_CMDQ_REG->data[0], 0, sizeof(DSI_CMDQ_REG->data[0])); */
				OUTREG32(&DSI_CMDQ_REG->data[0], 0);

				if (count > 1) {
					t2.CONFG = 2;
					t2.Data_ID = data_id;
					t2.WC16 = count + 1;

					OUTREG32(&DSI_CMDQ_REG->data[0].byte0, AS_UINT32(&t2));

					goto_addr = (UINT32) (&DSI_CMDQ_REG->data[1].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					MASKREG32(goto_addr & (~0x3), mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr = (UINT32) (&DSI_CMDQ_REG->data[1].byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para = (para_list[i] << ((goto_addr & 0x3) * 8));
						MASKREG32(goto_addr & (~0x3), mask_para, set_para);
					}

					OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 2 + (count) / 4);
				} else {
					t0.CONFG = 0;
					t0.Data0 = cmd;
					if (count) {
						t0.Data_ID = data_id;
						t0.Data1 = para_list[0];
					} else {
						t0.Data_ID = data_id;
						t0.Data1 = 0;
					}
					OUTREG32(&DSI_CMDQ_REG->data[0], AS_UINT32(&t0));
					OUTREG32(&DSI_REG->DSI_CMDQ_SIZE, 1);
				}

				if (force_update) {
					MMProfileLog(MTKFB_MMP_Events.DSICmd, MMProfileFlagStart);
					DSI_Start();
					for (i = 0; i < 10; i++)
						;
					_WaitForEngineNotBusy();
					MMProfileLog(MTKFB_MMP_Events.DSICmd, MMProfileFlagEnd);
				}
			}
		}
	} while (++index < size);
#endif
}



void DSI_set_cmdq(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int *pdata, unsigned int queue_size,
		  unsigned char force_update)
{
	/* DISPFUNC(); */
/* _WaitForEngineNotBusy(); */
	int j = 0;
	int i = 0;

	/* DISPCHECK("DSI_set_cmdq, module=%s, cmdq=0x%08x\n", ddp_get_module_name(module), cmdq); */

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (0 != DSI_REG[i]->DSI_MODE_CTRL.MODE) {
#if 0
			/* not in cmd mode */
			DSI_VM_CMD_CON_REG vm_cmdq;
			OUTREG32(&vm_cmdq, AS_UINT32(&DSI_REG[i]->DSI_VM_CMD_CON));
			pr_info("set cmdq in VDO mode\n");
			if (queue_size > 1) {	/* long packet */
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID = ((pdata[0] >> 8) & 0xFF);
				vm_cmdq.CM_DATA_0 = ((pdata[0] >> 16) & 0xFF);
				vm_cmdq.CM_DATA_1 = 0;
				OUTREG32(&DSI_REG[i]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
				for (j = 0; j < queue_size - 1; j++)
					OUTREG32(&DSI_VM_CMD_REG->data[j], AS_UINT32((pdata + j + 1)));
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_ID = ((pdata[0] >> 8) & 0xFF);
				vm_cmdq.CM_DATA_0 = ((pdata[0] >> 16) & 0xFF);
				vm_cmdq.CM_DATA_1 = ((pdata[0] >> 24) & 0xFF);
				OUTREG32(&DSI_REG->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
			}
			/* start DSI VM CMDQ */
			if (force_update) {
				MMProfileLogEx(MTKFB_MMP_Events.DSICmd, MMProfileFlagStart,
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[0]),
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[1]));
				DSI_EnableVM_CMD();

				/* must wait VM CMD done? */
				MMProfileLogEx(MTKFB_MMP_Events.DSICmd, MMProfileFlagEnd,
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[2]),
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[3]));
			}
#endif
		} else {
			ASSERT(queue_size <= 32);
			DSI_WaitForNotBusy(module, cmdq);
#ifdef ENABLE_DSI_ERROR_REPORT
			if ((pdata[0] & 1)) {
				memcpy(_dsi_cmd_queue, pdata, queue_size * 4);
				_dsi_cmd_queue[queue_size++] = 0x4;
				pdata = (unsigned int *)_dsi_cmd_queue;
			} else {
				pdata[0] |= 4;
			}
#endif

			for (j = 0; j < queue_size; j++)
				DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[i]->data[j], AS_UINT32((pdata + j)));

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_CMDQ_SIZE, queue_size);

			if (force_update) {
				DSI_Start(module, cmdq);
				DSI_WaitForNotBusy(module, cmdq);
			}
		}
	}
}

void _copy_dsi_params(LCM_DSI_PARAMS *src, LCM_DSI_PARAMS *dst)
{
	memcpy((LCM_DSI_PARAMS *) dst, (LCM_DSI_PARAMS *) src, sizeof(LCM_DSI_PARAMS));
}



int DSI_Send_ROI(DISP_MODULE_ENUM module, void *handle, unsigned int x, unsigned int y, unsigned int width,
		 unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	DSI_set_cmdq(module, handle, data_array, 3, 1);
	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	DSI_set_cmdq(module, handle, data_array, 3, 1);

	/* data_array[0]= 0x002c3909; */
	/* DSI_set_cmdq(module, handle, data_array, 1, 0); */
}



static void lcm_set_reset_pin(UINT32 value)
{
/* xuecheng, swtich back to gpio mode, because mmsys seems can't keep low in suspend. */
#if 0
	DSI_OUTREG32(NULL, DDP_REG_BASE_MMSYS_CONFIG + 0x150, value);
#else
	mt_set_gpio_mode(GPIO106 | 0x80000000, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO106 | 0x80000000, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO106 | 0x80000000, value ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
#endif
}

static void lcm_udelay(UINT32 us)
{
	udelay(us);
}

static void lcm_mdelay(UINT32 ms)
{
	if (ms < 2)
		udelay(ms * 1000);	/* udelay set max value 2000us=2ms */
	else
		mdelay(ms);
}


void DSI_set_cmdq_V11_wrapper_DSI0(void *cmdq, unsigned int *pdata, unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI0, cmdq, pdata, queue_size, force_update);
}

void DSI_set_cmdq_V11_wrapper_DSI1(void *cmdq, unsigned int *pdata, unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI1, cmdq, pdata, queue_size, force_update);
}

void DSI_set_cmdq_V11_wrapper_DSIDual(void *cmdq, unsigned int *pdata, unsigned int queue_size,
				      unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSIDUAL, cmdq, pdata, queue_size, force_update);
}

void DSI_set_cmdq_V22_Wrapper_DSI0(void *cmdq, unsigned cmd, unsigned char count, unsigned char *para_list,
				   unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V22_Wrapper_DSI1(void *cmdq, unsigned cmd, unsigned char count, unsigned char *para_list,
				   unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V22_Wrapper_DSIDual(void *cmdq, unsigned cmd, unsigned char count, unsigned char *para_list,
				      unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSIDUAL, cmdq, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSI0(unsigned cmd, unsigned char count, unsigned char *para_list,
				  unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI0, NULL, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSI1(unsigned cmd, unsigned char count, unsigned char *para_list,
				  unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI1, NULL, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSIDual(unsigned cmd, unsigned char count, unsigned char *para_list,
				     unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSIDUAL, NULL, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSI0(LCM_setting_table_V3 *para_tbl, unsigned int size, unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSI0, NULL, para_tbl, size, force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSI1(LCM_setting_table_V3 *para_tbl, unsigned int size, unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSI1, NULL, para_tbl, size, force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSIDual(LCM_setting_table_V3 *para_tbl, unsigned int size, unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSIDUAL, NULL, para_tbl, size, force_update);
}

void DSI_set_cmdq_wrapper_DSI0(unsigned int *pdata, unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI0, NULL, pdata, queue_size, force_update);
}

void DSI_set_cmdq_wrapper_DSI1(unsigned int *pdata, unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI1, NULL, pdata, queue_size, force_update);
}

void DSI_set_cmdq_wrapper_DSIDual(unsigned int *pdata, unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSIDUAL, NULL, pdata, queue_size, force_update);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSI0(UINT8 cmd, UINT8 *buffer, UINT8 buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL, cmd, buffer, buffer_size);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSI1(UINT8 cmd, UINT8 *buffer, UINT8 buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI1, NULL, cmd, buffer, buffer_size);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSIDUAL(UINT8 cmd, UINT8 *buffer, UINT8 buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSIDUAL, NULL, cmd, buffer, buffer_size);
}

void DSI_set_cmdq_V2_DSIx(void *cmdq, unsigned cmd, unsigned char count, unsigned char *para_list,
			  unsigned char force_update)
{
	cmdqRecHandle cmdq_hd = (cmdqRecHandle) cmdq;
	DSI_OUTREGBIT(cmdq_hd, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
	DSI_OUTREGBIT(cmdq_hd, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);
	if (_dsi_context[0].swap_port || _dsi_context[1].swap_port) {
		_dsi_context[0].swap_port = 0;
		_dsi_context[1].swap_port = 0;
		DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq_hd, cmd, count, para_list, 0);
		DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq_hd, cmd, count, para_list, 1);
		_dsi_context[0].swap_port = 1;
		_dsi_context[1].swap_port = 1;
	} else {
		DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq_hd, cmd, count, para_list, 0);
		DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq_hd, cmd, count, para_list, 1);
	}
}

static const LCM_UTIL_FUNCS lcm_utils_dsi0;
static const LCM_UTIL_FUNCS lcm_utils_dsi1;
static const LCM_UTIL_FUNCS lcm_utils_dsidual;

void DSI_Swap_Port(int swap)
{
	int i = 0;
	DISP_MODULE_ENUM module = DISP_MODULE_DSIDUAL;

	DISPMSG("DSI_Swap_port, moduel=%s, %d\n", ddp_get_module_name(module), swap);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		_dsi_context[i].swap_port = swap;

	if (swap)
		DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_SWAP_PORT, 0x1);
}

int ddp_dsi_set_lcm_utils(DISP_MODULE_ENUM module, LCM_DRIVER *lcm_drv)
{
	LCM_UTIL_FUNCS *utils = NULL;
	if (lcm_drv == NULL) {
		DISPERR("lcm_drv is null\n");
		return -1;
	}

	if (module == DISP_MODULE_DSI0) {
		utils = &lcm_utils_dsi0;
	} else if (module == DISP_MODULE_DSI1) {
		utils = &lcm_utils_dsi1;
	} else if (module == DISP_MODULE_DSIDUAL) {
		utils = &lcm_utils_dsidual;
	} else {
		DISPERR("wrong module: %d\n", module);
		return -1;
	}

	utils->set_reset_pin = lcm_set_reset_pin;
	utils->udelay = lcm_udelay;
	utils->mdelay = lcm_mdelay;
	if (module == DISP_MODULE_DSI0) {
		utils->dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI0;
		utils->dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI0;
		utils->dsi_dcs_read_lcm_reg_v2 = DSI_dcs_read_lcm_reg_v2_wrapper_DSI0;
		utils->dsi_set_cmdq_V23 = DSI_set_cmdq_V2_DSIx;
		utils->dsi_set_cmdq_V22 = DSI_set_cmdq_V22_Wrapper_DSI0;
		utils->dsi_set_cmdq_V11 = DSI_set_cmdq_V11_wrapper_DSI0;
	} else if (module == DISP_MODULE_DSI1) {
		utils->dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI1;
		utils->dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI1;
		utils->dsi_dcs_read_lcm_reg_v2 = DSI_dcs_read_lcm_reg_v2_wrapper_DSI1;
		utils->dsi_set_cmdq_V23 = DSI_set_cmdq_V2_DSIx;
		utils->dsi_set_cmdq_V22 = DSI_set_cmdq_V22_Wrapper_DSI1;
		utils->dsi_set_cmdq_V11 = DSI_set_cmdq_V11_wrapper_DSI1;
	} else if (module == DISP_MODULE_DSIDUAL) {
		/* TODO: Ugly workaround, hope we can found better resolution */
		LCM_PARAMS lcm_param;
		lcm_drv->get_params(&lcm_param);

		utils->dsi_swap_port = DSI_Swap_Port;

		if (lcm_param.lcm_cmd_if == LCM_INTERFACE_DSI0) {
			utils->dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI0;
			utils->dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI0;
			utils->dsi_dcs_read_lcm_reg_v2 = DSI_dcs_read_lcm_reg_v2_wrapper_DSI0;
			utils->dsi_set_cmdq_V23 = DSI_set_cmdq_V2_DSIx;
			utils->dsi_set_cmdq_V22 = DSI_set_cmdq_V22_Wrapper_DSI0;
			utils->dsi_set_cmdq_V11 = DSI_set_cmdq_V11_wrapper_DSI0;
		} else if (lcm_param.lcm_cmd_if == LCM_INTERFACE_DSI1) {
			utils->dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI1;
			utils->dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI1;
			utils->dsi_dcs_read_lcm_reg_v2 = DSI_dcs_read_lcm_reg_v2_wrapper_DSI1;
			utils->dsi_set_cmdq_V23 = DSI_set_cmdq_V2_DSIx;
			utils->dsi_set_cmdq_V22 = DSI_set_cmdq_V22_Wrapper_DSI1;
			utils->dsi_set_cmdq_V11 = DSI_set_cmdq_V11_wrapper_DSI1;
		} else {
			utils->dsi_set_cmdq = DSI_set_cmdq_wrapper_DSIDual;
			utils->dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSIDual;
			utils->dsi_dcs_read_lcm_reg_v2 = DSI_dcs_read_lcm_reg_v2_wrapper_DSIDUAL;
			utils->dsi_set_cmdq_V23 = DSI_set_cmdq_V2_DSIx;
			utils->dsi_set_cmdq_V22 = DSI_set_cmdq_V22_Wrapper_DSIDual;
			utils->dsi_set_cmdq_V11 = DSI_set_cmdq_V11_wrapper_DSIDual;
		}
	}

	utils->set_gpio_out = mt_set_gpio_out;
	utils->set_gpio_mode = mt_set_gpio_mode;
	utils->set_gpio_dir = mt_set_gpio_dir;
	utils->set_gpio_pull_enable = (int (*)(unsigned int, unsigned char))mt_set_gpio_pull_enable;

	lcm_drv->set_util_funcs(utils);

	return 0;
}


static int dsi0_te_enable = 1;
static int dsi1_te_enable;
static int dsidual_te_enable = 1;


void DSI_ChangeClk(DISP_MODULE_ENUM module, UINT32 clk)
{
	int i = 0;
	if (clk > 1250 || clk < 50)
		return;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		LCM_DSI_PARAMS *dsi_params = &_dsi_context[i].dsi_params;
		dsi_params->PLL_CLOCK = clk;
		DSI_WaitForNotBusy(module, NULL);
		DSI_PHY_clk_switch(module, NULL, false);
		DSI_PHY_clk_setting(module, NULL, dsi_params);
		DSI_PHY_TIMCONFIG(module, NULL, dsi_params);
	}
}

int ddp_dsi_init(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	DSI_STATUS ret = DSI_STATUS_OK;
	int i = 0;
	DISPFUNC();

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		/* DSI_OUTREG32(cmdq, 0xf0000048, 0x80000000); */
		DSI_OUTREG32(cmdq, DISPSYS_CONFIG_BASE + 0x108, 0xffffffff);
		DSI_OUTREG32(cmdq, DISPSYS_CONFIG_BASE + 0x118, 0xffffffff);
		/* DSI_OUTREG32(MMSYS_CONFIG_BASE+0xC08, 0xffffffff); */
	}
	DSI_REG[0] = DISPSYS_DSI0_BASE;
	DSI_PHY_REG[0] = DDP_REG_BASE_MIPITX0;
	DSI_CMDQ_REG[0] = DISPSYS_DSI0_BASE + 0x200;
	DSI_REG[1] = DISPSYS_DSI1_BASE;
	DSI_PHY_REG[1] = DDP_REG_BASE_MIPITX1;
	DSI_CMDQ_REG[1] = DISPSYS_DSI1_BASE + 0x200;
	DSI_VM_CMD_REG[0] = (PDSI_VM_CMDQ_REGS) (DISPSYS_DSI0_BASE + 0x134);
	DSI_VM_CMD_REG[1] = (PDSI_VM_CMDQ_REGS) (DDP_REG_BASE_DSI1 + 0x134);

	DISPCHECK("DSI0 Base Address:0x%lx, PHY:0x%lx\n", (unsigned long)DSI_REG[0], (unsigned long)DSI_PHY_REG[0]);
	DISPCHECK("DSI1 Base Address:0x%lx, PHY:0x%lx\n", (unsigned long)DSI_REG[1], (unsigned long)DSI_PHY_REG[1]);

	memset(&_dsi_context, 0, sizeof(_dsi_context));

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, CMD_DONE, 1);
		DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, RD_RDY, 1);
		DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, TE_RDY, 1);
		DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, VM_CMD_DONE, 1);
/* DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG,DSI_REG[i]->DSI_INTEN,EXT_TE,1); */
		DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, VM_DONE, 1);
		init_waitqueue_head(&_dsi_cmd_done_wait_queue[i]);
		init_waitqueue_head(&_dsi_dcs_read_wait_queue[i]);
		init_waitqueue_head(&_dsi_wait_bta_te[i]);
		init_waitqueue_head(&_dsi_wait_ext_te[i]);
		init_waitqueue_head(&_dsi_wait_vm_done_queue[i]);
		init_waitqueue_head(&_dsi_wait_vm_cmd_done_queue[i]);
	}

	disp_register_module_irq_callback(DISP_MODULE_DSI0, _DSI_INTERNAL_IRQ_Handler);
	disp_register_module_irq_callback(DISP_MODULE_DSI1, _DSI_INTERNAL_IRQ_Handler);
	disp_register_module_irq_callback(DISP_MODULE_DSIDUAL, _DSI_INTERNAL_IRQ_Handler);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DISPCHECK("dsi%d init finished\n", i);

#ifndef CONFIG_MTK_FPGA
	if (MIPITX_IsEnabled(module, cmdq)) {
		s_isDsiPowerOn = TRUE;
		set_mipi26m(1);
		if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
			ret += enable_clock(MT_CG_DISP1_DSI0_ENGINE, "DSI0");
			ret += enable_clock(MT_CG_DISP1_DSI0_DIGITAL, "DSI0");

			if (ret > 0)
				DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", "DSI0 power manager API return FALSE\n");
		}

		if (module == DISP_MODULE_DSI1 || module == DISP_MODULE_DSIDUAL) {
			ret += enable_clock(MT_CG_DISP1_DSI1_ENGINE, "DSI1");
			ret += enable_clock(MT_CG_DISP1_DSI1_DIGITAL, "DSI1");

			if (ret > 0)
				DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", "DSI1 power manager API return FALSE\n");
		}

		DSI_BackupRegisters(module, NULL);
	}
#endif

	return DSI_STATUS_OK;
}

int ddp_dsi_deinit(DISP_MODULE_ENUM module, void *cmdq_handle)
{

}

void _dump_dsi_params(LCM_DSI_PARAMS *dsi_config)
{
	int i = 0;

	if (dsi_config) {
		switch (dsi_config->mode) {
		case CMD_MODE:
			DISPCHECK("[DDPDSI] DSI Mode: CMD_MODE\n");
			break;
		case SYNC_PULSE_VDO_MODE:
			DISPCHECK("[DDPDSI] DSI Mode: SYNC_PULSE_VDO_MODE\n");
			break;
		case SYNC_EVENT_VDO_MODE:
			DISPCHECK("[DDPDSI] DSI Mode: SYNC_EVENT_VDO_MODE\n");
			break;
		case BURST_VDO_MODE:
			DISPCHECK("[DDPDSI] DSI Mode: BURST_VDO_MODE\n");
			break;
		default:
			DISPCHECK("[DDPDSI] DSI Mode: Unknown\n");
			break;
		}

#ifdef ROME_TODO
#error
#endif
		DISPCHECK("[DDPDSI] vact: %d, vbp: %d, vfp: %d, vact_line: %d, hact: %d, hbp: %d, hfp: %d,",
			dsi_config->vertical_sync_active, dsi_config->vertical_backporch,
			dsi_config->vertical_frontporch,
			dsi_config->vertical_active_line, dsi_config->horizontal_sync_active,
			dsi_config->horizontal_backporch, dsi_config->horizontal_frontporch);
		DISPCHECK(" hblank: %d\n", dsi_config->horizontal_blanking_pixel);

		DISPCHECK("[DDPDSI] pll_select: %d, pll_div1: %d, pll_div2: %d, fbk_div: %d,fbk_sel: %d, rg_bir: %d\n",
			dsi_config->pll_select, dsi_config->pll_div1, dsi_config->pll_div2, dsi_config->fbk_div,
			dsi_config->fbk_sel, dsi_config->rg_bir);

		DISPCHECK("[DDPDSI] rg_bic: %d, rg_bp: %d, PLL_CLOCK: %d, dsi_clock: %d, ssc_range: %d,",
			dsi_config->rg_bic, dsi_config->rg_bp, dsi_config->PLL_CLOCK, dsi_config->dsi_clock,
			dsi_config->ssc_range);
		DISPCHECK(" ssc_disable: %d, compatibility_for_nvk: %d, cont_clock: %d\n",
			dsi_config->compatibility_for_nvk, dsi_config->cont_clock, dsi_config->ssc_disable);

		DISPCHECK("[DDPDSI] lcm_ext_te_enable: %d, noncont_clock: %d, noncont_clock_period: %d\n",
			  dsi_config->lcm_ext_te_enable, dsi_config->noncont_clock, dsi_config->noncont_clock_period);
	}

	return;
}

static void DSI_PHY_CLK_LP_PerLine_config(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	int i;
	DSI_PHY_TIMCON0_REG timcon0;	/* LPX */
	DSI_PHY_TIMCON2_REG timcon2;	/* CLK_HS_TRAIL, CLK_HS_ZERO */
	DSI_PHY_TIMCON3_REG timcon3;	/* CLK_HS_EXIT, CLK_HS_POST, CLK_HS_PREP */
	DSI_HSA_WC_REG hsa;
	DSI_HBP_WC_REG hbp;
	DSI_HFP_WC_REG hfp, new_hfp;
	DSI_BLLP_WC_REG bllp;
	DSI_PSCTRL_REG ps;
	UINT32 hstx_ckl_wc, new_hstx_ckl_wc;
	UINT32 v_a, v_b, v_c, lane_num;
	LCM_DSI_MODE_CON dsi_mode;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		lane_num = dsi_params->LANE_NUM;
		dsi_mode = dsi_params->mode;

		if (dsi_mode == CMD_MODE)
			continue;

		/* vdo mode */
		DSI_OUTREG32(cmdq, &hsa, AS_UINT32(&DSI_REG[i]->DSI_HSA_WC));
		DSI_OUTREG32(cmdq, &hbp, AS_UINT32(&DSI_REG[i]->DSI_HBP_WC));
		DSI_OUTREG32(cmdq, &hfp, AS_UINT32(&DSI_REG[i]->DSI_HFP_WC));
		DSI_OUTREG32(cmdq, &bllp, AS_UINT32(&DSI_REG[i]->DSI_BLLP_WC));
		DSI_OUTREG32(cmdq, &ps, AS_UINT32(&DSI_REG[i]->DSI_PSCTRL));
		DSI_OUTREG32(cmdq, &hstx_ckl_wc, AS_UINT32(&DSI_REG[i]->DSI_HSTX_CKL_WC));
		DSI_OUTREG32(cmdq, &timcon0, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON0));
		DSI_OUTREG32(cmdq, &timcon2, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON2));
		DSI_OUTREG32(cmdq, &timcon3, AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON3));

		/* 1. sync_pulse_mode */
		/* Total    WC(A) = HSA_WC + HBP_WC + HFP_WC + PS_WC + 32 */
		/* CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP + CLK_HS_ZERO)*lane_num */
		/* CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL)*lane_num */
		/* HSTX_CKLP_WC = A - B */
		/* Limitation: B + C < HFP_WC */
		if (dsi_mode == SYNC_PULSE_VDO_MODE) {
			v_a = hsa.HSA_WC + hbp.HBP_WC + hfp.HFP_WC + ps.DSI_PS_WC + 32;
			v_b = (timcon3.CLK_HS_EXIT + timcon0.LPX + timcon3.CLK_HS_PRPR + timcon2.CLK_ZERO) * lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) * lane_num;

			DISPCHECK("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n", (v_a - v_b), hstx_ckl_wc);
			/* DISPCHECK("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",(v_b+v_c),hfp); */
			DISPCHECK("===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, (v_b + v_c + DIFF_CLK_LANE_LP));
			DSI_OUTREG32(cmdq, &new_hfp, AS_UINT32(&DSI_REG[i]->DSI_HFP_WC));
			v_a = hsa.HSA_WC + hbp.HBP_WC + new_hfp.HFP_WC + ps.DSI_PS_WC + 32;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, (v_a - v_b));
			DSI_OUTREG32(cmdq, &new_hstx_ckl_wc, AS_UINT32(&DSI_REG[i]->DSI_HSTX_CKL_WC));
			DISPCHECK("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n", new_hstx_ckl_wc, new_hfp.HFP_WC);
		}
		/* 2. sync_event_mode */
		/* Total    WC(A) = HBP_WC + HFP_WC + PS_WC + 26 */
		/* CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP + CLK_HS_ZERO)*lane_num */
		/* CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL)*lane_num */
		/* HSTX_CKLP_WC = A - B */
		/* Limitation: B + C < HFP_WC */
		else if (dsi_mode == SYNC_EVENT_VDO_MODE) {
			v_a = hbp.HBP_WC + hfp.HFP_WC + ps.DSI_PS_WC + 26;
			v_b = (timcon3.CLK_HS_EXIT + timcon0.LPX + timcon3.CLK_HS_PRPR + timcon2.CLK_ZERO) * lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) * lane_num;

			DISPCHECK("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n", (v_a - v_b), hstx_ckl_wc);
			/* DISPCHECK("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",(v_b+v_c),hfp); */
			DISPCHECK("===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, (v_b + v_c + DIFF_CLK_LANE_LP));
			DSI_OUTREG32(cmdq, &new_hfp, AS_UINT32(&DSI_REG[i]->DSI_HFP_WC));
			v_a = hbp.HBP_WC + new_hfp.HFP_WC + ps.DSI_PS_WC + 26;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, (v_a - v_b));
			DSI_OUTREG32(cmdq, &new_hstx_ckl_wc, AS_UINT32(&DSI_REG[i]->DSI_HSTX_CKL_WC));
			DISPCHECK("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n", new_hstx_ckl_wc, new_hfp.HFP_WC);

		}
		/* 3. burst_mode */
		/* Total    WC(A) = HBP_WC + HFP_WC + PS_WC + BLLP_WC + 32 */
		/* CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP + CLK_HS_ZERO)*lane_num */
		/* CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL)*lane_num */
		/* HSTX_CKLP_WC = A - B */
		/* Limitation: B + C < HFP_WC */
		else if (dsi_mode == BURST_VDO_MODE) {
			v_a = hbp.HBP_WC + hfp.HFP_WC + ps.DSI_PS_WC + bllp.BLLP_WC + 32;
			v_b = (timcon3.CLK_HS_EXIT + timcon0.LPX + timcon3.CLK_HS_PRPR + timcon2.CLK_ZERO) * lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) * lane_num;

			DISPCHECK("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n", (v_a - v_b), hstx_ckl_wc);
			/* DISPCHECK("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",(v_b+v_c),hfp); */
			DISPCHECK("===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, (v_b + v_c + DIFF_CLK_LANE_LP));
			DSI_OUTREG32(cmdq, &new_hfp, AS_UINT32(&DSI_REG[i]->DSI_HFP_WC));
			v_a = hbp.HBP_WC + new_hfp.HFP_WC + ps.DSI_PS_WC + bllp.BLLP_WC + 32;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, (v_a - v_b));
			DSI_OUTREG32(cmdq, &new_hstx_ckl_wc, AS_UINT32(&DSI_REG[i]->DSI_HSTX_CKL_WC));
			DISPCHECK("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n", new_hstx_ckl_wc, new_hfp.HFP_WC);
		}
	}

}

int ddp_dsi_config(DISP_MODULE_ENUM module, disp_ddp_path_config *config, void *cmdq)
{
	int i = 0;
	int mipitx_config_dirty = 0;
	if (!config->dst_dirty) {
		if (atomic_read(&PMaster_enable) == 0)
			return 0;
	}
	DISPFUNC();
	DISPCHECK("===>run here 00 Pmaster: clk:%d\n", _dsi_context[0].dsi_params.PLL_CLOCK);
	LCM_DSI_PARAMS *dsi_config = &(config->dispif_config.dsi);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		_copy_dsi_params(dsi_config, &(_dsi_context[i].dsi_params));
		_dsi_context[i].lcm_width = config->dst_w;
		_dsi_context[i].lcm_height = config->dst_h;
		_dump_dsi_params(&(_dsi_context[i].dsi_params));
		if (dsi_config->mode != CMD_MODE) {
			/* not enable TE in vdo mode */
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, TE_RDY, 0);
		} else {
			/*enable TE in cmd mode*/
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, TE_RDY, 1);
		}
	}
	DISPCHECK("===>01Pmaster: clk:%d\n", _dsi_context[0].dsi_params.PLL_CLOCK);
	if (dsi_config->mode != CMD_MODE)
		dsi_currect_mode = 1;

#ifndef CONFIG_MTK_FPGA
	if ((MIPITX_IsEnabled(module, cmdq)) && (atomic_read(&PMaster_enable) == 0)) {
		DISPCHECK("mipitx is already init\n");
		goto done;
	} else {
		DISPCHECK("MIPITX is not inited, will config mipitx clock now\n");
		DISPCHECK("===>Pmaster:CLK SETTING??==> clk:%d\n", _dsi_context[0].dsi_params.PLL_CLOCK);
		DSI_PHY_clk_setting(module, NULL, dsi_config);
	}

#endif
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (dsi_config->mode == CMD_MODE
		    || ((dsi_config->switch_mode_enable == 1) && (dsi_config->switch_mode == CMD_MODE)))
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, EXT_TE, 1);
	}
	/* DSI_Reset(module, cmdq_handle); */
	DSI_TXRX_Control(module, cmdq, dsi_config);
	DSI_PS_Control(module, cmdq, dsi_config, config->dst_w, config->dst_h);
	DSI_PHY_TIMCONFIG(module, cmdq, dsi_config);

	if (dsi_config->mode != CMD_MODE
	    || ((dsi_config->switch_mode_enable == 1) && (dsi_config->switch_mode != CMD_MODE))) {
		DSI_Config_VDO_Timing(module, cmdq, dsi_config);
		DSI_Set_VM_CMD(module, cmdq);
	}
#if 0
	/* TODO: workaround for 8 lane left/right mode wqhd lcm panel */
	if (module == DISP_MODULE_DSIDUAL) {
		DSI_OUTREG32(cmdq, 0xF401A050, config->dst_w);
		DSI_OUTREG32(cmdq, 0xF401A054, config->dst_h);
		DSI_OUTREG32(cmdq, 0xF401A000, 9);
	}
#endif
	/* Enable clk low power per Line ; */
	if (dsi_config->clk_lp_per_line_enable)
		DSI_PHY_CLK_LP_PerLine_config(module, cmdq, dsi_config);

done:

	return 0;
}

int g_lcm_x = 0;
int g_lcm_y = 0;

int ddp_dsi_start(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	g_lcm_x = disp_helper_get_option(DISP_HELPER_OPTION_FAKE_LCM_X);
	g_lcm_y = disp_helper_get_option(DISP_HELPER_OPTION_FAKE_LCM_Y);

	DISPDBG("ddp_dsi_start, x=%d, y=%d\n", g_lcm_x, g_lcm_y);
	int i = 0;
	DISPFUNC();
	if (dprec_option_enabled()) {
		DISPDBG("dump before ddp_dsi_start\n");
		DSI_DumpRegisters(module, 2);
	}

	if (module == DISP_MODULE_DSIDUAL) {
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);

		if (_dsi_context[i].swap_port != 1) {
			DSI_Send_ROI(DISP_MODULE_DSI0, cmdq, g_lcm_x, g_lcm_y, _dsi_context[i].lcm_width,
				     _dsi_context[i].lcm_height);
		} else {
			DSI_Send_ROI(DISP_MODULE_DSI1, cmdq, g_lcm_x, g_lcm_y, _dsi_context[i].lcm_width,
				     _dsi_context[i].lcm_height);
		}
		/* must set DSI_START to 0 before set dsi_dual_en, don't know why.2014.02.15 */
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);

		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 1);
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 1);

		DSI_SetMode(module, cmdq, _dsi_context[i].dsi_params.mode);
		DSI_clk_HS_mode(module, cmdq, TRUE);
	} else {
		DSI_Send_ROI(module, cmdq, 0, 0, _dsi_context[i].lcm_width, _dsi_context[i].lcm_height);
		DSI_SetMode(module, cmdq, _dsi_context[i].dsi_params.mode);
		DSI_clk_HS_mode(module, cmdq, TRUE);
	}

	if (dprec_option_enabled()) {
		DISPDBG("dump after ddp_dsi_start\n");
		DSI_DumpRegisters(module, 2);
	}

	return 0;
}

int ddp_dsi_stop(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	int i = 0;
	unsigned int tmp = 0;
	DISPFUNC();
	/* ths caller should call wait_event_or_idle for frame stop event then. */
	/* DSI_SetMode(module, cmdq_handle, CMD_MODE); */

	if (_dsi_is_video_mode(module)) {
		DISPMSG("dsi is video mode\n");
		DSI_SetMode(module, cmdq_handle, CMD_MODE);

		i = DSI_MODULE_BEGIN(module);
		while (1) {
			tmp = INREG32(&DSI_REG[i]->DSI_INTSTA);
			if (!(tmp & 0x80000000))
				break;
		}

		i = DSI_MODULE_END(module);
		while (1) {
			DISPMSG("dsi%d is busy\n", i);
			tmp = INREG32(&DSI_REG[i]->DSI_INTSTA);
			if (!(tmp & 0x80000000))
				break;
		}

		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
			DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);
/* DSI_OUTREG32(NULL, 0xF401A000, 4); */
		}
	} else {
		DISPMSG("dsi is cmd mode\n");
		/* TODO: modify this with wait event */
		DSI_WaitForNotBusy(module, cmdq_handle);
		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
			DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);
			/* DSI_OUTREG32(NULL, 0xF401A000, 4); */
		}
	}
	DSI_clk_HS_mode(module, cmdq_handle, FALSE);
	return 0;
}

int ddp_dsi_switch_lcm_mode(DISP_MODULE_ENUM module, void *params)
{
	int i = 0;
	int ret = 0;
	LCM_DSI_MODE_SWITCH_CMD lcm_cmd = *((LCM_DSI_MODE_SWITCH_CMD *) (params));
	int mode = (int)(lcm_cmd.mode);

	if (dsi_currect_mode == mode) {
		DISPDBG("[ddp_dsi_switch_mode] not need switch mode, current mode = %d, switch to %d\n",
			dsi_currect_mode, mode);
		return 0;
	}

	if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI0)
		i = 0;
	else if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI0)
		i = 1;
	else {
		DISPDBG("dsi switch not support this cmd IF:%d\n", lcm_cmd.cmd_if);
		return -1;
	}

	/* if(mode == 0)//V2C */
	if (0) {		/* xuecheng try */
/* DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG,DSI_REG[i]->DSI_INTEN,EXT_TE,1); */
		DSI_OUTREG32(NULL, (unsigned long)(DSI_REG[i]) + 0x130,
			     0x00001521 | (lcm_cmd.addr << 16) | (lcm_cmd.val[0] << 24));
		DSI_OUTREGBIT(NULL, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(NULL, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		wait_vm_cmd_done = false;
		ret = wait_event_interruptible_timeout(_dsi_wait_vm_cmd_done_queue[i], wait_vm_cmd_done, HZ / 10);
		if (ret == 0)
			DISPERR("switch lcm from video mode to cmd mode, waiting vm done timeout!!\n");

#if 0
		DSI_OUTREG32(NULL, (unsigned long)(DSI_REG[i]) + 0x130,
			     0x00001539 | (lcm_cmd.addr << 16) | (lcm_cmd.val[1] << 24));
		DSI_OUTREGBIT(NULL, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(NULL, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		wait_vm_cmd_done = false;
		wait_event_interruptible(_dsi_wait_vm_cmd_done_queue[i], wait_vm_cmd_done);
#endif
	}
	return 0;
}

void dsi_analysis(DISP_MODULE_ENUM module);

int ddp_dsi_switch_mode(DISP_MODULE_ENUM module, void *cmdq_handle, void *params)
{
	int i = 0;
	int ret = 0;
	LCM_DSI_MODE_SWITCH_CMD lcm_cmd = *((LCM_DSI_MODE_SWITCH_CMD *) (params));
	int mode = (int)(lcm_cmd.mode);

	if (dsi_currect_mode == mode) {
		DISPDBG("[ddp_dsi_switch_mode] not need switch mode, current mode = %d, switch to %d\n",
			dsi_currect_mode, mode);
		return 0;
	}

	if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI0) {
		i = 0;
	} else if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI0) {
		i = 1;
	} else {
		pr_info("dsi switch not support this cmd IF:%d\n", lcm_cmd.cmd_if);
		return -1;
	}

	if (mode == 0) {
		DISPDBG("v2c switch begin\n");
#if 1
		DSI_SetBypassRack(module, cmdq_handle, 1);

		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x130,
			     0x00001521 | (lcm_cmd.addr << 16) | (lcm_cmd.val[0] << 24));
		DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x00000020, 0x20);	/* zxc try */

		DSI_SetVdoFrmMode(module, cmdq_handle, 1);
		DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x80000000, 0x0);	/* zxc try */

		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x60, 0);

		DSI_Start(module, cmdq_handle);
		DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x00000008, 0x8);	/* zxc try */

		DSI_SetSwitchMode(module, cmdq_handle, 0);	/*  */

		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x130,
			     0x00001539 | (lcm_cmd.addr << 16) | (lcm_cmd.val[1] << 24));
		DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		DSI_Start(module, cmdq_handle);

		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x1);	/* reset mutex for V2C */
		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x0);	/*  */
		DSI_MASKREG32(cmdq_handle, 0xF4020030, 0x3, 0x0);	/* mutex to cmd  mode */

		DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x80000000, 0x0);	/* zxc try */
		DSI_SetVdoFrmMode(module, cmdq_handle, 0);
		DSI_SetMode(module, cmdq_handle, 0);

		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x200 + 0, 0x002c3909);
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x60, 1);
		DSI_OUTREGBIT(cmdq_handle, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 1);

#if 0
		DSI_Reset(module, cmdq_handle);



		DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x00000008, 0x8);	/* zxc try */

		DSI_SetBypassRack(module, cmdq_handle, 1);
		DSI_OUTREG32(cmdq_handle, (unsigned int)(DSI_REG[i]) + 0x200 + 0, 0x00000020);
		/* DSI_OUTREG32(cmdq_handle,  (unsigned int)(DSI_REG[i])+0x200+4, 0x002c3909); */
		DSI_OUTREG32(cmdq_handle, (unsigned int)(DSI_REG[i]) + 0x60, 1);

		/* DSI_MASKREG32(cmdq_handle, 0xF4020020, 0x1, 0x1); // release mutex for video mode */

		DSI_Start(module, cmdq_handle);

		DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x00000004, 0x4);	/* zxc try */

		DSI_OUTREG32(cmdq_handle, (unsigned int)(DSI_REG[i]) + 0x200 + 0, 0x002c3909);
		DSI_OUTREG32(cmdq_handle, (unsigned int)(DSI_REG[i]) + 0x60, 1);

		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x1);	/* reset mutex for V2C */
		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x0);	/*  */
		DSI_MASKREG32(cmdq_handle, 0xF4020030, 0x3, 0x0);	/* mutex to cmd  mode */

		DSI_OUTREGBIT(cmdq_handle, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 1);
		/* ddp_dsi_start(module, cmdq_handle); */
#endif
		cmdqRecFlush(cmdq_handle);
		/* msleep(16); */
		cmdqRecReset(cmdq_handle);
		dsi_analysis(module);
		DSI_DumpRegisters(module, 2);
#else
		DSI_SetSwitchMode(module, cmdq_handle, 0);
		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x1);	/* reset mutex for V2C */
		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x0);	/*  */
		DSI_MASKREG32(cmdq_handle, 0xF4020030, 0x1, 0x0);	/* mutext to cmd  mode */
		cmdqRecFlush(cmdq_handle);
		cmdqRecReset(cmdq_handle);
		cmdqRecWaitNoClear(cmdq_handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
		DSI_SetMode(module, NULL, 0);
/* DSI_SetBypassRack(module, NULL, 0); */

#endif
		DISPDBG("v2c switch finished\n");
	} else {
		DISPDBG("c2v switch begin\n");
#if 0
		cmdqRecWaitNoClear(cmdq_handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
		DSI_SetMode(module, cmdq_handle, mode);
		/* DSI_SetSwitchMode(module, cmdq_handle, 1);//EXT TE could not use C2V */
		DSI_MASKREG32(cmdq_handle, 0xF4020030, 0x1, 0x1);	/* mutext to video mode */
		DSI_OUTREG32(cmdq_handle, (unsigned int)(DSI_REG[i]) + 0x200 + 0,
			     0x00001500 | (lcm_cmd.addr << 16) | (lcm_cmd.val[0] << 24));
		DSI_OUTREG32(cmdq_handle, (unsigned int)(DSI_REG[i]) + 0x200 + 4, 0x00000020);
		DSI_OUTREG32(cmdq_handle, (unsigned int)(DSI_REG[i]) + 0x60, 2);

		DSI_MASKREG32(cmdq_handle, 0xF4020020, 0x1, 0x1);	/* release mutex for video mode */

		DSI_Start(module, cmdq_handle);
		cmdqRecFlush(cmdq_handle);
		cmdqRecReset(cmdq_handle);
#else
		cmdqRecWaitNoClear(cmdq_handle, CMDQ_SYNC_TOKEN_STREAM_EOF);

		DSI_OUTREGBIT(cmdq_handle, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 0);

		DSI_SetMode(module, cmdq_handle, mode);
		DSI_SetSwitchMode(module, cmdq_handle, 1);	/* EXT TE could not use C2V */
		DSI_SetBypassRack(module, cmdq_handle, 1);
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x200 + 0,
			     0x00001500 | (lcm_cmd.addr << 16) | (lcm_cmd.val[0] << 24));
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x200 + 4, 0x00000020);
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x60, 2);

		DSI_Start(module, cmdq_handle);

		DSI_MASKREG32(cmdq_handle, 0xF4020030, 0x3, 0x1);	/* mutext to video mode */
		DSI_MASKREG32(cmdq_handle, 0xF4020020, 0x1, 0x1);	/* release mutex for video mode */
		/* DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x00000002, 0x2); */
		/* DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0x00000004, 0x4); */
		/* wait_cmd_done = false; */
		cmdqRecFlush(cmdq_handle);
		DISPDBG("after c2v switch, cmdq flushed\n");
		/* msleep(16); */
#if 1
		/* THIS IS NOT A GOOD DESIGN!!!!! */
		/* TEMP WORKAROUND FOR ESD/CV SWITCH */
		int wait_count = 100;
		while (wait_count) {
			DISPDBG("wait loop %d\n", wait_count);
			/* if(wait_cmd_done) */
			if (DSI_REG[i]->DSI_STATE_DBG6.CMCTL_STATE == 0x1) {
				DISPDBG("after c2v switch, dsi fsm is idle\n");
				break;
			}
			/*msleep(1);*/ /* msleep is not accurately sleep 1 ms, but may up to 20 ms*/
			/* use usleep_range(min, max) instead*/
			usleep_range(1000, 2000);
			wait_count--;
		}

		if (wait_count == 0) {
			DISPERR("after c2v switch, dsi state is not idle[0x%08x]\n",
				DSI_REG[i]->DSI_STATE_DBG6.CMCTL_STATE);
			dsi_analysis(module);
			DSI_DumpRegisters(module, 2);
			DSI_Reset(module, NULL);
			DSI_OUTREGBIT(NULL, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, C2V_SWITCH_ON, 0);
			DSI_DumpRegisters(module, 2);

			DSI_OUTREG32(NULL, (unsigned long)(DSI_REG[i]) + 0x60, 0);
			DSI_Start(module, NULL);
			DSI_DumpRegisters(module, 2);
		}

		dsi_analysis(module);
		DSI_DumpRegisters(module, 2);

#endif
		cmdqRecReset(cmdq_handle);

#endif
		DISPDBG("c2v switch finished\n");
	}

	dsi_currect_mode = mode;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		_dsi_context[i].dsi_params.mode = mode;

	return 0;
}

int ddp_dsi_ioctl(DISP_MODULE_ENUM module, void *cmdq_handle, unsigned int ioctl_cmd, unsigned long *params)
{
	int ret = 0;
	DISPFUNC();
	DDP_IOCTL_NAME ioctl = (DDP_IOCTL_NAME) ioctl_cmd;
	pr_info("[ddp_dsi_ioctl] index = %d\n", ioctl);
	switch (ioctl) {
	case DDP_STOP_VIDEO_MODE:
		{
			/* ths caller should call wait_event_or_idle for frame stop event then. */
			DSI_SetMode(module, cmdq_handle, CMD_MODE);
			/* TODO: modify this with wait event */

			if (0 != DSI_WaitVMDone(module))
				ret = -1;

			if (module == DISP_MODULE_DSIDUAL) {
				DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
				DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
				DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
				DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);
			}
			break;
		}

	case DDP_SWITCH_DSI_MODE:
		{
			ret = ddp_dsi_switch_mode(module, cmdq_handle, params);
			break;
		}
	case DDP_SWITCH_LCM_MODE:
		{
			ret = ddp_dsi_switch_lcm_mode(module, params);
			break;
		}
	case DDP_BACK_LIGHT:
		{
			unsigned int cmd = 0x51;
			unsigned int count = 1;
			unsigned int level = params[0];
			pr_info("[ddp_dsi_ioctl] level = %d module %d\n", level, module);
			if (module == DISP_MODULE_DSIDUAL) {
#if 1
				if (dsi_currect_mode == CMD_MODE) {
					DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL,
						      DSI_DUAL_EN, 0);
					DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL,
						      DSI_DUAL_EN, 0);

					if (_dsi_context[0].swap_port || _dsi_context[1].swap_port)
						DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq_handle, cmd, count, &level, 1);
					else
						DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq_handle, cmd, count, &level, 1);

					DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
					DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);
					DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL,
						      DSI_DUAL_EN, 1);
					DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL,
						      DSI_DUAL_EN, 1);
				}
#else
				pr_info("setbacklight module=%d\n", module);
				DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
				DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);
				if (_dsi_context[0].swap_port || _dsi_context[1].swap_port) {
					_dsi_context[0].swap_port = 0;
					_dsi_context[1].swap_port = 0;
					DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq_handle, cmd, count, &level, 0);
					DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq_handle, cmd, count, &level, 1);
					_dsi_context[0].swap_port = 1;
					_dsi_context[1].swap_port = 1;
				} else {
					DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq_handle, cmd, count, &level, 0);
					DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq_handle, cmd, count, &level, 1);
				}
#endif
			}
		}
	case DDP_PHY_CLK_CHANGE:
		{

			LCM_DSI_PARAMS *dsi_params = &_dsi_context[0].dsi_params;
			dsi_params->PLL_CLOCK = *params;
			/* DSI_WaitForNotBusy(module,cmdq_handle); */
			DSI_DisableClk(module, cmdq_handle);
			DSI_PHY_clk_change(module, cmdq_handle, dsi_params);
			DSI_EnableClk(module, cmdq_handle);
			DSI_PHY_TIMCONFIG(module, cmdq_handle, dsi_params);

			break;
		}
	default:
		break;
	}
	return ret;
}

/* static int mutex_id_for_latest_trigger = 0; */

int ddp_dsi_trigger(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	unsigned int data_array[16];

	/* mutex_id_for_latest_trigger = mutex_id; */

	if (_dsi_context[i].dsi_params.mode == CMD_MODE) {
		data_array[0] = 0x002c3909;
		DSI_set_cmdq(module, cmdq, data_array, 1, 0);
	}

	DSI_Start(module, cmdq);

	return 0;
}

int ddp_dsi_reset(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	DSI_Reset(module, cmdq_handle);

	return 0;
}


int ddp_dsi_power_on(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	int i = 0;
	int ret = 0;

	DISPFUNC();

	if (dprec_option_enabled())
		DSI_DumpRegisters(module, 2);

	if (!s_isDsiPowerOn) {
		set_mipi26m(1);
		if (is_ipoh_bootup) {
			if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
				ret += enable_clock(MT_CG_DISP1_DSI0_ENGINE, "DSI0");
				ret += enable_clock(MT_CG_DISP1_DSI0_DIGITAL, "DSI0");

				if (ret > 0)
					DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI",
						       "DSI0 power manager API return FALSE\n");
			}
#if 1
			if (module == DISP_MODULE_DSI1 || module == DISP_MODULE_DSIDUAL) {
				ret += enable_clock(MT_CG_DISP1_DSI1_ENGINE, "DSI1");
				ret += enable_clock(MT_CG_DISP1_DSI1_DIGITAL, "DSI1");

				if (ret > 0)
					DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI",
						       "DSI1 power manager API return FALSE\n");
			}
#endif
			s_isDsiPowerOn = TRUE;
			pr_info("ipoh dsi power on return\n");
			return DSI_STATUS_OK;
		}
		DSI_PHY_clk_switch(module, NULL, true);

		if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
			ret += enable_clock(MT_CG_DISP1_DSI0_ENGINE, "DSI0");
			ret += enable_clock(MT_CG_DISP1_DSI0_DIGITAL, "DSI0");

			if (ret > 0)
				DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", "DSI0 power manager API return FALSE\n");
		}
#if 1
		if (module == DISP_MODULE_DSI1 || module == DISP_MODULE_DSIDUAL) {
			ret += enable_clock(MT_CG_DISP1_DSI1_ENGINE, "DSI1");
			ret += enable_clock(MT_CG_DISP1_DSI1_DIGITAL, "DSI1");

			if (ret > 0)
				DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", "DSI1 power manager API return FALSE\n");
		}
#endif

		/* restore dsi register */
		DSI_RestoreRegisters(module, NULL);

		/* enable sleep-out mode */
		DSI_SleepOut(module, NULL);

		/* enter wakeup */
		DSI_Wakeup(module, NULL);

		/* enable clock */
		DSI_EnableClk(module, NULL);

		DSI_Reset(module, NULL);
#if 0
		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREG32(NULL, 0xF401A050, _dsi_context[i].lcm_width);
			DSI_OUTREG32(NULL, 0xF401A054, _dsi_context[i].lcm_height);
			DSI_OUTREG32(NULL, 0xF401A000, 9);
		}
#endif
		s_isDsiPowerOn = TRUE;
	}

	/* DSI_DumpRegisters(module,1); */
	return DSI_STATUS_OK;
}


int ddp_dsi_power_off(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	int i = 0;
	int ret = 0;
	DISPFUNC();
	/* DSI_DumpRegisters(module,1); */

	DSI_Swap_Port(0);
	if (s_isDsiPowerOn) {
		DSI_BackupRegisters(module, NULL);

		/* enter ULPS mode */
		DSI_clk_ULP_mode(module, NULL, 1);
		DSI_lane0_ULP_mode(module, NULL, 1);
		/* disable clock */
		DSI_DisableClk(module, NULL);

		if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
			ret += disable_clock(MT_CG_DISP1_DSI0_ENGINE, "DSI0");
			ret += disable_clock(MT_CG_DISP1_DSI0_DIGITAL, "DSI0");

			if (ret > 0)
				DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI0", "DSI0 power manager API return FALSE\n");
		}
#if 1
		if (module == DISP_MODULE_DSI1 || module == DISP_MODULE_DSIDUAL) {
			ret += disable_clock(MT_CG_DISP1_DSI1_ENGINE, "DSI1");
			ret += disable_clock(MT_CG_DISP1_DSI1_DIGITAL, "DSI1");

			if (ret > 0)
				DISP_LOG_PRINT(ANDROID_LOG_WARN, "DSI", "DSI1 power manager API return FALSE\n");
		}
#endif
		/* disable mipi pll */
		DSI_PHY_clk_switch(module, NULL, false);

		set_mipi26m(0);

		s_isDsiPowerOn = FALSE;
	}


	/* DSI_DumpRegisters(module,1); */
	return DSI_STATUS_OK;
}


int ddp_dsi_is_busy(DISP_MODULE_ENUM module)
{
	int i = 0;
	int busy = 0;
	DSI_INT_STATUS_REG status;
	/* DISPFUNC(); */

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		status = DSI_REG[i]->DSI_INTSTA;

		if (status.BUSY)
			busy++;
	}

	DISPDBG("%s is %s\n", ddp_get_module_name(module), busy ? "busy" : "idle");
	return busy;
}

int ddp_dsi_is_idle(DISP_MODULE_ENUM module)
{
	return !ddp_dsi_is_busy(module);
}

static const char *dsi_mode_spy(LCM_DSI_MODE_CON mode)
{
	switch (mode) {
	case CMD_MODE:
		return "CMD_MODE";
	case SYNC_PULSE_VDO_MODE:
		return "SYNC_PULSE_VDO_MODE";
	case SYNC_EVENT_VDO_MODE:
		return "SYNC_EVENT_VDO_MODE";
	case BURST_VDO_MODE:
		return "BURST_VDO_MODE";
	default:
		return "unknown";
	}
}

void dsi_analysis(DISP_MODULE_ENUM module)
{
	int i = 0;
	DDPDUMP("==DISP DSI ANALYSIS==\n");
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DDPDUMP("MIPITX Clock: %d\n", dsi_phy_get_clk(module));
		DDPDUMP("DSI%d Start:%x, Busy:%d, DSI_EN:%d, DSI_DUAL_EN:%d, MODE:%s, High Speed:%d, FSM State:%s\n",
			i,
			DSI_REG[i]->DSI_START.DSI_START,
			DSI_REG[i]->DSI_INTSTA.BUSY,
			DSI_REG[i]->DSI_COM_CTRL.DSI_EN,
			DSI_REG[i]->DSI_COM_CTRL.DSI_DUAL_EN,
			dsi_mode_spy(DSI_REG[i]->DSI_MODE_CTRL.MODE),
			DSI_REG[i]->DSI_PHY_LCCON.LC_HS_TX_EN,
			_dsi_cmd_mode_parse_state(DSI_REG[i]->DSI_STATE_DBG6.CMCTL_STATE));

		DDPDUMP("DSI%d WORD Counter: %d, LINE Counter: %d\n",
			i,
			DSI_REG[i]->DSI_STATE_DBG8.WORD_COUNTER,
			DSI_REG[i]->DSI_STATE_DBG9.LINE_COUNTER);

		DDPDUMP("DSI%d IRQ, RD_RDY:%d, CMD_DONE:%d, EXT_TE:%d, SLEEPOUT_DONE:%d, TE_RDY:%d,",
			i,
			DSI_REG[i]->DSI_INTSTA.RD_RDY,
			DSI_REG[i]->DSI_INTSTA.CMD_DONE,
			DSI_REG[i]->DSI_INTSTA.EXT_TE,
			DSI_REG[i]->DSI_INTSTA.SLEEPOUT_DONE,
			DSI_REG[i]->DSI_INTSTA.TE_RDY);
		DDPDUMP(" VM_CMD_DONE:%d, VM_DONE:%d\n",
			DSI_REG[i]->DSI_INTSTA.VM_CMD_DONE,
			DSI_REG[i]->DSI_INTSTA.VM_DONE);

		DDPDUMP("DSI%d Lane Num:%d, Ext_TE_EN:%d, Ext_TE_Edge:%d, HSTX_CKLP_EN:%d\n",
			i,
			DSI_REG[i]->DSI_TXRX_CTRL.LANE_NUM,
			DSI_REG[i]->DSI_TXRX_CTRL.EXT_TE_EN,
			DSI_REG[i]->DSI_TXRX_CTRL.EXT_TE_EDGE,
			DSI_REG[i]->DSI_TXRX_CTRL.HSTX_CKLP_EN);
	}
}

int ddp_dsi_dump(DISP_MODULE_ENUM module, int level)
{
	if (!s_isDsiPowerOn)
		return 0;

	if (level >= 1) {
		dsi_analysis(module);
		DSI_DumpRegisters(module, level);
	} else if (level >= 0) {
		DSI_DumpRegisters(module, level);
	}
}

extern int ddp_mutex_enable(int mutex_id, DDP_SCENARIO_ENUM scenario, void *handle);
int ddp_dsi_build_cmdq(DISP_MODULE_ENUM module, void *cmdq_trigger_handle, CMDQ_STATE state)
{
	int ret = 0;
	int i = 0;
	int dsi_i = 0;
	LCM_DSI_PARAMS *dsi_params = NULL;
	DSI_T0_INS t0;
	DSI_RX_DATA_REG read_data0;
	static cmdqBackupSlotHandle hSlot;

	if (DISP_MODULE_DSIDUAL == module)
		dsi_i = 0;
	else
		dsi_i = DSI_MODULE_to_ID(module);

	dsi_params = &_dsi_context[dsi_i].dsi_params;


	if (cmdq_trigger_handle == NULL) {
		DISPCHECK("cmdq_trigger_handle is NULL\n");
		return -1;
	}

	if (state == CMDQ_WAIT_LCM_TE) {
		/* need waiting te */
		if (module == DISP_MODULE_DSI0) {
			if (dsi0_te_enable == 0)
				return 0;

			ret = cmdqRecClearEventToken(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI0_TE_SOF);
			ret = cmdqRecWait(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI0_TE_SOF);
		} else if (module == DISP_MODULE_DSI1) {
			if (dsi1_te_enable == 0)
				return 0;

			ret = cmdqRecClearEventToken(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI1_TE_SOF);
			ret = cmdqRecWait(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI1_TE_SOF);
		} else if (module == DISP_MODULE_DSIDUAL) {
			if (dsidual_te_enable == 0)
				return 0;

			/* TODO: dsi 8 lane do not use te???? */
			/* ret = cmdqRecWait(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI0_TE_SOF); */
			if (_dsi_context[0].swap_port || _dsi_context[1].swap_port) {
				ret = cmdqRecClearEventToken(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI1_TE_SOF);
				ret = cmdqRecWait(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI1_TE_SOF);
			} else {
				ret = cmdqRecClearEventToken(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI0_TE_SOF);
				ret = cmdqRecWait(cmdq_trigger_handle, CMDQ_EVENT_MDP_DSI0_TE_SOF);
			}
		} else {
			DISPERR("wrong module: %s\n", ddp_get_module_name(module));
			return -1;
		}
	} else if (state == CMDQ_AFTER_STREAM_SOF) {
		if (disp_helper_get_option(DISP_HELPER_OPTION_DYNAMIC_SWITCH_UNDERFLOW_EN)) {
			/* for(i = DSI_MODULE_BEGIN(module);i <= DSI_MODULE_END(module);i++) */
			{
				/* DSI_POLLREG32(cmdq_trigger_handle, &DSI_REG[i]->DSI_STATE_DBG6, 0x00000001, 0x0); */
				DSI_POLLREG32(cmdq_trigger_handle, &DSI_REG[0]->DSI_STATE_DBG9, 0x0000000f, 0x1);
			}
		}
	} else if (state == CMDQ_CHECK_IDLE_AFTER_STREAM_EOF) {
		/* need waiting te */
		if (module == DISP_MODULE_DSI0) {
			ret = cmdqRecPoll(cmdq_trigger_handle, 0x1401b00c, 0, 0x80000000);
		} else if (module == DISP_MODULE_DSI1) {
			ret = cmdqRecPoll(cmdq_trigger_handle, 0x1401c00c, 0, 0x80000000);
		} else if (module == DISP_MODULE_DSIDUAL) {
			ret = cmdqRecPoll(cmdq_trigger_handle, 0x1401b00c, 0, 0x80000000);
/* TODO: trace root cause. workaround. dualdsi hanging because waiting dis1 */
/* ret = cmdqRecPoll(cmdq_trigger_handle, 0x1401c00c, 0, 0x80000000); */
		} else {
			DISPERR("wrong module: %s\n", ddp_get_module_name(module));
			return -1;
		}
	} else if (state == CMDQ_ESD_CHECK_READ) {
		/* enable dsi interrupt: RD_RDY/CMD_DONE (need do this here?) */
		DSI_OUTREGBIT(cmdq_trigger_handle, DSI_INT_ENABLE_REG, DSI_REG[dsi_i]->DSI_INTEN, RD_RDY, 1);
		DSI_OUTREGBIT(cmdq_trigger_handle, DSI_INT_ENABLE_REG, DSI_REG[dsi_i]->DSI_INTEN, CMD_DONE, 1);
		if (module == DISP_MODULE_DSIDUAL) {
			/* disable dual dsi */
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
		}
		for (i = 0; i < 3; i++) {
			if (dsi_params->lcm_esd_check_table[i].cmd == 0)
				break;

			/* 0. send read lcm command(short packet) */
			t0.CONFG = 0x04;	/* /BTA */
			t0.Data0 = dsi_params->lcm_esd_check_table[i].cmd;
			/* / 0xB0 is used to distinguish DCS cmd or Gerneric cmd, is that Right??? */
			t0.Data_ID = (t0.Data0 < 0xB0) ? DSI_DCS_READ_PACKET_ID : DSI_GERNERIC_READ_LONG_PACKET_ID;
			t0.Data1 = 0;


			/* write DSI CMDQ */
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_CMDQ_REG[dsi_i]->data[0], 0x00013700);
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_CMDQ_REG[dsi_i]->data[1], AS_UINT32(&t0));
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_CMDQ_SIZE, 2);

			/* start DSI */
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_START, 0);
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_START, 1);

			/* 1. wait DSI RD_RDY(must clear, in case of cpu RD_RDY interrupt handler) */
			if (dsi_i == 0) {
				ret =
				    cmdqRecPoll(cmdq_trigger_handle,
						disp_addr_convert((unsigned long)(DSI_REG[0]) + 0xc), 0x1, 0x00000001);
				ret =
				    cmdqRecWrite(cmdq_trigger_handle,
						 disp_addr_convert((unsigned long)(DSI_REG[0]) + 0xc), 0x0, 0x00000001);
			} else {	/* DSI1 */

				ret =
				    cmdqRecPoll(cmdq_trigger_handle,
						disp_addr_convert((unsigned long)(DSI_REG[1]) + 0xc), 0x1, 0x00000001);
				ret =
				    cmdqRecWrite(cmdq_trigger_handle,
						 disp_addr_convert((unsigned long)(DSI_REG[1]) + 0xc), 0x0, 0x00000001);
			}

			/* 2. save RX data */
			if (hSlot) {
				cmdqRecBackupRegisterToSlot(cmdq_trigger_handle, hSlot, i,
							    dsi_i ? disp_addr_convert((unsigned long)(DSI_REG[1]) +
										      0x74) :
							    disp_addr_convert((unsigned long)(DSI_REG[0]) + 0x74));
			}


			/* 3. write RX_RACK */
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_RACK_REG, DSI_REG[dsi_i]->DSI_RACK, DSI_RACK, 1);

			/* 4. polling not busy(no need clear) */
			if (dsi_i == 0) {
				ret =
				    cmdqRecPoll(cmdq_trigger_handle,
						disp_addr_convert((unsigned long)(DSI_REG[0]) + 0x0c), 0, 0x80000000);
			} else {	/* DSI1 */

				ret =
				    cmdqRecPoll(cmdq_trigger_handle,
						disp_addr_convert((unsigned long)(DSI_REG[1]) + 0x0c), 0, 0x80000000);
			}
			/* loop: 0~4 */
		}
		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[0]->DSI_START, 0);
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[1]->DSI_START, 0);
			/* enable dual dsi */
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 1);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 1);
		}

		/* DSI_OUTREGBIT(cmdq_trigger_handle, DSI_INT_ENABLE_REG,DSI_REG[dsi_i]->DSI_INTEN,RD_RDY,0); */
	} else if (state == CMDQ_ESD_CHECK_CMP) {

		DISPCHECK("[DSI]enter cmp\n");
		/* cmp just once and only 1 return value */
		for (i = 0; i < 3; i++) {
			if (dsi_params->lcm_esd_check_table[i].cmd == 0)
				break;

			DISPCHECK("[DSI]enter cmp i=%d\n", i);

			/* read data */
			if (hSlot) {
				/* read from slot */
				cmdqBackupReadSlot(hSlot, i, &read_data0);
			} else {
				/* read from dsi , support only one cmd read */
				if (i == 0)
					DSI_OUTREG32(NULL, &read_data0, AS_UINT32(&DSI_REG[dsi_i]->DSI_RX_DATA0));
			}

			MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse, AS_UINT32(&read_data0),
				       AS_UINT32(&(dsi_params->lcm_esd_check_table[i])));

			DISPCHECK("[DSI]enter cmp read_data0 byte0=0x%x byte1=0x%x byte2=0x%x byte3=0x%x\n",
				  read_data0.byte0, read_data0.byte1, read_data0.byte2, read_data0.byte3);
			DISPCHECK
			    ("[DSI]enter cmp check_table cmd=0x%x,count=0x%x,para_list[0]=0x%x,para_list[1]=0x%x\n",
			     dsi_params->lcm_esd_check_table[i].cmd, dsi_params->lcm_esd_check_table[i].count,
			     dsi_params->lcm_esd_check_table[i].para_list[0],
			     dsi_params->lcm_esd_check_table[i].para_list[1]);
			DISPCHECK("[DSI]enter cmp DSI+0x200=0x%x\n", AS_UINT32(DDP_REG_BASE_DSI0 + 0x200));
			DISPCHECK("[DSI]enter cmp DSI+0x204=0x%x\n", AS_UINT32(DDP_REG_BASE_DSI0 + 0x204));
			DISPCHECK("[DSI]enter cmp DSI+0x60=0x%x\n", AS_UINT32(DDP_REG_BASE_DSI0 + 0x60));
			DISPCHECK("[DSI]enter cmp DSI+0x74=0x%x\n", AS_UINT32(DDP_REG_BASE_DSI0 + 0x74));
			DISPCHECK("[DSI]enter cmp DSI+0x88=0x%x\n", AS_UINT32(DDP_REG_BASE_DSI0 + 0x88));
			DISPCHECK("[DSI]enter cmp DSI+0x0c=0x%x\n", AS_UINT32(DDP_REG_BASE_DSI0 + 0x0c));

			if (read_data0.byte1 == dsi_params->lcm_esd_check_table[i].para_list[0]) {
				/* clear rx data */
				/* DSI_OUTREG32(NULL, &DSI_REG[dsi_i]->DSI_RX_DATA0,0); */
				ret = 0;	/* esd pass */
			} else {
				ret = 1;	/* esd fail */
				break;
			}
		}

	} else if (state == CMDQ_ESD_ALLC_SLOT) {
		/* create 3 slot */
		cmdqBackupAllocateSlot(&hSlot, 3);
	} else if (state == CMDQ_ESD_FREE_SLOT) {
		if (hSlot) {
			cmdqBackupFreeSlot(hSlot);
			hSlot = 0;
		}
	} else if (state == CMDQ_STOP_VDO_MODE) {
		/* use cmdq to stop dsi vdo mode */
		/* 0. set dsi cmd mode */
		DSI_SetMode(module, cmdq_trigger_handle, CMD_MODE);

		/* 1. polling dsi not busy */
		i = DSI_MODULE_BEGIN(module);
		if (i == 0) {
			ret = cmdqRecPoll(cmdq_trigger_handle, 0x1401b00c, 0, 0x80000000);

			i = DSI_MODULE_END(module);
			if (i == 1)
				ret = cmdqRecPoll(cmdq_trigger_handle, 0x1401c00c, 0, 0x80000000);
		} else {	/* DSI1 */

			ret = cmdqRecPoll(cmdq_trigger_handle, 0x1401c00c, 0, 0x80000000);
		}

		/* 2.dual dsi need do reset DSI_DUAL_EN/DSI_START */
		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);
		}

		/* 3.disable HS */
		/* DSI_clk_HS_mode(module, cmdq_trigger_handle, FALSE); */

	} else if (state == CMDQ_START_VDO_MODE) {

		/* 0. dual dsi set DSI_START/DSI_DUAL_EN */
		if (module == DISP_MODULE_DSIDUAL) {
			/* must set DSI_START to 0 before set dsi_dual_en, don't know why.2014.02.15 */
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[1]->DSI_START, DSI_START, 0);

			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 1);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG, DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 1);

		}

		/* 1. set dsi vdo mode */
		DSI_SetMode(module, cmdq_trigger_handle, dsi_params->mode);

		/* 2. enable HS */
		/* DSI_clk_HS_mode(module, cmdq_trigger_handle, TRUE); */

		/* 3. enable mutex */
		/* ddp_mutex_enable(mutex_id_for_latest_trigger,0,cmdq_trigger_handle); */

		/* 4. start dsi */
		/* DSI_Start(module, cmdq_trigger_handle); */

	} else if (state == CMDQ_DSI_RESET) {
		DISPCHECK("CMDQ Timeout, Reset DSI\n");
		DSI_DumpRegisters(module, 1);
		DSI_Reset(module, NULL);
	}

	return ret;
}

DDP_MODULE_DRIVER ddp_driver_dsi0 = {
	.module = DISP_MODULE_DSI0,
	.init = ddp_dsi_init,
	.deinit = ddp_dsi_deinit,
	.config = ddp_dsi_config,
	.build_cmdq = ddp_dsi_build_cmdq,
	.trigger = ddp_dsi_trigger,
	.start = ddp_dsi_start,
	.stop = ddp_dsi_stop,
	.reset = ddp_dsi_reset,
	.power_on = ddp_dsi_power_on,
	.power_off = ddp_dsi_power_off,
	.is_idle = ddp_dsi_is_idle,
	.is_busy = ddp_dsi_is_busy,
	.dump_info = ddp_dsi_dump,
	.set_lcm_utils = ddp_dsi_set_lcm_utils,
	.ioctl = ddp_dsi_ioctl
};

DDP_MODULE_DRIVER ddp_driver_dsi1 = {
	.module = DISP_MODULE_DSI1,
	.init = ddp_dsi_init,
	.deinit = ddp_dsi_deinit,
	.config = ddp_dsi_config,
	.build_cmdq = ddp_dsi_build_cmdq,
	.trigger = ddp_dsi_trigger,
	.start = ddp_dsi_start,
	.stop = ddp_dsi_stop,
	.reset = ddp_dsi_reset,
	.power_on = ddp_dsi_power_on,
	.power_off = ddp_dsi_power_off,
	.is_idle = ddp_dsi_is_idle,
	.is_busy = ddp_dsi_is_busy,
	.dump_info = ddp_dsi_dump,
	.set_lcm_utils = ddp_dsi_set_lcm_utils,
	.ioctl = ddp_dsi_ioctl
};


DDP_MODULE_DRIVER ddp_driver_dsidual = {
	.module = DISP_MODULE_DSIDUAL,
	.init = ddp_dsi_init,
	.deinit = ddp_dsi_deinit,
	.config = ddp_dsi_config,
	.build_cmdq = ddp_dsi_build_cmdq,
	.trigger = ddp_dsi_trigger,
	.start = ddp_dsi_start,
	.stop = ddp_dsi_stop,
	.reset = ddp_dsi_reset,
	.power_on = ddp_dsi_power_on,
	.power_off = ddp_dsi_power_off,
	.is_idle = ddp_dsi_is_idle,
	.is_busy = ddp_dsi_is_busy,
	.dump_info = ddp_dsi_dump,
	.set_lcm_utils = ddp_dsi_set_lcm_utils,
	.ioctl = ddp_dsi_ioctl
};


const LCM_UTIL_FUNCS PM_lcm_utils_dsi0 = {
	.set_reset_pin = lcm_set_reset_pin,
	.udelay = lcm_udelay,
	.mdelay = lcm_mdelay,
	.dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI0,
	.dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI0
};

void *get_dsi_params_handle(UINT32 dsi_idx)
{
	if (dsi_idx != PM_DSI1)
		return (void *)(&_dsi_context[0].dsi_params);
	else
		return (void *)(&_dsi_context[1].dsi_params);
}

INT32 DSI_ssc_enable(UINT32 dsi_index, UINT32 en);

UINT32 PanelMaster_get_TE_status(UINT32 dsi_idx)
{
	if (dsi_idx == PM_DSI0 || dsi_idx == PM_DSI_DUAL)
		return dsi0_te_enable ? 1 : 0;
	else
		return dsi1_te_enable ? 1 : 0;
}

UINT32 PanelMaster_get_CC(UINT32 dsi_idx)
{
	DSI_TXRX_CTRL_REG tmp_reg;
	UINT32 idx = (dsi_idx == PM_DSI0 || dsi_idx == PM_DSI_DUAL) ? 0 : 1;
	DSI_READREG32(PDSI_TXRX_CTRL_REG, &tmp_reg, &DSI_REG[idx]->DSI_TXRX_CTRL);
	return tmp_reg.HSTX_CKLP_EN ? 1 : 0;
}

void PanelMaster_set_CC(UINT32 dsi_index, UINT32 enable)
{
	pr_info("set_cc :%d\n", enable);
	if (dsi_index == PM_DSI0) {
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[0]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
	} else if (dsi_index == PM_DSI1) {
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[1]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
	} else if (dsi_index == PM_DSI_DUAL) {
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[0]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[1]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
	}
}

void PanelMaster_DSI_set_timing(UINT32 dsi_index, MIPI_TIMING timing)
{
	UINT32 hbp_byte, hsa_byte;
	LCM_DSI_PARAMS *dsi_params;
	int fbconfig_dsiTmpBufBpp = 0;
	if (_dsi_context[dsi_index].dsi_params.data_format.format == LCM_DSI_FORMAT_RGB565)
		fbconfig_dsiTmpBufBpp = 2;
	else
		fbconfig_dsiTmpBufBpp = 3;
	dsi_params = get_dsi_params_handle(dsi_index);
	switch (timing.type) {
	case LPX:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, LPX, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, LPX, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, LPX, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, LPX, timing.value);
		}
		break;
	case HS_PRPR:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, HS_PRPR, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, HS_PRPR, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, HS_PRPR, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, HS_PRPR, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON0_REG,DSI_REG->DSI_PHY_TIMECON0,HS_PRPR,timing.value); */
		break;
	case HS_ZERO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, HS_ZERO, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, HS_ZERO, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, HS_ZERO, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, HS_ZERO, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON0_REG,DSI_REG->DSI_PHY_TIMECON0,HS_ZERO,timing.value); */
		break;
	case HS_TRAIL:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, HS_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, HS_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, HS_TRAIL, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, HS_TRAIL, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON0_REG,DSI_REG->DSI_PHY_TIMECON0,HS_TRAIL,timing.value); */
		break;
	case TA_GO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, TA_GO, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, TA_GO, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, TA_GO, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, TA_GO, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,TA_GO,timing.value); */
		break;
	case TA_SURE:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, TA_SURE, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, TA_SURE, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, TA_SURE, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, TA_SURE, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,TA_SURE,timing.value); */
		break;
	case TA_GET:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, TA_GET, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, TA_GET, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, TA_GET, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, TA_GET, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,TA_GET,timing.value); */
		break;
	case DA_HS_EXIT:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				      timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				      timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,DA_HS_EXIT,timing.value); */
		break;
	case CONT_DET:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2, CONT_DET, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2, CONT_DET, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2, CONT_DET, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2, CONT_DET, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON2_REG,DSI_REG->DSI_PHY_TIMECON2,CONT_DET,timing.value); */
		break;
	case CLK_ZERO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2, CLK_ZERO, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2, CLK_ZERO, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2, CLK_ZERO, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2, CLK_ZERO, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON2_REG,DSI_REG->DSI_PHY_TIMECON2,CLK_ZERO,timing.value); */
		break;
	case CLK_TRAIL:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2, CLK_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2, CLK_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2, CLK_TRAIL, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2, CLK_TRAIL, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON2_REG,DSI_REG->DSI_PHY_TIMECON2,CLK_TRAIL,timing.value); */
		break;
	case CLK_HS_PRPR:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				      timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				      timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON3_REG,DSI_REG->DSI_PHY_TIMECON3,CLK_HS_PRPR,timing.value); */
		break;
	case CLK_HS_POST:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_POST,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_POST,
				      timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_POST,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_POST,
				      timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON3_REG,DSI_REG->DSI_PHY_TIMECON3,CLK_HS_POST,timing.value); */
		break;
	case CLK_HS_EXIT:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				      timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				      timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON3_REG,DSI_REG->DSI_PHY_TIMECON3,CLK_HS_EXIT,timing.value); */
		break;
	case HPW:
		if (dsi_params->mode == SYNC_EVENT_VDO_MODE || dsi_params->mode == BURST_VDO_MODE)
			;/* do nothing */
		else
			timing.value = (timing.value * fbconfig_dsiTmpBufBpp - 10);

		timing.value = ALIGN_TO((timing.value), 4);
		if (dsi_index == PM_DSI0) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HSA_WC, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HSA_WC, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HSA_WC, timing.value);
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HSA_WC, timing.value);
		}
		break;
	case HFP:
		timing.value = timing.value * fbconfig_dsiTmpBufBpp - 12;
		timing.value = ALIGN_TO(timing.value, 4);
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[0]->DSI_HFP_WC, HFP_WC, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[1]->DSI_HFP_WC, HFP_WC, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[0]->DSI_HFP_WC, HFP_WC, timing.value);
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[1]->DSI_HFP_WC, HFP_WC, timing.value);
		}
		break;
	case HBP:
		if (dsi_params->mode == SYNC_EVENT_VDO_MODE || dsi_params->mode == BURST_VDO_MODE) {
			hbp_byte = ((timing.value + dsi_params->horizontal_sync_active) * fbconfig_dsiTmpBufBpp - 10);
		} else {
/* hsa_byte = (dsi_params->horizontal_sync_active * fbconfig_dsiTmpBufBpp - 10); */
			hbp_byte = timing.value * fbconfig_dsiTmpBufBpp - 10;
		}
		if (dsi_index == PM_DSI0) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
		} else {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
		}

		break;
	case VPW:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[0]->DSI_VACT_NL, VACT_NL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[1]->DSI_VACT_NL, VACT_NL, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[0]->DSI_VACT_NL, VACT_NL, timing.value);
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[1]->DSI_VACT_NL, VACT_NL, timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VACT_NL,timing.value); */
		break;
	case VFP:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[0]->DSI_VFP_NL, VFP_NL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[1]->DSI_VFP_NL, VFP_NL, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[0]->DSI_VFP_NL, VFP_NL, timing.value);
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[1]->DSI_VFP_NL, VFP_NL, timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VFP_NL, timing.value); */
		break;
	case VBP:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[0]->DSI_VBP_NL, VBP_NL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[1]->DSI_VBP_NL, VBP_NL, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[0]->DSI_VBP_NL, VBP_NL, timing.value);
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[1]->DSI_VBP_NL, VBP_NL, timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VBP_NL, timing.value); */
		break;
	case SSC_EN:
		DSI_ssc_enable(dsi_index, timing.value);
		break;
	default:
		pr_info("fbconfig dsi set timing :no such type!!\n");
	}
}

INT32 DSI_ssc_enable(UINT32 dsi_index, UINT32 en)
{
	UINT32 disable = en ? 0 : 1;
	if (dsi_index == PM_DSI0) {
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[0]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		_dsi_context[0].dsi_params.ssc_disable = disable;
	} else if (dsi_index == PM_DSI1) {
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[1]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		_dsi_context[1].dsi_params.ssc_disable = disable;
	} else if (dsi_index == PM_DSI_DUAL) {
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[0]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[1]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		_dsi_context[0].dsi_params.ssc_disable = _dsi_context[1].dsi_params.ssc_disable = disable;
	}
	return 0;
}

UINT32 PanelMaster_get_dsi_timing(UINT32 dsi_index, MIPI_SETTING_TYPE type)
{
	UINT32 dsi_val;
	PDSI_REGS dsi_reg;
	int fbconfig_dsiTmpBufBpp = 0;
	if (_dsi_context[dsi_index].dsi_params.data_format.format == LCM_DSI_FORMAT_RGB565)
		fbconfig_dsiTmpBufBpp = 2;
	else
		fbconfig_dsiTmpBufBpp = 3;
	if ((dsi_index == PM_DSI0) || (dsi_index == PM_DSI_DUAL))
		dsi_reg = DSI_REG[0];
	else
		dsi_reg = DSI_REG[1];
	switch (type) {
	case LPX:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.LPX;
		return dsi_val;
	case HS_PRPR:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_PRPR;
		return dsi_val;
	case HS_ZERO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_ZERO;
		return dsi_val;
	case HS_TRAIL:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_TRAIL;
		return dsi_val;
	case TA_GO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_GO;
		return dsi_val;
	case TA_SURE:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_SURE;
		return dsi_val;
	case TA_GET:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_GET;
		return dsi_val;
	case DA_HS_EXIT:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.DA_HS_EXIT;
		return dsi_val;
	case CONT_DET:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CONT_DET;
		return dsi_val;
	case CLK_ZERO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CLK_ZERO;
		return dsi_val;
	case CLK_TRAIL:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CLK_TRAIL;
		return dsi_val;
	case CLK_HS_PRPR:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_PRPR;
		return dsi_val;
	case CLK_HS_POST:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_POST;
		return dsi_val;
	case CLK_HS_EXIT:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_EXIT;
		return dsi_val;
	case HPW:
		{
			DSI_HSA_WC_REG tmp_reg;
			DSI_READREG32(PDSI_HSA_WC_REG, &tmp_reg, &dsi_reg->DSI_HSA_WC);
			dsi_val = (tmp_reg.HSA_WC + 10) / fbconfig_dsiTmpBufBpp;
			return dsi_val;
		}
	case HFP:
		{
			DSI_HFP_WC_REG tmp_hfp;
			DSI_READREG32(PDSI_HFP_WC_REG, &tmp_hfp, &dsi_reg->DSI_HFP_WC);
			dsi_val = ((tmp_hfp.HFP_WC + 12) / fbconfig_dsiTmpBufBpp);
			return dsi_val;
		}
	case HBP:
		{
			DSI_HBP_WC_REG tmp_hbp;
			LCM_DSI_PARAMS *dsi_params;
			dsi_params = get_dsi_params_handle(dsi_index);
			OUTREG32(&tmp_hbp, AS_UINT32(&dsi_reg->DSI_HBP_WC));
			if (dsi_params->mode == SYNC_EVENT_VDO_MODE || dsi_params->mode == BURST_VDO_MODE)
				return ((tmp_hbp.HBP_WC + 10) / fbconfig_dsiTmpBufBpp -
					dsi_params->horizontal_sync_active);
			else
				return (tmp_hbp.HBP_WC + 10) / fbconfig_dsiTmpBufBpp;
		}
	case VPW:
		{
			DSI_VACT_NL_REG tmp_vpw;
			DSI_READREG32(PDSI_VACT_NL_REG, &tmp_vpw, &dsi_reg->DSI_VACT_NL);
			dsi_val = tmp_vpw.VACT_NL;
			return dsi_val;
		}
	case VFP:
		{
			DSI_VFP_NL_REG tmp_vfp;
			DSI_READREG32(PDSI_VFP_NL_REG, &tmp_vfp, &dsi_reg->DSI_VFP_NL);
			dsi_val = tmp_vfp.VFP_NL;
			return dsi_val;
		}
	case VBP:
		{
			DSI_VBP_NL_REG tmp_vbp;
			DSI_READREG32(PDSI_VBP_NL_REG, &tmp_vbp, &dsi_reg->DSI_VBP_NL);
			dsi_val = tmp_vbp.VBP_NL;
			return dsi_val;
		}
	case SSC_EN:
		{
			if (_dsi_context[dsi_index].dsi_params.ssc_disable)
				dsi_val = 0;
			else
				dsi_val = 1;
			return dsi_val;
		}
	default:
		pr_info("fbconfig dsi set timing :no such type!!\n");
	}
	dsi_val = 0;
	return dsi_val;
}

unsigned int PanelMaster_set_PM_enable(unsigned int value)
{
	atomic_set(&PMaster_enable, value);
}
