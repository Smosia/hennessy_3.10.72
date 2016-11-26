#include "cmdq_device.h"
#include "cmdq_core.h"
#include "cmdq_virtual.h"

#ifndef CMDQ_OF_SUPPORT
#include <mach/mt_irq.h>
#endif

/* device tree */
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>

typedef struct CmdqDeviceStruct {
	struct device *pDev;
#ifdef CMDQ_USE_CCF
	struct clk *clk_gce;
#endif				/* defined(CMDQ_USE_CCF) */
	long regBaseVA;		/* considering 64 bit kernel, use long */
	long regBasePA;
	uint32_t irqId;
	uint32_t irqSecId;
} CmdqDeviceStruct;
static CmdqDeviceStruct gCmdqDev;
static long gMMSYS_CONFIG_Base_VA;
static long gAPXGPT2Count;

struct device *cmdq_dev_get(void)
{
	return gCmdqDev.pDev;
}

const uint32_t cmdq_dev_get_irq_id(void)
{
	return gCmdqDev.irqId;
}

const uint32_t cmdq_dev_get_irq_secure_id(void)
{
	return gCmdqDev.irqSecId;
}

const long cmdq_dev_get_module_base_VA_GCE(void)
{
	return gCmdqDev.regBaseVA;
}

const long cmdq_dev_get_module_base_PA_GCE(void)
{
	return gCmdqDev.regBasePA;
}

const long cmdq_dev_get_module_base_VA_MMSYS_CONFIG(void)
{
	return gMMSYS_CONFIG_Base_VA;
}

const long cmdq_dev_get_APXGPT2_count(void)
{
	return gAPXGPT2Count;
}

void cmdq_dev_init_module_base_VA(void)
{
	gMMSYS_CONFIG_Base_VA = 0;

#ifdef CMDQ_OF_SUPPORT
	gMMSYS_CONFIG_Base_VA = cmdq_dev_alloc_module_base_VA_by_name("mediatek,mmsys_config");

	if (0 == gMMSYS_CONFIG_Base_VA)
		gMMSYS_CONFIG_Base_VA = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MMSYS_CONFIG");
#endif

	cmdq_mdp_get_func()->initModuleBaseVA();
}

void cmdq_dev_deinit_module_base_VA(void)
{
#ifdef CMDQ_OF_SUPPORT
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MMSYS_CONFIG());
#else
	/* do nothing, registers' IOMAP will be destroyed by platform */
#endif

	cmdq_mdp_get_func()->deinitModuleBaseVA();
}

const long cmdq_dev_alloc_module_base_VA_by_name(const char *name)
{
	unsigned long VA;
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, name);
	VA = (unsigned long)of_iomap(node, 0);
	CMDQ_LOG("DEV: VA(%s): 0x%lx\n", name, VA);
	return VA;
}

void cmdq_dev_free_module_base_VA(const long VA)
{
	iounmap((void *)VA);
}

long cmdq_dev_get_gce_node_PA(struct device_node *node, int index)
{
	struct resource res;
	int status;
	long regBasePA = 0L;

	do {
		status = of_address_to_resource(node, index, &res);
		if (status < 0)
			break;

		regBasePA = (0L | res.start);
	} while (0);

	return regBasePA;
}

#ifndef CMDQ_USE_CCF
#define IMP_ENABLE_HW_CLOCK(FN_NAME, HW_NAME)	\
uint32_t cmdq_dev_enable_clock_##FN_NAME(bool enable)	\
{		\
	return cmdq_dev_enable_mtk_clock(enable, MT_CG_DISP0_##HW_NAME, "CMDQ_MDP_"#HW_NAME);	\
}

uint32_t cmdq_dev_enable_mtk_clock(bool enable, cgCLKID gateId, char *name)
{
	if (enable)
		enable_clock(gateId, name);
	else
		disable_clock(gateId, name);
	return 0;
}

bool cmdq_dev_mtk_clock_is_enable(cgCLKID gateId)
{
	return clock_is_on(gateId);
}

IMP_ENABLE_HW_CLOCK(SMI_COMMON, SMI_COMMON);

#else

typedef struct CmdqModuleClock {
	struct clk *clk_SMI_COMMON;
	struct clk *clk_SMI_LARB0;
	struct clk *clk_MTCMOS_DIS;
} CmdqModuleClock;
static CmdqModuleClock gCmdqModuleClock;

#define IMP_ENABLE_HW_CLOCK(FN_NAME, HW_NAME)	\
uint32_t cmdq_dev_enable_clock_##FN_NAME(bool enable)	\
{		\
	return cmdq_dev_enable_device_clock(enable, gCmdqModuleClock.clk_##HW_NAME, #HW_NAME "-clk");	\
}

void cmdq_dev_get_module_clock_by_dev(struct device *dev, const char *clkName,
				      struct clk **clk_module)
{
	*clk_module = devm_clk_get(dev, clkName);
	if (IS_ERR(*clk_module)) {
		/* error status print */
		CMDQ_ERR("DEV: cannot get module clock: %s\n", clkName);
	} else {
		/* message print */
		CMDQ_MSG("DEV: get module clock: %s\n", clkName);
	}
}

void cmdq_dev_get_module_clock_by_name(const char *name, const char *clkName,
				       struct clk **clk_module)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, name);

	*clk_module = of_clk_get_by_name(node, clkName);
	if (IS_ERR(*clk_module)) {
		/* error status print */
		CMDQ_ERR("DEV: byName: cannot get module clock: %s\n", clkName);
	} else {
		/* message print */
		CMDQ_MSG("DEV: byName: get module clock: %s\n", clkName);
	}
}

uint32_t cmdq_dev_enable_device_clock(bool enable, struct clk *clk_module, const char *clkName)
{
	int result = 0;

	if (IS_ERR(clk_module))
		return PTR_ERR(clk_module);

	if (enable) {
		result = clk_prepare_enable(clk_module);
		CMDQ_MSG("enable clock with module: %s, result: %d\n", clkName, result);
	} else {
		clk_disable_unprepare(clk_module);
		CMDQ_MSG("disable clock with module: %s\n", clkName);
	}

	return result;
}

bool cmdq_dev_device_clock_is_enable(struct clk *clk_module)
{
	return true;
}

/* special handle for MTCMOS */
uint32_t cmdq_dev_enable_clock_SMI_COMMON(bool enable)
{
	if (enable) {
		cmdq_dev_enable_device_clock(enable, gCmdqModuleClock.clk_MTCMOS_DIS,
					     "MTCMOS_DIS-clk");
		cmdq_dev_enable_device_clock(enable, gCmdqModuleClock.clk_SMI_COMMON,
					     "SMI_COMMON-clk");
	} else {
		cmdq_dev_enable_device_clock(enable, gCmdqModuleClock.clk_SMI_COMMON,
					     "SMI_COMMON-clk");
		cmdq_dev_enable_device_clock(enable, gCmdqModuleClock.clk_MTCMOS_DIS,
					     "MTCMOS_DIS-clk");
	}
	return 0;
}
#endif				/* !defined(CMDQ_USE_CCF) */

IMP_ENABLE_HW_CLOCK(SMI_LARB0, SMI_LARB0);
#ifdef CMDQ_USE_LEGACY
IMP_ENABLE_HW_CLOCK(MUTEX_32K, MUTEX_32K);
#endif
#undef IMP_ENABLE_HW_CLOCK

/* Common Clock Framework */
void cmdq_dev_init_module_clk(void)
{
#if defined(CMDQ_OF_SUPPORT) && defined(CMDQ_USE_CCF)
	cmdq_dev_get_module_clock_by_name("mediatek,SMI_COMMON", "smi-common",
					  &gCmdqModuleClock.clk_SMI_COMMON);
	cmdq_dev_get_module_clock_by_name("mediatek,SMI_COMMON", "smi-larb0",
					  &gCmdqModuleClock.clk_SMI_LARB0);
	cmdq_dev_get_module_clock_by_name("mediatek,SMI_COMMON", "mtcmos-dis",
					  &gCmdqModuleClock.clk_MTCMOS_DIS);
#endif
	cmdq_mdp_get_func()->initModuleCLK();
}

void cmdq_dev_enable_gce_clock(bool enable)
{
#ifndef CMDQ_USE_CCF
	cmdq_dev_enable_mtk_clock(enable, MT_CG_INFRA_GCE, "mtk_cmdq");
#else
	cmdq_dev_enable_device_clock(enable, gCmdqDev.clk_gce, "gce-clk");
#endif
}

bool cmdq_dev_gce_clock_is_enable(void)
{
#ifndef CMDQ_USE_CCF
	return cmdq_dev_mtk_clock_is_enable(MT_CG_INFRA_GCE);
#else
	return cmdq_dev_device_clock_is_enable(gCmdqDev.clk_gce);
#endif
}

void cmdq_dev_get_module_PA(const char *name, int index, long *startPA, long *endPA)
{
	int status;
	struct device_node *node = NULL;
	struct resource res;

	do {
		node = of_find_compatible_node(NULL, NULL, name);
		if (NULL == node)
			break;

		status = of_address_to_resource(node, index, &res);
		if (status < 0)
			break;

		*startPA = res.start;
		*endPA = res.end;
		CMDQ_MSG("DEV: PA(%s): start = 0x%lx, end = 0x%lx\n", name, *startPA, *endPA);
	} while (0);
}

/* Get MDP base address to user space */
void cmdq_dev_init_MDP_PA(struct device_node *node)
{
#ifdef CMDQ_OF_SUPPORT
	int status;
	uint32_t gceDispMutex[2] = {0, 0};
	uint32_t *pMDPBaseAddress = cmdq_core_get_whole_DTS_Data()->MDPBaseAddress;

	do {
		status = of_property_read_u32_array(node, "disp_mutex_reg", gceDispMutex, ARRAY_SIZE(gceDispMutex));
		if (status < 0)
			break;

		pMDPBaseAddress[CMDQ_MDP_PA_BASE_MM_MUTEX] = gceDispMutex[0];
	} while (0);
#endif
}

#ifdef CMDQ_OF_SUPPORT
void cmdq_dev_get_subsys_by_name(struct device_node *node, CMDQ_SUBSYS_ENUM subsys,
				  const char *grp_name, const char *dts_name)
{
	int status;
	uint32_t gceSubsys[3] = { 0, 0, 0 };
	SubsysStruct *gceSubsysStruct = NULL;

	do {
		if (subsys < 0 || subsys >= CMDQ_SUBSYS_MAX_COUNT)
			break;

		gceSubsysStruct = cmdq_core_get_whole_DTS_Data()->subsys;

		status = of_property_read_u32_array(node, dts_name, gceSubsys, ARRAY_SIZE(gceSubsys));
		if (status < 0) {
			gceSubsysStruct[subsys].subsysID = -1;
			break;
		}

		gceSubsysStruct[subsys].msb = gceSubsys[0];
		gceSubsysStruct[subsys].subsysID = gceSubsys[1];
		gceSubsysStruct[subsys].mask = gceSubsys[2];
		strncpy(gceSubsysStruct[subsys].grpName, grp_name, CMDQ_SUBSYS_GRPNAME_MAX-1);
	} while (0);
}

void cmdq_dev_test_subsys_correctness_impl(CMDQ_SUBSYS_ENUM subsys)
{
	SubsysStruct *gceSubsysStruct = NULL;

	if (subsys >= 0 && subsys < CMDQ_SUBSYS_MAX_COUNT) {
		gceSubsysStruct = cmdq_core_get_whole_DTS_Data()->subsys;

		if (gceSubsysStruct[subsys].subsysID != -1) {
			/* print subsys information from device tree */
			CMDQ_LOG("(%s), msb: 0x%08x, %d, 0x%08x\n",
				gceSubsysStruct[subsys].grpName, gceSubsysStruct[subsys].msb,
				gceSubsysStruct[subsys].subsysID, gceSubsysStruct[subsys].mask);
		}
	}
}
#endif

void cmdq_dev_init_subsys(struct device_node *node)
{
#ifdef CMDQ_OF_SUPPORT
#undef DECLARE_CMDQ_SUBSYS
#define DECLARE_CMDQ_SUBSYS(name, val, grp, dts_name) \
{	\
	cmdq_dev_get_subsys_by_name(node, val, #grp, #dts_name);	\
}
#include "cmdq_subsys_common.h"
#undef DECLARE_CMDQ_SUBSYS
#endif
}

#ifdef CMDQ_OF_SUPPORT
void cmdq_dev_get_event_value_by_name(struct device_node *node, CMDQ_EVENT_ENUM event, const char *dts_name)
{
	int status;
	uint32_t event_value;

	do {
		if (event < 0 || event >= CMDQ_MAX_HW_EVENT_COUNT)
			break;

		status = of_property_read_u32(node, dts_name, &event_value);
		if (status < 0)
			break;

		cmdq_core_set_event_table(event, event_value);
	} while (0);
}

void cmdq_dev_test_event_correctness_impl(CMDQ_EVENT_ENUM event, const char *event_name)
{
	int32_t eventValue = cmdq_core_get_event_value(event);

	if (eventValue >= 0 && eventValue < CMDQ_SYNC_TOKEN_MAX) {
		/* print event name from device tree */
		CMDQ_LOG("%s = %d\n", event_name, eventValue);
	}
}
#endif

void cmdq_dev_init_event_table(struct device_node *node)
{
#ifdef CMDQ_OF_SUPPORT
#undef DECLARE_CMDQ_EVENT
#define DECLARE_CMDQ_EVENT(name, val, dts_name) \
{	\
	cmdq_dev_get_event_value_by_name(node, val, #dts_name);	\
}
#include "cmdq_event_common.h"
#undef DECLARE_CMDQ_EVENT
#endif
}

void cmdq_dev_test_dts_correctness(void)
{
#ifdef CMDQ_OF_SUPPORT
#undef DECLARE_CMDQ_EVENT
#define DECLARE_CMDQ_EVENT(name, val, dts_name) \
{	\
		cmdq_dev_test_event_correctness_impl(val, #name);	\
}
#include "cmdq_event_common.h"
#undef DECLARE_CMDQ_EVENT

#undef DECLARE_CMDQ_SUBSYS
#define DECLARE_CMDQ_SUBSYS(name, val, grp, dts_name) \
{	\
		cmdq_dev_test_subsys_correctness_impl(val);	\
}
#include "cmdq_subsys_common.h"
#undef DECLARE_CMDQ_SUBSYS

	CMDQ_LOG("APXGPT2_Count = 0x%08lx\n", gAPXGPT2Count);
#endif
}

void cmdq_dev_init_device_tree(struct device_node *node)
{
	int status;
	uint32_t apxgpt2_count_value = 0;

	gAPXGPT2Count = 0;
	cmdq_core_init_DTS_data();
#ifdef CMDQ_OF_SUPPORT
	/* init GCE subsys */
	cmdq_dev_init_subsys(node);
	/* init event table */
	cmdq_dev_init_event_table(node);
	/* init MDP PA address */
	cmdq_dev_init_MDP_PA(node);
	do {
		status = of_property_read_u32(node, "apxgpt2_count", &apxgpt2_count_value);
		if (status < 0)
			break;

		gAPXGPT2Count = apxgpt2_count_value;
	} while (0);
#endif
}

void cmdq_dev_init(struct platform_device *pDevice)
{
	struct device_node *node = pDevice->dev.of_node;

	/* init cmdq device dependent data */
	do {
		memset(&gCmdqDev, 0x0, sizeof(CmdqDeviceStruct));

		gCmdqDev.pDev = &pDevice->dev;
#ifdef CMDQ_OF_SUPPORT
		gCmdqDev.regBaseVA = (unsigned long)of_iomap(node, 0);
		gCmdqDev.regBasePA = cmdq_dev_get_gce_node_PA(node, 0);
		gCmdqDev.irqId = irq_of_parse_and_map(node, 0);
		gCmdqDev.irqSecId = irq_of_parse_and_map(node, 1);
#ifdef CMDQ_USE_CCF
		gCmdqDev.clk_gce = devm_clk_get(&pDevice->dev, "GCE");
#endif				/* defined(CMDQ_USE_CCF) */
#endif

		CMDQ_LOG
		    ("[CMDQ] platform_dev: dev: %p, PA: %lx, VA: %lx, irqId: %d,  irqSecId:%d\n",
		     gCmdqDev.pDev, gCmdqDev.regBasePA, gCmdqDev.regBaseVA, gCmdqDev.irqId,
		     gCmdqDev.irqSecId);
	} while (0);

	/* init module VA */
	cmdq_dev_init_module_base_VA();
	/* init module clock */
	cmdq_dev_init_module_clk();
	/* init module PA for instruction count */
	cmdq_get_func()->initModulePAStat();
	/* init load HW information from device tree */
	cmdq_dev_init_device_tree(node);
}

void cmdq_dev_deinit(void)
{
	cmdq_dev_deinit_module_base_VA();

	/* deinit cmdq device dependent data */
	do {
#ifdef CMDQ_OF_SUPPORT
		cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_GCE());
		gCmdqDev.regBaseVA = 0;
#else
		/* do nothing */
#endif
	} while (0);
}
