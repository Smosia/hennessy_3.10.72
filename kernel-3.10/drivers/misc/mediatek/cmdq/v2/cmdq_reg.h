#ifndef __CMDQ_REG_H__
#define __CMDQ_REG_H__

#include <mach/sync_write.h>
#include <asm/io.h>

#include "cmdq_core.h"
#include "cmdq_device.h"

#define MMSYS_CONFIG_BASE	cmdq_dev_get_module_base_VA_MMSYS_CONFIG()

#define GCE_BASE_PA			cmdq_dev_get_module_base_PA_GCE()
#define GCE_BASE_VA			cmdq_dev_get_module_base_VA_GCE()

#define CMDQ_CORE_WARM_RESET         (GCE_BASE_VA + 0x000)
#define CMDQ_CURR_IRQ_STATUS         (GCE_BASE_VA + 0x010)
#define CMDQ_SECURE_IRQ_STATUS       (GCE_BASE_VA + 0x014)
#define CMDQ_CURR_LOADED_THR         (GCE_BASE_VA + 0x018)
#define CMDQ_THR_SLOT_CYCLES         (GCE_BASE_VA + 0x030)
#define CMDQ_THR_EXEC_CYCLES         (GCE_BASE_VA + 0x034)
#define CMDQ_THR_TIMEOUT_TIMER       (GCE_BASE_VA + 0x038)
#define CMDQ_BUS_CONTROL_TYPE        (GCE_BASE_VA + 0x040)
#define CMDQ_CURR_INST_ABORT         (GCE_BASE_VA + 0x020)
#define CMDQ_CURR_REG_ABORT          (GCE_BASE_VA + 0x050)

#define CMDQ_SECURITY_STA(id)        (GCE_BASE_VA + (0x030 * id) + 0x024)
#define CMDQ_SECURITY_SET(id)        (GCE_BASE_VA + (0x030 * id) + 0x028)
#define CMDQ_SECURITY_CLR(id)        (GCE_BASE_VA + (0x030 * id) + 0x02C)

#define CMDQ_SYNC_TOKEN_ID           (GCE_BASE_VA + 0x060)
#define CMDQ_SYNC_TOKEN_VAL          (GCE_BASE_VA + 0x064)
#define CMDQ_SYNC_TOKEN_UPD          (GCE_BASE_VA + 0x068)

#define CMDQ_GPR_R32(id)             (GCE_BASE_VA + (0x004 * id) + 0x80)
#define CMDQ_GPR_R32_PA(id)          (GCE_BASE_PA + (0x004 * id) + 0x80)

#define CMDQ_THR_WARM_RESET(id)      (GCE_BASE_VA + (0x080 * id) + 0x100)
#define CMDQ_THR_ENABLE_TASK(id)     (GCE_BASE_VA + (0x080 * id) + 0x104)
#define CMDQ_THR_SUSPEND_TASK(id)    (GCE_BASE_VA + (0x080 * id) + 0x108)
#define CMDQ_THR_CURR_STATUS(id)     (GCE_BASE_VA + (0x080 * id) + 0x10C)
#define CMDQ_THR_IRQ_STATUS(id)      (GCE_BASE_VA + (0x080 * id) + 0x110)
#define CMDQ_THR_IRQ_ENABLE(id)      (GCE_BASE_VA + (0x080 * id) + 0x114)
#define CMDQ_THR_SECURITY(id)        (GCE_BASE_VA + (0x080 * id) + 0x118)
#define CMDQ_THR_CURR_ADDR(id)       (GCE_BASE_VA + (0x080 * id) + 0x120)
#define CMDQ_THR_END_ADDR(id)        (GCE_BASE_VA + (0x080 * id) + 0x124)
#define CMDQ_THR_EXEC_CNT(id)        (GCE_BASE_VA + (0x080 * id) + 0x128)
#define CMDQ_THR_WAIT_TOKEN(id)      (GCE_BASE_VA + (0x080 * id) + 0x130)
#define CMDQ_THR_CFG(id)             (GCE_BASE_VA + (0x080 * id) + 0x140)
#define CMDQ_THR_INST_CYCLES(id)     (GCE_BASE_VA + (0x080 * id) + 0x150)
#define CMDQ_THR_INST_THRESX(id)     (GCE_BASE_VA + (0x080 * id) + 0x154)

#define CMDQ_THR_EXEC_CNT_PA(id)     (GCE_BASE_PA + (0x080 * id) + 0x128)

#ifdef CMDQ_USE_LEGACY
#define CMDQ_TEST_MMSYS_DUMMY_OFFSET (0x890)
#else
/* use DUMMY_3(0x89C) because DUMMY_0/1 is CLKMGR SW */
#define CMDQ_TEST_MMSYS_DUMMY_OFFSET (0x89C)
#endif

#define CMDQ_TEST_MMSYS_DUMMY_PA     (0x14000000 + CMDQ_TEST_MMSYS_DUMMY_OFFSET)
#define CMDQ_TEST_MMSYS_DUMMY_VA     (cmdq_dev_get_module_base_VA_MMSYS_CONFIG() + CMDQ_TEST_MMSYS_DUMMY_OFFSET)

#define CMDQ_APXGPT2_COUNT           (cmdq_dev_get_APXGPT2_count())

#define CMDQ_REG_GET32(addr)         (readl((void *)addr) & 0xFFFFFFFF)
#define CMDQ_REG_GET16(addr)         (readl((void *)addr) & 0x0000FFFF)


#define CMDQ_REG_GET64_GPR_PX(id)    cmdq_core_get_GPR64(id)
#define CMDQ_REG_SET64_GPR_PX(id, value)    cmdq_core_set_GPR64(id, value)

#define CMDQ_REG_SET32(addr, val)    mt_reg_sync_writel(val, (addr))


#endif				/* __CMDQ_REG_H__ */
