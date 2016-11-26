#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/kobject.h>

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/aee.h>
#include <linux/xlog.h>
#include <mach/mt_clkmgr.h>
#include <asm/io.h>

#include <linux/ioctl.h>
#include <linux/fs.h>

#if IS_ENABLED(CONFIG_COMPAT)
#include <linux/uaccess.h>
#include <linux/compat.h>
#endif

#include "mt_smi.h"
#include "smi_reg.h"
#include "smi_common.h"
#include "smi_debug.h"
#include "smi_info_util.h"
#if defined(SMI_RO)
#include "mmdvfs_mgr.h"
#endif
#define SMI_LOG_TAG "SMI"

#define SMI_DT_SUPPORT

#define LARB_BACKUP_REG_SIZE 128

#if defined(SMI_K2)
#define SMI_COMMON_BACKUP_REG_NUM   9

#define SF_HWC_PIXEL_MAX_NORMAL  (1920 * 1080 * 7)
#define SF_HWC_PIXEL_MAX_VR   (1920 * 1080 * 4 + 1036800)	/* 4.5 FHD size */
#define SF_HWC_PIXEL_MAX_VP   (1920 * 1080 * 7)
#define SF_HWC_PIXEL_MAX_ALWAYS_GPU  (1920 * 1080 * 1)
#elif defined(SMI_RO)
#define SMI_COMMON_BACKUP_REG_NUM   10

#define SF_HWC_PIXEL_MAX_NORMAL  (2560 * 1600 * 7)
#define SF_HWC_PIXEL_MAX_VR   (2560 * 1600 * 7)
#define SF_HWC_PIXEL_MAX_VP   (2560 * 1600 * 7)
#define SF_HWC_PIXEL_MAX_ALWAYS_GPU  (2560 * 1600 * 1)
#endif


#define SMIDBG(level, x...)            \
		do {                        \
			if (smi_debug_level >= (level))    \
				SMIMSG(x);            \
		} while (0)


#if defined(SMI_RO)
#define DEFINE_ATTR_RO(_name)			\
			static struct kobj_attribute _name##_attr = {	\
				.attr	= {			\
					.name = #_name,		\
					.mode = 0444,			\
				},					\
				.show	= _name##_show,		\
			}

#define DEFINE_ATTR_RW(_name)			\
			static struct kobj_attribute _name##_attr = {	\
				.attr	= {			\
					.name = #_name,		\
					.mode = 0644,			\
				},					\
				.show	= _name##_show,		\
				.store	= _name##_store,		\
			}

#define __ATTR_OF(_name)	(&_name##_attr.attr)
#endif

SMI_struct g_SMIInfo;

/* LARB BASE ADDRESS */
static unsigned long gLarbBaseAddr[SMI_LARB_NR] = { 0 };

#if defined(SMI_K2)
/* DT porting */
unsigned long smi_reg_base_common_ext = 0;
unsigned long smi_reg_base_barb0 = 0;
unsigned long smi_reg_base_barb1 = 0;
unsigned long smi_reg_base_barb2 = 0;
unsigned long smi_reg_base_barb3 = 0;
unsigned long smi_reg_base_barb4 = 0;

#define SMI_REG_REGION_MAX 6
#define SMI_COMMON_REG_INDX 0
#define SMI_LARB0_REG_INDX 1
#define SMI_LARB1_REG_INDX 2
#define SMI_LARB2_REG_INDX 3
#define SMI_LARB3_REG_INDX 4
#define SMI_LARB4_REG_INDX 5

static unsigned long gSMIBaseAddrs[SMI_REG_REGION_MAX];


/* #ifdef SMI_DT_SUPPORT */
char *smi_get_region_name(unsigned int region_indx);
/* #endif //SMI_DT_SUPPORT */

struct smi_device {
	struct device *dev;
	void __iomem *regs[SMI_REG_REGION_MAX];
};

#elif defined(SMI_RO)

/* SMI register base address array */
unsigned long smi_reg_bases[SMI_REG_BASE_NUM];

struct smi_device {
	struct device *dev;
	void __iomem *regs[SMI_REG_BASE_NUM];
};
#endif



static struct smi_device *smi_dev;

static struct device *smiDeviceUevent;

static struct cdev *pSmiDev;

static const unsigned int larb_port_num[SMI_LARB_NR] = { SMI_LARB0_PORT_NUM,
	SMI_LARB1_PORT_NUM, SMI_LARB2_PORT_NUM, SMI_LARB3_PORT_NUM,
	SMI_LARB4_PORT_NUM
};

static unsigned short int larb0_port_backup[SMI_LARB0_PORT_NUM];
static unsigned short int larb1_port_backup[SMI_LARB1_PORT_NUM];
static unsigned short int larb2_port_backup[SMI_LARB2_PORT_NUM];
static unsigned short int larb3_port_backup[SMI_LARB3_PORT_NUM];
static unsigned short int larb4_port_backup[SMI_LARB4_PORT_NUM];

/* SMI COMMON register list to be backuped */
#if defined(SMI_K2)
static unsigned short g_smi_common_backup_reg_offset[SMI_COMMON_BACKUP_REG_NUM] = {
	0x200, 0x204, 0x208, 0x20c, 0x210, 0x214, 0x230, 0x234, 0x238};
#elif defined(SMI_RO)
static unsigned short g_smi_common_backup_reg_offset[SMI_COMMON_BACKUP_REG_NUM] = {
	0x200, 0x204, 0x208, 0x20c, 0x210, 0x214, 0x220, 0x230, 0x234, 0x238 };
#endif
static unsigned int g_smi_common_backup[SMI_COMMON_BACKUP_REG_NUM];

static unsigned char larb_vc_setting[SMI_LARB_NR] = { 0, 2, 0, 1, 2 };

static unsigned short int *larb_port_backup[SMI_LARB_NR] = {
	larb0_port_backup, larb1_port_backup, larb2_port_backup, larb3_port_backup, larb4_port_backup };

/* To keep the HW's init value */
static int is_default_value_saved;
static unsigned int default_val_smi_l1arb[SMI_LARB_NR] = { 0 };

static unsigned int wifi_disp_transaction;


/* debug level */
static unsigned int smi_debug_level;

/* tuning mode, 1 for register ioctl */
static unsigned int smi_tuning_mode;

static unsigned int smi_profile = SMI_BWC_SCEN_NORMAL;

static unsigned int *pLarbRegBackUp[SMI_LARB_NR];
static int g_bInited;

MTK_SMI_BWC_MM_INFO g_smi_bwc_mm_info = { 0, 0, {0, 0}, {0, 0}, {0,
	0}, {0, 0}, 0, 0, 0, SF_HWC_PIXEL_MAX_NORMAL
};

#if defined(SMI_K2)
static unsigned long smi_reg_pa_base[SMI_REG_REGION_MAX] = { 0x14022000,
	0x14021000, 0x16010000, 0x15001000, 0x18001000, 0x17002000,
};
#endif

static void initSetting(void);
static void vpSetting(void);
static void vrSetting(void);

#if defined(SMI_K2)
static void icfpSetting(void);
static void vpWfdSetting(void);
#endif

static void smi_dumpLarb(unsigned int index);
static void smi_dumpCommon(void);
#if IS_ENABLED(CONFIG_COMPAT)
static long MTK_SMI_COMPAT_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
#define MTK_SMI_COMPAT_ioctl  NULL
#endif



/* Use this function to get base address of Larb resgister */
/* to support error checking */
unsigned long get_larb_base_addr(int larb_id)
{
	if (larb_id >= SMI_LARB_NR || larb_id < 0)
		return SMI_ERROR_ADDR;
	else
		return gLarbBaseAddr[larb_id];

}

int larb_clock_on(int larb_id)
{

#ifndef CONFIG_MTK_FPGA
	char name[30];
	sprintf(name, "smi+%d", larb_id);

	switch (larb_id) {
	case 0:
		enable_clock(MT_CG_DISP0_SMI_COMMON, name);
		enable_clock(MT_CG_DISP0_SMI_LARB0, name);
		break;
	case 1:
		enable_clock(MT_CG_DISP0_SMI_COMMON, name);
		enable_clock(MT_CG_VDEC1_LARB, name);
		break;
	case 2:
		enable_clock(MT_CG_DISP0_SMI_COMMON, name);
		enable_clock(MT_CG_IMAGE_LARB2_SMI, name);
		break;
	case 3:
		enable_clock(MT_CG_DISP0_SMI_COMMON, name);
		enable_clock(MT_CG_VENC_LARB, name);
		break;
	case 4:
		enable_clock(MT_CG_DISP0_SMI_COMMON, name);
		enable_clock(MT_CG_MJC_SMI_LARB, name);
		break;
	default:
		break;
	}
#endif				/* CONFIG_MTK_FPGA */

	return 0;
}

int larb_clock_off(int larb_id)
{

#ifndef CONFIG_MTK_FPGA
	char name[30];
	sprintf(name, "smi+%d", larb_id);

	switch (larb_id) {
	case 0:
		disable_clock(MT_CG_DISP0_SMI_LARB0, name);
		disable_clock(MT_CG_DISP0_SMI_COMMON, name);
		break;
	case 1:
		disable_clock(MT_CG_VDEC1_LARB, name);
		disable_clock(MT_CG_DISP0_SMI_COMMON, name);
		break;
	case 2:
		disable_clock(MT_CG_IMAGE_LARB2_SMI, name);
		disable_clock(MT_CG_DISP0_SMI_COMMON, name);
		break;
	case 3:
		disable_clock(MT_CG_VENC_LARB, name);
		disable_clock(MT_CG_DISP0_SMI_COMMON, name);
		break;
	case 4:
		disable_clock(MT_CG_MJC_SMI_LARB, name);
		disable_clock(MT_CG_DISP0_SMI_COMMON, name);
		break;
	default:
		break;
	}
#endif				/* CONFIG_MTK_FPGA */

	return 0;
}

static void backup_smi_common(void)
{
	int i;

	for (i = 0; i < SMI_COMMON_BACKUP_REG_NUM; i++) {
		g_smi_common_backup[i] = M4U_ReadReg32(SMI_COMMON_EXT_BASE,
						       (unsigned long)
						       g_smi_common_backup_reg_offset[i]);
	}
}

static void restore_smi_common(void)
{
	int i;

	for (i = 0; i < SMI_COMMON_BACKUP_REG_NUM; i++) {
		M4U_WriteReg32(SMI_COMMON_EXT_BASE,
			       (unsigned long)g_smi_common_backup_reg_offset[i],
			       g_smi_common_backup[i]);
	}
}


static void backup_larb_smi(int index)
{
	int port_index = 0;
	unsigned short int *backup_ptr = NULL;
	unsigned long larb_base = gLarbBaseAddr[index];
	unsigned long larb_offset = 0x200;
	int total_port_num = 0;

	/* boundary check for larb_port_num and larb_port_backup access */
	if (index < 0 || index >= SMI_LARB_NR)
		return;


	total_port_num = larb_port_num[index];
	backup_ptr = larb_port_backup[index];

	/* boundary check for port value access */
	if (total_port_num <= 0 || backup_ptr == NULL)
		return;


	for (port_index = 0; port_index < total_port_num; port_index++) {
		*backup_ptr = (unsigned short int)(M4U_ReadReg32(larb_base, larb_offset));
		backup_ptr++;
		larb_offset += 4;
	}

	/* backup smi common along with larb0, smi common clk is guaranteed to be on when processing larbs */
	if (index == 0)
		backup_smi_common();


	return;
}

static void restore_larb_smi(int index)
{
	int port_index = 0;
	unsigned short int *backup_ptr = NULL;
	unsigned long larb_base = gLarbBaseAddr[index];
	unsigned long larb_offset = 0x200;
	unsigned int backup_value = 0;
	int total_port_num = 0;

	/* boundary check for larb_port_num and larb_port_backup access */
	if (index < 0 || index >= SMI_LARB_NR)
		return;

	total_port_num = larb_port_num[index];
	backup_ptr = larb_port_backup[index];

	/* boundary check for port value access */
	if (total_port_num <= 0 || backup_ptr == NULL)
		return;

	/* restore smi common along with larb0, smi common clk is guaranteed to be on when processing larbs */
	if (index == 0)
		restore_smi_common();

	for (port_index = 0; port_index < total_port_num; port_index++) {
		backup_value = *backup_ptr;
		M4U_WriteReg32(larb_base, larb_offset, backup_value);
		backup_ptr++;
		larb_offset += 4;
	}

	/* we do not backup 0x20 because it is a fixed setting */
	M4U_WriteReg32(larb_base, 0x20, larb_vc_setting[index]);

	/* turn off EMI empty OSTD dobule, fixed setting */
	M4U_WriteReg32(larb_base, 0x2c, 4);

	return;
}


static int larb_reg_backup(int larb)
{
	unsigned int *pReg = pLarbRegBackUp[larb];
	unsigned long larb_base = gLarbBaseAddr[larb];

	*(pReg++) = M4U_ReadReg32(larb_base, SMI_LARB_CON);


	backup_larb_smi(larb);

	if (0 == larb)
		g_bInited = 0;

	return 0;
}

static int smi_larb_init(unsigned int larb)
{
	unsigned int regval = 0;
	unsigned int regval1 = 0;
	unsigned int regval2 = 0;
	unsigned long larb_base = get_larb_base_addr(larb);

	/* Clock manager enable LARB clock before call back restore already*/
	/* it will be disabled after restore call back returns */
	/* Got to enable OSTD before engine starts */
	regval = M4U_ReadReg32(larb_base, SMI_LARB_STAT);


	if (0 == regval) {
		SMIDBG(1, "Init OSTD for larb_base: 0x%lx\n", larb_base);
		M4U_WriteReg32(larb_base, SMI_LARB_OSTDL_SOFT_EN, 0xffffffff);
	} else {
		SMIMSG("Larb: 0x%lx is busy : 0x%x , port:0x%x,0x%x ,fail to set OSTD\n",
		       larb_base, regval, regval1, regval2);
		/* smi_dumpDebugMsg(); */
		if (smi_debug_level >= 1) {
			SMIERR("DISP_MDP LARB  0x%lx OSTD cannot be set:0x%x,port:0x%x,0x%x\n",
			       larb_base, regval, regval1, regval2);
		} else {
			dump_stack();
		}
	}

	restore_larb_smi(larb);

	return 0;
}

int larb_reg_restore(int larb)
{
	unsigned long larb_base = SMI_ERROR_ADDR;
	unsigned int regval = 0;
	unsigned int *pReg = NULL;

	larb_base = get_larb_base_addr(larb);

	/* The larb assign doesn't exist */
	if (larb_base == SMI_ERROR_ADDR) {
		SMIMSG("Can't find the base address for Larb%d\n", larb);
		return 0;
	}

	if (larb >= SMI_LARB_NR || larb < 0) {
		SMIMSG("Can't find the backup register value for Larb%d\n", larb);
		return 0;
	}

	pReg = pLarbRegBackUp[larb];

	SMIDBG(1, "+larb_reg_restore(), larb_idx=%d\n", larb);
	SMIDBG(1, "m4u part restore, larb_idx=%d\n", larb);
	/* warning: larb_con is controlled by set/clr */
	regval = *(pReg++);
	M4U_WriteReg32(larb_base, SMI_LARB_CON_CLR, ~(regval));
	M4U_WriteReg32(larb_base, SMI_LARB_CON_SET, (regval));


	smi_larb_init(larb);

	return 0;
}

/* callback after larb clock is enabled */
void on_larb_power_on(struct larb_monitor *h, int larb_idx)
{
	/* M4ULOG("on_larb_power_on(), larb_idx=%d\n", larb_idx); */
	larb_reg_restore(larb_idx);

	return;
}

/* callback before larb clock is disabled */
void on_larb_power_off(struct larb_monitor *h, int larb_idx)
{
	/* M4ULOG("on_larb_power_off(), larb_idx=%d\n", larb_idx); */
	larb_reg_backup(larb_idx);
}

static void restSetting(void)
{
	/* initialize OSTD to 1 */
	M4U_WriteReg32(LARB0_BASE, 0x200, 0x1);	/* disp_ovl0 */
	M4U_WriteReg32(LARB0_BASE, 0x204, 0x1);	/* disp_rdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x208, 0x1);	/* disp_wdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x20c, 0x1);	/* disp_ovl1 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 0x1);	/* disp_rdma1 */
	M4U_WriteReg32(LARB0_BASE, 0x214, 0x1);	/* disp_wdma1 */

	/* To be chech with DE since the offset seems to be worng for MDP rdma, wdma mad rot */
	M4U_WriteReg32(LARB0_BASE, 0x238, 0x1);	/* mdp_rdma */
	M4U_WriteReg32(LARB0_BASE, 0x23c, 0x1);	/* mdp_wdma */
	M4U_WriteReg32(LARB0_BASE, 0x240, 0x1);	/* mdp_wrot */


	M4U_WriteReg32(LARB1_BASE, 0x200, 0x1);	/* hw_vdec_mc_ext */
	M4U_WriteReg32(LARB1_BASE, 0x204, 0x1);	/* hw_vdec_pp_ext */
	M4U_WriteReg32(LARB1_BASE, 0x208, 0x1);	/* hw_vdec_vld_ext */
	M4U_WriteReg32(LARB1_BASE, 0x20c, 0x1);	/* hw_vdec_avc_mv_ext */
	M4U_WriteReg32(LARB1_BASE, 0x210, 0x1);	/* hw_vdec_pred_rd_ext */
	M4U_WriteReg32(LARB1_BASE, 0x214, 0x1);	/* hw_vdec_pred_wr_ext */
	M4U_WriteReg32(LARB1_BASE, 0x218, 0x1);	/* hw_vdec_ppwrap_ext */

	M4U_WriteReg32(LARB2_BASE, 0x200, 0x1);	/* imgo */
	M4U_WriteReg32(LARB2_BASE, 0x204, 0x1);	/* rrzo */
	M4U_WriteReg32(LARB2_BASE, 0x208, 0x1);	/* aao */
	M4U_WriteReg32(LARB2_BASE, 0x20c, 0x1);	/* esfko */
	M4U_WriteReg32(LARB2_BASE, 0x210, 0x1);	/* imgo_s */
	M4U_WriteReg32(LARB2_BASE, 0x214, 0x1);	/* lsci */
	M4U_WriteReg32(LARB2_BASE, 0x218, 0x1);	/* lsci_d */
	M4U_WriteReg32(LARB2_BASE, 0x21c, 0x1);	/* bpci */
	M4U_WriteReg32(LARB2_BASE, 0x220, 0x1);	/* bpci_d */
	M4U_WriteReg32(LARB2_BASE, 0x224, 0x1);	/* ufdi */
	M4U_WriteReg32(LARB2_BASE, 0x228, 0x1);	/* imgi */
	M4U_WriteReg32(LARB2_BASE, 0x22c, 0x1);	/* img2o */
	M4U_WriteReg32(LARB2_BASE, 0x230, 0x1);	/* img3o */
	M4U_WriteReg32(LARB2_BASE, 0x234, 0x1);	/* vipi */
	M4U_WriteReg32(LARB2_BASE, 0x238, 0x1);	/* vip2i */
	M4U_WriteReg32(LARB2_BASE, 0x23c, 0x1);	/* vip3i */
	M4U_WriteReg32(LARB2_BASE, 0x240, 0x1);	/* lcei */
	M4U_WriteReg32(LARB2_BASE, 0x244, 0x1);	/* rb */
	M4U_WriteReg32(LARB2_BASE, 0x248, 0x1);	/* rp */
	M4U_WriteReg32(LARB2_BASE, 0x24c, 0x1);	/* wr */


	M4U_WriteReg32(LARB3_BASE, 0x200, 0x1);	/* venc_rcpu */
	M4U_WriteReg32(LARB3_BASE, 0x204, 0x2);	/* venc_rec */
	M4U_WriteReg32(LARB3_BASE, 0x208, 0x1);	/* venc_bsdma */
	M4U_WriteReg32(LARB3_BASE, 0x20c, 0x1);	/* venc_sv_comv */
	M4U_WriteReg32(LARB3_BASE, 0x210, 0x1);	/* venc_rd_comv */
	M4U_WriteReg32(LARB3_BASE, 0x214, 0x1);	/* jpgenc_rdma */
	M4U_WriteReg32(LARB3_BASE, 0x218, 0x1);	/* jpgenc_bsdma */
	M4U_WriteReg32(LARB3_BASE, 0x21c, 0x1);	/* jpgdec_wdma */
	M4U_WriteReg32(LARB3_BASE, 0x220, 0x1);	/* jpgdec_bsdma */
	M4U_WriteReg32(LARB3_BASE, 0x224, 0x1);	/* venc_cur_luma */
	M4U_WriteReg32(LARB3_BASE, 0x228, 0x1);	/* venc_cur_chroma */
	M4U_WriteReg32(LARB3_BASE, 0x22c, 0x1);	/* venc_ref_luma */
	M4U_WriteReg32(LARB3_BASE, 0x230, 0x1);	/* venc_ref_chroma */


	M4U_WriteReg32(LARB4_BASE, 0x200, 0x1);	/* mjc_mv_rd */
	M4U_WriteReg32(LARB4_BASE, 0x204, 0x1);	/* mjc_mv_wr */
	M4U_WriteReg32(LARB4_BASE, 0x208, 0x1);	/* mjc_dma_rd */
	M4U_WriteReg32(LARB4_BASE, 0x20c, 0x1);	/* mjc_dma_wr */
}

/* Make sure clock is on */
static void initSetting(void)
{
	/* save default larb regs */
	if (!is_default_value_saved) {
		SMIMSG("Save default config:\n");
		default_val_smi_l1arb[0] = M4U_ReadReg32(SMI_COMMON_EXT_BASE,
							 REG_OFFSET_SMI_L1ARB0);
		default_val_smi_l1arb[1] = M4U_ReadReg32(SMI_COMMON_EXT_BASE,
							 REG_OFFSET_SMI_L1ARB1);
		default_val_smi_l1arb[2] = M4U_ReadReg32(SMI_COMMON_EXT_BASE,
							 REG_OFFSET_SMI_L1ARB2);
		default_val_smi_l1arb[3] = M4U_ReadReg32(SMI_COMMON_EXT_BASE,
							 REG_OFFSET_SMI_L1ARB3);
		default_val_smi_l1arb[4] = M4U_ReadReg32(SMI_COMMON_EXT_BASE,
							 REG_OFFSET_SMI_L1ARB4);
		SMIMSG("l1arb[0-2]= 0x%x,  0x%x, 0x%x\n", default_val_smi_l1arb[0],
		       default_val_smi_l1arb[1], default_val_smi_l1arb[2]);
		SMIMSG("l1arb[3-4]= 0x%x,  0x%x\n", default_val_smi_l1arb[3],
		       default_val_smi_l1arb[4]);

		is_default_value_saved = 1;
	}
	/* Keep the HW's init setting in REG_SMI_L1ARB0 ~ REG_SMI_L1ARB4 */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB0, default_val_smi_l1arb[0]);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB1, default_val_smi_l1arb[1]);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB2, default_val_smi_l1arb[2]);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB3, default_val_smi_l1arb[3]);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB4, default_val_smi_l1arb[4]);
#if defined(SMI_K2)
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x200, 0x1b);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE,
		       0x234, (0x1 << 31) + (0x1d << 26) + (0x1f << 21) + (0x0 << 20) + (0x3 << 15)
		       + (0x4 << 10) + (0x4 << 5) + 0x5);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x230, 0x1f + (0x8 << 5) + (0x7 << 10));
#elif defined(SMI_RO)
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x200, 0x1b);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x220, 0x1);	/* disp: emi0, other:emi1 */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x234,
		       (0x1 << 31) + (0x13 << 26) + (0x14 << 21) + (0x0 << 20) + (0x2 << 15) +
		       (0x3 << 10) + (0x4 << 5) + 0x5);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x238,
		       (0x2 << 25) + (0x3 << 20) + (0x4 << 15) + (0x5 << 10) + (0x6 << 5) + 0x8);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x230, 0x1f + (0x8 << 5) + (0x6 << 10));
#endif
	/* Set VC priority: MMSYS = ISP > VENC > VDEC = MJC */
	M4U_WriteReg32(LARB0_BASE, 0x20, 0x0);	/* MMSYS */
	M4U_WriteReg32(LARB1_BASE, 0x20, 0x2);	/* VDEC */
	M4U_WriteReg32(LARB2_BASE, 0x20, 0x0);	/* ISP */
	M4U_WriteReg32(LARB3_BASE, 0x20, 0x1);	/* VENC */
	M4U_WriteReg32(LARB4_BASE, 0x20, 0x2);	/* MJC */

	/* turn off EMI empty double OSTD */
	M4U_WriteReg32(LARB0_BASE, 0x2c, M4U_ReadReg32(LARB0_BASE, 0x2c) | (1 << 2));
	M4U_WriteReg32(LARB1_BASE, 0x2c, M4U_ReadReg32(LARB1_BASE, 0x2c) | (1 << 2));
	M4U_WriteReg32(LARB2_BASE, 0x2c, M4U_ReadReg32(LARB2_BASE, 0x2c) | (1 << 2));
	M4U_WriteReg32(LARB3_BASE, 0x2c, M4U_ReadReg32(LARB3_BASE, 0x2c) | (1 << 2));
	M4U_WriteReg32(LARB4_BASE, 0x2c, M4U_ReadReg32(LARB4_BASE, 0x2c) | (1 << 2));
#if defined(SMI_K2)
	/* for ISP HRT */
	M4U_WriteReg32(LARB2_BASE, 0x24, (M4U_ReadReg32(LARB2_BASE, 0x24) & 0xf7ffffff));

	/* for UI */
	restSetting();

	/* SMI common BW limiter */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x204, 0x1A5A);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x208, 0x1000);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x20C, 0x1000);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x210, 0x1000);
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x214, 0x1000);

	/* LARB 0 DISP+MDP */
	M4U_WriteReg32(LARB0_BASE, 0x200, 31);	/* disp_ovl0 */
	M4U_WriteReg32(LARB0_BASE, 0x204, 4);	/* disp_rdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x208, 6);	/* disp_wdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x20c, 31);	/* disp_ovl1 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 4);	/* disp_rdma1 */
	M4U_WriteReg32(LARB0_BASE, 0x214, 0x1);	/* disp_wdma1 */

	/* To be chech with DE since the offset seems to be worng for MDP rdma, wdma mad rot */
	M4U_WriteReg32(LARB0_BASE, 0x238, 2);	/* mdp_rdma */
	M4U_WriteReg32(LARB0_BASE, 0x23c, 0x1);	/* mdp_wdma */
	M4U_WriteReg32(LARB0_BASE, 0x240, 3);	/* mdp_wrot */

#endif
}

static void icfpSetting(void)
{
	/* Use VSS's setting before we get the ICFP simulation result */
	/* SMI BW limit */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB0, 0x129F);	/* LARB0, DISP+MDP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB1, 0x1000);	/* LARB1, VDEC */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB2, 0x1224);	/* LARB2, ISP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB3, 0x1112);	/* LARB3, VENC+JPG */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB4, 0x1000);	/* LARB4, MJC */

	/* SMI LARB config */

	restSetting();

	/* LARB 0 DISP+MDP */
	M4U_WriteReg32(LARB0_BASE, 0x200, 0x1F);	/* port 0: disp ovl0 */
	M4U_WriteReg32(LARB0_BASE, 0x204, 4);	/* disp_rdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x208, 6);	/* disp_wdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x20C, 0x1F);	/* port 3: disp ovl1 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 4);	/* disp_rdma1 */
	M4U_WriteReg32(LARB0_BASE, 0x214, 0x1);	/* disp_wdma1 */

	M4U_WriteReg32(LARB0_BASE, 0x238, 2);	/* mdp_rdma */
	M4U_WriteReg32(LARB0_BASE, 0x23c, 0x1);	/* mdp_wdma */
	M4U_WriteReg32(LARB0_BASE, 0x240, 3);	/* mdp_wrot */

	M4U_WriteReg32(LARB2_BASE, 0x200, 0xA);	/* port#0, imgo */
	M4U_WriteReg32(LARB2_BASE, 0x204, 0x4);	/* port#1, rrzo */
	M4U_WriteReg32(LARB2_BASE, 0x228, 0x2);	/* port#10, */

}

static void vrSetting(void)
{
#if defined(SMI_K2)
	/* SMI BW limit */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB0, 0x129F);	/* LARB0, DISP+MDP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB1, 0x1000);	/* LARB1, VDEC */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB2, 0x1224);	/* LARB2, ISP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB3, 0x1112);	/* LARB3, VENC+JPG */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB4, 0x1000);	/* LARB4, MJC */

	/* SMI LARB config */

	restSetting();

	/* LARB 0 DISP+MDP */
	M4U_WriteReg32(LARB0_BASE, 0x200, 0x1F);	/* port 0: disp ovl0 */
	M4U_WriteReg32(LARB0_BASE, 0x204, 4);	/* disp_rdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x208, 6);	/* disp_wdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x20C, 0x1F);	/* port 3: disp ovl1 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 4);	/* disp_rdma1 */
	M4U_WriteReg32(LARB0_BASE, 0x214, 0x1);	/* disp_wdma1 */

	M4U_WriteReg32(LARB0_BASE, 0x238, 2);	/* mdp_rdma */
	M4U_WriteReg32(LARB0_BASE, 0x23c, 0x1);	/* mdp_wdma */
	M4U_WriteReg32(LARB0_BASE, 0x240, 3);	/* mdp_wrot */

	M4U_WriteReg32(LARB2_BASE, 0x200, 0xA);	/* port#0, imgo */
	M4U_WriteReg32(LARB2_BASE, 0x204, 0x4);	/* port#1, rrzo */
	M4U_WriteReg32(LARB2_BASE, 0x228, 0x2);	/* port#10, */
#elif defined(SMI_RO)
	/* VR 4K */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB0, 0x1614);	/* LARB0, DISP+MDP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB1, 0x1000);	/* LARB1, VDEC */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB2, 0x11F7);	/* LARB2, ISP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB3, 0x1584);	/* LARB3, VENC+JPG */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB4, 0x1000);	/* LARB4, MJC */

	M4U_WriteReg32(LARB0_BASE, 0x200, 0x1f);	/* ovl_ch0_0+ovl_ch0_1 */
	M4U_WriteReg32(LARB0_BASE, 0x224, 0x2);	/* mdp_wdma */
	M4U_WriteReg32(LARB0_BASE, 0x228, 0x5);	/* mdp_wrot0; min(9,5) */
	M4U_WriteReg32(LARB2_BASE, 0x200, 0x8);	/* imgo */
	M4U_WriteReg32(LARB2_BASE, 0x208, 0x1);	/* aao */
	M4U_WriteReg32(LARB2_BASE, 0x20c, 0x1);	/* lsco */
	M4U_WriteReg32(LARB2_BASE, 0x210, 0x1);	/* esfko */
	M4U_WriteReg32(LARB2_BASE, 0x218, 0x1);	/* lsci */
	M4U_WriteReg32(LARB2_BASE, 0x220, 0x1);	/* bpci */
	M4U_WriteReg32(LARB2_BASE, 0x22c, 0x4);	/* imgi */
	M4U_WriteReg32(LARB2_BASE, 0x230, 0x1);	/* img2o */
	M4U_WriteReg32(LARB2_BASE, 0x244, 0x1);	/* lcei */
	M4U_WriteReg32(LARB3_BASE, 0x200, 0x1);	/* venc_rcpu */
	M4U_WriteReg32(LARB3_BASE, 0x204, 0x4);	/* venc_rec_frm */
	M4U_WriteReg32(LARB3_BASE, 0x208, 0x1);	/* venc_bsdma */
	M4U_WriteReg32(LARB3_BASE, 0x20c, 0x1);	/* venc_sv_comv */
	M4U_WriteReg32(LARB3_BASE, 0x210, 0x1);	/* venc_rd_comv */
	M4U_WriteReg32(LARB3_BASE, 0x230, 0x8);	/* venc_cur_luma */
	M4U_WriteReg32(LARB3_BASE, 0x234, 0x4);	/* venc_cur_chroma */
	M4U_WriteReg32(LARB3_BASE, 0x23c, 0x10);	/* venc_ref_chroma */

	/* MJC */
	M4U_WriteReg32(LARB4_BASE, 0x200, 0x1);	/* MJC_MVR */
	M4U_WriteReg32(LARB4_BASE, 0x204, 0x1);	/* MJC_MVW */
	M4U_WriteReg32(LARB4_BASE, 0x208, 0x1c);	/* MJC_RDMA */
	M4U_WriteReg32(LARB4_BASE, 0x20c, 0xa);	/* MJC_WDMA */

	/* VP concurrent settings */
	/* LARB0 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 0x8);	/* port 4:ovl_ch1_0/1 */
	M4U_WriteReg32(LARB0_BASE, 0x21C, 0x4);	/* port 7:mdp_rdma0; min(4,5) */
	M4U_WriteReg32(LARB0_BASE, 0x22C, 0x3);	/* port11:mdp_wrot1 */

	/* VDEC */
	M4U_WriteReg32(LARB1_BASE, 0x200, 0x1f);	/* port#0, mc */
	M4U_WriteReg32(LARB1_BASE, 0x204, 0x06);	/* port#1, pp */
	M4U_WriteReg32(LARB1_BASE, 0x208, 0x1);	/* port#2, ufo */
	M4U_WriteReg32(LARB1_BASE, 0x20c, 0x1);	/* port#3, vld */
	M4U_WriteReg32(LARB1_BASE, 0x210, 0x1);	/* port#4, vld2 */
	M4U_WriteReg32(LARB1_BASE, 0x214, 0x2);	/* port#5, mv */
	M4U_WriteReg32(LARB1_BASE, 0x218, 0x1);	/* port#6, pred rd */
	M4U_WriteReg32(LARB1_BASE, 0x21c, 0x1);	/* port#7, pred wr */
	M4U_WriteReg32(LARB1_BASE, 0x220, 0x1);	/* port#8, ppwrap */
#endif

}

static void vpSetting(void)
{
#if defined(SMI_K2)
	/* VP 4K */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x204, 0x13DB);	/* LARB0, DISP+MDP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x208, 0x117D);	/* LARB1, VDEC */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x20C, 0x1000);	/* LARB2, ISP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x210, 0x10AD);	/* LARB3, VENC+JPG */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x214, 0x1000);	/* LARB4, MJC */

	restSetting();

	M4U_WriteReg32(LARB0_BASE, 0x200, 0x12);	/* OVL_CH0_0+OVL_CH0_1 */
	M4U_WriteReg32(LARB0_BASE, 0x204, 4);	/* disp_rdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x208, 6);	/* disp_wdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x20C, 0x12);	/* port 3: disp ovl1 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 4);	/* OVL_CH1_0+OVL_CH1_1 */
	M4U_WriteReg32(LARB0_BASE, 0x238, 2);	/* mdp_rdma */
	M4U_WriteReg32(LARB0_BASE, 0x23c, 0x2);	/* mdp_wdma */
	M4U_WriteReg32(LARB0_BASE, 0x240, 0x3);	/* mdp_wrot */

	M4U_WriteReg32(LARB1_BASE, 0x200, 0x8);	/* port#0, mc */
	M4U_WriteReg32(LARB1_BASE, 0x204, 0x2);	/* port#1, pp */
	M4U_WriteReg32(LARB1_BASE, 0x208, 0x1);	/* port#2, ufo */
	M4U_WriteReg32(LARB1_BASE, 0x20c, 0x1);	/* port#3, vld */
	M4U_WriteReg32(LARB1_BASE, 0x210, 0x1);	/* port#4, vld2 */
	M4U_WriteReg32(LARB1_BASE, 0x214, 0x1);	/* port#5, mv */
	M4U_WriteReg32(LARB1_BASE, 0x218, 0x1);	/* port#6, pred rd */
	M4U_WriteReg32(LARB1_BASE, 0x21c, 0x1);	/* port#7, pred wr */
	M4U_WriteReg32(LARB1_BASE, 0x220, 0x1);	/* port#8, ppwrap */
#elif defined(SMI_RO)
	/* VP 4K */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB0, 0x17C0);	/* LARB0, DISP+MDP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB1, 0x161B);	/* LARB1, VDEC */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB2, 0x1000);	/* LARB2, ISP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB3, 0x1000);	/* LARB3, VENC+JPG */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, REG_OFFSET_SMI_L1ARB4, 0x1000);	/* LARB4, MJC */

	M4U_WriteReg32(LARB0_BASE, 0x200, 0x18);	/* port 0:ovl_ch0_0/1 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 0x8);	/* port 4:ovl_ch1_0/1 */
	M4U_WriteReg32(LARB0_BASE, 0x21C, 0x4);	/* port 7:mdp_rdma0; min(4,5) */
	M4U_WriteReg32(LARB0_BASE, 0x220, 0x4);	/* port 8:mdp_rdma1; min(4,5) */
	M4U_WriteReg32(LARB0_BASE, 0x228, 0x5);	/* port10:mdp_wrot0 */
	M4U_WriteReg32(LARB0_BASE, 0x22C, 0x3);	/* port11:mdp_wrot1 */

	M4U_WriteReg32(LARB1_BASE, 0x200, 0x1f);	/* port#0, mc */
	M4U_WriteReg32(LARB1_BASE, 0x204, 0x06);	/* port#1, pp */
	M4U_WriteReg32(LARB1_BASE, 0x208, 0x1);	/* port#2, ufo */
	M4U_WriteReg32(LARB1_BASE, 0x20c, 0x1);	/* port#3, vld */
	M4U_WriteReg32(LARB1_BASE, 0x210, 0x1);	/* port#4, vld2 */
	M4U_WriteReg32(LARB1_BASE, 0x214, 0x2);	/* port#5, mv */
	M4U_WriteReg32(LARB1_BASE, 0x218, 0x1);	/* port#6, pred rd */
	M4U_WriteReg32(LARB1_BASE, 0x21c, 0x1);	/* port#7, pred wr */
	M4U_WriteReg32(LARB1_BASE, 0x220, 0x1);	/* port#8, ppwrap */

	/* MJC */
	M4U_WriteReg32(LARB4_BASE, 0x200, 0x1);	/* MJC_MVR */
	M4U_WriteReg32(LARB4_BASE, 0x204, 0x1);	/* MJC_MVW */
	M4U_WriteReg32(LARB4_BASE, 0x208, 0x1c);	/* MJC_RDMA */
	M4U_WriteReg32(LARB4_BASE, 0x20c, 0xa);	/* MJC_WDMA */
#endif
}

static void vpWfdSetting(void)
{
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x204, 0x13DB);	/* LARB0, DISP+MDP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x208, 0x117D);	/* LARB1, VDEC */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x20C, 0x1000);	/* LARB2, ISP */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x210, 0x10AD);	/* LARB3, VENC+JPG */
	M4U_WriteReg32(SMI_COMMON_EXT_BASE, 0x214, 0x1000);	/* LARB4, MJC */

	restSetting();

	M4U_WriteReg32(LARB0_BASE, 0x200, 0x12);	/* OVL_CH0_0+OVL_CH0_1 */
	M4U_WriteReg32(LARB0_BASE, 0x204, 4);	/* disp_rdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x208, 6);	/* disp_wdma0 */
	M4U_WriteReg32(LARB0_BASE, 0x20C, 0x12);	/* port 3: disp ovl1 */
	M4U_WriteReg32(LARB0_BASE, 0x210, 0x4);	/* port 3: disp ovl1 */
	M4U_WriteReg32(LARB0_BASE, 0x238, 2);	/* mdp_rdma */
	M4U_WriteReg32(LARB0_BASE, 0x23c, 0x2);	/* OVL_CH1_0+OVL_CH1_1 */
	M4U_WriteReg32(LARB0_BASE, 0x240, 0x3);	/* mdp_wrot */

	M4U_WriteReg32(LARB1_BASE, 0x200, 0x8);	/* port#0, mc */
	M4U_WriteReg32(LARB1_BASE, 0x204, 0x2);	/* port#1, pp */

}

/* Fake mode check, e.g. WFD */
static int fake_mode_handling(MTK_SMI_BWC_CONFIG *p_conf, unsigned int *pu4LocalCnt)
{
	if (p_conf->scenario == SMI_BWC_SCEN_WFD) {
		if (p_conf->b_on_off) {
			wifi_disp_transaction = 1;
			SMIMSG("Enable WFD in profile: %d\n", smi_profile);
		} else {
			wifi_disp_transaction = 0;
			SMIMSG("Disable WFD in profile: %d\n", smi_profile);
		}
		return 1;
	} else {
		return 0;
	}
}

static int ovl_limit_uevent(int bwc_scenario, int ovl_pixel_limit)
{
	int err = 0;
	char *envp[3];
	char scenario_buf[32] = "";
	char ovl_limit_buf[32] = "";


	snprintf(scenario_buf, 31, "SCEN=%d", bwc_scenario);
	snprintf(ovl_limit_buf, 31, "HWOVL=%d", ovl_pixel_limit);

	envp[0] = scenario_buf;
	envp[1] = ovl_limit_buf;
	envp[2] = NULL;

	if (pSmiDev != NULL) {

		err = kobject_uevent_env(&(smiDeviceUevent->kobj), KOBJ_CHANGE, envp);
		SMIMSG("Notify OVL limitaion=%d, SCEN=%d", ovl_pixel_limit, bwc_scenario);
	}


	if (err < 0)
		SMIMSG(KERN_INFO "[%s] kobject_uevent_env error = %d\n", __func__, err);

	return err;
}

static int smi_bwc_config(MTK_SMI_BWC_CONFIG *p_conf, unsigned int *pu4LocalCnt)
{
	int i;
	int result = 0;
	unsigned int u4Concurrency = 0;
	MTK_SMI_BWC_SCEN eFinalScen;
	static MTK_SMI_BWC_SCEN ePreviousFinalScen = SMI_BWC_SCEN_CNT;

	if (smi_tuning_mode == 1) {
		SMIMSG("Doesn't change profile in tunning mode");
		return 0;
	}

	if ((SMI_BWC_SCEN_CNT <= p_conf->scenario) || (0 > p_conf->scenario)) {
		SMIERR("Incorrect SMI BWC config : 0x%x, how could this be...\n", p_conf->scenario);
		return -1;
	}
#if defined(SMI_RO)
	if (p_conf->b_on_off) {
		/* set mmdvfs step according to certain scenarios */
		mmdvfs_notify_scenario_enter(p_conf->scenario);
	} else {
		/* set mmdvfs step to default after the scenario exits */
		mmdvfs_notify_scenario_exit(p_conf->scenario);
	}
#endif
	spin_lock(&g_SMIInfo.SMI_lock);
	result = fake_mode_handling(p_conf, pu4LocalCnt);
	spin_unlock(&g_SMIInfo.SMI_lock);

	/* Fake mode is not a real SMI profile, so we need to return here */
	if (result == 1)
		return 0;

	spin_lock(&g_SMIInfo.SMI_lock);

	if (p_conf->b_on_off) {
		/* turn on certain scenario */
		g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario] += 1;

		if (NULL != pu4LocalCnt)
			pu4LocalCnt[p_conf->scenario] += 1;

	} else {
		/* turn off certain scenario */
		if (0 == g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario]) {
			SMIMSG("Too many turning off for global SMI profile:%d,%d\n",
			       p_conf->scenario, g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario]);
		} else {
			g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario] -= 1;
		}

		if (NULL != pu4LocalCnt) {
			if (0 == pu4LocalCnt[p_conf->scenario]) {
				SMIMSG
				    ("Process : %s did too many turning off for local SMI profile:%d,%d\n",
				     current->comm, p_conf->scenario,
				     pu4LocalCnt[p_conf->scenario]);
			} else {
				pu4LocalCnt[p_conf->scenario] -= 1;
			}
		}
	}

	for (i = 0; i < SMI_BWC_SCEN_CNT; i++) {
		if (g_SMIInfo.pu4ConcurrencyTable[i])
			u4Concurrency |= (1 << i);
	}
#if defined(SMI_RO)
	/* notify mmdvfs concurrency */
	mmdvfs_notify_scenario_concurrency(u4Concurrency);
#endif
	if ((1 << SMI_BWC_SCEN_MM_GPU) & u4Concurrency)
		eFinalScen = SMI_BWC_SCEN_MM_GPU;
	else if ((1 << SMI_BWC_SCEN_ICFP) & u4Concurrency)
		eFinalScen = SMI_BWC_SCEN_ICFP;
	else if ((1 << SMI_BWC_SCEN_VR_SLOW) & u4Concurrency)
		eFinalScen = SMI_BWC_SCEN_VR_SLOW;
	else if ((1 << SMI_BWC_SCEN_VR) & u4Concurrency)
		eFinalScen = SMI_BWC_SCEN_VR;
	else if ((1 << SMI_BWC_SCEN_VP) & u4Concurrency)
		eFinalScen = SMI_BWC_SCEN_VP;
	else if ((1 << SMI_BWC_SCEN_SWDEC_VP) & u4Concurrency)
		eFinalScen = SMI_BWC_SCEN_SWDEC_VP;
	else if ((1 << SMI_BWC_SCEN_VENC) & u4Concurrency)
		eFinalScen = SMI_BWC_SCEN_VENC;
	else
		eFinalScen = SMI_BWC_SCEN_NORMAL;

	if (ePreviousFinalScen == eFinalScen) {
		SMIMSG("Scen equal%d,don't change\n", eFinalScen);
		spin_unlock(&g_SMIInfo.SMI_lock);
		return 0;
	} else {
		ePreviousFinalScen = eFinalScen;
	}

	/* turn on larb clock */
	for (i = 0; i < SMI_LARB_NR; i++)
		larb_clock_on(i);

	smi_profile = eFinalScen;

	/* Bandwidth Limiter */
	switch (eFinalScen) {
	case SMI_BWC_SCEN_VP:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_VP");
		if (!wifi_disp_transaction)
			vpSetting();
		else
			vpWfdSetting();
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_VP;
		break;

	case SMI_BWC_SCEN_SWDEC_VP:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_SCEN_SWDEC_VP");
		vpSetting();
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_VP;
		break;

	case SMI_BWC_SCEN_ICFP:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_SCEN_ICFP");
		icfpSetting();
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_VR;
		break;
	case SMI_BWC_SCEN_VR:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_VR");
		vrSetting();
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_VR;
		break;

	case SMI_BWC_SCEN_VR_SLOW:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_VR");
		smi_profile = SMI_BWC_SCEN_VR_SLOW;
		vrSetting();
#if defined(SMI_K2)
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_NORMAL;
#elif defined(SMI_RO)
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_VR;
#endif
		break;

	case SMI_BWC_SCEN_VENC:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_SCEN_VENC");
		vrSetting();
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_NORMAL;
		break;

	case SMI_BWC_SCEN_NORMAL:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_SCEN_NORMAL");
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_NORMAL;
		initSetting();
		break;

	case SMI_BWC_SCEN_MM_GPU:
		SMIMSG("[SMI_PROFILE] : %s\n", "SMI_BWC_SCEN_MM_GPU");
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_NORMAL;
		initSetting();
		break;

	default:
		SMIMSG("[SMI_PROFILE] : %s %d\n", "initSetting", eFinalScen);
		initSetting();
		g_smi_bwc_mm_info.hw_ovl_limit = SF_HWC_PIXEL_MAX_NORMAL;
		break;
	}

	/*turn off larb clock */
	for (i = 0; i < SMI_LARB_NR; i++)
		larb_clock_off(i);

	spin_unlock(&g_SMIInfo.SMI_lock);

	ovl_limit_uevent(smi_profile, g_smi_bwc_mm_info.hw_ovl_limit);


	/* force 30 fps in VR slow motion, because disp driver set fps apis got mutex */
	/* call these APIs only when necessary */
	{
		static unsigned int current_fps;

		if ((eFinalScen == SMI_BWC_SCEN_VR_SLOW) && (current_fps != 30)) {
			/* force 30 fps in VR slow motion profile */
			primary_display_force_set_vsync_fps(30);
			current_fps = 30;
			SMIMSG("[SMI_PROFILE] set 30 fps\n");
		} else if ((eFinalScen != SMI_BWC_SCEN_VR_SLOW) && (current_fps == 30)) {	/* back to normal fps */
			current_fps = primary_display_get_fps();
			primary_display_force_set_vsync_fps(current_fps);
			SMIMSG("[SMI_PROFILE] back to %u fps\n", current_fps);
		}
	}

	SMIMSG("SMI_PROFILE to:%d %s,cur:%d,%d,%d,%d\n", p_conf->scenario,
	       (p_conf->b_on_off ? "on" : "off"), eFinalScen,
	       g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_NORMAL],
	       g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_VR],
	       g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_VP]);

	return 0;
}

struct larb_monitor larb_monitor_handler = {
	.level = LARB_MONITOR_LEVEL_HIGH,
	.backup = on_larb_power_off,
	.restore = on_larb_power_on
};

int smi_common_init(void)
{
	int i;

	for (i = 0; i < SMI_LARB_NR; i++) {
		pLarbRegBackUp[i] = kmalloc(LARB_BACKUP_REG_SIZE,
							    GFP_KERNEL | __GFP_ZERO);
		if (pLarbRegBackUp[i] == NULL)
			SMIERR("pLarbRegBackUp kmalloc fail %d\n", i);
	}

	/*
	 * make sure all larb power is on before we register callback func.
	 * then, when larb power is first off, default register value will be backed up.
	 */

	for (i = 0; i < SMI_LARB_NR; i++)
		larb_clock_on(i);

	/* apply init setting after kernel boot */
	initSetting();

	register_larb_monitor(&larb_monitor_handler);

	for (i = 0; i < SMI_LARB_NR; i++)
		larb_clock_off(i);

	return 0;
}

static int smi_open(struct inode *inode, struct file *file)
{
	file->private_data = kmalloc(SMI_BWC_SCEN_CNT * sizeof(unsigned int), GFP_ATOMIC);

	if (NULL == file->private_data) {
		SMIMSG("Not enough entry for DDP open operation\n");
		return -ENOMEM;
	}

	memset(file->private_data, 0, SMI_BWC_SCEN_CNT * sizeof(unsigned int));

	return 0;
}

static int smi_release(struct inode *inode, struct file *file)
{

#if 0
	unsigned long u4Index = 0;
	unsigned long u4AssignCnt = 0;
	unsigned long *pu4Cnt = (unsigned long *)file->private_data;
	MTK_SMI_BWC_CONFIG config;

	for (; u4Index < SMI_BWC_SCEN_CNT; u4Index += 1) {
		if (pu4Cnt[u4Index]) {
			SMIMSG("Process:%s does not turn off BWC properly , force turn off %d\n",
			       current->comm, u4Index);
			u4AssignCnt = pu4Cnt[u4Index];
			config.b_on_off = 0;
			config.scenario = (MTK_SMI_BWC_SCEN) u4Index;
			do {
				smi_bwc_config(&config, pu4Cnt);
			} while (0 < u4AssignCnt);
		}
	}
#endif

	if (NULL != file->private_data) {
		kfree(file->private_data);
		file->private_data = NULL;
	}

	return 0;
}


static long smi_ioctl(struct file *pFile, unsigned int cmd, unsigned long param)
{
	int ret = 0;

	/* unsigned long * pu4Cnt = (unsigned long *)pFile->private_data; */

	switch (cmd) {

		/* disable reg access ioctl by default for possible security holes */
		/* TBD: check valid SMI register range */
	case MTK_IOC_SMI_BWC_CONFIG:{
			MTK_SMI_BWC_CONFIG cfg;
			ret = copy_from_user(&cfg, (void *)param, sizeof(MTK_SMI_BWC_CONFIG));
			if (ret) {
				SMIMSG(" SMI_BWC_CONFIG, copy_from_user failed: %d\n", ret);
				return -EFAULT;
			}

			ret = smi_bwc_config(&cfg, NULL);

			break;
		}
		/* GMP start */
	case MTK_IOC_SMI_BWC_INFO_SET:{
			smi_set_mm_info_ioctl_wrapper(pFile, cmd, param);
			break;
		}
	case MTK_IOC_SMI_BWC_INFO_GET:{
			smi_get_mm_info_ioctl_wrapper(pFile, cmd, param);
			break;
		}
		/* GMP end */

	case MTK_IOC_SMI_DUMP_LARB:{
			unsigned int larb_index;

			ret = copy_from_user(&larb_index, (void *)param, sizeof(unsigned int));
			if (ret)
				return -EFAULT;

			smi_dumpLarb(larb_index);
		}
		break;

	case MTK_IOC_SMI_DUMP_COMMON:{
			unsigned int arg;

			ret = copy_from_user(&arg, (void *)param, sizeof(unsigned int));
			if (ret)
				return -EFAULT;

			smi_dumpCommon();

		}
		break;
#if defined(SMI_RO)
	case MTK_IOC_MMDVFS_CMD:
		{
			MTK_MMDVFS_CMD mmdvfs_cmd;

			if (copy_from_user(&mmdvfs_cmd, (void *)param, sizeof(MTK_MMDVFS_CMD)))
				return -EFAULT;

			mmdvfs_handle_cmd(&mmdvfs_cmd);

			if (copy_to_user
			    ((void *)param, (void *)&mmdvfs_cmd, sizeof(MTK_MMDVFS_CMD))) {
				return -EFAULT;
			}

			break;
		}
#endif
	default:
		return -1;
	}

	return ret;
}

static const struct file_operations smiFops = {
	.owner = THIS_MODULE,
	.open = smi_open,
	.release = smi_release,
	.unlocked_ioctl = smi_ioctl,
	.compat_ioctl = MTK_SMI_COMPAT_ioctl,
};

#if defined(SMI_RO)
static int smi_sel;

static ssize_t smi_sel_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *p = buf;

	p += sprintf(p, "%d\n", smi_sel);

	return p - buf;
}

static ssize_t smi_sel_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EPERM;

	smi_sel = val;

	return count;
}

static ssize_t smi_dbg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (smi_sel >= 0 && smi_sel < SMI_LARB_NR)
		smi_dumpLarb(smi_sel);
	else if (smi_sel == 999)
		smi_dumpCommon();

	return 0;
}

DEFINE_ATTR_RW(smi_sel);
DEFINE_ATTR_RO(smi_dbg);

static struct attribute *smi_attrs[] = {
	__ATTR_OF(smi_sel),
	__ATTR_OF(smi_dbg),
	NULL,
};

static struct attribute_group smi_attr_group = {
	.name = "smi",
	.attrs = smi_attrs,
};
#endif
static dev_t smiDevNo = MKDEV(MTK_SMI_MAJOR_NUMBER, 0);
static inline int smi_register(void)
{
	if (alloc_chrdev_region(&smiDevNo, 0, 1, "MTK_SMI")) {
		SMIERR("Allocate device No. failed");
		return -EAGAIN;
	}
	/* Allocate driver */
	pSmiDev = cdev_alloc();

	if (NULL == pSmiDev) {
		unregister_chrdev_region(smiDevNo, 1);
		SMIERR("Allocate mem for kobject failed");
		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(pSmiDev, &smiFops);
	pSmiDev->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(pSmiDev, smiDevNo, 1)) {
		SMIERR("Attatch file operation failed");
		unregister_chrdev_region(smiDevNo, 1);
		return -EAGAIN;
	}

	return 0;
}

static struct class *pSmiClass;

#if !defined(SMI_DT_SUPPORT) && defined(SMI_RO)
static int smi_probe(struct platform_device *pdev)
{
	struct device *smiDevice = NULL;

	if (NULL == pdev) {
		SMIERR("platform data missed");
		return -ENXIO;
	}

	if (smi_register()) {
		dev_err(&pdev->dev, "register char failed\n");
		return -EAGAIN;
	}

	pSmiClass = class_create(THIS_MODULE, "MTK_SMI");
	if (IS_ERR(pSmiClass)) {
		int ret = PTR_ERR(pSmiClass);
		SMIERR("Unable to create class, err = %d", ret);
		return ret;
	}

	smiDevice = device_create(pSmiClass, NULL, smiDevNo, NULL, "MTK_SMI");
	smiDeviceUevent = smiDevice;

	/* Adapt the legacy codes */
	smi_reg_bases[0] = 0xf4022000;	/* common */
	smi_reg_bases[1] = 0xf4021000;	/* larb 0 */
	smi_reg_bases[2] = 0xf6010000;
	smi_reg_bases[3] = 0xf5001000;
	smi_reg_bases[4] = 0xf8001000;
	smi_reg_bases[5] = 0xf7002000;	/* larb 4 */

	gLarbBaseAddr[0] = LARB0_BASE;
	gLarbBaseAddr[1] = LARB1_BASE;
	gLarbBaseAddr[2] = LARB2_BASE;
	gLarbBaseAddr[3] = LARB3_BASE;
	gLarbBaseAddr[4] = LARB4_BASE;

	smi_common_init();

	return 0;
}
#endif
static int smi_probe(struct platform_device *pdev)
{

	int i;

	static unsigned int smi_probe_cnt;
	struct device *smiDevice = NULL;

	SMIMSG("probing smi...");

	if (smi_probe_cnt != 0) {
		SMIERR("Only support 1 SMI driver probed\n");
		return 0;
	}
	smi_probe_cnt++;

	smi_dev = kmalloc(sizeof(struct smi_device), GFP_KERNEL);

	if (smi_dev == NULL) {
		SMIERR("Unable to allocate memory for smi driver");
		return -ENOMEM;
	}
	if (NULL == pdev) {
		SMIERR("platform data missed");
		return -ENXIO;
	}
	/* Keep the device structure */
	smi_dev->dev = &pdev->dev;

	/* Map registers */
#if defined(SMI_K2)
	for (i = 0; i < SMI_REG_REGION_MAX; i++) {
#elif defined(SMI_RO)
	for (i = 0; i < SMI_REG_BASE_NUM; i++) {
#endif
		smi_dev->regs[i] = (void *)of_iomap(pdev->dev.of_node, i);

		if (!smi_dev->regs[i]) {
			SMIERR("Unable to ioremap registers, of_iomap fail, i=%d\n", i);
			return -ENOMEM;
		}
		/* Record the register base in global variable */
#if defined(SMI_K2)
		gSMIBaseAddrs[i] = (unsigned long)(smi_dev->regs[i]);
		SMIMSG("DT, i=%d, region=%s, map_addr=0x%p, reg_pa=0x%lx\n", i,
		       smi_get_region_name(i), smi_dev->regs[i], smi_reg_pa_base[i]);
#elif defined(SMI_RO)
		smi_reg_bases[i] = (unsigned long)(smi_dev->regs[i]);
		SMIMSG("DT, i=%d, map_addr=0x%p", i, smi_dev->regs[i]);
#endif
	}


	if (smi_register()) {
		dev_err(&pdev->dev, "register char failed\n");
		return -EAGAIN;
	}

	pSmiClass = class_create(THIS_MODULE, "MTK_SMI");
	if (IS_ERR(pSmiClass)) {
		int ret = PTR_ERR(pSmiClass);
		SMIERR("Unable to create class, err = %d", ret);
		return ret;
	}

	smiDevice = device_create(pSmiClass, NULL, smiDevNo, NULL, "MTK_SMI");
	smiDeviceUevent = smiDevice;

	SMIMSG("SMI probe done.\n");

	/* To adapt the legacy codes */
#if defined(SMI_K2)
	smi_reg_base_common_ext = gSMIBaseAddrs[SMI_COMMON_REG_INDX];
	smi_reg_base_barb0 = gSMIBaseAddrs[SMI_LARB0_REG_INDX];
	smi_reg_base_barb1 = gSMIBaseAddrs[SMI_LARB1_REG_INDX];
	smi_reg_base_barb2 = gSMIBaseAddrs[SMI_LARB2_REG_INDX];
	smi_reg_base_barb3 = gSMIBaseAddrs[SMI_LARB3_REG_INDX];
	smi_reg_base_barb4 = gSMIBaseAddrs[SMI_LARB4_REG_INDX];
#endif
	gLarbBaseAddr[0] = LARB0_BASE;
	gLarbBaseAddr[1] = LARB1_BASE;
	gLarbBaseAddr[2] = LARB2_BASE;
	gLarbBaseAddr[3] = LARB3_BASE;
	gLarbBaseAddr[4] = LARB4_BASE;

	smi_common_init();

#if defined(SMI_RO)
	if (smiDevice) {
		int ret;
		ret = sysfs_create_group(&smiDevice->kobj, &smi_attr_group);
		if (ret)
			SMIMSG("create sysfs error");
	}
#endif
	return 0;

}

#if defined(SMI_K2)
char *smi_get_region_name(unsigned int region_indx)
{
	switch (region_indx) {
	case SMI_COMMON_REG_INDX:
		return "smi_common";
	case SMI_LARB0_REG_INDX:
		return "larb0";
	case SMI_LARB1_REG_INDX:
		return "larb1";
	case SMI_LARB2_REG_INDX:
		return "larb2";
	case SMI_LARB3_REG_INDX:
		return "larb3";
	case SMI_LARB4_REG_INDX:
		return "larb4";
	default:
		SMIMSG("invalid region id=%d", region_indx);
		return "unknown";
	}
}

void register_base_dump(void)
{
	int i = 0;
	unsigned long pa_value = 0;
	unsigned long va_value = 0;

	for (i = 0; i < SMI_REG_REGION_MAX; i++) {
		va_value = gSMIBaseAddrs[i];
		pa_value = virt_to_phys((void *)va_value);
		SMIMSG("REG BASE:%s-->VA=0x%lx,PA=0x%lx,SPEC=0x%lx\n",
		       smi_get_region_name(i), va_value, pa_value, smi_reg_pa_base[i]);
	}
}
#endif
static int smi_remove(struct platform_device *pdev)
{
	cdev_del(pSmiDev);
	unregister_chrdev_region(smiDevNo, 1);
	device_destroy(pSmiClass, smiDevNo);
	class_destroy(pSmiClass);
	return 0;
}

static int smi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int smi_resume(struct platform_device *pdev)
{
	return 0;
}

#ifdef SMI_DT_SUPPORT
static const struct of_device_id smi_of_ids[] = {
	{.compatible = "mediatek,SMI_COMMON",},
	{}
};
#endif				/* SMI_DT_SUPPORT */
static struct platform_driver smiDrv = {
	.probe = smi_probe,
	.remove = smi_remove,
	.suspend = smi_suspend,
	.resume = smi_resume,
	.driver = {
		   .name = "MTK_SMI",
		   .owner = THIS_MODULE,
#ifdef SMI_DT_SUPPORT
		   .of_match_table = smi_of_ids,
#endif				/* SMI_DT_SUPPORT */
		   }
};

static int __init smi_init(void)
{
	spin_lock_init(&g_SMIInfo.SMI_lock);
#if defined(SMI_RO)
	/* MMDVFS init */
	mmdvfs_init(&g_smi_bwc_mm_info);
#endif

	memset(g_SMIInfo.pu4ConcurrencyTable, 0, SMI_BWC_SCEN_CNT * sizeof(unsigned int));

	/* Informs the kernel about the function to be called */
	/* if hardware matching MTK_SMI has been found */
	if (platform_driver_register(&smiDrv)) {
		SMIERR("failed to register MAU driver");
		return -ENODEV;
	}

	return 0;
}

static void __exit smi_exit(void)
{
	platform_driver_unregister(&smiDrv);

}

static void smi_dumpCommonDebugMsg(int output_gce_buffer)
{
	unsigned long u4Base;
	int smiCommonClkEnabled = 0;

	smiCommonClkEnabled = clock_is_on(MT_CG_DISP0_SMI_COMMON);
	/* SMI COMMON dump */
	if (smi_debug_level == 0 && (!smiCommonClkEnabled)) {
		SMIMSG3(output_gce_buffer, "===SMI common clock is disabled===\n");
		return;
	}

	SMIMSG3(output_gce_buffer, "===SMI common reg dump, CLK: %d===\n", smiCommonClkEnabled);

	u4Base = SMI_COMMON_EXT_BASE;
	SMIMSG3(output_gce_buffer, "[0x200,0x204,0x208]=[0x%x,0x%x,0x%x]\n",
		M4U_ReadReg32(u4Base, 0x200), M4U_ReadReg32(u4Base, 0x204),
		M4U_ReadReg32(u4Base, 0x208));
	SMIMSG3(output_gce_buffer, "[0x20C,0x210,0x214]=[0x%x,0x%x,0x%x]\n",
		M4U_ReadReg32(u4Base, 0x20C), M4U_ReadReg32(u4Base, 0x210),
		M4U_ReadReg32(u4Base, 0x214));
	SMIMSG3(output_gce_buffer, "[0x220,0x230,0x234,0x238]=[0x%x,0x%x,0x%x,0x%x]\n",
		M4U_ReadReg32(u4Base, 0x220), M4U_ReadReg32(u4Base, 0x230),
		M4U_ReadReg32(u4Base, 0x234), M4U_ReadReg32(u4Base, 0x238));
	SMIMSG3(output_gce_buffer, "[0x400,0x404,0x408]=[0x%x,0x%x,0x%x]\n",
		M4U_ReadReg32(u4Base, 0x400), M4U_ReadReg32(u4Base, 0x404),
		M4U_ReadReg32(u4Base, 0x408));

}

static int smi_larb_clock_is_on(unsigned int larb_index)
{

	int result = 0;

	switch (larb_index) {
	case 0:
		result = clock_is_on(MT_CG_DISP0_SMI_LARB0);
		break;
	case 1:
		result = clock_is_on(MT_CG_VDEC1_LARB);
		break;
	case 2:
		result = clock_is_on(MT_CG_IMAGE_LARB2_SMI);
		break;
	case 3:
		result = clock_is_on(MT_CG_VENC_LARB);
		break;
	case 4:
		result = clock_is_on(MT_CG_MJC_SMI_LARB);
		break;
	default:
		result = 0;
		break;
	}

	return result;

}

static void smi_dumpLarbDebugMsg(unsigned int u4Index)
{
	unsigned long u4Base = 0;

	int larbClkEnabled = 0;

	u4Base = get_larb_base_addr(u4Index);

	larbClkEnabled = smi_larb_clock_is_on(u4Index);

	if (u4Base == SMI_ERROR_ADDR) {
		SMIMSG("Doesn't support reg dump for Larb%d\n", u4Index);

		return;
	} else if ((larbClkEnabled != 0) || smi_debug_level > 0) {
		SMIMSG("===SMI LARB%d reg dump, CLK: %d===\n", u4Index, larbClkEnabled);

		/* Staus Registers */
		SMIMSG("[0x0,0x8,0x10]=[0x%x,0x%x,0x%x]\n", M4U_ReadReg32(u4Base, 0x0),
		       M4U_ReadReg32(u4Base, 0x8), M4U_ReadReg32(u4Base, 0x10));
		SMIMSG("[0x24,0x50,0x60]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0x24), M4U_ReadReg32(u4Base, 0x50),
		       M4U_ReadReg32(u4Base, 0x60));
		SMIMSG("[0xa0,0xa4,0xa8]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0xa0), M4U_ReadReg32(u4Base, 0xa4),
		       M4U_ReadReg32(u4Base, 0xa8));
		SMIMSG("[0xac,0xb0,0xb4]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0xac), M4U_ReadReg32(u4Base, 0xb0),
		       M4U_ReadReg32(u4Base, 0xb4));
		SMIMSG("[0xb8,0xbc,0xc0]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0xb8), M4U_ReadReg32(u4Base, 0xbc),
		       M4U_ReadReg32(u4Base, 0xc0));
		SMIMSG("[0xc8,0xcc]=[0x%x,0x%x]\n", M4U_ReadReg32(u4Base, 0xc8),
		       M4U_ReadReg32(u4Base, 0xcc));
		/* Settings */
		SMIMSG("[0x200, 0x204, 0x208]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0x200), M4U_ReadReg32(u4Base, 0x204),
		       M4U_ReadReg32(u4Base, 0x208));

		SMIMSG("[0x20c, 0x210, 0x214]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0x20c), M4U_ReadReg32(u4Base, 0x210),
		       M4U_ReadReg32(u4Base, 0x214));

		SMIMSG("[0x218, 0x21c, 0x220]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0x218), M4U_ReadReg32(u4Base, 0x21c),
		       M4U_ReadReg32(u4Base, 0x220));

		SMIMSG("[0x224, 0x228, 0x22c]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0x224), M4U_ReadReg32(u4Base, 0x228),
		       M4U_ReadReg32(u4Base, 0x22c));

		SMIMSG("[0x230, 0x234, 0x238]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0x230), M4U_ReadReg32(u4Base, 0x234),
		       M4U_ReadReg32(u4Base, 0x238));

		SMIMSG("[0x23c, 0x240, 0x244]=[0x%x,0x%x,0x%x]\n",
		       M4U_ReadReg32(u4Base, 0x23c), M4U_ReadReg32(u4Base, 0x240),
		       M4U_ReadReg32(u4Base, 0x244));

		SMIMSG("[0x248, 0x24c]=[0x%x,0x%x]\n", M4U_ReadReg32(u4Base, 0x248),
		       M4U_ReadReg32(u4Base, 0x24c));
	} else {
		SMIMSG("===SMI LARB%d clock is disabled===\n", u4Index);
	}

}

static void smi_dump_format(unsigned long base, unsigned int from, unsigned int to)
{
	int i, j, left;
	unsigned int value[8];

	for (i = from; i <= to; i += 32) {
		for (j = 0; j < 8; j++)
			value[j] = M4U_ReadReg32(base, i + j * 4);

		SMIMSG2("%8x %x %x %x %x %x %x %x %x\n", i, value[0], value[1],
			value[2], value[3], value[4], value[5], value[6], value[7]);
	}

	left = ((from - to) / 4 + 1) % 8;

	if (left) {
		memset(value, 0, 8 * sizeof(unsigned int));

		for (j = 0; j < left; j++)
			value[j] = M4U_ReadReg32(base, i - 32 + j * 4);

		SMIMSG2("%8x %x %x %x %x %x %x %x %x\n", i - 32 + j * 4, value[0],
			value[1], value[2], value[3], value[4], value[5], value[6], value[7]);
	}
}

static void smi_dumpLarb(unsigned int index)
{
	unsigned long u4Base;

	u4Base = get_larb_base_addr(index);

	if (u4Base == SMI_ERROR_ADDR) {
		SMIMSG2("Doesn't support reg dump for Larb%d\n", index);

		return;
	} else {
		SMIMSG2("===SMI LARB%d reg dump base 0x%lx===\n", index, u4Base);

		smi_dump_format(u4Base, 0, 0x434);
		smi_dump_format(u4Base, 0xF00, 0xF0C);
	}
}

static void smi_dumpCommon(void)
{
	SMIMSG2("===SMI COMMON reg dump base 0x%lx===\n", SMI_COMMON_EXT_BASE);

	smi_dump_format(SMI_COMMON_EXT_BASE, 0x1A0, 0x418);
}

void smi_dumpDebugMsg(void)
{
	unsigned int u4Index;

	/* SMI COMMON dump, 0 stands for not pass log to CMDQ error dumping messages */
	smi_dumpCommonDebugMsg(0);

	/* dump all SMI LARB */
	for (u4Index = 0; u4Index < SMI_LARB_NR; u4Index++)
		smi_dumpLarbDebugMsg(u4Index);
}

int smi_debug_bus_hanging_detect_ext(unsigned int larbs, int show_dump, int output_gce_buffer)
{
	SMIMSG("Not support to write log into CMDQ error dumping messages\n");
	return 0;
}

int smi_debug_bus_hanging_detect(unsigned int larbs, int show_dump)
{

	int i = 0;
	int dump_time = 0;
	int is_smi_issue = 0;
	int status_code = 0;
	int output_gce_buffer = 0;
	/* Keep the dump result */
	unsigned char smi_common_busy_count = 0;
	volatile unsigned int reg_temp = 0;
	unsigned char smi_larb_busy_count[SMI_LARB_NR] = { 0 };
	unsigned char smi_larb_mmu_status[SMI_LARB_NR] = { 0 };

	/* dump resister and save resgister status */
	for (dump_time = 0; dump_time < 5; dump_time++) {
		unsigned int u4Index = 0;
		reg_temp = M4U_ReadReg32(SMI_COMMON_EXT_BASE, 0x400);
		if ((reg_temp & (1 << 30)) == 0) {
			/* smi common is busy */
			smi_common_busy_count++;
		}
		/* Dump smi common regs */
		if (show_dump != 0)
			smi_dumpCommonDebugMsg(output_gce_buffer);

		for (u4Index = 0; u4Index < SMI_LARB_NR; u4Index++) {
			unsigned long u4Base = get_larb_base_addr(u4Index);
			if (u4Base != SMI_ERROR_ADDR) {
				reg_temp = M4U_ReadReg32(u4Base, 0x0);
				if (reg_temp != 0) {
					/* Larb is busy */
					smi_larb_busy_count[u4Index]++;
				}
				smi_larb_mmu_status[u4Index] = M4U_ReadReg32(u4Base, 0xa0);
				if (show_dump != 0)
					smi_dumpLarbDebugMsg(u4Index);
			}
		}

	}

	/* Show the checked result */
	for (i = 0; i < SMI_LARB_NR; i++) {	/* Check each larb */
		if (SMI_DGB_LARB_SELECT(larbs, i)) {
			/* larb i has been selected */
			/* Get status code */

			if (smi_larb_busy_count[i] == 5) {	/* The larb is always busy */
				if (smi_common_busy_count == 5) {	/* smi common is always busy */
					status_code = 1;
				} else if (smi_common_busy_count == 0) {	/* smi common is always idle */
					status_code = 2;
				} else {
					status_code = 5;	/* smi common is sometimes busy and idle */
				}
			} else if (smi_larb_busy_count[i] == 0) {	/* The larb is always idle */
				if (smi_common_busy_count == 5) {	/* smi common is always busy */
					status_code = 3;
				} else if (smi_common_busy_count == 0) {	/* smi common is always idle */
					status_code = 4;
				} else {
					status_code = 6;	/* smi common is sometimes busy and idle */
				}
			} else {	/* sometime the larb is busy */
				if (smi_common_busy_count == 5) {	/* smi common is always busy */
					status_code = 7;
				} else if (smi_common_busy_count == 0) {	/* smi common is always idle */
					status_code = 8;
				} else {
					status_code = 9;	/* smi common is sometimes busy and idle */
				}
			}

			/* Send the debug message according to the final result */
			switch (status_code) {
			case 1:
			case 3:
			case 5:
			case 7:
			case 8:
				SMIMSG3(output_gce_buffer,
					"Larb%d Busy=%d/5, SMI Common Busy=%d/5, status=%d ==> Check engine's state first\n",
					i, smi_larb_busy_count[i], smi_common_busy_count,
					status_code);
				SMIMSG3(output_gce_buffer,
					"If the engine is waiting for Larb%ds' response, it needs SMI HW's check\n",
					i);
				break;
			case 2:
				if (smi_larb_mmu_status[i] == 0) {
					SMIMSG3(output_gce_buffer,
						"Larb%d Busy=%d/5, SMI Common Busy=%d/5, status=%d ==> Check engine state first\n",
						i, smi_larb_busy_count[i], smi_common_busy_count,
						status_code);
					SMIMSG3(output_gce_buffer,
						"If the engine is waiting for Larb%ds' response, it needs SMI HW's check\n",
						i);
				} else {
					SMIMSG3(output_gce_buffer,
						"Larb%d Busy=%d/5, SMI Common Busy=%d/5, status=%d ==> MMU port config error\n",
						i, smi_larb_busy_count[i], smi_common_busy_count,
						status_code);
					is_smi_issue = 1;
				}
				break;
			case 4:
			case 6:
			case 9:
				SMIMSG3(output_gce_buffer,
					"Larb%d Busy=%d/5, SMI Common Busy=%d/5, status=%d ==> not SMI issue\n",
					i, smi_larb_busy_count[i], smi_common_busy_count,
					status_code);
				break;
			default:
				SMIMSG3(output_gce_buffer,
					"Larb%d Busy=%d/5, SMI Common Busy=%d/5, status=%d ==> status unknown\n",
					i, smi_larb_busy_count[i], smi_common_busy_count,
					status_code);
				break;
			}
		}

	}

	return is_smi_issue;
}

void smi_client_status_change_notify(int module, int mode)
{

}

#if IS_ENABLED(CONFIG_COMPAT)
/* 32 bits process ioctl support: */
/* This is prepared for the future extension since currently the sizes of 32 bits */
/* and 64 bits smi parameters are the same. */

typedef struct {
	compat_int_t scenario;
	compat_int_t b_on_off;	/* 0 : exit this scenario , 1 : enter this scenario */
} MTK_SMI_COMPAT_BWC_CONFIG;

typedef struct {
	compat_int_t property;
	compat_int_t value1;
	compat_int_t value2;
} MTK_SMI_COMPAT_BWC_INFO_SET;

typedef struct {
	compat_uint_t flag;	/* Reserved */
	compat_int_t concurrent_profile;
	compat_int_t sensor_size[2];
	compat_int_t video_record_size[2];
	compat_int_t display_size[2];
	compat_int_t tv_out_size[2];
	compat_int_t fps;
	compat_int_t video_encode_codec;
	compat_int_t video_decode_codec;
	compat_int_t hw_ovl_limit;
} MTK_SMI_COMPAT_BWC_MM_INFO;

#define COMPAT_MTK_IOC_SMI_BWC_CONFIG      MTK_IOW(24, MTK_SMI_COMPAT_BWC_CONFIG)
#define COMPAT_MTK_IOC_SMI_BWC_INFO_SET    MTK_IOWR(28, MTK_SMI_COMPAT_BWC_INFO_SET)
#define COMPAT_MTK_IOC_SMI_BWC_INFO_GET    MTK_IOWR(29, MTK_SMI_COMPAT_BWC_MM_INFO)

static int compat_get_smi_bwc_config_struct(MTK_SMI_COMPAT_BWC_CONFIG __user *data32,
					    MTK_SMI_BWC_CONFIG __user *data)
{

	compat_int_t i;
	int err;

	/* since the int sizes of 32 A32 and A64 are equal so we don't convert them actually here */
	err = get_user(i, &(data32->scenario));
	err |= put_user(i, &(data->scenario));
	err |= get_user(i, &(data32->b_on_off));
	err |= put_user(i, &(data->b_on_off));

	return err;
}

static int compat_get_smi_bwc_mm_info_set_struct(MTK_SMI_COMPAT_BWC_INFO_SET __user *data32,
						 MTK_SMI_BWC_INFO_SET __user *data)
{

	compat_int_t i;
	int err;

	/* since the int sizes of 32 A32 and A64 are equal so we don't convert them actually here */
	err = get_user(i, &(data32->property));
	err |= put_user(i, &(data->property));
	err |= get_user(i, &(data32->value1));
	err |= put_user(i, &(data->value1));
	err |= get_user(i, &(data32->value2));
	err |= put_user(i, &(data->value2));

	return err;
}

static int compat_get_smi_bwc_mm_info_struct(MTK_SMI_COMPAT_BWC_MM_INFO __user *data32,
					     MTK_SMI_BWC_MM_INFO __user *data)
{
	compat_uint_t u;
	compat_int_t i;
	compat_int_t p[2];
	int err;

	/* since the int sizes of 32 A32 and A64 are equal so we don't convert them actually here */
	err = get_user(u, &(data32->flag));
	err |= put_user(u, &(data->flag));
	err |= get_user(i, &(data32->concurrent_profile));
	err |= put_user(i, &(data->concurrent_profile));
	err |= copy_from_user(p, &(data32->sensor_size), sizeof(p));
	err |= copy_to_user(&(data->sensor_size), p, sizeof(p));
	err |= copy_from_user(p, &(data32->video_record_size), sizeof(p));
	err |= copy_to_user(&(data->video_record_size), p, sizeof(p));
	err |= copy_from_user(p, &(data32->display_size), sizeof(p));
	err |= copy_to_user(&(data->display_size), p, sizeof(p));
	err |= copy_from_user(p, &(data32->tv_out_size), sizeof(p));
	err |= copy_to_user(&(data->tv_out_size), p, sizeof(p));
	err |= get_user(i, &(data32->fps));
	err |= put_user(i, &(data->fps));
	err |= get_user(i, &(data32->video_encode_codec));
	err |= put_user(i, &(data->video_encode_codec));
	err |= get_user(i, &(data32->video_decode_codec));
	err |= put_user(i, &(data->video_decode_codec));
	err |= get_user(i, &(data32->hw_ovl_limit));
	err |= put_user(i, &(data->hw_ovl_limit));


	return err;
}

static int compat_put_smi_bwc_mm_info_struct(MTK_SMI_COMPAT_BWC_MM_INFO __user *data32,
					     MTK_SMI_BWC_MM_INFO __user *data)
{

	compat_uint_t u;
	compat_int_t i;
	compat_int_t p[2];
	int err;

	/* since the int sizes of 32 A32 and A64 are equal so we don't convert them actually here */
	err = get_user(u, &(data->flag));
	err |= put_user(u, &(data32->flag));
	err |= get_user(i, &(data->concurrent_profile));
	err |= put_user(i, &(data32->concurrent_profile));
	err |= copy_from_user(p, &(data->sensor_size), sizeof(p));
	err |= copy_to_user(&(data32->sensor_size), p, sizeof(p));
	err |= copy_from_user(p, &(data->video_record_size), sizeof(p));
	err |= copy_to_user(&(data32->video_record_size), p, sizeof(p));
	err |= copy_from_user(p, &(data->display_size), sizeof(p));
	err |= copy_to_user(&(data32->display_size), p, sizeof(p));
	err |= copy_from_user(p, &(data->tv_out_size), sizeof(p));
	err |= copy_to_user(&(data32->tv_out_size), p, sizeof(p));
	err |= get_user(i, &(data->fps));
	err |= put_user(i, &(data32->fps));
	err |= get_user(i, &(data->video_encode_codec));
	err |= put_user(i, &(data32->video_encode_codec));
	err |= get_user(i, &(data->video_decode_codec));
	err |= put_user(i, &(data32->video_decode_codec));
	err |= get_user(i, &(data->hw_ovl_limit));
	err |= put_user(i, &(data32->hw_ovl_limit));
	return err;
}

static long MTK_SMI_COMPAT_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_MTK_IOC_SMI_BWC_CONFIG:
		{
			if (COMPAT_MTK_IOC_SMI_BWC_CONFIG == MTK_IOC_SMI_BWC_CONFIG) {
				/* SMIMSG("Optimized compct IOCTL: COMPAT_MTK_IOC_SMI_BWC_CONFIG"); */
				return filp->f_op->unlocked_ioctl(filp, cmd,
								  (unsigned long)compat_ptr(arg));
			} else {

				MTK_SMI_COMPAT_BWC_CONFIG __user *data32;
				MTK_SMI_BWC_CONFIG __user *data;
				int err;

				data32 = compat_ptr(arg);
				data = compat_alloc_user_space(sizeof(MTK_SMI_BWC_CONFIG));

				if (data == NULL)
					return -EFAULT;

				err = compat_get_smi_bwc_config_struct(data32, data);
				if (err)
					return err;

				ret = filp->f_op->unlocked_ioctl(filp, MTK_IOC_SMI_BWC_CONFIG,
								 (unsigned long)data);
				return ret;
			}
		}

	case COMPAT_MTK_IOC_SMI_BWC_INFO_SET:
		{

			if (COMPAT_MTK_IOC_SMI_BWC_INFO_SET == MTK_IOC_SMI_BWC_INFO_SET) {
				/* SMIMSG("Optimized compct IOCTL: COMPAT_MTK_IOC_SMI_BWC_INFO_SET"); */
				return filp->f_op->unlocked_ioctl(filp, cmd,
								  (unsigned long)compat_ptr(arg));
			} else {

				MTK_SMI_COMPAT_BWC_INFO_SET __user *data32;
				MTK_SMI_BWC_INFO_SET __user *data;
				int err;

				data32 = compat_ptr(arg);
				data = compat_alloc_user_space(sizeof(MTK_SMI_BWC_INFO_SET));
				if (data == NULL)
					return -EFAULT;

				err = compat_get_smi_bwc_mm_info_set_struct(data32, data);
				if (err)
					return err;

				return filp->f_op->unlocked_ioctl(filp, MTK_IOC_SMI_BWC_INFO_SET,
								  (unsigned long)data);
			}
		}

	case COMPAT_MTK_IOC_SMI_BWC_INFO_GET:
		{

			if (COMPAT_MTK_IOC_SMI_BWC_INFO_GET == MTK_IOC_SMI_BWC_INFO_GET) {
				SMIMSG("Optimized compct IOCTL: COMPAT_MTK_IOC_SMI_BWC_INFO_GET");
				return filp->f_op->unlocked_ioctl(filp, cmd,
								  (unsigned long)compat_ptr(arg));
			} else {
				MTK_SMI_COMPAT_BWC_MM_INFO __user *data32;
				MTK_SMI_BWC_MM_INFO __user *data;
				int err;

				data32 = compat_ptr(arg);
				data = compat_alloc_user_space(sizeof(MTK_SMI_BWC_MM_INFO));

				if (data == NULL)
					return -EFAULT;

				err = compat_get_smi_bwc_mm_info_struct(data32, data);
				if (err)
					return err;

				ret = filp->f_op->unlocked_ioctl(filp, MTK_IOC_SMI_BWC_INFO_GET,
								 (unsigned long)data);

				err = compat_put_smi_bwc_mm_info_struct(data32, data);

				if (err)
					return err;

				return ret;
			}
		}

	case MTK_IOC_SMI_DUMP_LARB:
	case MTK_IOC_SMI_DUMP_COMMON:
#if defined(SMI_RO)
	case MTK_IOC_MMDVFS_CMD:
#endif

		return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
	default:
		return -ENOIOCTLCMD;
	}

}

#endif

#if defined(SMI_RO)
subsys_initcall(smi_init);
#elif defined(SMI_K2)
module_init(smi_init);
#endif
module_exit(smi_exit);

module_param_named(debug_level, smi_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(tuning_mode, smi_tuning_mode, uint, S_IRUGO | S_IWUSR);
module_param_named(wifi_disp_transaction, wifi_disp_transaction, uint, S_IRUGO | S_IWUSR);

MODULE_DESCRIPTION("MTK SMI driver");
MODULE_AUTHOR("Frederic Chen<frederic.chen@mediatek.com>");
MODULE_LICENSE("GPL");
