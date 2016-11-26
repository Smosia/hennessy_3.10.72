#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "mt_smi.h"
#include "smi_common.h"
#include "smi_info_util.h"

#if defined(SMI_RO)
#include "mmdvfs_mgr.h"
#endif

int smi_set_mm_info_ioctl_wrapper(struct file *pFile, unsigned int cmd, unsigned long param)
{
	int ret = 0;
	MTK_SMI_BWC_INFO_SET cfg;
	ret = copy_from_user(&cfg, (void *)param, sizeof(MTK_SMI_BWC_INFO_SET));
	if (ret) {
		SMIMSG(" MTK_IOC_SMI_BWC_INFO_SET, copy_to_user failed: %d\n", ret);
		return -EFAULT;
	}
	/* Set the address to the value assigned by user space program */
	smi_bwc_mm_info_set(cfg.property, cfg.value1, cfg.value2);
	/* SMIMSG("Handle MTK_IOC_SMI_BWC_INFO_SET request... finish"); */
	return ret;
}


int smi_get_mm_info_ioctl_wrapper(struct file *pFile, unsigned int cmd, unsigned long param)
{
	int ret = 0;
	ret = copy_to_user((void *)param, (void *)&g_smi_bwc_mm_info, sizeof(MTK_SMI_BWC_MM_INFO));

	if (ret) {
		SMIMSG(" MTK_IOC_SMI_BWC_INFO_GET, copy_to_user failed: %d\n", ret);
		return -EFAULT;
	}
	/* SMIMSG("Handle MTK_IOC_SMI_BWC_INFO_GET request... finish"); */
	return ret;
}


void smi_bwc_mm_info_set(int property_id, long val1, long val2)
{

	switch (property_id) {
	case SMI_BWC_INFO_CON_PROFILE:
		g_smi_bwc_mm_info.concurrent_profile = (int)val1;
		break;
	case SMI_BWC_INFO_SENSOR_SIZE:
		g_smi_bwc_mm_info.sensor_size[0] = val1;
		g_smi_bwc_mm_info.sensor_size[1] = val2;
		break;
	case SMI_BWC_INFO_VIDEO_RECORD_SIZE:
		g_smi_bwc_mm_info.video_record_size[0] = val1;
		g_smi_bwc_mm_info.video_record_size[1] = val2;
		break;
	case SMI_BWC_INFO_DISP_SIZE:
		g_smi_bwc_mm_info.display_size[0] = val1;
		g_smi_bwc_mm_info.display_size[1] = val2;
		break;
	case SMI_BWC_INFO_TV_OUT_SIZE:
		g_smi_bwc_mm_info.tv_out_size[0] = val1;
		g_smi_bwc_mm_info.tv_out_size[1] = val2;
		break;
	case SMI_BWC_INFO_FPS:
		g_smi_bwc_mm_info.fps = (int)val1;
		break;
	case SMI_BWC_INFO_VIDEO_ENCODE_CODEC:
		g_smi_bwc_mm_info.video_encode_codec = (int)val1;
#if defined(SMI_RO)
		/* AVC @ 60 needs HPM */
		if (g_smi_bwc_mm_info.video_encode_codec == 2) {
			int is_smvr = 0;
			spin_lock(&g_SMIInfo.SMI_lock);
			is_smvr = g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_VR_SLOW] ? 1 : 0;
			spin_unlock(&g_SMIInfo.SMI_lock);
			if (is_smvr)
				mmdvfs_notify_scenario_enter(SMI_BWC_SCEN_VR_SLOW);
		}
#endif
		break;
	case SMI_BWC_INFO_VIDEO_DECODE_CODEC:
		g_smi_bwc_mm_info.video_decode_codec = (int)val1;
		break;
	}
}
