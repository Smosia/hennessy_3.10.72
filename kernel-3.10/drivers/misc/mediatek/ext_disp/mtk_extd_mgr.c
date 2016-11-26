/*****************************************************************************/
/*****************************************************************************/
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rtpm_prio.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/switch.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <mach/mt_typedefs.h>

#include "extd_log.h"
#include "extd_utils.h"
#include "extd_factory.h"
#include "mtk_extd_mgr.h"
#include <linux/earlysuspend.h>
#include <linux/suspend.h>


#define EXTD_DEVNAME "hdmitx"
#define EXTD_DEV_ID(id) (((id)>>16)&0x0ff)
#define EXTD_DEV_PARAM(id) ((id)&0x0ff)


static dev_t extd_devno;
static struct cdev *extd_cdev;
static struct class *extd_class;

static EXTD_DRIVER *extd_driver[DEV_MAX_NUM - 1];
static EXTD_DRIVER *extd_factory_driver[DEV_MAX_NUM - 1];

static void external_display_enable(unsigned long param)
{
	EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
	int enable = EXTD_DEV_PARAM(param);

	if (device_id >= DEV_MAX_NUM) {
		EXT_MGR_ERR("external_display_enable, device id is invalid!");
		return;
	}

	if (extd_driver[device_id] && extd_driver[device_id]->enable)
		extd_driver[device_id]->enable(enable);

	return;
}

static void external_display_power_enable(unsigned long param)
{
	EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
	int enable = EXTD_DEV_PARAM(param);

	if (device_id >= DEV_MAX_NUM) {
		EXT_MGR_ERR("external_display_power_enable, device id is invalid!");
		return;
	}

	if (extd_driver[device_id] && extd_driver[device_id]->power_enable)
		extd_driver[device_id]->power_enable(enable);

	return;
}

static void external_display_force_disable(unsigned long param)
{
	EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
	int enable = EXTD_DEV_PARAM(param);

	if (device_id >= DEV_MAX_NUM) {
		EXT_MGR_ERR("external_display_force_disable, device id is invalid!");
		return;
	}

	return;
}

static void external_display_set_resolution(unsigned long param)
{
	EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
	int res = EXTD_DEV_PARAM(param);

	if (device_id >= DEV_MAX_NUM) {
		EXT_MGR_ERR("external_display_set_resolution, device id is invalid!");
		return;
	}

	if (extd_driver[device_id] && extd_driver[device_id]->set_resolution)
		extd_driver[device_id]->set_resolution(res);

	return;
}

static int external_display_get_dev_info(unsigned long param, void *info)
{
	int ret = 0;
	EXTD_DEV_ID device_id = EXTD_DEV_ID(param);

	if (device_id >= DEV_MAX_NUM) {
		EXT_MGR_ERR("external_display_get_dev_info, device id is invalid!");
		return;
	}

	if (extd_driver[device_id] && extd_driver[device_id]->get_dev_info)
		ret = extd_driver[device_id]->get_dev_info(AP_GET_INFO, info);

	return ret;
}

static int external_display_get_capability(unsigned long param, void *info)
{
	int ret = 0;
	EXTD_DEV_ID device_id = EXTD_DEV_ID(param);
	device_id = DEV_MHL;

	if (device_id >= DEV_MAX_NUM) {
		EXT_MGR_ERR("external_display_get_capability, device id is invalid!");
		return;
	}

	if (extd_driver[device_id] && extd_driver[device_id]->get_capability)
		ret = extd_driver[device_id]->get_capability(info);

	return ret;
}

static long mtk_extd_mgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int r = 0;

	EXT_MGR_LOG("[EXTD]ioctl= %s(%d), arg = %lu\n", _extd_ioctl_spy(cmd), cmd & 0xff, arg);

	switch (cmd) {
	case MTK_HDMI_AUDIO_VIDEO_ENABLE:
		{
			/* 0xXY
			 * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
			 * the device id:
			 * X = 0 - mhl
			 * X = 1 - wifi display
			 */
			external_display_enable(arg);
			break;
		}
	case MTK_HDMI_POWER_ENABLE:
		{
			/* 0xXY
			 * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
			 * the device id:
			 * X = 0 - mhl
			 * X = 1 - wifi display
			 */
			external_display_power_enable(arg);
			break;
		}
	case MTK_HDMI_VIDEO_CONFIG:
		{
			/* 0xXY
			 * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
			 * the device id:
			 * X = 0 - mhl
			 * X = 1 - wifi display
			 */
			external_display_set_resolution(arg);
			break;
		}
	case MTK_HDMI_FORCE_FULLSCREEN_ON:
		/* case MTK_HDMI_FORCE_CLOSE: */
		{
			/* 0xXY
			 * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
			 * the device id:
			 * X = 0 - mhl
			 * X = 1 - wifi display
			 */
			arg = arg | 0x1;
			external_display_power_enable(arg);
			break;
		}
	case MTK_HDMI_FORCE_FULLSCREEN_OFF:
		/* case MTK_HDMI_FORCE_OPEN: */
		{
			/* 0xXY
			 * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
			 * the device id:
			 * X = 0 - mhl
			 * X = 1 - wifi display
			 */
			arg = arg & 0x0FF0000;
			external_display_power_enable(arg);
			break;
		}
	case MTK_HDMI_GET_DEV_INFO:
		{
			/* 0xXY
			 * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
			 * the device id:
			 * X = 0 - mhl
			 * X = 1 - wifi display
			 */
			r = external_display_get_dev_info(*((unsigned long *)argp), argp);
			break;
		}
	case MTK_HDMI_USBOTG_STATUS:
		{
			/* hdmi_set_USBOTG_status(arg); */
			break;
		}
	case MTK_HDMI_AUDIO_ENABLE:
		{
			EXT_MGR_LOG("[EXTD]hdmi_set_audio_enable, arg = %lu\n", arg);
			int enable = arg & 0x0FF;
			if (extd_driver[DEV_MHL] && extd_driver[DEV_MHL]->set_audio_enable)
				extd_driver[DEV_MHL]->set_audio_enable(enable);

			break;
		}
	case MTK_HDMI_VIDEO_ENABLE:
		{
			break;
		}
	case MTK_HDMI_AUDIO_CONFIG:
		{
			EXT_MGR_LOG("[EXTD]hdmi_audio_format, arg = %lu\n", arg);
			if (extd_driver[DEV_MHL] && extd_driver[DEV_MHL]->set_audio_format)
				extd_driver[DEV_MHL]->set_audio_format(arg);

			break;
		}
	case MTK_HDMI_IS_FORCE_AWAKE:
		{
			/* r = hdmi_is_force_awake(argp); */
			break;
		}
	case MTK_HDMI_GET_EDID:
		{
			if (extd_driver[DEV_MHL] && extd_driver[DEV_MHL]->get_edid)
				r = extd_driver[DEV_MHL]->get_edid(argp);

			break;
		}
	case MTK_HDMI_GET_CAPABILITY:
		{
			/* 0xXY
			 * the low 16 bits(Y) are for disable and enable, and the high 16 bits(X) are for device id
			 * the device id:
			 * X = 0 - mhl
			 * X = 1 - wifi display
			 */
			r = external_display_get_capability(*((unsigned long *)argp), argp);
			break;
		}
	case MTK_HDMI_SCREEN_CAPTURE:
		{
			/* no actions */
			break;
		}
	case MTK_HDMI_FACTORY_CHIP_INIT:
		{
			if (extd_factory_driver[DEV_MHL] && extd_factory_driver[DEV_MHL]->factory_mode_test)
				r = extd_factory_driver[DEV_MHL]->factory_mode_test(STEP1_CHIP_INIT, NULL);

			break;
		}
	case MTK_HDMI_FACTORY_JUDGE_CALLBACK:
		{
			if (extd_factory_driver[DEV_MHL] && extd_factory_driver[DEV_MHL]->factory_mode_test)
				r = extd_factory_driver[DEV_MHL]->factory_mode_test(STEP2_JUDGE_CALLBACK, argp);

			break;
		}
	case MTK_HDMI_FACTORY_START_DPI_AND_CONFIG:
		{
			if (extd_factory_driver[DEV_MHL] && extd_factory_driver[DEV_MHL]->factory_mode_test)
				r = extd_factory_driver[DEV_MHL]->factory_mode_test(STEP3_START_DPI_AND_CONFIG, arg);

			break;
		}
	case MTK_HDMI_FACTORY_DPI_STOP_AND_POWER_OFF:
		{
			/* /r = hdmi_factory_mode_test(STEP4_DPI_STOP_AND_POWER_OFF, NULL); */
			break;
		}
	case MTK_HDMI_FAKE_PLUG_IN:
		{
			int connect = arg & 0x0FF;
			if (extd_driver[DEV_MHL] && extd_driver[DEV_MHL]->fake_connect)
				extd_driver[DEV_MHL]->fake_connect(connect);

			break;
		}
	default:
		{
			EXT_MGR_ERR("[EXTD]ioctl(%d) arguments is not support\n", cmd & 0x0ff);
			r = -EFAULT;
			break;
		}
	}

	EXT_MGR_LOG("[EXTD]ioctl = %s(%d) done\n", _extd_ioctl_spy(cmd), cmd & 0x0ff);
	return r;
}

static int mtk_extd_mgr_open(struct inode *inode, struct file *file)
{
	EXT_MGR_FUNC();
	return 0;
}

static int mtk_extd_mgr_release(struct inode *inode, struct file *file)
{
	EXT_MGR_FUNC();
	return 0;
}

#ifdef CONFIG_COMPAT
static long mtk_extd_mgr_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -ENOIOCTLCMD;

	switch (cmd) {

		/* add cases here for 32bit/64bit conversion */
		/* ... */

	default:
		ret = mtk_extd_mgr_ioctl(file, cmd, arg);
	}

	return ret;
}
#endif

const struct file_operations external_display_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mtk_extd_mgr_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mtk_extd_mgr_compat_ioctl,
#endif
	.open = mtk_extd_mgr_open,
	.release = mtk_extd_mgr_release,
};

static const struct of_device_id extd_of_ids[] = {
	{.compatible = "mediatek,HDMI",},
	{}
};

static int mtk_extd_mgr_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct class_device *class_dev = NULL;

	EXT_MGR_FUNC();

	/* Allocate device number for hdmi driver */
	ret = alloc_chrdev_region(&extd_devno, 0, 1, EXTD_DEVNAME);

	if (ret) {
		EXT_MGR_LOG("alloc_chrdev_region fail\n");
		return -1;
	}

	/* For character driver register to system, device number binded to file operations */
	extd_cdev = cdev_alloc();
	extd_cdev->owner = THIS_MODULE;
	extd_cdev->ops = &external_display_fops;
	ret = cdev_add(extd_cdev, extd_devno, 1);

	/* For device number binded to device name(hdmitx), one class is corresponeded to one node */
	extd_class = class_create(THIS_MODULE, EXTD_DEVNAME);
	/* mknod /dev/hdmitx */
	class_dev = (struct class_device *)device_create(extd_class, NULL, extd_devno, NULL, EXTD_DEVNAME);

	EXT_MGR_LOG("[%s] out\n", __func__);

	return 0;
}

static int mtk_extd_mgr_remove(struct platform_device *pdev)
{
	EXT_MGR_FUNC();

	return 0;
}

#ifdef CONFIG_PM
int extd_pm_suspend(struct device *device)
{
	EXT_MGR_FUNC();

	return 0;
}

int extd_pm_resume(struct device *device)
{
	EXT_MGR_FUNC();

	return 0;
}

const struct dev_pm_ops extd_pm_ops = {
	.suspend = extd_pm_suspend,
	.resume = extd_pm_resume,
};
#endif

static struct platform_driver external_display_driver = {
	.probe = mtk_extd_mgr_probe,
	.remove = mtk_extd_mgr_remove,
	.driver = {
		   .name = EXTD_DEVNAME,
#ifdef CONFIG_PM
		   .pm = &extd_pm_ops,
#endif
		   .of_match_table = extd_of_ids,
		   }
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void extd_early_suspend(struct early_suspend *h)
{
	EXT_MGR_FUNC();
	int i = 0;

	for (i = DEV_MHL; i < DEV_MAX_NUM - 1; i++) {
		if (i != DEV_EINK && extd_driver[i]->power_enable)
			extd_driver[i]->power_enable(0);
	}
}

static void extd_late_resume(struct early_suspend *h)
{
	EXT_MGR_FUNC();
	int i = 0;

	for (i = DEV_MHL; i < DEV_MAX_NUM - 1; i++) {
		if (i != DEV_EINK && extd_driver[i]->power_enable)
			extd_driver[i]->power_enable(1);
	}
}

static struct early_suspend extd_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = extd_early_suspend,
	.resume = extd_late_resume,
};
#endif

static int __init mtk_extd_mgr_init(void)
{
	EXT_MGR_FUNC();
	int i = 0;

	if (platform_driver_register(&external_display_driver)) {
		EXT_MGR_ERR("[EXTD]failed to register mtkfb driver\n");
		return -1;
	}

	extd_driver[DEV_MHL] = EXTD_HDMI_Driver();
	extd_driver[DEV_EINK] = EXTD_EPD_Driver();
	extd_factory_driver[DEV_MHL] = EXTD_Factory_HDMI_Driver();

	for (i = DEV_MHL; i < DEV_MAX_NUM - 1; i++) {
		if (extd_driver[i]->init)
			extd_driver[i]->init();
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&extd_early_suspend_handler);
#endif
	return 0;
}

static void __exit mtk_extd_mgr_exit(void)
{

	device_destroy(extd_class, extd_devno);
	class_destroy(extd_class);
	cdev_del(extd_cdev);
	unregister_chrdev_region(extd_devno, 1);

}

module_init(mtk_extd_mgr_init);
module_exit(mtk_extd_mgr_exit);

MODULE_AUTHOR("www.mediatek.com>");
MODULE_DESCRIPTION("External Display Driver");
MODULE_LICENSE("GPL");
