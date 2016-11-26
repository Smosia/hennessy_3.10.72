#ifndef __MTKFB_VSYNC_H__
#define __MTKFB_VSYNC_H__


#define MTKFB_VSYNC_DEVNAME "mtkfb_vsync"

#define MTKFB_VSYNC_IOCTL_MAGIC      'V'

typedef enum {
	MTKFB_VSYNC_SOURCE_LCM = 0,
	MTKFB_VSYNC_SOURCE_HDMI = 1,
	MTKFB_VSYNC_SOURCE_EPD = 2,
} vsync_src;

#define MTKFB_VSYNC_IOCTL     _IOW(MTKFB_VSYNC_IOCTL_MAGIC, 1, vsync_src)

#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M) || defined(CONFIG_ARCH_MT6753)
void mtkfb_vsync_log_enable(int enable);
#endif

#endif				/* MTKFB_VSYNC_H */
