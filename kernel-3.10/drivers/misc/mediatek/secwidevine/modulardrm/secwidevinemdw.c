/**
 * @file   secwidevinemdwmdw.c
 * @brief  Open widevine modular drm secure driver and receive command from secure driver
 * @Author Rui Hu
 *
 **/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <mach/memory.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/switch.h>

/* only available for trustonic */
#include "mobicore_driver_api.h"
#include "secwidevinemdw.h"

#define secwidevinemdw_NAME     "secwidevinemdw"
#define DEFAULT_HANDLES_NUM (64)
#define MAX_OPEN_SESSIONS   10

/* Debug message event */
#define DBG_EVT_NONE        (0)       /* No event */
#define DBG_EVT_CMD         (1 << 0)  /* SEC CMD related event */
#define DBG_EVT_FUNC        (1 << 1)  /* SEC function event */
#define DBG_EVT_INFO        (1 << 2)  /* SEC information event */
#define DBG_EVT_WRN         (1 << 30) /* Warning event */
#define DBG_EVT_ERR         (1 << 31) /* Error event */
#define DBG_EVT_ALL         (0xffffffff)

#define DBG_EVT_MASK        (DBG_EVT_ALL)

#define MSG(evt, fmt, args...) \
do {    \
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		pr_debug("[secwidevinemdw_normalwd][%s] "fmt, secwidevinemdw_NAME, ##args); \
	}   \
} while (0)

#define MSG_FUNC() MSG(FUNC, "%s\n", __FUNCTION__)

typedef struct mc_session_handle mc_session_handle_t;

typedef enum
{
    WV_DRM_HDCP_OUTPUT_NOT_ALLOWED,
    WV_DRM_HDCP_OUTPUT_ALLOWED,
} WV_DRM_HDCP_OUTPUT;

#define HDCP_VERSION_ANY 0
#define HDCP_VERSION_1_0 100
#define HDCP_VERSION_2_0 200
#define HDCP_VERSION_2_1 210
#define HDCP_VERSION_2_2 220

struct task_struct *secwidevinemdwDci_th;

#define CMD_SEC_WIDEVINE_NOTIFY_HWC 1

typedef struct {
    u32 requiredhdcpversion;
    u32 currenthdcpversion;
} widevinemdw_req_t;

/*
 * DCI message data.
 */
typedef struct {
    union {
        int     command;
        int     response;
    };
    widevinemdw_req_t    request;
} dciMessage_t;

static DEFINE_MUTEX(secwidevinemdw_lock);

//This version just load secure driver
#define DR_SECWIDEVINEMDW_UUID {0x40, 0x18, 0x83, 0x11, 0xfa, 0xf3, 0x43, 0x48, \
                                0x8d, 0xb8, 0x88, 0xad, 0x39, 0x49, 0x6f, 0x9a}

static const struct mc_uuid_t secwidevinemdw_uuid = {DR_SECWIDEVINEMDW_UUID};

static struct mc_session_handle secwidevinemdwdr_session = {0};
static u32 secwidevinemdw_session_ref = 0;
static u32 secwidevinemdw_devid = MC_DEVICE_ID_DEFAULT;
static dciMessage_t *secwidevinemdw_dci = NULL;
static struct switch_dev secwidevinemdw_switch_data;

static int secwidevinemdw_execute(u32 cmd)
{

    mutex_lock(&secwidevinemdw_lock);

    if (NULL == secwidevinemdw_dci)
    {
        mutex_unlock(&secwidevinemdw_lock);
        MSG(ERR, "secwidevinemdw_dci not exist\n");
        return -ENODEV;
    }
    switch(cmd)
    {
    case CMD_SEC_WIDEVINE_NOTIFY_HWC:
        MSG(INFO, "%s: switch_set_state:%u\n", __func__, secwidevinemdw_dci->request.requiredhdcpversion);
        switch_set_state(&secwidevinemdw_switch_data,
                secwidevinemdw_dci->request.requiredhdcpversion);
        //Send uevent to notify hwc
        break;
    default:
        MSG(ERR, "Unkonw cmd\n");
        break;
    }

exit:

    mutex_unlock(&secwidevinemdw_lock);

    return 0;
}


void secwidevinemdw_listenDci(void)
{
    enum mc_result mc_ret = MC_DRV_OK;
    u32 cmdId;

    MSG(INFO, "%s: DCI listener.\n", __func__);

    for (;;) {

        MSG(INFO, "%s: Waiting for notification\n", __func__);

        /* Wait for notification from SWd */
        mc_ret = mc_wait_notification(&secwidevinemdwdr_session, MC_INFINITE_TIMEOUT);
        if (mc_ret != MC_DRV_OK) {
            MSG(ERR, "%s: mcWaitNotification failed, mc_ret=%d\n", __func__, mc_ret);
            break;
        }

        cmdId = secwidevinemdw_dci->command;
        u32 currentversion = secwidevinemdw_dci->request.currenthdcpversion;
        u32 requiredversion = secwidevinemdw_dci->request.requiredhdcpversion;
        MSG(INFO, "%s: wait notification done!! cmdId = 0x%x, current = 0x%x, required = 0x%x\n",
                __func__, cmdId, currentversion, requiredversion);
        /* Received exception. */
        mc_ret = secwidevinemdw_execute(cmdId);

        /* Notify the STH*/
        mc_ret = mc_notify(&secwidevinemdwdr_session);
        if (mc_ret != MC_DRV_OK) {
            MSG(ERR, "%s: mcNotify returned: %d\n", __func__, mc_ret);
            break;
        }
    }
}

//Open driver in open
static int secwidevinemdw_session_open(void)
{
    enum mc_result mc_ret = MC_DRV_OK;
    mutex_lock(&secwidevinemdw_lock);

    do {
        /* sessions reach max numbers ? */
        if (secwidevinemdw_session_ref > MAX_OPEN_SESSIONS)
        {
            MSG(WRN, "secwidevinemdw_session > 0x%x\n", MAX_OPEN_SESSIONS);
            break;
        }

        if (secwidevinemdw_session_ref > 0)
        {
            MSG(WRN, "secwidevinemdw_session already open");
            secwidevinemdw_session_ref++;
            break;
        }

        /* open device */
        mc_ret = mc_open_device(secwidevinemdw_devid);
        if (MC_DRV_OK != mc_ret)
        {
            MSG(ERR, "mc_open_device failed: %d\n", mc_ret);
            break;
        }

        /* allocating WSM for DCI */
        mc_ret = mc_malloc_wsm(secwidevinemdw_devid, 0, sizeof(dciMessage_t),
                (uint8_t **)&secwidevinemdw_dci, 0);
        if (MC_DRV_OK != mc_ret) {
            mc_close_device(secwidevinemdw_devid);
            MSG(ERR, "mc_malloc_wsm failed: %d\n", mc_ret);
            break;
        }

        // open session
        secwidevinemdwdr_session.device_id = secwidevinemdw_devid;
        mc_ret = mc_open_session(&secwidevinemdwdr_session, &secwidevinemdw_uuid,
            (uint8_t *)secwidevinemdw_dci, sizeof(dciMessage_t));

        if (MC_DRV_OK != mc_ret)
        {
            mc_free_wsm(secwidevinemdw_devid, (uint8_t *)secwidevinemdw_dci);
            secwidevinemdw_dci = NULL;
            mc_close_device(secwidevinemdw_devid);
            MSG(ERR, "mc_open_session failed: %d\n", mc_ret);
            break;
        }
        /* create a thread for listening DCI signals */
        secwidevinemdwDci_th = kthread_run(secwidevinemdw_listenDci, NULL, "secwidevinemdw_Dci");
        if (IS_ERR(secwidevinemdwDci_th))
        {
            mc_close_session(&secwidevinemdwdr_session);
            mc_free_wsm(secwidevinemdw_devid, (uint8_t *) secwidevinemdw_dci);
            secwidevinemdw_dci = NULL;
            mc_close_device(secwidevinemdw_devid);
            MSG(ERR, "%s, init kthread_run failed!\n", __func__);
            break;
        }
        secwidevinemdw_session_ref = 1;

    } while(0);

    MSG(INFO, "secwidevinemdw_session_open: ret=%d, ref=%d\n", mc_ret, secwidevinemdw_session_ref);
    MSG(INFO, "driver sessionId = %d, deviceId = %d\n",
                secwidevinemdwdr_session.session_id, secwidevinemdwdr_session.device_id);

    mutex_unlock(&secwidevinemdw_lock);

    if (MC_DRV_OK != mc_ret)
    {
        MSG(ERR, "secwidevinemdw_session_open fail");
        return -ENXIO;
    }

    return 0;
}

//Close trustlet and driver
static int secwidevinemdw_session_close(void)
{
    enum mc_result mc_ret = MC_DRV_OK;

    mutex_lock(&secwidevinemdw_lock);

    do {
        /* session is already closed ? */
        if (secwidevinemdw_session_ref == 0)
        {
            MSG(WRN, "secwidevinemdw_session already closed\n");
            break;
        }

        if (secwidevinemdw_session_ref > 1)
        {
            secwidevinemdw_session_ref--;
            break;
        }

        /* close session */
        mc_ret = mc_close_session(&secwidevinemdwdr_session);

        /* free WSM for DCI */
        mc_ret = mc_free_wsm(secwidevinemdw_devid, (uint8_t*) secwidevinemdw_dci);
        secwidevinemdw_dci = NULL;
        secwidevinemdw_session_ref = 0;

        /* close device */
        mc_ret = mc_close_device(secwidevinemdw_devid);
        if (MC_DRV_OK != mc_ret)
        {
            MSG(ERR, "mc_close_device failed: %d\n", mc_ret);
        }

    } while(0);

    MSG(INFO, "secwidevinemdw_session_close: ret=%d, ref=%d\n", mc_ret, secwidevinemdw_session_ref);

    mutex_unlock(&secwidevinemdw_lock);

    if (MC_DRV_OK != mc_ret)
    {
        return -ENXIO;
    }

    return 0;

}

static int secwidevinemdw_open(struct inode *inode, struct file *file)
{
    /* open session */
    if (secwidevinemdw_session_open() < 0)
    {
        MSG(ERR, "secwidevinemdw_open fail - secwidevinemdw_session_open fail");
        return -ENXIO;
    }
    return 0;
}

static int secwidevinemdw_release(struct inode *inode, struct file *file)
{
    int ret = 0;
    ret = secwidevinemdw_session_close();
    return ret;
}

static long secwidevinemdw_read(struct file *file, char *buf, u32 size, loff_t *offset)
{
    //ingore offsset
    u32 hdcpversion = secwidevinemdw_dci->request.currenthdcpversion;

    if(size < sizeof(hdcpversion))
    {
	MSG(ERR, "secwidevinemdw_read fail - buf size(%u) < sizeof(hdcpversion)", size);
        return -1;
    }
    memset(buf, 0, sizeof(hdcpversion));
    copy_to_user(buf, &hdcpversion, sizeof(hdcpversion));
    MSG(INFO, "secwidevinemdw_read: hdcpversion = %u\n", hdcpversion);
    return sizeof(hdcpversion);
}

static struct file_operations secwidevinemdw_fops = {
    .owner   = THIS_MODULE,
    .open    = secwidevinemdw_open,
    .release = secwidevinemdw_release,
    .unlocked_ioctl = NULL,
    .write   = NULL,
    .read    = secwidevinemdw_read,
};

static int __init secwidevinemdw_init(void)
{
    proc_create("secwidevinemdw1", (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH),
            NULL, &secwidevinemdw_fops);

    // To support modular drm v9, inform HWC the event
    secwidevinemdw_switch_data.name  = "widevine";
    secwidevinemdw_switch_data.index = 0;
    secwidevinemdw_switch_data.state = HDCP_VERSION_ANY;
    MSG(INFO, "secwidevinemdw_session_open: switch_dev_register");
    int ret = switch_dev_register(&secwidevinemdw_switch_data);
    if (ret)
    {
        MSG(INFO, "[secwidevinemdw]switch_dev_register failed, returned:%d!\n", ret);
    }
    return 0;
}

late_initcall(secwidevinemdw_init);
