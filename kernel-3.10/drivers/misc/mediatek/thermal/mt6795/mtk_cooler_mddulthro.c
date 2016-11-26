#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/err.h>
#include <linux/syscalls.h>
#include <asm/system.h>

#include "mach/mtk_thermal_monitor.h"
#include <mach/system.h>

extern int amddulthro_backoff(int level);

#define mtk_cooler_mddulthro_dprintk_always(fmt, args...) \
  do { pr_notice("thermal/cooler/mddulthro" fmt, ##args); } while(0)

#define mtk_cooler_mddulthro_dprintk(fmt, args...) \
    do { \
        if (1 == cl_mddulthro_klog_on) { \
            pr_notice("thermal/cooler/mddulthro" fmt, ##args); \
        } \
    } while(0)

#define MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO  3

#define MTK_CL_MDDULTHRO_GET_LIMIT(limit, state) \
    do { (limit) = (short) (((unsigned long) (state))>>16); } while(0)

#define MTK_CL_MDDULTHRO_SET_LIMIT(limit, state) \
    do { state = ((((unsigned long) (state))&0xFFFF) | ((short) limit<<16)); } while(0)

#define MTK_CL_MDDULTHRO_GET_CURR_STATE(curr_state, state) \
    do { curr_state = (((unsigned long) (state))&0xFFFF); } while(0)

#define MTK_CL_MDDULTHRO_SET_CURR_STATE(curr_state, state) \
    do { if (0 == curr_state) \
           state &= ~0x1; \
         else \
           state |= 0x1; \
    } while(0)

static int cl_mddulthro_klog_on = 0;
static struct thermal_cooling_device *cl_mddulthro_dev[MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO] = {0};
static unsigned long cl_mddulthro_state[MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO] = {0};
static int cl_mddulthro_cur_limit = 0;

static void 
mtk_cl_mddulthro_set_mddulthro_limit(void)
{
  // TODO: optimize
  int i = 0;
  int min_limit = 0;
  for (; i < MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO; i++)
  {
    unsigned long curr_state;

    MTK_CL_MDDULTHRO_GET_CURR_STATE(curr_state, cl_mddulthro_state[i]);
    if (1 == curr_state)
    {
      int limit;
      MTK_CL_MDDULTHRO_GET_LIMIT(limit, cl_mddulthro_state[i]);
      if ((min_limit < limit) && (limit > 0))
        min_limit = limit;
    }
  }

  if (min_limit != cl_mddulthro_cur_limit)
  {
    cl_mddulthro_cur_limit = min_limit;
#if 1
    if (0 >= cl_mddulthro_cur_limit)
    {
      int ret = 
        amddulthro_backoff(0);
      mtk_cooler_mddulthro_dprintk_always("mtk_cl_mddulthro_set_mddulthro_limit() ret %d limit=0\n", ret);
    }
    else
    {
      int ret = 
        amddulthro_backoff(cl_mddulthro_cur_limit);
      mtk_cooler_mddulthro_dprintk_always("mtk_cl_mddulthro_set_mddulthro_limit() ret %d limit=%d\n", cl_mddulthro_cur_limit);
    }
#endif
  }
}

static int 
mtk_cl_mddulthro_get_max_state(struct thermal_cooling_device *cdev,
                          unsigned long *state)
{        
  *state = 1;
  mtk_cooler_mddulthro_dprintk("mtk_cl_mddulthro_get_max_state() %s %d\n", cdev->type, *state);
  return 0;
}

static int 
mtk_cl_mddulthro_get_cur_state(struct thermal_cooling_device *cdev,
                          unsigned long *state)
{
  MTK_CL_MDDULTHRO_GET_CURR_STATE(*state, *((unsigned long*) cdev->devdata));
  mtk_cooler_mddulthro_dprintk("mtk_cl_mddulthro_get_cur_state() %s %d\n", cdev->type, *state);
  return 0;
}

static int 
mtk_cl_mddulthro_set_cur_state(struct thermal_cooling_device *cdev,
                          unsigned long state)
{
  mtk_cooler_mddulthro_dprintk("mtk_cl_mddulthro_set_cur_state() %s %d\n", cdev->type, state);
  MTK_CL_MDDULTHRO_SET_CURR_STATE(state, *((unsigned long*) cdev->devdata));
  mtk_cl_mddulthro_set_mddulthro_limit();
  
  return 0;
}

/* bind fan callbacks to fan device */
static struct thermal_cooling_device_ops mtk_cl_mddulthro_ops = {
  .get_max_state = mtk_cl_mddulthro_get_max_state,
  .get_cur_state = mtk_cl_mddulthro_get_cur_state,
  .set_cur_state = mtk_cl_mddulthro_set_cur_state,
};

static int mtk_cooler_mddulthro_register_ltf(void)
{
  int i;
  mtk_cooler_mddulthro_dprintk("register ltf\n");

  for (i = MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO; i-- > 0; )
  {
    char temp[20] = {0};
    sprintf(temp, "mtk-cl-mddulthro%02d", i);
    cl_mddulthro_dev[i] = mtk_thermal_cooling_device_register(temp, 
                                                         (void*) &cl_mddulthro_state[i], // put mddulthro state to cooler devdata
                                                         &mtk_cl_mddulthro_ops);
  }

  return 0;
}

static void mtk_cooler_mddulthro_unregister_ltf(void)
{
  int i;
  mtk_cooler_mddulthro_dprintk("unregister ltf\n");

  for (i = MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO; i-- > 0; )
  {
    if (cl_mddulthro_dev[i])
    {
      mtk_thermal_cooling_device_unregister(cl_mddulthro_dev[i]);
      cl_mddulthro_dev[i] = NULL;
      cl_mddulthro_state[i] = 0;
    }
  }
}

static int _mtk_cl_mddulthro_proc_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;

    mtk_cooler_mddulthro_dprintk("[_mtk_cl_mddulthro_proc_read] invoked.\n");

    /**
     * The format to print out: 
     *  kernel_log <0 or 1>
     *  <mtk-cl-mddulthro<ID>> <bcc limit>
     *  ..
     */
    if (NULL == data)
    {
        mtk_cooler_mddulthro_dprintk("[_mtk_cl_mddulthro_proc_read] null data\n");
    }
    else
    {
        int i = 0;

        p += sprintf(p, "klog %d\n", cl_mddulthro_klog_on);
        p += sprintf(p, "curr_limit %d\n", cl_mddulthro_cur_limit);

        for (; i < MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO; i++)
        {
            int limit;
            unsigned int curr_state;
            
            MTK_CL_MDDULTHRO_GET_LIMIT(limit, cl_mddulthro_state[i]);
            MTK_CL_MDDULTHRO_GET_CURR_STATE(curr_state, cl_mddulthro_state[i]);
        
            p += sprintf(p, "mtk-cl-mddulthro%02d lv %d, state %d\n", i, limit, curr_state);
        }
    }

    *start = buf + off;

    len = p - buf;
    if (len > off)
        len -= off;
    else
        len = 0;

    return len < count ? len  : count;
}

static ssize_t _mtk_cl_mddulthro_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    int len = 0;
    char desc[128];
    int klog_on, limit0, limit1, limit2;

    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';

    /**
     * sscanf format <klog_on> <mtk-cl-mddulthro00 limit> <mtk-cl-mddulthro01 limit> ...
     * <klog_on> can only be 0 or 1
     * <mtk-cl-mddulthro00 limit> can only be positive integer or -1 to denote no limit
     */

    if (NULL == data)
    {
        mtk_cooler_mddulthro_dprintk("[_mtk_cl_mddulthro_proc_write] null data\n");
        return -EINVAL;
    }

    // WARNING: Modify here if MTK_THERMAL_MONITOR_COOLER_MAX_EXTRA_CONDITIONS is changed to other than 3
#if (3 == MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO)
    MTK_CL_MDDULTHRO_SET_LIMIT(-1, cl_mddulthro_state[0]);
    MTK_CL_MDDULTHRO_SET_LIMIT(-1, cl_mddulthro_state[1]);
    MTK_CL_MDDULTHRO_SET_LIMIT(-1, cl_mddulthro_state[2]);

    if (1 <= sscanf(desc, "%d %d %d %d",
                    &klog_on, &limit0, &limit1, &limit2))
    {
        if (klog_on == 0 || klog_on == 1)
        {
          cl_mddulthro_klog_on = klog_on;
        }

        if (limit0 >= -1) MTK_CL_MDDULTHRO_SET_LIMIT(limit0, cl_mddulthro_state[0]);
        if (limit1 >= -1) MTK_CL_MDDULTHRO_SET_LIMIT(limit1, cl_mddulthro_state[1]);
        if (limit2 >= -1) MTK_CL_MDDULTHRO_SET_LIMIT(limit2, cl_mddulthro_state[2]);
        
        return count;
    }
    else
#else
#error "Change correspondent part when changing MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO!"
#endif
    {
        mtk_cooler_mddulthro_dprintk("[_mtk_cl_mddulthro_proc_write] bad argument\n");
    }

    return -EINVAL;
}

static int __init mtk_cooler_mddulthro_init(void)
{
  int err = 0;
  int i;

  for (i = MAX_NUM_INSTANCE_MTK_COOLER_MDDULTHRO; i-- > 0; )
  {
    cl_mddulthro_dev[i] = NULL;
    cl_mddulthro_state[i] = 0;
  }

  //cl_mddulthro_dev = NULL;

  mtk_cooler_mddulthro_dprintk("init\n");

  err = mtk_cooler_mddulthro_register_ltf();
  if (err)
    goto err_unreg;

  /* create a proc file */
  {
    struct proc_dir_entry *entry = NULL;
    entry = create_proc_entry("driver/mtk-cl-mddulthro", S_IRUGO | S_IWUSR | S_IWGRP, NULL);
    
    if (NULL != entry)
    {
      entry->read_proc = _mtk_cl_mddulthro_proc_read;
      entry->write_proc = _mtk_cl_mddulthro_proc_write;
      entry->data = cl_mddulthro_state;
      entry->gid = 1000; // allow system process to write this proc
    }
    mtk_cooler_mddulthro_dprintk("[mtk_cooler_mddulthro_init] proc file created: %x \n", entry->data);
  }

  return 0;

err_unreg:
  mtk_cooler_mddulthro_unregister_ltf();
  return err;
}

static void __exit mtk_cooler_mddulthro_exit(void)
{
  mtk_cooler_mddulthro_dprintk("exit\n");

  /* remove the proc file */
  remove_proc_entry("driver/mtk-cl-mddulthro", NULL);
    
  mtk_cooler_mddulthro_unregister_ltf();
}

module_init(mtk_cooler_mddulthro_init);
module_exit(mtk_cooler_mddulthro_exit);



