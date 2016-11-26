#ifndef __GRAV_H__
#define __GRAV_H__


#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/hwmsensor.h>
#include <linux/earlysuspend.h> 
#include <linux/hwmsen_dev.h>

//#define DEBUG

#ifdef DEBUG
#define GRAV_TAG					"<GRAVITYSENSOR> "
#define GRAV_FUN(f)				printk(KERN_ERR GRAV_TAG"%s\n", __func__)
#define GRAV_ERR(fmt, args...)		printk(KERN_ERR GRAV_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GRAV_LOG(fmt, args...)		printk(KERN_ERR GRAV_TAG fmt, ##args)
#define GRAV_VER(fmt, args...)   	printk(KERN_ERR GRAV_TAG"%s: "fmt, __func__, ##args) //((void)0)
#define GRAV_DBGMSG printk("%s, %d\n", __func__, __LINE__);
#else
#define GRAV_TAG					"<GRAVITYSENSOR> "
#define GRAV_FUN(f)
#define GRAV_ERR(fmt, args...)
#define GRAV_LOG(fmt, args...)
#define GRAV_VER(fmt, args...)
#define GRAV_DBGMSG
#endif
#define OP_GRAV_DELAY	0X01
#define	OP_GRAV_ENABLE	0X02
#define	OP_GRAV_GET_DATA	0X04

#define GRAV_INVALID_VALUE -1

#define EVENT_TYPE_GRAV_X              ABS_RX
#define EVENT_TYPE_GRAV_Y              ABS_Y
#define EVENT_TYPE_GRAV_Z              ABS_Z
#define EVENT_TYPE_GRAV_STATUS         REL_X

#define GRAV_VALUE_MAX (32767)
#define GRAV_VALUE_MIN (-32768)
#define GRAV_STATUS_MIN (0)
#define GRAV_STATUS_MAX (64)
#define GRAV_DIV_MAX (32767)
#define GRAV_DIV_MIN (1)
#define GRAV_AXIS_X 0
#define GRAV_AXIS_Y 1
#define GRAV_AXIS_Z 2

#define MAX_CHOOSE_GRAV_NUM 5
#define GRAV_AXES_NUM 3
struct grav_control_path
{
	int (*open_report_data)(int open);//open data rerport to HAL
	int (*enable_nodata)(int en);//only enable not report event to HAL
	int (*set_delay)(u64 delay);
	int (*access_data_fifo)(void);//version2.used for flush operate
	bool is_report_input_direct;
	bool is_support_batch;//version2.used for batch mode support flag
	int (*grav_calibration)(int type, int cali[3]);//version3 sensor common layer factory mode API1
};

struct grav_data_path
{
	int (*get_data)(int *x,int *y, int *z,int *status);
	int (*get_raw_data)(int *x,int *y, int *z);//version3 sensor common layer factory mode API2
	int vender_div;
};

struct grav_init_info
{
    char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver* platform_diver_addr;
};

struct grav_data{
	hwm_sensor_data grav_data ;
	int data_updata;
	//struct mutex lock;
};

struct grav_drv_obj {
    void *self;
	int polling;
	int (*grav_operate)(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout);
};

struct grav_context {
	struct input_dev   *idev;
	struct miscdevice   mdev;
	struct work_struct  report;
	struct mutex grav_op_mutex;
	atomic_t            delay; /*polling period for reporting input event*/
	atomic_t            wake;  /*user-space request to wake-up, used with stop*/
	struct timer_list   timer;  /* polling timer */
	atomic_t            trace;

	struct early_suspend    early_drv;
	atomic_t                early_suspend;
	//struct grav_drv_obj    drv_obj;
	struct grav_data       drv_data;
	int                   cali_sw[GRAV_AXES_NUM+1];
	struct grav_control_path   grav_ctl;
	struct grav_data_path   grav_data;
	bool			is_active_nodata;		// Active, but HAL don't need data sensor. such as orientation need
	bool			is_active_data;		// Active and HAL need data .
	bool is_first_data_after_enable;
	bool is_polling_run;
	bool is_batch_enable;	//version2.this is used for judging whether sensor is in batch mode
};

//driver API for internal  
//extern int grav_enable_nodata(int enable);
//extern int grav_attach(struct grav_drv_obj *obj);
//driver API for third party vendor

//for auto detect
extern int grav_driver_add(struct grav_init_info* obj) ;
extern int grav_data_report(int x, int y, int z,int status);
extern int grav_register_control_path(struct grav_control_path *ctl);
extern int grav_register_data_path(struct grav_data_path *data);
#endif
