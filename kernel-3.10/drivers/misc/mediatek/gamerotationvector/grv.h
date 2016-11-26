
#ifndef __GRV_H__
#define __GRV_H__


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
#define GRV_TAG					"<GRV> "
#define GRV_FUN(f)				printk(KERN_ERR GRV_TAG"%s\n", __func__)
#define GRV_ERR(fmt, args...)		printk(KERN_ERR GRV_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GRV_LOG(fmt, args...)		printk(KERN_ERR GRV_TAG fmt, ##args)
#define GRV_VER(fmt, args...)   	printk(KERN_ERR GRV_TAG"%s: "fmt, __func__, ##args) //((void)0)
#define GRV_DBGMSG printk("%s, %d\n", __func__, __LINE__);
#else
#define GRV_TAG					"<GRV> "
#define GRV_FUN(f)	
#define GRV_ERR(fmt, args...)
#define GRV_LOG(fmt, args...)
#define GRV_VER(fmt, args...) 
#define GRV_DBGMSG
#endif
#define OP_GRV_DELAY	0X01
#define	OP_GRV_ENABLE	0X02
#define	OP_GRV_GET_DATA	0X04

#define GRV_INVALID_VALUE -1

#define EVENT_TYPE_GRV_X          			ABS_RY
#define EVENT_TYPE_GRV_Y          			ABS_RZ
#define EVENT_TYPE_GRV_Z          			ABS_THROTTLE
#define EVENT_TYPE_GRV_SCALAR		ABS_RUDDER
#define EVENT_TYPE_GRV_STATUS     		REL_X

#define GRV_VALUE_MAX (32767)
#define GRV_VALUE_MIN (-32768)
#define GRV_STATUS_MIN (0)
#define GRV_STATUS_MAX (64)
#define GRV_DIV_MAX (32767)
#define GRV_DIV_MIN (1)


#define MAX_CHOOSE_GRV_NUM 5

struct grv_control_path
{
	int (*open_report_data)(int open);//open data rerport to HAL
	int (*enable_nodata)(int en);//only enable not report event to HAL
	int (*set_delay)(u64 delay);
	int (*access_data_fifo)(void);//version2.used for flush operate
	bool is_report_input_direct;
	bool is_support_batch;//version2.used for batch mode support flag
	int (*grv_calibration)(int type, int cali[3]);//version3 sensor common layer factory mode API1	
};

struct grv_data_path
{
	int (*get_data)(int *x,int *y, int *z, int *scalar, int *status);
	int (*get_raw_data)(int *x,int *y, int *z, int *scalar);//version3 sensor common layer factory mode API2
	int vender_div;
};

struct grv_init_info
{
    char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver* platform_diver_addr;
};

struct grv_data{
	hwm_sensor_data grv_data ;
	int data_updata;
	//struct mutex lock;
};

struct grv_drv_obj {
    void *self;
	int polling;
	int (*grv_operate)(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout);
};

struct grv_context {
	struct input_dev   *idev;
	struct miscdevice   mdev;
	struct work_struct  report;
	struct mutex grv_op_mutex;
	atomic_t            delay; /*polling period for reporting input event*/
	atomic_t            wake;  /*user-space request to wake-up, used with stop*/
	struct timer_list   timer;  /* polling timer */
	atomic_t            trace;

	struct early_suspend    early_drv;
	atomic_t                early_suspend;
	//struct grv_drv_obj    drv_obj;
	struct grv_data       drv_data;
	struct grv_control_path   grv_ctl;
	struct grv_data_path   grv_data;
	bool			is_active_nodata;		// Active, but HAL don't need data sensor. such as orientation need
	bool			is_active_data;		// Active and HAL need data .
	bool is_first_data_after_enable;
	bool is_polling_run;
	bool is_batch_enable;	//version2.this is used for judging whether sensor is in batch mode
};

//driver API for internal  
//extern int grv_enable_nodata(int enable);
//extern int grv_attach(struct grv_drv_obj *obj);
//driver API for third party vendor

//for auto detect
extern int grv_driver_add(struct grv_init_info* obj) ;
extern int grv_data_report(int x, int y, int z, int scalar, int status);
extern int grv_register_control_path(struct grv_control_path *ctl);
extern int grv_register_data_path(struct grv_data_path *data);

#endif
