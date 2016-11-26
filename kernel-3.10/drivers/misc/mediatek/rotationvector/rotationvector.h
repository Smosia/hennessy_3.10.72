
#ifndef __RV_H__
#define __RV_H__


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
#define RV_TAG					"<RV> "
#define RV_FUN(f)				printk(KERN_ERR RV_TAG"%s\n", __func__)
#define RV_ERR(fmt, args...)		printk(KERN_ERR RV_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define RV_LOG(fmt, args...)		printk(KERN_ERR RV_TAG fmt, ##args)
#define RV_VER(fmt, args...)   	printk(KERN_ERR RV_TAG"%s: "fmt, __func__, ##args) //((void)0)
#define RV_DBGMSG printk("%s, %d\n", __func__, __LINE__);
#else
#define RV_TAG					"<RV> "
#define RV_FUN(f)	
#define RV_ERR(fmt, args...)
#define RV_LOG(fmt, args...)
#define RV_VER(fmt, args...) 
#define RV_DBGMSG
#endif
#define OP_RV_DELAY	0X01
#define	OP_RV_ENABLE	0X02
#define	OP_RV_GET_DATA	0X04

#define RV_INVALID_VALUE -1

#define EVENT_TYPE_RV_X          			ABS_RY
#define EVENT_TYPE_RV_Y          			ABS_RZ
#define EVENT_TYPE_RV_Z          			ABS_THROTTLE
#define EVENT_TYPE_RV_SCALAR		ABS_RUDDER
#define EVENT_TYPE_RV_STATUS     		REL_X

#define RV_VALUE_MAX (32767)
#define RV_VALUE_MIN (-32768)
#define RV_STATUS_MIN (0)
#define RV_STATUS_MAX (64)
#define RV_DIV_MAX (32767)
#define RV_DIV_MIN (1)


#define MAX_CHOOSE_RV_NUM 5

struct rotationvector_control_path
{
	int (*open_report_data)(int open);//open data rerport to HAL
	int (*enable_nodata)(int en);//only enable not report event to HAL
	int (*set_delay)(u64 delay);
	int (*access_data_fifo)(void);//version2.used for flush operate
	bool is_report_input_direct;
	bool is_support_batch;//version2.used for batch mode support flag
	int (*rotationvector_calibration)(int type, int cali[3]);//version3 sensor common layer factory mode API1	
};

struct rotationvector_data_path
{
	int (*get_data)(int *x,int *y, int *z, int *scalar, int *status);
	int (*get_raw_data)(int *x,int *y, int *z, int *scalar);//version3 sensor common layer factory mode API2
	int vender_div;
};

struct rotationvector_init_info
{
    char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver* platform_diver_addr;
};

struct rotationvector_data{
	hwm_sensor_data rotationvector_data ;
	int data_updata;
	//struct mutex lock;
};

struct rotationvector_drv_obj {
    void *self;
	int polling;
	int (*rotationvector_operate)(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout);
};
 
struct rotationvector_context {
	struct input_dev   *idev;
	struct miscdevice   mdev;
	struct work_struct  report;
	struct mutex rotationvector_op_mutex;
	atomic_t            delay; /*polling period for reporting input event*/
	atomic_t            wake;  /*user-space request to wake-up, used with stop*/
	struct timer_list   timer;  /* polling timer */
	atomic_t            trace;

	struct early_suspend    early_drv;
	atomic_t                early_suspend;
	//struct rotationvector_drv_obj    drv_obj;
	struct rotationvector_data       drv_data;
	struct rotationvector_control_path   rotationvector_ctl;
	struct rotationvector_data_path   rotationvector_data;
	bool			is_active_nodata;		// Active, but HAL don't need data sensor. such as orientation need
	bool			is_active_data;		// Active and HAL need data .
	bool is_first_data_after_enable;
	bool is_polling_run;
	bool is_batch_enable;	//version2.this is used for judging whether sensor is in batch mode
};

//driver API for internal  
//extern int rotationvector_enable_nodata(int enable);
//extern int rotationvector_attach(struct rotationvector_drv_obj *obj);
//driver API for third party vendor

//for auto detect
extern int rotationvector_driver_add(struct rotationvector_init_info* obj) ;
extern int rotationvector_data_report(int x, int y, int z, int scalar, int status);
extern int rotationvector_register_control_path(struct rotationvector_control_path *ctl);
extern int rotationvector_register_data_path(struct rotationvector_data_path *data);

#endif
