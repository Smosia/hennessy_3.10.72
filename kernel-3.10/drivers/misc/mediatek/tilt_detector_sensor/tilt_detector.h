#ifndef __TILT_H__
#define __TILT_H__


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


#define TILT_TAG		"<TILT_DETECTOR> "
#define TILT_FUN(f)		printk(TILT_TAG"%s\n", __func__)
#define TILT_ERR(fmt, args...)	printk(TILT_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define TILT_LOG(fmt, args...)	printk(TILT_TAG fmt, ##args)
#define TILT_VER(fmt, args...)  printk(TILT_TAG"%s: "fmt, __func__, ##args) //((void)0)

//#define OP_TILT_DELAY		0X01
#define	OP_TILT_ENABLE		0X02
//#define OP_TILT_GET_DATA	0X04

#define TILT_INVALID_VALUE -1

#define EVENT_TYPE_TILT_VALUE		REL_X

#define TILT_VALUE_MAX (32767)
#define TILT_VALUE_MIN (-32768)
#define TILT_STATUS_MIN (0)
#define TILT_STATUS_MAX (64)
#define TILT_DIV_MAX (32767)
#define TILT_DIV_MIN (1)

typedef enum {
	TILT_DEACTIVATE,
	TILT_ACTIVATE,
	TILT_SUSPEND,
	TILT_RESUME
} tilt_state_e;

struct tilt_control_path
{
//	int (*enable_nodata)(int en);//only enable not report event to HAL
	int (*open_report_data)(int open);//open data rerport to HAL
//	int (*enable)(int en);
	//bool is_support_batch;//version2.used for batch mode support flag
};

struct tilt_data_path
{
	int (*get_data)(u16 *value, int *status);
};

struct tilt_init_info
{
    	char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver* platform_diver_addr;
};

struct tilt_data{
	hwm_sensor_data tilt_data ;
	int data_updata;
	//struct mutex lock;
};

struct tilt_drv_obj {
    void *self;
	int polling;
	int (*tilt_operate)(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout);
};

struct tilt_context {
	struct input_dev   *idev;
	struct miscdevice   mdev;
	struct work_struct  report;
	struct mutex tilt_op_mutex;
	atomic_t            wake;  /*user-space request to wake-up, used with stop*/
	atomic_t            trace;

	struct early_suspend    early_drv;
	atomic_t                early_suspend;
	atomic_t                suspend;

	struct tilt_data       drv_data;
	struct tilt_control_path   tilt_ctl;
	struct tilt_data_path   tilt_data;
	bool			is_active_nodata;		// Active, but HAL don't need data sensor. such as orientation need
	bool			is_active_data;		// Active and HAL need data .
	bool 		is_batch_enable;	//version2.this is used for judging whether sensor is in batch mode
};

extern int tilt_notify(void);
extern int tilt_driver_add(struct tilt_init_info* obj) ;
extern int tilt_register_control_path(struct tilt_control_path *ctl);
extern int tilt_register_data_path(struct tilt_data_path *data);

#endif
