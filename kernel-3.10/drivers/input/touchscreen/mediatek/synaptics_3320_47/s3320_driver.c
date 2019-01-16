/*
 * Copyright (C) 2013 LG Electironics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/dma-mapping.h>

#include <mach/wd_api.h>
#include <mach/eint.h>
#include <mach/mt_wdt.h>
#include <mach/mt_gpt.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <asm/uaccess.h>
#include <cust_eint.h>

#include "tpd.h"
#include "s3320_driver.h"

#define LGE_USE_SYNAPTICS_FW_UPGRADE
#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
#include <linux/workqueue.h>
#include "s3320_fw_y70.h"
#endif

#include "RefCode_F54.h"

// #include <mach/board_lge.h>
#include <linux/file.h>     //for file access
#include <linux/syscalls.h> //for file access
#include <linux/uaccess.h>  //for file access

#define TEMP_PCB_VER_CHECK
#ifdef TEMP_PCB_VER_CHECK
int gPCBver = 0;    // 0:HW_REV_A , 1:HW_REV_A_2, 2:HW_REV_B

#define HW_REV_A        0
#define HW_REV_A_2      1
#define HW_REV_B        2

#define HW_REV_A_RANGE        20      
#define HW_REV_A_2_RANGE      40
#define HW_REV_B_RANGE        60

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#endif /* TEMP_PCB_VER_CHECK */

/****************************************************************************
* Feature
****************************************************************************/
#define WXSERIES_FW
//#define MT_PROTOCOL_A
#define MAX_RESET_CNT 3
//#define LGE_USE_TOUCH_IC_POWER_REG_CONTROL

/****************************************************************************
* Constants / Definitions
****************************************************************************/
#define LGE_TOUCH_NAME						"lge_touch"
#define	TPD_DEV_NAME						"S3320"

#define TPD_I2C_ADDRESS						0x20

#define BUFFER_SIZE							128

#define MAX_NUM_OF_FINGER					10

#define REG_OBJECT_TYPE_AND_STATUS			0
#define X_POSITION_LSB						1
#define X_POSITION_MSB						2
#define Y_POSITION_LSB						3
#define Y_POSITION_MSB						4
#define PRESSURE							5
#define WX_VALUE							6
#define WY_VALUE							7
#define NUM_OF_EACH_FINGER_DATA_REG			8

#define MAX_POINT_SIZE_FOR_LPWG				12

#define TOUCH_PRESSED						1
#define TOUCH_RELEASED						0
#define BUTTON_CANCLED						0xFF


#define PAGE_MAX_NUM						5
#define PAGE_SELECT_REG						0xFF
#define DESCRIPTION_TABLE_START				0xE9

#define RMI_DEVICE_CONTROL					0x01
#define TOUCHPAD_SENSORS					0x12
#define CAPACITIVE_BUTTON_SENSORS			0x1A
#define FLASH_MEMORY_MANAGEMENT				0x34
#define MULTI_TAP_GESTURE					0x51

/* Function $01 (RMI_DEVICE_CONTROL) */
#define MANUFACTURER_ID_REG					(device_control.query_base)
#define CUSTOMER_FAMILY_REG						(device_control.query_base+2)
#define FW_REVISION_REG						(device_control.query_base+3)
#define PRODUCT_ID_REG						(device_control.query_base+11)
#define DEVICE_CONTROL_REG					(device_control.control_base)

#define DEVICE_CONTROL_NORMAL_OP		0x00  	//sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_SLEEP 			0x01	// sleep mode : go to sleep */
#define DEVICE_CONTROL_SLEEP_NO_RECAL	0x02	// sleep mode : go to sleep. no-recalibration */
#define DEVICE_CONTROL_NOSLEEP			0x04    // no sleep mode : don't go sleep
#define DEVICE_CHARGER_CONNECTED		0x20    //
#define DEVICE_CONTROL_CONFIGURED		0x80    //

#define INTERRUPT_ENABLE_REG				(device_control.control_base+1)
#define ABS0_MASK							0x04
#define BUTTON_MASK							0x10
#define DOZE_INTERVAL_REG				(device_control.control_base+2)
#define DEVICE_STATUS_REG					(device_control.data_base)
#define DEVICE_FAILURE_MASK					0x03
#define FW_CRC_FAILURE_MASK					0x04
#define FLASH_PROG_MASK						0x40
#define UNCONFIGURED_MASK					0x80
#define INTERRUPT_STATUS_REG				(device_control.data_base+1)

/* Function $12 (TOUCHPAD_SENSORS) */
#define TWO_D_FINGER_STATE_REG				(finger.data_base)
#define FINGER_STATE_MASK					0x03
#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS			(0x01)
#define F12_STYLUS_STATUS			(0x02)
#define F12_PALM_STATUS				(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)

/* Function $1A (CAPACITIVE_BUTTON_SENSORS) */
#define BUTTON_DATA_REG						(button.data_base)

/* Function $34 (FLASH_MEMORY_MANAGEMENT) */
#define FLASH_CONFIG_ID_REG					(flash_memory.control_base)
#define FLASH_CONTROL_REG					(flash_memory.data_base+2)
#define FLASH_STATUS_REG					(flash_memory.data_base+3)
#define FLASH_STATUS_MASK					0xFF


/* Function $51 (MULTI_TAP_GESTURE) */
#define MULTITAP_ENABLE_REG					(multi_tap.control_base)
#define ENABLE_MULTITAP_REPORTING_MASK		0x01
#define MULTITAP_COUNT_REG					(multi_tap.control_base)

#define MAXIMUM_INTERTAP_TIME_REG			(multi_tap.control_base+2)
#define MAXIMUM_INTERTAP_DISTANCE_REG		(multi_tap.control_base+3)

#define GESTURE_STATUS_REG					(multi_tap.data_base)
#define GESTURE_PROPERTY_REG				(multi_tap.data_base+1)

/*F51 Others */
#define LPWG_STATUS_REG				0x00 // 4-page
#define LPWG_DATA_REG				0x01 // 4-page
#define LPWG_TAPCOUNT_REG			0x31 // 4-page

#define LPWG_MIN_INTERTAP_REG			0x32 // 4-page
#define LPWG_MAX_INTERTAP_REG			0x33 // 4-page
#define LPWG_TOUCH_SLOP_REG			0x34 // 4-page
#define LPWG_TAP_DISTANCE_REG			0x35 // 4-page
#define LPWG_INTERRUPT_DELAY_REG		0x37 // 4-page

#define LPWG_TAPCOUNT_REG2			0x38 // 4-page
#define LPWG_MIN_INTERTAP_REG2			0x39 // 4-page
#define LPWG_MAX_INTERTAP_REG2			0x3A // 4-page
#define LPWG_TOUCH_SLOP_REG2			0x3B // 4-page
#define LPWG_TAP_DISTANCE_REG2			0x3C // 4-page
#define LPWG_INTERRUPT_DELAY_REG2		0x3E // 4-page
#define WAKEUP_GESTURE_ENABLE_REG		0x20		/* f12_info.ctrl_reg_addr[27] */
#define MISC_HOST_CONTROL_REG			0x3F
#define THERMAL_HIGH_FINGER_AMPLITUDE		0x80	/* finger_amplitude(0x80) = 0.5 */

/*LPWG Control Value*/
#define REPORT_MODE_CTRL	1
#define TCI_ENABLE_CTRL		2
#define TAP_COUNT_CTRL		3
#define MIN_INTERTAP_CTRL	4
#define MAX_INTERTAP_CTRL	5
#define TOUCH_SLOP_CTRL		6
#define TAP_DISTANCE_CTRL	7
#define INTERRUPT_DELAY_CTRL   8

#define TCI_ENABLE_CTRL2	22
#define TAP_COUNT_CTRL2		23
#define MIN_INTERTAP_CTRL2	24
#define MAX_INTERTAP_CTRL2	25
#define TOUCH_SLOP_CTRL2	26
#define TAP_DISTANCE_CTRL2	27
#define INTERRUPT_DELAY_CTRL2   28

/* Palm / Hover */
#define PALM_TYPE	3
#define HOVER_TYPE	5
#define MAX_PRESSURE	255

#define I2C_DELAY			50
#define UEVENT_DELAY			200
#define REBASE_DELAY			100
#define FILE_MOUNT_DELAY             20000
#define CAP_DIFF_MAX             500
#define CAP_MIN_MAX_DIFF             1000
#define KNOCKON_DELAY			60  /* 700ms */

#define TPD_HAVE_BUTTON

#ifdef TPD_HAVE_BUTTON
#ifdef LGE_USE_DOME_KEY
#define TPD_KEY_COUNT	2
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK , KEY_MENU};
#else
#define TPD_KEY_COUNT	3
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK, KEY_MENU, KEY_HOMEPAGE};
#endif
#endif

int case_mode = 1;

/****************************************************************************
* Macros
****************************************************************************/

#define TS_SNTS_GET_X_POSITION(_msb_reg, _lsb_reg) \
		(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_Y_POSITION(_msb_reg, _lsb_reg) \
		(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width_x, _width_y) \
			((_width_x - _width_y) > 0) ? _width_x : _width_y
#define TS_SNTS_GET_WIDTH_MINOR(_width_x, _width_y) \
			((_width_x - _width_y) > 0) ? _width_y : _width_x
#define TS_SNTS_GET_ORIENTATION(_width_y, _width_x) \
		((_width_y - _width_x) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure
#define MS_TO_NS(x)		(x * 1E6L)


/*Do Function Macro*/
#define DO_IF(do_work, goto_error) 								\
	do {												\
		if(do_work){										\
			TPD_ERR("[Touch E] Action Failed");	\
			goto goto_error;				\
		}							\
	} while (0)

#define DO_SAFE(do_work, goto_error) 				\
		DO_IF(unlikely((do_work) < 0), goto_error)
#define LPWG_PAGE				0x04
#define INTERRUPT_MASK_FLASH			0x01
#define INTERRUPT_MASK_ABS0			0x04
#define INTERRUPT_MASK_BUTTON			0x10
#define INTERRUPT_MASK_CUSTOM			0x40



/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct {
	u8 query_base;
	u8 command_base;
	u8 control_base;
	u8 data_base;
	u8 int_source_count;
	u8 function_exist;
} function_descriptor;

typedef struct {
	unsigned char finger_data[MAX_NUM_OF_FINGER][NUM_OF_EACH_FINGER_DATA_REG];
} touch_sensor_data;

typedef struct {
	unsigned int pos_x[MAX_NUM_OF_FINGER];
	unsigned int pos_y[MAX_NUM_OF_FINGER];
	unsigned char pressure[MAX_NUM_OF_FINGER];
	int total_num;
	char palm;
} touch_finger_info;

enum {
	IGNORE_INTERRUPT = 100,
	NEED_TO_OUT,
	NEED_TO_INIT,
};

enum {
	IC_CTRL_CODE_NONE = 0,
	IC_CTRL_BASELINE,
	IC_CTRL_READ,
	IC_CTRL_WRITE,
	IC_CTRL_RESET_CMD,
	IC_CTRL_REPORT_MODE,
	IC_CTRL_DOUBLE_TAP_WAKEUP_MODE,
};

enum {
	BASELINE_OPEN = 0,
	BASELINE_FIX,
	BASELINE_REBASE,
};

enum {
	TIME_EX_INIT_TIME,
	TIME_EX_FIRST_INT_TIME,
	TIME_EX_PREV_PRESS_TIME,
	TIME_EX_CURR_PRESS_TIME,
	TIME_EX_BUTTON_PRESS_START_TIME,
	TIME_EX_BUTTON_PRESS_END_TIME,
	TIME_EX_FIRST_GHOST_DETECT_TIME,
	TIME_EX_SECOND_GHOST_DETECT_TIME,
	TIME_EX_CURR_INT_TIME,
	TIME_EX_PROFILE_MAX
};

enum {
	LPWG_NONE = 0,
	LPWG_DOUBLE_TAP,
	LPWG_MULTI_TAP,
	LPWG_ACTIVE_MODE,
};

enum{
    LPWG_READ = 1,
    LPWG_ENABLE,
    LPWG_LCD_X,
    LPWG_LCD_Y,
    LPWG_ACTIVE_AREA_X1,
    LPWG_ACTIVE_AREA_X2,
    LPWG_ACTIVE_AREA_Y1,
    LPWG_ACTIVE_AREA_Y2,
    LPWG_TAP_COUNT,
    LPWG_DOUBLE_TAP_CHECK,
    LPWG_REPLY,
};

enum {
	IC_POWER_OFF = 0,
	IC_POWER_ON,
};
struct point {
    int x;
    int y;
};

struct st_i2c_msgs {
	struct i2c_msg *msg;
	int count;
} i2c_msgs;

struct foo_obj{
	struct kobject kobj;
	int interrupt;
};


struct synaptics_ts_f12_info {
	bool ctrl_reg_is_present[32];
	bool data_reg_is_present[16];
	u8 ctrl_reg_addr[32];
	u8 data_reg_addr[16];
};

static struct foo_obj *foo_obj;
static struct synaptics_ts_f12_info f12_info;


/****************************************************************************
* Variables
****************************************************************************/
static function_descriptor device_control;
static function_descriptor finger;
static function_descriptor button;
static function_descriptor flash_memory;
static function_descriptor multi_tap;


u8 device_control_page;
u8 finger_page;
u8 button_page;
u8 flash_memory_page;
u8 multi_tap_page;

u8 button_enable_mask;
int f54_window_crack;
int f54_window_crack_check_mode;



touch_finger_info pre_touch_info;
char button_prestate[TPD_KEY_COUNT];
char finger_prestate[MAX_NUM_OF_FINGER];

static bool Power_status=0;
static int Power_status_cnt=0;
char knock_on_type;
static int knock_on_enable = 0;

//extern int Touch_Quick_Cover_Closed;
/*knock on/code Parameter*/
u8 double_tap_enable = 0;
u8 multi_tap_enable = 0;
u8 double_tap_check = 0;
u8 multi_tap_count;
u8 lpwg_mode = 0;
u8 screen = 1;
u8 sensor = 1;
u8 qcover = 0;
u8 telepony = 0;
u32 Double_Tap_Area_X1 = 0;
u32 Double_Tap_Area_X2 = 0;
u32 Double_Tap_Area_Y1 = 0;
u32 Double_Tap_Area_Y2 = 0;

u32 lcd_touch_ratio_x;
u32 lcd_touch_ratio_y;
static u8 custom_gesture_status = 0;
static u8 gesture_property[MAX_POINT_SIZE_FOR_LPWG*4+1] = {0};
static struct point lpwg_data[MAX_POINT_SIZE_FOR_LPWG+1];
static char *lpwg_uevent[2][2] = {
	{ "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL },
	{ "TOUCH_GESTURE_WAKEUP=PASSWORD", NULL }
};
struct wake_lock knock_code_lock;
struct hrtimer multi_tap_timer;
struct work_struct multi_tap_work;
struct workqueue_struct* touch_multi_tap_wq;

//ticklewind.kim@lge.com
struct work_struct notify_work;
struct hrtimer notify_timer;

struct workqueue_struct* knock_set_work_wq;
struct delayed_work knock_set_work;

/* ghost finger detection */
int x_max;
int	y_max;
static int pressure_zero = 0;
touch_finger_info old_touch_info;
char button_oldstate[TPD_KEY_COUNT];
char finger_oldstate[MAX_NUM_OF_FINGER];
struct timeval t_ex_debug[TIME_EX_PROFILE_MAX];

#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
char fw_path[256];
unsigned char *fw_start;
unsigned long fw_size;
bool fw_force_update;
bool download_status;		/* avoid power off during F/W upgrade */
bool suspend_status;		/* avoid power off during F/W upgrade */
bool key_lock_status;
struct wake_lock fw_suspend_lock;
struct device *touch_fw_dev;
struct work_struct work_fw_upgrade;
#endif


struct i2c_client *ds4_i2c_client;

static u8* I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;


extern struct tpd_device *tpd;
struct i2c_client *tpd_i2c_client = NULL;
static int tpd_flag = 0;

struct class *touch_class;
struct workqueue_struct*	touch_wq;

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(notify_access);
static DEFINE_MUTEX(knock_access);

static DECLARE_WAIT_QUEUE_HEAD(tpd_waiter);


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern int FirmwareUpgrade ( struct i2c_client *client, const char* fw_path, unsigned long fw_size, unsigned char* fw_start );
extern UINT32 DISP_GetScreenHeight ( void );
extern UINT32 DISP_GetScreenWidth ( void );
extern int mtk_wdt_enable ( enum wk_wdt_en en );
// extern g_qem_check;
EXPORT_SYMBOL(double_tap_enable);
EXPORT_SYMBOL(multi_tap_enable);

extern void arch_reset(char mode, const char *cmd);


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static void touch_eint_interrupt_handler ( void );
static int synaptics_ic_status_check (void );
void synaptics_initialize ( struct i2c_client *client );
int synaptics_ts_read ( struct i2c_client *client, u8 reg, int num, u8 *buf );
int synaptics_ts_write ( struct i2c_client *client, u8 reg, u8 * buf, int len );
#ifdef LGE_USE_SYNAPTICS_FW_UPGRADE
static int synaptics_firmware_update ( struct work_struct *work_fw_upgrade );
#endif
static int synaptics_knock_lpwg ( struct i2c_client* client, u32 code, u32 value, struct point *data );

static int sleep_control(struct i2c_client* client, int mode, int recal);
static int lpwg_control(int mode);
static int sleep_control(struct i2c_client* client, int mode, int recal);
static int tci_control(struct i2c_client *client, int type, u8 value);
static int synaptics_ts_get_data(struct i2c_client *client);
static int get_tci_data(struct i2c_client *client, int count);


static enum hrtimer_restart synaptics_multi_tap_timer_handler ( struct hrtimer *multi_tap_timer );
static void synaptics_multi_tap_work ( struct work_struct *multi_tap_work );


/****************************************************************************
* Platform(AP) dependent functions
****************************************************************************/
static void synaptics_setup_eint ( void )
{
	TPD_FUN ();

	
	/* Configure GPIO settings for external interrupt pin  */
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	msleep(50);

	/* Configure external interrupt settings for external interrupt pin */
	//mt65xx_eint_set_sens ( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE );
	//mt65xx_eint_set_hw_debounce ( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN );
	mt_eint_registration( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, touch_eint_interrupt_handler, 1 );

	/* unmask external interrupt */
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

}


enum 
{
	TOUCH_POWER_OFF = 0,
	TOUCH_POWER_ON,
	TOUCH_SLEEP,
	TOUCH_NO_SLEEP,
};

void synaptics_power ( unsigned int on )
{	
	TPD_FUN ();

	if ( on )
	{
		//hwPowerOn ( MT6323_POWER_LDO_VGP1, VOL_1800, "1V8_MTK_LCD_IO" );
		msleep ( 100 );
	}
	else
	{
		//hwPowerDown ( MT6323_POWER_LDO_VGP1, "1V8_MTK_LCD_IO" );
		msleep ( 10 );
	}

	Power_status = on;
}

/****************************************************************************
* Synaptics I2C  Read / Write Funtions
****************************************************************************/
static int i2c_dma_write ( struct i2c_client *client, const uint8_t *buf, int len )
{
	int i = 0;
	for ( i = 0 ; i < len ; i++ )
	{
		I2CDMABuf_va[i] = buf[i];
	}

	if ( len < 8 )
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		if(download_status == 1)
		{
			client->timing = 400;
		}
		else
		{
			client->timing = 100;
		}
		return i2c_master_send(client, buf, len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		if(download_status == 1)
		{
			client->timing = 400;
		}
		else
		{
			client->timing = 100;
		}
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

static int i2c_dma_read ( struct i2c_client *client, uint8_t *buf, int len )
{
	int i = 0, ret = 0;
	if ( len < 8 )
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_recv(client, buf, len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);
		if ( ret < 0 )
		{
			return ret;
		}

		for ( i = 0 ; i < len ; i++ )
		{
			buf[i] = I2CDMABuf_va[i];
		}
	}

	return ret;
}

static int i2c_msg_transfer ( struct i2c_client *client, struct i2c_msg *msgs, int count )
{
	int i = 0, ret = 0;
	for ( i = 0 ; i < count ; i++ )
	{
		if ( msgs[i].flags & I2C_M_RD )
		{
			ret = i2c_dma_read(client, msgs[i].buf, msgs[i].len);
		}
		else
		{
			ret = i2c_dma_write(client, msgs[i].buf, msgs[i].len);
		}

		if ( ret < 0 )
		{
			return ret;
		}
	}

	return 0;
}

int synaptics_ts_read_f54 ( struct i2c_client *client, u8 reg, int num, u8 *buf )
{
	int message_count = ( ( num - 1 ) / BUFFER_SIZE ) + 2;
	int message_rest_count = num % BUFFER_SIZE;
	int i, data_len;

	if ( i2c_msgs.msg == NULL || i2c_msgs.count < message_count )
	{
		if ( i2c_msgs.msg != NULL )
			kfree(i2c_msgs.msg);

#if 0		
		i2c_msgs.msg = (struct i2c_msg*)kcalloc(message_count, sizeof(struct i2c_msg), GFP_KERNEL);	
#else
		i2c_msgs.msg = (struct i2c_msg*)kcalloc(message_count, sizeof(struct i2c_msg), GFP_KERNEL);
#endif
		i2c_msgs.count = message_count;
		//dev_dbg(&client->dev, "%s: Update message count %d(%d)\n",  __func__, i2c_msgs.count, message_count);
	}

	i2c_msgs.msg[0].addr = client->addr;
	i2c_msgs.msg[0].flags = 0;
	i2c_msgs.msg[0].len = 1;
	i2c_msgs.msg[0].buf = &reg;

	if ( !message_rest_count )
		message_rest_count = BUFFER_SIZE;
	for ( i = 0 ; i < message_count - 1 ; i++ )
	{
		if ( i == message_count - 1 )
			data_len = message_rest_count;
		else
			data_len = BUFFER_SIZE;

		i2c_msgs.msg[i + 1].addr = client->addr;
		i2c_msgs.msg[i + 1].flags = I2C_M_RD;
		i2c_msgs.msg[i + 1].len = data_len;
		i2c_msgs.msg[i + 1].buf = buf + BUFFER_SIZE * i;
	}

#if 1
	if ( i2c_msg_transfer(client, i2c_msgs.msg, message_count) < 0 )
	{
#else
	if (i2c_transfer(client->adapter, i2c_msgs.msg, message_count) < 0) {
#endif
		if ( printk_ratelimit() )
			TPD_ERR ( "transfer error\n" );
		return -EIO;
	}
	else
		return 0;
}
	EXPORT_SYMBOL ( synaptics_ts_read_f54 );

int synaptics_ts_read ( struct i2c_client *client, u8 reg, int num, u8 *buf )
{
	u8 retry = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = num,
			.buf = buf,
		},
	};

	while ( i2c_msg_transfer(client, msgs, 2) < 0 )
	{
		retry++;

		TPD_ERR ( "I2C 0x%X read retry\n", reg );

		if ( retry == 3 )
		{
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL ( synaptics_ts_read );

int synaptics_ts_write ( struct i2c_client *client, u8 reg, u8 * buf, int len )
{
	u8 retry = 0;

	unsigned char send_buf[len + 1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = len+1,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char) reg;
	memcpy(&send_buf[1], buf, len);

	while ( i2c_msg_transfer(client, msgs, 1) < 0 )
	{
		retry++;

		TPD_ERR ( "I2C 0x%X write retry\n", reg );

		if ( retry == 3 )
		{
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL ( synaptics_ts_write );

int synaptics_page_data_read ( struct i2c_client *client, u8 page, u8 reg, int size, u8 *data )
{
	int ret = 0;

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, &page, 1 );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	ret = synaptics_ts_read ( client, reg, size, data );
	if ( ret < 0 )
	{
		TPD_ERR ( "synaptics_page_data_read fail\n" );
		return ret;
	}

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, &device_control_page, 1 );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	return 0;
}

int synaptics_page_data_write ( struct i2c_client *client, u8 page, u8 reg, int size, u8 *data )
{
	int ret = 0;

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, &page, 1 );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	ret = synaptics_ts_write ( client, reg, data, size );
	if ( ret < 0 )
	{
		TPD_ERR ( "synaptics_page_data_write fail\n" );
		return ret;
	}

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, &device_control_page, 1 );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	return 0;
}

/****************************************************************************
* Touch malfunction Prevention Function
****************************************************************************/
static void synaptics_release_all_finger ( void )
{
	TPD_FUN ();
	unsigned int finger_count=0;

	/* Reset finger position data */
	memset(&old_touch_info, 0x0, sizeof(touch_finger_info));
	memset(&pre_touch_info, 0x0, sizeof(touch_finger_info));

	/* Reset finger & button status data */
	memset(finger_oldstate, 0x0, sizeof(char) * MAX_NUM_OF_FINGER);
	memset(button_oldstate, 0x0, sizeof(char) * TPD_KEY_COUNT);
	memset(finger_prestate, 0x0, sizeof(char) * MAX_NUM_OF_FINGER);
	memset(button_prestate, 0x0, sizeof(char) * TPD_KEY_COUNT);

#if defined(MT_PROTOCOL_A)
	input_mt_sync ( tpd->dev );
#else
	for(finger_count = 0; finger_count < MAX_NUM_OF_FINGER; finger_count++)
	{
		input_mt_slot( tpd->dev, finger_count);
		input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
	}
#endif
	input_sync ( tpd->dev );
}


void get_f12_info(struct i2c_client *client)
{
	int retval;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	int i;
	u8 offset;

	retval = synaptics_ts_read(client, finger.query_base+ 5, sizeof(query_5.data), query_5.data);
	
	if (retval < 0) {
			TPD_LOG("Failed to read from F12_2D_QUERY_05_Control_Presence register\n");
			return;
		}

	f12_info.ctrl_reg_is_present[0] = query_5.ctrl_00_is_present;
	f12_info.ctrl_reg_is_present[1] = query_5.ctrl_01_is_present;
	f12_info.ctrl_reg_is_present[2] = query_5.ctrl_02_is_present;
	f12_info.ctrl_reg_is_present[3] = query_5.ctrl_03_is_present;
	f12_info.ctrl_reg_is_present[4] = query_5.ctrl_04_is_present;
	f12_info.ctrl_reg_is_present[5] = query_5.ctrl_05_is_present;
	f12_info.ctrl_reg_is_present[6] = query_5.ctrl_06_is_present;
	f12_info.ctrl_reg_is_present[7] = query_5.ctrl_07_is_present;
	f12_info.ctrl_reg_is_present[8] = query_5.ctrl_08_is_present;
	f12_info.ctrl_reg_is_present[9] = query_5.ctrl_09_is_present;
	f12_info.ctrl_reg_is_present[10] = query_5.ctrl_10_is_present;
	f12_info.ctrl_reg_is_present[11] = query_5.ctrl_11_is_present;
	f12_info.ctrl_reg_is_present[12] = query_5.ctrl_12_is_present;
	f12_info.ctrl_reg_is_present[13] = query_5.ctrl_13_is_present;
	f12_info.ctrl_reg_is_present[14] = query_5.ctrl_14_is_present;
	f12_info.ctrl_reg_is_present[15] = query_5.ctrl_15_is_present;
	f12_info.ctrl_reg_is_present[16] = query_5.ctrl_16_is_present;
	f12_info.ctrl_reg_is_present[17] = query_5.ctrl_17_is_present;
	f12_info.ctrl_reg_is_present[18] = query_5.ctrl_18_is_present;
	f12_info.ctrl_reg_is_present[19] = query_5.ctrl_19_is_present;
	f12_info.ctrl_reg_is_present[20] = query_5.ctrl_20_is_present;
	f12_info.ctrl_reg_is_present[21] = query_5.ctrl_21_is_present;
	f12_info.ctrl_reg_is_present[22] = query_5.ctrl_22_is_present;
	f12_info.ctrl_reg_is_present[23] = query_5.ctrl_23_is_present;
	f12_info.ctrl_reg_is_present[24] = query_5.ctrl_24_is_present;
	f12_info.ctrl_reg_is_present[25] = query_5.ctrl_25_is_present;
	f12_info.ctrl_reg_is_present[26] = query_5.ctrl_26_is_present;
	f12_info.ctrl_reg_is_present[27] = query_5.ctrl_27_is_present;
	f12_info.ctrl_reg_is_present[28] = query_5.ctrl_28_is_present;
	f12_info.ctrl_reg_is_present[29] = query_5.ctrl_29_is_present;
	f12_info.ctrl_reg_is_present[30] = query_5.ctrl_30_is_present;
	f12_info.ctrl_reg_is_present[31] = query_5.ctrl_31_is_present;

	offset = 0;
	
	for (i = 0; i < 32; i++) {
			f12_info.ctrl_reg_addr[i] = finger.control_base + offset;
	
			if (f12_info.ctrl_reg_is_present[i])
				offset++;
		}
	/* data_reg_info setting */

	
	retval = synaptics_ts_read(client, (finger.query_base + 8), sizeof(query_8.data), query_8.data);

	if (retval < 0) {
		TPD_LOG("Failed to read from F12_2D_QUERY_08_Data_Presence register\n");
		return;
	}
	
	f12_info.data_reg_is_present[0] = query_8.data_00_is_present;
	f12_info.data_reg_is_present[1] = query_8.data_01_is_present;
	f12_info.data_reg_is_present[2] = query_8.data_02_is_present;
	f12_info.data_reg_is_present[3] = query_8.data_03_is_present;
	f12_info.data_reg_is_present[4] = query_8.data_04_is_present;
	f12_info.data_reg_is_present[5] = query_8.data_05_is_present;
	f12_info.data_reg_is_present[6] = query_8.data_06_is_present;
	f12_info.data_reg_is_present[7] = query_8.data_07_is_present;
	f12_info.data_reg_is_present[8] = query_8.data_08_is_present;
	f12_info.data_reg_is_present[9] = query_8.data_09_is_present;
	f12_info.data_reg_is_present[10] = query_8.data_10_is_present;
	f12_info.data_reg_is_present[11] = query_8.data_11_is_present;
	f12_info.data_reg_is_present[12] = query_8.data_12_is_present;
	f12_info.data_reg_is_present[13] = query_8.data_13_is_present;
	f12_info.data_reg_is_present[14] = query_8.data_14_is_present;
	f12_info.data_reg_is_present[15] = query_8.data_15_is_present;


	offset = 0;

	for (i = 0; i < 16; i++) {
		f12_info.data_reg_addr[i] = finger.data_base + offset;

		if (f12_info.data_reg_is_present[i])
			offset++;
	}
	
	/* print info */
		for (i = 0; i < 32; i++) {
			if (f12_info.ctrl_reg_is_present[i])
				TPD_LOG("f12_info.ctrl_reg_addr[%d]=0x%02X\n", i, f12_info.ctrl_reg_addr[i]);
		}
	
		for (i = 0; i < 16; i++) {
			if (f12_info.data_reg_is_present[i])
				TPD_LOG("f12_info.data_reg_addr[%d]=0x%02X\n", i, f12_info.data_reg_addr[i]);
		}
		return;



}


static void synaptics_ic_reset ( void )
{
	TPD_FUN ();

	mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );

	msleep ( 20 );
	mt_set_gpio_out ( GPIO_CTP_RST_PIN, GPIO_OUT_ZERO );
	msleep ( 10 );
	mt_set_gpio_out ( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
	msleep ( 100 );

	mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
}

static int synaptics_ic_status_check ( void )
{
	int ret;
	u8 device_status = 0;
	u8 temp = 0;

	/* read device status */
	 	
	ret = synaptics_ts_read ( tpd_i2c_client, DEVICE_STATUS_REG, 1, (u8 *) &device_status );
	if ( ret < 0 )
	{
		TPD_ERR ( "DEVICE_STATUS_REG read fail\n" );
		return -1;
	}

	/* ESD damage check */
	if ( ( device_status & DEVICE_FAILURE_MASK ) == DEVICE_FAILURE_MASK )
	{
		TPD_ERR ( "ESD damage occured. Reset Touch IC\n" );
		synaptics_ic_reset ();
		synaptics_initialize ( tpd_i2c_client );
		return -1;
	}

	/* Internal reset check */
	if ( ( ( device_status & UNCONFIGURED_MASK ) >> 7 ) == 1 )
	{
		TPD_ERR ( "Touch IC resetted internally. Reconfigure register setting\n");
		synaptics_initialize ( tpd_i2c_client );
		return -1;
	}

	return 0;
}

void touch_keylock_enable ( int key_lock )
{
	TPD_FUN ();

	if ( !key_lock )
	{
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
		key_lock_status = 0;
	}
	else
	{
		mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
		key_lock_status = 1;
	}
}
EXPORT_SYMBOL ( touch_keylock_enable );

/****************************************************************************
* Synaptics Knock_On Funtions
****************************************************************************/
static enum hrtimer_restart synaptics_multi_tap_timer_handler ( struct hrtimer *multi_tap_timer )
{
	TPD_FUN ();
	queue_work ( touch_multi_tap_wq, &multi_tap_work );
	return HRTIMER_NORESTART;
}

static void synaptics_multi_tap_work ( struct work_struct *multi_tap_work )
{
	TPD_FUN ();
	u8 r_mem = 0;

	tci_control(tpd_i2c_client,REPORT_MODE_CTRL,1);//wakeup gesture only
	DO_SAFE(sleep_control(tpd_i2c_client,0,0),error);
	/* uevent report */
	kobject_uevent_env ( &foo_obj->kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_MULTI_TAP-1] );

	error:
		return;
}
static int get_tci_data(struct i2c_client * client,int count)
{
	TPD_FUN();
	u8 i=0;

	if(!count)
	{
		TPD_ERR("Knock count = 0");
		return 0;
	}
	else
	{	
		TPD_LOG("Knock count = %d", count);
	}

	DO_SAFE(synaptics_page_data_read(client,LPWG_PAGE,LPWG_DATA_REG,4*count,&gesture_property),error);
	/*
	if(custom_gesture_status)
	{
		for(i=0; i<count; i++)
		{
			TPD_LOG ( "lpwg data %d: [0:0x%-4x 1:0x%-4x 2:0x%-4x 3:0x%-4x]\n", i, gesture_property[4*i], gesture_property[4*i+1], gesture_property[4*i+2], gesture_property[4*i+3] );
		}
	}
	*/
	return 0;
	error:
		return -1;
}

static int synaptics_ts_get_data(struct i2c_client *client)
{
		TPD_FUN();
	u8	i = 0;
	u8  device_status = 0;
	u8  gesture_status = 0;

	DO_SAFE(synaptics_ts_read(client, DEVICE_STATUS_REG, 1 , &device_status), error);
	//knock code => DEVICE_STATUS_REG = 0
	DO_IF((device_status & DEVICE_FAILURE_MASK) == DEVICE_FAILURE_MASK, error);

	DO_SAFE(synaptics_page_data_read(client, LPWG_PAGE, LPWG_STATUS_REG, 1, &gesture_status), error);
	TPD_LOG ( "LPWG_STATUS_REG = 0x%x", gesture_status );
	custom_gesture_status = gesture_status;

	if ( gesture_status & 0x1 )
	{  
		/* TCI-1 Double-Tap */
	    if ( double_tap_enable )
		{
			get_tci_data ( client, 2 );
			/* uevent report */
			kobject_uevent_env ( &foo_obj->kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_DOUBLE_TAP-1] );
			TPD_LOG ("Knock ON Occured!!\n");
	   	}
	}
	else if ( gesture_status & 0x2 )
	{
		/* TCI-2 Multi-Tap */
	    if ( multi_tap_enable )
		{
			get_tci_data ( client, multi_tap_count );
			wake_lock ( &knock_code_lock );
			tci_control ( client, REPORT_MODE_CTRL, 0 );

			hrtimer_try_to_cancel ( &multi_tap_timer );
			if ( !hrtimer_callback_running ( &multi_tap_timer ) )
			{
				TPD_LOG ("timer check\n");
				hrtimer_start ( &multi_tap_timer, ktime_set(0, MS_TO_NS(200)), HRTIMER_MODE_REL );
			}
    	}
	} 
	else 
	{
	   	TPD_ERR ( "Ignore interrupt [double_tap_enable=%d, multi_tap_enable=%d, gesture_status=0x%02x]\n", double_tap_enable, multi_tap_enable, gesture_status );
		goto error;
	}

	return 0;

error :
	return -1;
}


static int sleep_control(struct i2c_client* client, int mode, int recal)
{
	u8 curr = 0;
	u8 next = 0;

	/*
     * NORMAL == 0 : resume & lpwg state
	 * SLEEP  == 1 : uevent reporting time - sleep
	 * NO_CAL == 2 : proxi near - sleep when recal is not needed
     */
	DO_SAFE(synaptics_ts_read(client, DEVICE_CONTROL_REG, 1, &curr), error);

	TPD_LOG("CURR = %d",curr);
    next = (curr & 0xFC) | mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP;
			// (recal ? DEVICE_CONTROL_SLEEP : DEVICE_CONTROL_SLEEP_NO_RECAL);
	TPD_LOG("NEXT = %d", next);

    TPD_LOG( "%s : current = [%6s] => next [%6s]\n", "sleep_control",
		(curr == 0 ? "NORMAL" : (curr == 1 ? "SLEEP" : "NO_CAL")),
		(next == 0 ? "NORMAL" : (next == 1 ? "SLEEP" : "NO_CAL")));

	if ( curr != next )
	{
    DO_SAFE(synaptics_ts_write(client, DEVICE_CONTROL_REG, &next,1), error);
	}
	Power_status = (next == 0 ? TOUCH_NO_SLEEP : TOUCH_SLEEP) ;
	return 0;
error:
	return -1;

}



static int tci_control(struct i2c_client *client, int type, u8 value)
{
	
	
		u8 buffer[3] = {0};
		u8 temp = 0;
		u8 page = 0;
	
		switch ( type )
		{
			case REPORT_MODE_CTRL:
				page = device_control_page;
				DO_SAFE(synaptics_ts_write(client, PAGE_SELECT_REG, &page, 1), error);
				DO_SAFE(synaptics_ts_read(client, INTERRUPT_ENABLE_REG, 1, buffer), error);
				temp = value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0;
				DO_SAFE(synaptics_ts_write(client, INTERRUPT_ENABLE_REG, &temp,1), error);
				TPD_LOG("%s -> %s, Interrupt ENABLE %s",(value ? "normal" : "wakeup_gesure_only"),(value ? "wakeup_gesture_only" : "normal"),(value ? "only ABS masking(0x7B)" : "only ABS unmaking(0x04)"));
	
				DO_SAFE(synaptics_ts_read(client, f12_info.ctrl_reg_addr[20], 3, buffer), error);	
				buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
				DO_SAFE(synaptics_ts_write(client, f12_info.ctrl_reg_addr[20],buffer,3), error);
				break;
	
			case TCI_ENABLE_CTRL:
				page = LPWG_PAGE;
				DO_SAFE(synaptics_ts_write(client, PAGE_SELECT_REG, &page, 1), error);
				DO_SAFE(synaptics_ts_read(client, LPWG_TAPCOUNT_REG, 1, buffer), error);
				buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
				DO_SAFE(synaptics_ts_write(client, LPWG_TAPCOUNT_REG, buffer, 1), error);
				break;
	
			case TCI_ENABLE_CTRL2:
				DO_SAFE(synaptics_ts_read(client, LPWG_TAPCOUNT_REG2, 1, buffer), error);
				buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
				DO_SAFE(synaptics_ts_write(client, LPWG_TAPCOUNT_REG2, buffer, 1), error);
				break;
	
			case TAP_COUNT_CTRL:
				DO_SAFE(synaptics_ts_read(client, LPWG_TAPCOUNT_REG, 1, buffer), error);
				buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
				DO_SAFE(synaptics_ts_write(client, LPWG_TAPCOUNT_REG, buffer, 1), error);
				break;
	
			case TAP_COUNT_CTRL2:
				DO_SAFE(synaptics_ts_read(client, LPWG_TAPCOUNT_REG2, 1, buffer), error);
				buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
				DO_SAFE(synaptics_ts_write(client, LPWG_TAPCOUNT_REG2, buffer, 1), error);
				break;
	
			case MIN_INTERTAP_CTRL:
				DO_SAFE(synaptics_ts_write(client, LPWG_MIN_INTERTAP_REG, &value, 1), error);
				break;
	
			case MIN_INTERTAP_CTRL2:
				DO_SAFE(synaptics_ts_write(client, LPWG_MIN_INTERTAP_REG2, &value, 1), error);
				break;
	
			case MAX_INTERTAP_CTRL:
				DO_SAFE(synaptics_ts_write(client, LPWG_MAX_INTERTAP_REG, &value, 1), error);
				break;
	
			case MAX_INTERTAP_CTRL2:
				DO_SAFE(synaptics_ts_write(client, LPWG_MAX_INTERTAP_REG2, &value, 1), error);
				break;
	
			case TOUCH_SLOP_CTRL:
				DO_SAFE(synaptics_ts_write(client, LPWG_TOUCH_SLOP_REG, &value, 1), error);
				break;
	
			case TOUCH_SLOP_CTRL2:
				DO_SAFE(synaptics_ts_write(client, LPWG_TOUCH_SLOP_REG2, &value, 1), error);
				break;
	
			case TAP_DISTANCE_CTRL:
				DO_SAFE(synaptics_ts_write(client, LPWG_TAP_DISTANCE_REG, &value, 1), error);
				break;
	
			case TAP_DISTANCE_CTRL2:
				DO_SAFE(synaptics_ts_write(client, LPWG_TAP_DISTANCE_REG2, &value, 1), error);
				break;
	
			case INTERRUPT_DELAY_CTRL:
				temp = value ? ((KNOCKON_DELAY << 1) | 0x1) : 0;
				DO_SAFE(synaptics_ts_write(client, LPWG_INTERRUPT_DELAY_REG, &temp, 1), error);
				break;
	
			case INTERRUPT_DELAY_CTRL2:
				temp = value ? ((KNOCKON_DELAY << 1) | 0x1) : 0;
				DO_SAFE(synaptics_ts_write(client, LPWG_INTERRUPT_DELAY_REG2, &temp, 1), error);
				break;
	
			default:
				break;
		}
	
	error:
		return -1;
	

}


static int lpwg_control(int mode)
{
	TPD_FUN();
	TPD_LOG("lpwg_control MODE = %d", mode);
		switch (mode) {
		case LPWG_DOUBLE_TAP:	
			TPD_LOG("DOUBLE TAP REGISTER SETTING");
			
			tci_control(tpd_i2c_client, TCI_ENABLE_CTRL, 1);		// tci enable
			tci_control(tpd_i2c_client, TAP_COUNT_CTRL, 2); 		// tap count = 2
			tci_control(tpd_i2c_client, MIN_INTERTAP_CTRL, 0);		// min inter_tap = 0
			tci_control(tpd_i2c_client, MAX_INTERTAP_CTRL, 70); 	// max inter_tap = 700ms
			tci_control(tpd_i2c_client, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
			tci_control(tpd_i2c_client, TAP_DISTANCE_CTRL, 10); 	// tap distance = 10mm
			tci_control(tpd_i2c_client, INTERRUPT_DELAY_CTRL, 0);	// interrupt delay = 0ms
			tci_control(tpd_i2c_client, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
			tci_control(tpd_i2c_client, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only

				break;
		case LPWG_MULTI_TAP:	
			TPD_LOG("MULTI TAP REGISTER SETTING");	// TCI-1 and TCI-2
			tci_control(tpd_i2c_client, TCI_ENABLE_CTRL, 1);		// tci-1 enable
			tci_control(tpd_i2c_client, TAP_COUNT_CTRL, 2); 		// tap count = 2
			tci_control(tpd_i2c_client, MIN_INTERTAP_CTRL, 0);		// min inter_tap = 0
			tci_control(tpd_i2c_client, MAX_INTERTAP_CTRL, 70); 	// max inter_tap = 700ms
			tci_control(tpd_i2c_client, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
			tci_control(tpd_i2c_client, TAP_DISTANCE_CTRL, 7);		// tap distance = 7mm
			tci_control(tpd_i2c_client, INTERRUPT_DELAY_CTRL, double_tap_check);	// interrupt delay = 0ms
	
	
			tci_control(tpd_i2c_client, TCI_ENABLE_CTRL2, 1);		// tci-2 enable
			tci_control(tpd_i2c_client, TAP_COUNT_CTRL2, multi_tap_count); // tap count = "user_setting"			/*temp*/
			tci_control(tpd_i2c_client, MIN_INTERTAP_CTRL2, 0); 	// min inter_tap = 0
			tci_control(tpd_i2c_client, MAX_INTERTAP_CTRL2, 70);	// max inter_tap = 700ms
			tci_control(tpd_i2c_client, TOUCH_SLOP_CTRL2, 100); 	// touch_slop = 10mm
			tci_control(tpd_i2c_client, TAP_DISTANCE_CTRL2, 255);	// tap distance = MAX
			tci_control(tpd_i2c_client, INTERRUPT_DELAY_CTRL2, 0);	// interrupt delay = 0ms
			tci_control(tpd_i2c_client, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only
				break;
		default:
			TPD_LOG("IDLE REGISTER SETTING");
			tci_control(tpd_i2c_client, TCI_ENABLE_CTRL, 0);		// tci-1 disable
			tci_control(tpd_i2c_client, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
			tci_control(tpd_i2c_client, REPORT_MODE_CTRL, 0);		// normal
			sleep_control(tpd_i2c_client, 1, 0);
				break;
			}
	
		return 0;

}

static int synaptics_knock_lpwg ( struct i2c_client* client, u32 code, u32 value, struct point *data )
{
	TPD_FUN();
	u8 buf = 0;
	int i = 0, ret = 0;
	
	switch ( code )
	{
		case LPWG_READ:
			if ( multi_tap_enable )
			{
				if ( custom_gesture_status == 0 )
				{
					data[0].x = 1;
					data[0].y = 1;
					data[1].x = -1;
					data[1].y = -1;
					break;
				}

				for ( i = 0 ; i < multi_tap_count ; i++ )
				{
					data[i].x = TS_SNTS_GET_X_POSITION(gesture_property[4*i+1],gesture_property[4*i]);
					data[i].y = TS_SNTS_GET_Y_POSITION(gesture_property[4*i+3],gesture_property[4*i+2]);
					TPD_LOG ( "TAP Position x[%3d], y[%3d]\n", data[i].x, data[i].y );
					// '-1' should be assinged to the last data.
					// Each data should be converted to LCD-resolution.
				}
				data[i].x = -1;
				data[i].y = -1;
			}
			break;

		case LPWG_ENABLE:
			break;

		case LPWG_LCD_X:
			lcd_touch_ratio_x = x_max / value;
			TPD_LOG ( "LPWG GET X LCD INFO = %d , RATIO_X = %d", value, lcd_touch_ratio_x );
			break;

		case LPWG_LCD_Y:
			// If touch-resolution is not same with LCD-resolution,
			// position-data should be converted to LCD-resolution.
			lcd_touch_ratio_y = y_max / value;
			TPD_LOG ( "LPWG GET Y LCD INFO = %d , RATIO_Y = %d", value, lcd_touch_ratio_y );
			break;

		case LPWG_ACTIVE_AREA_X1:
			break;
		case LPWG_ACTIVE_AREA_X2:
			break;
		case LPWG_ACTIVE_AREA_Y1:
			break;
		case LPWG_ACTIVE_AREA_Y2:
			// Quick Cover Area ==> will modify
			break;

		case LPWG_TAP_COUNT:
			if ( value )
			{
				multi_tap_count = value;
				case_mode = 2;
				tci_control(client,TAP_COUNT_CTRL2,multi_tap_count);
			}
			else
			{
				multi_tap_count = 2;
				tci_control(client,TAP_COUNT_CTRL2,2);
				case_mode = 1;
			}
			break;
		case LPWG_DOUBLE_TAP_CHECK:
			TPD_LOG ( "LPWG_DOUBLE_TAP_CHECK = %d\n", value);
			double_tap_check = value;
			if (multi_tap_enable) {
				tci_control(client, INTERRUPT_DELAY_CTRL, value);
			}
			break;
		case LPWG_REPLY:
			// Do something, if you need.
			ret = synaptics_ts_read ( client, DEVICE_CONTROL_REG, 1, &buf );
			if ( ret < 0 )
			{
				TPD_ERR ( "DEVICE_CONTROL_REG read fail\n" );
				return -1;
			}
			buf = ( buf & 0xFC ) | DEVICE_CONTROL_NORMAL_OP;
			ret = synaptics_ts_write ( client, DEVICE_CONTROL_REG, &buf, 1 );
			if ( ret < 0 )
			{
				TPD_ERR ( "DEVICE_CONTROL_REG write fail\n" );
				return -1;
			}
			break;

		default:
			break;
	}

	return 0;
}

static int synaptics_set_qcover_area(u32 X1, u32 X2, u32 Y1, u32 Y2)
{
	TPD_FUN ();
	int ret = 0;
	u8 buffer[50] = {0};
	int i=0;
	u8 doubleTap_area_reg_addr = f12_info.ctrl_reg_addr[18];
	u32 value[4] = {X1,X2,Y1,Y2};
	buffer[0] = value[0];
	buffer[1] = value[0]>>8;
	buffer[2] = value[2];
	buffer[3] = value[2]>>8;
	buffer[4] = value[1];
	buffer[5] = value[1]>>8;
	buffer[6] = value[3];
	buffer[7] = value[3]>>8;
	synaptics_page_data_write(tpd_i2c_client, device_control_page, doubleTap_area_reg_addr, 8, buffer);
	return ret;
}
static int synaptics_lpwg_update_all( struct delayed_work *knock_set_work )
{
	TPD_FUN();

	bool req_lpwg_param = false;
	int sleep_status = 0;
	int lpwg_status = 0;
	u8 interrupt_status = 0;
	u8 finger_threshold = 0;

	TPD_LOG("===================MODE CHANGE SETTING===================");

	if(suspend_status == 1)
	{
		synaptics_ic_reset();
		synaptics_initialize ( tpd_i2c_client );
/*
		synaptics_page_data_read(tpd_i2c_client,0x00,0x1a,1, &finger_threshold);
		TPD_LOG("FINGER THRESHOLD idle = %d",finger_threshold);
		finger_threshold = 0x5b;
		synaptics_page_data_write ( tpd_i2c_client, 0x00, 0x1a, 1, &finger_threshold );
		synaptics_page_data_read(tpd_i2c_client,0x00,0x1a,1, &finger_threshold);
		TPD_LOG("FINGER THRESHOLD suspend = %d",finger_threshold);
*/		
	}
	//lpwg_mode = 1, screen = 0, sensor = 1, qcover = 0
	//ther is no case qcover 0
	
	if(screen == 1)//idle status
	{
		TPD_LOG("LCD = ON idle SETTING");
		sleep_status = 1;
		lpwg_status  = 0;
	}
	else if(screen == 0 && qcover == 1)// only knock on when qcover close up (knock code is disable)
	{
		TPD_LOG("LCD = OFF, QCOVER = CLOSED");
		sleep_status = 1;
		lpwg_status = 1; 
		synaptics_set_qcover_area(Double_Tap_Area_X1, Double_Tap_Area_X2, Double_Tap_Area_Y1, Double_Tap_Area_Y2);//Set Knock on/code area when qcover is closed
	}
	
	else if(screen == 0 && qcover == 0 &&sensor == 1)//can use knock on/code when hand close up
	{
		TPD_LOG("LCD = OFF, QCOVER = OPEN, SENSOR = FAR, LPWG_MODE = %s", lpwg_mode == 1 ? "double tap mode" : (lpwg_mode == 2 ? "multi tap mode" : (lpwg_mode == 0 ? "Idle mode" : "None")));
		sleep_status = 1;
		lpwg_status = lpwg_mode;
	}
	else if(screen == 0 && qcover == 0 && sensor == 0)// but not use knock on/code when sensor is near
	{
		TPD_LOG("LCD = OFF, QCOVER = OPEN, SENSOR = NEAR");
		sleep_status = 0;
		lpwg_status = case_mode;
		//req_lpwg_param = true;
	}
	DO_SAFE(sleep_control(tpd_i2c_client, sleep_status, 0), error);

	if(req_lpwg_param == false)
	{
		DO_SAFE(lpwg_control(lpwg_status), error);
		// this 0
	}
	else
	{
		//set knock parameter
	}	
	
	if ( key_lock_status == 0 )
	{
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	}
	else
	{
		mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
	}
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		return 0; 

	error : 
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
		return -1;

}



/****************************************************************************
* Touch Interrupt Service Routines
****************************************************************************/
static void touch_eint_interrupt_handler ( void )
{
	mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
	tpd_flag = 1;
	wake_up_interruptible ( &tpd_waiter );
}

static int synaptics_touch_event_handler ( void *unused )
{
	int ret = 0;
	u8 int_status;
	u8 int_enable;
	u8 finger_count, button_count;
	u8 reg_num, finger_order;
	int width_min = 0, width_max = 0;
	int width_orientation;
	int index;
	touch_sensor_data sensor_data;
	touch_finger_info finger_info;
	char report_enable = 0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 

	sched_setscheduler ( current, SCHED_RR, &param );

	do
	{
		set_current_state ( TASK_INTERRUPTIBLE );
		wait_event_interruptible ( tpd_waiter, tpd_flag != 0 );

		tpd_flag = 0;
		set_current_state ( TASK_RUNNING );

		mutex_lock ( &i2c_access );




		index = 0;
		memset(&sensor_data, 0x0, sizeof(touch_sensor_data));
		memset(&finger_info, 0x0, sizeof(touch_finger_info));
		pressure_zero = 0;

		ret = synaptics_ic_status_check ();
		if ( ret != 0 )
		{
			TPD_LOG("ignore ic status ");
			goto exit_work;
		}
		/* read interrupt information */	
		ret = synaptics_ts_read ( tpd_i2c_client, INTERRUPT_STATUS_REG, 1, (u8 *) &int_status );
		if ( ret < 0 )
		{
			TPD_ERR ( "INTERRUPT_STATUS_REG read fail\n" );
			goto exit_work;
		}
		if(int_status == 0)
		{
			TPD_LOG("KNOCK ISSUE ");
			//goto exit_work;
		}
		
		/* Knock On or Knock Code Check */

		if ( suspend_status && knock_on_enable )
		{
			if ( download_status == 1 )
			{
				TPD_LOG ( "knock_on is not checked (F/W downloading...)\n" );
			}
			else
			{
				ret = synaptics_ts_get_data(tpd_i2c_client);
				if ( ret != 0 )
				{
					TPD_LOG ( "Touch Knock_On fail\n" );
				}
			}

			goto exit_work;
		}

		ret = synaptics_ts_read ( tpd_i2c_client, INTERRUPT_ENABLE_REG, 1, (u8 *) &int_enable );
		if ( ret < 0 )
		{
			TPD_ERR ( "INTERRUPT_ENABLE_REG read fail\n" );
			goto exit_work;
		}

		/* read finger state & finger data */
		ret = synaptics_ts_read ( tpd_i2c_client, TWO_D_FINGER_STATE_REG, sizeof(sensor_data), (u8 *) &sensor_data.finger_data[0] );
		if (ret < 0)
		{
			TPD_ERR ( "TWO_D_FINGER_STATE_REG read fail\n" );
			goto exit_work;
		}

		/* Finger Event Processing */
		if ( ( int_status & ABS0_MASK ) && ( int_enable & ABS0_MASK ) )
		{
			for ( finger_count = 0 ; finger_count < MAX_NUM_OF_FINGER ; finger_count++ )
			{
				if (sensor_data.finger_data[finger_count][0] == F12_FINGER_STATUS
					|| sensor_data.finger_data[finger_count][0] == F12_STYLUS_STATUS
					/*
					|| sensor_data.finger_data[finger_count][0] == F12_PALM_STATUS
					|| sensor_data.finger_data[finger_count][0] == F12_GLOVED_FINGER_STATUS*/
					)
				{
					finger_info.pos_x[finger_count] = TS_SNTS_GET_X_POSITION(sensor_data.finger_data[finger_count][X_POSITION_MSB], sensor_data.finger_data[finger_count][X_POSITION_LSB]);
					finger_info.pos_y[finger_count] = TS_SNTS_GET_Y_POSITION(sensor_data.finger_data[finger_count][Y_POSITION_MSB], sensor_data.finger_data[finger_count][Y_POSITION_LSB]);

					width_max = TS_SNTS_GET_WIDTH_MAJOR( sensor_data.finger_data[finger_count][WX_VALUE], sensor_data.finger_data[finger_count][WY_VALUE]);
					width_min = TS_SNTS_GET_WIDTH_MINOR( sensor_data.finger_data[finger_count][WX_VALUE], sensor_data.finger_data[finger_count][WY_VALUE]);
					width_orientation = TS_SNTS_GET_ORIENTATION( sensor_data.finger_data[finger_count][WX_VALUE], sensor_data.finger_data[finger_count][WY_VALUE]);

					finger_info.pressure[finger_count] = TS_SNTS_GET_PRESSURE(sensor_data.finger_data[finger_count][PRESSURE]);
					if(qcover == 1)//726 1606
					{	//if (c_data->raw_x < X1 || c_data->raw_x > X2 || c_data->raw_y < Y1 || c_data->raw_y > Y2)
						if(finger_info.pos_x[finger_count] < Double_Tap_Area_X1|| finger_info.pos_x[finger_count] > Double_Tap_Area_X2 
							|| finger_info.pos_y[finger_count] < Double_Tap_Area_Y1 || finger_info.pos_y[finger_count] > Double_Tap_Area_Y2 )
						{	 
							continue;
						}	
					}
					/*
					if(Touch_Quick_Cover_Closed ==1)
					{	//if (c_data->raw_x < X1 || c_data->raw_x > X2 || c_data->raw_y < Y1 || c_data->raw_y > Y2)
						if(finger_info.pos_x[finger_count] < 150 || finger_info.pos_x[finger_count] > 930 
							|| finger_info.pos_y[finger_count] < 38 || finger_info.pos_y[finger_count] > 818 )
						{	 
							TPD_LOG("TOUCH QUICK COVER CLOSED");
							continue;
						}	
					}
					*/
#if !defined(MT_PROTOCOL_A)
					input_mt_slot( tpd->dev, finger_count);
					input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, true);
#endif
					input_report_abs ( tpd->dev, ABS_MT_POSITION_X, finger_info.pos_x[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_POSITION_Y, finger_info.pos_y[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_PRESSURE, finger_info.pressure[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_WIDTH_MAJOR, width_max );
					input_report_abs ( tpd->dev, ABS_MT_WIDTH_MINOR, width_min );
					input_report_abs ( tpd->dev, ABS_MT_ORIENTATION, width_orientation );

					input_report_abs ( tpd->dev, ABS_MT_TRACKING_ID, finger_count );

					report_enable = 1;

					old_touch_info.pos_x[finger_count] = pre_touch_info.pos_x[finger_count];
					old_touch_info.pos_y[finger_count] = pre_touch_info.pos_y[finger_count];
					old_touch_info.pressure[finger_count]= pre_touch_info.pressure[finger_count];
					pre_touch_info.pos_x[finger_count] = finger_info.pos_x[finger_count];
					pre_touch_info.pos_y[finger_count] = finger_info.pos_y[finger_count];
					pre_touch_info.pressure[finger_count] = finger_info.pressure[finger_count];
					index++;

					/* ignore Key event during Finger event processing */
					if ( finger_prestate[finger_count] == TOUCH_RELEASED )
					{
						finger_oldstate[finger_count] = finger_prestate[finger_count];
						finger_prestate[finger_count] = TOUCH_PRESSED;
						TPD_LOG ( "%d key is %s ( x/y = %d, %d)\n", finger_count, finger_prestate[finger_count] ? "pressed" : "released", 
						pre_touch_info.pos_x[finger_count], pre_touch_info.pos_y[finger_count] );
					}
				}
				else if ( finger_prestate[finger_count] == TOUCH_PRESSED )
				{
#if !defined(MT_PROTOCOL_A)
					input_mt_slot(tpd->dev, finger_count);
					input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
#endif
					finger_oldstate[finger_count] = finger_prestate[finger_count];
					finger_prestate[finger_count] = TOUCH_RELEASED;
					TPD_LOG ( "%d key is %s ( x/y =                    %d, %d)\n", finger_count,finger_prestate[finger_count] ? "pressed" : "released", 
						pre_touch_info.pos_x[finger_count], pre_touch_info.pos_y[finger_count] );
					report_enable = 1;

					pre_touch_info.pos_x[finger_count] = 0;
					pre_touch_info.pos_y[finger_count] = 0;
				}

				if ( report_enable )
				{
#if defined(MT_PROTOCOL_A)
					input_mt_sync ( tpd->dev );
#endif
					report_enable = 0;
				}
			}

			finger_info.total_num = index;
			old_touch_info.total_num = pre_touch_info.total_num;
			pre_touch_info.total_num = finger_info.total_num;

			input_sync ( tpd->dev );
		}
	

exit_work:
		mutex_unlock ( &i2c_access );
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	} while ( !kthread_should_stop () );

	return 0;
}

/****************************************************************************
* Synaptics Notify Funtions
****************************************************************************/
static enum hrtimer_restart synaptics_notify_timer_handler ( struct hrtimer *notify_timer )
{
	TPD_FUN ();
	
	queue_delayed_work ( knock_set_work_wq, &knock_set_work, 0 );

	return HRTIMER_NORESTART;
}

/****************************************************************************
* SYSFS function for Touch
****************************************************************************/
static ssize_t show_knock_on_type ( struct device *dev, struct device_attribute *attr, char *buf )
{
	TPD_FUN ();
	int ret = 0;
	ret = sprintf ( buf, "%d\n", knock_on_type );
	return ret;
}
static DEVICE_ATTR ( knock_on_type, 0664, show_knock_on_type, NULL );


static ssize_t show_lpwg_data ( struct device *dev, struct device_attribute *attr, char *buf )
{
	TPD_FUN ();
	int i = 0, ret = 0;

	memset(lpwg_data, 0, sizeof(struct point)*MAX_POINT_SIZE_FOR_LPWG);

	synaptics_knock_lpwg ( tpd_i2c_client, LPWG_READ, 0, lpwg_data );
	for ( i = 0 ; i < MAX_POINT_SIZE_FOR_LPWG ; i++ )
	{
		if ( lpwg_data[i].x == -1 && lpwg_data[i].y == -1 )
		{
			break;
		}
		ret += sprintf ( buf+ret, "%d %d\n", lpwg_data[i].x, lpwg_data[i].y );
	}

	return ret;
}

static ssize_t store_lpwg_data ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	TPD_FUN ();
	int reply = 0;

	sscanf ( buf, "%d", &reply );
	TPD_LOG ( "LPWG RESULT = %d ", reply );

	synaptics_knock_lpwg ( tpd_i2c_client, LPWG_REPLY, reply, NULL );

	wake_unlock ( &knock_code_lock );

	return size;
}
static DEVICE_ATTR ( lpwg_data, 0664, show_lpwg_data, store_lpwg_data );

static ssize_t store_lpwg_notify ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	int type = 0;
	int value[4] = {0};
	int ret = 0;
	u8 buffer[2] = {0,};
	
	sscanf ( buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3] );


	if(type == 6 || type == 1 || type ==7)
	{
//		TPD_LOG("TYPE %d is not use", type);
		return size;
	}
	TPD_LOG ( "[store_lpwg_notify]touch notify type = %d , value[0] = %d, value[1] = %d, value[2] = %d, value[3] = %d ", type, value[0], value[1], value[2], value[3] );
	mutex_lock(&knock_access);

	switch ( type )
	{
		case 1:			
			break;
			
        case 2:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_LCD_X, value[0], NULL );
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_LCD_Y, value[1], NULL );
			TPD_LOG("LPWG_LCD_X = %d",value[0]);
			TPD_LOG("LPWG_LCD_Y = %d",value[1]);
			break;

        case 3:
			Double_Tap_Area_X1 = value[0]*2;//set lcd / touch 
			Double_Tap_Area_X2 = value[1]*2;
			Double_Tap_Area_Y1 = value[2]*2;
			Double_Tap_Area_Y2 = value[3]*2;

			if(suspend_status == 0 && qcover == 1)
			{
				synaptics_release_all_finger ();
			}
			break;

        case 4:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_TAP_COUNT, value[0], NULL );
			break;
		case 6:
			break;

			
      	case 7:	

			break;
		case 8:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_DOUBLE_TAP_CHECK, value[0], NULL );
			break;
		case 9:
			if(cancel_delayed_work_sync(&knock_set_work))
			{  
				TPD_LOG("Pending queueu work");
			}
			else
			{
				TPD_LOG("Cancle knock set work");
			}
			ret = hrtimer_try_to_cancel ( &notify_timer );
			TPD_LOG("notify timer cancle  = %d ",ret	);
			if ( ret  < 0 )								  //  0 when the timer was not active  
			{
				TPD_LOG("Pending htimer callback");
				hrtimer_cancel(&notify_timer);
			}
			
			int *v = &value[0];
			lpwg_mode = *(v + 0);
			screen    = *(v + 1);
			sensor    = *(v + 2);
			qcover    = *(v + 3);

			
			if(lpwg_mode == 1)//Knock on
			{
				knock_on_enable   = 1; 
				double_tap_enable = 1;
				multi_tap_enable  = 0;
				
			}
			else if(lpwg_mode == 2)//Knock code
			{
				knock_on_enable   = 1;
				double_tap_enable = 1;
				multi_tap_enable  = 1; 
			}
			else//idle 
			{
				knock_on_enable = 0;
				double_tap_enable = 0;
				multi_tap_enable = 0;
			}
			//TPD_LOG("lpwg_mode = %d, screen = %d, sensor = %d, qcover = %d, knock_on_enable = %d, double_tap_enable = %d, multi_tap_enable = %d"
			//	  , lpwg_mode ,screen ,sensor ,qcover, knock_on_enable, double_tap_enable, multi_tap_enable);
				
			/*Touch IC Sleep control */
			/*sensor 0 => sensor NEAR touch ic disble*/
			/*sensor 1 => sensor FAR normal operation(enable knock on/code)*/
			if ( suspend_status == 1 && screen == 1 )
				break;
			
			if(suspend_status == 1 && qcover == 0 /*&& sensor ==0*/)
			{
				DO_SAFE(sleep_control(tpd_i2c_client, sensor, 0), error);			
			}
			else if(suspend_status == 1 && qcover == 1 /*&& sensor == 0*/)//quick cover is close ==> normal operation 
			{
				DO_SAFE(sleep_control(tpd_i2c_client, 1, 0), error);			
				hrtimer_start ( &notify_timer, ktime_set(0, MS_TO_NS(50)), HRTIMER_MODE_REL );
			}
			if(ret == 1)
			{
				hrtimer_start ( &notify_timer, ktime_set(0, MS_TO_NS(50)), HRTIMER_MODE_REL );
				TPD_LOG("Cancle notify timer");
			}
			TPD_LOG("END NOTIFY");		
			break;
		case 10:
			telepony = value[0];
			if(telepony == 1 &&suspend_status == 0)//call status is ringing && idle status .==> sensitivity down
			{
			}
			else if(telepony != 1 && suspend_status == 0)//call status is idle && idle status. ==> sensitivity is normal(pen)
			{
				buffer[0] = 0x19;
				buffer[1] = 0x1e;
				DO_SAFE(synaptics_page_data_write ( tpd_i2c_client, 0x00, 0x17, 2, buffer ), error);
				buffer[0] = 0x26;
				buffer[1] = 0x14;
				DO_SAFE(synaptics_page_data_write ( tpd_i2c_client, 0x00, 0x1A, 2, buffer ), error);
			}
			break;
		default:
			break;
		}
	mutex_unlock(&knock_access);
	return size;

	error :
		mutex_unlock(&knock_access);
		return -1;
}
static DEVICE_ATTR ( lpwg_notify, 0664, NULL, store_lpwg_notify );

void write_time_log(char *filename, char *data, int data_include)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu \n\n",
		my_date.tm_mon + 1,my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);
	set_fs(KERNEL_DS);
	if (filename == NULL)
		fname = "/mnt/sdcard/touch_self_test.txt";
	else
		fname = filename;
	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
    TPD_LOG("write open %s, fd : %d \n", (fd >= 0)? "success": "fail", fd);
	if (fd >= 0) {
		if (data_include && data != NULL)
			sys_write(fd, data, strlen(data));
		sys_write(fd, time_string, strlen(time_string));
		sys_close(fd);
	}
	set_fs(old_fs);
}
/*
static ssize_t show_sd ( struct device *dev, struct device_attribute *attr, char *buf )
{
	TPD_FUN();
	TPD_LOG("B2 BASE SHOW SD");
	int ret = 0;
	int full_raw_cap = 0;
	int trx_to_trx = 0;
	int trx_to_gnd = 0;
	int high_resistance = 0;
	int noise_delta = 0;
	int adc_range = 0;
	int sensor_speed = 0;
	char *temp_buf = NULL;
	int len = 0;

	if ( !suspend_status )
	{
	
		write_time_log(NULL, NULL, 0);//modify
		msleep(10);
	
		//write_firmware_version_log(ts);//modify

		mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );

		SCAN_PDT();
		TPD_LOG("After SCAN_PDT()");
		
		full_raw_cap = F54Test('a', 0, buf);
		TPD_LOG("After F54Test(a)=>%d",full_raw_cap);
			msleep(30);
			
		noise_delta = F54Test('x', 0, buf);
		TPD_LOG("After F54Test(x)=>%d",noise_delta);
				

		trx_to_trx = F54Test('f', 0, buf);
		TPD_LOG("After F54Test(f)=>%d",trx_to_trx);
				

		trx_to_gnd = F54Test('e', 0, buf);
		TPD_LOG("After F54Test(e)=>%d",trx_to_gnd);
				

		high_resistance = F54Test('g', 0, buf);
		TPD_LOG("After F54Test(g)=> %d",high_resistance);
				
		
		TPD_LOG("noise_delta = %d ,trx_to_trx = %d ,trx_to_gnd = %d ,high_resistance = %d",noise_delta ,trx_to_trx ,trx_to_gnd, high_resistance );
		synaptics_initialize(tpd_i2c_client);	
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
		msleep(30);

		TPD_LOG("AFTER S3528 INITIALIZE");
		ret = sprintf(buf+ret, "========RESULT=======\n");
		ret += sprintf(buf+ret, "Channel Status : %s", (trx_to_trx && trx_to_gnd && high_resistance) ? "Pass\n" : "Fail\n");
		ret += sprintf(buf+ret, "Raw Data : %s", (full_raw_cap && noise_delta) ? "Pass\n" : "Fail\n");

		TPD_LOG("ret = %d",ret);

	}
	else
	{
		write_time_log(NULL, NULL, 0);
		ret += sprintf ( buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n" );
	}

	return ret;
}
static DEVICE_ATTR ( sd, 0664, show_sd, NULL );
*/
static struct attribute *lge_touch_attrs[] = {
	&dev_attr_knock_on_type.attr, 
	&dev_attr_lpwg_data.attr,
	&dev_attr_lpwg_notify.attr,
	//&dev_attr_sd.attr,
	NULL,
};

static struct attribute_group lge_touch_group = {
	.name = LGE_TOUCH_NAME,
	.attrs = lge_touch_attrs,
};


#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
static ssize_t synaptics_store_firmware ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	int ret;
	char path[256] = {0};

	sscanf ( buf, "%s", path );
	TPD_LOG ( "Firmware image update: %s\n", path );

	if ( !strncmp ( path, "1", 1 ) )
	{
		fw_force_update = 1;
	}
	else
	{
		memcpy ( fw_path, path, sizeof(fw_path) );
	}

	ret = synaptics_firmware_update ( tpd_i2c_client );

	synaptics_initialize ( tpd_i2c_client );

	return size;
}
static DEVICE_ATTR ( firmware, 0664, NULL, synaptics_store_firmware );
#endif

static ssize_t synaptics_show_version ( struct device *dev, struct device_attribute *attr, char *buf )
{
	char fw_config_id[5] = {0};
	char fw_product_id[11] = {0};
	int ret;

	ret = synaptics_ts_read ( tpd_i2c_client, PRODUCT_ID_REG, 10, &fw_product_id[0] );
	if ( ret < 0 )
	{
		TPD_ERR ( "PRODUCT_ID_REG read fail\n" );
	}
	ret = synaptics_ts_read ( tpd_i2c_client, FLASH_CONFIG_ID_REG, 4, &fw_config_id[0] );
	if ( ret < 0 )
	{
		TPD_ERR ( "FLASH_CONFIG_ID_REG read fail\n" );
	}
	return sprintf ( buf, "%02x%02x%02x%02x\n", fw_config_id[0],fw_config_id[1],fw_config_id[2],fw_config_id[3]);
}
static DEVICE_ATTR ( version, 0664, synaptics_show_version, NULL );

static ssize_t synaptics_store_write ( struct device *dev, struct device_attribute *attr, char *buf, size_t size )
{
	u8 temp = 0;	
	u8 reg = 0;
	u8 value = 0;
	unsigned int page = 0;
	int ret;

	sscanf ( buf, "%d %x %x", &page, (unsigned int *)&reg, (unsigned int *)&value );
	TPD_LOG ( "(write) page=%d, reg=0x%x, value=0x%x\n", page, reg, value );

	ret = synaptics_page_data_write ( tpd_i2c_client, page, reg, 1, &value );
	if ( ret < 0 )
	{
		TPD_ERR ( "REGISTER write fail\n" );
	}

	return size;
}
static DEVICE_ATTR ( write, 0664, NULL, synaptics_store_write );

static ssize_t synaptics_store_read ( struct device *dev, struct device_attribute *attr, char *buf, size_t size )
{
	u8 temp = 0;
	u8 reg = 0;
	unsigned int page = 0;
	int ret;

	sscanf ( buf, "%d %x", &page, (unsigned int*)&reg );


	
	ret = synaptics_page_data_read ( tpd_i2c_client, page, reg, 1, &temp );
	if ( ret < 0 )
	{
		TPD_ERR ( "REGISTER read fail\n" );
	}

	TPD_LOG ( "(read) page=%d, reg=0x%x, value=0x%x\n", page, reg, temp );

	return size;
}
static DEVICE_ATTR ( read, 0664, NULL, synaptics_store_read );

/****************************************************************************
* Synaptics_Touch_IC Initialize Function
****************************************************************************/
static void read_page_description_table ( struct i2c_client *client )
{
	TPD_FUN ();
	int ret = 0;
	function_descriptor buffer;
	u8 page_num;
	u16 u_address;

	memset(&buffer, 0x0, sizeof(function_descriptor));

	for ( page_num = 0 ; page_num < PAGE_MAX_NUM ; page_num++ )
	{
		ret = synaptics_ts_write ( client, PAGE_SELECT_REG, &page_num, 1 );
		if ( ret < 0 )
		{
			TPD_ERR ( "PAGE_SELECT_REG write fail (page_num=%d)\n", page_num );
		}

		for ( u_address = DESCRIPTION_TABLE_START ; u_address > 10 ; u_address -= sizeof(function_descriptor) )
		{
			ret = synaptics_ts_read ( client, u_address, sizeof(buffer), (u8 *) &buffer );
			if ( ret < 0 )
			{
				TPD_ERR ( "function_descriptor read fail\n" );
				return;
			}

			TPD_LOG ("buffer.function_exist=%x, page_num=%d\n",buffer.function_exist,page_num);
			if ( buffer.function_exist == 0 )
				break;

			switch ( buffer.function_exist )
			{
				case RMI_DEVICE_CONTROL:
					device_control = buffer;
					device_control_page = page_num;
					break;
				case TOUCHPAD_SENSORS:
					finger = buffer;
					finger_page = page_num;
					break;
				case CAPACITIVE_BUTTON_SENSORS:
					button = buffer;
					button_page = page_num;
					break;
				case FLASH_MEMORY_MANAGEMENT:
					flash_memory = buffer;
					flash_memory_page = page_num;
					break;
				case MULTI_TAP_GESTURE:
					multi_tap = buffer;
					multi_tap_page = page_num;
					break;
			}
		}
	}

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, &device_control_page, 1 );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
	}
	get_f12_info(client);
}

#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
static int synaptics_firmware_check ( struct i2c_client *client )
{
	TPD_FUN ();
	int ret;
	u8 device_status = 0;
	u8 flash_control = 0;
	u8 flash_status = 0;
	u8 fw_ver, image_ver;
	char fw_config_id[5] = {0};
	char fw_product_id[11] = {0};
	char image_config_id[5] = {0};
	char image_product_id[11] = {0};

	/* read Firmware information in Download Image */
#ifdef TEMP_PCB_VER_CHECK
    if ( gPCBver==HW_REV_B )
    {   
        fw_start = (unsigned char *) &SynaFirmware_RevB[0];
        fw_size = sizeof(SynaFirmware_RevB);
        strncpy ( image_product_id, &SynaFirmware_RevB[0x0040], 6 );
        strncpy ( image_config_id, &SynaFirmware_RevB[0x16d00], 4 );
        TPD_LOG ( "Firmware : gPCBver(%d) : Rev.B \n",gPCBver );
    }
    else    // ( gPCBver<=HW_REV_A_2 ) || ( gPCBver==HW_REV_A )
    {
    	fw_start = (unsigned char *) &SynaFirmware[0];
    	fw_size = sizeof(SynaFirmware);
    	strncpy ( image_product_id, &SynaFirmware[0x0040], 6 );
    	strncpy ( image_config_id, &SynaFirmware[0x16d00], 4 );
        TPD_LOG ( "Firmware : gPCBver(%d) : Rev.A or Rev.A-2 \n",gPCBver );        
    }
#else    
	fw_start = (unsigned char *) &SynaFirmware[0];
	fw_size = sizeof(SynaFirmware);
	strncpy ( image_product_id, &SynaFirmware[0x0040], 6 );
	strncpy ( image_config_id, &SynaFirmware[0x16d00], 4 );
#endif /* TEMP_PCB_VER_CHECK */

	if ( fw_path[0] != 0 )
	{
		TPD_LOG ( "Firmware force update by fw file\n" );
		return -1;
	}
	else if ( fw_force_update == 1 )
	{
		TPD_LOG ( "Firmware force update by buffer 1 [%02x%02x%02x%02x Ver]\n", image_config_id[0], image_config_id[1], image_config_id[2], image_config_id[3] );
		return -1;
	}

	ret = synaptics_ts_read ( client, DEVICE_STATUS_REG, 1, &device_status );
	if ( ret < 0 )
	{
		TPD_ERR ( "DEVICE_STATUS_REG read fail\n" );
	}
	ret = synaptics_ts_read ( client, FLASH_CONTROL_REG, 1, &flash_control );
	if ( ret < 0 )
	{
		TPD_ERR ( "FLASH_CONTROL_REG read fail\n" );
	}

	ret = synaptics_ts_read ( client, FLASH_STATUS_REG, 1, &flash_status );
	if ( ret < 0 )
	{
		TPD_ERR ( "FLASH_STATUS_REG read fail\n" );
	}

	if ( ( device_status & FLASH_PROG_MASK ) || ( device_status & FW_CRC_FAILURE_MASK ) != 0 || ( flash_status & FLASH_STATUS_MASK ) != 0)
	{
		TPD_ERR ( "Firmware has a problem. [device_status=%x, flash_control=%x]\nso it needs Firmware update. [%02x%02x%02x%02x Ver]\n", device_status, flash_control, image_config_id[0], image_config_id[1], image_config_id[2], image_config_id[3] );
		return -1;
	}
	else
	{
		/* read Firmware information in Touch IC */
		ret = synaptics_ts_read ( client, PRODUCT_ID_REG, 10, &fw_product_id[0] );
		if ( ret < 0 )
		{
			TPD_ERR ( "PRODUCT_ID_REG read fail\n" );
		}
		else
		{
			TPD_LOG ( "PRODUCT_ID_REG : %s \n", fw_product_id );
		}

		ret = synaptics_ts_read ( client, FLASH_CONFIG_ID_REG, 4, &fw_config_id[0] );
		if ( ret < 0 )
		{
			TPD_ERR ( "FLASH_CONFIG_ID_REG read fail\n" );
		}
		else
		{
			TPD_LOG ( "FLASH_CONFIG_ID_REG : %02x%02x%02x%02x \n", fw_config_id[0], fw_config_id[1], fw_config_id[2], fw_config_id[3] );
		}
#ifdef WXSERIES_FW
		fw_ver = fw_config_id[3] & 0x7F;
		image_ver = image_config_id[3] & 0x7F;
#else
		fw_ver = ( int ) simple_strtol ( &fw_config_id[3], NULL, 10 );
		image_ver = ( int ) simple_strtol ( &image_config_id[3], NULL, 10 );
#endif

		TPD_LOG("fw_ver : 0x%02x, image_ver : 0x%02x\n", fw_ver , image_ver);

		if ( ( image_ver != fw_ver ) ) //temp
		{
			TPD_LOG ( "[%02x ver ==> %02x ver] Firmware Update\n", fw_ver, image_ver );
			return -1;
		}
		else
		{
			TPD_LOG ( "No need to update Firmware\n" );
			return 0;
		}
	}
}

static int synaptics_firmware_update ( struct work_struct *work_fw_upgrade )
{
	TPD_FUN ();
	int ret;
	
	struct i2c_client *client;

	if ( tpd_i2c_client != NULL ) 
		client = tpd_i2c_client;
	else
		return 0;

	ret	= synaptics_firmware_check ( client );
	
	if ( ret != 0 )
	{
		if ( !download_status )
		{
			download_status = 1;
			wake_lock ( &fw_suspend_lock );

			mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );

			if ( !knock_on_enable && suspend_status )
			{
				if ( ! Power_status )
				synaptics_power ( 1 );
			}

			mtk_wdt_enable ( WK_WDT_DIS );
			TPD_LOG ( "Watchdog disable\n" );

			ret = FirmwareUpgrade ( client, fw_path, fw_size, fw_start );
			if ( ret < 0 )
			{
				TPD_ERR ( "Firmware update Fail!!!\n" );
			}
			else
			{
				TPD_ERR ( "Firmware upgrade Complete\n" );
			}

			mtk_wdt_enable ( WK_WDT_EN );
			TPD_LOG ( "Watchdog enable\n" );
			mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
			memset(fw_path, 0x00, sizeof(fw_path));
			fw_force_update = 0;

			synaptics_ic_reset ();

			wake_unlock ( &fw_suspend_lock );
			
			download_status = 0;

			arch_reset(0,NULL);
		}
		else
		{
			TPD_ERR ( "Firmware Upgrade process is aready working on\n" );
		}

		read_page_description_table ( client );
	}

	synaptics_initialize ( client );
}
#endif

void synaptics_initialize( struct i2c_client *client )
{
	TPD_FUN ();
	int ret = 0;
	u8 temp = 0;

	temp = DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED;
	ret = synaptics_ts_write ( client, DEVICE_CONTROL_REG, &temp, 1 );
	if (ret < 0)
	{
		TPD_ERR ( "DEVICE_CONTROL_REG write fail\n" );
	}

	ret = synaptics_ts_read ( client, INTERRUPT_ENABLE_REG, 1, &temp );
	if ( ret < 0 )
	{
		TPD_ERR ( "INTERRUPT_ENABLE_REG read fail\n" );
	}
	temp = temp | INTERRUPT_MASK_ABS0;
	ret = synaptics_ts_write ( client, INTERRUPT_ENABLE_REG, &temp, 1 );
	if (ret < 0)
	{
		TPD_ERR ( "INTERRUPT_ENABLE_REG write fail\n" );
	}

	ret = synaptics_ts_read ( client, INTERRUPT_STATUS_REG, 1, &temp ); // It always should be done last.==> Interrupt Pin Clear.
	if ( ret < 0 )
	{
		TPD_ERR ( "INTERRUPT_STATUS_REG read fail\n" );
	}
	TPD_LOG ( "INTERRUPT_STATUS_REG value = %d\n", temp );

	return 0;
}

static int synaptics_workqueue_init ( void )
{
	TPD_FUN ();

#ifdef LGE_USE_SYNAPTICS_FW_UPGRADE
	touch_wq = create_singlethread_workqueue ( "touch_wq" );
	if ( touch_wq )
	{
		INIT_WORK ( &work_fw_upgrade, synaptics_firmware_update );

		wake_lock_init ( &fw_suspend_lock, WAKE_LOCK_SUSPEND, "fw_wakelock" );
	}
	else
	{
		goto err_workqueue_init;
	}
#endif

	touch_multi_tap_wq = create_singlethread_workqueue ( "touch_multi_tap_wq" );
	if ( touch_multi_tap_wq )
	{
		INIT_WORK ( &multi_tap_work, synaptics_multi_tap_work );
		hrtimer_init ( &multi_tap_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		multi_tap_timer.function = synaptics_multi_tap_timer_handler;

		wake_lock_init ( &knock_code_lock, WAKE_LOCK_SUSPEND, "knock_code" );
	}
	else
	{
		goto err_workqueue_init;
	}

	knock_set_work_wq = create_singlethread_workqueue ( "knock_set_work_wq" );
	if ( knock_set_work_wq )
	{
		INIT_DELAYED_WORK ( &knock_set_work, synaptics_lpwg_update_all );
		hrtimer_init ( &notify_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		notify_timer.function = synaptics_notify_timer_handler;
	}
	else
	{
		goto err_workqueue_init;
	}

	return 0;

err_workqueue_init:
	TPD_ERR ( "create_singlethread_workqueue failed\n" );
	return -1;
}

void synaptics_init_sysfs ( void )
{
	TPD_FUN ();
	int err;

	touch_class = class_create ( THIS_MODULE, "touch" );

#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
	touch_fw_dev = device_create ( touch_class, NULL, 0, NULL, "firmware" );
	err = device_create_file ( touch_fw_dev, &dev_attr_firmware );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [firmware] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_firmware );
	}

	err = device_create_file ( touch_fw_dev, &dev_attr_version );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [version] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_version );
	}
#endif

	err = device_create_file ( touch_fw_dev, &dev_attr_write );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [write] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_write );
	}

	err = device_create_file ( touch_fw_dev, &dev_attr_read );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [read] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_read );
	}

	sysfs_create_group ( tpd->dev->dev.kobj.parent, &lge_touch_group );
}
EXPORT_SYMBOL ( synaptics_init_sysfs );

#define to_foo_obj(x) container_of(x, struct foo_obj, kobj)
struct foo_attribute {
	struct attribute attr;
	ssize_t (*show)(struct foo_obj *foo, struct foo_attribute *attr, char *buf);
	ssize_t (*store)(struct foo_obj *foo, struct foo_attribute *attr, const char *buf, size_t count);
};
#define to_foo_attr(x) container_of(x, struct foo_attribute, attr)
static ssize_t foo_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf){
		struct foo_attribute *attribute;
	struct foo_obj *foo;

	attribute = to_foo_attr(attr);
	foo = to_foo_obj(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(foo, attribute, buf);
}
static ssize_t foo_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct foo_attribute *attribute;
	struct foo_obj *foo;

	attribute = to_foo_attr(attr);
	foo = to_foo_obj(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(foo, attribute, buf, len);
}
static const struct sysfs_ops foo_sysfs_ops = {
	.show = foo_attr_show,
	.store = foo_attr_store,
};
static void foo_release(struct kobject *kobj)
{
	struct foo_obj *foo;

	foo = to_foo_obj(kobj);
	kfree(foo);
}

static ssize_t foo_show(struct foo_obj *foo_obj, struct foo_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", foo_obj->interrupt);
}
static ssize_t foo_store(struct foo_obj *foo_obj, struct foo_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%du", &foo_obj->interrupt);
	return count;
}
static struct foo_attribute foo_attribute =__ATTR(interrupt, 0664, foo_show, foo_store);
static struct attribute *foo_default_attrs[] = {
	&foo_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};
static struct kobj_type foo_ktype = {
	.sysfs_ops = &foo_sysfs_ops,
	.release = foo_release,
	.default_attrs = foo_default_attrs,
};
static struct kset *example_kset;

static struct foo_obj *create_foo_obj(const char *name){
	struct foo_obj *foo;
	int retval;
	foo = kzalloc(sizeof(*foo), GFP_KERNEL);
	if (!foo)
		return NULL;
	foo->kobj.kset = example_kset;
	retval = kobject_init_and_add(&foo->kobj, &foo_ktype, NULL, "%s", name);
	if (retval) {
		kobject_put(&foo->kobj);
		return NULL;
	}
	kobject_uevent(&foo->kobj, KOBJ_ADD);
	return foo;
}

#ifdef TEMP_PCB_VER_CHECK
static int get_pcb_version(void)
{
    int data[4] = {0, 0, 0, 0};
    int rawvalue    = 0;
    int ret         = 0;
    int PCBvoltage  = 0;

    ret = IMM_GetOneChannelValue(1, data, &rawvalue);
    if (ret == 0)
    {
        TPD_LOG("[%s] success to get adc value channel(1)=(0x%x) (%d) (ret=%d)\n", __func__, rawvalue, rawvalue, ret);
        TPD_LOG("[%s] data[0]=0x%x, data[1]=0x%x\n\n",__func__, data[0], data[1]);
    } else {
        TPD_LOG("[%s] fail to get adc value (ret=%d)\n", __func__, ret);
    }

    PCBvoltage = (int)(rawvalue * 150 / 4096);

    if ( PCBvoltage>HW_REV_B_RANGE )
        gPCBver = HW_REV_B;    // rev.b
    else if ( (PCBvoltage>HW_REV_A_RANGE) && (PCBvoltage<HW_REV_B_RANGE) )
        gPCBver = HW_REV_A_2;    // rev.a-2
    else
        gPCBver = HW_REV_A;    // rev.a
    
    TPD_LOG("[LGE_TEST] PCBvoltage(%d), gPCBver(%d)\n", PCBvoltage, gPCBver);
    
    return gPCBver;
}
#endif /* TEMP_PCB_VER_CHECK */

/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/
static int synaptics_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	TPD_FUN ();
	int ret = 0;
	struct foo_obj *foo;
	struct task_struct *thread = NULL;

	/* i2c_check_functionality */
	if ( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) )
	{
		TPD_ERR ( "i2c functionality check error\n" );
		goto err_probing;
	}

#ifdef TEMP_PCB_VER_CHECK
    get_pcb_version();
#endif /* TEMP_PCB_VER_CHECK */
    
	/* X, Y max touch position */
	x_max = DISP_GetScreenWidth ();
	y_max = DISP_GetScreenHeight ();

	tpd_i2c_client = client;
	ds4_i2c_client = client;

	/* Turn on the power for Touch */
	//synaptics_power ( 0 );
	//synaptics_power ( 1 );

	mt_set_gpio_mode ( GPIO_CTP_RST_PIN, GPIO_MODE_00 );
	mt_set_gpio_dir ( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
	mt_set_gpio_out ( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
	synaptics_ic_reset ();

	/* Initialize work queue  */
	ret = synaptics_workqueue_init ();
	if ( ret != 0 )
	{
		TPD_ERR ( "synaptics_workqueue_init failed\n" );
		goto err_probing;
	}

	thread = kthread_run ( synaptics_touch_event_handler, 0, TPD_DEVICE );
	if ( IS_ERR ( thread ) )
	{
		ret = PTR_ERR ( thread );
		TPD_ERR ( "failed to create kernel thread: %d\n", ret );
		goto err_probing;
	}

	/* Configure external ( GPIO ) interrupt */
	synaptics_setup_eint ();

	/* Find register map */
	read_page_description_table ( client );

	/* Initialize for Knock function  */
	knock_on_type = 1;
	example_kset = kset_create_and_add ( "lge", NULL, kernel_kobj );
	foo_obj = create_foo_obj ( LGE_TOUCH_NAME );

	/* Touch Firmware Update */
#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
	queue_work ( touch_wq, &work_fw_upgrade );
#endif

	tpd_load_status = 1;
	return 0;

err_probing:
	return ret;
}

static int synaptics_i2c_remove(struct i2c_client *client)
{
	TPD_FUN ();
	return 0;
}

static int synaptics_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	TPD_FUN ();
	strcpy ( info->type, "mtk-tpd" );
	return 0;
}

static const struct i2c_device_id tpd_i2c_id[] = { { TPD_DEV_NAME, 0 },	{} };

static struct i2c_driver tpd_i2c_driver = {
	.driver.name = "mtk-tpd",
	.probe = synaptics_i2c_probe,
	.remove = synaptics_i2c_remove,
	.detect = synaptics_i2c_detect,
	.id_table = tpd_i2c_id,
};

/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static int synaptics_local_init ( void )
{
	TPD_FUN ();

	if ( i2c_add_driver ( &tpd_i2c_driver ) != 0 )
	{
		TPD_ERR ( "i2c_add_driver failed\n" );
		return -1;
	}

	if ( tpd_load_status == 0 )
	{
		TPD_ERR ( "touch driver probing failed\n" );
		i2c_del_driver ( &tpd_i2c_driver );
		return -1;
	}

	tpd_type_cap = 1;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_suspend ( struct early_suspend *h )
{
	TPD_FUN ();
	int ret = 0;

	suspend_status = 1;

	if ( cancel_delayed_work_sync ( &knock_set_work ) )
	{
		TPD_LOG ( "pending queue work\n" );
	}
	if ( hrtimer_try_to_cancel ( &notify_timer ) < 0 )
	{
		TPD_LOG ( "pending callback\n" );
		hrtimer_cancel ( &notify_timer );
	}

	synaptics_release_all_finger ();

	ret = hrtimer_start ( &notify_timer, ktime_set(0, MS_TO_NS(50)), HRTIMER_MODE_REL );
	TPD_LOG("hrtimer_start return value  = %d",ret);
}

static void synaptics_resume ( struct early_suspend *h )
{
	TPD_FUN ();

	if ( cancel_delayed_work_sync ( &knock_set_work ) )
	{
		TPD_LOG ( "pending queue work\n" );
	}
	if ( hrtimer_try_to_cancel ( &notify_timer ) < 0 )
	{
		TPD_LOG ( "pending callback\n" );
		hrtimer_cancel ( &notify_timer );
	}

	synaptics_ic_reset ();
	synaptics_initialize ( tpd_i2c_client );

	if ( key_lock_status == 0 )
	{
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	}
	else
	{
		mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
	}

	suspend_status = 0;
}
#endif

static struct i2c_board_info __initdata i2c_tpd = {	I2C_BOARD_INFO ( TPD_DEV_NAME, TPD_I2C_ADDRESS ) };

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TPD_DEV_NAME,
	.tpd_local_init = synaptics_local_init,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = synaptics_suspend,
	.resume = synaptics_resume,
#endif
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init synaptics_driver_init ( void )
{
	TPD_FUN ();

	I2CDMABuf_va = (u8 *) dma_alloc_coherent ( NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL );
	if ( !I2CDMABuf_va )
	{
		TPD_ERR ( "Allocate Touch DMA I2C Buffer failed!\n" );
		return -ENOMEM;
	}

	i2c_register_board_info ( 0, &i2c_tpd, 1 );
	if ( tpd_driver_add ( &tpd_device_driver ) < 0 )
	{
		TPD_ERR ( "tpd_driver_add failed\n" );
	}

	return 0;
}

static void __exit synaptics_driver_exit ( void )
{
	TPD_FUN ();

	tpd_driver_remove ( &tpd_device_driver );

	if ( I2CDMABuf_va )
	{
		dma_free_coherent ( NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa );
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
	}
	if ( touch_multi_tap_wq )
	{
		destroy_workqueue ( touch_multi_tap_wq );
	}
}


module_init ( synaptics_driver_init );
module_exit ( synaptics_driver_exit );

MODULE_DESCRIPTION ( "Synaptics 3320 Touchscreen Driver for MTK platform" );
MODULE_AUTHOR ( "Junmo Kang <junmo.kang@lge.com>" );
MODULE_LICENSE ( "GPL" );

/* End Of File */
