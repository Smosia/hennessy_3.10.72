#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include <kd_camera_hw.h>
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

//#include <mach/mt_pm_ldo.h>
//#include "pmic_drv.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

#undef PK_ERR
#define PK_ERR  PK_DBG


#define LM3643_NAME "leds-LM3643-1"

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static bool  g_strobe_On;

//static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);


#define STROBE_DEVICE_ID 0x63


static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */

#define LM3643_REG_ENABLE       	0x01
#define LM3643_REG_LED1_FLASH  0x03
#define LM3643_REG_LED2_FLASH  0x04
#define LM3643_REG_LED1_TORCH  0x05
#define LM3643_REG_LED2_TORCH  0x06
#define LM3643_REG_TIMING           0x08

//#define FLASH_ENABLE  (GPIO69 | 0x80000000)   //#define GPIO_CAM_SWSEL	                   (GPIO69 | 0x80000000)   //CAM_SWSEL
//#define FLASH_TORCH_ENABLE  (GPIO43| 0x80000000)
//#define FLASH_STROBE_ENABLE  (GPIO42 | 0x80000000)

#define FLASH_TORCH_ENABLE (GPIO_CAMERA_FLASH_EN_PIN)
#define FLASH_STROBE_ENABLE (GPIO_CAMERA_FLASH_MODE_PIN)

#define FLASH_GPIO_ENT GPIO13

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);
static struct i2c_client *LM3643_i2c_client = NULL;
void flash_set_i2c_gpio(void);

struct LM3643_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3643_chip_data {
	struct i2c_client *client;

	struct LM3643_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};


static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	#if 1
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);
	#else
	u8 buffer[2];
	buffer[0] = reg;
	buffer[1] = val;
	ret = i2c_master_send(client, (char *)buffer, 2);
	#endif
	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	#if 1
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);
	if (val < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, val);
		return val;
	}
	#else
	ret = i2c_master_send(client, (char *)&reg, 1);
	if (ret < 0)
	{
		PK_ERR("send dummy is %d\n", ret);
		return -1;
	}
	
	ret = i2c_master_recv(client, val, 1);
	if (ret < 0)
	{
		PK_ERR("recv dummy is %d\n", ret);
		return -1;
	}
	#endif

	return val;
}

int readReg(int reg)
{
    char buf[2];
	int val=0;
    buf[0]=reg;
   // iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
	val=LM3643_read_reg(LM3643_i2c_client,buf[0]);
    PK_DBG("qq reg=%x val=%x qq\n", buf[0],val);
    return val;
}

int writeReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
	
   // iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);
	LM3643_write_reg(LM3643_i2c_client,buf[0],buf[1]);

   return 0;
}

enum
{
	e_DutyNum = 23,
};
static int isMovieMode[e_DutyNum]={1,1,1,1,0,0,0,0,0,0,0,0,0,0,0};
static int torchDuty[e_DutyNum]=    {35,71,106,127,0,0,0,0,0,0,0,0,0,0,0};
//52,105,156,179ma
static int flashDuty[e_DutyNum]=     {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101};
//static int flashDutylt[e_DutyNum]=     {8,12,15,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110};//lenovo-sw sunliang modfiy 2015_4_14
//200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500ma
int m_duty1=0;
int m_duty2=0;
//static int torch_flag=0;
int LED1Closeflag = 0;
int LED2Closeflag = 0;
int flashEnable_LM3643_1(void)
{
	
	
		
		//mt_set_gpio_mode(FLASH_TORCH_ENABLE, 0);
		//mt_set_gpio_dir(FLASH_TORCH_ENABLE, GPIO_DIR_OUT);
		//mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ONE);		
		//mt_set_gpio_mode(FLASH_STROBE_ENABLE, 0);
		//mt_set_gpio_dir(FLASH_STROBE_ENABLE, GPIO_DIR_OUT);
		//mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ONE);
	return 0;
}
int flashDisable_LM3643_1(void)
{
	
      return 0;
}


int setDuty_LM3643_1(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	PK_DBG(" setDuty_LM3643_1 line=%d\n",__LINE__);
	return 0;
}



int flashEnable_LM3643_2(void)
{
	int temp;

	PK_DBG("flashEnable_LM3643_2\n");
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);
	//set gpio 
	mt_set_gpio_mode(FLASH_TORCH_ENABLE, 0);
	mt_set_gpio_dir(FLASH_TORCH_ENABLE, GPIO_DIR_OUT);
	mt_set_gpio_mode(FLASH_STROBE_ENABLE, 0);
	mt_set_gpio_dir(FLASH_STROBE_ENABLE, GPIO_DIR_OUT);		

	temp=readReg(LM3643_REG_ENABLE);
	
	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		//flash_set_i2c_gpio();
		writeReg(LM3643_REG_ENABLE, temp & 0xF0);//close		
		//mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ZERO);		
		//mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ZERO);		
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
			{
			//mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ONE);	
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFA);//torch mode
			}
		else
			{
			//mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ONE);
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFE);//flash mode
			}
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
			{
			//mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ONE);	
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xF9);//torch mode
			}
		else
			{
			//mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ONE);
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFD);//flash mode		
			}
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) & (isMovieMode[m_duty2] == 1))
			{
			//mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ONE);	
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFB);//torch mode
			}
		else
			{
			//mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ONE);
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFF);//flash mode
			}
	}
	return 0;
}
int flashDisable_LM3643_2(void)
{
	flashEnable_LM3643_2();
	return 0;
}


int setDuty_LM3643_2(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty2=duty;

	PK_DBG("setDuty_LM3643_2:m_duty = %d, m_duty2 = %d!\n", m_duty1, m_duty2);
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
		{
			writeReg(LM3643_REG_LED2_TORCH, torchDuty[m_duty2]);
		}
		else
		{
	 	    writeReg(LM3643_REG_LED2_FLASH, flashDuty[m_duty2]);
		}
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
		{
			writeReg(LM3643_REG_LED1_TORCH, torchDuty[m_duty1]);
		}
		else
		{
			writeReg(LM3643_REG_LED1_FLASH, flashDuty[m_duty1]);	
		}		
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) && ((isMovieMode[m_duty2] == 1)))
		{
			writeReg(LM3643_REG_LED1_TORCH, torchDuty[m_duty1]);
			writeReg(LM3643_REG_LED2_TORCH, torchDuty[m_duty2]);
		}
		else
		{
	 	    writeReg(LM3643_REG_LED1_FLASH, flashDuty[m_duty1]);
			writeReg(LM3643_REG_LED2_FLASH, flashDuty[m_duty2]);
		}
	}
	return 0;
}

static int LM3643_chip_init(struct LM3643_chip_data *chip)
{


	return 0;
}
#if 0
static int LM3643_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, LM3643_NAME);
	return 0;
}
#endif
 
static int LM3643_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;
 
	int err = -1;

	PK_DBG("LM3643_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("LM3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3643_chip_init(chip)<0)
		goto err_chip_init;

	LM3643_i2c_client = client;
	if(LM3643_i2c_client != NULL)
		LM3643_i2c_client->addr=LM3643_i2c_client->addr-0x0c;
	PK_DBG("LM3643 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("LM3643 probe is failed\n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}
 

static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3642_of_match[] = {
	{.compatible = "mediatek,strobe_main1"},
	{},
};
#endif
 
static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		   .name = LM3643_NAME,
#ifdef CONFIG_OF
		   .of_match_table = LM3642_of_match,
#endif 
		   },
	.probe	= LM3643_probe,
	//.detect = LM3643_detect,
	.remove   = LM3643_remove,
	.id_table = LM3643_id,
};
 
#if 0
struct LM3643_platform_data LM3643_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3643={ I2C_BOARD_INFO(LM3643_NAME, 0x6f), \
													.platform_data = &LM3643_pdata,};
#endif
static int __init LM3643_init(void)
{
	int ret;
	printk("LM3643_init\n");
	//i2c_register_board_info(2, &i2c_LM3643, 1);
	//i2c_register_board_info(2, &i2c_LM3643, 1);//lenovo.sw wangsx3 use I2C2.change for EVT2
	ret=i2c_add_driver(&LM3643_i2c_driver);
	printk("LM3643_init ret=%d\n",ret);
	return ret;
}

static void __exit LM3643_exit(void)
{
	i2c_del_driver(&LM3643_i2c_driver);
}
module_init(LM3643_init);
module_exit(LM3643_exit);

void flash_set_i2c_gpio(void)
  {
    //PK_DBG("mytest mt_get_gpio_mode I2C2_SDA=%d,I2C2_SCL=%d\n",  mt_get_gpio_mode(GPIO_I2C2_SDA_PIN_FLASH),mt_get_gpio_mode(GPIO_I2C2_SCL_PIN_FLASH));
    //set i2c gpio mode
   mt_set_gpio_mode(GPIO_I2C3_SDA_PIN, GPIO_MODE_01);
   mt_set_gpio_mode(GPIO_I2C3_SCA_PIN, GPIO_MODE_01);
   PK_DBG("mytest mt_get_gpio_in I2C2_SDA=%d,I2C2_SCL=%d\n",  mt_get_gpio_mode(GPIO_I2C3_SDA_PIN),mt_get_gpio_mode(GPIO_I2C3_SCA_PIN));

  }
int init_LM3643(void)
{
	int err;
	PK_DBG(" init_LM3643\n");
	//set i2c gpio mode
        mt_set_gpio_mode(GPIO_I2C3_SDA_PIN, GPIO_MODE_01);
        mt_set_gpio_mode(GPIO_I2C3_SCA_PIN, GPIO_MODE_01);
        PK_DBG("mytest mt_get_gpio_mode I2C2_SDA=%d,I2C2_SCL=%d\n",  mt_get_gpio_mode(GPIO_I2C3_SDA_PIN),mt_get_gpio_mode(GPIO_I2C3_SCA_PIN));
	err =  writeReg(LM3643_REG_ENABLE,0x00);
      err =  writeReg(LM3643_REG_TIMING, 0x1F);
	return err;
}
//digital io
#define CAMERA_POWER_VCAM_IO        PMIC_APP_MAIN_CAMERA_POWER_IO
#define mode_name  "kd_camera_hw"
int FL_Enable(void)
{
/*
	//mt_set_gpio_mode(FLASH_ENABLE, 0);
	//mt_set_gpio_dir(FLASH_ENABLE, GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_ENABLE, GPIO_OUT_ONE);	
	if(torch_flag)
	{
		mt_set_gpio_mode(FLASH_TORCH_ENABLE, 0);
		mt_set_gpio_dir(FLASH_TORCH_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ONE);				
	}
	else
	{
		mt_set_gpio_mode(FLASH_STROBE_ENABLE, 0);
		mt_set_gpio_dir(FLASH_STROBE_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ONE);
	}
     flashEnable_LM3643_2();
	PK_DBG(" FL_Enable line=%d torch_flag=%d \n",__LINE__,torch_flag);
*/
/*wuyt3 add  temp solution  for torch*/
	LED2Closeflag = 1;
    	flashEnable_LM3643_2();
	PK_DBG(" FL_Enable line=%d \n",__LINE__);
/*end*/
	return 0;
}



int FL_Disable(void)
{
/*
	if(torch_flag)
	{
		mt_set_gpio_mode(FLASH_TORCH_ENABLE, 0);
		mt_set_gpio_dir(FLASH_TORCH_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ZERO);		
	}
	else
	{
		mt_set_gpio_mode(FLASH_STROBE_ENABLE, 0);
		mt_set_gpio_dir(FLASH_STROBE_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ZERO);		
	}
*/	
/*wuyt3 add  temp solution  for torch*/
	LED2Closeflag = 1;
  	flashDisable_LM3643_2();
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
/*end*/
    return 0;
}

int FL_dim_duty(unsigned int duty)
{
    setDuty_LM3643_1(duty);
    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}




int FL_Init(void)
{
	//mt_set_gpio_mode(FLASH_ENABLE, 0);
	//mt_set_gpio_dir(FLASH_ENABLE, GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_ENABLE, GPIO_OUT_ONE);	
	
    init_LM3643();

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	PK_DBG("LED1_FL_Uninit!\n");
	//mt_set_gpio_mode(FLASH_ENABLE, 0);
	//mt_set_gpio_dir(FLASH_ENABLE, GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_ENABLE, GPIO_OUT_ZERO);	
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("LED1TimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
			m_duty1 = arg;
    		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
				LED1Closeflag = 0;
    			FL_Enable();
    		}
    		else
    		{
    			LED1Closeflag = 1;
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
		i4RetValue=readReg(arg);
		PK_DBG("  arg=%d,i4RetValue=%d\n",(int)arg,i4RetValue);
    	    break;



		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = false;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);