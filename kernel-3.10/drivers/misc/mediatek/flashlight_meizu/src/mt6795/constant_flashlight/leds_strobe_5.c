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
#include <linux/err.h>
#include <linux/errno.h>
#include "kd_flashlight.h"
#include "meizu_flash.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_flashlight_type.h"
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/

#define TAG_NAME "[leds_strobe_5.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
#else
	#define PK_DBG(a, ...)
#endif


#define I2C_STROBE_MAIN_CHANNEL_5        5

/******************************************************************************
 * local variables
******************************************************************************/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif

/*****************************************************************************
Functions
*****************************************************************************/
static struct i2c_client *LM3643_i2c_client = NULL;

static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writting at 0x%02x\n", reg);
	
	return ret;
}

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

//=========================

static int LM3643_chip_init(struct LM3643_chip_data *chip)
{
	return 0;
}

static int LM3643_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	printk("LM3643 IC5 probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG(KERN_ERR  "LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_DBG("LM3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3643_chip_init(chip)<0)
		goto err_chip_init;

	LM3643_i2c_client = client;
    	
   	/*inital led1 register,if bit[7]=1,it will affect led2's value */
	LM3643_write_reg(LM3643_i2c_client, REG_TORCH_LED1_BR, 0x00);
	LM3643_write_reg(LM3643_i2c_client, REG_FLASH_LED1_BR, 0x00);
	printk("LM3643 IC5 Initializing is done -\n");
    return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("LM3643_IC5 probe is failed \n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);


   if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3643_NAME "leds-LM3643-IC5"
static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};


static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		.name  = LM3643_NAME,
	},
	.probe	= LM3643_probe,
	.remove   = LM3643_remove,
	.id_table = LM3643_id,
};

static struct LM3643_platform_data LM3643_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3643={ I2C_BOARD_INFO(LM3643_NAME, I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR), \
													.platform_data = &LM3643_pdata,};

static int __init LM3643_init(void)
{
	i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL_5, &i2c_LM3643, 1);
	return i2c_add_driver(&LM3643_i2c_driver);
}

static void __exit LM3643_exit(void)
{
   i2c_del_driver(&LM3643_i2c_driver);
}


module_init(LM3643_init);
module_exit(LM3643_exit);


/*********************************************
 * Flash LED control function
 *
 * LM3643 IC5
 *********************************************/

int Flash_Setduty_IC5(unsigned char led_num,int duty,bool mode)
{
    int buf[2];
    if(duty>DUTY_NUM-1)
        duty=DUTY_NUM-1;
    if(duty<0)
        duty=0;

 if(led_num == COOL_LED){  //set duty for cool led
    if(mode == TORCH_MODE){
        buf[0]=REG_TORCH_LED1_BR;
    	buf[1]=gTorchDuty[duty];
    }
    else{
    	buf[0]=REG_FLASH_LED1_BR;
    	buf[1]=gFlashDuty[duty];
    }
    LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
 }

 else if(led_num == WARM_LED){  //set duty for warm led
    if(mode == TORCH_MODE){
	buf[0]=REG_TORCH_LED2_BR;
    	buf[1]=gTorchDuty[duty];
    	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
    }
    else{
	buf[0]=REG_FLASH_LED2_BR;
    	buf[1]=gFlashDuty[duty];
    	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
    }
 }
 else{
    	PK_DBG("Flash_Setduty error led selection:%d!!\n",led_num);
	return -1;
 }

    
    PK_DBG("Flash_Setduty led_num:%d,duty:%d,mode:%d\n",led_num,duty,mode);
    return 0;
}

int Flash_Enable_IC5(unsigned char led_num,bool mode)
{
	
    int buf[2];
    
 if(led_num == BOTH_LED){  //Enable both cool and warm led at the same time 
    
    if(mode == TORCH_MODE){
    	buf[1] = 0xfb;
    }
    else{
	buf[1] = 0xff;
    }
 }
 else if(led_num == COOL_LED){  //Enable cool led
    buf[0]=REG_ENABLE;
#if SINGLE_CONTROL
    buf[1]=LM3643_read_reg(LM3643_i2c_client,buf[0]);
    if(mode == TORCH_MODE){
    	buf[1]=buf[1]|0x09;
    }
    else{
	buf[1]=buf[1]|0x0d;
    }
#else
    if(mode == TORCH_MODE){
    	buf[1]=0xf9;
    }
    else{
	buf[1]=0xfd;
    }
#endif
 }
 else if(led_num == WARM_LED){  //Enable warm led
    buf[0]=REG_ENABLE;
#if SINGLE_CONTROL
    buf[1]=LM3643_read_reg(LM3643_i2c_client,buf[0]);
    if(mode == TORCH_MODE){
    	buf[1]=buf[1]|0xa;
    }
    else{
	buf[1]=buf[1]|0xe;
    }
#else
    if(mode == TORCH_MODE){
    	buf[1]=0xfa;
    }
    else{
	buf[1]=0xfe;
    }
#endif
 }
 else{
    	PK_DBG("Flash_Enable error led selection:%d!!\n",led_num);
	return -1;
 }

    LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
    
    PK_DBG("Flash_Enable led_num:%d,mode:%d\n",led_num,mode);
    return 0;
}


int Flash_Disable_IC5(unsigned char led_num)
{
    int buf[2];
    
 if(led_num == BOTH_LED){  //Disable cool led
    buf[1] = 0x00;
 }
 else if(led_num == COOL_LED){  //Disable cool led
    buf[0]=REG_ENABLE;
    buf[1]=LM3643_read_reg(LM3643_i2c_client,buf[0]);
    buf[1]=buf[1]&0xfe;
 }
 else if(led_num == WARM_LED){  //Disable warm led
    buf[0]=REG_ENABLE;
    buf[1]=LM3643_read_reg(LM3643_i2c_client,buf[0]);
    buf[1]=buf[1]&0xfd;
 }
 else{
    PK_DBG("FLash_Disable error led selection:%d!!\n",led_num);
    return -1;
 }
   
    LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
    
    PK_DBG(" FL_Disable led_num:%d\n",led_num);
    return 0;
}

