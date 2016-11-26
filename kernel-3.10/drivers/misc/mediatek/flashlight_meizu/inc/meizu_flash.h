
/*****************************************************************************
 *
 * Filename:
 * ---------
 * 	meizu_flash.h
 *
 * Description:
 * 	M80 LM3643 IC will use this common define 
 * ------------
 * author:lcz@meizu.com
 ****************************************************************************/
#ifndef _MEIZU_FLASH_H
#define _MEIZU_FLASH_H

/*Flash register define*/
#define REG_ENABLE          0x01
#define REG_FLASH_LED1_BR   0x03
#define REG_FLASH_LED2_BR   0x04
#define REG_TORCH_LED1_BR   0x05
#define REG_TORCH_LED2_BR   0x06
#define REG_FLASH_TOUT      0x08
#define REG_FLAG0           0x0a
#define REG_FLAG1           0x0b

#define SINGLE_CONTROL   0

#define I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR    0x63

/*Selection of LED*/
#define BOTH_LED  0
#define COOL_LED  1
#define WARM_LED  2

/*LED light up mode*/
#define TORCH_MODE  true
#define FLASH_MODE  false

/*LED duty define*/
#define DUTY_NUM   21
#define TORCH_NUM  5
#define FACTORY_TEST_DUTY 0

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

extern int gTorchDuty[5];
extern int gFlashDuty[21];

#if 0

/**************************M80 B2 Version,flash+torch mode*****************************************************************************/
//Torch current(mA)x1: 11,23,34,46,57,
//Torch current(mA)x5: 55,113,171,230,289,
static int gTorchDuty[TORCH_NUM]={7,16,24,32,40};

//current(mA)x1: 11,23,34,46,57,70,81,93,105,116,128,140,152,163,175,187,198,210,222,234,245
//current(mA)x5: 55,113,171,230,289,348,406,465,532,582,640,700,758,817,875,934,993,1051,1110,1168,1227
static int gFlashDuty[21]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,0x14};
/**************************************************************************************************************************************/

//M80 B2 Version,torch mode only:
//current(mA)x1: 5,11,16,21,21,26,,93,105,116,128,140,152,163,175,187,198,210,222,234,245
//current(mA)x5: 26,55,82,105,130,289,348,406,465,532,582,640,700,758,817,875,934,993,1051,1110,1168,1227
//static int gFlashDuty[32]={3,7,11,15,19,23,27,31,35,39,43,47,51,55,59,63,67,71,75,79,83,87,91,95,99,103,107,111,115,119,123,127};


/**************************M80 Old Version ****************************************************************************/
//Torch current(mA): 50,75,100,125,150
static int gTorchDuty[5]={0x22,0x33,0x43,0x54,0x66};

//current(mA): 23,46,70,93,115,140,163,187,210,234,257,292,362,397,432,467,491,502,538,572,607,642,677,725,771,818,865,912,959,1020,1078,1136,1200
static int gFlashDuty[32]={0x01,0x3,0x05,0x07,0x09,0x0b,0x0d,0x0f,0x11,0x13,0x15,0x18,0x1b,0x1e,0x21,0x24,0x27,0x2a,0x2d,0x31,0x35,0x39,0x3d,0x40,0x44,0x48,0x4c,0x50,0x55,0x5a,0x5f,0x66};

#endif
	
/*****LM3643 IC No.1, I2C bus:1************/
int Flash_Setduty_IC1(unsigned char led_num,int duty,bool mode);
int Flash_Enable_IC1(unsigned char led_num,bool mode);
int Flash_Disable_IC1(unsigned char led_num);

/*****LM3643 IC No.2, I2C bus:2************/
int Flash_Setduty_IC2(unsigned char led_num,int duty,bool mode);
int Flash_Enable_IC2(unsigned char led_num,bool mode);
int Flash_Disable_IC2(unsigned char led_num);

/*****LM3643 IC No.3, I2C bus:3************/
int Flash_Setduty_IC3(unsigned char led_num,int duty,bool mode);
int Flash_Enable_IC3(unsigned char led_num,bool mode);
int Flash_Disable_IC3(unsigned char led_num);

/*****LM3643 IC No.4, I2C bus:0************/
int Flash_Setduty_IC4(unsigned char led_num,int duty,bool mode);
int Flash_Enable_IC4(unsigned char led_num,bool mode);
int Flash_Disable_IC4(unsigned char led_num);

/*****LM3643 IC No.5, I2C bus:5************/
int Flash_Setduty_IC5(unsigned char led_num,int duty,bool mode);
int Flash_Enable_IC5(unsigned char led_num,bool mode);
int Flash_Disable_IC5(unsigned char led_num);

/*****LM3643 control interface for all IC************/
int Flash_Setduty(unsigned char led_num,int duty,bool mode);
int Flash_Enable(unsigned char led_num,bool mode);
int Flash_Disable(unsigned char led_num);

#endif
