/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <mach/charging.h>
#include "bq24296.h"
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_charging.h>

#include <mach/upmu_sw.h>
#include <cust_pmic.h>

//Add for build error
#if 1
#define upmu_set_rg_bc11_bias_en mt6332_upmu_set_rg_bc12_bias_en
#define upmu_set_rg_bc11_vsrc_en mt6332_upmu_set_rg_bc12_vsrc_en
#define upmu_set_rg_bc11_vref_vth mt6332_upmu_set_rg_bc12_vref_vth
#define upmu_set_rg_bc11_cmp_en mt6332_upmu_set_rg_bc12_cmp_en
#define upmu_set_rg_bc11_ipd_en mt6332_upmu_set_rg_bc12_ipd_en
#define upmu_set_rg_bc11_ipu_en mt6332_upmu_set_rg_bc12_ipu_en
#define upmu_set_rg_bc11_rst mt6332_upmu_set_rg_bc12_rst
#define upmu_set_rg_bc11_bb_ctrl mt6332_upmu_set_rg_bc12_bb_ctrl
#define upmu_get_rgs_bc11_cmp_out mt6332_upmu_get_rgs_bc12_cmp_out
#define upmu_set_rg_usbdl_rst mt6332_upmu_set_rg_usbdl_en
#define upmu_get_rgs_vcdt_hv_det mt6332_upmu_get_rgs_chr_hv_det

#endif

extern unsigned int get_pmic_mt6332_cid(void);

 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))
#define PSEL_PIN 0

 // ============================================================ //
 //global variable
 // ============================================================ //

#include <cust_gpio_usage.h>
int gpio_number   = GPIO_SWCHARGER_EN_PIN; 
int gpio_off_mode = GPIO_SWCHARGER_EN_PIN_M_GPIO;
int gpio_on_mode  = GPIO_SWCHARGER_EN_PIN_M_GPIO;

#if PSEL_PIN
int gpio_psel_number   = GPIO_CHR_PSEL_PIN; 
int gpio_psel_adapter_mode = GPIO_CHR_PSEL_PIN_M_GPIO;
int gpio_psel_usb_mode  = GPIO_CHR_PSEL_PIN_M_GPIO;
#endif

int gpio_off_dir  = GPIO_DIR_OUT;
int gpio_off_out  = GPIO_OUT_ONE;
int gpio_on_dir   = GPIO_DIR_OUT;
int gpio_on_out   = GPIO_OUT_ZERO;

int gpio_psel_adapter_dir  = GPIO_DIR_OUT;
int gpio_psel_usb_dir   = GPIO_DIR_OUT;
int gpio_psel_adapter_out  = GPIO_OUT_ZERO;
int gpio_psel_usb_out   = GPIO_OUT_ONE;


kal_bool charging_type_det_done = KAL_TRUE;

CHARGER_TYPE bq24296_charging_type =STANDARD_HOST;

const kal_uint32 VBAT_CV_VTH[]=
{
	BATTERY_VOLT_03_500000_V,   BATTERY_VOLT_03_520000_V,	BATTERY_VOLT_03_540000_V,   BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V,   BATTERY_VOLT_03_600000_V,	BATTERY_VOLT_03_620000_V,   BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V,	BATTERY_VOLT_03_680000_V,	BATTERY_VOLT_03_700000_V,	BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V,	BATTERY_VOLT_03_760000_V,	BATTERY_VOLT_03_780000_V,	BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V,	BATTERY_VOLT_03_840000_V,	BATTERY_VOLT_03_860000_V,	BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V,	BATTERY_VOLT_03_920000_V,	BATTERY_VOLT_03_940000_V,	BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V,	BATTERY_VOLT_04_000000_V,	BATTERY_VOLT_04_020000_V,	BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V,	BATTERY_VOLT_04_080000_V,	BATTERY_VOLT_04_100000_V,	BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V,   BATTERY_VOLT_04_160000_V,	BATTERY_VOLT_04_180000_V,   BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V,   BATTERY_VOLT_04_240000_V,	BATTERY_VOLT_04_260000_V,   BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_320000_V,	BATTERY_VOLT_04_340000_V,   BATTERY_VOLT_04_360000_V,	
	BATTERY_VOLT_04_380000_V,   BATTERY_VOLT_04_400000_V,	BATTERY_VOLT_04_420000_V,   BATTERY_VOLT_04_440000_V	
	
};

const kal_uint32 CS_VTH[]=
{
	CHARGE_CURRENT_550_00_MA,   CHARGE_CURRENT_650_00_MA,	CHARGE_CURRENT_750_00_MA, CHARGE_CURRENT_850_00_MA,
	CHARGE_CURRENT_950_00_MA,   CHARGE_CURRENT_1050_00_MA,	CHARGE_CURRENT_1150_00_MA, CHARGE_CURRENT_1250_00_MA
}; 

 const kal_uint32 INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,	 CHARGE_CURRENT_500_00_MA,	 CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_MAX
 }; 

const kal_uint32 BQ24296_SET_VOLTAGE[]=
{
	BATTERY_VOLT_03_500000_V,   BATTERY_VOLT_03_520000_V,	BATTERY_VOLT_03_540000_V,   BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V,   BATTERY_VOLT_03_600000_V,	BATTERY_VOLT_03_620000_V,   BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V,	BATTERY_VOLT_03_680000_V,	BATTERY_VOLT_03_700000_V,	BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V,	BATTERY_VOLT_03_760000_V,	BATTERY_VOLT_03_780000_V,	BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V,	BATTERY_VOLT_03_840000_V,	BATTERY_VOLT_03_860000_V,	BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V,	BATTERY_VOLT_03_920000_V,	BATTERY_VOLT_03_940000_V,	BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V,	BATTERY_VOLT_04_000000_V,	BATTERY_VOLT_04_020000_V,	BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V,	BATTERY_VOLT_04_080000_V,	BATTERY_VOLT_04_100000_V,	BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V,   BATTERY_VOLT_04_160000_V,	BATTERY_VOLT_04_180000_V,   BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V,   BATTERY_VOLT_04_240000_V,	BATTERY_VOLT_04_260000_V,   BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_320000_V,	BATTERY_VOLT_04_340000_V,   BATTERY_VOLT_04_360000_V,	
	BATTERY_VOLT_04_380000_V,   BATTERY_VOLT_04_400000_V	
};

 const kal_uint32 BQ24296_SET_CURRENT[]=
{
	CHARGE_CURRENT_500_00_MA,   CHARGE_CURRENT_550_00_MA,	CHARGE_CURRENT_600_00_MA, CHARGE_CURRENT_650_00_MA,
	CHARGE_CURRENT_700_00_MA,   CHARGE_CURRENT_750_00_MA,	CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_850_00_MA,
	CHARGE_CURRENT_900_00_MA,   CHARGE_CURRENT_950_00_MA,	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1050_00_MA,
	CHARGE_CURRENT_1100_00_MA,   CHARGE_CURRENT_1150_00_MA,	CHARGE_CURRENT_1200_00_MA, CHARGE_CURRENT_1250_00_MA,
	CHARGE_CURRENT_1300_00_MA,   CHARGE_CURRENT_1350_00_MA,	CHARGE_CURRENT_1400_00_MA, CHARGE_CURRENT_1450_00_MA,
	CHARGE_CURRENT_1500_00_MA,   CHARGE_CURRENT_1550_00_MA,	CHARGE_CURRENT_1600_00_MA, CHARGE_CURRENT_1650_00_MA,
	CHARGE_CURRENT_1700_00_MA,   CHARGE_CURRENT_1750_00_MA,	CHARGE_CURRENT_1800_00_MA, CHARGE_CURRENT_1850_00_MA,
	CHARGE_CURRENT_1900_00_MA,   CHARGE_CURRENT_1950_00_MA,	CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_2050_00_MA,
      CHARGE_CURRENT_2100_00_MA,   CHARGE_CURRENT_2150_00_MA,	CHARGE_CURRENT_2200_00_MA, CHARGE_CURRENT_2250_00_MA,
	CHARGE_CURRENT_2300_00_MA,   CHARGE_CURRENT_2350_00_MA,	CHARGE_CURRENT_2400_00_MA, CHARGE_CURRENT_2450_00_MA,
	CHARGE_CURRENT_2500_00_MA,   CHARGE_CURRENT_2550_00_MA,	CHARGE_CURRENT_2600_00_MA, CHARGE_CURRENT_2650_00_MA,
	CHARGE_CURRENT_2700_00_MA,   CHARGE_CURRENT_2750_00_MA,	CHARGE_CURRENT_2800_00_MA, CHARGE_CURRENT_2850_00_MA,
	CHARGE_CURRENT_2900_00_MA,   CHARGE_CURRENT_2950_00_MA,	CHARGE_CURRENT_3000_00_MA
 };


 const kal_uint32 BQ24296_SET_INPUT_CURRENT[]=
 {
	 CHARGE_CURRENT_100_00_MA,	 CHARGE_CURRENT_150_00_MA,	 CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_900_00_MA,
        CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1500_00_MA,CHARGE_CURRENT_2000_00_MA,CHARGE_CURRENT_3000_00_MA 	 
 }; 


 const kal_uint32 VCDT_HV_VTH[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };

 // ============================================================ //
 // function prototype
 // ============================================================ //
 
 
 // ============================================================ //
 //extern variable
 // ============================================================ //
 
 // ============================================================ //
 //extern function
 // ============================================================ //
 extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
 extern bool mt_usb_is_device(void);
 extern void Charger_Detect_Init(void);
 extern void Charger_Detect_Release(void);
 extern void mt_power_off(void);
 
 // ============================================================ //
 kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
 kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
		{
				return i;
		}
	}

    battery_xlog_printk(BAT_LOG_CRTI, "NO register value match \r\n");
	//TODO: ASSERT(0);	// not find the value
	return 0;
}


 static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
 {
    /*this function is to find the closest value in the array.    modified by kolery 2014.09.12*/
    
	 kal_uint32 i;
	 kal_uint32 max_value_in_last_element;
 
	 if(pList[0] < pList[1])
		 max_value_in_last_element = KAL_TRUE; //array is in ascending order 
	 else
		 max_value_in_last_element = KAL_FALSE; //array is in descending order 
 
	 if(max_value_in_last_element == KAL_TRUE)
	 {//max value in the last element
             if(level > pList[number-1])
			 {
				 return pList[number-1] ;
			 }
             
		 for(i = (number-1); i >=1; i--)	 
		 {
		        if(level == pList[i])
			 {
				 return pList[i];          
			 }	                    
			 else if((level < pList[i]) && (level > pList[i -1]))
			 {
				 return pList[i];          
			 }	  
		 }

              if(level <= pList[0])
			 {
				 return pList[0] ;
			 }
	
	 }
	 else
	 {//max value in the first element
             if(level > pList[0])
			 {
				 return pList[0] ;
			 }
             
		 for(i = 0; i <= number-2; i++)	 
		 {
		        if(level == pList[i])
			 {
				 return pList[i];          
			 }
			 else if((level < pList[i]) && (level > pList[i + 1]))
			 {   
				 return pList[i];
			 }	  
		 }

              if(level <= pList[number - 1])
			 {
				 return pList[number - 1] ;
			 }
	
	 }
 }


static void hw_bc11_dump_register(void)
{
	/*kal_uint32 reg_val = 0;
	kal_uint32 reg_num = CHR_CON18;
	kal_uint32 i = 0;

	for(i=reg_num ; i<=CHR_CON19 ; i+=2)
	{
		reg_val = upmu_get_reg_value(i);
		battery_xlog_printk(BAT_LOG_FULL, "Chr Reg[0x%x]=0x%x \r\n", i, reg_val);
	}*/
}


 static void hw_bc11_init(void)
 {
 	 msleep(300);
	 Charger_Detect_Init();
		 
	 //RG_BC11_BIAS_EN=1	
	 upmu_set_rg_bc11_bias_en(0x1);
	 //RG_BC11_VSRC_EN[1:0]=00
	 upmu_set_rg_bc11_vsrc_en(0x0);
	 //RG_BC11_VREF_VTH = [1:0]=00
	 upmu_set_rg_bc11_vref_vth(0x0);
	 //RG_BC11_CMP_EN[1.0] = 00
	 upmu_set_rg_bc11_cmp_en(0x0);
	 //RG_BC11_IPU_EN[1.0] = 00
	 upmu_set_rg_bc11_ipu_en(0x0);
	 //RG_BC11_IPD_EN[1.0] = 00
	 upmu_set_rg_bc11_ipd_en(0x0);
	 //BC11_RST=1
	 upmu_set_rg_bc11_rst(0x1);
	 //BC11_BB_CTRL=1
	 upmu_set_rg_bc11_bb_ctrl(0x1);
 
 	 //msleep(10);
 	 mdelay(50);

	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
    		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_init() \r\n");
		hw_bc11_dump_register();
	 }	
	 
 }
 
 
 static U32 hw_bc11_DCD(void)
 {
	 U32 wChargerAvail = 0;
 
	 //RG_BC11_IPU_EN[1.0] = 10
	 upmu_set_rg_bc11_ipu_en(0x2);
	 //RG_BC11_IPD_EN[1.0] = 01
	 upmu_set_rg_bc11_ipd_en(0x1);
	 //RG_BC11_VREF_VTH = [1:0]=01
	 upmu_set_rg_bc11_vref_vth(0x1);
	 //RG_BC11_CMP_EN[1.0] = 10
	 upmu_set_rg_bc11_cmp_en(0x2);
 
	 //msleep(20);
	 mdelay(80);

 	 wChargerAvail = upmu_get_rgs_bc11_cmp_out();
	 
	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_DCD() \r\n");
		hw_bc11_dump_register();
	 }
	 
	 //RG_BC11_IPU_EN[1.0] = 00
	 upmu_set_rg_bc11_ipu_en(0x0);
	 //RG_BC11_IPD_EN[1.0] = 00
	 upmu_set_rg_bc11_ipd_en(0x0);
	 //RG_BC11_CMP_EN[1.0] = 00
	 upmu_set_rg_bc11_cmp_en(0x0);
	 //RG_BC11_VREF_VTH = [1:0]=00
	 upmu_set_rg_bc11_vref_vth(0x0);
 
	 return wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepA1(void)
 {
	U32 wChargerAvail = 0;

	//RG_BC11_IPD_EN[1:0] = 01
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH[1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
	// RG_BC11_CMP_EN[1:0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepB1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	//upmu_set_rg_bc11_ipu_en(0x1);
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	//upmu_set_rg_bc11_vref_vth(0x2);
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepC1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepC1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepA2(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_VSRC_EN[1.0] = 10 
	upmu_set_rg_bc11_vsrc_en(0x2);
	//RG_BC11_IPD_EN[1:0] = 01
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA2() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepB2(void)
 {
	U32 wChargerAvail = 0;
 
	//RG_BC11_IPU_EN[1:0]=10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x1);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB2() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static void hw_bc11_done(void)
 {
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=0
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_BIAS_EN=0
	upmu_set_rg_bc11_bias_en(0x0); 
 
	Charger_Detect_Release();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_done() \r\n");
		hw_bc11_dump_register();
	}
    
 }


 static kal_uint32 charging_hw_init(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	static bool charging_init_flag = KAL_FALSE;

	

    printk("[bq24296]  charging_hw_init\n");
	
    //upmu_set_rg_bc11_bb_ctrl(1);    //BC11_BB_CTRL   
    //upmu_set_rg_bc11_rst(1);        //BC11_RST
	
	//upmu_set_rg_usbdl_set(0);       //force leave USBDL mode
	upmu_set_rg_usbdl_rst(1);		//force leave USBDL mode

       mt_set_gpio_mode(gpio_number,gpio_on_mode);  
	mt_set_gpio_dir(gpio_number,gpio_on_dir);
	mt_set_gpio_out(gpio_number,gpio_on_out);
		
	 bq24296_set_en_hiz(0x00);

	if(STANDARD_CHARGER == bq24296_charging_type)
   	{//Adapter Charging
    	bq24296_set_iinlim(0x6); //IN current limit at 2A    
    	bq24296_set_ichg(0x18);  //Fast Charging Current Limit at 2A
	}
	else if(STANDARD_HOST == bq24296_charging_type)
   	{//USB Charging
    	bq24296_set_iinlim(0x2); //IN current limit at 500mA
    	bq24296_set_ichg(0x0);  //Fast Charging Current Limit at 500mA
	}

        bq24296_set_vindpm(0x8); //VIN DPM check 4.52V
        bq24296_set_reg_rst(0x0);
        bq24296_set_wdt_rst(0x1); //kick watchdog	
        bq24296_set_sys_min(0x5); //Minimum system voltage 3.5V	
        bq24296_set_iprechg(0x2); //preCharge Current limit 256mA
        bq24296_set_iterm(0x1); //Termination Current limit 256mA
        bq24296_set_BHot(0x2);//boost mode thermal protection 65 degrees
        bq24296_set_vreg(0x35);//Charge Voltage Limit 4.35V  
        bq24296_set_batlowv(0x1); //BATLOWV 3.0V
        bq24296_set_vrechg(0x0); //VRECHG 0.1V 
        bq24296_set_en_term(0x1); //Enable termination
        bq24296_set_watchdog(0x1); //WDT 40s
        bq24296_set_en_timer(0x1); //Enable charge timer
        bq24296_set_chg_timer(0x2);//Fast Chare Timer Setting 12hrs
        bq24296_set_int_mask(0x0); //Disable fault interrupt
    
	return status;
 }


 static kal_uint32 charging_dump_register(void *data)
 {
 	kal_uint32 status = STATUS_OK;

	bq24296_dump_register();
   	
	return status;
 }	


 static kal_uint32 charging_enable(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);
		
	if(KAL_TRUE == enable)
	{		
	 	bq24296_set_en_hiz(0x00);

        	bq24296_set_chg_config(0x01);   //charging enable

		printk("[bq24296]  charging_enable TRUE");
	}
    
	else if(KAL_FALSE == enable)
	{
		#if defined(CONFIG_USB_MTK_HDRC_HCD)
   			if(mt_usb_is_device())
		#endif 			

        	bq24296_set_chg_config(0x00);   //charging disable

            printk("[bq24296]  charging_enable FALSE");
	}
		
	return status;
 }


 static kal_uint32 charging_set_cv_voltage(void *data)
 {
 	kal_uint32 status = STATUS_OK;
      kal_uint32 set_chr_voltage;
	kal_uint32 set_voltage_register_value;
	kal_uint32 array_size;

	array_size = GETARRAYNUM(BQ24296_SET_VOLTAGE);
	set_chr_voltage = bmt_find_closest_level(BQ24296_SET_VOLTAGE, array_size, *(kal_uint32 *)data);

    printk("[bq24296]charging_set_cv_voltage, set_chr_voltage =%d\n", set_chr_voltage / 1000);

     if(BATTERY_VOLT_03_500000_V == set_chr_voltage)
     {//  3.504V
        printk("[bq24296]charging_set_cv_voltage, 3.504V\n");
        set_voltage_register_value = 0x00;
     }
     else
     {//  3.504V ~ 4.4V        
       set_voltage_register_value = (set_chr_voltage / 1000  - 3504) / 16; //0x00 ~ 0x38
     }
             
     printk("[bq24296]charging_set_current, REG04[7:2]=0x%x\n", set_voltage_register_value);
    
     bq24296_set_vreg(set_voltage_register_value); 
    
	return status;
 } 	


 static kal_uint32 charging_get_current(void *data)
 {
    kal_uint32 status = STATUS_OK;
    kal_uint32 array_size;
    kal_uint8 reg_value;
	
    //Get current level
    array_size = GETARRAYNUM(CS_VTH);
    bq24296_read_interface(0x1, &reg_value, 0x3, 0x6);	//IINLIM
    *(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
	
    return status;
 }  
  


 static kal_uint32 charging_set_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
      kal_uint32 set_chr_current;
	kal_uint32 set_current_register_value;
	kal_uint32 array_size;

	array_size = GETARRAYNUM(BQ24296_SET_CURRENT);
	set_chr_current = bmt_find_closest_level(BQ24296_SET_CURRENT, array_size, *(kal_uint32 *)data);

    printk("[bq24296]charging_set_current, set_chr_current=%d\n",set_chr_current / 100);

     if(CHARGE_CURRENT_500_00_MA == set_chr_current)
     {//500mA  
        printk("[bq24296]charging_set_current, 500mA\n");
        set_current_register_value = 0x00;
     }
     else
     {//500 ~ 3000mA        
        if(set_chr_current > CHARGE_CURRENT_2000_00_MA)
        {
            #ifdef KAITO_PROJECT
            set_chr_current = CHARGE_CURRENT_2000_00_MA;//500 ~ 2000mA  for kaito
            #endif
        }
             
        set_current_register_value = (set_chr_current / 100  - 512) / 64; //0x00 ~ 0x26, for kaito : 0x00 ~ 0x23

        printk("[bq24296]charging_set_current, REG02[7:2]=0x%x\n",set_current_register_value);
     }

	bq24296_set_ichg(set_current_register_value); 
    
	return status;
 } 	
  

 
 static kal_uint32 charging_set_input_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_input_current;
      kal_uint32 temp;
	kal_uint32 array_size;

    	array_size = GETARRAYNUM(BQ24296_SET_INPUT_CURRENT);
    	set_chr_input_current = bmt_find_closest_level(BQ24296_SET_INPUT_CURRENT, array_size, *(kal_uint32 *)data);

         if(set_chr_input_current < CHARGE_CURRENT_500_00_MA)
        {//100  150  500
            set_chr_input_current = CHARGE_CURRENT_500_00_MA;//min 500mA
        }
         
        if(set_chr_input_current > CHARGE_CURRENT_2000_00_MA)
        {
            #ifdef KAITO_PROJECT
            set_chr_input_current = CHARGE_CURRENT_2000_00_MA;//max 2000mA for kaito
            #endif
        }
        
      printk("[bq24296]charging_set_input_current, set_chr_input_current=%d mA\n",set_chr_input_current / 100);
      for(temp = 0; temp < array_size; temp++)
      {
           if(BQ24296_SET_INPUT_CURRENT[temp] == set_chr_input_current)
           {printk("[bq24296]charging_set_input_current, REG00[2:0]=0x%x\n",temp);
            bq24296_set_iinlim(temp);  
            break;
           }
      } 

     return status;
 } 	


 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = bq24296_get_chrg_stat();
	
	if(ret_val == 0x3) //Charge Termination Done
		*(kal_uint32 *)data = KAL_TRUE;
	else
		*(kal_uint32 *)data = KAL_FALSE;
	
	return status;
 } 	


 static kal_uint32 charging_reset_watch_dog_timer(void *data)
 {
	 kal_uint32 status = STATUS_OK;
 
	 bq24296_set_wdt_rst(1);
	 
	 return status;
 }
 
 
  static kal_uint32 charging_set_hv_threshold(void *data)
  {
	 kal_uint32 status = STATUS_OK;
 	/*
	 kal_uint32 set_hv_voltage;
	 kal_uint32 array_size;
	 kal_uint16 register_value;
	 kal_uint32 voltage = *(kal_uint32*)(data);
	 
	 array_size = GETARRAYNUM(VCDT_HV_VTH);
	 set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	 register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
	 upmu_set_rg_vcdt_hv_vth(register_value);
 	*/
	 return status;
  }
 
 
  static kal_uint32 charging_get_hv_status(void *data)
  {
  	kal_uint32 status = STATUS_OK;
 	#if 0
	   *(kal_bool*)(data) = upmu_get_rgs_vcdt_hv_det();
	#else
	   	kal_uint32 val=0;
		#if defined(CONFIG_POWER_EXT)
    		*(kal_bool*)(data) = 0;
		#else
    		if(get_pmic_mt6332_cid()==PMIC6332_E1_CID_CODE)
		    {
		        *(kal_bool*)(data) = 0;
		    }
		    else
		    {
		        val= mt6332_upmu_get_rgs_chr_hv_det();
		        *(kal_bool*)(data) = val;
		    }
		#endif
    	if(val==1)
    	{
        	battery_log(BAT_LOG_CRTI,"[charging_get_hv_status] HV detected by HW (%d)\n", val);
    	}
	#endif   
	
	return status;
  }


 static kal_uint32 charging_get_battery_status(void *data)
 {
	   kal_uint32 status = STATUS_OK;
 	#if 0
 	   upmu_set_baton_tdet_en(1);	
	   upmu_set_rg_baton_en(1);
	   *(kal_bool*)(data) = upmu_get_rgs_baton_undet();
	#else
	   kal_uint32 ret=0;

    	pmic_config_interface(MT6332_BATON_CON0, 0x1, MT6332_PMIC_RG_BATON_EN_MASK, MT6332_PMIC_RG_BATON_EN_SHIFT);
    	pmic_config_interface(MT6332_TOP_CKPDN_CON0_CLR, 0x80C0, 0xFFFF, 0); //enable BIF clock
    	pmic_config_interface(MT6332_LDO_CON2, 0x1, MT6332_PMIC_RG_VBIF28_EN_MASK, MT6332_PMIC_RG_VBIF28_EN_SHIFT);

    	mdelay(1);
	    ret = mt6332_upmu_get_bif_bat_lost();
	    if(ret == 0)
	    {
	        *(kal_bool*)(data) = 0; // battery exist
	        battery_log(BAT_LOG_FULL,"[charging_get_battery_status] battery exist.\n");
	    }
	    else
	    {
	        *(kal_bool*)(data) = 1; // battery NOT exist
	        battery_log(BAT_LOG_CRTI,"[charging_get_battery_status] battery NOT exist.\n");
	    }
	#endif

	   return status;
 }


 static kal_uint32 charging_get_charger_det_status(void *data)
 {
	kal_uint32 status = STATUS_OK;
 
	#if 0
	  // *(kal_bool*)(data) = upmu_get_rgs_chrdet();
	#else
		kal_uint32 val=0;
		pmic_config_interface(0x10A, 0x1, 0xF, 8);
		pmic_config_interface(0x10A, 0x17,0xFF,0);
		pmic_read_interface(0x108,	 &val,0x1, 1);
		*(kal_bool*)(data) = val;
		battery_log(BAT_LOG_CRTI,"[charging_get_charger_det_status][JJP][20140401] CHRDET status = %d\n", val);
		/*	if(val == 0)
			g_charger_type = CHARGER_UNKNOWN;*/
	#endif   
	return status;
 }


kal_bool charging_type_detection_done(void)
{
	 return charging_type_det_done;
}


 static kal_uint32 charging_get_charger_type(void *data)
 {
	 kal_uint32 status = STATUS_OK;
#if defined(CONFIG_POWER_EXT)
	 *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else    
	charging_type_det_done = KAL_FALSE;

	/********* Step initial  ***************/		 
	hw_bc11_init();
 
	/********* Step DCD ***************/  
	if(1 == hw_bc11_DCD())
	{
		/********* Step A1 ***************/
		if(1 == hw_bc11_stepA1())
		{
			*(CHARGER_TYPE*)(data) = APPLE_2_1A_CHARGER;
			battery_xlog_printk(BAT_LOG_CRTI, "step A1 : Apple 2.1A CHARGER!\r\n");
		}	 
		else
		{
			*(CHARGER_TYPE*)(data) = NONSTANDARD_CHARGER;
			battery_xlog_printk(BAT_LOG_CRTI, "step A1 : Non STANDARD CHARGER!\r\n");
		}
	}
	else
	{
		 /********* Step A2 ***************/
		 if(1 == hw_bc11_stepA2())
		 {
			 /********* Step B2 ***************/
			 if(1 == hw_bc11_stepB2())
			 {    
			    #if PSEL_PIN
			    mt_set_gpio_mode(gpio_psel_number,gpio_psel_adapter_mode);  
				mt_set_gpio_dir(gpio_psel_number,gpio_psel_adapter_dir);
				mt_set_gpio_out(gpio_psel_number,gpio_psel_adapter_out);
				#endif
				 *(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
				 battery_xlog_printk(BAT_LOG_CRTI, "step B2 : STANDARD CHARGER!\r\n");
			 }
			 else
			 {
			 		#if PSEL_PIN
			        mt_set_gpio_mode(gpio_psel_number,gpio_psel_usb_mode);  
			        mt_set_gpio_dir(gpio_psel_number,gpio_psel_usb_dir);
			        mt_set_gpio_out(gpio_psel_number,gpio_psel_usb_out);
			        #endif
				 *(CHARGER_TYPE*)(data) = CHARGING_HOST;
				 battery_xlog_printk(BAT_LOG_CRTI, "step B2 :  Charging Host!\r\n");
			 }
		 }
		 else
		 {
		 	#if PSEL_PIN
		    mt_set_gpio_mode(gpio_psel_number,gpio_psel_usb_mode);  
			mt_set_gpio_dir(gpio_psel_number,gpio_psel_usb_dir);
			mt_set_gpio_out(gpio_psel_number,gpio_psel_usb_out);
			#endif
			*(CHARGER_TYPE*)(data) = STANDARD_HOST;
			battery_xlog_printk(BAT_LOG_CRTI, "step A2 : Standard USB Host!\r\n");
		 }
 
	}

	bq24296_charging_type = *(CHARGER_TYPE*)(data);
 
	 /********* Finally setting *******************************/
	 hw_bc11_done();
	printk("[bq24296][%s]bq24296_charging_type=%d\n",__func__,bq24296_charging_type);
 	charging_type_det_done = KAL_TRUE;
#endif
	 return status;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
    kal_uint32 status = STATUS_OK;

    if(slp_get_wake_reason() == WR_PCM_TIMER)
        *(kal_bool*)(data) = KAL_TRUE;
    else
        *(kal_bool*)(data) = KAL_FALSE;

    battery_xlog_printk(BAT_LOG_CRTI, "[bq24296]slp_get_wake_reason=%d\n", slp_get_wake_reason());
       
    return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
    kal_uint32 status = STATUS_OK;

    battery_xlog_printk(BAT_LOG_CRTI, "[bq24296]charging_set_platform_reset\n");
 
    arch_reset(0,NULL);
        
    return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    *(kal_uint32*)(data) = get_boot_mode();

    battery_xlog_printk(BAT_LOG_CRTI, "[bq24296]get_boot_mode=%d\n", get_boot_mode());
         
    return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    battery_xlog_printk(BAT_LOG_CRTI, "[bq24296]charging_set_power_off=\n");
    mt_power_off();
         
    return status;
}

 static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
 };

 
 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
 kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 kal_int32 status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
 
	 return status;
 }


