/*****************************************************************************
 * 
 * Reversed or Xiaomi Redmi Note 3 MTK by:
 * --------- 
 * 	Smosia 
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
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_gpio_usage.h>
#include <cust_charging.h>
#include "bq24296.h"

// ============================================================ //
// Define
// ============================================================ //
#define CONFIG_USB_MTK_HDRC_HCD
#define HIGH_BATTERY_VOLTAGE_SUPPORT
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

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
// Global variable
// ============================================================ //
//static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

kal_bool charging_type_det_done = KAL_TRUE;

CHARGER_TYPE bq24296_charging_type = STANDARD_HOST;

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
	51200,  57600,  64000,  70400,
	76800,  83200,  89600,  96000,
	102400, 108800, 115200, 121600,
	128000, 134400, 140800, 147200,
	153600, 160000, 166400, 172800,
	179200, 185600, 192000, 198400,
	204800, 211200, 217600, 224000
}; 

const kal_uint32 INPUT_CS_VTH[]=
{
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_150_00_MA, CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1500_00_MA, CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_MAX
}; 

const kal_uint32 VCDT_HV_VTH[]=
{
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
};

/*#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#define SW_POLLING_PERIOD 100 //100 ms
#define MSEC_TO_NSEC(x)		(x * 1000000UL)

static DEFINE_MUTEX(diso_polling_mutex);
static DECLARE_WAIT_QUEUE_HEAD(diso_polling_thread_wq);
static struct hrtimer diso_kthread_timer;
static kal_bool diso_thread_timeout = KAL_FALSE;
static struct delayed_work diso_polling_work;
static void diso_polling_handler(struct work_struct *work);
static DISO_Polling_Data DISO_Polling;
#else
DISO_IRQ_Data DISO_IRQ;
#endif
int g_diso_state  = 0;
int vin_sel_gpio_number   = (CUST_GPIO_VIN_SEL | 0x80000000); 

static char *DISO_state_s[8] = {
  "IDLE",
  "OTG_ONLY",
  "USB_ONLY",
  "USB_WITH_OTG",
  "DC_ONLY",
  "DC_WITH_OTG",
  "DC_WITH_USB",
  "DC_USB_OTG",
};
#endif*/

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
extern int hw_charging_get_charger_type(void);
extern void mt_power_off(void);
extern kal_uint32 mt6311_get_chip_id(void);
extern int is_mt6311_exist(void);
extern int is_mt6311_sw_ready(void);

static kal_uint32 charging_error = false;
static kal_uint32 charging_get_error_state(void);
static kal_uint32 charging_set_error_state(void *data);
// ============================================================ //
kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		pr_notice("Can't find the parameter \r\n");	
		return parameter[0];
	}
}

kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

	pr_info("array_size = %d \r\n", array_size);

	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
			return i;
	}

	pr_notice("NO register value match. val=%d\r\n", val);

	return 0;
}

static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
{
	kal_uint32 i;
	kal_uint32 max_value_in_last_element;

	if(pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if(max_value_in_last_element == KAL_TRUE)
	{
		for(i = (number-1); i != 0; i--)	 //max value in the last element
		{
			if(pList[i] <= level)
				return pList[i];
			pr_notice("current plist number %i\n", pList[i]);
		}

		pr_notice("Can't find closest level, small value first, number %i, level %i \r\n", number, level);
		return pList[0];
	}
	else
	{
		for(i = 0; i< number; i++)  // max value in the first element
		{
			if(pList[i] <= level)
				return pList[i];
		}

		pr_notice("Can't find closest level, large value first \r\n"); 	 
		return pList[number -1];
	}
}

static kal_uint32 charging_hw_init(void *data)
{
	kal_uint32 status = STATUS_OK;

	#if defined NT35596_FHD_TIANMA_PHANTOM
	pr_notice("tiama driver 1\n");
	#endif

	mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN,GPIO_MODE_GPIO);  
	mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN,GPIO_OUT_ZERO);    

	bq24296_set_en_hiz(0x0); //High impedance stata off

	bq24296_set_vindpm(0x9); //VIN DPM check 4.6V
	bq24296_set_reg_rst(0x0);
	bq24296_set_wdt_rst(0x1); //Kick watchdog	
	bq24296_set_sys_min(0x5); //Minimum system voltage 3.5V	
	bq24296_set_iprechg(0x0); //Precharge current 128mA
	bq24296_set_iterm(0x0); //Termination current 128mA
	bq24296_set_otg_config(0x0); 
	bq24296_set_vreg(0x38); //VREG 4.4V

	bq24296_set_batlowv(0x1); //BATLOWV 3.0V
	bq24296_set_vrechg(0x0); //VRECHG 0.1V (4.108V) (4.3V)
	bq24296_set_en_term(0x1); //Enable termination
	bq24296_set_watchdog(0x1); //WDT 40s
	bq24296_set_en_timer(0x0); //Disable charge timer
	bq24296_set_int_mask(0x0); //Disable fault interrupt

	return status;
}

static kal_uint32 charging_dump_register(void *data)
{
	pr_notice("charging_dump_register\r\n");

	bq24296_dump_register();

	return STATUS_OK;
}	

static kal_uint32 charging_enable(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);
	kal_uint32 bootmode = 0;

	if(KAL_TRUE == enable)
	{
		bq24296_set_en_hiz(0x0);	            	
		bq24296_set_chg_config(0x1); // charger enable
		pr_notice("[charging_enable] KAL_TRUE == enable\n");
	}
	else
	{
		if(mt_usb_is_device())
		{
			bq24296_set_chg_config(0x0);
			if (charging_get_error_state()) {
				pr_notice("[charging_enable] bq24296_set_en_hiz(0x1)\n");
				bq24296_set_en_hiz(0x1);	// disable power path
			}
		}

		bootmode = get_boot_mode();
		if ((bootmode == META_BOOT) || (bootmode == ADVMETA_BOOT))
		{
			pr_notice("[charging_enable] (bootmode == META_BOOT) || (bootmode == ADVMETA_BOOT)\n");
			bq24296_set_iinlim(0x0);
			//bq24296_set_en_hiz(0x1);
		}

	}

	return status;
}

static kal_uint32 charging_set_cv_voltage(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 array_size;
	kal_uint32 set_cv_voltage;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);	
	static kal_int16 pre_register_value = -1;

	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	if(cv_value >= BATTERY_VOLT_04_400000_V)
		cv_value = 4400000;
	#else
	//use nearest value
	if(BATTERY_VOLT_04_200000_V == cv_value)
		cv_value = 4208000;
	#endif

	array_size = GETARRAYNUM(VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv_value);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size, set_cv_voltage);
	bq24296_set_vreg(register_value); 
	pr_notice("[charging_set_cv_voltage] set voltage %i, register value %i\n", set_cv_voltage, register_value);
	//for jeita recharging issue
	if (pre_register_value != register_value)
	{
		bq24296_set_chg_config(1);
		pr_notice("[charging_set_cv_voltage] pre_register_value != register_value \n");
	}

	pre_register_value = register_value;

	return status;
}

static kal_uint32 charging_get_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	//kal_uint32 array_size;
	//kal_uint8 reg_value;

	kal_uint8 ret_val=0;    
	kal_uint8 ret_force_20pct=0;

	//Get current level
	bq24296_read_interface(bq24296_CON2, &ret_val, CON2_ICHG_MASK, CON2_ICHG_SHIFT);

	//Get Force 20% option
	bq24296_read_interface(bq24296_CON2, &ret_force_20pct, CON2_FORCE_20PCT_MASK, CON2_FORCE_20PCT_SHIFT);
	pr_notice("[charging_get_current] ret_val %i, ret_val_20_perc %i\n", ret_val, ret_force_20pct);
	//Parsing
	ret_val = (ret_val*64) + 512;

	if (ret_force_20pct == 0)
	{
		//Get current level
		//array_size = GETARRAYNUM(CS_VTH);
		//*(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
		*(kal_uint32 *)data = ret_val;
	}   
	else
	{
		//Get current level
		//array_size = GETARRAYNUM(CS_VTH_20PCT);
		//*(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
		//return (int)(ret_val<<1)/10;
		*(kal_uint32 *)data = (int)(ret_val<<1)/10;
	}

	pr_notice("[charging_get_current] out data =  %i\n", *(kal_uint32 *)data);
	
	return status;
}  

static kal_uint32 charging_set_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = *(kal_uint32 *)data;

	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
	bq24296_set_ichg(register_value);
	pr_notice("[charging_set_current] set current %i, register value %i\n", set_chr_current, register_value);

	return status;
} 	

static kal_uint32 charging_set_input_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 current_value = *(kal_uint32 *)data;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);
	
	bq24296_set_iinlim(register_value);
	pr_notice("[charging_set_input_current] set input current %i, register value %i\n", set_chr_current, register_value);

	return status;
} 	

static kal_uint32 charging_get_charging_status(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = bq24296_get_chrg_stat();

	if(ret_val == 0x3)
		*(kal_uint32 *)data = KAL_TRUE;
	else
		*(kal_uint32 *)data = KAL_FALSE;

	return status;
} 	

static kal_uint32 charging_reset_watch_dog_timer(void *data)
{
	kal_uint32 status = STATUS_OK;

	pr_info("charging_reset_watch_dog_timer\r\n");

	bq24296_set_wdt_rst(0x1); //Kick watchdog

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
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH,register_value);
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

	return status;
}

static kal_uint32 charging_get_charger_det_status(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 val=0;
	pmic_config_interface(0x10A, 0x1, 0xF, 8);
	pmic_config_interface(0x10A, 0x17,0xFF,0);
	pmic_read_interface(0x108,	 &val,0x1, 1);
	*(kal_bool*)(data) = val;
	battery_log(BAT_LOG_CRTI,"[charging_get_charger_det_status][JJP][20140401] CHRDET status = %d\n", val);
	if(val == 0)
		bq24296_charging_type = CHARGER_UNKNOWN;

	return status;
}

kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
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

static kal_uint32 charging_get_charger_type(void *data)
{
	kal_uint32 status = STATUS_OK;  

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
				 *(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
				 battery_xlog_printk(BAT_LOG_CRTI, "step B2 : STANDARD CHARGER!\r\n");
			 }
			 else
			 {
				 *(CHARGER_TYPE*)(data) = CHARGING_HOST;
				 battery_xlog_printk(BAT_LOG_CRTI, "step B2 :  Charging Host!\r\n");
			 }
		 }
		 else
		 {
			*(CHARGER_TYPE*)(data) = STANDARD_HOST;
			battery_xlog_printk(BAT_LOG_CRTI, "step A2 : Standard USB Host!\r\n");
		 }
 
	}

	bq24296_charging_type = *(CHARGER_TYPE*)(data);
 
	/********* Finally setting *******************************/
	hw_bc11_done();
	printk("[bq24296][%s]bq24296_charging_type=%d\n",__func__,bq24296_charging_type);
 	charging_type_det_done = KAL_TRUE;
	
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

static kal_uint32 charging_get_power_source(void *data)
{
	kal_uint32 status = STATUS_OK;

	*(kal_bool *)data = KAL_FALSE;

	return status;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	kal_uint32 increase = *(kal_uint32*)(data);
	kal_uint32 charging_status = KAL_FALSE;

	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_400000_V;
	#else
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_200000_V;
	#endif

	charging_get_charging_status(&charging_status);
	pr_notice("[charging_set_ta_current_pattern] start charging_status=%i\n",charging_status);
	if(KAL_FALSE == charging_status)
	{
		pr_notice("[charging_set_ta_current_pattern] KAL_FALSE == charging_status\n");
		charging_set_cv_voltage(&cv_voltage);  //Set CV 
		bq24296_set_ichg(0x0);  //Set charging current 500ma
		bq24296_set_chg_config(0x1);  //Enable Charging
	}

	if(increase == KAL_TRUE)
	{
		pr_notice("[charging_set_ta_current_pattern] increase == KAL_TRUE\n");
		bq24296_set_iinlim(0x0); /* 100mA */
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_increase() on 1");
		msleep(85);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_increase() off 1");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_increase() on 2");
		msleep(85);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_increase() off 2");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_increase() on 3");
		msleep(281);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_increase() off 3");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_increase() on 4");
		msleep(281);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_increase() off 4");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_increase() on 5");
		msleep(281);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_increase() off 5");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_increase() on 6");
		msleep(485);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_increase() off 6");
		msleep(50);

		pr_notice("mtk_ta_increase() end \n");

		bq24296_set_iinlim(0x2); /* 500mA */
		msleep(200);
	} else {
		bq24296_set_iinlim(0x0); /* 100mA */
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_decrease() on 1");
		msleep(281);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_decrease() off 1");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_decrease() on 2");
		msleep(281);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_decrease() off 2");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_decrease() on 3");
		msleep(281);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_decrease() off 3");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_decrease() on 4");
		msleep(85);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_decrease() off 4");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_decrease() on 5");
		msleep(85);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_decrease() off 5");
		msleep(85);

		bq24296_set_iinlim(0x2); /* 500mA */
		pr_info("mtk_ta_decrease() on 6");
		msleep(485);

		bq24296_set_iinlim(0x0); /* 100mA */
		pr_info("mtk_ta_decrease() off 6");
		msleep(50);

		pr_notice("mtk_ta_decrease() end \n");

		bq24296_set_iinlim(0x2); /* 500mA */
	}

	return STATUS_OK;
}

static kal_uint32 charging_diso_init(void *data)
{
	kal_uint32 status = STATUS_OK;

	return status;	
}

static kal_uint32 charging_get_diso_state(void *data)
{
	kal_uint32 status = STATUS_OK;

	return status;
}

static kal_uint32 charging_get_error_state(void)
{
	return charging_error;
}

static kal_uint32 charging_set_error_state(void *data)
{
	kal_uint32 status = STATUS_OK;
	charging_error = *(kal_uint32*)(data);

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
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
	,charging_diso_init
	,charging_get_diso_state
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