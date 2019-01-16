/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 OV5670mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/barrier.h>
//#include <asm/compiler.h>
//#include <asm/cmpxchg.h>
//#include <asm/switch_to.h>
//#include <asm/system_info.h>
//#include <asm/system_misc.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov5670_2nd_mipi_Sensor.h"
#include <linux/dev_info.h>

#define PFX "ov5670_camera_sensor"

#define OV5670_2ND_OTP 

//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);



static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = OV5670_2ND_MIPI_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0x523c51f6,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 102850000,				//record different mode's pclk
		.linelength = 1676,				//record different mode's linelength
		.framelength = 2045,			//record different mode's framelength
		.startx = 1,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 960,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 102850000,
		.linelength = 1676,
		.framelength = 2045,
		.startx = 1,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1920,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 102850000,
		.linelength = 2012,
		.framelength = 2045,
		.startx = 1,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1920,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 240,	//less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps  
	},
	.normal_video = {
		.pclk = 102850000,
		.linelength = 1676,
		.framelength = 2045,
		.startx = 1,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 960,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 102850000,
		.linelength = 1676,
		.framelength = 511,
		.startx = 1,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 102850000,
		.linelength = 1676,
		.framelength = 2045,
		.startx = 1,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 960,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0x7fff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 2,		//enter capture delay frame num
	.pre_delay_frame = 2, 		//enter preview delay frame num
	.video_delay_frame = 2,		//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
	.i2c_addr_table = {0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4C00,					//current shutter
	.gain = 0x0200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x6c,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 2592, 1944,	  0,	0, 2592, 1944, 1296,  972, 0000, 0000, 1296,  972,	  1,	0, 1280,  960}, // Preview 
 { 2592, 1944,	  0,	0, 2592, 1944, 2592, 1944, 0000, 0000, 2592,  1944,	  1,	0, 2560, 1920}, // capture 
 { 2592, 1944,	  0,	0, 2592, 1944, 1296,  972, 0000, 0000, 1296,  972,	  1,	0, 1280,  960}, // video 
 { 2592, 1944,	  0,	0, 2592, 1944, 640,   480, 0000, 0000,  640,  480,	  1,	0, 	640,  480}, //hight speed video 
 { 2592, 1944,	  0,	0, 2592, 1944, 1296,  972, 0000, 0000, 1296,  972,	  1,	0, 1280,  960}};// slim video 


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

/**************************ov5670 otp start ***************************************/
//pei_add

#ifdef OV5670_2ND_OTP

struct otp_struct {
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
};

char Module_information1[20] = "sunny";
char strmonth1[3];
char strday1[3];

static int RG_Ratio_Typical = 0x110;   // 315
static int BG_Ratio_Typical = 0x163;	//291
static int RG_Ratio_Typical_Ofilm = 0x110;   // 315
static int BG_Ratio_Typical_Ofilm = 0x163;	//291
static int RG_Ratio_Typical_Sunny = 0x108;   // 315
static int BG_Ratio_Typical_Sunny = 0x12D;	//291
static int Module_Id;
static int IR_Id;
static int month;
static int day;
static int module_temp = 1;
// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int check_otp_wb(int index)
{
	printk("pxs_ov5670_otp ++++++++++++++++  %s \n", __func__); //pei_add	

		int flag;
		//set 0x5002[3] to 0
		int temp1;
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002, 0x00  | (temp1 & (~0x08)));
		write_cmos_sensor(0x3d84, 0xC0);
		//partial mode OTP write start address
		write_cmos_sensor(0x3d88, 0x70);
		write_cmos_sensor(0x3d89, 0x20);
		// partial mode OTP write end address
		write_cmos_sensor(0x3d8A, 0x70);
		write_cmos_sensor(0x3d8B, 0x20);
		// read otp into buffer
		write_cmos_sensor(0x3d81, 0x01);
		mDELAY(10);

		//select group
		flag = read_cmos_sensor(0x7020);
	
		
		if (index == 1)
		{ 
			flag =(flag>>6) &0x03;			
		}
		else if (index == 2)
		{ 
			flag =(flag>>4) &0x03;				
		}
		else if (index == 3)
		{ 
			flag =(flag>>2) &0x03;	
		}
		// clear otp buffer
		write_cmos_sensor( 0x7020, 0x00);
		//set 0x5002[3] to 1
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002, 0x08 | (temp1 & (~0x08)));
		if (flag == 0x00) {
		printk("pxs_ov5670_otp  retun 0,group index is empty; 		  %s \n",__func__); //pei_add				
			return 0;
		}
		else if (flag & 0x02) {
		printk("pxs_ov5670_otp  return 1,group index has invalid data; %s \n", __func__); //pei_add		
			return 1;
		}
		else {
		printk("pxs_ov5670_otp  return 2 ,group index has valid data ; %s \n", __func__); //pei_add		
			return 2;
		}
}


static int check_otp_info(int index)
{
	printk("pxs_ov5670_otp ++++++++++++++++  %s \n", __func__); //pei_add	

		int flag;
		//set 0x5002[3] to 0
		int temp1;
	
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002, 0x00  | (temp1 & (~0x08)));
		write_cmos_sensor(0x3d84, 0xC0);
		//partial mode OTP write start address
		write_cmos_sensor(0x3d88, 0x70);
		write_cmos_sensor(0x3d89, 0x10);
		// partial mode OTP write end address
		write_cmos_sensor(0x3d8A, 0x70);
		write_cmos_sensor(0x3d8B, 0x10);
		// read otp into buffer
		write_cmos_sensor(0x3d81, 0x01);
		mDELAY(10);

		//select group
		flag = read_cmos_sensor(0x7010);
	
		
		if (index == 1)
		{ 
			flag =(flag>>6) &0x03;			
		}
		else if (index == 2)
		{ 
			flag =(flag>>4) &0x03;				
		}
		else if (index == 3)
		{ 
			flag =(flag>>2) &0x03;	
		}
		// clear otp buffer
		write_cmos_sensor(0x7010, 0x00);
		//set 0x5002[3] to 1
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002, 0x08 | (temp1 & (~0x08)));
		if (flag == 0x00) {
		printk("pxs_ov5670_otp  retun 0,group index is empty; 		  %s \n",__func__); //pei_add				
			return 0;
		}
		else if (flag & 0x02) {
		printk("pxs_ov5670_otp  return 1,group index has invalid data; %s \n", __func__); //pei_add		
			return 1;
		}
		else {
		printk("pxs_ov5670_otp  return 2 ,group index has valid data ; %s \n", __func__); //pei_add		
			return 2;
		}
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
static int read_otp_info(int index, struct otp_struct *otp_ptr)
{
	printk("pxs_ov5670_otp ++++++++++++++++  %s \n", __func__); //pei_add	

		int i;
		int start_addr, end_addr;
		//set 0x5002[3] to 0
		int temp1;
		int product_daycmo;
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002, 0x00  | (temp1 & (~0x08)));
		if (index == 1) {
			start_addr = 0x7011;
			end_addr = 0x7015;
		}
		else if (index == 2) {
			start_addr = 0x7016;
			end_addr = 0x701a;
		}
		else if (index == 3) {
			start_addr = 0x701b;
			end_addr = 0x701f;
		}
		write_cmos_sensor(0x3d84, 0xC0);
		//partial mode OTP write start address
		write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
		write_cmos_sensor(0x3d89, start_addr & 0xff);
		// partial mode OTP write end address
		write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
		write_cmos_sensor(0x3d8B, end_addr & 0xff);
		// read otp into buffer
		write_cmos_sensor(0x3d81, 0x01);
		mDELAY(10);

		(*otp_ptr).module_integrator_id = read_cmos_sensor(start_addr);
		(*otp_ptr).lens_id = read_cmos_sensor(start_addr + 1);
		(*otp_ptr).production_year = read_cmos_sensor(start_addr + 2);
		(*otp_ptr).production_month = read_cmos_sensor(start_addr + 3);
		(*otp_ptr).production_day = read_cmos_sensor(start_addr + 4);
//add tianqiangqiang
		product_daycmo = read_cmos_sensor(0x701f);
		printk("0x701f is %d \n",product_daycmo);
		printk("index  %d \n",index);
		printk("production_year1  %d \n",(*otp_ptr).production_year); 
		printk("production_month1 %d \n",(*otp_ptr).production_month); 
		printk("production_day1  %d \n",(*otp_ptr).production_day); 
//end		
		// clear otp buffer
		for (i=start_addr; i<=end_addr; i++) {
			write_cmos_sensor(i, 0x00);
		}
		//set 0x5002[3] to 1
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002,  0x08 | (temp1 & (~0x08)));
		return 0;
}

static kal_uint16 read_otp_module_id(void)
{

	struct otp_struct current_otp;
		int i;
		int otp_index;
		int temp;
		// R/G and B/G of current camera module is read out from sensor OTP
		// check first OTP with valid data
		for(i=1;i<=3;i++) {
			temp = check_otp_wb(i);
			
			if (temp == 2) {
				otp_index = i;
				break;
			}
		}
		printk("pxs_ov5670_otp	otp_index=%d,  %s \n",otp_index, __func__); //pei_add		
		if (i>3) {
		// no valid wb OTP data
		printk("pxs_ov5670_otp	no valid wb OTP data  %s \n", __func__); //pei_add
			return 1;
		}
		read_otp_info(otp_index, &current_otp);
		return current_otp.module_integrator_id;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
static int read_otp_wb(int index, struct otp_struct *otp_ptr)
{
	printk("pxs_ov5670_otp ++++++++++++++++  %s \n", __func__); //pei_add	

		int i;
		int temp;
		int start_addr, end_addr;
		//set 0x5002[3] to 0
		int temp1;
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002, 0x00 | (temp1 & (~0x08)));
		if (index == 1) {
			start_addr = 0x7021;
			end_addr = 0x7023;
		}
		else if (index == 2) {
			start_addr = 0x7024;
			end_addr = 0x7026;
		}
		else if (index == 3) {
			start_addr = 0x7027;
			end_addr = 0x7029;
		}
		write_cmos_sensor(0x3d84, 0xC0);
		//partial mode OTP write start address
		write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
		write_cmos_sensor(0x3d89, start_addr & 0xff);
		// partial mode OTP write end address
		write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
		write_cmos_sensor(0x3d8B, end_addr & 0xff);
		// read otp into buffer
		write_cmos_sensor(0x3d81, 0x01);
		mDELAY(10);

		temp = read_cmos_sensor(start_addr + 2);
		(*otp_ptr).rg_ratio = (read_cmos_sensor(start_addr)<<2) + ((temp>>6) & 0x03);
		(*otp_ptr).bg_ratio = (read_cmos_sensor(start_addr + 1)<<2) + ((temp>>4) & 0x03);
		
		// clear otp buffer
		for (i=start_addr; i<=end_addr; i++) {
			write_cmos_sensor(i, 0x00);
		}
		//set 0x5002[3] to 1
		temp1 = read_cmos_sensor(0x5002);
		write_cmos_sensor(0x5002, 0x08 | (temp1 & (~0x08)));

		printk("pxs_ov5670_otp rg_ratio =%x,bg_ratio=%x, %s \n",(*otp_ptr).rg_ratio ,(*otp_ptr).bg_ratio, __func__); //pei_add		

		
		return 0;
}


// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
	printk("pxs_ov5670_otp ++++++++++++++++  %s \n", __func__); //pei_add	

	int AA1 = read_cmos_sensor(0x5032);
	int AA2 = read_cmos_sensor(0x5033);
	int AA3 = read_cmos_sensor(0x5034);
	int AA4 = read_cmos_sensor(0x5035);
	int AA5 = read_cmos_sensor(0x5036);
	int AA6 = read_cmos_sensor(0x5037);
printk("pxs_ov5670_otp AA1=%x ,AA2=%x ,AA3=%x ,AA4=%x ,AA5=%x ,AA6=%x \n ", AA1,AA2,AA3,AA4,AA5,AA6);
	
	if (R_gain>0x400) {
		write_cmos_sensor(0x5032, R_gain>>8);
		write_cmos_sensor(0x5033, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
		write_cmos_sensor(0x5034, G_gain>>8);
		write_cmos_sensor(0x5035, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
		write_cmos_sensor(0x5036, B_gain>>8);
		write_cmos_sensor(0x5037, B_gain & 0x00ff);
	}
		int AA11 = read_cmos_sensor(0x5032);
		int AA12 = read_cmos_sensor(0x5033);
		int AA13 = read_cmos_sensor(0x5034);
		int AA14 = read_cmos_sensor(0x5035);
		int AA15 = read_cmos_sensor(0x5036);
		int AA16 = read_cmos_sensor(0x5037);
printk("pxs_ov5670_otp2 AA11=%x ,AA12=%x ,AA13=%x ,AA14=%x ,AA15=%x ,AA16=%x \n ", AA11,AA12,AA13,AA14,AA15,AA16);



	
	return 0;
}



// call this function after OV5670 initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_wb()
{
	printk("pxs_ov5670_otp ++++++++++++++++  %s \n", __func__); //pei_add	

		struct otp_struct current_otp;
		int i;
		int otp_index, otp_info_index;
		int temp;
		int rg,bg;
		
		// R/G and B/G of current camera module is read out from sensor OTP
		// check first OTP with valid data
		for(i=1;i<=3;i++) {
			temp = check_otp_wb(i);
			
			if (temp == 2) {
				otp_index = i;
				break;
			}
		}
		for(i=1;i<=3;i++) {
			temp = check_otp_info(i);
			
			if (temp == 2) {
				otp_info_index = i;
				break;
			}
		}
		printk("pxs_ov5670_otp	otp_index=%d,  otp_info_index=%d,   %s \n",otp_index, otp_info_index, __func__); //pei_add		
		if (i>3) {
		// no valid wb OTP data
		printk("pxs_ov5670_otp	no valid wb OTP data  %s \n", __func__); //pei_add
			return 1;
		}
		read_otp_info(otp_info_index, &current_otp);
		Module_Id = current_otp.module_integrator_id;
		IR_Id = current_otp.lens_id;
//add tianqiangqiang
		month = current_otp.production_month;
		day = current_otp.production_day;
		sprintf(strmonth1,"%d",month);
		sprintf(strday1,"%d",day);
		if (module_temp == 1)
		{
		strcat(Module_information1,strmonth1);
		strcat(Module_information1,"-");
		strcat(Module_information1,strday1);
		module_temp--;
		}
		if (IR_Id == 2)
		{
		strcpy(Module_information1,"sunny");
		}	

		printk("production_year  %d \n",current_otp.production_year); 
		printk("production_month  %d \n",current_otp.production_month); 
		printk("production_day  %d \n",current_otp.production_day); 
		printk("strmonth1  %s \n", strmonth1); 
		printk("strday1    %s \n", strday1); 
//end
	if (IR_Id == 0x01)
	{
		switch(current_otp.module_integrator_id)
		{
			case 0x07:
				RG_Ratio_Typical = RG_Ratio_Typical_Ofilm;
				BG_Ratio_Typical = BG_Ratio_Typical_Ofilm;
				break;
			case 0x01:
				RG_Ratio_Typical = RG_Ratio_Typical_Sunny;
				BG_Ratio_Typical = BG_Ratio_Typical_Sunny;
				break;
			default:
				break;
		}
	}
		printk("<%s:%d>module id[%x] Module_Id[%x]\n", __func__, __LINE__, current_otp.module_integrator_id, Module_Id);
		printk("<%s:%d>ir id[%x] IR_Id[%x]\n", __func__, __LINE__,current_otp.lens_id, IR_Id);
		read_otp_wb(otp_index, &current_otp);
		rg = current_otp.rg_ratio ;
		bg = current_otp.bg_ratio;	
		//calculate G gain
		int nR_G_gain, nB_G_gain, nG_G_gain;
		int R_gain,	B_gain,	G_gain ;          //pei_add
		int nBase_gain;
		nR_G_gain = (RG_Ratio_Typical*1000) / rg;
		nB_G_gain = (BG_Ratio_Typical*1000) / bg;
		nG_G_gain = 1000;
		
printk("pxs_ov5670_otp	rg=%x,bg=%x,  %s \n",rg,bg, __func__); //pei_add
printk("pxs_ov5670_otp  nR_G_gain=%x, nB_G_gain=%x, nG_G_gain=%x,  %s \n",nR_G_gain,nB_G_gain,nG_G_gain, __func__); //pei_add	

		
		if (nR_G_gain < 1000 || nB_G_gain < 1000)
		{
			if (nR_G_gain <nB_G_gain)
				nBase_gain = nR_G_gain;
			else	
				nBase_gain = nB_G_gain;
		}
		else
		{ 
			nBase_gain =nG_G_gain;
		}
		R_gain = 0x400 * nR_G_gain / (nBase_gain);
		B_gain = 0x400 * nB_G_gain / (nBase_gain);
		G_gain = 0x400 * nG_G_gain / (nBase_gain);
		update_awb_gain(R_gain, G_gain, B_gain);
		
printk("pxs_ov5670_otp  R_gain=%x, G_gain=%x, B_gain=%x,  %s \n",R_gain ,B_gain,G_gain, __func__); //pei_add	

		
		return 0;
}

#endif






/**************************ov5670 otp  end  ***************************************/


static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x380c, (imgsensor.line_length >> 8) & 0xFF);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
  
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
    unsigned long flags;
	kal_uint16 realtime_fps = 0;
	
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	imgsensor.frame_length = (imgsensor.frame_length>>1)<<1;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	shutter = (shutter>>1)<<1;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
        else {
        // Extend frame length
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        }
	} else {
		// Extend frame length
		write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	write_cmos_sensor(0x3502, (shutter << 4) & 0xF0);
	write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);	  
	write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);	
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	
	reg_gain = ((gain * 128)/BASEGAIN);
	//reg_gain = ((gain /BASEGAIN)<<7)+((gain % BASEGAIN)<<7/BASEGAIN);
	reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	kal_uint16 iGain =1;
	kal_uint8 ChangeFlag=0x07;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X	*/
	/* [4:9] = M meams M X		 */
	/* Total gain = M + N /16 X   */

	//
	if (gain < BASEGAIN || gain > 8 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 8 * BASEGAIN)
			gain = 8 * BASEGAIN;		 
		}

		iGain = gain/BASEGAIN;
		
		if(iGain<2){
			ChangeFlag= 0x00;
		}
		else if(iGain<4){
			ChangeFlag= 0x01;
		}
		else if(iGain<8){
			ChangeFlag= 0x03;
		}
		else{
			ChangeFlag= 0x07;
		}
 
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d ,reg[0x366a]= %d, reg_gain = 0x%x\n ", gain, ChangeFlag, reg_gain);

	write_cmos_sensor(0x301d, 0xf0);
	write_cmos_sensor(0x3209, 0x00);
	write_cmos_sensor(0x320a, 0x01);
	
	//group write  hold
	//group 0:delay 0x366a for one frame
	write_cmos_sensor(0x3208, 0x00);
	write_cmos_sensor(0x366a, ChangeFlag);
	write_cmos_sensor(0x3208, 0x10);

	//group 1:all other registers( gain)
	write_cmos_sensor(0x3208, 0x01);
	write_cmos_sensor(0x3508, reg_gain >> 8);
	write_cmos_sensor(0x3509, reg_gain & 0xFF);    

	write_cmos_sensor(0x3208, 0x11);

	//group lanch
	write_cmos_sensor(0x320B, 0x15);
	write_cmos_sensor(0x3208, 0xA1);
	
	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);

	write_cmos_sensor(0x3820, 0x81);   //enable ihdr
 	
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
				// Extend frame length first
				write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
				write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3508, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
			write_cmos_sensor(0x450b, 0x00);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			write_cmos_sensor(0x450b, 0x00);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
			write_cmos_sensor(0x450b, 0x20);		
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			write_cmos_sensor(0x450b, 0x20);
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("OV5670_Sensor_Init_2lane E\n");
	
	write_cmos_sensor(0x0103,0x01);// ; software reset
	mDELAY(10);
	write_cmos_sensor(0x0100, 0x00);// ; software standby
	write_cmos_sensor(0x0300, 0x04); 
	write_cmos_sensor(0x0301, 0x00); 
	write_cmos_sensor(0x0302, 0x69); //;78 MIPI datarate 960 -> 840
	write_cmos_sensor(0x0303, 0x00); 
	write_cmos_sensor(0x0304, 0x03); 
	write_cmos_sensor(0x0305, 0x01); 
	write_cmos_sensor(0x0306, 0x01); 
	write_cmos_sensor(0x030a, 0x00); 
	write_cmos_sensor(0x030b, 0x00); 
	write_cmos_sensor(0x030c, 0x00); 
	write_cmos_sensor(0x030d, 0x1e); 
	write_cmos_sensor(0x030e, 0x00); 
	write_cmos_sensor(0x030f, 0x06); 
	write_cmos_sensor(0x0312, 0x01); 
	write_cmos_sensor(0x3000, 0x00); 
	write_cmos_sensor(0x3002, 0x61);   //21->61
	write_cmos_sensor(0x3005, 0xf0); 
	write_cmos_sensor(0x3007, 0x00); 
	write_cmos_sensor(0x300d, 0x00); 
	write_cmos_sensor(0x3010, 0x40); 
	write_cmos_sensor(0x3015, 0x0f); 
	write_cmos_sensor(0x3018, 0x32); 
	write_cmos_sensor(0x301a, 0xf0); 
	write_cmos_sensor(0x301b, 0xf0); 
	write_cmos_sensor(0x301c, 0xf0); 
	write_cmos_sensor(0x301d, 0xf0); 
	write_cmos_sensor(0x301e, 0xf0); 
	write_cmos_sensor(0x3030, 0x00); 
	write_cmos_sensor(0x3031, 0x0a); 
	write_cmos_sensor(0x303c, 0xff); 
	write_cmos_sensor(0x303e, 0xff); 
	write_cmos_sensor(0x3040, 0xf0); 
	write_cmos_sensor(0x3041, 0x00); 
	write_cmos_sensor(0x3042, 0xf0); 
	write_cmos_sensor(0x3106, 0x11); 
	write_cmos_sensor(0x3500, 0x00); 
	write_cmos_sensor(0x3501, 0x7b); 
	write_cmos_sensor(0x3502, 0x00); 
	write_cmos_sensor(0x3503, 0x04); 
	write_cmos_sensor(0x3504, 0x03); 
	write_cmos_sensor(0x3505, 0x83); 
	write_cmos_sensor(0x3508, 0x07); 
	write_cmos_sensor(0x3509, 0x80); 
	write_cmos_sensor(0x350e, 0x04); 
	write_cmos_sensor(0x350f, 0x00); 
	write_cmos_sensor(0x3510, 0x00); 
	write_cmos_sensor(0x3511, 0x02); 
	write_cmos_sensor(0x3512, 0x00); 
	write_cmos_sensor(0x3601, 0xc8); 
	write_cmos_sensor(0x3610, 0x88); 
	write_cmos_sensor(0x3612, 0x48); 
	write_cmos_sensor(0x3614, 0x5b); 
	write_cmos_sensor(0x3615, 0x96); 
	write_cmos_sensor(0x3621, 0xd0); 
	write_cmos_sensor(0x3622, 0x00); 
	write_cmos_sensor(0x3623, 0x00); 
	write_cmos_sensor(0x3633, 0x13); 
	write_cmos_sensor(0x3634, 0x13); 
	write_cmos_sensor(0x3635, 0x13); 
	write_cmos_sensor(0x3636, 0x13); 
	write_cmos_sensor(0x3645, 0x13); 
	write_cmos_sensor(0x3646, 0x82); 
	write_cmos_sensor(0x3650, 0x00); 
	write_cmos_sensor(0x3652, 0xff); 
	write_cmos_sensor(0x3655, 0x20); 
	write_cmos_sensor(0x3656, 0xff); 
	write_cmos_sensor(0x365a, 0xff); 
	write_cmos_sensor(0x365e, 0xff); 
	write_cmos_sensor(0x3668, 0x00); 
	write_cmos_sensor(0x366a, 0x07); 
	write_cmos_sensor(0x366e, 0x10); 
	write_cmos_sensor(0x366d, 0x00); 
	write_cmos_sensor(0x366f, 0x80); 
	write_cmos_sensor(0x3700, 0x28); 
	write_cmos_sensor(0x3701, 0x10); 
	write_cmos_sensor(0x3702, 0x3a); 
	write_cmos_sensor(0x3703, 0x19); 
	write_cmos_sensor(0x3704, 0x10); 
	write_cmos_sensor(0x3705, 0x00); 
	write_cmos_sensor(0x3706, 0x66); 
	write_cmos_sensor(0x3707, 0x08); 
	write_cmos_sensor(0x3708, 0x34); 
	write_cmos_sensor(0x3709, 0x40); 
	write_cmos_sensor(0x370a, 0x01); 
	write_cmos_sensor(0x370b, 0x1b); 
	write_cmos_sensor(0x3714, 0x24); 
	write_cmos_sensor(0x371a, 0x3e); 
	write_cmos_sensor(0x3733, 0x00); 
	write_cmos_sensor(0x3734, 0x00); 
	write_cmos_sensor(0x373a, 0x05); 
	write_cmos_sensor(0x373b, 0x06); 
	write_cmos_sensor(0x373c, 0x0a); 
	write_cmos_sensor(0x373f, 0xa0); 
	write_cmos_sensor(0x3755, 0x00); 
	write_cmos_sensor(0x3758, 0x00); 
	write_cmos_sensor(0x375b, 0x0e);
	write_cmos_sensor(0x3766, 0x5f); 
	write_cmos_sensor(0x3768, 0x00); 
	write_cmos_sensor(0x3769, 0x22); 
	write_cmos_sensor(0x3773, 0x08); 
	write_cmos_sensor(0x3774, 0x1f); 
	write_cmos_sensor(0x3776, 0x06); 
	write_cmos_sensor(0x37a0, 0x88); 
	write_cmos_sensor(0x37a1, 0x5c); 
	write_cmos_sensor(0x37a7, 0x88); 
	write_cmos_sensor(0x37a8, 0x70); 
	write_cmos_sensor(0x37aa, 0x88); 
	write_cmos_sensor(0x37ab, 0x48); 
	write_cmos_sensor(0x37b3, 0x66); 
	write_cmos_sensor(0x37c2, 0x04); 
	write_cmos_sensor(0x37c5, 0x00); 
	write_cmos_sensor(0x37c8, 0x00); 
	write_cmos_sensor(0x3800, 0x00); 
	write_cmos_sensor(0x3801, 0x0c); 
	write_cmos_sensor(0x3802, 0x00); 
	write_cmos_sensor(0x3803, 0x04); 
	write_cmos_sensor(0x3804, 0x0a); 
	write_cmos_sensor(0x3805, 0x33); 
	write_cmos_sensor(0x3806, 0x07); 
	write_cmos_sensor(0x3807, 0xa3); 
	write_cmos_sensor(0x3808, 0x0a); 
	write_cmos_sensor(0x3809, 0x20); 
	write_cmos_sensor(0x380a, 0x07); 
	write_cmos_sensor(0x380b, 0x98); 
	write_cmos_sensor(0x380c, 0x06); // ;06
	write_cmos_sensor(0x380d, 0x8c); // ;8c
	write_cmos_sensor(0x380e, 0x07); 
	write_cmos_sensor(0x380f, 0xb8); 
	write_cmos_sensor(0x3811, 0x04); 
	write_cmos_sensor(0x3813, 0x02); 
	write_cmos_sensor(0x3814, 0x01); 
	write_cmos_sensor(0x3815, 0x01); 
	write_cmos_sensor(0x3816, 0x00); 
	write_cmos_sensor(0x3817, 0x00); 
	write_cmos_sensor(0x3818, 0x00); 
	write_cmos_sensor(0x3819, 0x00); 
	write_cmos_sensor(0x3820, 0x80); 
	write_cmos_sensor(0x3821, 0x46); 
	write_cmos_sensor(0x3822, 0x48); 
	write_cmos_sensor(0x3826, 0x00); 
	write_cmos_sensor(0x3827, 0x08); 
	write_cmos_sensor(0x382a, 0x01); 
	write_cmos_sensor(0x382b, 0x01); 
	write_cmos_sensor(0x3830, 0x08); 
	write_cmos_sensor(0x3836, 0x02); 
	write_cmos_sensor(0x3837, 0x00); 
	write_cmos_sensor(0x3838, 0x10); 
	write_cmos_sensor(0x3841, 0xff); 
	write_cmos_sensor(0x3846, 0x48); 
	write_cmos_sensor(0x3861, 0x00); 
	write_cmos_sensor(0x3862, 0x04); 
	write_cmos_sensor(0x3863, 0x06); 
	write_cmos_sensor(0x3a11, 0x01); 
	write_cmos_sensor(0x3a12, 0x78); 
	write_cmos_sensor(0x3b00, 0x00); 
	write_cmos_sensor(0x3b02, 0x00); 
	write_cmos_sensor(0x3b03, 0x00); 
	write_cmos_sensor(0x3b04, 0x00); 
	write_cmos_sensor(0x3b05, 0x00); 
	write_cmos_sensor(0x3c00, 0x89); 
	write_cmos_sensor(0x3c01, 0xab); 
	write_cmos_sensor(0x3c02, 0x01); 
	write_cmos_sensor(0x3c03, 0x00); 
	write_cmos_sensor(0x3c04, 0x00); 
	write_cmos_sensor(0x3c05, 0x03); 
	write_cmos_sensor(0x3c06, 0x00); 
	write_cmos_sensor(0x3c07, 0x05); 
	write_cmos_sensor(0x3c0c, 0x00); 
	write_cmos_sensor(0x3c0d, 0x00); 
	write_cmos_sensor(0x3c0e, 0x00); 
	write_cmos_sensor(0x3c0f, 0x00); 
	write_cmos_sensor(0x3c40, 0x00); 
	write_cmos_sensor(0x3c41, 0xa3); 
	write_cmos_sensor(0x3c43, 0x7d); 
	write_cmos_sensor(0x3c45, 0xd7); 
	write_cmos_sensor(0x3c47, 0xfc); 
	write_cmos_sensor(0x3c50, 0x05); 
	write_cmos_sensor(0x3c52, 0xaa); 
	write_cmos_sensor(0x3c54, 0x71); 
	write_cmos_sensor(0x3c56, 0x80); 
	write_cmos_sensor(0x3d85, 0x17); 
	write_cmos_sensor(0x3f03, 0x00); 
	write_cmos_sensor(0x3f0a, 0x00); 
	write_cmos_sensor(0x3f0b, 0x00); 
	write_cmos_sensor(0x4001, 0x60); 
	write_cmos_sensor(0x4009, 0x0d); 
	write_cmos_sensor(0x4020, 0x00); 
	write_cmos_sensor(0x4021, 0x00); 
	write_cmos_sensor(0x4022, 0x00); 
	write_cmos_sensor(0x4023, 0x00); 
	write_cmos_sensor(0x4024, 0x00); 
	write_cmos_sensor(0x4025, 0x00); 
	write_cmos_sensor(0x4026, 0x00); 
	write_cmos_sensor(0x4027, 0x00); 
	write_cmos_sensor(0x4028, 0x00); 
	write_cmos_sensor(0x4029, 0x00); 
	write_cmos_sensor(0x402a, 0x00); 
	write_cmos_sensor(0x402b, 0x00); 
	write_cmos_sensor(0x402c, 0x00); 
	write_cmos_sensor(0x402d, 0x00); 
	write_cmos_sensor(0x402e, 0x00); 
	write_cmos_sensor(0x402f, 0x00); 
	write_cmos_sensor(0x4040, 0x00); 
	write_cmos_sensor(0x4041, 0x03); 
	write_cmos_sensor(0x4042, 0x00); 
	write_cmos_sensor(0x4043, 0x7a); 
	write_cmos_sensor(0x4044, 0x00); 
	write_cmos_sensor(0x4045, 0x7a); 
	write_cmos_sensor(0x4046, 0x00); 
	write_cmos_sensor(0x4047, 0x7a); 
	write_cmos_sensor(0x4048, 0x00); 
	write_cmos_sensor(0x4049, 0x7a); 
	write_cmos_sensor(0x4303, 0x00); 
	write_cmos_sensor(0x4307, 0x30); 
	write_cmos_sensor(0x4500, 0x58); 
	write_cmos_sensor(0x4501, 0x04); 
	write_cmos_sensor(0x4502, 0x40); 
	write_cmos_sensor(0x4503, 0x10); 
	write_cmos_sensor(0x4508, 0xaa); 
	write_cmos_sensor(0x4509, 0xaa); 
	write_cmos_sensor(0x450a, 0x00); 
	write_cmos_sensor(0x450b, 0x00); 
	write_cmos_sensor(0x4600, 0x01); 
	write_cmos_sensor(0x4601, 0x03); 
	write_cmos_sensor(0x4700, 0xa4); 
	write_cmos_sensor(0x4800, 0x6c); 
	write_cmos_sensor(0x4816, 0x53); 
	write_cmos_sensor(0x481f, 0x40); 
	write_cmos_sensor(0x4837, 0x13); // ;11 mipi global timing datarate 940 -> 840
	write_cmos_sensor(0x5000, 0x56); 
	write_cmos_sensor(0x5001, 0x01); 
	write_cmos_sensor(0x5002, 0x28); 
	write_cmos_sensor(0x5004, 0x0c); 
	write_cmos_sensor(0x5006, 0x0c); 
	write_cmos_sensor(0x5007, 0xe0); 
	write_cmos_sensor(0x5008, 0x01); 
	write_cmos_sensor(0x5009, 0xb0); 
	write_cmos_sensor(0x5901, 0x00); 
	write_cmos_sensor(0x5a01, 0x00); 
	write_cmos_sensor(0x5a03, 0x00); 
	write_cmos_sensor(0x5a04, 0x0c); 
	write_cmos_sensor(0x5a05, 0xe0); 
	write_cmos_sensor(0x5a06, 0x09); 
	write_cmos_sensor(0x5a07, 0xb0); 
	write_cmos_sensor(0x5a08, 0x06); 
	write_cmos_sensor(0x5e00, 0x00); 
	write_cmos_sensor(0x3618, 0x2a); 
								   
	//;Ally031414					  
	write_cmos_sensor(0x3734, 0x40); //	;; Improve HFPN
	write_cmos_sensor(0x5b00, 0x01);  // ;; [2:0] otp start addr[10:8]
	write_cmos_sensor(0x5b01, 0x10);  // ;; [7:0] otp start addr[7:0]
	write_cmos_sensor(0x5b02, 0x01);  // ;; [2:0] otp end addr[10:8]
	write_cmos_sensor(0x5b03, 0xDB);  // ;; [7:0] otp end addr[7:0]
	write_cmos_sensor(0x3d8c, 0x71); //; Header address high byte
	write_cmos_sensor(0x3d8d, 0xEA); //; Header address low byte
	write_cmos_sensor(0x4017, 0x08); // ; threshold= 2LSB for full size
								  
	//;Strong DPC1.53				 
	write_cmos_sensor(0x5780, 0x3e); 
	write_cmos_sensor(0x5781, 0x0f); 
	write_cmos_sensor(0x5782, 0x44); 
	write_cmos_sensor(0x5783, 0x02); 
	write_cmos_sensor(0x5784, 0x01); 
	write_cmos_sensor(0x5785, 0x00); 
	write_cmos_sensor(0x5786, 0x00); 
	write_cmos_sensor(0x5787, 0x04); 
	write_cmos_sensor(0x5788, 0x02); 
	write_cmos_sensor(0x5789, 0x0f); 
	write_cmos_sensor(0x578a, 0xfd); 
	write_cmos_sensor(0x578b, 0xf5); 
	write_cmos_sensor(0x578c, 0xf5); 
	write_cmos_sensor(0x578d, 0x03); 
	write_cmos_sensor(0x578e, 0x08); 
	write_cmos_sensor(0x578f, 0x0c); 
	write_cmos_sensor(0x5790, 0x08); 
	write_cmos_sensor(0x5791, 0x04); 
	write_cmos_sensor(0x5792, 0x00); 
	write_cmos_sensor(0x5793, 0x52); 
	write_cmos_sensor(0x5794, 0xa3); 
	//;Ping 					  
	write_cmos_sensor(0x380e, 0x07); //; fps fine adjustment
	write_cmos_sensor(0x380f, 0xfd); //; fps fine adjustment
	write_cmos_sensor(0x3503, 0x00); //; real gain [2]   gain no delay, shutter no delay
	//;added					 
//	write_cmos_sensor(0x3d85, 0x17); 
//	write_cmos_sensor(0x3655, 0x20); 
	write_cmos_sensor(0x0100, 0x01); //;01							   

	
#ifdef OV5670_2ND_OTP
	printk("pxs_ov5670_otp ++++++++++++++++  %s \n", __func__); //pei_add	
	mdelay(40);

	update_otp_wb(); //pei_add
#endif

}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF(" OV5670PreviewSetting_2lane enter\n");

	//@@PV_Quarter_size_30fps_800Mbps/lane_1296x972							  
	//99 1296 972 															  
	//;;102 3601	157c														  
	//;;PCLK=HTS*VTS*fps=0x68c*0x7fd*30=1676*2045*30=102.85M					  
																			  
	write_cmos_sensor(0x0100, 0x00);  // 	
	
	write_cmos_sensor(0x3501, 0x73);  // 							  
	write_cmos_sensor(0x3502, 0x00);  // 							  
	write_cmos_sensor(0x3508, 0x03);  // 							  
	write_cmos_sensor(0x3509, 0x00);  // 	
	write_cmos_sensor(0x3623, 0x00);  // 	
	write_cmos_sensor(0x366e, 0x08);  // 							  
	write_cmos_sensor(0x370b, 0x1b);  // 							  
	write_cmos_sensor(0x3808, 0x05);  // 							  
	write_cmos_sensor(0x3809, 0x10);  // 							  
	write_cmos_sensor(0x380a, 0x03);  // 							  
	write_cmos_sensor(0x380b, 0xcc);  //;c0							  
	write_cmos_sensor(0x380c, 0x06);  // 							  
	write_cmos_sensor(0x380d, 0x8c);  // 							  
	write_cmos_sensor(0x380e, 0x07);  //;03							  
	write_cmos_sensor(0x380f, 0xfd);  //;e0	


	
	write_cmos_sensor(0x3814, 0x03);  // 							  
	write_cmos_sensor(0x3820, 0x90);  // 							  
	write_cmos_sensor(0x3821, 0x47);  // 							  
	write_cmos_sensor(0x382a, 0x03);  // 	
	

	write_cmos_sensor(0x3845, 0x02);  // 							  
	write_cmos_sensor(0x4009, 0x05);  // 							  
	write_cmos_sensor(0x4502, 0x44);  // 							  
	write_cmos_sensor(0x4508, 0x55);  // 							  
	write_cmos_sensor(0x4509, 0x55);  // 	
	write_cmos_sensor(0x450a, 0x00);  // 	
	write_cmos_sensor(0x4600, 0x00);  // 							  
	write_cmos_sensor(0x4601, 0x81);  // 							  
	write_cmos_sensor(0x4017, 0x10);  //; threshold = 4LSB for Binning 
	write_cmos_sensor(0x400a, 0x02);  //;							  
	write_cmos_sensor(0x400b, 0x00);  //;	
	
	write_cmos_sensor(0x0100, 0x01);  // 

	
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("OV5670CaptureSetting_2lane enter! currefps:%d\n",currefps);
	if (currefps == 240) { //24fps for PIP
		write_cmos_sensor(0x0100, 0x00); 
		
		write_cmos_sensor(0x3501, 0x5f); //long exposure
		write_cmos_sensor(0x3502, 0xd0);  //long exposure
		
		write_cmos_sensor(0x3508, 0x03);  //gain
		write_cmos_sensor(0x3509, 0x00);  //gain
		
		write_cmos_sensor(0x366e, 0x10); 
		write_cmos_sensor(0x3623, 0x00);  // 
		write_cmos_sensor(0x370b, 0x1b); 
		write_cmos_sensor(0x3808, 0x0a); 
		write_cmos_sensor(0x3809, 0x20); 
		write_cmos_sensor(0x380a, 0x07); 
		write_cmos_sensor(0x380b, 0x98); 
		write_cmos_sensor(0x380c, ((imgsensor_info.cap1.linelength >> 8) & 0xFF)); // hts 
		write_cmos_sensor(0x380d, (imgsensor_info.cap1.linelength & 0xFF)); 	   // hts
		write_cmos_sensor(0x380e, ((imgsensor_info.cap1.framelength >> 8) & 0xFF));  // vts 
		write_cmos_sensor(0x380f, (imgsensor_info.cap1.framelength & 0xFF));		 // vts 

		
		write_cmos_sensor(0x3814, 0x01); 
		write_cmos_sensor(0x3820, 0x80); 
		write_cmos_sensor(0x3821, 0x46); 
		write_cmos_sensor(0x382a, 0x01);
		
		write_cmos_sensor(0x4009, 0x0d); 
		write_cmos_sensor(0x4502, 0x40); 
		write_cmos_sensor(0x4508, 0xaa); 
		write_cmos_sensor(0x4509, 0xaa); 
		write_cmos_sensor(0x450a, 0x00);  // 
		write_cmos_sensor(0x4600, 0x01); 
		write_cmos_sensor(0x4601, 0x03); 
		write_cmos_sensor(0x4017, 0x08); //threshold= 2LSB for full size
		write_cmos_sensor(0x400a, 0x02); //
		write_cmos_sensor(0x400b, 0x00); //
		
		write_cmos_sensor(0x0100, 0x01);
		
	} else{ // for 30fps need ti update
		write_cmos_sensor(0x0100, 0x00); 
		
		write_cmos_sensor(0x3501, 0x5f); //long exposure
		write_cmos_sensor(0x3502, 0xd0);  //long exposure
		
		write_cmos_sensor(0x3508, 0x03);  //gain
		write_cmos_sensor(0x3509, 0x00);  //gain

		write_cmos_sensor(0x3623, 0x00);  //gain
		write_cmos_sensor(0x366e, 0x10); 
		write_cmos_sensor(0x370b, 0x1b); 
		write_cmos_sensor(0x3808, 0x0a); 
		write_cmos_sensor(0x3809, 0x20); 
		write_cmos_sensor(0x380a, 0x07); 
		write_cmos_sensor(0x380b, 0x98); 
		write_cmos_sensor(0x380c, 0x06); // hts 
		write_cmos_sensor(0x380d, 0x8c); 	   // hts
		write_cmos_sensor(0x380e, 0x07);  // vts 
		write_cmos_sensor(0x380f, 0xfd);		 // vts 

		
		write_cmos_sensor(0x3814, 0x01); 
		write_cmos_sensor(0x3820, 0x80); 
		write_cmos_sensor(0x3821, 0x46); 
		write_cmos_sensor(0x382a, 0x01);
		
		write_cmos_sensor(0x3845, 0x00);  //v_offset for auto size mode
		
		write_cmos_sensor(0x4009, 0x0d); 
		write_cmos_sensor(0x4502, 0x40); 
		write_cmos_sensor(0x4508, 0xaa); 
		write_cmos_sensor(0x4509, 0xaa); 
		write_cmos_sensor(0x450a, 0x00);  // 
		write_cmos_sensor(0x4600, 0x01); 
		write_cmos_sensor(0x4601, 0x03); 
		write_cmos_sensor(0x4017, 0x08); //threshold= 2LSB for full size
		write_cmos_sensor(0x400a, 0x02); //
		write_cmos_sensor(0x400b, 0x00); //
		
		write_cmos_sensor(0x0100, 0x01);
		
	} 
	
		
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting Enter! currefps:%d\n",currefps);
	
		//@@PV_Quarter_size_30fps_800Mbps/lane_1296x972							  
	//99 1296 972 															  
	//;;102 3601	157c														  
	//;;PCLK=HTS*VTS*fps=0x68c*0x7fd*30=1676*2045*30=102.85M					  
																			  
	write_cmos_sensor(0x0100, 0x00);  // 	
	
	write_cmos_sensor(0x3501, 0x73);  // 							  
	write_cmos_sensor(0x3502, 0x00);  // 							  
	write_cmos_sensor(0x3508, 0x01);  // 							  
	write_cmos_sensor(0x3509, 0x80);  // 
	write_cmos_sensor(0x3623, 0x00);  //gain
	write_cmos_sensor(0x366e, 0x08);  // 							  
	write_cmos_sensor(0x370b, 0x1b);  // 							  
	write_cmos_sensor(0x3808, 0x05);  // 							  
	write_cmos_sensor(0x3809, 0x10);  // 							  
	write_cmos_sensor(0x380a, 0x03);  // 							  
	write_cmos_sensor(0x380b, 0xcc);  //;c0							  
	write_cmos_sensor(0x380c, ((imgsensor_info.normal_video.linelength >> 8) & 0xFF)); // hts
	write_cmos_sensor(0x380d, (imgsensor_info.normal_video.linelength & 0xFF)); 	   // hts
	write_cmos_sensor(0x380e, ((imgsensor_info.normal_video.framelength >> 8) & 0xFF));  // vts 
	write_cmos_sensor(0x380f, (imgsensor_info.normal_video.framelength & 0xFF));		 // vts 						  
	write_cmos_sensor(0x3814, 0x03);  // 							  
	write_cmos_sensor(0x3820, 0x90);  // 							  
	write_cmos_sensor(0x3821, 0x47);  // 							  
	write_cmos_sensor(0x382a, 0x03);  // 							  
	write_cmos_sensor(0x3845, 0x02);  // 							  
	write_cmos_sensor(0x4009, 0x05);  // 							  
	write_cmos_sensor(0x4502, 0x44);  // 							  
	write_cmos_sensor(0x4508, 0x55);  // 							  
	write_cmos_sensor(0x4509, 0x55);  // 
	write_cmos_sensor(0x450a, 0x00);  // 
	write_cmos_sensor(0x4600, 0x00);  // 							  
	write_cmos_sensor(0x4601, 0x81);  // 							  
	write_cmos_sensor(0x4017, 0x10);  //; threshold = 4LSB for Binning 
	write_cmos_sensor(0x400a, 0x02);  //;							  
	write_cmos_sensor(0x400b, 0x00);  //;	
	
	write_cmos_sensor(0x0100, 0x01);  // 	
}
static void hs_video_setting()
{ 
	LOG_INF("hs_video_setting enter!\n");

//VGA 120fps
	write_cmos_sensor(0x0100,0x00);
	write_cmos_sensor(0x3501,0x1f);
	write_cmos_sensor(0x3502,0x00);
	write_cmos_sensor(0x3508,0x07);
	write_cmos_sensor(0x3509,0x80);
	write_cmos_sensor(0x3623,0x04);
	write_cmos_sensor(0x366e,0x08);
	write_cmos_sensor(0x370b,0x1b);
	write_cmos_sensor(0x3808,0x02);
	write_cmos_sensor(0x3809,0x84);
	write_cmos_sensor(0x380a,0x01);
	write_cmos_sensor(0x380b,0xe0);
	write_cmos_sensor(0x380c,0x06);
	write_cmos_sensor(0x380d,0x8c);
	write_cmos_sensor(0x380e,0x01);
	write_cmos_sensor(0x380f,0xff);
	write_cmos_sensor(0x3814,0x07);
	write_cmos_sensor(0x3820,0x90);
	write_cmos_sensor(0x3821,0xc6);
	write_cmos_sensor(0x382a,0x07);
	write_cmos_sensor(0x3845,0x00);
	write_cmos_sensor(0x4009,0x05);
	write_cmos_sensor(0x4502,0x40);
	write_cmos_sensor(0x4508,0x55);
	write_cmos_sensor(0x4509,0x55);
	write_cmos_sensor(0x450a,0x02);
	write_cmos_sensor(0x4600,0x00);
	write_cmos_sensor(0x4601,0x40);
	write_cmos_sensor(0x4017,0x10);
	write_cmos_sensor(0x400a,0x02);
	write_cmos_sensor(0x400b,0x00);
	write_cmos_sensor(0x0100,0x01);

	
				//@@1080_60fps_800Mbps/lane
/*
		//;;99 1920 1080
		//;;102 3601  1770
		//;;PCLK=HTS*VTS*fps=0x574*0x4cb*60=1396*1227*60=102.85M
		write_cmos_sensor(0x0100, 0x00); //
		write_cmos_sensor(0x3501, 0x45); //
		write_cmos_sensor(0x3502, 0x00); //
		write_cmos_sensor(0x3508, 0x02); //
		write_cmos_sensor(0x3509, 0x80); //
		write_cmos_sensor(0x366e, 0x10); //
		write_cmos_sensor(0x370b, 0x05); //; reduce code for 1080p format only
		write_cmos_sensor(0x3808, 0x07); // H output size : 0x780= 1920
		write_cmos_sensor(0x3809, 0x80); //
		write_cmos_sensor(0x380a, 0x04); //  V output size: 0x438 =1080
		write_cmos_sensor(0x380b, 0x38); //
		write_cmos_sensor(0x380c, ((imgsensor_info.hs_video.linelength >> 8) & 0xFF)); // hts
		write_cmos_sensor(0x380d, (imgsensor_info.hs_video.linelength & 0xFF)); 	   // hts
		write_cmos_sensor(0x380e, ((imgsensor_info.hs_video.framelength >> 8) & 0xFF));  // vts 
		write_cmos_sensor(0x380f, (imgsensor_info.hs_video.framelength & 0xFF));		 // vts 
		write_cmos_sensor(0x3814, 0x01); //
		write_cmos_sensor(0x3820, 0x80); //
		write_cmos_sensor(0x3821, 0x46); //
		write_cmos_sensor(0x382a, 0x01); //
		write_cmos_sensor(0x3845, 0x00); //
		write_cmos_sensor(0x4009, 0x0d); //
		write_cmos_sensor(0x4502, 0x40); //
		write_cmos_sensor(0x4508, 0xaa); //
		write_cmos_sensor(0x4509, 0xaa); //
		write_cmos_sensor(0x4600, 0x00); //
		write_cmos_sensor(0x4601, 0xc0); //
		write_cmos_sensor(0x4017, 0x08); // ; threshold= 2LSB for full size
		write_cmos_sensor(0x400a, 0x01); //; blc dark limit to 354 code
		write_cmos_sensor(0x400b, 0x62); //; blc dark limit to 354 code
		write_cmos_sensor(0x0100, 0x01); //
*/
}


static void slim_video_setting()
{
	LOG_INF("slim_video_setting enter!\n");
	
		//@@PV_Quarter_size_30fps_800Mbps/lane_1296x972 						  
	//99 1296 972															  
	//;;102 3601	157c														  
	//;;PCLK=HTS*VTS*fps=0x68c*0x7fd*30=1676*2045*30=102.85M					  
																			  
	write_cmos_sensor(0x0100, 0x00);  //	
	
	write_cmos_sensor(0x3501, 0x73);  //							  
	write_cmos_sensor(0x3502, 0x00);  //							  
	write_cmos_sensor(0x3508, 0x01);  //							  
	write_cmos_sensor(0x3509, 0x80);  //							  
	write_cmos_sensor(0x366e, 0x08);  //	
	write_cmos_sensor(0x3623, 0x00);  //gain
	write_cmos_sensor(0x370b, 0x1b);  //							  
	write_cmos_sensor(0x3808, 0x05);  //							  
	write_cmos_sensor(0x3809, 0x10);  //							  
	write_cmos_sensor(0x380a, 0x03);  //							  
	write_cmos_sensor(0x380b, 0xcc);  //;c0 						  
	write_cmos_sensor(0x380c, ((imgsensor_info.slim_video.linelength >> 8) & 0xFF)); // hts
	write_cmos_sensor(0x380d, (imgsensor_info.slim_video.linelength & 0xFF)); 	   // hts
	write_cmos_sensor(0x380e, ((imgsensor_info.slim_video.framelength >> 8) & 0xFF));  // vts 
	write_cmos_sensor(0x380f, (imgsensor_info.slim_video.framelength & 0xFF));		 // vts 						  
	write_cmos_sensor(0x3814, 0x03);  //							  
	write_cmos_sensor(0x3820, 0x90);  //							  
	write_cmos_sensor(0x3821, 0x47);  //							  
	write_cmos_sensor(0x382a, 0x03);  //							  
	write_cmos_sensor(0x3845, 0x02);  //							  
	write_cmos_sensor(0x4009, 0x05);  //							  
	write_cmos_sensor(0x4502, 0x44);  //							  
	write_cmos_sensor(0x4508, 0x55);  //							  
	write_cmos_sensor(0x4509, 0x55);  //	
	write_cmos_sensor(0x450a, 0x00);  // 
	write_cmos_sensor(0x4600, 0x00);  //							  
	write_cmos_sensor(0x4601, 0x81);  //							  
	write_cmos_sensor(0x4017, 0x10);  //; threshold = 4LSB for Binning 
	write_cmos_sensor(0x400a, 0x02);  //;							  
	write_cmos_sensor(0x400b, 0x00);  //;	
	
	write_cmos_sensor(0x0100, 0x01);  //	

}



/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//kal_uint16 module_id;
	struct devinfo_struct *dev_ofilm, *dev_sunny;//(struct devinfo_struct*)kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
    static kal_uint8 devinfo_add = 2;
    // dev->device_type = "Camera";
    // dev->device_vendor = "Ofilm";
    // dev->device_ic = "OV5670";
    // dev->device_version = DEVINFO_NULL;
    // dev->device_module = DEVINFO_NULL;
    // dev->device_info = DEVINFO_NULL;
    // LOG_INF("<%s:%d>devinfo_add[%d]dev[%x]\n", __func__, __LINE__, devinfo_add, dev);
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
			if (*sensor_id == 0x5670) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				if(devinfo_add !=0)
			    {
			    	sensor_init();
			    	//Module_Id = read_otp_module_id();

			    	/*dev_ofilm = (struct devinfo_struct*)kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
				    dev_ofilm->device_type = "Camera";
				    dev_ofilm->device_vendor = "Ofilm";
				    dev_ofilm->device_ic = "OV5670";
				    dev_ofilm->device_version = DEVINFO_NULL;
				    dev_ofilm->device_module = DEVINFO_NULL;
				    dev_ofilm->device_info = DEVINFO_NULL;
				    LOG_INF("<%s:%d>devinfo_add[%d]dev[%x]\n", __func__, __LINE__, devinfo_add, dev_ofilm);*/

				    dev_sunny = (struct devinfo_struct*)kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
				    dev_sunny->device_type = "Camera";
				    dev_sunny->device_vendor = &Module_information1;//"Sunny";
				    dev_sunny->device_ic = "OV5670";
				    dev_sunny->device_version = DEVINFO_NULL;
				    dev_sunny->device_module = DEVINFO_NULL;
				    dev_sunny->device_info = DEVINFO_NULL;
				    LOG_INF("<%s:%d>devinfo_add[%d]dev[%x]\n", __func__, __LINE__, devinfo_add, dev_sunny);
				    LOG_INF("<%s:%d>Module_Id:%x\n", __func__, __LINE__, Module_Id);
				    LOG_INF("<%s:%d>IR_Id_flt:%x\n", __func__, __LINE__, IR_Id);
				if (IR_Id == 0x01)
				{
				    switch(Module_Id)
				    {
				    	// case 0x07:
				    	// 	dev_ofilm->device_used = DEVINFO_USED;
				    	// 	dev_sunny->device_used = DEVINFO_UNUSED;
				    	// 	break;
				    	case 0x01:
				    		//dev_ofilm->device_used = DEVINFO_UNUSED;
				    		dev_sunny->device_used = DEVINFO_USED;
				    		*sensor_id = OV5670_2ND_MIPI_SENSOR_ID;
				    		break;
				    	default :
				    		dev_sunny->device_used = DEVINFO_UNUSED;
				    		*sensor_id = 0xFFFFFFFF;
				    		break;
				    }
				}
				else
				{
						dev_sunny->device_used = DEVINFO_UNUSED;
				    		*sensor_id = 0xFFFFFFFF;
				}

			    	//dev_ofilm->device_used = DEVINFO_USED;
			    	//DEVINFO_CHECK_ADD_DEVICE(dev_ofilm);
			    	DEVINFO_CHECK_ADD_DEVICE(dev_sunny);
			    	devinfo_add = 0;
			    }	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x,0x%0x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		
        if(devinfo_add == 1)
    	{
    		/*dev_ofilm = (struct devinfo_struct*)kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
		    dev_ofilm->device_type = "Camera";
		    dev_ofilm->device_vendor = "Ofilm";
		    dev_ofilm->device_ic = "OV5670";
		    dev_ofilm->device_version = DEVINFO_NULL;
		    dev_ofilm->device_module = DEVINFO_NULL;
		    dev_ofilm->device_info = DEVINFO_NULL;
		    LOG_INF("<%s:%d>devinfo_add[%d]dev[%x]\n", __func__, __LINE__, devinfo_add, dev_ofilm);*/
		    dev_sunny = (struct devinfo_struct*)kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
		    dev_sunny->device_type = "Camera";
		    dev_sunny->device_vendor = &Module_information1;//"Sunny_flt";
		    dev_sunny->device_ic = "OV5670";
		    dev_sunny->device_version = DEVINFO_NULL;
		    dev_sunny->device_module = DEVINFO_NULL;
		    dev_sunny->device_info = DEVINFO_NULL;
		    LOG_INF("<%s:%d>devinfo_add[%d]dev[%x]\n", __func__, __LINE__, devinfo_add, dev_sunny);
		    //dev_ofilm->device_used = DEVINFO_UNUSED;
		    dev_sunny->device_used = DEVINFO_UNUSED;
		    
	    	//dev_ofilm->device_used = DEVINFO_USED;
	    	//DEVINFO_CHECK_ADD_DEVICE(dev_ofilm);
	    	DEVINFO_CHECK_ADD_DEVICE(dev_sunny);
    		devinfo_add = 0;
    	}
    	else
    		if (devinfo_add > 0)
    			devinfo_add--;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
    
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 

	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
			if (sensor_id == 0x5670)
				sensor_id = OV5670_2ND_MIPI_SENSOR_ID;
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x4C00;
	imgsensor.gain = 0x0200;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	set_mirror_flip(IMAGE_HV_MIRROR);
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x4303[3]: 1 enable,  0 disable
		// only supports a special color bar test pattern
		write_cmos_sensor(0x4303,0x08);
	} else {
		// 0x4303[3]: 1 enable,  0 disable
		// only supports a special color bar test pattern
		write_cmos_sensor(0x4303,0x00);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long*)feature_para;
	unsigned long long *feature_return_data = (unsigned long long*)feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode(*feature_data);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", *feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_SET_HDR:
			LOG_INF("ihdr enable :%d\n", *feature_data_16);
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = *feature_data_16;
			imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;	  
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",*feature_data,*(feature_data+1),*(feature_data+2));
            ihdr_write_shutter_gain(*feature_data,*(feature_data+1),*(feature_data+2));
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV5670_2ND_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5670_MIPI_RAW_SensorInit	*/
