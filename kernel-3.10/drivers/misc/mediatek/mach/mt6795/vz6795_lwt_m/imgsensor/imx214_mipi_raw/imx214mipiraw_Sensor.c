/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX214mipi_Sensor.c
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
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx214mipiraw_Sensor.h"

#define PFX "IMX214_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
#define OTP 0
#if OTP //EEPROM for imx214 with k6795
#define OTP_SIZE 452
u8 OTPData[OTP_SIZE];
extern  int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata);
#endif

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = IMX214_SENSOR_ID,
	
	.checksum_value =0xc15d2913,
	
	.pre = {
        .pclk = 320000000,              //record different mode's pclk
		.linelength = 5008,				//record different mode's linelength
        .framelength = 2110,         //record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk =480000000,
		.linelength = 5008,
		.framelength = 3160,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk =387200000,
		.linelength = 5008,
		.framelength = 3220,
		.startx = 0,
		.starty =0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,	
	},
	.normal_video = {
		.pclk = 480000000,
		.linelength = 5008,
		.framelength = 3160,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 387200000,
		.linelength = 5008,
		.framelength =1288,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1184,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
	},
	.slim_video = {
		.pclk =387200000,
		.linelength = 5008,
		.framelength = 1288,
		.startx = 0,
		.starty =0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1184,
		.mipi_data_lp2hs_settle_dc =85,
		.max_framerate = 600,
	},
	.margin = 10,
	.min_shutter = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame =1,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3, 
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	
	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34,0x20,0xff},
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x34,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 4208, 3120,	  0,	0, 4208, 3120, 2104,  1560, 0000, 0000, 2104,  1560,	  0,	0, 2104, 1560}, // Preview 
 { 4208, 3120,	  0,	0, 4208, 3120, 4208,  3120, 0000, 0000, 4208,  3120,	  0,	0, 4208, 3120}, // capture 
 { 4208, 3120,	  0,	0, 4208, 3120, 4208,  3120, 0000, 0000, 4208,  3120,	  0,	0, 4208, 3120}, // video 
 { 4208, 3120,	  0,    0, 4208, 3120, 2104,  1560, 0000,  376, 2104,  1184,	  0,	0, 2104, 1184}, //hight speed video 
 { 4208, 3120,	  0,    0, 4208, 3120, 2104,  1560, 0000,  376, 2104,  1184,	  0,	0, 2104, 1184}};// slim video 

SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=    
{/* Preview mode setting */
 {0x02, 0x0a,   0x00,   0x08, 0x40, 0x00, 
  0x00, 0x2b, 0x0838, 0x0618, 0x01, 0x35, 0x0200, 0x0001,
  0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
  /* Video mode setting */
 {0x02, 0x0a,   0x00,   0x08, 0x40, 0x00, 
  0x00, 0x2b, 0x1070, 0x0c30, 0x01, 0x35, 0x0200, 0x0001,
  0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
  /* Capture mode setting */
 {0x02, 0x0a,   0x00,   0x08, 0x40, 0x00, 
  0x00, 0x2b, 0x1070, 0x0c30, 0x01, 0x35, 0x0200, 0x0001,
  0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000}}; 

typedef struct
{
    MUINT16 DarkLimit_H;
    MUINT16 DarkLimit_L;
    MUINT16 OverExp_Min_H;
    MUINT16 OverExp_Min_L;
    MUINT16 OverExp_Max_H;
    MUINT16 OverExp_Max_L;
}SENSOR_ATR_INFO, *pSENSOR_ATR_INFO;

SENSOR_ATR_INFO sensorATR_Info[4]=    
{/* Strength Range Min */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /* Strength Range Std */
    {0x00, 0x32, 0x00, 0x3c, 0x03, 0xff},
    /* Strength Range Max */
    {0x3f, 0xff, 0x3f, 0xff, 0x3f, 0xff},
    /* Strength Range Custom */
    {0x3F, 0xFF, 0x00, 0x0, 0x3F, 0xFF}};

#define MIPI_MaxGainIndex 159

static kal_uint16 sensorGainMapping[MIPI_MaxGainIndex][2] = {
	
	{64 , 1  },
	{65 , 8  },
	{66 , 13 },
	{67 , 23 },
	{68 , 27 },
	{69 , 36 },
	{70 , 41 },
	{71 , 49 },
	{72 , 53 },
	{73 , 61 },
	{74 , 69 },
	{75 , 73 },
	{76 , 80 },
	{77 , 88 },
	{78 , 91 },
	{79 , 98 },
	{80 , 101},
	{81 , 108},
	{82 , 111},
	{83 , 117},
	{84 , 120},
	{85 , 126},
	{86 , 132},
	{87 , 135},
	{88 , 140},
	{89 , 143},
	{90 , 148},
	{91 , 151},
	{92 , 156},
	{93 , 161},
	{94 , 163},
	{95 , 168},
	{96 , 170},
	{97 , 175},
	{98 , 177},
	{99 , 181},
	{100, 185},
	{101, 187},
	{102, 191},
	{103, 193},
	{104, 197},
	{105, 199},
	{106, 203},
	{107, 205},
	{108, 207},
	{109, 212},
	{110, 214},
	{111, 217},
	{112, 219},
	{113, 222},
	{114, 224},
	{115, 227},
	{116, 230},
	{117, 232},
	{118, 234},
	{119, 236},
	{120, 239},
	{122, 244},
	{123, 245},
	{124, 248},
	{125, 249},
	{126, 252},
	{127, 253},
	{128, 256},
	{129, 258},
	{130, 260},
	{131, 262},
	{132, 263},
	{133, 266},
	{134, 268},
	{136, 272},
	{138, 274},
	{139, 276},
	{140, 278},
	{141, 280},
	{143, 282},
	{144, 284},
	{145, 286},
	{147, 288},
	{148, 290},
	{149, 292},
	{150, 294},
	{152, 296},
	{153, 298},
	{155, 300},
	{156, 302},
	{157, 304},
	{159, 306},
	{161, 308},
	{162, 310},
	{164, 312},
	{166, 314},
	{167, 316},
	{169, 318},
	{171, 320},
	{172, 322},
	{174, 324},
	{176, 326},
	{178, 328},
	{180, 330},
	{182, 332},
	{184, 334},
	{186, 336},
	{188, 338},
	{191, 340},
	{193, 342},
	{195, 344},
	{197, 346},
	{200, 348},
	{202, 350},
	{205, 352},
	{207, 354},
	{210, 356},
	{212, 358},
	{216, 360},
	{218, 362},
	{221, 364},
	{225, 366},
	{228, 368},
	{231, 370},
	{234, 372},
	{237, 374},
	{241, 376},
	{244, 378},
	{248, 380},
	{252, 382},
	{256, 384},
	{260, 386},
	{264, 388},
	{269, 390},
	{273, 392},
	{278, 394},
	{282, 396},
	{287, 398},
	{292, 400},
	{298, 402},
	{303, 404},
	{309, 406},
	{315, 408},
	{321, 410},
	{328, 412},
	{334, 414},
	{341, 416},
	{349, 418},
	{356, 420},
	{364, 422},
	{372, 424},
	{381, 426},
	{390, 428},
	{399, 430},
	{410, 432},
	{420, 434},
	{431, 436},
	{443, 438},
	{455, 440},
	{468, 442},
	{482, 444},
	{497, 446},
	{512, 448}	
};

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
      iReadReg((u16) addr ,(u8*)&get_byte, imgsensor.i2c_write_id);
      return get_byte;
}

#define write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1,  imgsensor.i2c_write_id)

static kal_uint32 imx214_ATR(UINT16 DarkLimit, UINT16 OverExp)
{
    write_cmos_sensor(0x6e50,sensorATR_Info[DarkLimit].DarkLimit_H);
    write_cmos_sensor(0x6e51,sensorATR_Info[DarkLimit].DarkLimit_L);
    write_cmos_sensor(0x9340,sensorATR_Info[OverExp].OverExp_Min_H);
    write_cmos_sensor(0x9341,sensorATR_Info[OverExp].OverExp_Min_L);
    write_cmos_sensor(0x9342,sensorATR_Info[OverExp].OverExp_Max_H);
    write_cmos_sensor(0x9343,sensorATR_Info[OverExp].OverExp_Max_L);
    write_cmos_sensor(0x9706,0x10);
    write_cmos_sensor(0x9707,0x03);
    write_cmos_sensor(0x9708,0x03);
    write_cmos_sensor(0x9e24,0x00);
    write_cmos_sensor(0x9e25,0x8c);
    write_cmos_sensor(0x9e26,0x00);
    write_cmos_sensor(0x9e27,0x94);
    write_cmos_sensor(0x9e28,0x00);
    write_cmos_sensor(0x9e29,0x96);
    LOG_INF("DarkLimit 0x6e50(0x%x), 0x6e51(0x%x)\n",sensorATR_Info[DarkLimit].DarkLimit_H,
                                                     sensorATR_Info[DarkLimit].DarkLimit_L);
    LOG_INF("OverExpMin 0x9340(0x%x), 0x9341(0x%x)\n",sensorATR_Info[OverExp].OverExp_Min_H,
                                                     sensorATR_Info[OverExp].OverExp_Min_L);
    LOG_INF("OverExpMin 0x9342(0x%x), 0x9343(0x%x)\n",sensorATR_Info[OverExp].OverExp_Max_H,
                                                     sensorATR_Info[OverExp].OverExp_Max_L);
    return ERROR_NONE;
}
static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
       write_cmos_sensor(0x0104, 1); 
	   
	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0);
  
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
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
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0104, 1); 
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0);
	    }
	} else {
		// Extend frame length
		write_cmos_sensor(0x0104, 1); 
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0);
	}

	// Update Shutter
		write_cmos_sensor(0x0104, 1);      	
             write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
             write_cmos_sensor(0x0203, shutter  & 0xFF);	
             write_cmos_sensor(0x0104, 0);   

	LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

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
	kal_uint8 iI;

	for (iI = 0; iI < (MIPI_MaxGainIndex-1); iI++) {
			if(gain <= sensorGainMapping[iI][0]){	
				break;
			}
		}

	return sensorGainMapping[iI][1];
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

	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 32 * BASEGAIN)
			gain = 32 * BASEGAIN;		 
	}
 
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	  write_cmos_sensor(0x0104, 1);
      write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	  write_cmos_sensor(0x0205, reg_gain & 0xFF);
      write_cmos_sensor(0x0104, 0);

	
	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    kal_uint16 reg_gain;
    spin_lock(&imgsensor_drv_lock);
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)       
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
    if (imgsensor.autoflicker_en) { 
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        write_cmos_sensor(0x0104, 1); 
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0);
        }
    } else {
        write_cmos_sensor(0x0104, 1); 
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0);
    }
    write_cmos_sensor(0x0104, 1);
    /* Long exposure */
    write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor(0x0203, le  & 0xFF);
    /* Short exposure */
    write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor(0x0225, se  & 0xFF); 
    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain; 
    spin_unlock(&imgsensor_drv_lock);
    /* Global analog Gain for Long expo*/
    write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
    write_cmos_sensor(0x0205, reg_gain & 0xFF);
    /* Global analog Gain for Short expo*/
    write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
    write_cmos_sensor(0x0217, reg_gain & 0xFF);
    write_cmos_sensor(0x0104, 0);

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	kal_uint8 itemp;

	itemp=read_cmos_sensor(0x0101);
	itemp &= ~0x03;

	switch(image_mirror)
		{

		   case IMAGE_NORMAL:
		   	     write_cmos_sensor(0x0101, itemp);
			      break;
			
		   case IMAGE_V_MIRROR:		   	
			     write_cmos_sensor(0x0101, itemp | 0x02);
			     break;
			
		   case IMAGE_H_MIRROR:		   	
			     write_cmos_sensor(0x0101, itemp | 0x01);
			     break;
			
		   case IMAGE_HV_MIRROR:		   	
			     write_cmos_sensor(0x0101, itemp | 0x03);
			     break;
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
	LOG_INF("E\n");
    //init setting
    write_cmos_sensor(0x0136,0x18);
    write_cmos_sensor(0x0137,0x00);

    write_cmos_sensor(0x0101,0x00);
    write_cmos_sensor(0x0105,0x01);
    write_cmos_sensor(0x0106,0x01);
    write_cmos_sensor(0x4550,0x02);
    write_cmos_sensor(0x4601,0x00);
    write_cmos_sensor(0x4642,0x05);
    write_cmos_sensor(0x6276,0x00);
    write_cmos_sensor(0x900E,0x06);
    write_cmos_sensor(0xA802,0x90);
    write_cmos_sensor(0xA803,0x11);
    write_cmos_sensor(0xA804,0x62);
    write_cmos_sensor(0xA805,0x77);
    write_cmos_sensor(0xA806,0xAE);
    write_cmos_sensor(0xA807,0x34);
    write_cmos_sensor(0xA808,0xAE);
    write_cmos_sensor(0xA809,0x35);
    write_cmos_sensor(0xA80A,0x62);
    write_cmos_sensor(0xA80B,0x83);
    write_cmos_sensor(0xAE33,0x00);

    write_cmos_sensor(0x4174,0x00);
    write_cmos_sensor(0x4175,0x11);
    write_cmos_sensor(0x4612,0x29);
    write_cmos_sensor(0x461B,0x12);
    write_cmos_sensor(0x461F,0x06);
    write_cmos_sensor(0x4635,0x07);
    write_cmos_sensor(0x4637,0x30);
    write_cmos_sensor(0x463F,0x18);
    write_cmos_sensor(0x4641,0x0D);
    write_cmos_sensor(0x465B,0x12);
    write_cmos_sensor(0x465F,0x11);
    write_cmos_sensor(0x4663,0x11);
    write_cmos_sensor(0x4667,0x0F);
    write_cmos_sensor(0x466F,0x0F);
    write_cmos_sensor(0x470E,0x09);
    write_cmos_sensor(0x4909,0xAB);
    write_cmos_sensor(0x490B,0x95);
    write_cmos_sensor(0x4915,0x5D);
    write_cmos_sensor(0x4A5F,0xFF);
    write_cmos_sensor(0x4A61,0xFF);
    write_cmos_sensor(0x4A73,0x62);
    write_cmos_sensor(0x4A85,0x00);
    write_cmos_sensor(0x4A87,0xFF);
    write_cmos_sensor(0x583C,0x04);
    write_cmos_sensor(0x620E,0x04);
    write_cmos_sensor(0x6EB2,0x01);
    write_cmos_sensor(0x6EB3,0x00);
    write_cmos_sensor(0x9300,0x02);

    write_cmos_sensor(0x3001,0x07);
    write_cmos_sensor(0x6D12,0x3F);
    write_cmos_sensor(0x6D13,0xFF);
    write_cmos_sensor(0x9344,0x03);
    write_cmos_sensor(0x9706,0x10);
    write_cmos_sensor(0x9707,0x03);
    write_cmos_sensor(0x9708,0x03);
    write_cmos_sensor(0x9E04,0x01);
    write_cmos_sensor(0x9E05,0x00);
    write_cmos_sensor(0x9E0C,0x01);
    write_cmos_sensor(0x9E0D,0x02);
    write_cmos_sensor(0x9E24,0x00);
    write_cmos_sensor(0x9E25,0x8C);
    write_cmos_sensor(0x9E26,0x00);
    write_cmos_sensor(0x9E27,0x94);
    write_cmos_sensor(0x9E28,0x00);
    write_cmos_sensor(0x9E29,0x96);
    //write_cmos_sensor(0x5041,0x00);//no embedded data 

    write_cmos_sensor(0x69DB,0x01);
    write_cmos_sensor(0x6957,0x01);
    write_cmos_sensor(0x6987,0x17);
    write_cmos_sensor(0x698A,0x03);
    write_cmos_sensor(0x698B,0x03);
    write_cmos_sensor(0x0B8E,0x01);
    write_cmos_sensor(0x0B8F,0x00);
    write_cmos_sensor(0x0B90,0x01);
    write_cmos_sensor(0x0B91,0x00);
    write_cmos_sensor(0x0B92,0x01);
    write_cmos_sensor(0x0B93,0x00);
    write_cmos_sensor(0x0B94,0x01);
    write_cmos_sensor(0x0B95,0x00);
    write_cmos_sensor(0x6E50,0x00);
    write_cmos_sensor(0x6E51,0x32);
    write_cmos_sensor(0x9340,0x00);
    write_cmos_sensor(0x9341,0x3C);
    write_cmos_sensor(0x9342,0x03);
    write_cmos_sensor(0x9343,0xFF);
    write_cmos_sensor(0x0101,0x03);
}	/*	sensor_init  */


static void preview_setting(void)
{
	//Preview 2104*1560 30fps 24M MCLK 4lane 608Mbps/lane
	// preview 30.01fps
    write_cmos_sensor(0x0100,0x00);                       
    write_cmos_sensor(0x0114,0x03);                       
    write_cmos_sensor(0x0220,0x00);                       
    write_cmos_sensor(0x0221,0x11);                       
    write_cmos_sensor(0x0222,0x01);                       
    write_cmos_sensor(0x0340,0x08);                       
    write_cmos_sensor(0x0341,0x3E);                       
    write_cmos_sensor(0x0342,0x13);                       
    write_cmos_sensor(0x0343,0x90);                       
    write_cmos_sensor(0x0344,0x00);                       
    write_cmos_sensor(0x0345,0x00);                       
    write_cmos_sensor(0x0346,0x00);                       
    write_cmos_sensor(0x0347,0x00);                       
    write_cmos_sensor(0x0348,0x10);                       
    write_cmos_sensor(0x0349,0x6F);                       
    write_cmos_sensor(0x034A,0x0C);                       
    write_cmos_sensor(0x034B,0x2F);                       
    write_cmos_sensor(0x0381,0x01);                       
    write_cmos_sensor(0x0383,0x01);                       
    write_cmos_sensor(0x0385,0x01);                       
    write_cmos_sensor(0x0387,0x01);                       
    write_cmos_sensor(0x0900,0x01);                       
    write_cmos_sensor(0x0901,0x22);                       
    write_cmos_sensor(0x0902,0x02);                       
    write_cmos_sensor(0x3000,0x35);                       
    write_cmos_sensor(0x3054,0x01);                       
    write_cmos_sensor(0x305C,0x11);     
                      
    write_cmos_sensor(0x0112,0x0A);                       
    write_cmos_sensor(0x0113,0x0A);                       
    write_cmos_sensor(0x034C,0x08);                       
    write_cmos_sensor(0x034D,0x38);                       
    write_cmos_sensor(0x034E,0x06);                       
    write_cmos_sensor(0x034F,0x18);                       
    write_cmos_sensor(0x0401,0x00);                       
    write_cmos_sensor(0x0404,0x00);                       
    write_cmos_sensor(0x0405,0x10);                       
    write_cmos_sensor(0x0408,0x00);                       
    write_cmos_sensor(0x0409,0x00);                       
    write_cmos_sensor(0x040A,0x00);                       
    write_cmos_sensor(0x040B,0x00);                       
    write_cmos_sensor(0x040C,0x08);                       
    write_cmos_sensor(0x040D,0x38);                       
    write_cmos_sensor(0x040E,0x06);                       
    write_cmos_sensor(0x040F,0x18);
                           
    write_cmos_sensor(0x0301,0x05);                       
    write_cmos_sensor(0x0303,0x02);                       
    write_cmos_sensor(0x0305,0x03);                       
    write_cmos_sensor(0x0306,0x00);                       
    write_cmos_sensor(0x0307,0x64);                       
    write_cmos_sensor(0x0309,0x0A);                       
    write_cmos_sensor(0x030B,0x01);                       
    write_cmos_sensor(0x0310,0x00);
                           
    write_cmos_sensor(0x0820,0x0C);                       
    write_cmos_sensor(0x0821,0x80);                       
    write_cmos_sensor(0x0822,0x00);                       
    write_cmos_sensor(0x0823,0x00); 
                          
    write_cmos_sensor(0x3A03,0x06);                       
    write_cmos_sensor(0x3A04,0x68);                       
    write_cmos_sensor(0x3A05,0x01); 
                          
    write_cmos_sensor(0x0B06,0x01);                       
    write_cmos_sensor(0x30A2,0x00);
                           
    write_cmos_sensor(0x30B4,0x00);
                           
    write_cmos_sensor(0x3A02,0xFF); 
                          
    write_cmos_sensor(0x3011,0x00);                       
    write_cmos_sensor(0x3013,0x00);
                           
    write_cmos_sensor(0x0202,0x08);                       
    write_cmos_sensor(0x0203,0x34);                       
    write_cmos_sensor(0x0224,0x01);                       
    write_cmos_sensor(0x0225,0xF4);
                           
    write_cmos_sensor(0x0204,0x00);                       
    write_cmos_sensor(0x0205,0x00);                       
    write_cmos_sensor(0x020E,0x01);                       
    write_cmos_sensor(0x020F,0x00);                       
    write_cmos_sensor(0x0210,0x01);                       
    write_cmos_sensor(0x0211,0x00);                       
    write_cmos_sensor(0x0212,0x01);                       
    write_cmos_sensor(0x0213,0x00);                       
    write_cmos_sensor(0x0214,0x01);                       
    write_cmos_sensor(0x0215,0x00);                       
    write_cmos_sensor(0x0216,0x00);                       
    write_cmos_sensor(0x0217,0x00); 
                          
    write_cmos_sensor(0x4170,0x00);                       
    write_cmos_sensor(0x4171,0x10);                       
    write_cmos_sensor(0x4176,0x00);                       
    write_cmos_sensor(0x4177,0x3C);                       
    write_cmos_sensor(0xAE20,0x04);                       
    write_cmos_sensor(0xAE21,0x5C);

    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);
}   /*  preview_setting  */

static void preview_setting_HDR(void)
{
    LOG_INF("preview_setting_mHDR\n");
    write_cmos_sensor(0x0100,0x00);                       
    write_cmos_sensor(0x0114,0x03);                       
    write_cmos_sensor(0x0220,0x01);                       
    write_cmos_sensor(0x0221,0x22);                       
    write_cmos_sensor(0x0222,0x08);                       
    write_cmos_sensor(0x0340,0x08);                       
    write_cmos_sensor(0x0341,0x3E);                       
    write_cmos_sensor(0x0342,0x13);                       
    write_cmos_sensor(0x0343,0x90);                       
    write_cmos_sensor(0x0344,0x00);                       
    write_cmos_sensor(0x0345,0x00);                       
    write_cmos_sensor(0x0346,0x00);                       
    write_cmos_sensor(0x0347,0x00);                       
    write_cmos_sensor(0x0348,0x10);                       
    write_cmos_sensor(0x0349,0x6F);                       
    write_cmos_sensor(0x034A,0x0C);                       
    write_cmos_sensor(0x034B,0x2F);                       
    write_cmos_sensor(0x0381,0x01);                       
    write_cmos_sensor(0x0383,0x01);                       
    write_cmos_sensor(0x0385,0x01);                       
    write_cmos_sensor(0x0387,0x01);                       
    write_cmos_sensor(0x0900,0x00);                       
    write_cmos_sensor(0x0901,0x00);                       
    write_cmos_sensor(0x0902,0x00);                       
    write_cmos_sensor(0x3000,0x35);                       
    write_cmos_sensor(0x3054,0x01);                       
    write_cmos_sensor(0x305C,0x11);     
                      
    write_cmos_sensor(0x0112,0x0A);                       
    write_cmos_sensor(0x0113,0x0A);                       
    write_cmos_sensor(0x034C,0x08);                       
    write_cmos_sensor(0x034D,0x38);                       
    write_cmos_sensor(0x034E,0x06);                       
    write_cmos_sensor(0x034F,0x18);                       
    write_cmos_sensor(0x0401,0x00);                       
    write_cmos_sensor(0x0404,0x00);                       
    write_cmos_sensor(0x0405,0x10);                       
    write_cmos_sensor(0x0408,0x00);                       
    write_cmos_sensor(0x0409,0x00);                       
    write_cmos_sensor(0x040A,0x00);                       
    write_cmos_sensor(0x040B,0x00);                       
    write_cmos_sensor(0x040C,0x08);                       
    write_cmos_sensor(0x040D,0x38);                       
    write_cmos_sensor(0x040E,0x06);                       
    write_cmos_sensor(0x040F,0x18);
                           
    write_cmos_sensor(0x0301,0x05);                       
    write_cmos_sensor(0x0303,0x02);                       
    write_cmos_sensor(0x0305,0x03);                       
    write_cmos_sensor(0x0306,0x00);                       
    write_cmos_sensor(0x0307,0x64);                       
    write_cmos_sensor(0x0309,0x0A);                       
    write_cmos_sensor(0x030B,0x01);                       
    write_cmos_sensor(0x0310,0x00);
                           
    write_cmos_sensor(0x0820,0x0C);                       
    write_cmos_sensor(0x0821,0x80);                       
    write_cmos_sensor(0x0822,0x00);                       
    write_cmos_sensor(0x0823,0x00); 
                          
    write_cmos_sensor(0x3A03,0x06);                       
    write_cmos_sensor(0x3A04,0xE8);                       
    write_cmos_sensor(0x3A05,0x01); 
                          
    write_cmos_sensor(0x0B06,0x01);                       
    write_cmos_sensor(0x30A2,0x00);
                           
    write_cmos_sensor(0x30B4,0x00);
                           
    write_cmos_sensor(0x3A02,0x06); 
                          
    write_cmos_sensor(0x3011,0x00);                       
    write_cmos_sensor(0x3013,0x01);
                           
    write_cmos_sensor(0x0202,0x08);                       
    write_cmos_sensor(0x0203,0x34);                       
    write_cmos_sensor(0x0224,0x01);                       
    write_cmos_sensor(0x0225,0x06);
                           
    write_cmos_sensor(0x0204,0x00);                       
    write_cmos_sensor(0x0205,0x00);                       
    write_cmos_sensor(0x020E,0x01);                       
    write_cmos_sensor(0x020F,0x00);                       
    write_cmos_sensor(0x0210,0x01);                       
    write_cmos_sensor(0x0211,0x00);                       
    write_cmos_sensor(0x0212,0x01);                       
    write_cmos_sensor(0x0213,0x00);                       
    write_cmos_sensor(0x0214,0x01);                       
    write_cmos_sensor(0x0215,0x00);                       
    write_cmos_sensor(0x0216,0x00);                       
    write_cmos_sensor(0x0217,0x00); 
                          
    write_cmos_sensor(0x4170,0x00);                       
    write_cmos_sensor(0x4171,0x10);                       
    write_cmos_sensor(0x4176,0x00);                       
    write_cmos_sensor(0x4177,0x3C);                       
    write_cmos_sensor(0xAE20,0x04);                       
    write_cmos_sensor(0xAE21,0x5C);
    //mHDR  relation setting                  
    write_cmos_sensor(0x3010,0x00);                       
    write_cmos_sensor(0x6D3A,0x00); // 0: 16X16, 1:8X8    
    write_cmos_sensor(0x3011,0x00);                       
    write_cmos_sensor(0x3013,0x01); // STATS output Enable
    write_cmos_sensor(0x5068,0x35);                       
    write_cmos_sensor(0x5069,0x01);                       
    write_cmos_sensor(0x30C2,0x00);                       
    write_cmos_sensor(0x30C3,0x40);                       
    write_cmos_sensor(0x610A,0x08);
    //[0]:1 hdr enable, [1]:0 LE/SE use same gain, 1 LE/SE separate gain
    //[5]:0 auto, 1:direct 
    write_cmos_sensor(0x0220,0x01);
    // hdr binning mode
    write_cmos_sensor(0x0221,0x22); 
    //LE/SE ration 1,2,4,8
    write_cmos_sensor(0x0222,0x08);
    //ATR
    imx214_ATR(3,3);
#if 0
    /*
    *   FAE PROVIDIE
    */
    write_cmos_sensor(0x9313,0x40);
    write_cmos_sensor(0x9318,0x0C);
    write_cmos_sensor(0x9319,0x00);
    write_cmos_sensor(0x9344,0x00);
    write_cmos_sensor(0x9348,0x01);
    write_cmos_sensor(0x9349,0xC2);
    write_cmos_sensor(0x934A,0x2);
    write_cmos_sensor(0x934B,0xE4);
    write_cmos_sensor(0x934E,0x0);

    /*
    *   DIFF for RATIO:1 & 8
    */
    //write_cmos_sensor(0x934F,0x47);  // RATIO: 1
    write_cmos_sensor(0x934F,0xC7);  // RATIO: 8

    write_cmos_sensor(0x9354,0x01);
    write_cmos_sensor(0x9355,0x80);
    write_cmos_sensor(0x9356,0x2);
    write_cmos_sensor(0x9357,0x1C);

    // General
    write_cmos_sensor(0x6939,0x3);
    write_cmos_sensor(0x693b,0x3);
    write_cmos_sensor(0x4550,0x2);
    write_cmos_sensor(0x6227,0x11);
#endif
    /*
    *   END OF FAE PROVIDIE
    */
    // Normal: 0x00, ZigZag: 0x01
    if(imgsensor.ihdr_mode == 9)
    {
        write_cmos_sensor(0x30b2,0x01);                       
        write_cmos_sensor(0x30b3,0x01);
        write_cmos_sensor(0x30b4,0x01);                       
        write_cmos_sensor(0x30b5,0x01);
        write_cmos_sensor(0x30b6,0x01);                       
        write_cmos_sensor(0x30b7,0x01);
        write_cmos_sensor(0x30b8,0x01);                       
        write_cmos_sensor(0x30b9,0x01);
        write_cmos_sensor(0x30ba,0x01);                       
        write_cmos_sensor(0x30bb,0x01);
        write_cmos_sensor(0x30bc,0x01);
    }
    else
    {
        write_cmos_sensor(0x30b2,0x00);                       
        write_cmos_sensor(0x30b3,0x00);
        write_cmos_sensor(0x30b4,0x00);                       
        write_cmos_sensor(0x30b5,0x00);
        write_cmos_sensor(0x30b6,0x00);                       
        write_cmos_sensor(0x30b7,0x00);
        write_cmos_sensor(0x30b8,0x00);                       
        write_cmos_sensor(0x30b9,0x00);
        write_cmos_sensor(0x30ba,0x00);                       
        write_cmos_sensor(0x30bb,0x00);
        write_cmos_sensor(0x30bc,0x00);
    }

    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);
    LOG_INF("preview_setting_mHDR mode 0x0220(0x01), Ratio 0x0222(0x08)\n");

}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
    // full size 29.76fps  
    // capture setting 4208*3120  480MCLK 1.2Gp/lane
    
    if(currefps==300)
    {   
        // full size 30.33ps 
        write_cmos_sensor(0x0100,0x00);                       
        write_cmos_sensor(0x0114,0x03);                       
        write_cmos_sensor(0x0220,0x00);                       
        write_cmos_sensor(0x0221,0x11);                       
        write_cmos_sensor(0x0222,0x01);                       
        write_cmos_sensor(0x0340,0x0C);                       
        write_cmos_sensor(0x0341,0x58);                       
        write_cmos_sensor(0x0342,0x13);                       
        write_cmos_sensor(0x0343,0x90);                       
        write_cmos_sensor(0x0344,0x00);                       
        write_cmos_sensor(0x0345,0x00);                       
        write_cmos_sensor(0x0346,0x00);                       
        write_cmos_sensor(0x0347,0x00);                       
        write_cmos_sensor(0x0348,0x10);                       
        write_cmos_sensor(0x0349,0x6F);                       
        write_cmos_sensor(0x034A,0x0C);                       
        write_cmos_sensor(0x034B,0x2F);                       
        write_cmos_sensor(0x0381,0x01);                       
        write_cmos_sensor(0x0383,0x01);                       
        write_cmos_sensor(0x0385,0x01);                       
        write_cmos_sensor(0x0387,0x01);                       
        write_cmos_sensor(0x0900,0x00);                       
        write_cmos_sensor(0x0901,0x00);                       
        write_cmos_sensor(0x0902,0x00);                       
        write_cmos_sensor(0x3000,0x35);                       
        write_cmos_sensor(0x3054,0x01);                       
        write_cmos_sensor(0x305C,0x11);     
               
        write_cmos_sensor(0x0112,0x0A);                       
        write_cmos_sensor(0x0113,0x0A);                       
        write_cmos_sensor(0x034C,0x10);                       
        write_cmos_sensor(0x034D,0x70);                       
        write_cmos_sensor(0x034E,0x0C);                       
        write_cmos_sensor(0x034F,0x30);                       
        write_cmos_sensor(0x0401,0x00);                       
        write_cmos_sensor(0x0404,0x00);                       
        write_cmos_sensor(0x0405,0x10);                       
        write_cmos_sensor(0x0408,0x00);                       
        write_cmos_sensor(0x0409,0x00);                       
        write_cmos_sensor(0x040A,0x00);                       
        write_cmos_sensor(0x040B,0x00);                       
        write_cmos_sensor(0x040C,0x10);                       
        write_cmos_sensor(0x040D,0x70);                       
        write_cmos_sensor(0x040E,0x0C);                       
        write_cmos_sensor(0x040F,0x30);
                    
        write_cmos_sensor(0x0301,0x05);                       
        write_cmos_sensor(0x0303,0x02);                       
        write_cmos_sensor(0x0305,0x03);                       
        write_cmos_sensor(0x0306,0x00);                       
        write_cmos_sensor(0x0307,0x96);                       
        write_cmos_sensor(0x0309,0x0A);                       
        write_cmos_sensor(0x030B,0x01);                       
        write_cmos_sensor(0x0310,0x00);
                    
        write_cmos_sensor(0x0820,0x12);                       
        write_cmos_sensor(0x0821,0xC0);                       
        write_cmos_sensor(0x0822,0x00);                       
        write_cmos_sensor(0x0823,0x00); 
                   
        write_cmos_sensor(0x3A03,0x09);                       
        write_cmos_sensor(0x3A04,0x20);                       
        write_cmos_sensor(0x3A05,0x01); 
                   
        write_cmos_sensor(0x0B06,0x01);                       
        write_cmos_sensor(0x30A2,0x00);
                    
        write_cmos_sensor(0x30B4,0x00);
                    
        write_cmos_sensor(0x3A02,0xff); 
                   
        write_cmos_sensor(0x3011,0x00);                       
        write_cmos_sensor(0x3013,0x01);
                    
        write_cmos_sensor(0x0202,0x0C);                       
        write_cmos_sensor(0x0203,0x4E);                       
        write_cmos_sensor(0x0224,0x01);                       
        write_cmos_sensor(0x0225,0xF4);
                    
        write_cmos_sensor(0x0204,0x00);                       
        write_cmos_sensor(0x0205,0x00);                       
        write_cmos_sensor(0x020E,0x01);                       
        write_cmos_sensor(0x020F,0x00);                       
        write_cmos_sensor(0x0210,0x01);                       
        write_cmos_sensor(0x0211,0x00);                       
        write_cmos_sensor(0x0212,0x01);                       
        write_cmos_sensor(0x0213,0x00);                       
        write_cmos_sensor(0x0214,0x01);                       
        write_cmos_sensor(0x0215,0x00);                       
        write_cmos_sensor(0x0216,0x00);                       
        write_cmos_sensor(0x0217,0x00); 
                   
        write_cmos_sensor(0x4170,0x00);                       
        write_cmos_sensor(0x4171,0x10);                       
        write_cmos_sensor(0x4176,0x00);                       
        write_cmos_sensor(0x4177,0x3C);                       
        write_cmos_sensor(0xAE20,0x04);                       
        write_cmos_sensor(0xAE21,0x5C);

        write_cmos_sensor(0x0138,0x01);
        write_cmos_sensor(0x0100,0x01);
        
    }
    else{
        // full siez 24pfs
        write_cmos_sensor(0x0100,0x00);

        write_cmos_sensor(0x0114,0x03);
        write_cmos_sensor(0x0220,0x00);
        write_cmos_sensor(0x0221,0x11);
        write_cmos_sensor(0x0222,0x01);
        write_cmos_sensor(0x0340,0x0C);
        write_cmos_sensor(0x0341,0x94);
        write_cmos_sensor(0x0342,0x13);
        write_cmos_sensor(0x0343,0x90);
        write_cmos_sensor(0x0344,0x00);
        write_cmos_sensor(0x0345,0x00);
        write_cmos_sensor(0x0346,0x00);
        write_cmos_sensor(0x0347,0x00);
        write_cmos_sensor(0x0348,0x10);
        write_cmos_sensor(0x0349,0x6F);
        write_cmos_sensor(0x034A,0x0C);
        write_cmos_sensor(0x034B,0x2F);
        write_cmos_sensor(0x0381,0x01);
        write_cmos_sensor(0x0383,0x01);
        write_cmos_sensor(0x0385,0x01);
        write_cmos_sensor(0x0387,0x01);
        write_cmos_sensor(0x0900,0x00);
        write_cmos_sensor(0x0901,0x00);
        write_cmos_sensor(0x0902,0x00);
        write_cmos_sensor(0x3000,0x35);
        write_cmos_sensor(0x3054,0x01);
        write_cmos_sensor(0x305C,0x11);

        write_cmos_sensor(0x0112,0x0A);
        write_cmos_sensor(0x0113,0x0A);
        write_cmos_sensor(0x034C,0x10);
        write_cmos_sensor(0x034D,0x70);
        write_cmos_sensor(0x034E,0x0C);
        write_cmos_sensor(0x034F,0x30);
        write_cmos_sensor(0x0401,0x00);
        write_cmos_sensor(0x0404,0x00);
        write_cmos_sensor(0x0405,0x10);
        write_cmos_sensor(0x0408,0x00);
        write_cmos_sensor(0x0409,0x00);
        write_cmos_sensor(0x040A,0x00);
        write_cmos_sensor(0x040B,0x00);
        write_cmos_sensor(0x040C,0x10);
        write_cmos_sensor(0x040D,0x70);
        write_cmos_sensor(0x040E,0x0C);
        write_cmos_sensor(0x040F,0x30);

        write_cmos_sensor(0x0301,0x05);
        write_cmos_sensor(0x0303,0x02);
        write_cmos_sensor(0x0305,0x03);
        write_cmos_sensor(0x0306,0x00);
        write_cmos_sensor(0x0307,0x79);
        write_cmos_sensor(0x0309,0x0A);
        write_cmos_sensor(0x030B,0x01);
        write_cmos_sensor(0x0310,0x00);

        write_cmos_sensor(0x0820,0x0F);
        write_cmos_sensor(0x0821,0x20);
        write_cmos_sensor(0x0822,0x00);
        write_cmos_sensor(0x0823,0x00);

        write_cmos_sensor(0x3A03,0x08);
        write_cmos_sensor(0x3A04,0xC0);
        write_cmos_sensor(0x3A05,0x02);

        write_cmos_sensor(0x0B06,0x01);
        write_cmos_sensor(0x30A2,0x00);
        write_cmos_sensor(0x30B4,0x00);
        write_cmos_sensor(0x3A02,0xFF);
        write_cmos_sensor(0x3013,0x00);
        write_cmos_sensor(0x0202,0x0C);
        write_cmos_sensor(0x0203,0x8A);
        write_cmos_sensor(0x0224,0x01);
        write_cmos_sensor(0x0225,0xF4);
        write_cmos_sensor(0x0204,0x00);
        write_cmos_sensor(0x0205,0x00);
        write_cmos_sensor(0x020E,0x01);
        write_cmos_sensor(0x020F,0x00);
        write_cmos_sensor(0x0210,0x01);
        write_cmos_sensor(0x0211,0x00);
        write_cmos_sensor(0x0212,0x01);
        write_cmos_sensor(0x0213,0x00);
        write_cmos_sensor(0x0214,0x01);
        write_cmos_sensor(0x0215,0x00);
        write_cmos_sensor(0x0216,0x00);
        write_cmos_sensor(0x0217,0x00);
        write_cmos_sensor(0x4170,0x00);
        write_cmos_sensor(0x4171,0x10);
        write_cmos_sensor(0x4176,0x00);
        write_cmos_sensor(0x4177,0x3C);
        write_cmos_sensor(0xAE20,0x04);
        write_cmos_sensor(0xAE21,0x5C);

        write_cmos_sensor(0x0138,0x01);
        write_cmos_sensor(0x0100,0x01);
    }
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
    // full size 30.33ps 
    write_cmos_sensor(0x0100,0x00);                       
    write_cmos_sensor(0x0114,0x03);                       
    write_cmos_sensor(0x0220,0x00);                       
    write_cmos_sensor(0x0221,0x11);                       
    write_cmos_sensor(0x0222,0x01);                       
    write_cmos_sensor(0x0340,0x0C);                       
    write_cmos_sensor(0x0341,0x58);                       
    write_cmos_sensor(0x0342,0x13);                       
    write_cmos_sensor(0x0343,0x90);                       
    write_cmos_sensor(0x0344,0x00);                       
    write_cmos_sensor(0x0345,0x00);                       
    write_cmos_sensor(0x0346,0x00);                       
    write_cmos_sensor(0x0347,0x00);                       
    write_cmos_sensor(0x0348,0x10);                       
    write_cmos_sensor(0x0349,0x6F);                       
    write_cmos_sensor(0x034A,0x0C);                       
    write_cmos_sensor(0x034B,0x2F);                       
    write_cmos_sensor(0x0381,0x01);                       
    write_cmos_sensor(0x0383,0x01);                       
    write_cmos_sensor(0x0385,0x01);                       
    write_cmos_sensor(0x0387,0x01);                       
    write_cmos_sensor(0x0900,0x00);                       
    write_cmos_sensor(0x0901,0x00);                       
    write_cmos_sensor(0x0902,0x00);                       
    write_cmos_sensor(0x3000,0x35);                       
    write_cmos_sensor(0x3054,0x01);                       
    write_cmos_sensor(0x305C,0x11);     
               
    write_cmos_sensor(0x0112,0x0A);                       
    write_cmos_sensor(0x0113,0x0A);                       
    write_cmos_sensor(0x034C,0x10);                       
    write_cmos_sensor(0x034D,0x70);                       
    write_cmos_sensor(0x034E,0x0C);                       
    write_cmos_sensor(0x034F,0x30);                       
    write_cmos_sensor(0x0401,0x00);                       
    write_cmos_sensor(0x0404,0x00);                       
    write_cmos_sensor(0x0405,0x10);                       
    write_cmos_sensor(0x0408,0x00);                       
    write_cmos_sensor(0x0409,0x00);                       
    write_cmos_sensor(0x040A,0x00);                       
    write_cmos_sensor(0x040B,0x00);                       
    write_cmos_sensor(0x040C,0x10);                       
    write_cmos_sensor(0x040D,0x70);                       
    write_cmos_sensor(0x040E,0x0C);                       
    write_cmos_sensor(0x040F,0x30);
                    
    write_cmos_sensor(0x0301,0x05);                       
    write_cmos_sensor(0x0303,0x02);                       
    write_cmos_sensor(0x0305,0x03);                       
    write_cmos_sensor(0x0306,0x00);                       
    write_cmos_sensor(0x0307,0x96);                       
    write_cmos_sensor(0x0309,0x0A);                       
    write_cmos_sensor(0x030B,0x01);                       
    write_cmos_sensor(0x0310,0x00);
                    
    write_cmos_sensor(0x0820,0x12);                       
    write_cmos_sensor(0x0821,0xC0);                       
    write_cmos_sensor(0x0822,0x00);                       
    write_cmos_sensor(0x0823,0x00); 
                   
    write_cmos_sensor(0x3A03,0x09);                       
    write_cmos_sensor(0x3A04,0x20);                       
    write_cmos_sensor(0x3A05,0x01); 
                   
    write_cmos_sensor(0x0B06,0x01);                       
    write_cmos_sensor(0x30A2,0x00);
                    
    write_cmos_sensor(0x30B4,0x00);
                    
    write_cmos_sensor(0x3A02,0xff); 
                   
    write_cmos_sensor(0x3011,0x00);                       
    write_cmos_sensor(0x3013,0x01);
                    
    write_cmos_sensor(0x0202,0x0C);                       
    write_cmos_sensor(0x0203,0x4E);                       
    write_cmos_sensor(0x0224,0x01);                       
    write_cmos_sensor(0x0225,0xF4);
                    
    write_cmos_sensor(0x0204,0x00);                       
    write_cmos_sensor(0x0205,0x00);                       
    write_cmos_sensor(0x020E,0x01);                       
    write_cmos_sensor(0x020F,0x00);                       
    write_cmos_sensor(0x0210,0x01);                       
    write_cmos_sensor(0x0211,0x00);                       
    write_cmos_sensor(0x0212,0x01);                       
    write_cmos_sensor(0x0213,0x00);                       
    write_cmos_sensor(0x0214,0x01);                       
    write_cmos_sensor(0x0215,0x00);                       
    write_cmos_sensor(0x0216,0x00);                       
    write_cmos_sensor(0x0217,0x00); 
                   
    write_cmos_sensor(0x4170,0x00);                       
    write_cmos_sensor(0x4171,0x10);                       
    write_cmos_sensor(0x4176,0x00);                       
    write_cmos_sensor(0x4177,0x3C);                       
    write_cmos_sensor(0xAE20,0x04);                       
    write_cmos_sensor(0xAE21,0x5C);

    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);

}

static void fullsize_setting_HDR(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
    // full size 30.33ps 
    write_cmos_sensor(0x0100,0x00);                       
    write_cmos_sensor(0x0114,0x03);                       
    write_cmos_sensor(0x0220,0x01);                       
    write_cmos_sensor(0x0221,0x11);                       
    write_cmos_sensor(0x0222,0x08);                       
    write_cmos_sensor(0x0340,0x0C);                       
    write_cmos_sensor(0x0341,0x58);                       
    write_cmos_sensor(0x0342,0x13);                       
    write_cmos_sensor(0x0343,0x90);                       
    write_cmos_sensor(0x0344,0x00);                       
    write_cmos_sensor(0x0345,0x00);                       
    write_cmos_sensor(0x0346,0x00);                       
    write_cmos_sensor(0x0347,0x00);                       
    write_cmos_sensor(0x0348,0x10);                       
    write_cmos_sensor(0x0349,0x6F);                       
    write_cmos_sensor(0x034A,0x0C);                       
    write_cmos_sensor(0x034B,0x2F);                       
    write_cmos_sensor(0x0381,0x01);                       
    write_cmos_sensor(0x0383,0x01);                       
    write_cmos_sensor(0x0385,0x01);                       
    write_cmos_sensor(0x0387,0x01);                       
    write_cmos_sensor(0x0900,0x00);                       
    write_cmos_sensor(0x0901,0x00);                       
    write_cmos_sensor(0x0902,0x00);                       
    write_cmos_sensor(0x3000,0x35);                       
    write_cmos_sensor(0x3054,0x01);                       
    write_cmos_sensor(0x305C,0x11);     
               
    write_cmos_sensor(0x0112,0x0A);                       
    write_cmos_sensor(0x0113,0x0A);                       
    write_cmos_sensor(0x034C,0x10);                       
    write_cmos_sensor(0x034D,0x70);                       
    write_cmos_sensor(0x034E,0x0C);                       
    write_cmos_sensor(0x034F,0x30);                       
    write_cmos_sensor(0x0401,0x00);                       
    write_cmos_sensor(0x0404,0x00);                       
    write_cmos_sensor(0x0405,0x10);                       
    write_cmos_sensor(0x0408,0x00);                       
    write_cmos_sensor(0x0409,0x00);                       
    write_cmos_sensor(0x040A,0x00);                       
    write_cmos_sensor(0x040B,0x00);                       
    write_cmos_sensor(0x040C,0x10);                       
    write_cmos_sensor(0x040D,0x70);                       
    write_cmos_sensor(0x040E,0x0C);                       
    write_cmos_sensor(0x040F,0x30);
                    
    write_cmos_sensor(0x0301,0x05);                       
    write_cmos_sensor(0x0303,0x02);                       
    write_cmos_sensor(0x0305,0x03);                       
    write_cmos_sensor(0x0306,0x00);                       
    write_cmos_sensor(0x0307,0x96);                       
    write_cmos_sensor(0x0309,0x0A);                       
    write_cmos_sensor(0x030B,0x01);                       
    write_cmos_sensor(0x0310,0x00);
                    
    write_cmos_sensor(0x0820,0x12);                       
    write_cmos_sensor(0x0821,0xC0);                       
    write_cmos_sensor(0x0822,0x00);                       
    write_cmos_sensor(0x0823,0x00); 
                   
    write_cmos_sensor(0x3A03,0x08);                       
    write_cmos_sensor(0x3A04,0x90);                       
    write_cmos_sensor(0x3A05,0x01); 
                   
    write_cmos_sensor(0x0B06,0x01);                       
    write_cmos_sensor(0x30A2,0x00);
                    
    write_cmos_sensor(0x30B4,0x00);
                    
    write_cmos_sensor(0x3A02,0x06); 
                   
    write_cmos_sensor(0x3011,0x00);                       
    write_cmos_sensor(0x3013,0x01);
                    
    write_cmos_sensor(0x0202,0x0C);                       
    write_cmos_sensor(0x0203,0x4E);                       
    write_cmos_sensor(0x0224,0x01);                       
    write_cmos_sensor(0x0225,0x89);
                    
    write_cmos_sensor(0x0204,0x00);                       
    write_cmos_sensor(0x0205,0x00);                       
    write_cmos_sensor(0x020E,0x01);                       
    write_cmos_sensor(0x020F,0x00);                       
    write_cmos_sensor(0x0210,0x01);                       
    write_cmos_sensor(0x0211,0x00);                       
    write_cmos_sensor(0x0212,0x01);                       
    write_cmos_sensor(0x0213,0x00);                       
    write_cmos_sensor(0x0214,0x01);                       
    write_cmos_sensor(0x0215,0x00);                       
    write_cmos_sensor(0x0216,0x00);                       
    write_cmos_sensor(0x0217,0x00); 
                   
    write_cmos_sensor(0x4170,0x00);                       
    write_cmos_sensor(0x4171,0x10);                       
    write_cmos_sensor(0x4176,0x00);                       
    write_cmos_sensor(0x4177,0x3C);                       
    write_cmos_sensor(0xAE20,0x04);                       
    write_cmos_sensor(0xAE21,0x5C);
    //mHDR  relation setting                  
    write_cmos_sensor(0x3010,0x00);                       
    write_cmos_sensor(0x6D3A,0x00); // 0: 16X16, 1:8X8    
    write_cmos_sensor(0x3011,0x00);                       
    write_cmos_sensor(0x3013,0x01); // STATS output Enable
    write_cmos_sensor(0x5068,0x35);                       
    write_cmos_sensor(0x5069,0x01);                       
    write_cmos_sensor(0x30C2,0x00);                       
    write_cmos_sensor(0x30C3,0x40);                       
    write_cmos_sensor(0x610A,0x08);
    //[0]:1 hdr enable, [1]:0 LE/SE use same gain, 1 LE/SE separate gain
    //[5]:0 auto, 1:direct 
    write_cmos_sensor(0x0220,0x01);
    // hdr binning mode
    write_cmos_sensor(0x0221,0x11); 
    //LE/SE ration 1,2,4,8
    write_cmos_sensor(0x0222,0x08);
    //ATR
    imx214_ATR(3,3);
    
    // Normal: 0x00, ZigZag: 0x01
    if(imgsensor.ihdr_mode == 9)
    {
        write_cmos_sensor(0x30b2,0x01);                       
        write_cmos_sensor(0x30b3,0x01);
        write_cmos_sensor(0x30b4,0x01);                       
        write_cmos_sensor(0x30b5,0x01);
        write_cmos_sensor(0x30b6,0x01);                       
        write_cmos_sensor(0x30b7,0x01);
        write_cmos_sensor(0x30b8,0x01);                       
        write_cmos_sensor(0x30b9,0x01);
        write_cmos_sensor(0x30ba,0x01);                       
        write_cmos_sensor(0x30bb,0x01);
        write_cmos_sensor(0x30bc,0x01);
    }
    else
    {
        write_cmos_sensor(0x30b2,0x00);                       
        write_cmos_sensor(0x30b3,0x00);
        write_cmos_sensor(0x30b4,0x00);                       
        write_cmos_sensor(0x30b5,0x00);
        write_cmos_sensor(0x30b6,0x00);                       
        write_cmos_sensor(0x30b7,0x00);
        write_cmos_sensor(0x30b8,0x00);                       
        write_cmos_sensor(0x30b9,0x00);
        write_cmos_sensor(0x30ba,0x00);                       
        write_cmos_sensor(0x30bb,0x00);
        write_cmos_sensor(0x30bc,0x00);
    }
    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);

}

static void hs_video_setting()
{
	LOG_INF("E\n");
	//1080p 60fps
	write_cmos_sensor(0x0100,0x00);	
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x01);
	write_cmos_sensor(0x0340,0x05);
	write_cmos_sensor(0x0341,0x08);
	write_cmos_sensor(0x0342,0x13);
	write_cmos_sensor(0x0343,0x90);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x01);
	write_cmos_sensor(0x0347,0x78);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0A);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x02);
	write_cmos_sensor(0x3000,0x35);
	write_cmos_sensor(0x3054,0x01);
	write_cmos_sensor(0x305C,0x11);
	
	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x08);
	write_cmos_sensor(0x034D,0x38);
	write_cmos_sensor(0x034E,0x04);
	write_cmos_sensor(0x034F,0xA0);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x08);
	write_cmos_sensor(0x040D,0x38);
	write_cmos_sensor(0x040E,0x04);
	write_cmos_sensor(0x040F,0xA0);
	
	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x03);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x79);//79
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x0310,0x00);
	
	write_cmos_sensor(0x0820,0x0F);//0F
	write_cmos_sensor(0x0821,0x20);// 20
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);
	write_cmos_sensor(0x3A03,0x06);
	write_cmos_sensor(0x3A04,0x68);
	write_cmos_sensor(0x3A05,0x01);
	write_cmos_sensor(0x0B06,0x01);
	write_cmos_sensor(0x30A2,0x00);
	write_cmos_sensor(0x30B4,0x00);
	write_cmos_sensor(0x3A02,0xFF);
	write_cmos_sensor(0x3013,0x00);
	write_cmos_sensor(0x0202,0x04);
	write_cmos_sensor(0x0203,0xFE);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);
	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x4170,0x00);
	write_cmos_sensor(0x4171,0x10);
	write_cmos_sensor(0x4176,0x00);
	write_cmos_sensor(0x4177,0x3C);
	write_cmos_sensor(0xAE20,0x04);
	write_cmos_sensor(0xAE21,0x5C);

	write_cmos_sensor(0x0138,0x01);
      write_cmos_sensor(0x0100,0x01);	

}

static void slim_video_setting()
{
	LOG_INF("E\n");
	hs_video_setting();
}

static void vhdr_setting()
{
     	LOG_INF("E\n");
write_cmos_sensor(0x0114,0x03);
write_cmos_sensor(0x0220,0x01);
write_cmos_sensor(0x0221,0x22);
write_cmos_sensor(0x0222,0x10);
write_cmos_sensor(0x0340,0x06);
write_cmos_sensor(0x0341,0x68);
write_cmos_sensor(0x0342,0x13);
write_cmos_sensor(0x0343,0x90);
write_cmos_sensor(0x0344,0x00);
write_cmos_sensor(0x0345,0x00);
write_cmos_sensor(0x0346,0x01);
write_cmos_sensor(0x0347,0x78);
write_cmos_sensor(0x0348,0x10);
write_cmos_sensor(0x0349,0x6F);
write_cmos_sensor(0x034A,0x0A);
write_cmos_sensor(0x034B,0xB7);
write_cmos_sensor(0x0381,0x01);
write_cmos_sensor(0x0383,0x01);
write_cmos_sensor(0x0385,0x01);
write_cmos_sensor(0x0387,0x01);
write_cmos_sensor(0x0900,0x00);
write_cmos_sensor(0x0901,0x00);
write_cmos_sensor(0x0902,0x00);
write_cmos_sensor(0x3000,0x35);
write_cmos_sensor(0x3054,0x01);
write_cmos_sensor(0x305C,0x11);

write_cmos_sensor(0x0112,0x0A);
write_cmos_sensor(0x0113,0x0A);
write_cmos_sensor(0x034C,0x08);
write_cmos_sensor(0x034D,0x38);
write_cmos_sensor(0x034E,0x04);
write_cmos_sensor(0x034F,0xA0);
write_cmos_sensor(0x0401,0x00);
write_cmos_sensor(0x0404,0x00);
write_cmos_sensor(0x0405,0x10);
write_cmos_sensor(0x0408,0x00);
write_cmos_sensor(0x0409,0x00);
write_cmos_sensor(0x040A,0x00);
write_cmos_sensor(0x040B,0x00);
write_cmos_sensor(0x040C,0x08);
write_cmos_sensor(0x040D,0x38);
write_cmos_sensor(0x040E,0x04);
write_cmos_sensor(0x040F,0xA0);

write_cmos_sensor(0x0301,0x05);
write_cmos_sensor(0x0303,0x02);
write_cmos_sensor(0x0305,0x03);
write_cmos_sensor(0x0306,0x00);
write_cmos_sensor(0x0307,0x4D);
write_cmos_sensor(0x0309,0x0A);
write_cmos_sensor(0x030B,0x01);
write_cmos_sensor(0x0310,0x00);
write_cmos_sensor(0x0820,0x09);
write_cmos_sensor(0x0821,0xA0);
write_cmos_sensor(0x0822,0x00);
write_cmos_sensor(0x0823,0x00);
write_cmos_sensor(0x3A03,0x06);
write_cmos_sensor(0x3A04,0xE8);
write_cmos_sensor(0x3A05,0x01);
write_cmos_sensor(0x0B06,0x01);
write_cmos_sensor(0x30A2,0x00);
write_cmos_sensor(0x30B4,0x00);
write_cmos_sensor(0x3A02,0x06);
write_cmos_sensor(0x3013,0x01);
write_cmos_sensor(0x0202,0x06);
write_cmos_sensor(0x0203,0x5E);
write_cmos_sensor(0x0224,0x00);
write_cmos_sensor(0x0225,0xCB);
write_cmos_sensor(0x0204,0x00);
write_cmos_sensor(0x0205,0x00);
write_cmos_sensor(0x020E,0x01);
write_cmos_sensor(0x020F,0x00);
write_cmos_sensor(0x0210,0x01);
write_cmos_sensor(0x0211,0x00);
write_cmos_sensor(0x0212,0x01);
write_cmos_sensor(0x0213,0x00);
write_cmos_sensor(0x0214,0x01);
write_cmos_sensor(0x0215,0x00);
write_cmos_sensor(0x0216,0x00);
write_cmos_sensor(0x0217,0x00);
write_cmos_sensor(0x4170,0x00);
write_cmos_sensor(0x4171,0x10);
write_cmos_sensor(0x4176,0x00);
write_cmos_sensor(0x4177,0x3C);
write_cmos_sensor(0xAE20,0x04);
write_cmos_sensor(0xAE21,0x5C);

write_cmos_sensor(0x0138,0x01);
write_cmos_sensor(0x0100,0x01);		

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
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x0016) << 8) | read_cmos_sensor(0x0017));
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
#if OTP		
		
				iReadData(0x00,452,OTPData);
#endif
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
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
	LOG_INF("PLATFORM:MT6595,MIPI 2LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x0016) << 8) | read_cmos_sensor(0x0017));
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
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
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
    if((imgsensor.ihdr_mode == 2) || (imgsensor.ihdr_mode == 9))
        preview_setting_HDR();
    else
	    preview_setting();
	//hs_video_setting();

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
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
    // 
    if((imgsensor.ihdr_mode == 2) || (imgsensor.ihdr_mode == 9))
        fullsize_setting_HDR(imgsensor.current_fps);
    else
	    capture_setting(imgsensor.current_fps); 
	
	
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
    if(imgsensor.ihdr_mode == 2)
	    fullsize_setting_HDR(imgsensor.current_fps);
	else
        normal_video_setting(imgsensor.current_fps);
	
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
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	
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
	//sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
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
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0601,0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0601,0x0000);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 imx214_awb_gain(SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
    LOG_INF("imx214_awb_gain\n");
    UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

    grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
    rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
    bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
    gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

    LOG_INF("[imx214_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GR, grgain_32);
    LOG_INF("[imx214_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n", pSetSensorAWB->ABS_GAIN_R, rgain_32);
    LOG_INF("[imx214_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n", pSetSensorAWB->ABS_GAIN_B, bgain_32);
    LOG_INF("[imx214_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

    write_cmos_sensor(0x0b8e, (grgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b8f, grgain_32 & 0xFF);
    write_cmos_sensor(0x0b90, (rgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b91, rgain_32 & 0xFF);
    write_cmos_sensor(0x0b92, (bgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b93, bgain_32 & 0xFF);
    write_cmos_sensor(0x0b94, (gbgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b95, gbgain_32 & 0xFF);
    return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
    SENSOR_VC_INFO_STRUCT *pvcinfo;
    SET_SENSOR_AWB_GAIN *pSetSensorAWB=(SET_SENSOR_AWB_GAIN *)feature_para;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
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
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
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
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data;
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
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
        case SENSOR_FEATURE_GET_VC_INFO:
            LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", *feature_data);
            pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            }
            break;
        case SENSOR_FEATURE_SET_AWB_GAIN:
            imx214_awb_gain(pSetSensorAWB);
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

//kin0603
UINT32 IMX214_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
//UINT32 IMX214_MIPI_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/
