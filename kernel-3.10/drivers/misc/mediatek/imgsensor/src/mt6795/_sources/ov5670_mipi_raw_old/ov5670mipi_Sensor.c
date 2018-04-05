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
#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov5670mipi_Sensor.h"

#define PFX "ov5670_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

   
static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = OV5670MIPI_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
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
	.normal_video = {
		.pclk = 102850000,
		.linelength = 1676,
		.framelength = 2045,
		.startx = 1,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 960,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 300,
	},
	.margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0x7fff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	//.mipi_sensor_type = MIPI_OPHY_NCSI2; //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	//.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO; //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
	.i2c_addr_table = {0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4C00,					//current shutter
	.gain = 0x0200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.i2c_write_id = 0x6c,//record current sensor's i2c write id
};


/* Sensor output window information */

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
	kal_int16 dummy_line;
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
	kal_uint32 frame_length = 0;
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
	shutter = (shutter >> 1) << 1;
	
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
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));		
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
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
	write_cmos_sensor(0x3002, 0x61); 
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

	write_cmos_sensor(0x0100, 0x00); //;01


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
	write_cmos_sensor(0x3623, 0x00);	
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
	LOG_INF("Johann: ready to get image sensor id\n");
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, write id: 0x%x, actual value: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
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
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
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
	imgsensor.current_fps = 300;
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.current_fps = imgsensor_info.cap.max_framerate;
		imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);

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
	normal_video_setting(imgsensor.current_fps);
	
	
	return ERROR_NONE;
}	/*	normal_video   */




static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
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

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	
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
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 
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
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
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
	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
	{
		imgsensor.current_fps = 296;
		}
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
	{
		imgsensor.current_fps = 146;
		}
	else
	{
		imgsensor.current_fps = framerate*10;
	}
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("current framerate = %d\n ", imgsensor.current_fps);
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
			set_dummy();			
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
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
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
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			*framerate = imgsensor_info.cap.max_framerate;
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
			set_shutter(*feature_data_16);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode((BOOL) *feature_data_16);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data_16);
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
			set_video_mode(*feature_data_16);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, *(feature_data_32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, (MUINT32 *)(*(feature_data_32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data_16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
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

UINT32 OV5670_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5670_MIPI_RAW_SensorInit	*/