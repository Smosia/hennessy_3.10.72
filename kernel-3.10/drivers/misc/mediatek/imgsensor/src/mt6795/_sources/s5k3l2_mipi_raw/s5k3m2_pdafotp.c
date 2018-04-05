//liuying@wind-mobi.com 20150208 add begin
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/xlog.h>


#define PFX "S5K3M2_pdafotp"
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define S5K3M2_EEPROM_READ_ID  0xA0   			//liuying 20150210
#define S5K3M2_EEPROM_WRITE_ID   0xA1			//liuying 20150210
#define S5K3M2_I2C_SPEED        100  
#define S5K3M2_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
BYTE s5k3m2_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;
//liuying@wind-mobi.com 20150309 add begin
static int s5k3m2_otp_read_lsc = 0;
extern u8 s5k3m2_eeprom_lsc_data[];//liuying 20150310
//u8 s5k3m2_eeprom_lsc_data[2048]= {0};//liuying 20150310
/*
typedef struct {
#if 0
	u16	   ChipInfo; //chip id, lot Id, Chip No. Etc
	u8     IdGroupWrittenFlag; //"Bit[7:6]: Flag of WB_Group_0  00: empty  01: valid group 11 or 10: invalid group"
	u8     ModuleInfo; //MID, 0x02 for truly
	u8     Year;
	u8     Month;
	u8     Day;
	u8     LensInfo;
	u8     VcmInfo;
	u8     DriverIcInfo;
	u8     LightTemp;
#endif
       u8     flag;
	u32   CaliVer;//0xff000b01
	u16   SerialNum;
	u8     Version;//0x01
	u8     AwbAfInfo;//0xF
	u8     UnitAwbR;
	u8     UnitAwbGr;
	u8     UnitAwbGb;
	u8     UnitAwbB;
	u8     GoldenAwbR;
	u8     GoldenAwbGr;
	u8     GoldenAwbGb;
	u8     GoldenAwbB;
	u16   AfInfinite;
	u16   AfMacro;
	u16   LscSize;//0x03A4
	//u8     Lsc[MAX_LSC_SIZE];
}OTP_MTK_TYPE;

typedef union {
        u8 Data[DATA_SIZE];
        OTP_MTK_TYPE       MtkOtpData;
} OTP_DATA;
*/

//extern OTP_DATA s5k3m2_otp_data; 
//liuying@wind-mobi.com 20150309 add end


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3M2_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(S5K3M2_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3M2_EEPROM_READ_ID)<0){
		printk("---lanhai %s----\n", __func__);
		return false;
	}
    return true;
}

static bool _read_3m2_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_3m2_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size){
	int i;
	addr = 0x0782;
	size = 1404;
	
	LOG_INF("read 3m2 eeprom, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_3m2_eeprom(addr, s5k3m2_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

	//if((s5k3m2_eeprom_data[0] != 1) || (s5k3m2_eeprom_data[498] != 1) || (s5k3m2_eeprom_data[1306] != 1))
	//{
		//LOG_INF("3M2 PDAF eeprom data invalid\n");
		//return false;
	//}
	#if 0
	for(i = 0; i < 496; i++)   // proc1 data
	{
		data[i] = s5k3m2_eeprom_data[i+1];
	}

	for(i = 496; i < 1302; i++)  // proc2 data
	{
		data[i] = s5k3m2_eeprom_data[i+3];
	}

	for(i = 1302; i < 1404; i++)  // proc3 data
	{
		data[i] = s5k3m2_eeprom_data[i+5];
	}	

	for(i = 0; i < 1404; i++)
	{
		LOG_INF("data[%d] = %d\n", i, data[i]);
	}
	#endif
	
	memcpy(data, s5k3m2_eeprom_data, size);

	
    return true;
}

//liuying@wind-mobi.com 20150309 add begin
int read_3m2_otp_size(kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	int i = 0;
	for(i = 0; i < size; i++){
		if(!_read_3m2_eeprom(addr, s5k3m2_eeprom_lsc_data, size))
			return -1;
	}
    return 0;
}

int  read_3m2_eeprom_for_mtk(void ){
	int i;
	kal_uint16 addr = 0x0000;
	kal_uint32 size =  5;//5;//liuying 20150310 total 1894 byte
	
	LOG_INF("check s5k3m2 OTP readed =%d \n",s5k3m2_otp_read_lsc);

	//if(1 == s5k3m2_otp_read_lsc ) {
		//LOG_INF("OTP have readed ! skip\n");
		//return 0;
	//}
	//spin_lock(&g_CAM_CALLock);
	s5k3m2_otp_read_lsc = 1;
	//spin_unlock(&g_CAM_CALLock);
	
      #if 0
	//read calibration version 0xff000b01
	read_3m2_otp_size(0x0001,&s5k3m2_otp_data.Data[0x01],4);

	//read module id
	read_3m2_otp_size(0x0005,&s5k3m2_otp_data.Data[0x05],1);

	//read AF config
	//read_3m2_otp_size(0x3B13,&s5k3m2_otp_data.Data[0x07],2);

	//read AWB
	//read_3m2_otp_size(0x01,0x3B15,&s5k3m2_otp_data.Data[0x09],8);

	//read AF calibration
	read_3m2_otp_size(0x0014,&s5k3m2_otp_data.Data[0x011],4);

       //read LSC data 1868 size
	read_3m2_otp_size(0x001A,&s5k3m2_otp_data.Data[0x015],1868);
      #else
	for(i = 0; i < size; i++){
		if(!_read_3m2_eeprom(addr, s5k3m2_eeprom_lsc_data, size))
			return -1;
	}
      #endif
	   
    return 0;
}
//liuying@wind-mobi.com 20150309 add end

kal_uint16 read_3m2_eeprom_reg(kal_uint32 addr)
{
    //kdSetI2CSpeed(S5K3M2_I2C_SPEED);
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd, 2, (u8*)&get_byte, 1, S5K3M2_EEPROM_READ_ID);
    return get_byte;
}

//liuying@wind-mobi.com 20150208 add end