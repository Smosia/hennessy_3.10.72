#ifndef __GF_SPI_TEE_H
#define __GF_SPI_TEE_H

#include <linux/types.h>

/**********************IO Magic**********************/
#define GF_IOC_MAGIC	'g'  //define magic number

struct gf_ioc_transfer {
	u8 cmd;
	u8 flag;
	u16 addr;
	u32 len;
	u8 *buf;
};

//send command to secure driver through DCI
struct gf_secdrv_cmd{
	u8 cmd;
	u8 reserve[3];
	u32 len;
	u8 buf[32];
};

struct spi_speed_setting{
	u32 high_speed;
	u32 low_speed;
};


//define commands
#define GF_IOC_CMD	_IOWR(GF_IOC_MAGIC, 1, struct gf_ioc_transfer)
#define GF_IOC_REINIT	_IO(GF_IOC_MAGIC, 0)
#define GF_IOC_SETSPEED	_IOW(GF_IOC_MAGIC, 2, u32)
#define GF_IOC_STOPTIMER	_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_STARTTIMER	_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_SETMODE	_IOW(GF_IOC_MAGIC, 6,u32)
#define GF_IOC_GETMODE	_IOR(GF_IOC_MAGIC, 7,u32)
#define GF_IOC_GETCHIPID	_IOR(GF_IOC_MAGIC, 8, unsigned int *)

/*for TEE version*/
#define GF_IOC_SECURE_INIT	  _IO(GF_IOC_MAGIC, 10)
#define GF_IOC_INIT_SPI_SPEED_VARIABLE _IOW(GF_IOC_MAGIC, 12, struct spi_speed_setting)
#define GF_IOC_HAS_ESD_WD	_IOW(GF_IOC_MAGIC, 13,u32)
#define GF_IOC_INIT_FP_EINT	_IOW(GF_IOC_MAGIC, 14,u32)

#define GF_IOC_CONNECT_SECURE_DRIVER	_IO(GF_IOC_MAGIC, 16)
#define GF_IOC_CLOSE_SECURE_DRIVER		_IO(GF_IOC_MAGIC, 17)
#define GF_IOC_DCI_TESTCASE	_IOW(GF_IOC_MAGIC, 18, struct gf_secdrv_cmd)
#define GF_IOC_SET_IRQ		_IOW(GF_IOC_MAGIC, 19, u32)
#define GF_IOC_GET_EVENT_TYPE		_IOR(GF_IOC_MAGIC, 20, u32)

#define GF_IOC_CLEAR_IRQ_STATUS		_IOW(GF_IOC_MAGIC, 22, u32)
#define GF_IOC_SPI_SETSPEED			_IOW(GF_IOC_MAGIC, 23, u32)
#define GF_IOC_RESET_SAMPLE_STATUS			_IOW(GF_IOC_MAGIC, 24, u32)



#define GF_IOC_NMW_SET_MODE             _IOW(GF_IOC_MAGIC, 300, u32)
#define GF_IOC_NMW_GET_IMAGE            _IOW(GF_IOC_MAGIC, 301, u32)
#define GF_IOC_NMW_FLASH_CONFIG         _IOW(GF_IOC_MAGIC, 302, u32)
        
//#define HW_BUFF_ROW 		(144)//(68 * 1.5 + 10)
//#define HW_BUFF_COL 		(96)
#define HW_BUFF_ROW 		(118)//(68 * 1.5 + 10)
#define HW_BUFF_COL 		(112)

#define HW_BUFF_RESERVED    (1)
#define HW_BUFF_DATA_OFFSET (7)
#define HW_BUFF_STATUS_LEN  (9)
#define FRAME_BUFF_LEN  	(HW_BUFF_ROW*HW_BUFF_COL)
#define VER_NUM_OFFSET      (6)


#define HW_BUFF_DATA_LEN    (HW_BUFF_ROW*HW_BUFF_COL)
#define HW_BUFF_LEN         (HW_BUFF_DATA_LEN + HW_BUFF_STATUS_LEN)


/* not used */
#define GF_IOC_SEND_SENSOR_CMD				 _IOW(GF_IOC_MAGIC, 25, u32)
#define SENSOR_CMD_SAMPLE_SUSPEND		(0x02)
#define SENSOR_CMD_SAMPLE_RESUME	   (0x03)


//#define  GF_IOC_MAXNR    26  //THIS MACRO IS NOT USED NOW...


#endif	//__GF_SPI_TEE_H
