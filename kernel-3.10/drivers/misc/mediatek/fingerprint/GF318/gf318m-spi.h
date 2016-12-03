#ifndef __GF66XX_SPI_H
#define __GF66XX_SPI_H

#include <linux/types.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>

/********************GF66XX Mapping**********************/
#define GF66XX_BASE             (0x8000)
#define GF66XX_OFFSET(x)        (GF66XX_BASE + x)

#define GF66XX_VERSION		    GF66XX_OFFSET(0)
#define GF66XX_CONFIG_DATA 	    GF66XX_OFFSET(0x40)
#define GF66XX_CFG_ADDR  	    GF66XX_OFFSET(0x47)
#define GF66XX_MODE_STATUS      GF66XX_OFFSET(0x043)
//#define GF66XX_MIXER_DATA	    GF66XX_OFFSET(0x140)
#define GF66XX_BUFFER_STATUS	GF66XX_OFFSET(0x140)
#define GF66XX_KEY_DATA         GF66XX_OFFSET(0x142)
#define GF66XX_NOISE_DATA       GF66XX_OFFSET(0x144)
#define GF66XX_LONG_PRESS_STDP  GF66XX_OFFSET(0x146)
#define GF66XX_BUFFER_DATA	    GF66XX_OFFSET(0x141)


#define GF66XX_BUF_STA_MASK	    (0x1<<7)
#define	GF66XX_BUF_STA_READY	(0x1<<7)
#define	GF66XX_BUF_STA_BUSY	    (0x0<<7)

#define	GF66XX_IMAGE_MASK	    (0x1<<6)
#define	GF66XX_IMAGE_ENABLE	    (0x1)
#define	GF66XX_IMAGE_DISABLE	(0x0)

#define	GF66XX_KEY_MASK		    (GF66XX_HOME_KEY_MASK | \
                                 GF66XX_MENU_KEY_MASK | \
                                 GF66XX_BACK_KEY_MASK )
//#define	GF66XX_KEY_ENABLE	    (0x1)
//#define	GF66XX_KEY_DISABLE	    (0x0)

//#define	GF66XX_KEY_STA		    (0x1<<4)

//home key
#define	GF66XX_HOME_KEY_MASK	(0x1<<5)
#define	GF66XX_HOME_KEY_ENABL   (0x1)
#define	GF66XX_HOME_KEY_DISABLE (0x0)

#define	GF66XX_HOME_KEY_STA	    (0x1<<4)
//menu key
#define	GF66XX_MENU_KEY_MASK    (0x1<<3)
#define	GF66XX_MENU_KEY_ENABLE	(0x1)
#define	GF66XX_MENU_KEY_DISABLE	(0x0)

#define	GF66XX_MENU_KEY_STA		(0x1<<2)
//back key
#define	GF66XX_BACK_KEY_MASK    (0x1<<1)
#define	GF66XX_BACK_KEY_ENABLE  (0x1)
#define	GF66XX_BACK_KEY_DISABLE (0x0)

#define	GF66XX_BACK_KEY_STA	    (0x1<<0)


#define	GF66XX_IMAGE_MODE	    (0x00)
#define	GF66XX_KEY_MODE		    (0x01)
#define GF66XX_SLEEP_MODE       (0x02)
#define GF66XX_FF_MODE	    	(0x03)
#define GF66XX_DEBUG_MODE	    (0x56)

/**********************GF66XX ops****************************/
#define GF66XX_W                0xF0
#define GF66XX_R                0xF1
#define GF66XX_WDATA_OFFSET	    (0x3)	// means to the offset of the buffer,which read from register 
#define GF66XX_RDATA_OFFSET	    (0x5)	// means to the offset of the buffer,to be writen to register,which copy from user_space
#define GF66XX_CFG_LEN			(249)	/*config data length*/
/**********************************************************/

/**********************IO Magic**********************/
#define  GF66XX_IOC_MAGIC    'g'  //define magic number
struct gf66xx_ioc_transfer {
	u8 cmd;
       u8 reserved;
	u16 addr;
	u32 len;
	u32 buf;
};
//define commands
/*read/write GF66XX registers*/
#define  GF66XX_IOC_CMD	_IOWR(GF66XX_IOC_MAGIC, 1, struct gf66xx_ioc_transfer)
#define  GF66XX_IOC_REINIT	_IO(GF66XX_IOC_MAGIC, 0)
#define  GF66XX_IOC_SETSPEED _IOW(GF66XX_IOC_MAGIC, 2, u32)
#define  GF66XX_IOC_STOPTIMER   _IO(GF66XX_IOC_MAGIC, 3)
#define  GF66XX_IOC_STARTTIMER  _IO(GF66XX_IOC_MAGIC, 4)

#define  GF66XX_IOC_MAXNR 5

/*******************Refering to platform*****************************/
#define ONTIM_MTK6735

#ifdef ONTIM_MTK6735
#define 		GF66XX_POWER_SOURCE_CUSTOM    PMIC_APP_CAP_TOUCH_VDD   //used as same to VGP1_PMU for TP
#else
#define          GF66XX_PWR_PIN		(GPIO45 | 0x80000000)
#define 		GF66XX_PWR_PIN_M_GPIO   GPIO_MODE_00
#endif

#define		GF66XX_IRQ_NUM		3//4  
       
#define 		GF66XX_INT_PIN 		GPIO_FPC_EINT_PIN /*GPIO3*/   //(GPIO4 | 0x80000000)
//#define 		GF66XX_INT_PIN 		(GPIO3 | 0x80000000)
#define 		GF66XX_RST_PIN		GPIO_FPC_RST_PIN  /*GPIO19*/   //(GPIO44 | 0x80000000)
//#define 		GF66XX_RST_PIN		(GPIO192 | 0x80000000)
#define 		GF66XX_SPI_RESET_PIN_M_GPIO   GPIO_MODE_00
#define 		GF66XX_SPI_RESET_PIN_M_DAIPCMOUT   GPIO_MODE_01

#define 		GF66XX_SPI_EINT_PIN_M_GPIO   GPIO_MODE_00
#define 		GF66XX_SPI_EINT_PIN_M_EINT   GPIO_MODE_00

#define		GF66XX_SPI_SCK_PIN		(GPIO166 | 0x80000000)
#define		GF66XX_SPI_SCK_PIN_M_GPIO	GPIO_MODE_00
#define		GF66XX_SPI_SCK_PIN_M_SCK	GPIO_MODE_01

#define		GF66XX_SPI_CS_PIN		(GPIO169 | 0x80000000)
#define		GF66XX_SPI_CS_PIN_M_GPIO	GPIO_MODE_00
#define		GF66XX_SPI_CS_PIN_M_CS		GPIO_MODE_01
	
#define		GF66XX_SPI_MOSI_PIN		(GPIO168 | 0x80000000)
#define		GF66XX_SPI_MOSI_PIN_M_GPIO	GPIO_MODE_00
#define		GF66XX_SPI_MOSI_PIN_M_MOSI	GPIO_MODE_01

#define		GF66XX_SPI_MISO_PIN		(GPIO167 | 0x80000000)
#define		GF66XX_SPI_MISO_PIN_M_GPIO	GPIO_MODE_00
#define		GF66XX_SPI_MISO_PIN_M_MISO	GPIO_MODE_01


/*************************************************************/
#define GF66XX_FASYNC 		1//If support fasync mechanism.
//#undef GF66XX_FASYNC
//#define CHARGER_DETECT      1
struct gf66xx_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev 	*input;
    struct work_struct  spi_work;
    struct timer_list   gf66xx_timer;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;
    unsigned long irq_gpio;
    unsigned long reset_gpio;
    unsigned long power_gpio;
    unsigned char config[GF66XX_CFG_LEN+GF66XX_WDATA_OFFSET];
#ifdef CHARGER_DETECT
    struct usb_phy      *phy;
    struct notifier_block  nb;
#endif //CHARGER_DETECT
#ifdef GF66XX_FASYNC
    struct  fasync_struct *async;
#endif
};

/*GPIO pin reference*/
int gf66xx_parse_dts(struct gf66xx_dev *gf66xx_dev);
void gf66xx_spi_pins_config(void);

/*Power Management*/
int gf66xx_power_on(struct gf66xx_dev *gf66xx_dev);
int gf66xx_power_off(struct gf66xx_dev *gf66xx_dev);

int gf66xx_hw_reset(struct gf66xx_dev *gf66xx_dev, unsigned int delay_ms);
/*IRQ reference.*/
int gf66xx_irq_setup(struct gf66xx_dev *gf66xx_dev, void (*irq_handler)(void));
int gf66xx_irq_release(struct gf66xx_dev *gf66xx_dev);
int gf66xx_irq_num(struct gf66xx_dev *gf66xx_dev);
int gf66xx_irq_open(int irq_no);
int gf66xx_irq_close(int irq_no);

int gf66xx_spi_write_bytes(struct gf66xx_dev *gf66xx_dev,u16 addr, u32 data_len, u8 *tx_buf);
int gf66xx_spi_read_bytes(struct gf66xx_dev *gf66xx_dev,u16 addr, u32 data_len, u8 *rx_buf);
int gf66xx_fw_update(struct gf66xx_dev* gf66xx_dev, unsigned char *buf, unsigned short len);
#endif //__GF66XX_SPI_H
