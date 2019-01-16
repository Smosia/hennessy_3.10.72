
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <linux/delay.h>
	#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>


#endif
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(0,fmt)
#else
#define LCD_DEBUG(fmt)  pr_debug(fmt)
#endif
//static unsigned char lcd_id_pins_value = 0xFF;
static const unsigned char LCD_MODULE_ID = 0x01; //  haobing modified 2013.07.11
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									1
#define FRAME_WIDTH  										(1440)
#define FRAME_HEIGHT 										(2560)
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN

#define REGFLAG_PORT_SWAP									0xFFFA
#define REGFLAG_DELAY             							0xFFFC	
#define REGFLAG_END_OF_TABLE      							0xFFFD   // END OF REGISTERS MARKER

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static const unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmd_by_cmdq_dual(handle,cmd,count,ppara,force_update)    lcm_util.dsi_set_cmdq_V23(handle,cmd,count,ppara,force_update);
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
#define dsi_swap_port(swap)   								lcm_util.dsi_swap_port(swap)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
* Define
*****************************************************************************/

#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

/***************************************************************************** 
* GLobal Variable
*****************************************************************************/
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *tps65132_i2c_client = NULL;


/***************************************************************************** 
* Function Prototype
*****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/***************************************************************************** 
* Data Structure
*****************************************************************************/

struct tps65132_dev	{	
struct i2c_client	*client;

};

static const struct i2c_device_id tps65132_id[] = {
{ I2C_ID_NAME, 0 },
{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver tps65132_iic_driver = {
    .id_table	= tps65132_id,
    .probe		= tps65132_probe,
    .remove		= tps65132_remove,
    .driver		= {
    .owner	= THIS_MODULE,
    .name	= "tps65132",
},
};
/***************************************************************************** 
* Extern Area
*****************************************************************************/ 

/***************************************************************************** 
* Function
*****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
//    LCD_DEBUG( "tps65132_iic_probe\n");
//    LCD_DEBUG("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
    tps65132_i2c_client  = client;		
    return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
//    LCD_DEBUG( "tps65132_remove\n");
    tps65132_i2c_client = NULL;
    i2c_unregister_device(client);
    return 0;
}


int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
    int ret = 0;
    struct i2c_client *client = tps65132_i2c_client;

	if(client == NULL)
	{
//		LCD_DEBUG("ERROR!!tps65132_i2c_client is null\n");
		return 0;
	}
	
    char write_data[2]={0};	
    write_data[0]= addr;
    write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
    if(ret<0)
//        LCD_DEBUG("tps65132 write data fail !!\n");	
    return ret ;
}
EXPORT_SYMBOL_GPL(tps65132_write_bytes);


/*
* module load/unload record keeping
*/

static int __init tps65132_iic_init(void)
{
//    LCD_DEBUG( "tps65132_iic_init\n");
    i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
//    LCD_DEBUG( "tps65132_iic_init2\n");
    i2c_add_driver(&tps65132_iic_driver);
//    LCD_DEBUG( "tps65132_iic_init success\n");	
    return 0;
}

static void __exit tps65132_iic_exit(void)
{
//    LCD_DEBUG( "tps65132_iic_exit\n");
    i2c_del_driver(&tps65132_iic_driver);  
}

module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 

#endif

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = 
{
    {0x2A, 4, {0x00, 0x00, 0x05, 0x9F}},
    {0x2B, 4, {0x00, 0x00, 0x09, 0xFF}},
    {0x11, 0, {}},
    {REGFLAG_DELAY, 80, {}},
    {0x44, 2, {0x00, 0x00}},
    {0x35, 1, {0x00}},
    {0x36, 1, {0x00}},
    {0x3A, 1, {0x77}},
    {0x29, 0, {}},
    {REGFLAG_DELAY, 50, {}},
    {0xB0,1,{0x04}},
    {0xCE,25,{0x3D,0x40,0x46,0x4E,0x57,0x60,0x6A,0x75,0x81,0x8E,0x9B,0xAA,0xB9,0xC8,0xD8,0xE6,0xFF,0x26,0x00,0x04,0x04,0x00,0x00,0x69,0x5A}},
// for CABC on
    {0x51, 1, {0xFF}},
    {0x55, 1, {0x00}},
    {0x53, 1, {0x2C}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 0
static struct LCM_setting_table lcm_set_window[] = {
    {0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
    {0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_normal_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {0xB0, 1, {0x00}},
    {0xB1, 1, {0x01}},
    {REGFLAG_DELAY, 20, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 24, {}},
    {0x34, 1, {0x00}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 80, {}},

// for CABC off
    {0x53, 1, {0x00}},
    {0x55, 1, {0x00}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
            case REGFLAG_DELAY :
#ifdef BUILD_LK
    			dprintf(0, "[LK]REGFLAG_DELAY\n");
#endif
           		if(table[i].count <= 10)
           			MDELAY(table[i].count);
            	else
            		MDELAY(table[i].count);
           		break;

            case REGFLAG_END_OF_TABLE :
            	break;
			case REGFLAG_PORT_SWAP:	
#ifdef BUILD_LK
    			dprintf(0, "[LK]push_table end\n");
#endif
				dsi_swap_port(1);
                break;
            default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->lcm_if = LCM_INTERFACE_DSI_DUAL;
    params->lcm_cmd_if = LCM_INTERFACE_DSI0;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
    params->dsi.dual_dsi_type = DUAL_DSI_CMD;
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;
	params->dsi.ssc_disable=1;
    params->dsi.ssc_range = 4;
    //video mode timing

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 1;
    params->dsi.vertical_backporch					= 7;
    params->dsi.vertical_frontporch					= 7;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 40;
    params->dsi.horizontal_backporch				= 80;
    params->dsi.horizontal_frontporch				= 100;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.PLL_CLOCK = 450; //this value must be in MTK suggested table
    params->dsi.ufoe_enable  = 1;
    params->dsi.ufoe_params.lr_mode_en = 1;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable      = 0;

	params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x2C;

	params->dsi.clk_lp_per_line_enable = 0;



	// for R63419A Lane Swap
	params->dsi.lane_swap_en = 0;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_0] 	= MIPITX_PHY_LANE_2;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_1] 	= MIPITX_PHY_LANE_CK;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_2] 	= MIPITX_PHY_LANE_0;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_3] 	= MIPITX_PHY_LANE_1;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_CK] 	= MIPITX_PHY_LANE_3;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_RX] 	= MIPITX_PHY_LANE_2;
	
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_0] 	= MIPITX_PHY_LANE_0;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_1] 	= MIPITX_PHY_LANE_3;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_2] 	= MIPITX_PHY_LANE_2;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_3] 	= MIPITX_PHY_LANE_1;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_CK] 	= MIPITX_PHY_LANE_CK;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_RX] 	= MIPITX_PHY_LANE_0;
}

#ifdef BUILD_LK

#define TPS65132_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t TPS65132_i2c;

static int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    TPS65132_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
    TPS65132_i2c.mode = ST_MODE;
    TPS65132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&TPS65132_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#else

//	extern int mt8193_i2c_write(u16 addr, u32 data);
//	extern int mt8193_i2c_read(u16 addr, u32 *data);

//	#define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data)
//#define TPS65132_read_byte(add)  mt8193_i2c_read(add)


#endif

static void lcm_init_power(void)
{
#if 1
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x0a;
	/* mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00); */
	/* mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT); */
	/* mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE); */

//	LCD_DEBUG("lcm_init_power\n") ;
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);


#ifdef BUILD_LK
	mt6331_upmu_set_rg_vgp1_en(1);
	mt6331_upmu_set_rg_vcam_io_en(1);
	ret=TPS65132_write_byte(cmd,data);
	if(ret) 	
	dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write error----\n",cmd);		
	else
	dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write success----\n",cmd);			
#else
	pr_debug("%s, begin\n", __func__);
	hwPowerOn(MT6331_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	
	pr_debug("%s, end\n", __func__);
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	MDELAY(500);
	cmd=0x01;
	data=0x0a;
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
	if(ret) 	
		dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write error----\n",cmd);		
	else
		dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
#endif

}

static void lcm_resume_power(void)
{
#if 1
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x0a;
	/* mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00); */
	/* mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT); */
	/* mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE); */

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);

//	LCD_DEBUG("lcm_resume_power\n") ;

#ifdef BUILD_LK
	mt6331_upmu_set_rg_vgp1_en(1);
	mt6331_upmu_set_rg_vcam_io_en(1);
	ret=TPS65132_write_byte(cmd,data);
	if(ret) 	
	dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write error----\n",cmd);		
	else
	dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write success----\n",cmd);			
#else
	pr_debug("%s, begin\n", __func__);
	hwPowerOn(MT6331_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	
	pr_debug("%s, end\n", __func__);
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	MDELAY(500);
	cmd=0x01;
	data=0x0a;
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
	if(ret) 	
		dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write error----\n",cmd);		
	else
		dprintf(0, "[LK]r63423----tps6132----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	pr_debug("[KERNEL]r63423----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
#endif

}

static void lcm_suspend_power(void)
{
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
#ifdef BUILD_LK
#else
	pr_debug("%s, begin\n", __func__);
	hwPowerDown(MT6331_POWER_LDO_VGP1, "LCM_DRV");	
	pr_debug("%s, end\n", __func__);
#endif
}


static void lcm_init(void)
{

    SET_RESET_PIN(1);
    MDELAY(1);
    //MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);

    // when phone initial , config output high, enable backlight drv chip  
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
	
#ifdef BUILD_LK
    dprintf(0, "[LK]push_table end\n");
#endif
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);  
#if 0
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ZERO);
#endif
	SET_RESET_PIN(1);
	MDELAY(10);

	SET_RESET_PIN(0);
	MDELAY(10);
}
static void lcm_resume(void)
{
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	lcm_init(); 
}
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    /*BEGIN PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/
    //delete high speed packet
    //data_array[0]=0x00290508;
    //dsi_set_cmdq(data_array, 1, 1);
    /*END PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);	
}

#define LCM_ID_R63419 (0x3419)
#define LCM_ID_R63423 (0x3423)

static unsigned int lcm_compare_id(void)
{

	unsigned char buffer[5];
	unsigned int array[16];  
	unsigned int lcd_id = 0;
	SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);
	array[0] = 0x00053700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
  
	read_reg_v2(0xBF, buffer, 5);
	MDELAY(20);
	lcd_id = (buffer[2] << 8 )| buffer[3];

#ifdef BUILD_LK
    dprintf(0, "%s, LK r63423 debug: r63423 id = 0x%08x\n", __func__, lcd_id);
#else
    pr_debug("%s, kernel r63423 horse debug: r63423 id = 0x%08x\n", __func__, lcd_id);
#endif

    if(lcd_id == LCM_ID_R63423)
        return 1;
    else
        return 0;

}

static void lcm_setbacklight_cmdq(void* handle,unsigned int level)
{
#ifdef BUILD_LK
	dprintf(0,"%s,lk R63423 backlight: level = %d\n", __func__, level);
#else
	pr_debug("%s, kernel R63423 backlight: level = %d\n", __func__, level);
#endif
	// Refresh value of backlight level.
	
	unsigned int cmd = 0x51;
	unsigned int count =1;
	unsigned int value = level;
	dsi_set_cmd_by_cmdq_dual(handle, cmd, count, &value, 1);
	//lcm_backlight_level_setting[0].para_list[0] = level;
	//push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}
               

LCM_DRIVER r63423_wqhd_truly_phantom_lcm_drv=
{
    .name           = "r63423_wqhd_truly_phantom",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,  
    .init_power		= lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
    .set_backlight_cmdq  = lcm_setbacklight_cmdq,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif

};
/* END PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
