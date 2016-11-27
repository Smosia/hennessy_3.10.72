/*
 * Reversed by LazyC0DEr
 *
 *
 */
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
#include <linux/spinlock.h>

#endif
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include "tps65132_i2c.h"
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

static int lcm_intialized;

static LCM_UTIL_FUNCS lcm_util;

static raw_spinlock_t boe_SpinLock;

static int boe_value;
static int boe_first_vlue;
static int boe_second_vlue;

static int global_brightnest_level = 0;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n) 											(lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

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

static const unsigned char LCD_MODULE_ID = 0x02;
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									   0
#define FRAME_WIDTH  										   (1080)
#define FRAME_HEIGHT 										   (1920)


#define REGFLAG_DELAY			 0xFC
#define REGFLAG_END_OF_TABLE		 0xFD   // END OF REGISTERS MARKER


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


struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};



static struct LCM_setting_table lcm_backlight_level_setting[] = {
    { 0xFF, 1, {0x00}},
    { REGFLAG_DELAY, 1, {}},
    { 0xFB, 1, {0x01}},
    { 0x51, 1, {0xFF}},
    { REGFLAG_END_OF_TABLE, 0, {}},
    
};


static struct LCM_setting_table lcm_backlight_disable[] = {
    {0x55, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0, {}}
};

static struct LCM_setting_table lcm_backlight_enable[] = {
    {0x55, 1, {0x01}},
    {REGFLAG_END_OF_TABLE, 0, {}}
};



static struct LCM_setting_table lcm_suspend_setting[] = {
    { 0xff, 0x01, {0x00}},
    { REGFLAG_DELAY, 1,{}},
    { 0x28, 0x00, {}},
    { REGFLAG_DELAY, 20,{}},
    { 0x10, 0x00, {}},
    { REGFLAG_DELAY, 120,{}},
    { REGFLAG_END_OF_TABLE, 0x00,{}},
    
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0XFF, 1, {0X01}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X0, 1, {0X01}},
{0X1, 1, {0X55}},
{0X2, 1, {0X59}},
{0X4, 1, {0X0C}},
{0X5, 1, {0X3A}},
{0X6, 1, {0X55}},
{0X7, 1, {0XD5}},
{0XD, 1, {0X9D}},
{0XE, 1, {0X9D}},
{0XF, 1, {0XE0}},
{0X10, 1, {0X03}},
{0X11, 1, {0X3C}},
{0X12, 1, {0X50}},
{0X15, 1, {0X60}},
{0X16, 1, {0X14}},
{0X17, 1, {0X14}},
{0X44, 1, {0X68}},
{0X45, 1, {0X88}},
{0X46, 1, {0X78}},
{0X68, 1, {0X13}},
{0X6D, 1, {0X33}},
{0X75, 1, {0X00}},
{0X76, 1, {0X00}},
{0X77, 1, {0X00}},
{0X78, 1, {0X22}},
{0X79, 1, {0X00}},
{0X7A, 1, {0X51}},
{0X7B, 1, {0X00}},
{0X7C, 1, {0X73}},
{0X7D, 1, {0X00}},
{0X7E, 1, {0X8D}},
{0X7F, 1, {0X00}},
{0X80, 1, {0XA4}},
{0X81, 1, {0X00}},
{0X82, 1, {0XB8}},
{0X83, 1, {0X00}},
{0X84, 1, {0XCA}},
{0X85, 1, {0X00}},
{0X86, 1, {0XD9}},
{0X87, 1, {0X01}},
{0X88, 1, {0X0D}},
{0X89, 1, {0X01}},
{0X8A, 1, {0X36}},
{0X8B, 1, {0X01}},
{0X8C, 1, {0X75}},
{0X8D, 1, {0X01}},
{0X8E, 1, {0XA7}},
{0X8F, 1, {0X01}},
{0X90, 1, {0XF2}},
{0X91, 1, {0X02}},
{0X92, 1, {0X2B}},
{0X93, 1, {0X02}},
{0X94, 1, {0X2B}},
{0X95, 1, {0X02}},
{0X96, 1, {0X61}},
{0X97, 1, {0X02}},
{0X98, 1, {0X9D}},
{0X99, 1, {0X02}},
{0X9A, 1, {0XC6}},
{0X9B, 1, {0X03}},
{0X9C, 1, {0X01}},
{0X9D, 1, {0X03}},
{0X9E, 1, {0X27}},
{0X9F, 1, {0X03}},
{0XA0, 1, {0X63}},
{0XA2, 1, {0X03}},
{0XA3, 1, {0X6B}},
{0XA4, 1, {0X03}},
{0XA5, 1, {0X72}},
{0XA6, 1, {0X03}},
{0XA7, 1, {0X7E}},
{0XA9, 1, {0X03}},
{0XAA, 1, {0X87}},
{0XAB, 1, {0X03}},
{0XAC, 1, {0X93}},
{0XAD, 1, {0X03}},
{0XAE, 1, {0X9B}},
{0XAF, 1, {0X03}},
{0XB0, 1, {0XA2}},
{0XB1, 1, {0X03}},
{0XB2, 1, {0XFF}},
{0XB3, 1, {0X00}},
{0XB4, 1, {0X00}},
{0XB5, 1, {0X00}},
{0XB6, 1, {0X22}},
{0XB7, 1, {0X00}},
{0XB8, 1, {0X51}},
{0XB9, 1, {0X00}},
{0XBA, 1, {0X73}},
{0XBB, 1, {0X00}},
{0XBC, 1, {0X8D}},
{0XBD, 1, {0X00}},
{0XBE, 1, {0XA4}},
{0XBF, 1, {0X00}},
{0XC0, 1, {0XB8}},
{0XC1, 1, {0X00}},
{0XC2, 1, {0XCA}},
{0XC3, 1, {0X00}},
{0XC4, 1, {0XD9}},
{0XC5, 1, {0X01}},
{0XC6, 1, {0X0D}},
{0XC7, 1, {0X01}},
{0XC8, 1, {0X36}},
{0XC9, 1, {0X01}},
{0XCA, 1, {0X75}},
{0XCB, 1, {0X01}},
{0XCC, 1, {0XA7}},
{0XCD, 1, {0X01}},
{0XCE, 1, {0XF2}},
{0XCF, 1, {0X02}},
{0XD0, 1, {0X2B}},
{0XD1, 1, {0X02}},
{0XD2, 1, {0X2B}},
{0XD3, 1, {0X02}},
{0XD4, 1, {0X61}},
{0XD5, 1, {0X02}},
{0XD6, 1, {0X9D}},
{0XD7, 1, {0X02}},
{0XD8, 1, {0XC6}},
{0XD9, 1, {0X03}},
{0XDA, 1, {0X01}},
{0XDB, 1, {0X03}},
{0XDC, 1, {0X27}},
{0XDD, 1, {0X03}},
{0XDE, 1, {0X63}},
{0XDF, 1, {0X03}},
{0XE0, 1, {0X6B}},
{0XE1, 1, {0X03}},
{0XE2, 1, {0X72}},
{0XE3, 1, {0X03}},
{0XE4, 1, {0X7E}},
{0XE5, 1, {0X03}},
{0XE6, 1, {0X87}},
{0XE7, 1, {0X03}},
{0XE8, 1, {0X93}},
{0XE9, 1, {0X03}},
{0XEA, 1, {0X9B}},
{0XEB, 1, {0X03}},
{0XEC, 1, {0XA2}},
{0XED, 1, {0X03}},
{0XEE, 1, {0XFF}},
{0XEF, 1, {0X00}},
{0XF0, 1, {0XD6}},
{0XF1, 1, {0X00}},
{0XF2, 1, {0XDD}},
{0XF3, 1, {0X00}},
{0XF4, 1, {0XEA}},
{0XF5, 1, {0X00}},
{0XF6, 1, {0XF7}},
{0XF7, 1, {0X01}},
{0XF8, 1, {0X03}},
{0XF9, 1, {0X01}},
{0XFA, 1, {0X0D}},
{0XFF, 1, {0X02}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X0, 1, {0X01}},
{0X1, 1, {0X17}},
{0X2, 1, {0X01}},
{0X3, 1, {0X21}},
{0X4, 1, {0X01}},
{0X5, 1, {0X2A}},
{0X6, 1, {0X01}},
{0X7, 1, {0X4B}},
{0X8, 1, {0X01}},
{0X9, 1, {0X68}},
{0XA, 1, {0X01}},
{0XB, 1, {0X98}},
{0XC, 1, {0X01}},
{0XD, 1, {0XC0}},
{0XE, 1, {0X02}},
{0XF, 1, {0X00}},
{0X10, 1, {0X02}},
{0X11, 1, {0X34}},
{0X12, 1, {0X02}},
{0X13, 1, {0X35}},
{0X14, 1, {0X02}},
{0X15, 1, {0X68}},
{0X16, 1, {0X02}},
{0X17, 1, {0XA4}},
{0X18, 1, {0X02}},
{0X19, 1, {0XCC}},
{0X1A, 1, {0X03}},
{0X1B, 1, {0X07}},
{0X1C, 1, {0X03}},
{0X1D, 1, {0X2F}},
{0X1E, 1, {0X03}},
{0X1F, 1, {0X6A}},
{0X20, 1, {0X03}},
{0X21, 1, {0X71}},
{0X22, 1, {0X03}},
{0X23, 1, {0X85}},
{0X24, 1, {0X03}},
{0X25, 1, {0X9A}},
{0X26, 1, {0X03}},
{0X27, 1, {0XB2}},
{0X28, 1, {0X03}},
{0X29, 1, {0XCA}},
{0X2A, 1, {0X03}},
{0X2B, 1, {0XE2}},
{0X2D, 1, {0X03}},
{0X2F, 1, {0XF5}},
{0X30, 1, {0X03}},
{0X31, 1, {0XFF}},
{0X32, 1, {0X00}},
{0X33, 1, {0XD6}},
{0X34, 1, {0X00}},
{0X35, 1, {0XDD}},
{0X36, 1, {0X00}},
{0X37, 1, {0XEA}},
{0X38, 1, {0X00}},
{0X39, 1, {0XF7}},
{0X3A, 1, {0X01}},
{0X3B, 1, {0X03}},
{0X3D, 1, {0X01}},
{0X3F, 1, {0X0D}},
{0X40, 1, {0X01}},
{0X41, 1, {0X17}},
{0X42, 1, {0X01}},
{0X43, 1, {0X21}},
{0X44, 1, {0X01}},
{0X45, 1, {0X2A}},
{0X46, 1, {0X01}},
{0X47, 1, {0X4B}},
{0X48, 1, {0X01}},
{0X49, 1, {0X68}},
{0X4A, 1, {0X01}},
{0X4B, 1, {0X98}},
{0X4C, 1, {0X01}},
{0X4D, 1, {0XC0}},
{0X4E, 1, {0X02}},
{0X4F, 1, {0X00}},
{0X50, 1, {0X02}},
{0X51, 1, {0X34}},
{0X52, 1, {0X02}},
{0X53, 1, {0X35}},
{0X54, 1, {0X02}},
{0X55, 1, {0X68}},
{0X56, 1, {0X02}},
{0X58, 1, {0XA4}},
{0X59, 1, {0X02}},
{0X5A, 1, {0XCC}},
{0X5B, 1, {0X03}},
{0X5C, 1, {0X07}},
{0X5D, 1, {0X03}},
{0X5E, 1, {0X2F}},
{0X5F, 1, {0X03}},
{0X60, 1, {0X6A}},
{0X61, 1, {0X03}},
{0X62, 1, {0X71}},
{0X63, 1, {0X03}},
{0X64, 1, {0X85}},
{0X65, 1, {0X03}},
{0X66, 1, {0X9A}},
{0X67, 1, {0X03}},
{0X68, 1, {0XB2}},
{0X69, 1, {0X03}},
{0X6A, 1, {0XCA}},
{0X6B, 1, {0X03}},
{0X6C, 1, {0XE2}},
{0X6D, 1, {0X03}},
{0X6E, 1, {0XF5}},
{0X6F, 1, {0X03}},
{0X70, 1, {0XFF}},
{0X71, 1, {0X00}},
{0X72, 1, {0XCD}},
{0X73, 1, {0X00}},
{0X74, 1, {0XD5}},
{0X75, 1, {0X00}},
{0X76, 1, {0XE3}},
{0X77, 1, {0X00}},
{0X78, 1, {0XEF}},
{0X79, 1, {0X00}},
{0X7A, 1, {0XFB}},
{0X7B, 1, {0X01}},
{0X7C, 1, {0X06}},
{0X7D, 1, {0X01}},
{0X7E, 1, {0X11}},
{0X7F, 1, {0X01}},
{0X80, 1, {0X1B}},
{0X81, 1, {0X01}},
{0X82, 1, {0X24}},
{0X83, 1, {0X01}},
{0X84, 1, {0X46}},
{0X85, 1, {0X01}},
{0X86, 1, {0X63}},
{0X87, 1, {0X01}},
{0X88, 1, {0X94}},
{0X89, 1, {0X01}},
{0X8A, 1, {0XBB}},
{0X8B, 1, {0X01}},
{0X8C, 1, {0XFC}},
{0X8D, 1, {0X02}},
{0X8E, 1, {0X32}},
{0X8F, 1, {0X02}},
{0X90, 1, {0X32}},
{0X91, 1, {0X02}},
{0X92, 1, {0X66}},
{0X93, 1, {0X02}},
{0X94, 1, {0XA3}},
{0X95, 1, {0X02}},
{0X96, 1, {0XCC}},
{0X97, 1, {0X03}},
{0X98, 1, {0X0B}},
{0X99, 1, {0X03}},
{0X9A, 1, {0X38}},
{0X9B, 1, {0X03}},
{0X9C, 1, {0X8A}},
{0X9D, 1, {0X03}},
{0X9E, 1, {0X97}},
{0X9F, 1, {0X03}},
{0XA0, 1, {0X97}},
{0XA2, 1, {0X03}},
{0XA3, 1, {0X99}},
{0XA4, 1, {0X03}},
{0XA5, 1, {0X9B}},
{0XA6, 1, {0X03}},
{0XA7, 1, {0X9D}},
{0XA9, 1, {0X03}},
{0XAA, 1, {0X9F}},
{0XAB, 1, {0X03}},
{0XAC, 1, {0XA1}},
{0XAD, 1, {0X03}},
{0XAE, 1, {0XA2}},
{0XAF, 1, {0X00}},
{0XB0, 1, {0XCD}},
{0XB1, 1, {0X00}},
{0XB2, 1, {0XD5}},
{0XB3, 1, {0X00}},
{0XB4, 1, {0XE3}},
{0XB5, 1, {0X00}},
{0XB6, 1, {0XEF}},
{0XB7, 1, {0X00}},
{0XB8, 1, {0XFB}},
{0XB9, 1, {0X01}},
{0XBA, 1, {0X06}},
{0XBB, 1, {0X01}},
{0XBC, 1, {0X11}},
{0XBD, 1, {0X01}},
{0XBE, 1, {0X1B}},
{0XBF, 1, {0X01}},
{0XC0, 1, {0X24}},
{0XC1, 1, {0X01}},
{0XC2, 1, {0X46}},
{0XC3, 1, {0X01}},
{0XC4, 1, {0X63}},
{0XC5, 1, {0X01}},
{0XC6, 1, {0X94}},
{0XC7, 1, {0X01}},
{0XC8, 1, {0XBB}},
{0XC9, 1, {0X01}},
{0XCA, 1, {0XFC}},
{0XCB, 1, {0X02}},
{0XCC, 1, {0X32}},
{0XCD, 1, {0X02}},
{0XCE, 1, {0X32}},
{0XCF, 1, {0X02}},
{0XD0, 1, {0X66}},
{0XD1, 1, {0X02}},
{0XD2, 1, {0XA3}},
{0XD3, 1, {0X02}},
{0XD4, 1, {0XCC}},
{0XD5, 1, {0X03}},
{0XD6, 1, {0X0B}},
{0XD7, 1, {0X03}},
{0XD8, 1, {0X38}},
{0XD9, 1, {0X03}},
{0XDA, 1, {0X8A}},
{0XDB, 1, {0X03}},
{0XDC, 1, {0X97}},
{0XDD, 1, {0X03}},
{0XDE, 1, {0X97}},
{0XDF, 1, {0X03}},
{0XE0, 1, {0X99}},
{0XE1, 1, {0X03}},
{0XE2, 1, {0X9B}},
{0XE3, 1, {0X03}},
{0XE4, 1, {0X9D}},
{0XE5, 1, {0X03}},
{0XE6, 1, {0X9F}},
{0XE7, 1, {0X03}},
{0XE8, 1, {0XA1}},
{0XE9, 1, {0X03}},
{0XEA, 1, {0XA2}},
{0XFF, 1, {0X05}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X0, 1, {0X35}},
{0X1, 1, {0X08}},
{0X2, 1, {0X06}},
{0X3, 1, {0X04}},
{0X4, 1, {0X34}},
{0X5, 1, {0X1A}},
{0X6, 1, {0X1A}},
{0X7, 1, {0X16}},
{0X8, 1, {0X16}},
{0X9, 1, {0X22}},
{0XA, 1, {0X22}},
{0XB, 1, {0X1E}},
{0XC, 1, {0X1E}},
{0XD, 1, {0X05}},
{0XE, 1, {0X40}},
{0XF, 1, {0X40}},
{0X10, 1, {0X40}},
{0X11, 1, {0X40}},
{0X12, 1, {0X40}},
{0X13, 1, {0X40}},
{0X14, 1, {0X35}},
{0X15, 1, {0X09}},
{0X16, 1, {0X07}},
{0X17, 1, {0X04}},
{0X18, 1, {0X34}},
{0X19, 1, {0X1C}},
{0X1A, 1, {0X1C}},
{0X1B, 1, {0X18}},
{0X1C, 1, {0X18}},
{0X1D, 1, {0X24}},
{0X1E, 1, {0X24}},
{0X1F, 1, {0X20}},
{0X20, 1, {0X20}},
{0X21, 1, {0X05}},
{0X22, 1, {0X40}},
{0X23, 1, {0X40}},
{0X24, 1, {0X40}},
{0X25, 1, {0X40}},
{0X26, 1, {0X40}},
{0X27, 1, {0X40}},
{0X28, 1, {0X35}},
{0X29, 1, {0X07}},
{0X2A, 1, {0X09}},
{0X2B, 1, {0X04}},
{0X2D, 1, {0X34}},
{0X2F, 1, {0X20}},
{0X30, 1, {0X20}},
{0X31, 1, {0X24}},
{0X32, 1, {0X24}},
{0X33, 1, {0X18}},
{0X34, 1, {0X18}},
{0X35, 1, {0X1C}},
{0X36, 1, {0X1C}},
{0X37, 1, {0X05}},
{0X38, 1, {0X40}},
{0X39, 1, {0X40}},
{0X3A, 1, {0X40}},
{0X3B, 1, {0X40}},
{0X3D, 1, {0X40}},
{0X3F, 1, {0X40}},
{0X40, 1, {0X35}},
{0X41, 1, {0X06}},
{0X42, 1, {0X08}},
{0X43, 1, {0X04}},
{0X44, 1, {0X34}},
{0X45, 1, {0X1E}},
{0X46, 1, {0X1E}},
{0X47, 1, {0X22}},
{0X48, 1, {0X22}},
{0X49, 1, {0X16}},
{0X4A, 1, {0X16}},
{0X4B, 1, {0X1A}},
{0X4C, 1, {0X1A}},
{0X4D, 1, {0X05}},
{0X4E, 1, {0X40}},
{0X4F, 1, {0X40}},
{0X50, 1, {0X40}},
{0X51, 1, {0X40}},
{0X52, 1, {0X40}},
{0X53, 1, {0X40}},
{0X54, 1, {0X08}},
{0X55, 1, {0X06}},
{0X56, 1, {0X08}},
{0X58, 1, {0X06}},
{0X59, 1, {0X1B}},
{0X5A, 1, {0X1B}},
{0X5B, 1, {0X48}},
{0X5C, 1, {0X0E}},
{0X5D, 1, {0X01}},
{0X65, 1, {0X00}},
{0X66, 1, {0X44}},
{0X67, 1, {0X00}},
{0X68, 1, {0X48}},
{0X69, 1, {0X0E}},
{0X6A, 1, {0X06}},
{0X6B, 1, {0X20}},
{0X6C, 1, {0X08}},
{0X6D, 1, {0X00}},
{0X76, 1, {0X00}},
{0X77, 1, {0X00}},
{0X78, 1, {0X02}},
{0X79, 1, {0X00}},
{0X7A, 1, {0X0A}},
{0X7B, 1, {0X05}},
{0X7C, 1, {0X00}},
{0X7D, 1, {0X0D}},
{0X7E, 1, {0X33}},
{0X7F, 1, {0X33}},
{0X80, 1, {0X33}},
{0X81, 1, {0X00}},
{0X82, 1, {0X00}},
{0X83, 1, {0X00}},
{0X84, 1, {0X30}},
{0X85, 1, {0XFF}},
{0X86, 1, {0XFF}},
{0XBB, 1, {0X88}},
{0XB7, 1, {0XFF}},
{0XB8, 1, {0X00}},
{0XBA, 1, {0X13}},
{0XBC, 1, {0X95}},
{0XBD, 1, {0XAA}},
{0XBE, 1, {0X08}},
{0XBF, 1, {0XA3}},
{0XC8, 1, {0X00}},
{0XC9, 1, {0X00}},
{0XCA, 1, {0X00}},
{0XCB, 1, {0X00}},
{0XCC, 1, {0X12}},
{0XCF, 1, {0X44}},
{0XD0, 1, {0X00}},
{0XD1, 1, {0X00}},
{0XD4, 1, {0X15}},
{0XD5, 1, {0XBF}},
{0XD6, 1, {0X22}},
{0X90, 1, {0X78}},
{0X91, 1, {0X10}},
{0X92, 1, {0X10}},
{0X97, 1, {0X08}},
{0X98, 1, {0X00}},
{0X99, 1, {0X00}},
{0X9B, 1, {0X68}},
{0X9C, 1, {0X0A}},
{0XFF, 1, {0X00}},
{REGFLAG_DELAY, 1, {}},
{0X36, 1, {0X00}},
{0XFF, 1, {0X01}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X6E, 1, {0X00}},
{0XFF, 1, {0X04}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X8, 1, {0X06}},
{0XFF, 1, {0X00}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X51, 1, {0XFF}},
{0X53, 1, {0X24}},
{0X55, 1, {0X00}},
{0XD3, 1, {0X12}},
{0XD4, 1, {0X16}},
{0X11, 0, {}},
{REGFLAG_DELAY, 120, {}},
{0X29, 0, {}},
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0, {}},
};

static struct LCM_setting_table lcm_resume_setting[] = {
{0XFF, 1, {0X01}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X0, 1, {0X01}},
{0X1, 1, {0X55}},
{0X2, 1, {0X59}},
{0X4, 1, {0X0C}},
{0X5, 1, {0X3A}},
{0X6, 1, {0X55}},
{0X7, 1, {0XD5}},
{0XD, 1, {0X9D}},
{0XE, 1, {0X9D}},
{0XF, 1, {0XE0}},
{0X10, 1, {0X03}},
{0X11, 1, {0X3C}},
{0X12, 1, {0X50}},
{0X15, 1, {0X60}},
{0X16, 1, {0X14}},
{0X17, 1, {0X14}},
{0X44, 1, {0X68}},
{0X45, 1, {0X88}},
{0X46, 1, {0X78}},
{0X68, 1, {0X13}},
{0X6D, 1, {0X33}},
{0X75, 1, {0X00}},
{0X76, 1, {0X00}},
{0X77, 1, {0X00}},
{0X78, 1, {0X22}},
{0X79, 1, {0X00}},
{0X7A, 1, {0X51}},
{0X7B, 1, {0X00}},
{0X7C, 1, {0X73}},
{0X7D, 1, {0X00}},
{0X7E, 1, {0X8D}},
{0X7F, 1, {0X00}},
{0X80, 1, {0XA4}},
{0X81, 1, {0X00}},
{0X82, 1, {0XB8}},
{0X83, 1, {0X00}},
{0X84, 1, {0XCA}},
{0X85, 1, {0X00}},
{0X86, 1, {0XD9}},
{0X87, 1, {0X01}},
{0X88, 1, {0X0D}},
{0X89, 1, {0X01}},
{0X8A, 1, {0X36}},
{0X8B, 1, {0X01}},
{0X8C, 1, {0X75}},
{0X8D, 1, {0X01}},
{0X8E, 1, {0XA7}},
{0X8F, 1, {0X01}},
{0X90, 1, {0XF2}},
{0X91, 1, {0X02}},
{0X92, 1, {0X2B}},
{0X93, 1, {0X02}},
{0X94, 1, {0X2B}},
{0X95, 1, {0X02}},
{0X96, 1, {0X61}},
{0X97, 1, {0X02}},
{0X98, 1, {0X9D}},
{0X99, 1, {0X02}},
{0X9A, 1, {0XC6}},
{0X9B, 1, {0X03}},
{0X9C, 1, {0X01}},
{0X9D, 1, {0X03}},
{0X9E, 1, {0X27}},
{0X9F, 1, {0X03}},
{0XA0, 1, {0X63}},
{0XA2, 1, {0X03}},
{0XA3, 1, {0X6B}},
{0XA4, 1, {0X03}},
{0XA5, 1, {0X72}},
{0XA6, 1, {0X03}},
{0XA7, 1, {0X7E}},
{0XA9, 1, {0X03}},
{0XAA, 1, {0X87}},
{0XAB, 1, {0X03}},
{0XAC, 1, {0X93}},
{0XAD, 1, {0X03}},
{0XAE, 1, {0X9B}},
{0XAF, 1, {0X03}},
{0XB0, 1, {0XA2}},
{0XB1, 1, {0X03}},
{0XB2, 1, {0XFF}},
{0XB3, 1, {0X00}},
{0XB4, 1, {0X00}},
{0XB5, 1, {0X00}},
{0XB6, 1, {0X22}},
{0XB7, 1, {0X00}},
{0XB8, 1, {0X51}},
{0XB9, 1, {0X00}},
{0XBA, 1, {0X73}},
{0XBB, 1, {0X00}},
{0XBC, 1, {0X8D}},
{0XBD, 1, {0X00}},
{0XBE, 1, {0XA4}},
{0XBF, 1, {0X00}},
{0XC0, 1, {0XB8}},
{0XC1, 1, {0X00}},
{0XC2, 1, {0XCA}},
{0XC3, 1, {0X00}},
{0XC4, 1, {0XD9}},
{0XC5, 1, {0X01}},
{0XC6, 1, {0X0D}},
{0XC7, 1, {0X01}},
{0XC8, 1, {0X36}},
{0XC9, 1, {0X01}},
{0XCA, 1, {0X75}},
{0XCB, 1, {0X01}},
{0XCC, 1, {0XA7}},
{0XCD, 1, {0X01}},
{0XCE, 1, {0XF2}},
{0XCF, 1, {0X02}},
{0XD0, 1, {0X2B}},
{0XD1, 1, {0X02}},
{0XD2, 1, {0X2B}},
{0XD3, 1, {0X02}},
{0XD4, 1, {0X61}},
{0XD5, 1, {0X02}},
{0XD6, 1, {0X9D}},
{0XD7, 1, {0X02}},
{0XD8, 1, {0XC6}},
{0XD9, 1, {0X03}},
{0XDA, 1, {0X01}},
{0XDB, 1, {0X03}},
{0XDC, 1, {0X27}},
{0XDD, 1, {0X03}},
{0XDE, 1, {0X63}},
{0XDF, 1, {0X03}},
{0XE0, 1, {0X6B}},
{0XE1, 1, {0X03}},
{0XE2, 1, {0X72}},
{0XE3, 1, {0X03}},
{0XE4, 1, {0X7E}},
{0XE5, 1, {0X03}},
{0XE6, 1, {0X87}},
{0XE7, 1, {0X03}},
{0XE8, 1, {0X93}},
{0XE9, 1, {0X03}},
{0XEA, 1, {0X9B}},
{0XEB, 1, {0X03}},
{0XEC, 1, {0XA2}},
{0XED, 1, {0X03}},
{0XEE, 1, {0XFF}},
{0XEF, 1, {0X00}},
{0XF0, 1, {0XD6}},
{0XF1, 1, {0X00}},
{0XF2, 1, {0XDD}},
{0XF3, 1, {0X00}},
{0XF4, 1, {0XEA}},
{0XF5, 1, {0X00}},
{0XF6, 1, {0XF7}},
{0XF7, 1, {0X01}},
{0XF8, 1, {0X03}},
{0XF9, 1, {0X01}},
{0XFA, 1, {0X0D}},
{0XFF, 1, {0X02}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X0, 1, {0X01}},
{0X1, 1, {0X17}},
{0X2, 1, {0X01}},
{0X3, 1, {0X21}},
{0X4, 1, {0X01}},
{0X5, 1, {0X2A}},
{0X6, 1, {0X01}},
{0X7, 1, {0X4B}},
{0X8, 1, {0X01}},
{0X9, 1, {0X68}},
{0XA, 1, {0X01}},
{0XB, 1, {0X98}},
{0XC, 1, {0X01}},
{0XD, 1, {0XC0}},
{0XE, 1, {0X02}},
{0XF, 1, {0X00}},
{0X10, 1, {0X02}},
{0X11, 1, {0X34}},
{0X12, 1, {0X02}},
{0X13, 1, {0X35}},
{0X14, 1, {0X02}},
{0X15, 1, {0X68}},
{0X16, 1, {0X02}},
{0X17, 1, {0XA4}},
{0X18, 1, {0X02}},
{0X19, 1, {0XCC}},
{0X1A, 1, {0X03}},
{0X1B, 1, {0X07}},
{0X1C, 1, {0X03}},
{0X1D, 1, {0X2F}},
{0X1E, 1, {0X03}},
{0X1F, 1, {0X6A}},
{0X20, 1, {0X03}},
{0X21, 1, {0X71}},
{0X22, 1, {0X03}},
{0X23, 1, {0X85}},
{0X24, 1, {0X03}},
{0X25, 1, {0X9A}},
{0X26, 1, {0X03}},
{0X27, 1, {0XB2}},
{0X28, 1, {0X03}},
{0X29, 1, {0XCA}},
{0X2A, 1, {0X03}},
{0X2B, 1, {0XE2}},
{0X2D, 1, {0X03}},
{0X2F, 1, {0XF5}},
{0X30, 1, {0X03}},
{0X31, 1, {0XFF}},
{0X32, 1, {0X00}},
{0X33, 1, {0XD6}},
{0X34, 1, {0X00}},
{0X35, 1, {0XDD}},
{0X36, 1, {0X00}},
{0X37, 1, {0XEA}},
{0X38, 1, {0X00}},
{0X39, 1, {0XF7}},
{0X3A, 1, {0X01}},
{0X3B, 1, {0X03}},
{0X3D, 1, {0X01}},
{0X3F, 1, {0X0D}},
{0X40, 1, {0X01}},
{0X41, 1, {0X17}},
{0X42, 1, {0X01}},
{0X43, 1, {0X21}},
{0X44, 1, {0X01}},
{0X45, 1, {0X2A}},
{0X46, 1, {0X01}},
{0X47, 1, {0X4B}},
{0X48, 1, {0X01}},
{0X49, 1, {0X68}},
{0X4A, 1, {0X01}},
{0X4B, 1, {0X98}},
{0X4C, 1, {0X01}},
{0X4D, 1, {0XC0}},
{0X4E, 1, {0X02}},
{0X4F, 1, {0X00}},
{0X50, 1, {0X02}},
{0X51, 1, {0X34}},
{0X52, 1, {0X02}},
{0X53, 1, {0X35}},
{0X54, 1, {0X02}},
{0X55, 1, {0X68}},
{0X56, 1, {0X02}},
{0X58, 1, {0XA4}},
{0X59, 1, {0X02}},
{0X5A, 1, {0XCC}},
{0X5B, 1, {0X03}},
{0X5C, 1, {0X07}},
{0X5D, 1, {0X03}},
{0X5E, 1, {0X2F}},
{0X5F, 1, {0X03}},
{0X60, 1, {0X6A}},
{0X61, 1, {0X03}},
{0X62, 1, {0X71}},
{0X63, 1, {0X03}},
{0X64, 1, {0X85}},
{0X65, 1, {0X03}},
{0X66, 1, {0X9A}},
{0X67, 1, {0X03}},
{0X68, 1, {0XB2}},
{0X69, 1, {0X03}},
{0X6A, 1, {0XCA}},
{0X6B, 1, {0X03}},
{0X6C, 1, {0XE2}},
{0X6D, 1, {0X03}},
{0X6E, 1, {0XF5}},
{0X6F, 1, {0X03}},
{0X70, 1, {0XFF}},
{0X71, 1, {0X00}},
{0X72, 1, {0XCD}},
{0X73, 1, {0X00}},
{0X74, 1, {0XD5}},
{0X75, 1, {0X00}},
{0X76, 1, {0XE3}},
{0X77, 1, {0X00}},
{0X78, 1, {0XEF}},
{0X79, 1, {0X00}},
{0X7A, 1, {0XFB}},
{0X7B, 1, {0X01}},
{0X7C, 1, {0X06}},
{0X7D, 1, {0X01}},
{0X7E, 1, {0X11}},
{0X7F, 1, {0X01}},
{0X80, 1, {0X1B}},
{0X81, 1, {0X01}},
{0X82, 1, {0X24}},
{0X83, 1, {0X01}},
{0X84, 1, {0X46}},
{0X85, 1, {0X01}},
{0X86, 1, {0X63}},
{0X87, 1, {0X01}},
{0X88, 1, {0X94}},
{0X89, 1, {0X01}},
{0X8A, 1, {0XBB}},
{0X8B, 1, {0X01}},
{0X8C, 1, {0XFC}},
{0X8D, 1, {0X02}},
{0X8E, 1, {0X32}},
{0X8F, 1, {0X02}},
{0X90, 1, {0X32}},
{0X91, 1, {0X02}},
{0X92, 1, {0X66}},
{0X93, 1, {0X02}},
{0X94, 1, {0XA3}},
{0X95, 1, {0X02}},
{0X96, 1, {0XCC}},
{0X97, 1, {0X03}},
{0X98, 1, {0X0B}},
{0X99, 1, {0X03}},
{0X9A, 1, {0X38}},
{0X9B, 1, {0X03}},
{0X9C, 1, {0X8A}},
{0X9D, 1, {0X03}},
{0X9E, 1, {0X97}},
{0X9F, 1, {0X03}},
{0XA0, 1, {0X97}},
{0XA2, 1, {0X03}},
{0XA3, 1, {0X99}},
{0XA4, 1, {0X03}},
{0XA5, 1, {0X9B}},
{0XA6, 1, {0X03}},
{0XA7, 1, {0X9D}},
{0XA9, 1, {0X03}},
{0XAA, 1, {0X9F}},
{0XAB, 1, {0X03}},
{0XAC, 1, {0XA1}},
{0XAD, 1, {0X03}},
{0XAE, 1, {0XA2}},
{0XAF, 1, {0X00}},
{0XB0, 1, {0XCD}},
{0XB1, 1, {0X00}},
{0XB2, 1, {0XD5}},
{0XB3, 1, {0X00}},
{0XB4, 1, {0XE3}},
{0XB5, 1, {0X00}},
{0XB6, 1, {0XEF}},
{0XB7, 1, {0X00}},
{0XB8, 1, {0XFB}},
{0XB9, 1, {0X01}},
{0XBA, 1, {0X06}},
{0XBB, 1, {0X01}},
{0XBC, 1, {0X11}},
{0XBD, 1, {0X01}},
{0XBE, 1, {0X1B}},
{0XBF, 1, {0X01}},
{0XC0, 1, {0X24}},
{0XC1, 1, {0X01}},
{0XC2, 1, {0X46}},
{0XC3, 1, {0X01}},
{0XC4, 1, {0X63}},
{0XC5, 1, {0X01}},
{0XC6, 1, {0X94}},
{0XC7, 1, {0X01}},
{0XC8, 1, {0XBB}},
{0XC9, 1, {0X01}},
{0XCA, 1, {0XFC}},
{0XCB, 1, {0X02}},
{0XCC, 1, {0X32}},
{0XCD, 1, {0X02}},
{0XCE, 1, {0X32}},
{0XCF, 1, {0X02}},
{0XD0, 1, {0X66}},
{0XD1, 1, {0X02}},
{0XD2, 1, {0XA3}},
{0XD3, 1, {0X02}},
{0XD4, 1, {0XCC}},
{0XD5, 1, {0X03}},
{0XD6, 1, {0X0B}},
{0XD7, 1, {0X03}},
{0XD8, 1, {0X38}},
{0XD9, 1, {0X03}},
{0XDA, 1, {0X8A}},
{0XDB, 1, {0X03}},
{0XDC, 1, {0X97}},
{0XDD, 1, {0X03}},
{0XDE, 1, {0X97}},
{0XDF, 1, {0X03}},
{0XE0, 1, {0X99}},
{0XE1, 1, {0X03}},
{0XE2, 1, {0X9B}},
{0XE3, 1, {0X03}},
{0XE4, 1, {0X9D}},
{0XE5, 1, {0X03}},
{0XE6, 1, {0X9F}},
{0XE7, 1, {0X03}},
{0XE8, 1, {0XA1}},
{0XE9, 1, {0X03}},
{0XEA, 1, {0XA2}},
{0XFF, 1, {0X05}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X0, 1, {0X35}},
{0X1, 1, {0X08}},
{0X2, 1, {0X06}},
{0X3, 1, {0X04}},
{0X4, 1, {0X34}},
{0X5, 1, {0X1A}},
{0X6, 1, {0X1A}},
{0X7, 1, {0X16}},
{0X8, 1, {0X16}},
{0X9, 1, {0X22}},
{0XA, 1, {0X22}},
{0XB, 1, {0X1E}},
{0XC, 1, {0X1E}},
{0XD, 1, {0X05}},
{0XE, 1, {0X40}},
{0XF, 1, {0X40}},
{0X10, 1, {0X40}},
{0X11, 1, {0X40}},
{0X12, 1, {0X40}},
{0X13, 1, {0X40}},
{0X14, 1, {0X35}},
{0X15, 1, {0X09}},
{0X16, 1, {0X07}},
{0X17, 1, {0X04}},
{0X18, 1, {0X34}},
{0X19, 1, {0X1C}},
{0X1A, 1, {0X1C}},
{0X1B, 1, {0X18}},
{0X1C, 1, {0X18}},
{0X1D, 1, {0X24}},
{0X1E, 1, {0X24}},
{0X1F, 1, {0X20}},
{0X20, 1, {0X20}},
{0X21, 1, {0X05}},
{0X22, 1, {0X40}},
{0X23, 1, {0X40}},
{0X24, 1, {0X40}},
{0X25, 1, {0X40}},
{0X26, 1, {0X40}},
{0X27, 1, {0X40}},
{0X28, 1, {0X35}},
{0X29, 1, {0X07}},
{0X2A, 1, {0X09}},
{0X2B, 1, {0X04}},
{0X2D, 1, {0X34}},
{0X2F, 1, {0X20}},
{0X30, 1, {0X20}},
{0X31, 1, {0X24}},
{0X32, 1, {0X24}},
{0X33, 1, {0X18}},
{0X34, 1, {0X18}},
{0X35, 1, {0X1C}},
{0X36, 1, {0X1C}},
{0X37, 1, {0X05}},
{0X38, 1, {0X40}},
{0X39, 1, {0X40}},
{0X3A, 1, {0X40}},
{0X3B, 1, {0X40}},
{0X3D, 1, {0X40}},
{0X3F, 1, {0X40}},
{0X40, 1, {0X35}},
{0X41, 1, {0X06}},
{0X42, 1, {0X08}},
{0X43, 1, {0X04}},
{0X44, 1, {0X34}},
{0X45, 1, {0X1E}},
{0X46, 1, {0X1E}},
{0X47, 1, {0X22}},
{0X48, 1, {0X22}},
{0X49, 1, {0X16}},
{0X4A, 1, {0X16}},
{0X4B, 1, {0X1A}},
{0X4C, 1, {0X1A}},
{0X4D, 1, {0X05}},
{0X4E, 1, {0X40}},
{0X4F, 1, {0X40}},
{0X50, 1, {0X40}},
{0X51, 1, {0X40}},
{0X52, 1, {0X40}},
{0X53, 1, {0X40}},
{0X54, 1, {0X08}},
{0X55, 1, {0X06}},
{0X56, 1, {0X08}},
{0X58, 1, {0X06}},
{0X59, 1, {0X1B}},
{0X5A, 1, {0X1B}},
{0X5B, 1, {0X48}},
{0X5C, 1, {0X0E}},
{0X5D, 1, {0X01}},
{0X65, 1, {0X00}},
{0X66, 1, {0X44}},
{0X67, 1, {0X00}},
{0X68, 1, {0X48}},
{0X69, 1, {0X0E}},
{0X6A, 1, {0X06}},
{0X6B, 1, {0X20}},
{0X6C, 1, {0X08}},
{0X6D, 1, {0X00}},
{0X76, 1, {0X00}},
{0X77, 1, {0X00}},
{0X78, 1, {0X02}},
{0X79, 1, {0X00}},
{0X7A, 1, {0X0A}},
{0X7B, 1, {0X05}},
{0X7C, 1, {0X00}},
{0X7D, 1, {0X0D}},
{0X7E, 1, {0X33}},
{0X7F, 1, {0X33}},
{0X80, 1, {0X33}},
{0X81, 1, {0X00}},
{0X82, 1, {0X00}},
{0X83, 1, {0X00}},
{0X84, 1, {0X30}},
{0X85, 1, {0XFF}},
{0X86, 1, {0XFF}},
{0XBB, 1, {0X88}},
{0XB7, 1, {0XFF}},
{0XB8, 1, {0X00}},
{0XBA, 1, {0X13}},
{0XBC, 1, {0X95}},
{0XBD, 1, {0XAA}},
{0XBE, 1, {0X08}},
{0XBF, 1, {0XA3}},
{0XC8, 1, {0X00}},
{0XC9, 1, {0X00}},
{0XCA, 1, {0X00}},
{0XCB, 1, {0X00}},
{0XCC, 1, {0X12}},
{0XCF, 1, {0X44}},
{0XD0, 1, {0X00}},
{0XD1, 1, {0X00}},
{0XD4, 1, {0X15}},
{0XD5, 1, {0XBF}},
{0XD6, 1, {0X22}},
{0X90, 1, {0X78}},
{0X91, 1, {0X10}},
{0X92, 1, {0X10}},
{0X97, 1, {0X08}},
{0X98, 1, {0X00}},
{0X99, 1, {0X00}},
{0X9B, 1, {0X68}},
{0X9C, 1, {0X0A}},
{0XFF, 1, {0X00}},
{REGFLAG_DELAY, 1, {}},
{0X36, 1, {0X00}},
{0XFF, 1, {0X01}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X6E, 1, {0X00}},
{0XFF, 1, {0X04}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X8, 1, {0X06}},
{0XFF, 1, {0X00}},
{REGFLAG_DELAY, 1, {}},
{0XFB, 1, {0X01}},
{0X51, 1, {0XFF}},
{0X53, 1, {0X24}},
{0X55, 1, {0X00}},
{0XD3, 1, {0X12}},
{0XD4, 1, {0X16}},
{0X11, 0, {}},
{REGFLAG_DELAY, 120, {}},
{0X29, 0, {}},
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0, {}},
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
                    if(table[i].count <= 10)
                        MDELAY(table[i].count);
                    else
                        MDELAY(table[i].count);
                    break;
                    
                case REGFLAG_END_OF_TABLE :
                    break;
                    
                default:
                    dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            }
        }
    }
    
    
    
    static void tps65132_enable(bool enable){
        int i;
        mt_set_gpio_mode(GPIO_MHL_RST_B_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_MHL_RST_B_PIN, GPIO_DIR_OUT);
        mt_set_gpio_mode(GPIO_MHL_EINT_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_MHL_EINT_PIN, GPIO_DIR_OUT);
        if (enable){
            mt_set_gpio_out(GPIO_MHL_EINT_PIN, GPIO_OUT_ONE);
            MDELAY(12);
            mt_set_gpio_out(GPIO_MHL_RST_B_PIN, GPIO_OUT_ONE);
            MDELAY(12);
            for (i=0; i < 3; i++){
                if ((tps65132_write_bytes(0, 0xF) & 0x1f)==0) break;
                MDELAY(5);
            }
        }else{
            mt_set_gpio_out(GPIO_MHL_RST_B_PIN, GPIO_OUT_ZERO);
            MDELAY(12);
            mt_set_gpio_out(GPIO_MHL_EINT_PIN, GPIO_OUT_ZERO);
            MDELAY(12);
        }
        
        
    }
    
    #ifndef BUILD_LK
    static void KTD3116_Boe_SendData(unsigned char value){
        int i;
        raw_spin_lock_irq(&boe_SpinLock);
        mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ONE);
        UDELAY(15);
        for (i=7; i >= 0; i--){
            if ((value >> i)&1){
                mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ZERO);
                UDELAY(10);
                mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ONE);
                UDELAY(30);
            }else{
                mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ZERO);
                UDELAY(30);
                mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ONE);
                UDELAY(10);
            }
        }
        mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ZERO);
        UDELAY(15);
        mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ONE);
        UDELAY(350);
        raw_spin_unlock_irq(&boe_SpinLock);
    }
    #endif
    
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
        
        params->disp_vendor1 = "boe";
        params->disp_vendor2 = "boe";
        params->disp_class = "nt35532";
        params->sres = "1080*1920";
        params->physical_width = 68;
        params->dsi.noncont_clock = 1;
        params->dsi.esd_check_enable = 1;
        params->dsi.customization_esd_check_enable = 1;
        params->dsi.lcm_esd_check_table[0].count = 1;
        params->physical_height = 121;
        params->dsi.mode = 3;
        params->dsi.PLL_CLOCK = 475;
        params->dsi.lcm_esd_check_table[0].cmd = 0xA;
        params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9Cu;
        params->type = 2;
        params->dsi.data_format.format = 2;
        params->dsi.PS = 2;
        params->dsi.vertical_sync_active = 2;
        params->width = 1080;
        params->dsi.horizontal_active_pixel = 1080;
        params->height = 1920;
        params->dsi.vertical_active_line = 1920;
        params->dsi.switch_mode_enable = 0;
        params->dsi.data_format.color_order = 0;
        params->dsi.data_format.trans_seq = 0;
        params->dsi.data_format.padding = 0;
        params->dsi.LANE_NUM = 4;
        params->dsi.horizontal_sync_active = 4;
        params->dsi.vertical_backporch = 16;
        params->dsi.vertical_frontporch = 16;
        params->dsi.horizontal_backporch = 76;
        params->dsi.horizontal_frontporch = 76;
    }
    
    static void lcm_init(void)
    {
        tps65132_enable(TRUE);
        MDELAY(20);
        mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(20);
        
        // when phone initial , config output high, enable backlight drv chip  
        push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
        MDELAY(20);
    }
    
    static void lcm_suspend(void)
    {
        push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);  
        tps65132_enable(FALSE);
        MDELAY(10);
        
        mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
        MDELAY(20);
    }
    
    static void lcm_resume(void)
    {
        tps65132_enable(TRUE);
        MDELAY(15);
        mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
        MDELAY(5);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(20);
        #ifdef BUILD_LK
        push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  
        #else
        if (lcm_intialized){
            push_table(lcm_resume_setting, sizeof(lcm_resume_setting) / sizeof(struct LCM_setting_table), 1);  
        }else{
            push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  
        }
        #endif
    }
    
    
    static unsigned int lcm_compare_id(void)
    {
        
        
        unsigned int id=0;
        unsigned char buffer[2];
        unsigned int array[16];  
        
        mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(1);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
        MDELAY(10);
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
        MDELAY(10);
        
        array[0] = 0x23700;// read id return two byte,version and id
        dsi_set_cmdq(array, 1, 1);
        array[0] = 0xFF1500;
        dsi_set_cmdq(array, 1, 1);
        array[0] = 0x1FB1500;
        dsi_set_cmdq(array, 1, 1);
        MDELAY(10);
        read_reg_v2(0xF4, buffer, 1);
        MDELAY(20);
        id = buffer[0]; //we only need ID
        #ifdef BUILD_LK
        dprintf(0, "%s, LK NT35532 debug: NT35532 id = 0x%08x\n", __func__, id);
        #else
        printk("%s, kernel NT35532 horse debug: NT35532 id = 0x%08x\n", __func__, id);
        #endif
        return (id == 0x32)?1:0;
    }
    
    
    static void lcm_setbacklight_cmdq(void* handle, unsigned int level)
    {
        #ifdef BUILD_LK
        
        if ( level )
        {
            if ( level - 1 > 3 )
            {
                if ( level >= 0xFF )
                    level = 0xFF;
            }
            else
            {
                level = 4;
            }
            mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ONE);
            MDELAY(10);
        }
        else
        {
            mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ZERO);
            MDELAY(30);
        }
        push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);  
        
        #else
        if (level != boe_value)
        {
            boe_value = level;
            boe_first_vlue = level;
            if (boe_first_vlue!=0 || boe_second_vlue !=0 ){
                mt_set_gpio_mode(GPIO_MHL_POWER_CTRL_PIN, GPIO_MODE_00);
                mt_set_gpio_dir(GPIO_MHL_POWER_CTRL_PIN, GPIO_DIR_OUT);
                
                if (level){	
                    
                    
                    
                    if (level - 1 > 3){
                        if (level > 255)
                            level = 255;
                    }else{
                        level = 4;
                    }
                    
                    global_brightnest_level = level;
                    
                    
                    if (level < 31){
                        KTD3116_Boe_SendData(64 - level*2);
                    }else{
                        if (boe_first_vlue >= 31){
                            if (boe_second_vlue >= 31){
                                mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ONE);
                                MDELAY(10);
                            }else{
                                KTD3116_Boe_SendData(0);
                            }
                        }	
                    }
                }else{
                    mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ZERO);
                    MDELAY(30); //10
                }
                
                lcm_backlight_level_setting[4].para_list[0] = (unsigned char)level;
                push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);  
                boe_second_vlue = boe_first_vlue;
            }
        }
        
        #endif
    }
    
    #ifndef BUILD_LK
    static void lcm_cabc_enable_cmdq(unsigned int mode)
    {
        if (mode){
            push_table(lcm_backlight_enable, sizeof(lcm_backlight_enable) / sizeof(struct LCM_setting_table), 1);  
        }else{
            push_table(lcm_backlight_disable, sizeof(lcm_backlight_disable) / sizeof(struct LCM_setting_table), 1);  
        }
        
    }
    #endif
    
    
    
    LCM_DRIVER nt35532_fhd_boe_vdo_lcm_drv=
    {
        .name           	= "nt35532_fhd_boe_vdo_lcm",
        .set_util_funcs 	= lcm_set_util_funcs,
        .get_params     	= lcm_get_params,
        .init           	= lcm_init,
        .suspend        	= lcm_suspend,
        .resume         	= lcm_resume,
        .compare_id     	= lcm_compare_id,
        .set_backlight_cmdq	= lcm_setbacklight_cmdq,
        #ifndef BUILD_LK
        .set_pwm			= lcm_cabc_enable_cmdq,
        #endif
        
    };
    
