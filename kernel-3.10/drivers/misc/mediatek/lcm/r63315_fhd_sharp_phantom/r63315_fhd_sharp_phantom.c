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

static int sharp_value;
static int sharp_first_vlue;
static int sharp_second_vlue;

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

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  										   (1080)
#define FRAME_HEIGHT 										   (1920)


#define REGFLAG_DELAY           0xFC
#define REGFLAG_END_OF_TABLE    0xFD   // END OF REGISTERS MARKER


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------


struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0, {}}
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
    {0x28, 0x00, {}},
    {REGFLAG_DELAY, 40,{}},
    {0x10, 0x00, {}},
    {REGFLAG_DELAY, 100,{}},
    {REGFLAG_END_OF_TABLE, 0x00,{}},
    
};

static struct LCM_setting_table lcm_initialization_setting[] = {
    {0xb0, 0x01, {0x00}},
    {0xc7, 0x1e, {0x05, 0x14, 0x1e, 0x2a, 0x3b, 0x4a, 0x54, 0x62, 0x47, 0x50, 0x5d, 0x69, 0x70, 0x74, 0x77, 0x07, 0x16, 0x20, 0x2a, 0x3b, 0x4a, 0x54, 0x62, 0x47, 0x4f, 0x5a, 0x66, 0x6d, 0x70, 0x73}},
    {0xc8, 0x13, {0x01, 0x00, 0x00, 0xfd, 0x02, 0xfc, 0x00, 0x00, 0xff, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x01, 0xf5, 0xf6, 0xfc, 0x00}},
    {0xd6, 0x01, {0x01}},
    {0xb0, 0x01, {0x03}},
    {0x35, 0x01, {0x00}},
    {0x51, 0x01, {0xff}},
    {0x53, 0x01, {0x24}},
    {0x55, 0x01, {0x00}},
    {0x29, 0x00, {}},
    {REGFLAG_DELAY, 20,{}},
    {0x11, 0x00, {}},
    {REGFLAG_DELAY, 120,{}},
    {REGFLAG_END_OF_TABLE, 0x00,{}},
    
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    
    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) 
        {
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

static int get_backlight_pos(struct LCM_setting_table *table, unsigned int count)
{
    int i;
    
    for(i = count - 1; i >=0; i--)
    {
        if (table[i].cmd == 0x51)
            return i;
    }
    return -1;
}

static void tps65132_enable(bool enable){
    int i;
    mt_set_gpio_mode(GPIO_MHL_RST_B_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_MHL_RST_B_PIN, GPIO_DIR_OUT);
    mt_set_gpio_mode(GPIO_MHL_EINT_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_MHL_EINT_PIN, GPIO_DIR_OUT);
    if (enable)
    {
        mt_set_gpio_out(GPIO_MHL_EINT_PIN, GPIO_OUT_ONE);
        MDELAY(12);
        mt_set_gpio_out(GPIO_MHL_RST_B_PIN, GPIO_OUT_ONE);
        MDELAY(12);
        for (i=0; i < 3; i++)
        {
            if ((tps65132_write_bytes(0, 0xF) & 0x1f)==0) 
                break;
            MDELAY(5);
        }
    }
    else
    {
        mt_set_gpio_out(GPIO_MHL_RST_B_PIN, GPIO_OUT_ZERO);
        MDELAY(12);
        mt_set_gpio_out(GPIO_MHL_EINT_PIN, GPIO_OUT_ZERO);
        MDELAY(12);
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
    
    params->disp_vendor1 = "sharp";
    params->disp_vendor2 = "sharp";
    params->disp_class = "r63315";
    params->sres = "1080*1920";
    
    params->dsi.horizontal_frontporch = 90;
    params->physical_width = 68;
    params->physical_height = 121;
    params->dsi.mode = 3;
    params->dsi.horizontal_sync_active = 10;
    params->dsi.horizontal_backporch = 50;
    params->dsi.PLL_CLOCK = 475;
    params->type = 2;
    params->dsi.data_format.format = 2;
    params->dsi.PS = 2;
    params->dsi.HS_TRAIL = 2;
    params->dsi.vertical_sync_active = 2;
    params->width = FRAME_WIDTH;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->dsi.vertical_active_line = FRAME_HEIGHT;
    params->dsi.switch_mode_enable = 0;
    params->dsi.data_format.color_order = 0;
    params->dsi.data_format.trans_seq = 0;
    params->dsi.data_format.padding = 0;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.noncont_clock = 1;
    params->dsi.esd_check_enable = 1;
    params->dsi.LANE_NUM = 4;
    params->dsi.vertical_backporch = 4;
    params->dsi.vertical_frontporch = 4;
    
}

static void lcm_init(void)
{
    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    MDELAY(5);

    // when phone initial , config output high, enable backlight drv chip  
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);  
    
    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    MDELAY(10);
    tps65132_enable(FALSE);
}

static void lcm_resume(void)
{
#ifndef BUILD_LK
    int bpos;
#endif 
    
    tps65132_enable(TRUE);
    MDELAY(20);
    
    lcm_init();    
    
#ifdef BUILD_LK
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
#else
    bpos = get_backlight_pos(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table));  
    if (bpos >= 0 )
        lcm_initialization_setting[bpos].para_list[0] = (unsigned char)global_brightnest_level;
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  
#endif 
}

static unsigned int lcm_compare_id(void)
{
    unsigned int id=0;
    unsigned char buffer[6];
    unsigned int array[16];  
    
    tps65132_enable(TRUE);
    MDELAY(20);
    
    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    MDELAY(10);
    
    array[0] = 0x53700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    MDELAY(20);
    array[0] = 0x4B02900;
    dsi_set_cmdq(array, 1, 1);
    MDELAY(50);
    read_reg_v2(0xBF, buffer, 5);
    id = buffer[3] | (buffer[2] << 8);

#ifdef BUILD_LK
    dprintf(0, "%s, LK r63315 debug: r63315 id = 0x%08x\n", __func__, id);
#else
    printk("%s, kernel r63315  debug: r63315 id = 0x%08x\n", __func__, id);
#endif

    return (id == 0x3315)?1:0;
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
    lcm_backlight_level_setting[0].para_list[0] = (unsigned char)level;
    push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);  

#else
    if (level != sharp_value)
    {
        sharp_value = level;
        sharp_first_vlue = level;
        if (sharp_first_vlue!=0 || sharp_second_vlue !=0 )
        {
            mt_set_gpio_mode(GPIO_MHL_POWER_CTRL_PIN, GPIO_MODE_00);
            mt_set_gpio_dir(GPIO_MHL_POWER_CTRL_PIN, GPIO_DIR_OUT);
            
            if (level)
            { 
                if (level - 1 > 3)
                {
                    if (level > 255)
                        level = 255;
                }
                else
                {
                    level = 4;
                }
                
                global_brightnest_level = level;
                
                mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ONE);
                MDELAY(10);
            }
            else
            {
                mt_set_gpio_out(GPIO_MHL_POWER_CTRL_PIN, GPIO_OUT_ZERO);
                MDELAY(30); //10
            }
            
            lcm_backlight_level_setting[0].para_list[0] = (unsigned char)level;
            push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);  
            sharp_second_vlue = sharp_first_vlue;
        }
    }
#endif
}

#ifndef BUILD_LK
static void lcm_cabc_enable_cmdq(unsigned int mode)
{
    if (mode)
        push_table(lcm_backlight_enable, sizeof(lcm_backlight_enable) / sizeof(struct LCM_setting_table), 1);  
    else
        push_table(lcm_backlight_disable, sizeof(lcm_backlight_disable) / sizeof(struct LCM_setting_table), 1);  
}
#endif

LCM_DRIVER r63315_fhd_sharp_phantom_lcm_drv=
{
    .name           	= "r63315_fhd_sharp_phantom",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,/*sharp init fun.*/
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
    .compare_id     	= lcm_compare_id,
    .set_backlight_cmdq	= lcm_setbacklight_cmdq,
    #ifndef BUILD_LK
    .set_pwm			= lcm_cabc_enable_cmdq,
    #endif
    
};
