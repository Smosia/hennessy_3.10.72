#ifndef _CPT_clap070wp03xg_wxga_lvds_
#define _CPT_clap070wp03xg_wxga_lvds_

#include "ddp_hal.h"

typedef unsigned char kal_uint8;
typedef struct {
	unsigned char cmd;
	unsigned char data;
} sn65dsi8x_setting_table;

extern unsigned int GPIO_LCD_PWR;
extern unsigned int GPIO_LCD_PWR_EN;
extern unsigned int GPIO_LCD_PWR2_EN;
extern unsigned int GPIO_LCD_BRIDGE_EN;

/* int lcm_vgp_supply_disable(void); */
/* int lcm_vgp_supply_enable(void); */

/* extern void DSI_clk_HS_mode(DISP_MODULE_ENUM module, void *cmdq, bool enter); */

#endif
