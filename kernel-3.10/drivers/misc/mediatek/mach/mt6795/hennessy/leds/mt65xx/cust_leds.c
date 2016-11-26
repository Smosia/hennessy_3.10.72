#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

extern int mtkfb_set_backlight_level(unsigned int level);

#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64 
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256 
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

void lct_set_button_keypad(int mode){
	mt_set_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_DIR_OUT);
if (mode){
	mt_set_gpio_out(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_OUT_ONE);
}else{
	mt_set_gpio_out(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_OUT_ZERO);
}
}

unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;
    
    mapped_level = level;
       
	return mapped_level;
}

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red", 		MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	{"green", 		MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"blue", 		MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
        {"yellow",		MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
	{"white", 		MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
	{"cyan", 		MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
	{"violet", 		MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
	{"jogball-backlight", 	MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",	MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",    MT65XX_LED_MODE_GPIO, (long)lct_set_button_keypad,{0}},
	{"lcd-backlight",       MT65XX_LED_MODE_CUST_LCM, (long)mtkfb_set_backlight_level,{0}},
};  

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

