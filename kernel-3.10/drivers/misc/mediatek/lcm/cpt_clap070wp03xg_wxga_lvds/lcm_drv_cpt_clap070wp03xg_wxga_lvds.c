#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
/* #include <linux/regulator/consumer.h> */
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

#include "cpt_clap070wp03xg_wxga_lvds.h"

void lcm_request_gpio_control(struct device *dev)
{
	GPIO_LCD_PWR = of_get_named_gpio(dev->of_node, "gpio_lcm_pwr", 0);
	GPIO_LCD_PWR_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_pwr_en", 0);
	GPIO_LCD_PWR2_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_pwr2_en", 0);
	GPIO_LCD_BRIDGE_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_bridge_en", 0);

	gpio_request(GPIO_LCD_PWR, "GPIO_LCD_PWR");
	pr_debug("[KE/LCM] GPIO_LCD_PWR = 0x%x\n", GPIO_LCD_PWR);

	gpio_request(GPIO_LCD_PWR_EN, "GPIO_LCD_PWR_EN");
	pr_debug("[KE/LCM] GPIO_LCD_PWR_EN = 0x%x\n", GPIO_LCD_PWR_EN);

	gpio_request(GPIO_LCD_PWR2_EN, "GPIO_LCD_PWR2_EN");
	pr_debug("[KE/LCM] GPIO_LCD_PWR2_EN = 0x%x\n", GPIO_LCD_PWR2_EN);

	gpio_request(GPIO_LCD_BRIDGE_EN, "GPIO_LCD_BRIDGE_EN");
	pr_debug("[KE/LCM] GPIO_LCD_BRIDGE_EN = 0x%x\n", GPIO_LCD_BRIDGE_EN);
}

static int lcm_probe(struct device *dev)
{
	lcm_request_gpio_control(dev);

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,mt6592-lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "cpt_clap070wp03xg_wxga_lvds",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_debug("LCM: Register panel driver for cpt_clap070wp03xg_wxga_lvds\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_debug("LCM: Unregister this driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
