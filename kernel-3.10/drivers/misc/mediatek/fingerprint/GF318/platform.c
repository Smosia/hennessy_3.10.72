#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/sort.h>

#include <asm/irq.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>

//#include <mach/irqs.h>
#include <linux/of_irq.h>
#include <asm/uaccess.h>

#include <linux/io.h>   
   
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/eint.h>
#include <cust_eint.h>

#include "gf318m-spi.h"

int gf66xx_parse_dts(struct gf66xx_dev *gf66xx_dev)
{
    int retval = 0;
    printk("ahead of gf66xx_parse_dts\n");
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
#ifndef ONTIM_MTK6735
     gf66xx_dev->power_gpio =  GF66XX_PWR_PIN;
    /*power pin*/
    retval =  mt_set_gpio_mode(gf66xx_dev->power_gpio, GF66XX_PWR_PIN_M_GPIO);
    if(retval < 0) {
        pr_info("Failed to set power gpio. retval = %d\n", retval);
        return retval;
    } else {
        retval = mt_set_gpio_dir(gf66xx_dev->power_gpio, GPIO_DIR_OUT);
        if(retval < 0) {
            pr_info("Failed to set power gpio as output. retval = %d\n", retval);
            return retval;
        }
    }
#endif
    gf66xx_dev->irq_gpio =  GF66XX_INT_PIN;
    gf66xx_dev->reset_gpio =  GF66XX_RST_PIN;
	
    /*reset pin as output GPIO with pull-up*/ 
       printk(KERN_ERR"init Reset pin in gf66xx_parse_dts \n");
	/*
	retval = mt_set_gpio_mode(gf66xx_dev->reset_gpio, GF66XX_SPI_RESET_PIN_M_GPIO);
	retval = mt_set_gpio_dir(gf66xx_dev->reset_gpio, GPIO_DIR_OUT);
      if(retval < 0) {
	    pr_info("Failed to set reset gpio as output. retval = %d\n", retval);
          return retval;
      }
      */
       retval = mt_set_gpio_mode(GF66XX_RST_PIN, GPIO_MODE_00);
	printk("mt_set_gpio_mode Ok.\n");
	if (retval != 0) {
		printk(KERN_ERR"[GF318M]gpio_request (reset) failed.\n");
		return retval;
	}
	 retval = mt_set_gpio_dir(GF66XX_RST_PIN, GPIO_DIR_OUT);
        if (retval != 0) {
		printk(KERN_ERR "gpio_direction_output(reset) failed.\n");
			return retval;
	} 
       printk("init Reset pin done in gf66xx_parse_dts \n");
	   
    retval = mt_set_gpio_pull_enable(gf66xx_dev->reset_gpio, GPIO_PULL_ENABLE);
    if(retval < 0) {
	    pr_info("Failed to set reset gpio pull. retval = %d\n", retval);
        return retval;
    }
    retval = mt_set_gpio_pull_select(gf66xx_dev->reset_gpio, GPIO_PULL_UP);
    if(retval < 0) {
	    pr_info("Failed to set reset gpio pull-up. retval = %d\n", retval);
        return retval;
    }
	printk("end of gf66xx_parse_dts\n");
    return 0;
}

int gf66xx_power_on(struct gf66xx_dev *gf66xx_dev)
{
   /*power enable*/
     int retval = 0;
    printk("ahead of gf66xx_power_on\n");
    if(gf66xx_dev == NULL) {
	  printk("gf66xx_dev == NULL \n");
         pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
#ifndef ONTIM_MTK6735
    retval =  mt_set_gpio_out(gf66xx_dev->power_gpio, GPIO_OUT_ONE);
    if(retval < 0) {
        pr_info("Failed to output power(enable). retval = %d\n", retval);
    } else {
        msleep(10);
    }
   return retval;
#else
    #ifdef GF66XX_POWER_SOURCE_CUSTOM 
	printk("GF66XX_POWER_SOURCE_CUSTOM \n");
        //hwPowerOn(GF66XX_POWER_SOURCE_CUSTOM, VOL_2800, "GX_FP");    //this power controled by TP
    #else
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "GX_FP");
    #endif
#endif
	printk("end of gf66xx_power_on\n");
}

int gf66xx_power_off(struct gf66xx_dev *gf66xx_dev)
{
    /*power disable*/
    int retval = 0;
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
#ifndef ONTIM_MTK6735
    retval = mt_set_gpio_out(gf66xx_dev->power_gpio, GPIO_OUT_ZERO);
    if(retval < 0) {
        pr_info("Failed to output power(disable). retval = %d\n", retval);
    }
    return retval;
#else
     #ifdef GF66XX_POWER_SOURCE_CUSTOM 
	printk("GF66XX_POWER_SOURCE_CUSTOM \n");
        //hwPowerDown(GF66XX_POWER_SOURCE_CUSTOM, VOL_2800, "GX_FP");    //this power controled by TP
    #else
        hwPowerDown(MT65XX_POWER_LDO_VGP2, VOL_2800, "GX_FP");
    #endif
#endif
}

int gf66xx_irq_setup(struct gf66xx_dev *gf66xx_dev, void (*irq_handler)(void )) //need change yfpan
{
    /*IRQ pin as input gpio. no pull-down/pull-up.*/
    int retval = 0;
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }

	retval = mt_set_gpio_mode(gf66xx_dev->irq_gpio, GF66XX_SPI_EINT_PIN_M_EINT); //set to eint MODE for enable eint function
    if(retval < 0) {
        printk(KERN_ERR"[gf66xx_irq_setup]Failed to set IRQ pin as INT. retval = %d\n", retval);
        return retval;
    }
	retval = mt_set_gpio_dir(gf66xx_dev->irq_gpio, GPIO_DIR_IN); 
    if(retval < 0) {
        printk(KERN_ERR"[gf66xx_irq_setup]Failed to set IRQ as input. retval = %d\n", retval);
        return retval;
    }
	retval = mt_set_gpio_pull_enable(gf66xx_dev->irq_gpio, GPIO_PULL_DISABLE);
    if(retval < 0) {
         printk(KERN_ERR"[gf66xx_irq_setup]Failed to set pull for IRQ pin. retval = %d\n", retval);
        return retval;
    }
	 printk(KERN_ERR"[gf66xx_irq_setup]:SPI GPIO EINT PIN mode:num:%d, %d, dir:%d,pullen:%d,pullup%d\n",(int)gf66xx_dev->irq_gpio,
				(int)mt_get_gpio_mode(gf66xx_dev->irq_gpio), (int)mt_get_gpio_dir(gf66xx_dev->irq_gpio),
				(int)mt_get_gpio_pull_enable(gf66xx_dev->irq_gpio),(int)mt_get_gpio_pull_select(gf66xx_dev->irq_gpio));    
	
    mt_eint_registration(gf66xx_dev->spi->irq, EINTF_TRIGGER_RISING, irq_handler, 1);         // 0:auto mask is no
    mt_eint_mask(gf66xx_dev->spi->irq); 
	
	return 0;
}

void gf66xx_spi_pins_config(void)
{
	/*cs*/
    mt_set_gpio_mode(GF66XX_SPI_CS_PIN, GF66XX_SPI_CS_PIN_M_CS);
    mt_set_gpio_pull_enable(GF66XX_SPI_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GF66XX_SPI_CS_PIN, GPIO_PULL_UP);
	/*sck*/
    mt_set_gpio_mode(GF66XX_SPI_SCK_PIN, GF66XX_SPI_SCK_PIN_M_SCK);
    mt_set_gpio_pull_enable(GF66XX_SPI_SCK_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GF66XX_SPI_SCK_PIN, GPIO_PULL_DOWN);
	/*miso*/
    mt_set_gpio_mode(GF66XX_SPI_MISO_PIN, GF66XX_SPI_MISO_PIN_M_MISO);
    mt_set_gpio_pull_enable(GF66XX_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GF66XX_SPI_MISO_PIN, GPIO_PULL_UP);
	/*mosi*/
    mt_set_gpio_mode(GF66XX_SPI_MOSI_PIN, GF66XX_SPI_MOSI_PIN_M_MOSI);
    mt_set_gpio_pull_enable(GF66XX_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GF66XX_SPI_MOSI_PIN, GPIO_PULL_UP);

    msleep(1);
}

int gf66xx_irq_num(struct gf66xx_dev *gf66xx_dev)
{
	return GF66XX_IRQ_NUM;
}
int gf66xx_irq_open(int irq_no)
{
	printk(KERN_ERR" [gf66xx_irq_open] unmask int!\n");
	mt_eint_unmask(irq_no);   
        return 0;
}
int gf66xx_irq_close(int irq_no)
{
	mt_eint_mask(irq_no);  
	printk(KERN_ERR" [gf66xx_irq_close] mask int!\n");
	return 0;
}

int gf66xx_irq_release(struct gf66xx_dev *gf66xx_dev)
{
    int retval = 0;
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }

//	mt_eint_mask(gf66xx_dev->spi->irq);   
	mt_set_gpio_pull_enable(gf66xx_dev->irq_gpio, 0);
	mt_set_gpio_pull_select(gf66xx_dev->irq_gpio,  0);
	retval = mt_set_gpio_mode(gf66xx_dev->irq_gpio, GF66XX_SPI_EINT_PIN_M_GPIO); //set to eint MODE for enable eint function
    if(retval < 0) {
        pr_info("[gf66xx_irq_release]Failed to set IRQ pin as GPIO. retval = %d\n", retval);
    } else {
	    pr_info("[gf66xx_irq_release]SPI GPIO EINT PIN mode:num:%d, %d, dir:%d,pullen:%d,pullup%d\n",(int)gf66xx_dev->irq_gpio,
				(int)mt_get_gpio_mode(gf66xx_dev->irq_gpio),(int)mt_get_gpio_dir(gf66xx_dev->irq_gpio),
				(int)mt_get_gpio_pull_enable(gf66xx_dev->irq_gpio),(int)mt_get_gpio_pull_select(gf66xx_dev->irq_gpio));    
    }
    return retval;
}
int gf66xx_hw_reset(struct gf66xx_dev *gf66xx_dev, unsigned int delay_ms) 
{
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
	mt_set_gpio_out(gf66xx_dev->reset_gpio, GPIO_OUT_ZERO); 			 
	msleep(5);  //delay for power to reset  typical:10ms max:50ms
	mt_set_gpio_out(gf66xx_dev->reset_gpio, GPIO_OUT_ONE);
    msleep(delay_ms);
    return 0;
}

