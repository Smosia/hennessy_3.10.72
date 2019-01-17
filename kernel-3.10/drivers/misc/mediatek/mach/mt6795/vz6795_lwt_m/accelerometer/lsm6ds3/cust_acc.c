#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>


/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw_lsm = {
    .i2c_num = 3,
    #ifdef CONFIG_CM865_MAINBOARD
    .direction = 2,
    #else
    .direction = 0,
    #endif
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw_lsm(void) 
{
    return &cust_acc_hw_lsm;
}