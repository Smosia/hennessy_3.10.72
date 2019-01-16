#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <mach/upmu_common.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 3,
    .i2c_addr	= {0x60,0,0,0},
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    /* MTK: modified to support AAL */
    .als_level  = {0, 419, 952, 1557, 3673, 9092, 9177, 15211, 26458, 32570, 38895, 53287, 65535, 65535, 65535}, 
    .als_value  = {0, 133, 303, 503, 1003, 2004, 3008, 5001, 8009, 10000, 12000, 16000, 20000, 20000, 20000}, 
    .ps_threshold_high = 32,
    .ps_threshold_low = 22,
    .is_batch_supported_ps = false,
    .is_batch_supported_als = false,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

