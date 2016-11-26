#ifndef _BATTERY_METER_H
#define _BATTERY_METER_H

#include <mach/mt_typedefs.h>
#include "cust_battery_meter.h"
#include <mach/charging.h>

/* ============================================================ */
/* define */
/* ============================================================ */
#define FG_CURRENT_AVERAGE_SIZE 30

/* ============================================================ */
/* ENUM */
/* ============================================================ */

/* ============================================================ */
/* structure */
/* ============================================================ */

#define FGD_NL_MSG_T_HDR_LEN 12
#define FGD_NL_MSG_MAX_LEN 9200

struct fgd_nl_msg_t {
	unsigned int fgd_cmd;
	unsigned int fgd_data_len;
	unsigned int fgd_ret_data_len;
	char fgd_data[FGD_NL_MSG_MAX_LEN];
};

enum {
	FG_MAIN = 1,
	FG_SUSPEND = 2,
	FG_RESUME = 4,
	FG_CHARGER = 8,
	FG_INIT = 16
};

enum {
	HW_FG,
	SW_FG,
	AUXADC
};


/* ============================================================ */
/* typedef */
/* ============================================================ */
typedef struct {
	INT32 BatteryTemp;
	INT32 TemperatureR;
} BATT_TEMPERATURE;

#if !defined(CONFIG_MTK_HAFG_20)
struct battery_meter_custom_data {

	/* mt_battery_meter.h */

	/* ADC resister */
	int r_bat_sense;
	int r_i_sense;
	int r_charger_1;
	int r_charger_2;

	int temperature_t0;
	int temperature_t1;
	int temperature_t2;
	int temperature_t3;
	int temperature_t;

	int fg_meter_resistance;

	/* Qmax for battery  */
	int q_max_pos_50;
	int q_max_pos_25;
	int q_max_pos_0;
	int q_max_neg_10;
	int q_max_pos_50_h_current;
	int q_max_pos_25_h_current;
	int q_max_pos_0_h_current;
	int q_max_neg_10_h_current;

	int oam_d5;		/* 1 : D5,   0: D2 */

	int change_tracking_point;
	int cust_tracking_point;
	int cust_r_sense;
	int cust_hw_cc;
	int aging_tuning_value;
	int cust_r_fg_offset;
	int ocv_board_compesate;
	int r_fg_board_base;
	int r_fg_board_slope;
	int car_tune_value;

	/* HW Fuel gague  */
	int current_detect_r_fg;
	int minerroroffset;
	int fg_vbat_average_size;
	int r_fg_value;
	int cust_poweron_delta_capacity_tolrance;
	int cust_poweron_low_capacity_tolrance;
	int cust_poweron_max_vbat_tolrance;
	int cust_poweron_delta_vbat_tolrance;
	int cust_poweron_delta_hw_sw_ocv_capacity_tolrance;

	int fixed_tbat_25;

	/* Dynamic change wake up period of battery thread when suspend */
	int vbat_normal_wakeup;
	int vbat_low_power_wakeup;
	int normal_wakeup_period;
	int low_power_wakeup_period;
	int close_poweroff_wakeup_period;


	/* mt_battery_meter.h */
	int bat_ntc;
	int rbat_pull_up_r;
	int rbat_pull_up_volt;

};
#else

struct battery_meter_custom_data {

	/* cust_battery_meter.h */
	int soc_flow;

	int hw_fg_force_use_sw_ocv;

	/* ADC resister */
	int r_bat_sense;
	int r_i_sense;
	int r_charger_1;
	int r_charger_2;

	int temperature_t0;
	int temperature_t1;
	int temperature_t2;
	int temperature_t3;
	int temperature_t;

	int fg_meter_resistance;

	/* Qmax for battery  */
	int q_max_pos_50;
	int q_max_pos_25;
	int q_max_pos_0;
	int q_max_neg_10;
	int q_max_pos_50_h_current;
	int q_max_pos_25_h_current;
	int q_max_pos_0_h_current;
	int q_max_neg_10_h_current;

	int oam_d5;	/* 1 : D5,   0: D2 */

	int change_tracking_point;
	int cust_tracking_point;
	int cust_r_sense;
	int cust_hw_cc;
	int aging_tuning_value;
	int cust_r_fg_offset;
	int ocv_board_compesate;
	int r_fg_board_base;
	int r_fg_board_slope;
	int car_tune_value;

	/* HW Fuel gague  */
	int current_detect_r_fg;
	int minerroroffset;
	int fg_vbat_average_size;
	int r_fg_value;
	int difference_hwocv_rtc;
	int difference_hwocv_swocv;
	int difference_swocv_rtc;
	int max_swocv;

	int max_hwocv;
	int max_vbat;
	int difference_hwocv_vbat;

	int suspend_current_threshold;
	int ocv_check_time;
	int shutdown_system_voltage;
	int recharge_tolerance;
	int fixed_tbat_25;

	int batterypseudo100;
	int batterypseudo1;

	/* Dynamic change wake up period of battery thread when suspend*/
	int vbat_normal_wakeup;
	int vbat_low_power_wakeup;
	int normal_wakeup_period;
	int low_power_wakeup_period;
	int close_poweroff_wakeup_period;

	int init_soc_by_sw_soc;
	int sync_ui_soc_imm;                  /*3. ui soc sync to fg soc immediately*/
	int mtk_enable_aging_algorithm; /*6. q_max aging algorithm*/
	int md_sleep_current_check;     /*5. gauge adjust by ocv 9. md sleep current check*/
	int q_max_by_current;           /*7. qmax variant by current loading.*/
	int q_max_sys_voltage;		/*8. qmax variant by sys voltage.*/

	int shutdown_gauge0;
	int shutdown_gauge1_xmins;
	int shutdown_gauge1_mins;

	int min_charging_smooth_time;

	/* SW Fuel gauge */
	int apsleep_battery_voltage_compensate;


};

#endif

typedef enum {
	FG_DAEMON_CMD_GET_INIT_FLAG,
	FG_DAEMON_CMD_GET_SOC,
	FG_DAEMON_CMD_GET_DOD0,
	FG_DAEMON_CMD_GET_DOD1,
	FG_DAEMON_CMD_GET_HW_OCV,
	FG_DAEMON_CMD_GET_HW_FG_INIT_CURRENT,
	FG_DAEMON_CMD_GET_HW_FG_CURRENT,
	FG_DAEMON_CMD_GET_HW_FG_INIT_CURRENT_SIGN,
	FG_DAEMON_CMD_GET_HW_FG_CURRENT_SIGN,
	FG_DAEMON_CMD_GET_HW_FG_CAR_ACT,
	FG_DAEMON_CMD_GET_TEMPERTURE,
	FG_DAEMON_CMD_DUMP_REGISTER,
	FG_DAEMON_CMD_CHARGING_ENABLE,
	FG_DAEMON_CMD_GET_BATTERY_INIT_VOLTAGE,
	FG_DAEMON_CMD_GET_BATTERY_VOLTAGE,
	FG_DAEMON_CMD_FGADC_RESET,
	FG_DAEMON_CMD_GET_BATTERY_PLUG_STATUS,
	FG_DAEMON_CMD_GET_RTC_SPARE_FG_VALUE,
	FG_DAEMON_CMD_IS_CHARGER_EXIST,
	FG_DAEMON_CMD_IS_BATTERY_FULL,	/* bat_is_battery_full, */
	FG_DAEMON_CMD_SET_BATTERY_FULL,	/* bat_set_battery_full, */
	FG_DAEMON_CMD_SET_RTC,	/* set RTC, */
	FG_DAEMON_CMD_SET_POWEROFF,	/* set Poweroff, */
	FG_DAEMON_CMD_IS_KPOC,	/* is KPOC, */
	FG_DAEMON_CMD_GET_BOOT_REASON,	/* g_boot_reason, */
	FG_DAEMON_CMD_GET_CHARGING_CURRENT,
	FG_DAEMON_CMD_GET_CHARGER_VOLTAGE,
	FG_DAEMON_CMD_GET_SHUTDOWN_COND,
	FG_DAEMON_CMD_GET_CUSTOM_SETTING,
	FG_DAEMON_CMD_GET_UI_SOC,
	FG_DAEMON_CMD_GET_CV_VALUE,
	FG_DAEMON_CMD_GET_DURATION_TIME,
	FG_DAEMON_CMD_GET_TRACKING_TIME,
	FG_DAEMON_CMD_GET_CURRENT_TH,
	FG_DAEMON_CMD_GET_CHECK_TIME,
	FG_DAEMON_CMD_GET_DIFFERENCE_VOLTAGE_UPDATE,
	FG_DAEMON_CMD_GET_AGING1_LOAD_SOC,
	FG_DAEMON_CMD_GET_AGING1_UPDATE_SOC,
	FG_DAEMON_CMD_GET_SHUTDOWN_SYSTEM_VOLTAGE,
	FG_DAEMON_CMD_GET_CHARGE_TRACKING_TIME,
	FG_DAEMON_CMD_GET_DISCHARGE_TRACKING_TIME,
	FG_DAEMON_CMD_GET_SHUTDOWN_GAUGE0,
	FG_DAEMON_CMD_GET_SHUTDOWN_GAUGE1_XMINS,
	FG_DAEMON_CMD_GET_SHUTDOWN_GAUGE1_MINS,
	FG_DAEMON_CMD_SET_SUSPEND_TIME,
	FG_DAEMON_CMD_SET_WAKEUP_SMOOTH_TIME,
	FG_DAEMON_CMD_SET_IS_CHARGING,
	FG_DAEMON_CMD_SET_RBAT,
	FG_DAEMON_CMD_SET_SWOCV,
	FG_DAEMON_CMD_SET_DOD0,
	FG_DAEMON_CMD_SET_DOD1,
	FG_DAEMON_CMD_SET_QMAX,
	FG_DAEMON_CMD_SET_SOC,
	FG_DAEMON_CMD_SET_UI_SOC,
	FG_DAEMON_CMD_SET_UI_SOC2,
	FG_DAEMON_CMD_SET_INIT_FLAG,
	FG_DAEMON_CMD_SET_DAEMON_PID,
	FG_DAEMON_CMD_NOTIFY_DAEMON,
	FG_DAEMON_CMD_CHECK_FG_DAEMON_VERSION,
	FG_DAEMON_CMD_SET_OAM_V_OCV,
	FG_DAEMON_CMD_SET_OAM_R,
	FG_DAEMON_CMD_GET_SUSPEND_TIME,
	FG_DAEMON_CMD_GET_SUSPEND_CAR,
	FG_DAEMON_CMD_IS_HW_OCV_UPDATE,

	FG_DAEMON_CMD_FROM_USER_NUMBER
} FG_DAEMON_CTRL_CMD_FROM_USER;


/* ============================================================ */
/* External Variables */
/* ============================================================ */
extern struct battery_custom_data batt_cust_data;
extern struct battery_meter_custom_data batt_meter_cust_data;
extern char *saved_command_line;

extern kal_uint32 battery_tracking_time;
extern kal_uint32 wake_up_smooth_time;

extern bool bat_spm_timeout;

extern unsigned int sleep_total_time;
extern BATTERY_VOLTAGE_ENUM cv_voltage;
extern kal_bool g_battery_soc_ready;


/* ============================================================ */
/* External function */
/* ============================================================ */
extern kal_int32 battery_meter_get_battery_voltage(kal_bool update);
extern kal_int32 battery_meter_get_charging_current_imm(void);
extern kal_int32 battery_meter_get_charging_current(void);
extern kal_int32 battery_meter_get_battery_current(void);
extern kal_bool battery_meter_get_battery_current_sign(void);
extern kal_int32 battery_meter_get_car(void);
extern kal_int32 battery_meter_get_battery_temperature(void);
extern kal_int32 battery_meter_get_charger_voltage(void);
extern kal_int32 battery_meter_get_battery_percentage(void);
extern kal_int32 battery_meter_initial(void);
extern kal_int32 battery_meter_reset(void);
extern kal_int32 battery_meter_sync(kal_int32 bat_i_sense_offset);

extern kal_int32 battery_meter_get_battery_zcv(void);
extern kal_int32 battery_meter_get_battery_nPercent_zcv(void);	/* 15% zcv,  15% can be customized */
extern kal_int32 battery_meter_get_battery_nPercent_UI_SOC(void);	/* tracking point */

extern kal_int32 battery_meter_get_tempR(kal_int32 dwVolt);
extern kal_int32 battery_meter_get_tempV(void);
extern kal_int32 battery_meter_get_VSense(void);	/* isense voltage */

extern int wakeup_fg_algo(int flow_state);
#ifdef MTK_MULTI_BAT_PROFILE_SUPPORT
extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
#endif

extern void bat_update_thread_wakeup(void);
#ifdef CONFIG_MTK_MULTI_BAT_PROFILE_SUPPORT
extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
#endif



#endif				/* #ifndef _BATTERY_METER_H */
