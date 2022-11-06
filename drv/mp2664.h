#ifndef __MP22664_H
#define __MP22664_H

#include <stdint.h>

#define MP22664_ADDR                0x09

#define REG_INPUT_SOURCE_CONTROL    0x00 /* Input Source Control Register */
#define REG_POWER_ON_CONFIG_CONTROL 0x01 /* Power-On Configuration Register */
#define REG_CHARGE_CURRENT_CONTROL  0x02 /* Charge Current Control Register */
#define REG_CHARGE_TERM_CURRENT     0x03 /* Pre-Charge/Termination Current */
#define REG_CHARGE_VOLTAGE_CONTROL  0x04 /* Charge Voltage Control Register */
#define REG_CHARGE_TIME_CONTROL     0x05 /* Charge Termination/Timer Control Register */
#define REG_MISC_OPERATION_CONTROL  0x06 /* Miscellaneous Operation Control Register */
#define REG_SYSTEM_STATUS           0x07 /* System Status Register */
#define REG_FAULT                   0x08 /* Fault Register */

#define CURRENT_LIMIT_SHIFT_BIT 0
#define CURRENT_LIMIT_MASK_BIT	3
typedef enum{
	INPUT_CURRENT_LIMIT_085mA,
	INPUT_CURRENT_LIMIT_130mA,
	INPUT_CURRENT_LIMIT_175mA,
	INPUT_CURRENT_LIMIT_220mA,
	INPUT_CURRENT_LIMIT_265mA,
	INPUT_CURRENT_LIMIT_310mA,
	INPUT_CURRENT_LIMIT_355mA,
	INPUT_CURRENT_LIMIT_455mA
} Current_Limit_t;

#define VOLTAGE_MIN_SHIFT_BIT	3
#define VOLTAGE_MIN_MASK_BIT	4
typedef enum{
	INPUT_MIN_VOLTAGE_3V88,
	INPUT_MIN_VOLTAGE_3V96,
	INPUT_MIN_VOLTAGE_4V04,
	INPUT_MIN_VOLTAGE_4V12,
	INPUT_MIN_VOLTAGE_4V20,
	INPUT_MIN_VOLTAGE_4V28,
	INPUT_MIN_VOLTAGE_4V36,
	INPUT_MIN_VOLTAGE_4V44,
	INPUT_MIN_VOLTAGE_4V52,
	INPUT_MIN_VOLTAGE_4V60,
	INPUT_MIN_VOLTAGE_4V68,
	INPUT_MIN_VOLTAGE_4V76,
	INPUT_MIN_VOLTAGE_4V84,
	INPUT_MIN_VOLTAGE_4V92,
	INPUT_MIN_VOLTAGE_5V00,
	INPUT_MIN_VOLTAGE_5V08
} Voltage_Min_t;

#define BATTERY_THRESHOLD_SHIFT_BIT	0
#define BATTERY_THRESHOLD_MASK_BIT	3
typedef enum{
	BATTERY_UVLO_THRESHOLD_2V4,
	BATTERY_UVLO_THRESHOLD_2V5,
	BATTERY_UVLO_THRESHOLD_2V6,
	BATTERY_UVLO_THRESHOLD_2V7,
	BATTERY_UVLO_THRESHOLD_2V8,
	BATTERY_UVLO_THRESHOLD_2V9,
	BATTERY_UVLO_THRESHOLD_3V0,
	BATTERY_UVLO_THRESHOLD_3V1
} Battery_Threshold_t;

#define CHARGE_CURRENT_SHIFT_BIT	0
#define CHARGE_CURRENT_MASK_BIT	5
typedef enum{
	CHARGE_CURRENT_008mA,
	CHARGE_CURRENT_025mA,
	CHARGE_CURRENT_042mA,
	CHARGE_CURRENT_059mA,
	CHARGE_CURRENT_076mA,
	CHARGE_CURRENT_093mA,
	CHARGE_CURRENT_110mA,
	CHARGE_CURRENT_127mA,
	CHARGE_CURRENT_144mA,
	CHARGE_CURRENT_161mA,
	CHARGE_CURRENT_178mA,
	CHARGE_CURRENT_195mA,
	CHARGE_CURRENT_212mA,
	CHARGE_CURRENT_229mA,
	CHARGE_CURRENT_246mA,
	CHARGE_CURRENT_263mA,
	CHARGE_CURRENT_280mA,
	CHARGE_CURRENT_297mA,
	CHARGE_CURRENT_314mA,
	CHARGE_CURRENT_331mA,
	CHARGE_CURRENT_348mA,
	CHARGE_CURRENT_365mA,
	CHARGE_CURRENT_382mA,
	CHARGE_CURRENT_399mA,
	CHARGE_CURRENT_416mA,
	CHARGE_CURRENT_433mA,
	CHARGE_CURRENT_450mA,
	CHARGE_CURRENT_467mA,
	CHARGE_CURRENT_484mA,
	CHARGE_CURRENT_501mA,
	CHARGE_CURRENT_518mA,
	CHARGE_CURRENT_535mA
} Charge_Current_Setting_t;

#define DISCHARGE_CURRENT_SHIFT_BIT	3
#define DISCHARGE_CURRENT_MASK_BIT	4
typedef enum{
	DISCHARGE_CURRENT_0200mA,
	DISCHARGE_CURRENT_0400mA,
	DISCHARGE_CURRENT_0600mA,
	DISCHARGE_CURRENT_0800mA,
	DISCHARGE_CURRENT_1000mA,
	DISCHARGE_CURRENT_1200mA,
	DISCHARGE_CURRENT_1400mA,
	DISCHARGE_CURRENT_1600mA,
	DISCHARGE_CURRENT_1800mA,
	DISCHARGE_CURRENT_2000mA,
	DISCHARGE_CURRENT_2200mA,
	DISCHARGE_CURRENT_2400mA,
	DISCHARGE_CURRENT_2600mA,
	DISCHARGE_CURRENT_2800mA,
	DISCHARGE_CURRENT_3000mA,
	DISCHARGE_CURRENT_3200mA
} Discharge_Current_Limit_t;

#define PRE_CURRENT_SHIFT_BIT	0
#define PRE_CURRENT_MASK_BIT	2
typedef enum{
	CURRENT_06mA,
	CURRENT_13mA,
	CURRENT_20mA,
	CURRENT_27mA
} Pre_Charge_Current_t;

#define BATTERY_REGULATION_SHIFT_BIT	2
#define BATTERY_REGULATION_MASK_BIT	6
typedef enum{
	BATTERY_REGULATION_3600mV,
	BATTERY_REGULATION_3615mV,
	BATTERY_REGULATION_3630mV,
	BATTERY_REGULATION_3645mV,
	BATTERY_REGULATION_3660mV,
	BATTERY_REGULATION_3675mV,
	BATTERY_REGULATION_3690mV,
	BATTERY_REGULATION_3705mV,
	BATTERY_REGULATION_3720mV,
	BATTERY_REGULATION_3735mV,
	BATTERY_REGULATION_3750mV,
	BATTERY_REGULATION_3765mV,
	BATTERY_REGULATION_3780mV,
	BATTERY_REGULATION_3795mV,
	BATTERY_REGULATION_3810mV,
	BATTERY_REGULATION_3825mV,
	BATTERY_REGULATION_3840mV,
	BATTERY_REGULATION_3855mV,
	BATTERY_REGULATION_3870mV,
	BATTERY_REGULATION_3885mV,
	BATTERY_REGULATION_3900mV,
	BATTERY_REGULATION_3915mV,
	BATTERY_REGULATION_3930mV,
	BATTERY_REGULATION_3945mV,
	BATTERY_REGULATION_3960mV,
	BATTERY_REGULATION_3975mV,
	BATTERY_REGULATION_3990mV,
	BATTERY_REGULATION_4005mV,
	BATTERY_REGULATION_4020mV,
	BATTERY_REGULATION_4035mV,
	BATTERY_REGULATION_4050mV,
	BATTERY_REGULATION_4065mV,
	BATTERY_REGULATION_4080mV,
	BATTERY_REGULATION_4095mV,
	BATTERY_REGULATION_4110mV,
	BATTERY_REGULATION_4125mV,
	BATTERY_REGULATION_4140mV,
	BATTERY_REGULATION_4155mV,
	BATTERY_REGULATION_4170mV,
	BATTERY_REGULATION_4185mV,
	BATTERY_REGULATION_4200mV,
	BATTERY_REGULATION_4215mV,
	BATTERY_REGULATION_4230mV,
	BATTERY_REGULATION_4245mV,
	BATTERY_REGULATION_4260mV,
	BATTERY_REGULATION_4275mV,
	BATTERY_REGULATION_4290mV,
	BATTERY_REGULATION_4305mV,
	BATTERY_REGULATION_4320mV,
	BATTERY_REGULATION_4335mV,
	BATTERY_REGULATION_4350mV,
	BATTERY_REGULATION_4365mV,
	BATTERY_REGULATION_4380mV,
	BATTERY_REGULATION_4395mV,
	BATTERY_REGULATION_4410mV,
	BATTERY_REGULATION_4425mV,
	BATTERY_REGULATION_4440mV,
	BATTERY_REGULATION_4455mV,
	BATTERY_REGULATION_4470mV,
	BATTERY_REGULATION_4485mV,
	BATTERY_REGULATION_4500mV,
	BATTERY_REGULATION_4515mV,
	BATTERY_REGULATION_4530mV,
	BATTERY_REGULATION_4545mV
} Battery_Regulation_Voltage_t;

#define WATCHDOG_TIMER_SHIFT_BIT	4
#define WATCHDOG_TIMER_MASK_BIT	2
typedef enum{
	TIMER_DISABLE,
	TIMER_40s,
	TIMER_80s,
	TIMER_160s
} Watchdog_Timer_t;

#define CHAGRE_TIMER_SHIFT_BIT	1
#define CHAGRE_TIMER_MASK_BIT	2
typedef enum{
	CHAGRE_3HRS,
	CHAGRE_5HRS,
	CHAGRE_8HRS,
	CHAGRE_12HRS
} Fast_Charge_Timer_t;

#define THERMAL_THRESHOLD_SHIFT_BIT	0
#define THERMAL_THRESHOLD_MASK_BIT	2
typedef enum{
	THERMAL_60C,
	THERMAL_80C,
	THERMAL_100C,
	THERMAL_120C
} Thermal_Regulation_Threshold_t;

#define CHARGE_STATUS_SHIFT_BIT	3
#define CHARGE_STATUS_MASK_BIT	2
typedef enum{
	NOT_CHARGING,
	PRE_CHARGE,
	CHARGE,
	CHARGE_DONE
} Charge_Status_t;

void mp2664_read_all_register(void);
void mp2664_reset_register_default(void);
uint8_t mp2664_read_input_source_control_register(void);
uint8_t mp2664_read_power_on_configuration_register(void);
uint8_t mp2664_read_charge_current_control_register(void);
uint8_t mp2664_read_pre_charge_term_current_register(void);
uint8_t mp2664_read_charge_voltage_control_register(void);
uint8_t mp2664_read_charge_termi_timer_control_register(void);
uint8_t mp2664_read_misc_operation_control_register(void);
uint8_t mp2664_read_system_status_register(void);
uint8_t mp2664_read_fault_register(void);

/* Input Source Control Register */
Current_Limit_t mp2664_get_current_input_limit(void);
void mp2664_set_current_input_limit(Current_Limit_t current);
Voltage_Min_t mp2664_get_voltage_input_min(void);
void mp2664_set_voltage_input_min(Voltage_Min_t voltage);
uint8_t mp2664_get_enable_highz(void);
void mp2664_set_enable_highz(uint8_t value);

/* Power-On Configuration Register */
Battery_Threshold_t mp2664_get_battery_uvlo_threshold(void);
void mp2664_set_battery_uvlo_threshold(Battery_Threshold_t current);
uint8_t mp2664_get_charger_configuration(void);
void mp2664_set_charger_configuration(uint8_t value);
uint8_t mp2664_get_i2c_watchdog_reset(void);
void mp2664_set_i2c_watchdog_reset(uint8_t value);
uint8_t mp2664_get_register_reset(void);
void mp2664_set_register_reset(uint8_t value);

/* Charge Current Control Register */
Charge_Current_Setting_t mp2664_get_fast_charge_current_setting(void);
void mp2664_set_fast_charge_current_setting(Charge_Current_Setting_t current);

/* Pre-Charge/Termination Current */
Pre_Charge_Current_t mp2664_get_pre_charge_current(void);
void mp2664_set_pre_charge_current(Pre_Charge_Current_t current);
uint8_t mp2664_get_pcb_otp_enable(void);
void mp2664_set_pcb_otp_enable(uint8_t value);
Discharge_Current_Limit_t mp2664_get_discharge_current_limit(void);
void mp2664_set_discharge_current_limit(Discharge_Current_Limit_t current);

/* Charge Voltage Control Register */
uint8_t mp2664_get_battery_recharge_threshold(void);
void mp2664_set_battery_recharge_threshold(uint8_t value);
uint8_t mp2664_get_fast_charge_threshold(void);
void mp2664_set_fast_charge_threshold(uint8_t value);
Battery_Regulation_Voltage_t mp2664_get_battery_regulation_voltage(void);
void mp2664_set_battery_regulation_voltage(Battery_Regulation_Voltage_t voltage);

/* Charge Termination/Timer Control Register */
uint8_t mp2664_get_term_timer_control(void);
void mp2664_set_term_timer_control(uint8_t value);
Fast_Charge_Timer_t mp2664_get_fast_charge_timer(void);
void mp2664_set_fast_charge_timer(Fast_Charge_Timer_t hours);
uint8_t mp2664_get_safety_timer_setting(void);
void mp2664_set_safety_timer_setting(uint8_t value);
Watchdog_Timer_t mp2664_get_watchdog_timer_limit(void);
void mp2664_set_watchdog_timer_limit(Watchdog_Timer_t value);
uint8_t mp2664_get_termination_setting(void);
void mp2664_set_termination_setting(uint8_t value);

/* Miscellaneous Operation Control Register */
Thermal_Regulation_Threshold_t mp2664_get_thermal_regulation_threshold(void);
void mp2664_set_thermal_regulation_threshold(Thermal_Regulation_Threshold_t value);
uint8_t mp2664_get_enable_ntc(void);
void mp2664_set_enable_ntc(uint8_t value);
uint8_t mp2664_get_fet_disable(void);
void mp2664_set_fet_disable(uint8_t value);
uint8_t mp2664_get_tmr2x_enable(void);
void mp2664_set_tmr2x_enable(uint8_t value);

/* System Status Register */
uint8_t mp2664_get_thermal_status(void);
uint8_t mp2664_get_power_good_status(void);
uint8_t mp2664_get_ppm_status(void);
Charge_Status_t mp2664_get_charge_status(void);
uint8_t mp2664_get_revision_number(void);

/* Fault Register */
uint8_t mp2664_get_ntc_fault0(void);
uint8_t mp2664_get_ntc_fault1(void);
uint8_t mp2664_get_safety_timer(void);
uint8_t mp2664_get_battery_fault(void);
uint8_t mp2664_get_thermal_shutdown(void);
uint8_t mp2664_get_vin_fault(void);
uint8_t mp2664_get_watchdog_fault(void);


void mp2664_test_reg0(void);
void mp2664_test_reg1(void);
void mp2664_test_reg2(void);
void mp2664_test_reg3(void);
void mp2664_test_reg4(void);
void mp2664_test_reg5(void);
void mp2664_test_reg6(void);
void mp2664_test_reg7(void);
void mp2664_test_reg8(void);

#endif /* MP22664_H_ */
