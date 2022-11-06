#include "mp2664.h"
#include "hal_i2c.h"
#include <stdio.h>


uint8_t get_bits(uint8_t reg, uint8_t shift, uint8_t mask)
{
	uint8_t temp = 0;
	switch(mask){
		case 1:
			temp = 0x01;
			break;
		case 2:
			temp = 0x03;
			break;
		case 3:
			temp = 0x07;
			break;
		case 4:
			temp = 0x0f;
			break;
		case 5:
			temp = 0x1f;
			break;
		case 6:
			temp = 0x3f;
			break;
		case 7:
			temp = 0x7f;
			break;
		default:
			temp = 0xff;
			break;
	}
	temp = temp << shift;
	reg = reg & temp;
	reg = reg >> shift;
	return reg;
}

uint8_t modify_reg(uint8_t reg, uint8_t data, uint8_t shift, uint8_t mask)
{
	uint8_t temp = 0;
	switch(mask){
		case 1:
			temp = 0x01;
			break;
		case 2:
			temp = 0x03;
			break;
		case 3:
			temp = 0x07;
			break;
		case 4:
			temp = 0x0f;
			break;
		case 5:
			temp = 0x1f;
			break;
		case 6:
			temp = 0x3f;
			break;
		case 7:
			temp = 0x7f;
			break;
		default:
			temp = 0xff;
			break;
	}

	temp = temp << shift;
	temp = ~temp;
	reg = reg & temp;
	data = data << shift;
	reg = reg | data;
	return reg;
}

void mp2664_init(void)
{
    //i2c_init();
}

char *names[] = {
	"(0x00) Input Source Control:            ",
	"(0x01) Power-On Configuration:          ",
	"(0x02) Charge Current Control:          ",
	"(0x03) Pre-Charge/Termination Current:  ",
	"(0x04) Charge Voltage Control:          ",
	"(0x05) Charge Termination/Timer:        ",
	"(0x06) Miscellaneous Operation Control: ",
	"(0x07) System Status:                   ",
	"(0x08) Fault:                           "
};


void mp2664_read_all_register(void)
{
	uint8_t buffer[9];
	printf("mp2664_read_all_register:\n");
	i2c_read_registers(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, buffer, sizeof(buffer));
	for(uint8_t reg = 0; reg < 9; reg++)
	{
		printf("    %s 0x%02x\n", names[reg], buffer[reg]);
	}
}

void mp2664_reset_register_default(void)
{
	uint8_t defaults[7] = {0x4f, 0x04, 0x0e, 0x4a, 0xa3, 0x4a, 0x4b};
	printf("mp2664_reset_register_default:\n");
	i2c_write_registers(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, defaults, sizeof(defaults));
	printf("    done!\n");
}

uint8_t mp2664_read_input_source_control_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, &data);
	return data;
}

uint8_t mp2664_read_power_on_configuration_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	return data;
}

uint8_t mp2664_read_charge_current_control_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_CURRENT_CONTROL, &data);
	return data;
}

uint8_t mp2664_read_pre_charge_term_current_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, &data);
	return data;
}

uint8_t mp2664_read_charge_voltage_control_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, &data);
	return data;
}

uint8_t mp2664_read_charge_termi_timer_control_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	return data;
}

uint8_t mp2664_read_misc_operation_control_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	return data;
}

uint8_t mp2664_read_system_status_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_SYSTEM_STATUS, &data);
	return data;
}

uint8_t mp2664_read_fault_register(void)
{
	uint8_t data;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	return data;
}

/* Input Source Control Register */
Current_Limit_t mp2664_get_current_input_limit(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, &data);
	result = get_bits(data, CURRENT_LIMIT_SHIFT_BIT, CURRENT_LIMIT_MASK_BIT);
	return result;
}

void mp2664_set_current_input_limit(Current_Limit_t current)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, &data);
	result = modify_reg(data, current, CURRENT_LIMIT_SHIFT_BIT, CURRENT_LIMIT_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, result);
}

Voltage_Min_t mp2664_get_voltage_input_min(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, &data);
	result = get_bits(data, VOLTAGE_MIN_SHIFT_BIT, VOLTAGE_MIN_MASK_BIT);
	return result;
}

void mp2664_set_voltage_input_min(Voltage_Min_t voltage)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, &data);
	result = modify_reg(data, voltage, VOLTAGE_MIN_SHIFT_BIT, VOLTAGE_MIN_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, result);
}

/**
 * return:
 * 0: Disable
 * 1: Enable
 */
uint8_t mp2664_get_enable_highz(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, &data);
	result = get_bits(data, 7, 1);
	return result;
}

/**
 * value:
 * 0: Disable
 * 1: Enable
 */
void mp2664_set_enable_highz(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, &data);
	result = modify_reg(data, value, 7, 1);
	i2c_write_register(MP22664_ADDR, REG_INPUT_SOURCE_CONTROL, result);
}

/* Power-On Configuration Register */
Battery_Threshold_t mp2664_get_battery_uvlo_threshold(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = get_bits(data, BATTERY_THRESHOLD_SHIFT_BIT, BATTERY_THRESHOLD_MASK_BIT);
	return result;
}

void mp2664_set_battery_uvlo_threshold(Battery_Threshold_t current)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = modify_reg(data, current, BATTERY_THRESHOLD_SHIFT_BIT, BATTERY_THRESHOLD_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, result);
}

/**
 * return:
 * 0: Charge enable
 * 1: Charge disable
 */
uint8_t mp2664_get_charger_configuration(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = get_bits(data, 3, 1);
	return result;
}

/**
 * value:
 * 0: Charge enable
 * 1: Charge disable
 */
void mp2664_set_charger_configuration(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = modify_reg(data, value, 3, 1);
	i2c_write_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, result);
}

/**
 * return:
 * 0: Normal
 * 1: Reset
 */
uint8_t mp2664_get_i2c_watchdog_reset(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = get_bits(data, 6, 1);
	return result;
}

/**
 * value:
 * 0: Normal
 * 1: Reset
 */
void mp2664_set_i2c_watchdog_reset(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = modify_reg(data, value, 6, 1);
	i2c_write_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, result);
}

/**
 * return:
 * 0: Keep current setting
 * 1: Reset
 */
uint8_t mp2664_get_register_reset(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = get_bits(data, 7, 1);
	return result;
}

/**
 * value:
 * 0: Keep current setting
 * 1: Reset
 */
void mp2664_set_register_reset(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, &data);
	result = modify_reg(data, value, 7, 1);
	i2c_write_register(MP22664_ADDR, REG_POWER_ON_CONFIG_CONTROL, result);
}

/* Charge Current Control Register */
Charge_Current_Setting_t mp2664_get_fast_charge_current_setting(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_CURRENT_CONTROL, &data);
	result = get_bits(data, CHARGE_CURRENT_SHIFT_BIT, CHARGE_CURRENT_MASK_BIT);
	return result;
}

void mp2664_set_fast_charge_current_setting(Charge_Current_Setting_t current)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_CURRENT_CONTROL, &data);
	result = modify_reg(data, current, CHARGE_CURRENT_SHIFT_BIT, CHARGE_CURRENT_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_CURRENT_CONTROL, result);
}

/* Pre-Charge/Termination Current */
Pre_Charge_Current_t mp2664_get_pre_charge_current(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, &data);
	result = get_bits(data, PRE_CURRENT_SHIFT_BIT, PRE_CURRENT_MASK_BIT);
	return result;
}

void mp2664_set_pre_charge_current(Pre_Charge_Current_t current)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, &data);
	result = modify_reg(data, current, PRE_CURRENT_SHIFT_BIT, PRE_CURRENT_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, result);
}

/**
 * return:
 * 0: Enable
 * 1: Disable
 */
uint8_t mp2664_get_pcb_otp_enable(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, &data);
	result = get_bits(data, 2, 1);
	return result;
}

/**
 * value:
 * 0: Enable
 * 1: Disable
 */
void mp2664_set_pcb_otp_enable(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, &data);
	result = modify_reg(data, value, 2, 1);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, result);
}

Discharge_Current_Limit_t mp2664_get_discharge_current_limit(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, &data);
	result = get_bits(data, DISCHARGE_CURRENT_SHIFT_BIT, DISCHARGE_CURRENT_MASK_BIT);
	return result;
}

void mp2664_set_discharge_current_limit(Discharge_Current_Limit_t current)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, &data);
	result = modify_reg(data, current, DISCHARGE_CURRENT_SHIFT_BIT, DISCHARGE_CURRENT_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TERM_CURRENT, result);
}

/* Charge Voltage Control Register */

/**
 * return:
 * 0: 150mV
 * 1: 300mV
 */
uint8_t mp2664_get_battery_recharge_threshold(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, &data);
	result = get_bits(data, 0, 1);
	return result;
}

/**
 * value:
 * 0: 150mV
 * 1: 300mV
 */
void mp2664_set_battery_recharge_threshold(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, &data);
	result = modify_reg(data, value, 0, 1);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, result);
}

/**
 * return:
 * 0: 2.8V
 * 1: 3.0V
 */
uint8_t mp2664_get_fast_charge_threshold(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, &data);
	result = get_bits(data, 1, 1);
	return result;
}

/**
 * value:
 * 0: 2.8V
 * 1: 3.0V
 */
void mp2664_set_fast_charge_threshold(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, &data);
	result = modify_reg(data, value, 1, 1);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, result);
}

Battery_Regulation_Voltage_t mp2664_get_battery_regulation_voltage(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, &data);
	result = get_bits(data, BATTERY_REGULATION_SHIFT_BIT, BATTERY_REGULATION_MASK_BIT);
	return result;
}

void mp2664_set_battery_regulation_voltage(Battery_Regulation_Voltage_t voltage)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, &data);
	result = modify_reg(data, voltage, BATTERY_REGULATION_SHIFT_BIT, BATTERY_REGULATION_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_VOLTAGE_CONTROL, result);
}

/* Charge Termination/Timer Control Register */
/**
 * return:
 * 0: Disable
 * 1: Enable
 */
uint8_t mp2664_get_term_timer_control(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = get_bits(data, 0, 1);
	return result;
}

/**
 * value:
 * 0: Disable
 * 1: Enable
 */
void mp2664_set_term_timer_control(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = modify_reg(data, value, 0, 1);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, result);
}

Fast_Charge_Timer_t mp2664_get_fast_charge_timer(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = get_bits(data, CHAGRE_TIMER_SHIFT_BIT, CHAGRE_TIMER_MASK_BIT);
	return result;
}

void mp2664_set_fast_charge_timer(Fast_Charge_Timer_t hours)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = modify_reg(data, hours, CHAGRE_TIMER_SHIFT_BIT, CHAGRE_TIMER_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, result);
}

/**
 * return:
 * 0: Disable
 * 1: Enable
 */
uint8_t mp2664_get_safety_timer_setting(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = get_bits(data, 3, 1);
	return result;
}

/**
 * value:
 * 0: Disable
 * 1: Enable
 */
void mp2664_set_safety_timer_setting(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = modify_reg(data, value, 3, 1);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, result);
}

Watchdog_Timer_t mp2664_get_watchdog_timer_limit(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = get_bits(data, WATCHDOG_TIMER_SHIFT_BIT, WATCHDOG_TIMER_MASK_BIT);
	return result;
}

void mp2664_set_watchdog_timer_limit(Watchdog_Timer_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = modify_reg(data, value, WATCHDOG_TIMER_SHIFT_BIT, WATCHDOG_TIMER_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, result);
}

/**
 * return:
 * 0: Disable
 * 1: Enable
 */
uint8_t mp2664_get_termination_setting(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = get_bits(data, 6, 1);
	return result;
}

/**
 * value:
 * 0: Disable
 * 1: Enable
 */
void mp2664_set_termination_setting(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, &data);
	result = modify_reg(data, value, 6, 1);
	i2c_write_register(MP22664_ADDR, REG_CHARGE_TIME_CONTROL, result);
}

/* Miscellaneous Operation Control Register */
Thermal_Regulation_Threshold_t mp2664_get_thermal_regulation_threshold(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = get_bits(data, THERMAL_THRESHOLD_SHIFT_BIT, THERMAL_THRESHOLD_MASK_BIT);
	return result;
}

void mp2664_set_thermal_regulation_threshold(Thermal_Regulation_Threshold_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = modify_reg(data, value, THERMAL_THRESHOLD_SHIFT_BIT, THERMAL_THRESHOLD_MASK_BIT);
	i2c_write_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, result);
}

/**
 * return:
 * 0: Disable
 * 1: Enable
 */
uint8_t mp2664_get_enable_ntc(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = get_bits(data, 3, 1);
	return result;
}

/**
 * value:
 * 0: Disable
 * 1: Enable
 */
void mp2664_set_enable_ntc(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = modify_reg(data, value, 3, 1);
	i2c_write_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, result);
}

/**
 * return:
 * 0: Enable
 * 1: Turn off
 */
uint8_t mp2664_get_fet_disable(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = get_bits(data, 5, 1);
	return result;
}

/**
 * value:
 * 0: Enable
 * 1: Turn off
 */
void mp2664_set_fet_disable(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = modify_reg(data, value, 5, 1);
	i2c_write_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, result);
}

/**
 * return:
 * 0: Disable 2X extended safety timer during PPM
 * 1: Enable 2X extended safety timer during PPM
 */
uint8_t mp2664_get_tmr2x_enable(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = get_bits(data, 6, 1);
	return result;
}

/**
 * value:
 * 0: Disable 2X extended safety timer during PPM
 * 1: Enable 2X extended safety timer during PPM
 */
void mp2664_set_tmr2x_enable(uint8_t value)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, &data);
	result = modify_reg(data, value, 6, 1);
	i2c_write_register(MP22664_ADDR, REG_MISC_OPERATION_CONTROL, result);
}

/* System Status Register */
/**
 * return:
 * 0: No thermal regulation
 * 1: In thermal regulation
 */
uint8_t mp2664_get_thermal_status(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_SYSTEM_STATUS, &data);
	result = get_bits(data, 0, 1);
	return result;
}

/**
 * return:
 * 0: Power fail
 * 1: Power good
 */
uint8_t mp2664_get_power_good_status(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_SYSTEM_STATUS, &data);
	result = get_bits(data, 1, 1);
	return result;
}

/**
 * return:
 * 0: No PPM
 * 1: In PPM
 */
uint8_t mp2664_get_ppm_status(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_SYSTEM_STATUS, &data);
	result = get_bits(data, 2, 1);
	return result;
}

Charge_Status_t mp2664_get_charge_status(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_SYSTEM_STATUS, &data);
	result = get_bits(data, CHARGE_STATUS_SHIFT_BIT, CHARGE_STATUS_MASK_BIT);
	return result;
}


uint8_t mp2664_get_revision_number(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_SYSTEM_STATUS, &data);
	result = get_bits(data, 5, 2);
	return result;
}

/* Fault Register */
/**
 * return:
 * 0: Normal
 * 1: Cold
 */
uint8_t mp2664_get_ntc_fault0(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	result = get_bits(data, 0, 1);
	return result;
}

/**
 * return:
 * 0: Normal
 * 1: Cold
 */
uint8_t mp2664_get_ntc_fault1(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	result = get_bits(data, 1, 1);
	return result;
}

/**
 * return:
 * 0: Normal
 * 1: Safety timer expiration
 */
uint8_t mp2664_get_safety_timer(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	result = get_bits(data, 2, 1);
	return result;
}

/**
 * return:
 * 0: Normal
 * 1: Battery OVP
 */
uint8_t mp2664_get_battery_fault(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	result = get_bits(data, 3, 1);
	return result;
}

/**
 * return:
 * 0: Normal
 * 1: Thermal shutdown
 */
uint8_t mp2664_get_thermal_shutdown(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	result = get_bits(data, 4, 1);
	return result;
}

/**
 * return:
 * 0: Normal
 * 1:  Input fault (OVP or bad source)
 */
uint8_t mp2664_get_vin_fault(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	result = get_bits(data, 5, 1);
	return result;
}

/**
 * return:
 * 0: Normal
 * 1: Watchdog timer expiration
 */
uint8_t mp2664_get_watchdog_fault(void)
{
	uint8_t data;
	uint8_t result;
	i2c_read_register(MP22664_ADDR, REG_FAULT, &data);
	result = get_bits(data, 6, 1);
	return result;
}

void mp2664_test_reg0(void)
{
	uint8_t data = 0 ;
	mp2664_set_current_input_limit(INPUT_CURRENT_LIMIT_310mA);
	data = mp2664_get_current_input_limit();
	printf("current_input_limit: 0x%02x\n", data);

	mp2664_set_voltage_input_min(INPUT_MIN_VOLTAGE_4V76);
	data = mp2664_get_voltage_input_min();
	printf("voltage_input_min: 0x%02x\n", data);

	mp2664_set_enable_highz(1);
	data = mp2664_get_enable_highz();
	printf("enable_highz: 0x%02x\n", data);
}

void mp2664_test_reg1(void)
{
	uint8_t data = 0 ;
	mp2664_set_battery_uvlo_threshold(BATTERY_UVLO_THRESHOLD_3V0);
	data = mp2664_get_battery_uvlo_threshold();
	printf("battery_uvlo_threshold: 0x%02x\n", data);

	mp2664_set_charger_configuration(1);
	data = mp2664_get_charger_configuration();
	printf("charger_configuration: 0x%02x\n", data);

	mp2664_set_i2c_watchdog_reset(1);
	data = mp2664_get_i2c_watchdog_reset();
	printf("i2c_watchdog_reset: 0x%02x\n", data);

	mp2664_set_register_reset(1);
	data = mp2664_get_register_reset();
	printf("register_reset: 0x%02x\n", data);
}

void mp2664_test_reg2(void)
{
	uint8_t data = 0 ;
	mp2664_set_fast_charge_current_setting(CHARGE_CURRENT_535mA);
	data = mp2664_get_fast_charge_current_setting();
	printf("fast_charge_current_setting: 0x%02x\n", data);
}

void mp2664_test_reg3(void)
{
	uint8_t data = 0 ;
	mp2664_set_pre_charge_current(CURRENT_27mA);
	data = mp2664_get_pre_charge_current();
	printf("pre_charge_current: 0x%02x\n", data);

	mp2664_set_pcb_otp_enable(1);
	data = mp2664_get_pcb_otp_enable();
	printf("pcb_otp_enable: 0x%02x\n", data);

	mp2664_set_discharge_current_limit(DISCHARGE_CURRENT_3200mA);
	data = mp2664_get_discharge_current_limit();
	printf("discharge_current_limit: 0x%02x\n", data);
}

void mp2664_test_reg4(void)
{
	uint8_t data = 0 ;
	mp2664_set_battery_recharge_threshold(0);
	data = mp2664_get_battery_recharge_threshold();
	printf("battery_recharge_threshold: 0x%02x\n", data);

	mp2664_set_fast_charge_threshold(0);
	data = mp2664_get_fast_charge_threshold();
	printf("fast_charge_threshold: 0x%02x\n", data);

	mp2664_set_battery_regulation_voltage(BATTERY_REGULATION_4545mV);
	data = mp2664_get_battery_regulation_voltage();
	printf("battery_regulation_voltage: 0x%02x\n", data);
}

void mp2664_test_reg5(void)
{
	uint8_t data = 0 ;
	mp2664_set_term_timer_control(1);
	data = mp2664_get_term_timer_control();
	printf("term_timer_control: 0x%02x\n", data);

	mp2664_set_fast_charge_timer(CHAGRE_3HRS);
	data = mp2664_get_fast_charge_timer();
	printf("fast_charge_timer: 0x%02x\n", data);

	mp2664_set_safety_timer_setting(0);
	data = mp2664_get_safety_timer_setting();
	printf("safety_timer_setting: 0x%02x\n", data);

	mp2664_set_watchdog_timer_limit(CHAGRE_12HRS);
	data = mp2664_get_watchdog_timer_limit();
	printf("watchdog_timer_limit: 0x%02x\n", data);

	mp2664_set_termination_setting(0);
	data = mp2664_get_termination_setting();
	printf("termination_setting: 0x%02x\n", data);
}

void mp2664_test_reg6(void)
{
	uint8_t data = 0 ;
	mp2664_set_thermal_regulation_threshold(THERMAL_60C);
	data = mp2664_get_thermal_regulation_threshold();
	printf("thermal_regulation_threshold: 0x%02x\n", data);

	mp2664_set_enable_ntc(0);
	data = mp2664_get_enable_ntc();
	printf("get_enable_ntc: 0x%02x\n", data);

	mp2664_set_fet_disable(1);
	data = mp2664_get_fet_disable();
	printf("get_fet_disable: 0x%02x\n", data);

	mp2664_set_tmr2x_enable(0);
	data = mp2664_get_tmr2x_enable();
	printf("tmr2x_enable: 0x%02x\n", data);
}

void mp2664_test_reg7(void)
{
	uint8_t data = 0 ;
	data = mp2664_get_thermal_status();
	printf("thermal_status: 0x%02x\n", data);

	data = mp2664_get_power_good_status();
	printf("power_good_status: 0x%02x\n", data);

	data = mp2664_get_ppm_status();
	printf("ppm_status: 0x%02x\n", data);

	data = mp2664_get_charge_status();
	printf("charge_status: 0x%02x\n", data);

	data = mp2664_get_revision_number();
	printf("revision_number: 0x%02x\n", data);
}

void mp2664_test_reg8(void)
{
	uint8_t data = 0 ;
	data = mp2664_get_ntc_fault0();
	printf("ntc_fault0: 0x%02x\n", data);

	data = mp2664_get_ntc_fault1();
	printf("ntc_fault1: 0x%02x\n", data);

	data = mp2664_get_safety_timer();
	printf("safety_timer: 0x%02x\n", data);

	data = mp2664_get_battery_fault();
	printf("battery_fault: 0x%02x\n", data);

	data = mp2664_get_thermal_shutdown();
	printf("thermal_shutdown: 0x%02x\n", data);

	data = mp2664_get_vin_fault();
	printf("vin_fault: 0x%02x\n", data);

	data = mp2664_get_watchdog_fault();
	printf("watchdog_fault: 0x%02x\n", data);
}

