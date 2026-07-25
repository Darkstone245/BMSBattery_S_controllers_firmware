/*
 * Compatibility support for LishuiFOC "No.2" UART2 display protocol
 */

#include <stdint.h>
#include "stm8s.h"
#include "display_no2.h"
#include "config.h"
#include "uart.h"
#include "ACAcontrollerState.h"
#include "ACAeeprom.h"
#include "interrupts.h"
#include "brake.h"

#ifdef DISPLAY_TYPE_NO2

#define NO2_RX_FRAME_SIZE 20
#define NO2_TX_FRAME_SIZE 14

static uint8_t ui8_no2_rx_buffer[NO2_RX_FRAME_SIZE];
static uint8_t ui8_no2_rx_counter = 0;
static uint8_t ui8_no2_tx_buffer[NO2_TX_FRAME_SIZE] = {0x02, 0x0E, 0x01, 0x00, 0x80, 0x00, 0x00, 0x2C, 0x00, 0xF9, 0x00, 0x00, 0xFF, 0x00};

static uint8_t no2_checksum(const uint8_t *buffer, uint8_t length) {
	uint8_t i;
	uint8_t checksum = 0;

	for (i = 0; i < (uint8_t) (length - 1); i++) {
		checksum ^= buffer[i];
	}

	return checksum;
}

static uint16_t no2_get_wheel_time_ms(void) {
	uint16_t wheel_period_ms;

	if (((ui16_aca_flags & EXTERNAL_SPEED_SENSOR) == EXTERNAL_SPEED_SENSOR)) {
		if (ui16_time_ticks_between_speed_interrupt > 65000) {
			wheel_period_ms = 64000;
		} else {
			wheel_period_ms = (uint16_t) ((float) ui16_time_ticks_between_speed_interrupt / ((float) ui16_pwm_cycles_second / 1000.0));
		}
	} else {
		if (ui32_erps_filtered == 0) {
			wheel_period_ms = 64000;
		} else {
			wheel_period_ms = (uint16_t) (1000.0 * (float) ui8_gear_ratio / (float) ui32_erps_filtered);
		}
	}

	return wheel_period_ms;
}

static uint16_t no2_get_current_deci_amp(void) {
	int16_t adc_delta = (int16_t) ui16_BatteryCurrent - (int16_t) ui16_current_cal_b + 1;

	if ((adc_delta <= 0) || (ui8_current_cal_a == 0)) {
		return 0;
	}

	return (uint16_t) ((((uint16_t) adc_delta) * 100U) / ui8_current_cal_a);
}

static void no2_apply_rx_values(void) {
	uint8_t assist_level = ui8_no2_rx_buffer[4] & 0x0F;
	uint8_t headlight_on = (ui8_no2_rx_buffer[5] >> 5) & 0x01;
	uint8_t push_assist = (ui8_no2_rx_buffer[5] >> 1) & 0x01;
	uint8_t speed_limit_kph = ui8_no2_rx_buffer[12];

	ui8_assistlevel_global = assist_level + 80;
	ui8_walk_assist = push_assist;
	light_stat = (light_stat & ~128) | (headlight_on << 7);

	if (speed_limit_kph != ui8_speedlimit_kph) {
		ui8_speedlimit_kph = speed_limit_kph;
		eeprom_write(OFFSET_MAX_SPEED_DEFAULT, speed_limit_kph);
	}
}

static void no2_prepare_tx_values(void) {
	uint16_t wheel_time_ms = no2_get_wheel_time_ms();
	uint16_t battery_current_deci_amp = no2_get_current_deci_amp();
	uint8_t i;

	ui8_no2_tx_buffer[3] = 0; // error code
	ui8_no2_tx_buffer[4] = (uint8_t) (brake_is_set() ? (1U << 5) : 0U);
	ui8_no2_tx_buffer[6] = (uint8_t) (battery_current_deci_amp >> 8);
	ui8_no2_tx_buffer[7] = (uint8_t) (battery_current_deci_amp & 0xFF);
	ui8_no2_tx_buffer[8] = (uint8_t) (wheel_time_ms >> 8);
	ui8_no2_tx_buffer[9] = (uint8_t) (wheel_time_ms & 0xFF);
	ui8_no2_tx_buffer[13] = no2_checksum(ui8_no2_tx_buffer, NO2_TX_FRAME_SIZE);

	for (i = 0; i < NO2_TX_FRAME_SIZE; i++) {
		uart_put_buffered(ui8_no2_tx_buffer[i]);
	}
}

void display_init(void) {
	// noop; UART is configured globally
}

void display_update(void) {
	uart_fill_rx_packet_buffer(ui8_no2_rx_buffer, NO2_RX_FRAME_SIZE, &ui8_no2_rx_counter);

	if (ui8_no2_rx_counter < NO2_RX_FRAME_SIZE) {
		return;
	}

	ui8_no2_rx_counter = 0;

	if (ui8_no2_rx_buffer[0] != 0x02) {
		return;
	}

	if (no2_checksum(ui8_no2_rx_buffer, NO2_RX_FRAME_SIZE) != ui8_no2_rx_buffer[NO2_RX_FRAME_SIZE - 1]) {
		return;
	}

	no2_apply_rx_values();
	no2_prepare_tx_values();
}

#endif
