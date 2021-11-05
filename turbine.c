/*
 * turbine.c
 *
 *  Created on: Oct 27, 2021
 *      Author: jens
 */

#include "ch.h"
#include "hal.h"
#include "turbine.h"
#include "commands.h"
#include "buffer.h"
#include "crc.h"
#include "eeprom.h"
#include "terminal.h"
#include "mc_interface.h"
#include "utils.h"
#include "timeout.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define TBCONF_SIGNATURE		2215648314
#define TURBINE_CONF_NUM				10
#define EEPROM_BASE_CUSTOM		4000

// Private types
typedef struct {
	volatile bool is_taken;
	turbine_configuration conf;
} turbine_conf_container_t;

// Private variables
static volatile bool m_init_done = false;
static THD_WORKING_AREA(timer_thread_wa, 256);
static THD_FUNCTION(timer_thread, arg);
static volatile bool timer_thd_stop;
static turbine_conf_container_t m_turbine_confs[TURBINE_CONF_NUM] = {{0}};
static int m_turbine_conf_highest = 0;
//static volatile turbine_configuration m_conf;
static turbine_configuration m_conf;


turbine_configuration *turbine_conf_alloc(void) {
	for (int i = 0;i < TURBINE_CONF_NUM;i++) {
		if (i > m_turbine_conf_highest) {
			m_turbine_conf_highest = i;
		}
		if (!m_turbine_confs[i].is_taken) {
			m_turbine_confs[i].is_taken = true;
			return &m_turbine_confs[i].conf;
		}
	}

	m_turbine_conf_highest++;

	return 0;
}

void turbine_conf_free(turbine_configuration *conf) {
	for (int i = 0;i < TURBINE_CONF_NUM;i++) {
		if (&m_turbine_confs[i].conf == conf) {
			m_turbine_confs[i].is_taken = false;
			return;
		}
	}
}

int32_t turbine_conf_serialize(uint8_t *buffer, const turbine_configuration *conf) {
	int32_t ind = 0;
	buffer_append_uint32(buffer, TBCONF_SIGNATURE, &ind);
	buffer_append_float32(buffer, conf->V_out_max, 1000, &ind);
	buffer_append_float32(buffer, conf->V_in_max, 1000, &ind);
	buffer_append_float32(buffer, conf->V_out_min, 1000, &ind);
	buffer_append_float32(buffer, conf->I_out_max, 1000, &ind);
	buffer_append_float32(buffer, conf->tip_speed_ratio, 1000, &ind);
	buffer_append_float32(buffer, conf->W_losses, 1000, &ind);
	buffer_append_float32(buffer, conf->W_max, 1000, &ind);
	return ind;
}

bool turbine_conf_deserialize(const uint8_t *buffer, turbine_configuration *conf) {
	int32_t ind = 0;
	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != TBCONF_SIGNATURE) {
		return false;
	}
	conf->V_out_max = buffer_get_float32(buffer, 1000, &ind);
	conf->V_in_max = buffer_get_float32(buffer, 1000, &ind);
	conf->V_out_min = buffer_get_float32(buffer, 1000, &ind);
	conf->I_out_max = buffer_get_float32(buffer, 1000, &ind);
	conf->tip_speed_ratio = buffer_get_float32(buffer, 1000, &ind);
	conf->W_losses = buffer_get_float32(buffer, 1000, &ind);
	conf->W_max = buffer_get_float32(buffer, 1000, &ind);
	return true;
}

void turbine_conf_defaults(turbine_configuration *conf) {
	conf->V_out_max = 14.4;
	conf->V_in_max = 40;
	conf->V_out_min = 9;
	conf->I_out_max = 20;
	conf->tip_speed_ratio = 5.5;
	conf->W_losses = 35;
	conf->W_max = 500;
}

const volatile turbine_configuration* turbine_conf_get(void) {
	return &m_conf;
}


/**
 * Get turbine_configuration CRC
 *
 * @param conf
 * Pointer to turbine_configuration or NULL for current turbine conf
 *
 * @return
 * CRC16 (with crc field in struct temporarily set to zero).
 */
unsigned turbine_calc_crc(turbine_configuration* conf) {
	if(NULL == conf)
		conf = &m_conf;

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(turbine_configuration));
	conf->crc = crc_old;
	return crc_new;
}

/**
 * Read turbine_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a turbine_configuration struct to write the read configuration to.
 */
void turbine_conf_read(turbine_configuration *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	for (unsigned int i = 0;i < (sizeof(turbine_configuration) / 2);i++) {
		if (EE_ReadVariable(EEPROM_BASE_CUSTOM + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} else {
			is_ok = false;
			break;
		}
	}

	// check CRC
#ifdef TEST_BAD_APP_CRC
	conf->crc++;
#endif
	if(conf->crc != turbine_calc_crc(conf)) {
		is_ok = false;
//		mc_interface_fault_stop(FAULT_CODE_FLASH_CORRUPTION_APP_CFG, false, false);
		fault_data f;
		f.fault = FAULT_CODE_FLASH_CORRUPTION_APP_CFG;
		terminal_add_fault_data(&f);
	}

	// Set the default configuration
	if (!is_ok) {
		turbine_conf_defaults(conf);
	}
}

/**
 * Write turbine_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool turbine_conf_store(turbine_configuration *conf) {
	int motor_old = mc_interface_get_motor_thread();

	mc_interface_select_motor_thread(1);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_lock();

	mc_interface_select_motor_thread(2);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_lock();

	utils_sys_lock_cnt();

	timeout_configure_IWDT_slowest();

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	conf->crc = turbine_calc_crc(conf);

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (unsigned int i = 0;i < (sizeof(turbine_configuration) / 2);i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;

		if (EE_WriteVariable(EEPROM_BASE_CUSTOM + i, var) != FLASH_COMPLETE) {
			is_ok = false;
			break;
		}
	}
	FLASH_Lock();

	timeout_configure_IWDT();

	chThdSleepMilliseconds(100);

	mc_interface_select_motor_thread(1);
	mc_interface_unlock();
	mc_interface_select_motor_thread(2);
	mc_interface_unlock();

	utils_sys_unlock_cnt();

	mc_interface_select_motor_thread(motor_old);

	return is_ok;
}







void turbine_conf_set(turbine_configuration *conf) {
	m_conf = *conf;
	//m_conf.crc = configuration->crc;
	// osv osv
}


static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("Turbine timer");

	for(;;) {
		const float dt = 0.001;

		if (timer_thd_stop) {
			timer_thd_stop = false;
			return;
		}

		/*timer_update(&m_motor_1, dt);
		input_current_offset_measurement();
		run_pid_control_speed(dt, &m_motor_1);*/

		chThdSleepMilliseconds(1);
	}
}

void turbine_printf(const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer + 1, 254, format, arg);
	if (len > 254)
		len = 254;
	print_buffer[len] = '\0';

	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, len+1);
		//commands_send_packet((unsigned char*)print_buffer, (len < 254) ? len + 1 : 255);
	}
}

static void process_packet(unsigned char *data, unsigned int len) {
	int32_t i = 0;
	int int_data = buffer_get_int32(data, &i);
	data[len] = '\0';
	//commands_send_app_data(data, len);

	turbine_printf("hello no.%d world! You typed: %s.", 1, (char*)data);
	turbine_printf("hello no.%d world! You typed: %s.", 2, "NO");
}

void turbine_init(void) {
	m_init_done = false;

	/*comm_usb_serial_init();
	packet_init(send_packet_raw, process_packet, PACKET_HANDLER);

	chMtxObjectInit(&send_mutex);*/

	// Threads
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

	commands_set_app_data_handler(&process_packet);

	m_init_done = true;
}

void turbine_deinit(void) {
	if (!m_init_done) {
		return;
	}

	m_init_done = false;

	timer_thd_stop = true;
	while (timer_thd_stop) {
		chThdSleepMilliseconds(1);
	}
}
