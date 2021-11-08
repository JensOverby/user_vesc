/*
 * turbine.h
 *
 *  Created on: Oct 27, 2021
 *      Author: jens
 */

#ifndef TURBINE_H_
#define TURBINE_H_

//#include "conf_general.h"

typedef struct {
	float V_out_max;
	float V_in_max;
	float V_out_min;
	float I_out_max;
	float tip_speed_ratio;
	float W_losses;
	float W_max;

	// Protect from flash corruption.
	uint16_t crc;
} turbine_configuration;




// Functions
turbine_configuration *turbine_conf_alloc(void);
void turbine_conf_free(turbine_configuration *conf);

const volatile turbine_configuration* turbine_conf_get(void);
void turbine_conf_set(turbine_configuration *conf);

int32_t turbine_conf_serialize(uint8_t *buffer, const turbine_configuration *conf);
bool turbine_conf_deserialize(const uint8_t *buffer, turbine_configuration *conf);
void turbine_conf_defaults(turbine_configuration *conf);

void turbine_conf_read(turbine_configuration *conf);
bool turbine_conf_store(turbine_configuration *conf);


// Functions
void turbine_init(void);
void turbine_printf(const char* format, ...);
void turbine_set_callbacks(void(*interruptNotify)(void), void(*dataHandler)(unsigned char *data, unsigned int len));

#endif /* TURBINE_H_ */
