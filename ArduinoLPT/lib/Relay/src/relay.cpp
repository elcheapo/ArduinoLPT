/*
 * relay.cpp
 *
 *  Created on: Nov 3, 2013
 *      Author: francois
 */

#include "Arduino.h"

#include "io_port.h"
#include "i2c_port.h"
#include "relay.h"



void relay_on(const relais_t * id) {
	t_io temp_io;
	// get relais from flash
	get_t_io(temp_io, &id->port);
	if (pgm_read_byte(&id->inverted) != 0)
		temp_io.port->clear_mask(temp_io.mask);
	else
		temp_io.port->set_mask(temp_io.mask);
}
void relay_off(const relais_t * id){
	t_io temp_io;
	// get relais from flash
	get_t_io(temp_io, &id->port);
	if (pgm_read_byte(&id->inverted) != 0)
		temp_io.port->set_mask(temp_io.mask);
	else
		temp_io.port->clear_mask(temp_io.mask);
}
