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
	Io_Port * temp_io;
	temp_io = (Io_Port*)pgm_read_word(&id->drive);
	if (pgm_read_byte(&id->inverted) != 0)
		temp_io->clear_mask(pgm_read_byte(&id->mask));
	else
		temp_io->set_mask(pgm_read_byte(&id->mask));
}
void relay_off(const relais_t * id){
	Io_Port * temp_io;
	temp_io = (Io_Port*)pgm_read_word(&id->drive);
	if (pgm_read_byte(&id->inverted) != 0)
		temp_io->set_mask(pgm_read_byte(&id->mask));
	else
		temp_io->clear_mask(pgm_read_byte(&id->mask));
}

