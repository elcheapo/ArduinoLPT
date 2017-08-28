/*
 * aiguillage.cpp
 *
 *  Created on: Nov 3, 2013
 *      Author: francois
 */


#include "Arduino.h"
#include "io_port.h"
#include "i2c_port.h"
#include "relay.h"
#include "aiguillage.h"

#define DEBUG
//#undef DEBUG

// use  150 ms pulse for PECO

aiguille::aiguille(const relais_t * _act_droit, const relais_t * _act_devie, a_type _type) {
	act_droit = _act_droit;
	act_devie = _act_devie;
	type = _type;
	state = s_unknown;
}

void aiguille::set_state(a_state position) {
	uint8_t i, count=0;
	const relais_t * act_relais;

	// Relais already in the right position ? just return
	if (position == state) return;
	// if we don't care, skip
	if (position == s_dontcare) return;

	if(position == s_droit) {
		act_relais = act_droit;
		state = s_droit;
	} else {
		act_relais = act_devie;
		state = s_devie;
	}
	relay_on(act_relais);
	delay(150);
	relay_off(act_relais);
	delay(50);
}


