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
	locked = false;
}

bool aiguille::set_state(a_state position) {
	const relais_t * act_relais;

	// Relais already in the right position ? just return
	if (position == state) return true;
	// if we don't care, skip
	if (position == s_dontcare) return true;

	if (!locked) {
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
		return true;
	}
	return false;
}

bool aiguille::set_state_and_lock(a_state position) {
	const relais_t * act_relais;
	if (locked == true) return false;
	locked = true;
	// Relais already in the right position ? just return
	if (position == state) return true;
	// if we don't care, skip
	if (position == s_dontcare) return true;

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
	return true;
}

void aiguille::unlock(void) {
	locked = false;
}

