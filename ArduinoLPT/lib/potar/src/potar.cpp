/*
 * potar.cpp
 *
 *  Created on: 23 ao�t 2017
 *      Author: florrain
 */

#include <Arduino.h>
#include "potar.h"

Potar::Potar (uint8_t _pin) {
	pin = _pin;
	value = 0;
}
Potar::Potar (void) {
	value = 0;
}

uint16_t Potar::get(void) {
    	return value;
    }
void Potar::set_value(uint16_t set_value) {
	value = set_value;
}
void Potar::read_A_pin(){
	value = analogRead(pin);
}




