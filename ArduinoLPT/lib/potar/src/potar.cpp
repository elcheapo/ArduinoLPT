/*
 * potar.cpp
 *
 *  Created on: 23 août 2017
 *      Author: florrain
 */

#include <Arduino.h>
#include "potar.h"

Potar::Potar (uint8_t _pin) {
	pin = _pin;
}

uint16_t Potar::get(void) {
    	return value;
    }
void Potar::read_A_pin(){
	value = analogRead(pin);
}




