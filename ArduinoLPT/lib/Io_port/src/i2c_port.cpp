/*
 * i2c_port.cpp
 *
 *  Created on: 31 Nov. 2013
 *      Author: florrain
 */
#include "Arduino.h"
#include "io_port.h"
#include "i2c_port.h"

//#define DEBUG
#undef DEBUG


I2c_Port::I2c_Port(uint8_t _i2c_address) {
	i2c_address = _i2c_address;
	current_value = 0;
}

void I2c_Port::write (uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	Wire.write(x);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));

}

void I2c_Port::clear_mask (uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	Wire.write(~(x) & current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
}
void I2c_Port::set_mask(uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	Wire.write( x | current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
}

uint8_t I2c_Port::read (void) {
	return current_value;
}


