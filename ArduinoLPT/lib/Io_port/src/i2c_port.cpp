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
	input_mask =0;
	time_stamp = 0;
}

void I2c_Port::write (uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	current_value = x | input_mask; // make sure we are not changing "input" pins
#ifdef DEBUG
	Serial.print(F("Write, value = 0x"));
	Serial.println(current_value,16);
#endif
	Wire.write(current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));

}

void I2c_Port::clear_mask (uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	current_value = (~(x) & current_value) | input_mask;
#ifdef DEBUG
	Serial.print(F("Clear Mask, value = 0x"));
	Serial.println(current_value,16);
#endif
	Wire.write(current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
}
void I2c_Port::set_mask(uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	current_value = x | current_value | input_mask;
#ifdef DEBUG
	Serial.print(F("Set Mask, value = 0x"));
	Serial.println(current_value,16);
#endif
	Wire.write(current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
}

void I2c_Port::read (void) {
	uint8_t temp;
	Wire.requestFrom(i2c_address, (uint8_t)1);
	if ( Wire.available() !=0 ) {
		temp = Wire.read();
	}
	time_stamp = millis();
	current_value = temp;
}

uint8_t I2c_Port::read_cached(void) {
	return current_value;
}
uint8_t I2c_Port::read_cached(uint32_t & _time_stamp) {
	_time_stamp = time_stamp;
	return current_value;
}


void I2c_Port::set_input(uint8_t mask) {
	uint8_t ret;
	input_mask = mask;
	// Set the INPUT PIN to 1
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	Wire.write(mask);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));

}
