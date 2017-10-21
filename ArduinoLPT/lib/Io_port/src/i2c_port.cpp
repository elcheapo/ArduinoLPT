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
	current_value = x;
	Serial.print(F("Write, value = 0x"));
	Serial.println(current_value,16);
	Wire.write(current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));

}

void I2c_Port::clear_mask (uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	current_value = ~(x) & current_value;
	Serial.print(F("Clear Mask, value = 0x"));
	Serial.println(current_value,16);
	Wire.write(current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
}
void I2c_Port::set_mask(uint8_t x) {
	uint8_t ret;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	current_value = x | current_value;
	Serial.print(F("Set Mask, value = 0x"));
	Serial.println(current_value,16);
	Wire.write(current_value);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
}

uint8_t I2c_Port::read (void) {
	return current_value;
}

void I2C_Port::set_input(uint8_t mask) {

}
