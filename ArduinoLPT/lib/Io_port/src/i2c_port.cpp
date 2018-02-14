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
	read_value = 0;
	write_value =0;
	input_mask =0;
	time_stamp = 0;
	modified = 0;
}

void I2c_Port::write_i2c (void) {
	uint8_t ret;
	pinMode(5,OUTPUT);
	digitalWrite(5,1);
	if (modified == 0) return;
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
#ifdef DEBUG
	Serial.print(F("Write, value = 0x"));
	Serial.println(current_value,16);
#endif
	Wire.write(write_value | input_mask); // make sure we are not changing "input" pins
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
	modified = 0;
	digitalWrite(5,0);
}

void I2c_Port::read_i2c (void) {
	uint8_t temp;
	Wire.requestFrom(i2c_address, (uint8_t)1);
	if ( Wire.available() !=0 ) {
		temp = Wire.read();
	}
	time_stamp = millis();
	read_value = temp;
}

void I2c_Port::set_input_i2c(void) {
	uint8_t ret;
	// Set the INPUT PIN to 1
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	Wire.write(input_mask);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));

}



void I2c_Port::set_input(uint8_t mask) {
	uint8_t ret;
	input_mask |= mask;
	// Set the INPUT PIN to 1
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	Wire.write(mask);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));

}
