/*
 * i2c_keyboard.cpp
 *
 *  Created on: 20 juin 2012
 *      Author: florrain
 */
#include "Arduino.h"
#include "utility/lptkbd.h"

#include "Wire.h"
#include "I2C_keyboard.h"

#ifndef TRUE
#define TRUE (0==0)
#define FALSE (!TRUE)
#endif
#undef DEBUG

I2c_Keyboard::I2c_Keyboard(uint8_t _i2c_address) : Lptkbd() {
	i2c_address = _i2c_address;
}

void I2c_Keyboard::scan(void) {
	uint8_t i,temp,ret;
	// Keyboard connection is 	C3 C2 C1 L4 L3 L2 L1 C4
	// I2C Pin					P7 P6 P5 P4 P3 P2 P1 P0
	const uint8_t col_drive[]={ 0xdf, 0xbf, 0x7f, 0xfe };

	for (i=0; i<4; i++) {
		// scan column 1
		Wire.beginTransmission(i2c_address); // transmit to PCF8574
		Wire.write(col_drive[i]);
		ret = Wire.endTransmission();
//#ifdef DEBUG
		if (ret != 0) Serial.print(F("NoACK"));
//#endif
		Wire.requestFrom(i2c_address, 1);
		if ( Wire.available() !=0 ) {
			temp = Wire.read();
		}
#ifdef DEBUG
		if (temp != 0) {
			Serial.print(F(" Col"));
			Serial.write(0x31+i);
			Serial.print(F(" = 0x"));
			Serial.print(temp,16);
		} else {
			Serial.write('.');
		}
#endif
		column[i] = ((temp >> 1) & 0x0f) ^ 0x0f; // 0000 L4 L3 L2 L1 format
	}
#ifdef DEBUG
	Serial.println();
#endif
	return;
}

void I2c_Keyboard::scan_col(uint8_t col) {
	uint8_t temp,ret;
	// Keyboard connection is 	C3 C2 C1 L4 L3 L2 L1 C4
	// I2C Pin					P7 P6 P5 P4 P3 P2 P1 P0
	const uint8_t col_drive[]={ 0xdf, 0xbf, 0x7f, 0xfe };

	// scan column col
	Wire.beginTransmission(i2c_address); // transmit to PCF8574
	Wire.write(col_drive[col]);
	ret = Wire.endTransmission();
	if (ret != 0) Serial.print(F("NoACK"));
	Wire.requestFrom(i2c_address, 1);
	if ( Wire.available() !=0 ) {
		temp = Wire.read();
		column[col] = ((temp >> 1) & 0x0f) ^ 0x0f; // 0000 L4 L3 L2 L1 format
	}
}



