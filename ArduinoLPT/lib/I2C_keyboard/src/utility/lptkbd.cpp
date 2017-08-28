/*
 * Lptkbd.c
 *
 *  Created on: 12 avr. 2013
 *      Author: florrain
 */

/*
 * i2c_keyboard.cpp
 *
 *  Created on: 20 juin 2012
 *      Author: florrain
 */
#include <Arduino.h>
#include "lptkbd.h"

#undef DEBUG

Lptkbd::Lptkbd(void){
	for (uint8_t i=0; i<4; i++) {
		column[i]=0;
	}
}



#define TEL_KEYBOARD
#undef NORMAL_KEYBOARD
int8_t Lptkbd::get_key(void) {
#ifdef DEBUG
	Serial.print(F("COL : "));
	Serial.print(column[0],16);
	Serial.print(column[1],16);
	Serial.print(column[2],16);
	Serial.println(column[3],16);
#endif
#ifdef NORMAL_KEYBOARD
	if (column[0] != 0) {
		switch (column[0]) {
		case 0x10: return 0xf1;
		case 0x08: return '1';
		case 0x04: return '4';
		case 0x02: return '7';
		case 0x01: return '<';
		default: return 0;
		}
	}
	if (column[1] != 0) {
		switch (column[1]) {
		case 0x10: return 0xf2;
		case 0x08: return '2';
		case 0x04: return '5';
		case 0x02: return '8';
		case 0x01: return '0';
		default: return 0;
		}
	}
	if (column[2] != 0) {
		switch (column[2]) {
		case 0x10: return '#';
		case 0x08: return '3';
		case 0x04: return '6';
		case 0x02: return '9';
		case 0x01: return '>';
		default: return 0;
		}
	}
	if (column[3] != 0) {
		switch (column[3]) {
		case 0x10: return '*';
		case 0x08: return 'U';
		case 0x04: return 'D';
		case 0x02: return '[';
		case 0x01: return ']';
		default: return 0;
		}
	}
	return 0;
}
#endif
#ifdef TEL_KEYBOARD // Old remote
	if (column[0] != 0) {
		switch (column[0]) {
		case 0x08: return '*';
		case 0x04: return '7';
		case 0x02: return '4';
		case 0x01: return '1';
		default: return 0;
		}
	}
	if (column[1] != 0) {
		switch (column[1]) {
		case 0x08: return '0';
		case 0x04: return '8';
		case 0x02: return '5';
		case 0x01: return '2';
		default: return 0;
		}
	}
	if (column[2] != 0) {
		switch (column[2]) {
		case 0x08: return '#';
		case 0x04: return '9';
		case 0x02: return '6';
		case 0x01: return '3';
		default: return 0;
		}
	}
	if (column[3] != 0) {
		switch (column[3]) {
		case 0x08: return 'D';
		case 0x04: return 'C';
		case 0x02: return 'B';
		case 0x01: return 'A';
		default: return 0;
		}
	}
	return 0;
}
#endif
int8_t Lptkbd::get_key_debounced(uint8_t & last) {
	/* Only return key presses once */
	int8_t key=get_key();
	if (key == last)
		return 0;
	last = key;
	return key;
}


uint8_t Lptkbd::get_star_line(void) {
	uint8_t temp=0;
	if ((column[0] & 0x08) != 0 ) temp |= 0x08;
	if ((column[1] & 0x08) != 0 ) temp |= 0x04;
	if ((column[2] & 0x08) != 0 ) temp |= 0x02;
	if ((column[3] & 0x08) != 0 ) temp |= 0x01;
	return temp;
}

int8_t Lptkbd::get_star_line_debounced(uint8_t & last) {
	/* Only return key presses once */
	int8_t key=get_star_line();
	if (key == last)
		return 0;
	last = key;
	return key;
}



