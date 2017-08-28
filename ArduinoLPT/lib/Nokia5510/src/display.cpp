/*
 * display.cpp
 *
 *  Created on: 12 avr. 2013
 *      Author: florrain
 *  This is a mostly virtual class used to implement a display on the Nokia 5510 lcd display
 *  either connected directly or through a radio link
 *
 */

#include "Arduino.h"
#include "display.h"

Display::Display( uint8_t led) {
	led_line = led;
	current_x = 0;
	current_y = 0;
	for (uint8_t i = 0; i < NUM_LINES; i++ )
		for (uint8_t j = 0; j < NUM_COL; j++ )
			lcd_text[i][j]='A';
	lcd_update = 0;
}

void Display::clear(void) {
	// High level Clear
	for (uint8_t i = 0; i<NUM_LINES; i++)
		for(uint8_t j = 0; j < NUM_COL;j++)
			lcd_text[i][j]=' ';
}

void Display::clearline(void) {
	for(uint8_t i = 0; i < NUM_COL;i++)
		lcd_text[current_y][i]=' ';
}

/*
 * Will store characters to be printed in a RAM array
 * LcdUpdate will display everything at once
 * 0<= x <= 3, 0<= y <= 15
 */

size_t Display::write(uint8_t character){
	if ((character >= 0x20) && (character <= 0x89)) {
		lcd_text[current_y][current_x] = character;
		lcd_update=1;
		current_x ++;
		if (current_x >= NUM_COL) {
			current_x = 0;
			current_y ++;
		}
	} else if (character == '\n') { // Process LF
		current_y ++;
	} else if (character == '\r') { // Process Return
		current_x = 0;
	} else if (character == '\b') { // process Backspace
		if (current_x != 0) current_x --;
	}
	if (current_y >= NUM_LINES)
		current_y = 0;
	return 1;
}

void Display::go(uint8_t x, uint8_t y) {
	if(current_x < NUM_COL) current_x = x;
	if(current_y < NUM_LINES) current_y = y;
}

void Display::pseudo_led(uint8_t led,uint8_t on_off) {
	if (led_line == 0 ) return;
	if (led < 10) {
		pseudo_led_value[led] = on_off;
		lcd_update = 1;
	}
}

uint8_t Display::get_pseudo_led (uint8_t no) {
	if (led_line == 0) return 0;
	return pseudo_led_value[no];
}

void Display::bar(uint8_t col, uint8_t heigh) {
	bar(col, heigh, 5);
}

void Display::bar(uint8_t col, uint8_t height, uint8_t max) {
	if (height > 40) height=40;
	for (uint8_t i = 0; i<max; i++){
		switch (height) {
		default:
		case 8:
			lcd_text[5-i][col] = 0x89;
			height -=8;
			break;
		case 7:
			lcd_text[5-i][col] = 0x88;
			height = 0;
			break;
		case 6:
			lcd_text[5-i][col] = 0x87;
			height = 0;
			break;
		case 5:
			lcd_text[5-i][col] = 0x86;
			height = 0;
			break;
		case 4:
			lcd_text[5-i][col] = 0x85;
			height = 0;
			break;
		case 3:
			lcd_text[5-i][col] = 0x84;
			height = 0;
			break;
		case 2:
			lcd_text[5-i][col] = 0x83;
			height = 0;
			break;
		case 1:
			lcd_text[5-i][col] = 0x82;
			height = 0;
			break;
		case 0:
			lcd_text[5-i][col] = ' ';
			height = 0;
			break;
		}
	}
	lcd_update=1;
}

void Display::menu(const __FlashStringHelper *line1, \
		const __FlashStringHelper *line2, \
		const __FlashStringHelper *line3, \
		const __FlashStringHelper *line4, \
		const __FlashStringHelper *line5, \
		const __FlashStringHelper *line6) {
	const char *pline1 = (const char *)line1;
	const char *pline2 = (const char *)line2;
	const char *pline3 = (const char *)line3;
	const char *pline4 = (const char *)line4;
	const char *pline5 = (const char *)line5;
	const char *pline6 = (const char *)line6;
	for( uint8_t i=0; i<12; i++){
		lcd_text[0][i] = pgm_read_byte(pline1++);
		lcd_text[1][i] = pgm_read_byte(pline2++);
		lcd_text[2][i] = pgm_read_byte(pline3++);
		lcd_text[3][i] = pgm_read_byte(pline4++);
		lcd_text[4][i] = pgm_read_byte(pline5++);
		lcd_text[5][i] = pgm_read_byte(pline6++);
	}
	lcd_update = 1;
}






