/*
 * nokialcd.c
 *
 *  Created on: Feb 19, 2011
 *      Author: francois
 */
/*
This Code has extra features
including a XY positioning function on Display
and a Line Draw function on Nokia 3310 LCD
It is modded from the original
http://www.arduino.cc/playground/Code/PCD8544
*/
// Mods by Jim Park
// jim(^dOt^)buzz(^aT^)gmail(^dOt^)com
// hope it works for you

#include "Arduino.h"
#include "SPI.h"
#include "nokia5510.h"

#define nop()  __asm__ __volatile__("nop")
#define LED_LINE 1


#define LCD_C     0
#define LCD_D     1

#define LCD_X     84
#define LCD_Y     48

static const PROGMEM uint8_t ASCII[][5] =
{
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c backslash
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
,{0x00, 0x06, 0x09, 0x09, 0x06} // 7f ?
,{0x00, 0x08, 0x1c, 0x3e, 0x7f} // 80 <
,{0x7f, 0x3e, 0x1c, 0x08, 0x00} // 81 >
,{0x80, 0x80, 0x80, 0x80, 0x80} // 82 _
,{0xc0, 0xc0, 0xc0, 0xc0, 0xc0} // 83
,{0xe0, 0xe0, 0xe0, 0xe0, 0xe0} // 84
,{0xf0, 0xf0, 0xf0, 0xf0, 0xf0} // 85
,{0xf8, 0xf8, 0xf8, 0xf8, 0xf8} // 86
,{0xfc, 0xfc, 0xfc, 0xfc, 0xfc} // 87
,{0xfe, 0xfe, 0xfe, 0xfe, 0xfe} // 88
,{0xff, 0xff, 0xff, 0xff, 0xff} // 89 Full Bar


};

static const PROGMEM uint8_t LED_STATE[][8] = {
 { 0x00,0x00,0x3F,0x21,0x21,0x21,0x21,0x3F }
,{ 0x00,0x00,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F }
,{ 0x00,0x00,0x3F,0x39,0x39,0x39,0x39,0x3F }
};

void Nokia5510::lcd_reset(void) {
	pinMode(pin_ss,OUTPUT);
	pinMode(pin_cd,OUTPUT);
	pinMode(pin_rst,OUTPUT);

	digitalWrite(pin_ss, 1);
	digitalWrite(pin_cd, 1);
	digitalWrite(pin_rst, 0); // Reset LCD
	delay(2);
	digitalWrite(pin_rst, 1); // Release LCD Reset

}

Nokia5510::Nokia5510(uint8_t _pin_ss, uint8_t _pin_cd, uint8_t _pin_rst): Display(LED_LINE) {
	pin_ss = _pin_ss;
	pin_cd = _pin_cd;
	pin_rst = _pin_rst;
}

void Nokia5510::lcd_print(uint8_t character) {
	uint8_t temp;
	LcdWrite(LCD_D, 0x00);
	for (uint8_t index = 0; index < 5; index++) {
		temp = pgm_read_byte(&ASCII[character - 0x20][index]);
		LcdWrite(LCD_D,temp);
	}
	LcdWrite(LCD_D, 0x00);
}

void Nokia5510::low_level_clear(void) {
	// Low level Clear
	gotoXY(0,0);
	for (int index = 0; index < LCD_X * LCD_Y / 8; index++) {
		LcdWrite(LCD_D, 0x00);
	}
}

void Nokia5510::begin(void) {
	LcdWrite( LCD_C, 0x21 );  // LCD Extended Commands.
	LcdWrite( LCD_C, 0xBf );  // Set LCD Vop (Contrast). //B1
	LcdWrite( LCD_C, 0x04 );  // Set Temp coefficent. //0x04
	LcdWrite( LCD_C, 0x14 );  // LCD bias mode 1:48. //0x13
	LcdWrite( LCD_C, 0x20);	// Normal commands
	LcdWrite( LCD_C, 0x0C);	// LCD in normal mode. 0x0d for inverse
	lcd_update = 0;
}

void Nokia5510::LcdWrite(uint8_t dc, uint8_t data) {
#ifdef DEBUG
	Serial3.print(data,16); Serial3.print(' ');
#endif
	if (dc == 0)
		digitalWrite(pin_cd,0);
	else
		digitalWrite(pin_cd,1);
	digitalWrite(pin_ss,0);
	nop();
	SPI.transfer(data);
	digitalWrite(pin_ss,1);
}

// gotoXY routine to position cursor
// x - range: 0 to 84
// y - range: 0 to 5

void Nokia5510::gotoXY(uint8_t x, uint8_t y) {
	LcdWrite( LCD_C, 0x80 | x);  // Column.
	LcdWrite( LCD_C, 0x40 | y);  // Row.

}
void Nokia5510::normal(void) {
	LcdWrite( LCD_C, 0x0C);	// LCD in normal mode. 0x0d for inverse
}

void Nokia5510::inverse(void) {
	LcdWrite( LCD_C, 0x0D);	// LCD in normal mode. 0x0d for inverse
}

void Nokia5510::update(void) {
	uint8_t i,j,temp;
	if (lcd_update != 0) {
		begin();
		if (led_line != 0) {
			gotoXY(0,led_line-1); // Beginning OF LED line
			// write pseudo_leds
			for (i =0; i<10; i++) {
				for (j=0; j<8; j++) {
					if (pseudo_led_value[i] == 0)
						temp = pgm_read_byte(&LED_STATE[0][j]);
					else if (pseudo_led_value[i] == 1)
						temp = pgm_read_byte(&LED_STATE[1][j]);
					else
						temp = pgm_read_byte(&LED_STATE[2][j]);
					LcdWrite(LCD_D,temp);
				}
			}
			// finish line
			LcdWrite(LCD_D,0);
			LcdWrite(LCD_D,0);
			LcdWrite(LCD_D,0);
			LcdWrite(LCD_D,0);
		}
/* Now rewrite text */
		for (i=0; i < NUM_LINES; i++) {
			if (i != led_line-1																																																															){
				gotoXY(0,i);
				for (j=0; j< NUM_COL; j++) {
#ifdef DEBUG
					Serial.write(lcd_text[i][j]);
#endif
					lcd_print(lcd_text[i][j]);
				}
#ifdef DEBUG
				Serial.println();
#endif
				}
		}
		lcd_update =0;
	}

}


