

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "config.h"
#include <aiguillage.h>
#include <dcc_timer.h>
#include <HardwareSerial.h>
#include <i2c_port.h>
#include <I2C_keyboard.h>
#include <nokia5510.h>
#include <organizer.h>
#include <potar.h>
#include <relay.h>
#include <stddef.h>
#include <stdint.h>
#include <SPI.h>
#include <utility/lptkbd.h>
#include <Wire.h>
#include <WString.h>
#include "radio.h"
#include "programmer.h"

#if 0
//I2c_Port i2c_port1(0x21);
//I2c_Port feux(0x22);


const relais_t relais[] PROGMEM = {
		{&i2c_port1, 0x01, 1} //
		,{&i2c_port1, 0x02, 1} //
		,{&i2c_port1, 0x04, 1} //
		,{&i2c_port1, 0x08, 1} //
		,{&i2c_port1, 0x80, 0} //
		,{&i2c_port1, 0x40, 0} //
		,{&i2c_port1, 0x20, 0} //
		,{&i2c_port1, 0x10, 0} //
};
#endif

typedef struct {
	uint32_t timestamp;
	uint8_t occupied;
} track;



//I2c_Keyboard kbd(0x20);
I2c_Keyboard kbd(0x38);
// Uses PCF8474 for current detection on 8 track segments
I2c_Port detector(0x21);
Nokia5510 lcd(PIN_SS, PIN_DC,PIN_RST);
DCC_timer dcc_control;
Potar alarm(CURRENT_SENSE); // Current measurement on Analog 0
Potar pot1(1);
Potar Radiopot1, Radiopot2;
//Potar pot2(0);
//Potar pot3(0);
//aiguille aiguillage(&relais[3],&relais[2],t_peco); // Aiguillage type Peco

SPISettings fastSPI(8000000, MSBFIRST, SPI_MODE0);

/* things to do */
uint8_t which_one;
uint8_t top_level_delay;
uint8_t last;
uint8_t current_alarm;
track track_status[8];


tmode station_mode;

#include <helper_function.h>

//The setup function is called once at startup of the sketch
void setup() {
	uint8_t i,ret,status;
	// Add your initialization code here
	DIDR0 = 0x03; // see page 257 of datasheet, disable digital pin on pin used for ADC
	OCR0A = 0x80; // interrupt every ms on timer 0
	PRR=0; // all peripheral activated
	DDRD |= (1<<PIN_CSN)|(1<<PIN_CE);

	pinMode(A2, OUTPUT);
	digitalWrite(A2,LOW);
	pinMode(A3, OUTPUT);
	digitalWrite(A3,LOW);
	TIMSK0 |= 1<<OCIE0A;
	which_one = 0;
	current_alarm = 0;
	last_detect = 0;
	top_level_delay = 1; // wait until everything is initialized before we enable the helper functions
	Serial.begin(115200);
	SPI.begin();
	lcd.lcd_reset();
	lcd.menu(F("            "),
			F("  LPT       "),
			F("  Arduino   "),
			F("  Training  "),
			F("  v1.00     "),
			F(" \x80\x80\x80\x80 - \x81\x81\x81\x81"));
	lcd.update();
	delay (500);
	Serial.println(F("Scanning I2C bus"));
	Wire.begin();
	for (i=1; i<127; i++) {
		// scan I2C bus
		Wire.beginTransmission(i); // transmit to PCF8574
		ret = Wire.endTransmission();
		if (ret == 0) {
			Serial.print(F("I2C dev at address : 0x"));
			Serial.println(i,16);
		}
	}
	Serial.println(F("Done"));
	status = radio_pl_init_prx();
	if ((status & 0x80) == 0) {
		Serial.println(F("Radio OK"));
	} else {
		Serial.print(F("Radio NOT OK : 0x"));
		Serial.println(status,16);
//		while (1); // We can ignore error if it is not there
		// If higher bit of radio status is not 0 - we have a wiring issue ...
	}


	//	i2c_port1.write(0xff);
	last = 0;
	// Current Detector are all inputs
	detector.set_mask(0xff);


	//	dcc_control.begin(analog);


}

// The loop function is called in an endless loop
void loop()
{
	uint8_t key;
	uint16_t position;
	//Add your repeated code here
	top_level_delay = 0;
	init_organizer();
	dcc_control.end();
	//	buzzer_on(50000);
	delay(100);
	// Select Analog / Digital
	station_mode = dcc_off;
	lcd.menu(F("            "),
			F(" Select     "),
			F(" A = Analog "),
			F(" B = Read Adr"),
			F(" D = digital"),
			F(" \x80\x80\x80\x80 - \x81\x81\x81\x81"));

	key = 0;
	while (key == 0) {
		delay(200);
		key = kbd.get_key();
		switch (key) {
		case 'A':
			Serial.println(F("ANALOG"));
			break;
		case 'B':
			Serial.println(F("READ ADDRESS"));
			break;
		case 'D':
			Serial.println(F("DIGITAL"));
			break;
		default:
			break;
		}
	}
	lcd.clear();

	switch (key) {
	// station_mode is set to Digital or Analog

	case 'A': {
		uint16_t speed;
		tdirection direction;
		uint8_t key;
		// Analog mode
		station_mode = analog;
		dcc_control.begin(analog);
		digitalWrite(A2,HIGH);
		while (1) {
			//			Serial.write('a');
			delay (200);
			if (current_alarm != 0) {
				lcd.menu(F("            "),
						F(" ANALOGIQUE "),
						F(" COURT      "),
						F("    CIRCUIT "),
						F("            "),
						F(" * : Sortie "));
				key=kbd.get_key_debounced(last);
				if (key == '*') {
					current_alarm = 0;
					break;
				}
			} else {
				lcd.menu(F("            "),
						F(" ANALOGIQUE "),
						F(" Vit :      "),
						F("            "),
						F("            "),
						F("            "));
				key=kbd.get_key_debounced(last);
				if (key == '*') break;
				switch (key) {
#if 0
				case 'A':
					aiguillage.set_state(s_droit);
					break;
				case 'B':
					aiguillage.set_state(s_devie);
					break;
#endif
				case '0' ... '9':
				key = key - '0';
				if (lcd.get_pseudo_led(key) == 0) {
					lcd.pseudo_led(key,1);
				} else {
					lcd.pseudo_led(key,0);

				}
				break;
				case '#':
					buzzer_on(1000);
					break;
				}
				// Set PWM according to Radiopot1
				position = Radiopot1.get();
				if (position > 530) {
					speed = position-520;
					direction= forward;
				} else if (position < 490) {
					speed = 500-position;
					direction= backward;
				} else {
					speed = 0;
					direction = off;
				}
				dcc_control.analog_set_speed_and_direction(speed,direction);
				lcd.go(7,2);
				lcd.print(speed);
				lcd.go(3,3);
				if (direction == forward) {
					lcd.print(F("\x81\x81\x81\x81"));
				} else if (direction == backward) {
					lcd.print(F("\x80\x80\x80\x80"));
				} else {
					lcd.print(F("----"));
				}
			}
		}
		break;
	}
	case 'D' : {
		// digital mode
		int8_t ret;
		uint16_t value;
		locomem * loco_ptr;
		station_mode = digital;
		dcc_control.begin(digital);
		digitalWrite(A2,HIGH);
		lcd.menu(F("            "),
				F(" DIGITAL    "),
				F(" Adresse    "),
				F("Loc 1:      "),
				F("Loc 2:      "),
				F("Loc 3:      ")
		);
		Serial.write('1');
		dcc_control.set_queue();
		delay(200);
		// Get address for Loco 1
		lcd.go(6,3);
		ret = getint(value);
		if (ret == 0) {
			if (value != 0) {
				loco_ptr= new_loco(value);
				if (loco_ptr != NULL) loco_ptr->control_by = 1;
			}
		}
		Serial.write('2');
		// Get address for Loco 2
		lcd.go(6,4);
		ret = getint(value);
		if (ret == 0) {
			if (value != 0) {
				loco_ptr= new_loco(value);
				if (loco_ptr != NULL) loco_ptr->control_by = 2;
			}
		}
#if 0
		Serial.write('3');
		// Get address for Loco 3
		lcd.go(6,5);
		ret = getint(value);
		if (ret == 0) {
			if (value != 0) {
				loco_ptr= new_loco(value);
				if (loco_ptr != NULL) loco_ptr->control_by = 3;
			}
		}
#endif
		// At this point we have up to 3 loco ready
		// We can now read the pot values to set the speed ...
		while (1) {
			uint16_t position;
			uint8_t key,speed;
			locomem * loco_ptr;
			// Now update the display
			delay(200);
			if (current_alarm != 0) {
				lcd.menu(F("            "),
						F(" DIGITAL    "),
						F(" COURT      "),
						F("    CIRCUIT "),
						F("            "),
						F(" * : Sortie "));
				key=kbd.get_key_debounced(last);
				if (key == '*') {
					current_alarm = 0;
					break;
				}
			} else {
				lcd.menu(F("            "),
						F(" DIGITAL    "),
						F("Adr : Speed "),
						F("    :       "),
						F("    :       "),
						F("    :       ")
				);
				key = kbd.get_key_debounced(last);
				if (key == '*') break;
				loco_ptr = find_control(1);
				// control function for loco 1
				switch (key) {
#if 0
				case 'A':
					aiguillage.set_state(s_droit);
					break;
				case 'B':
					aiguillage.set_state(s_devie);
					break;
#endif
					// Key 0 to 9 : functions 0 to 9
				case '0' : {
					if (loco_ptr==NULL) break;
					loco_ptr->fl ^= 0x01; // fct 0
					break;
				}
				case '1' : {
					if (loco_ptr==NULL) break; // fct 1
					loco_ptr->f4_f1 ^= 0x01;
					break;
				}
				case '2' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f4_f1 ^= 0x02; //fct 2
					break;
				}
				case '3' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f4_f1 ^= 0x04; // fct 3
					break;
				}
				case '4' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f4_f1 ^= 0x08; // fct 4
					break;
				}
				case '5' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f8_f5 ^= 0x01; // fct 5
					break;
				}
				case '6' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f8_f5 ^= 0x02; break; // fct 6
				}
				case '7' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f8_f5 ^= 0x04; break; // fct 7
				}
				case '8' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f8_f5 ^= 0x08; break; // fct 8
				}
				case '9' : {
					if (loco_ptr==NULL) break;
					loco_ptr->f12_f9 ^= 0x01; break; // fct 9
				}

				}
				// Loco controled by Radiopot1
				if (loco_ptr != NULL) {
					lcd.go(0,3);
					lcd.print(loco_ptr->address);
					position = Radiopot1.get();
					if (position > 530) {
						speed = (position-520) >> 2;
						loco_ptr->speed = speed;
						lcd.go(4,3);
						lcd.write(0x81);
						lcd.print(speed);
					} else if (position < 490) {
						speed = ((500-position) >> 2) | 0x80;
						loco_ptr->speed = speed;
						lcd.go(4,3);
						lcd.write(0x80);
						lcd.print(speed & 0x7f);
					} else {
						loco_ptr->speed = 1;
						lcd.go(4,3);
						lcd.write('-');
						lcd.print(F(" 0"));
					}
					loco_ptr->speed = speed;
				}
				// Loco controled by pot2
				loco_ptr = find_control(2);
				if (loco_ptr != NULL) {
					lcd.go(0,4);
					lcd.print(loco_ptr->address);
					position = Radiopot2.get();
					if (position > 530) {
						speed = (position-520) >> 2;
						loco_ptr->speed = speed;
						lcd.go(4,4);
						lcd.write(0x81);
						lcd.print(speed);
					} else if (position < 490) {
						speed = ((500-position) >> 2) | 0x80;
						loco_ptr->speed = speed;
						lcd.go(4,4);
						lcd.write(0x80);
						lcd.print(speed & 0x7f);
					} else {
						loco_ptr->speed = 1;
						lcd.go(4,4);
						lcd.write('-');
						lcd.print(F(" 0"));
					}
				}
#if 0

				// Loco controled by pot3
				loco_ptr = find_control(3);
				if (loco_ptr != NULL) {
					lcd.go(0,5);
					lcd.print(loco_ptr->address);
					position = pot3.get();
					if (position > 530) {
						speed = (position-520) >> 2;
						loco_ptr->speed = speed;
						lcd.go(4,5);
						lcd.write(0x81);
						lcd.print(speed);
					} else if (position < 490) {
						speed = ((500-position) >> 2) | 0x80;
						loco_ptr->speed = speed;
						lcd.go(4,5);
						lcd.write(0x80);
						lcd.print(speed & 0x7f);
					} else {
						loco_ptr->speed = 1;
						lcd.go(4,5);
						lcd.write('-');
						lcd.print(F(" 0"));
					}
				}
#endif
			}
		}
		break;
	}
	case 'B': {
		uint8_t address,constructeur,version;
		bool no_loco;
		station_mode = digital;
		dcc_control.begin(digital);
		dcc_control.set_direct();
		digitalWrite(A3,HIGH);
		lcd.menu(F("            "),
				F(" Lecture    "),
				F(" Adresse    "),
				F("Adr  :      "),
				F("Const:      "),
				F("Ver  :      ")
		);
		delay(100);
		no_loco = true;
		while (no_loco == true) {
			if (set_programmer(&dcc_control,CURRENT_SENSE) == true) {
				// Loco detected
				address = direct_mode_read(1);
				lcd.go(7,3);
				lcd.print(address);
				delay(100);
				constructeur = direct_mode_read(8);
				lcd.go(7,4);
				lcd.print(constructeur);
				delay(100);
				version = direct_mode_read(7);
				lcd.go(7,5);
				lcd.print(version);
				delay(100);
				last = 0;
				key = 0;
				no_loco = false;
				while (key==0) {
					key=kbd.get_key_debounced(last);
					delay(100);
				}
			} else {
				lcd.go(7,3);
				lcd.print(F("NON"));
				delay(100);
			}
		}
		break;
	}
	default:
		break;
	}
}


