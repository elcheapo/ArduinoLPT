

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


/*
 *             ----------------------------9----||----------------------7----
 *    A_NW    /----------------------------8----||----------------------6----\  A_NE
 *           |                                                                |
 *           |                                                                |
 *           10                                                               |
 *           |   Prog Track                                                   5
 *           |  ------------- A_GARAGE                                        |
 *    A_SW   \---------------\--------1----||--------------3-----------------/ A_SE
 *            ------------------------2----||--------------4-----------------
 *
 *
 */



typedef struct {
	uint32_t timestamp;
	uint8_t occupied;
	const t_io * sensor;
} t_track;

typedef struct {
	uint8_t dcc_address;
	uint8_t track_segment;
	locomem * loco;
	uint8_t reversed;
	uint8_t constructeur;
	uint8_t version;
} t_loco_on_track;

typedef struct {
	t_io red_light;
	t_io green_light;
} t_signal;

typedef enum {l_off, l_red, l_green} l_state;

I2c_Keyboard kbd(0x20);
//I2c_Keyboard kbd(0x38);

// Define 5 I2C extender
I2c_Port i2c_port1(0x21);
I2c_Port i2c_port2(0x22);
I2c_Port i2c_port3(0x23);
I2c_Port i2c_port4(0x24);
I2c_Port i2c_port5(0x25);

// Define occupancy detector

#define O_V1 0
#define O_V2 1
#define O_V3 2
#define O_V4 3
#define O_V5 4
#define O_V6 5
#define O_V7 6
#define O_V8 7
#define O_V9 8
#define O_V10 9


const t_io occupancy[] PROGMEM = {
		{&i2c_port1, 0x0} // segment de voie N°1
		,{&i2c_port1, 0x0} // segment de voie N°2
		,{&i2c_port4, 0x01} // segment de voie N°3 -OK
		,{&i2c_port4, 0x02} // segment de voie N°4 -OK
		,{&i2c_port4, 0x08} // segment de voie N°5 -OK
		,{&i2c_port5, 0x02} // segment de voie N°6 -OK
		,{&i2c_port5, 0x01} // segment de voie N°7-OK
		,{&i2c_port1, 0x0} // segment de voie N°8
		,{&i2c_port2, 0x0} // segment de voie N°9
		,{&i2c_port2, 0x0} // segment de voie N°10
};

// Define red / green lights
#define TL_V1 0
#define TL_V2 1
#define TL_V10 2
#define TL_V7 3
#define TL_V6 4
#define TL_V5 5

const t_signal traffic_lights[] PROGMEM = {
		{ {&i2c_port3, 0x0} /* 1 -> 10 Rouge */ ,{&i2c_port3, 0x0}} // 1 -> 10 Vert */
		,{{&i2c_port3, 0x0} /* 2 -> 10 Rouge */ ,{&i2c_port3, 0x0}} /* 2 -> 10 Vert */
		,{{&i2c_port3, 0x0} /* 10 -> 8/9 Rouge */ ,{&i2c_port3, 0x0}} /* 10 -> 8/9 Vert */
		,{{&i2c_port3, 0x0} /* 7 -> 5 Rouge */ ,{&i2c_port3, 0x0}} /* 7 -> 5 Vert */
		,{{&i2c_port2, 0x0} /* 6 -> 5 Rouge */	,{&i2c_port2, 0x0}} /* 6 -> 5 Vert */
		,{{&i2c_port2, 0x0} /* 5 -> 3/4 Rouge*/ ,{&i2c_port2, 0x0}} /* 5 -> 3/4 Vert */
};


// Define relays for 5 aiguillages

// define 5 aiguillages North / South / East / West / Garage
#define A_NE 0
#define A_NW 1
#define A_SE 2
#define A_SW 3
#define A_GARAGE 4

const relais_t relais[] PROGMEM = {
		{{&i2c_port4, 0x10},0} // NE droit -OK
		,{{&i2c_port4, 0x20},0} // NE Devie -OK
		,{{&i2c_port1, 0x01},0} // NW Droit OK
		,{{&i2c_port1, 0x02},0} // NW Devie -OK
		,{{&i2c_port4, 0x40},0} // SE droit OK
		,{{&i2c_port4, 0x80},0} // SE Devie  OK
		,{{&i2c_port2, 0x04},0} // SW droit OK
		,{{&i2c_port2, 0x08},0} // SW Devie OK
		,{{&i2c_port2, 0x01},0} // Garage Droit OK
		,{{&i2c_port2, 0x02},0} // Grarge Devie OK
};

aiguille aiguillage[5] = {
	{&relais[0],&relais[1],t_peco} // Aiguillage type Peco
	, {&relais[2],&relais[3],t_peco} // Aiguillage type Peco
	, {&relais[4],&relais[5],t_peco} // Aiguillage type Peco
	, {&relais[6],&relais[7],t_peco} // Aiguillage type Peco
	, {&relais[8],&relais[9],t_peco} // Aiguillage type Peco
};


/* List of possibles "trips", positive number is a track segment, negative number is a stop for n second */
const int8_t trips [][8] PROGMEM = {
		{5,3,1,10,9,7,50,0},
		{5,3,1,10,8,6,50,0},
		{5,4,2,10,9,7,50,0},
		{5,4,2,10,9,7,50,0},
		{5,3,1,10,9,-10,7,50},
		{5,3,1,10,8,-10,6,50},
		{5,4,2,10,9,-10,7,50},
		{5,4,2,10,9,-10,7,50}
};

#define MAX_TRACKS 10
t_track tracks[MAX_TRACKS] = {
		{0,0,&occupancy[0]}
		,{0,0,&occupancy[1]}
		,{0,0,&occupancy[2]}
		,{0,0,&occupancy[3]}
		,{0,0,&occupancy[4]}
		,{0,0,&occupancy[5]}
		,{0,0,&occupancy[6]}
		,{0,0,&occupancy[7]}
		,{0,0,&occupancy[8]}
		,{0,0,&occupancy[9]}
};


Nokia5510 lcd(PIN_SS, PIN_DC,PIN_RST);
DCC_timer dcc_control;
Potar alarm(CURRENT_SENSE); // Current measurement on Analog 0
Potar pot1(1);
Potar Radiopot1, Radiopot2;

uint8_t radio_ok;

t_loco_on_track loco1, loco2;


//Potar pot2(0);
//Potar pot3(0);


/* things to do */
uint8_t which_one;
uint8_t top_level_delay;
uint8_t last;
uint8_t current_alarm;

tmode station_mode;

#include <helper_function.h>

//The setup function is called once at startup of the sketch
void setup() {
	uint8_t i,ret,status;
	// Add your initialization code here
	// Set PB0 to deactivate regulator until we are ready
	REGULATOR_OFF;

	DIDR0 = 0x03; // see page 257 of datasheet, disable digital pin on pin used for ADC
	OCR0A = 0x80; // interrupt every ms on timer 0
	PRR=0; // all peripheral activated
	DDRD |= (1<<PIN_CSN)|(1<<PIN_CE);

	pinMode(A2, OUTPUT);
	digitalWrite(A2,LOW);
	pinMode(A3, OUTPUT);
	digitalWrite(A3,LOW);
	// We use millis() to account for time measurements
//	TIMSK0 |= 1<<OCIE0A;
	which_one = 0;
	current_alarm = 0;
	top_level_delay = 1; // wait until everything is initialized before we enable the helper functions
	radio_ok = 0;
	Serial.begin(115200);
	SPI.begin();
	SPI.beginTransaction(SPISettings (8000000, MSBFIRST, SPI_MODE0));
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
#if 0
	/* Dump T_io's */
	Serial.println(F("Occupancy :"));
	uint8_t * ptr = (uint8_t *)&occupancy[0];
	for (uint8_t i = 0; i < MAX_TRACKS; i++) {
		Serial.print(F("0x"));
		Serial.print(pgm_read_word(ptr + 3*i),16);
		Serial.print(F(" 0x"));
		Serial.println(pgm_read_byte(ptr + 3*i + 2),16);
	}
	Serial.println(F("Occupancy via get_tio :"));

	t_io test_io;
	for (uint8_t i = 0; i< MAX_TRACKS; i++) {
		get_t_io(test_io, &occupancy[i]);
		Serial.print(F("0x"));
		Serial.print((uint16_t)test_io.port,16);
		Serial.print(F(" 0x"));
		Serial.println((uint8_t)test_io.mask,16);


	}
	Serial.println(F("Occupancy via tracks :"));

	for (uint8_t i = 0; i< MAX_TRACKS; i++) {
		get_t_io(test_io, tracks[i].sensor);
		Serial.print(F("0x"));
		Serial.print((uint16_t)test_io.port,16);
		Serial.print(F(" 0x"));
		Serial.println((uint8_t)test_io.mask,16);


	}
 while (1);
#endif
#if 0
	/* Dump T_io's */
	Serial.println(F("Traffic_lights :"));
	uint8_t * ptr = (uint8_t *)&traffic_lights[0];
	for (uint8_t i = 0; i < 12; i++) {
		Serial.print(F("0x"));
		Serial.print(pgm_read_word(ptr + 3*i),16);
		Serial.print(F(" 0x"));
		Serial.println(pgm_read_byte(ptr + 3*i + 2),16);
	}
	Serial.println(F("traffic_lights via get_tio :"));

	t_io test_io;
	t_signal * signal1;
	for (uint8_t i = 0; i< 6; i++) {
		signal1 = &traffic_lights[i];
		get_t_io(test_io, &signal1->red_light);
		Serial.print(F("0x"));
		Serial.print((uint16_t)test_io.port,16);
		Serial.print(F(" 0x"));
		Serial.println((uint8_t)test_io.mask,16);
		get_t_io(test_io, &signal1->green_light);
		Serial.print(F("0x"));
		Serial.print((uint16_t)test_io.port,16);
		Serial.print(F(" 0x"));
		Serial.println((uint8_t)test_io.mask,16);


	}

 while (1);
#endif

	last = 0;
	// Current Detector are all inputs
	for(uint8_t i =0 ; i<MAX_TRACKS; i++) {
		t_io io_port;
		get_t_io(io_port,&occupancy[i]);
		io_port.port->set_input(io_port.mask);
	}
	i2c_port1.set_input_i2c();
	i2c_port2.set_input_i2c();
	i2c_port3.set_input_i2c();
	i2c_port4.set_input_i2c();
	i2c_port5.set_input_i2c();
	/* The above code also sets all outputs to 0, which is needed not to drive the points */
	/* but the lights will be on so turn them off ... */
	for (uint8_t i = 0; i<6; i++) {
		set_light(&traffic_lights[i], l_off);
	}
	// Now the pins to drive the points are 0
	// Turn ON the regulator
	REGULATOR_ON;
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
	}
	lcd.clear();

	switch (key) {
	// station_mode is set to Digital or Analog
	case '1': {
		Serial.println(F("Testing Current sensor"));
		// Testing Current sensor
		lcd.menu(F("            "),
				F(" TESTING    "),
				F(" Current    "),
				F(" Sensors    "),
				F("            "),
				F(" * : Sortie "));
		while (1) {
			delay(200);
			lcd.go(0,4);
			for (uint8_t i = 0; i < MAX_TRACKS; i++) {
				if (tracks[i].occupied != 0) {
					lcd.write('X');
				} else {
					lcd.write('-');
				}
			}
			key=kbd.get_key_debounced(last);
			if (key == '*') break;
		}
		break;
	}
	case '2': {
		Serial.println(F("Testing fisrt point"));
		lcd.menu(F("            "),
				F(" TESTING    "),
				F(" First      "),
				F("Aiguillage  "),
				F("            "),
				F(" * : Sortie "));
		while (1) {
			delay(200);
			lcd.go(0,4);
			aiguillage[0].set_state(s_droit);
			lcd.print(F(" Droit "));
			delay(200);
			aiguillage[0].set_state(s_devie);
			lcd.go(0,4);
			lcd.print(F(" Devie "));
			delay(200);
			key=kbd.get_key_debounced(last);
			if (key == '*') break;
		}
		break;
	}
	case '3': {
		Serial.println(F("Testing all points"));
		lcd.menu(F("            "),
				F(" TESTING    "),
				F(" All        "),
				F("Aiguillage  "),
				F("            "),
				F(" * : Sortie "));
		while (1) {
			for (uint8_t i = 0; i<5; i++) {
				delay(500);
				lcd.go(0,4);
				aiguillage[i].set_state(s_droit);
				lcd.print (i,10);
				lcd.print(F(" Droit "));
				delay(1000);
				aiguillage[i].set_state(s_devie);
				lcd.go(0,4);
				lcd.print (i,10);
				lcd.print(F(" Devie "));
				delay(1000);
			}
			key=kbd.get_key_debounced(last);
			if (key == '*') break;
		}
		break;
	}
	case '4': {
		Serial.println(F("Testing all points"));
		lcd.menu(F("            "),
				F(" TESTING    "),
				F(" All        "),
				F(" Feux       "),
				F("            "),
				F(" * : Sortie "));
		while (1) {
			for (uint8_t i = 0; i<6; i++) {
				delay(500);
				lcd.go(0,4);
				set_light(&traffic_lights[i], l_red);
				lcd.print (i,10);
				lcd.print(F(" Rouge "));
				delay(500);
				set_light(&traffic_lights[i], l_green);
				lcd.go(0,4);
				lcd.print (i,10);
				lcd.print(F(" Green  "));
				delay(500);
				set_light(&traffic_lights[i], l_off);
			}
			key=kbd.get_key_debounced(last);
			if (key == '*') break;
		}
		break;
	}

	case 'A': {
		uint16_t speed;
		tdirection direction;
		uint8_t key;
		// Analog mode
		Serial.println(F("Analog"));
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
				if (radio_ok != 0) {
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
				lcd.go(0,5);
				for (uint8_t i = 0; i < MAX_TRACKS; i++) {
					if (tracks[i].occupied != 0) {
						lcd.write('X');
					} else {
						lcd.write('-');
					}
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

		Serial.println(F("Digital"));

		dcc_control.begin(digital);
		// Activate main track, disable programming track
		digitalWrite(A2,HIGH);
		digitalWrite(A3,LOW);

		lcd.menu(F("            "),
				F(" DIGITAL    "),
				F(" Adresse    "),
				F("Loc 1:      "),
				F("Loc 2:      "),
				F("            ")
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
			// Afficher les feux verts et rouges
			light_control();

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
						F("            ")
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
					if (radio_ok != 0) {
						position = Radiopot1.get();
					} else {
						position=512;
					}
					if (position > 530) {
						speed = ((position-520) >> 2) + 2 ;
						loco_ptr->speed = speed;
						lcd.go(4,3);
						lcd.write(0x81);
						lcd.print(speed);
					} else if (position < 494) {
						speed = (((500-position) >> 2) + 2) | 0x80;
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
					if (radio_ok != 0) {
						position = Radiopot2.get();
					} else {
						position=512;
					}
					if (position > 530) {
						speed = ((position-520) >> 2) + 2;
						loco_ptr->speed = speed;
						lcd.go(4,4);
						lcd.write(0x81);
						lcd.print(speed);
					} else if (position < 494) {
						speed = (((500-position) >> 2) + 2) | 0x80;
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
				lcd.go(0,5);
				for (uint8_t i = 0; i < MAX_TRACKS; i++) {
					if (tracks[i].occupied != 0) {
						lcd.write('X');
					} else {
						lcd.write('-');
					}
				}

			}
		}
		break;
	}
	case 'B': {
		uint8_t address,constructeur,version;
		Serial.print(F("Reading Adress"));
		delay(100);
		station_mode = digital;
		dcc_control.begin(digital);
		dcc_control.set_direct();
		// Power up Prog Track, disable main track
		digitalWrite(A3,HIGH);
		digitalWrite(A2,LOW);
		lcd.menu(F("            "),
				F(" Lecture    "),
				F(" Adresse    "),
				F("Adr  :      "),
				F("Const:      "),
				F("Ver  :      ")
		);
		delay(100);
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
		} else {
			lcd.go(7,3);
			lcd.print(F("NON"));
			delay(100);
		}
		key = 0;
		while (key==0) {
			// Wait until keypress
			key=kbd.get_key_debounced(last);
			delay(100);
		}
		break;
	}
	case 'C' : {
		// digital mode - Automatic
		station_mode = digital;
		uint16_t position;
		uint8_t speed;


		Serial.println(F("Digital"));
		// 1) Read Loco address on prog track
		uint8_t address,constructeur,version;
		Serial.print(F("Reading Adress"));
		delay(100);
		aiguillage[A_GARAGE].set_state(s_devie);

		station_mode = digital;
		dcc_control.begin(digital);
		dcc_control.set_direct();

		// Power up Prog Track, disable main track
		digitalWrite(A3,HIGH);
		digitalWrite(A2,LOW);
		lcd.menu(F("            "),
				F(" Mettre une "),
				F("Locomotive  "),
				F("sur la voie "),
				F("de garage   "),
				F("            ")
		);
		delay(100);
		int8_t loco_detected = 0;
		while (loco_detected == 0) {
			if (set_programmer(&dcc_control,CURRENT_SENSE) == true) {
				// Loco detected
				address = direct_mode_read(1);
				constructeur = direct_mode_read(8);
				version = direct_mode_read(7);
				lcd.go(0,5);
				lcd.print(F("Detected"));
				delay(100);
				loco_detected = 1;
			} else {
				lcd.go(0,5);
				lcd.print(F("Pas de Loco"));
				delay(100);
			}
			if (kbd.get_key_debounced(last) == '*') {
				loco_detected = -1;
			}

		}
		if (loco_detected == -1 ) break;
		// Loco detected at address "address"
		loco1.loco = new_loco(address);
		if (loco1.loco == NULL) break;
		// Loco controlled by POT 1
		loco1.loco->control_by = 1;
		loco1.constructeur = constructeur;
		loco1.version = version;
		// Start normal DCC mode
		dcc_control.set_queue();
		aiguillage[A_GARAGE].set_state(s_droit);

		//switch on Garage + main track
		digitalWrite(A3,HIGH);
		digitalWrite(A2,HIGH);
		lcd.menu(F("            "),
				F(" Demarrer la"),
				F("Locomotive  "),
				F("vers la voie"),
				F("principale  "),
				F("Adr:        ")
		);
		buzzer_on(200);
		lcd.go (5,5);
		lcd.print(address, 10);
		delay(200);
		// Loco controlled by Radiopot1
		while(1) {
			if (radio_ok != 0) {
				position = Radiopot1.get();
			} else {
				position=512;
			}
			if (position > 530) {
				speed = ((position-520) >> 2) + 2 ;
			} else if (position < 494) {
				speed = (((500-position) >> 2) + 2) | 0x80;
			} else {
				speed = 1;
			}
			loco1.loco->speed = speed;
			// Are we seeing the loco on track 1 yet ?
			if (tracks[O_V1].occupied != 0) {
				// The guy moved the loco in the correct direction so now we know the reverse/direct direction ...
				loco1.reversed = speed & 0x80;

			}
			if (tracks[O_V3].occupied != 0) {
				// Now we take over the loco
				loco1.reversed = speed & 0x80;
				buzzer_on(200);
				loco1.loco->speed = 1;
				break; // from the while loop
			}
			delay (200);
		}
		aiguillage[A_GARAGE].set_state(s_devie);
		aiguillage[A_SW].set_state(s_devie);
		aiguillage[A_NW].set_state(s_droit);
		aiguillage[A_NE].set_state(s_droit);
		aiguillage[A_SE].set_state(s_devie);
		if (loco1.reversed  == 0) {
			loco1.loco->speed = 0x80 + 50;
		} else {
			loco1.loco->speed = 50;
		}
		// Now wait until we see the loco in O_V7
		while (tracks[O_V10].occupied == 0);
		buzzer_on(100);
		while (tracks[O_V9].occupied == 0);
		buzzer_on(100);
		while (tracks[O_V7].occupied == 0);
		lcd.menu(F("            "),
				F(" Loco 1     "),
				F(" en         "),
				F(" position   "),
				F("            "),
				F("            ")
		);
		delay(2000);
		loco1.loco->speed = 1;
		buzzer_on(200);
		/********************************************************************/
		// No do the second loco
		/********************************************************************/
		aiguillage[A_GARAGE].set_state(s_devie);

		// Power up Prog Track, disable main track
		digitalWrite(A3,HIGH);
		digitalWrite(A2,LOW);
		lcd.menu(F("            "),
				F(" Mettre une "),
				F("Locomotive  "),
				F("sur la voie "),
				F("de garage   "),
				F("            ")
		);
		delay(100);
		loco_detected = 0;
		while (loco_detected == 0) {
			if (set_programmer(&dcc_control,CURRENT_SENSE) == true) {
				// Loco detected
				address = direct_mode_read(1);
				constructeur = direct_mode_read(8);
				version = direct_mode_read(7);
				lcd.go(0,5);
				lcd.print(F("Detected"));
				delay(100);
				loco_detected = 1;
			} else {
				lcd.go(0,5);
				lcd.print(F("Pas de Loco"));
				delay(100);
			}
			if (kbd.get_key_debounced(last) == '*') {
				loco_detected = -1;
			}

		}
		if (loco_detected == -1 ) break;
		// Loco detected at address "address"
		loco2.loco = new_loco(address);
		if (loco2.loco == NULL) break;
		// Loco controlled by POT 1
		loco2.loco->control_by = 1;
		loco2.constructeur = constructeur;
		loco2.version = version;
		// Start normal DCC mode
		dcc_control.set_queue();
		aiguillage[A_GARAGE].set_state(s_droit);

		//switch on Garage + main track
		digitalWrite(A3,HIGH);
		digitalWrite(A2,HIGH);
		lcd.menu(F("            "),
				F(" Demarrer la"),
				F("Locomotive  "),
				F("vers la voie"),
				F("principale  "),
				F("Adr:        ")
		);
		buzzer_on(200);
		lcd.go (5,5);
		lcd.print(address, 10);
		delay(300);
		// Loco controlled by Radiopot1
		while(1) {
			if (radio_ok != 0) {
				position = Radiopot1.get();
			} else {
				position=512;
			}
			if (position > 530) {
				speed = ((position-520) >> 2) + 2 ;
			} else if (position < 494) {
				speed = (((500-position) >> 2) + 2) | 0x80;
			} else {
				speed = 1;
			}
			loco2.loco->speed = speed;
			// Are we seeing the loco on track 1 yet ?
			if (tracks[O_V1].occupied != 0) {
				// The guy moved the loco in the correct direction so now we know the reverse/direct direction ...
				loco2.reversed = speed & 0x80;

			}
			if (tracks[O_V3].occupied != 0) {
				// Now we take over the loco
				loco2.reversed = speed & 0x80;
				buzzer_on(200);
				loco2.loco->speed = 1;
				break; // from the while loop
			}
			delay(200);
		}

		aiguillage[A_GARAGE].set_state(s_devie);
		aiguillage[A_SW].set_state(s_devie);
		aiguillage[A_NW].set_state(s_devie);
		aiguillage[A_NE].set_state(s_devie);
		aiguillage[A_SE].set_state(s_devie);
		if (loco2.reversed  == 0) {
			loco2.loco->speed = 0x80 + 50;
		} else {
			loco2.loco->speed = 50;
		}
		// Now wait until we see the loco in O_V6
		while (tracks[O_V10].occupied == 0);
		buzzer_on(100);
		delay(150);
		while (tracks[O_V8].occupied == 0);
		buzzer_on(100);
		delay(150);
		while (tracks[O_V6].occupied == 0);
		lcd.menu(F("            "),
				F(" Loco 2     "),
				F(" en         "),
				F(" position   "),
				F("            "),
				F("            ")
		);
		delay(2000);
		loco2.loco->speed = 1;
		buzzer_on(200);


		/**************************************************************************************************************/
		loco1.loco->control_by = 1;
		loco2.loco->control_by = 2;


// Now we are ready to run ... the loco should never bump into each other ...


		dcc_control.begin(digital);
		dcc_control.set_queue();
		// Activate main track, disable programming track
		digitalWrite(A2,HIGH);
		digitalWrite(A3,LOW);

		// At this point we have 2 loco ready
		// We can now read the pot values to set the speed ...
		while (1) {
			uint16_t position;
			uint8_t key,speed;
			locomem * loco_ptr;
			// Afficher les feux verts et rouges
			light_control();

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
						F("            ")
				);
				key = kbd.get_key_debounced(last);
				if (key == '*') break;
				loco_ptr = find_control(1);
				// control function for loco 1
				switch (key) {
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
				case 'A' : {
					aiguillage[A_NW].set_state(s_droit);
					aiguillage[A_NE].set_state(s_droit);
				}
				case 'B' : {
					aiguillage[A_NW].set_state(s_devie);
					aiguillage[A_NE].set_state(s_devie);
				}
				case 'C' : {
					aiguillage[A_SW].set_state(s_devie);
					aiguillage[A_SE].set_state(s_devie);
				}
				case 'D' : {
					aiguillage[A_SW].set_state(s_droit);
					aiguillage[A_SE].set_state(s_droit);
				}
				default:
					break;
				}
				// Loco controled by Radiopot1
				if (loco_ptr != NULL) {
					lcd.go(0,3);
					lcd.print(loco_ptr->address);
					if (radio_ok != 0) {
						position = Radiopot1.get();
					} else {
						position=512;
					}
					if (position > 530) {
						speed = ((position-520) >> 2) + 2 ;
						loco_ptr->speed = speed;
						lcd.go(4,3);
						lcd.write(0x81);
						lcd.print(speed);
					} else if (position < 494) {
						speed = (((500-position) >> 2) + 2) | 0x80;
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
					if (radio_ok != 0) {
						position = Radiopot2.get();
					} else {
						position=512;
					}
					if (position > 530) {
						speed = ((position-520) >> 2) + 2;
						loco_ptr->speed = speed;
						lcd.go(4,4);
						lcd.write(0x81);
						lcd.print(speed);
					} else if (position < 494) {
						speed = (((500-position) >> 2) + 2) | 0x80;
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

				lcd.go(0,5);
				for (uint8_t i = 0; i < MAX_TRACKS; i++) {
					if (tracks[i].occupied != 0) {
						lcd.write('X');
					} else {
						lcd.write('-');
					}
				}

			}
		}
		break;
	}
	default:
		break;
	} /* End Case */
}



