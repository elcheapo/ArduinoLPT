
#include <Arduino.h>
#include <io_port.h>
#include <i2c_port.h>
#include <relay.h>
#include <I2C_keyboard.h>
#include <display.h>
#include <nokia5510.h>
#include <dcc_timer.h>
#include <aiguillage.h>
#include <potar.h>
#include <Wire.h>
#include <SPI.h>


I2c_Port i2c_port1(0x21);

const relais_t relais[] PROGMEM = {
		{&i2c_port1, 0x01, 1} //
		,{&i2c_port1, 0x02, 1} //
		,{&i2c_port1, 0x04, 1} //
		,{&i2c_port1, 0x08, 1} //
		,{&i2c_port1, 0x80, 1} //
		,{&i2c_port1, 0x40, 1} //
		,{&i2c_port1, 0x20, 1} //
		,{&i2c_port1, 0x10, 1} //
};


I2c_Keyboard kbd(0x20);
Nokia5510 lcd(6,7,8);
DCC_timer dcc_control;
Potar pot1(2);
Potar pot2(1);
Potar pot3(0);
aiguille aiguillage(&relais[1],&relais[0],t_peco); // Aiguillage type Peco



/* things to do */
uint8_t which_one;
uint8_t top_level_delay;

ISR(TIMER1_OVF_vect) {
	dcc_control.timer_overflow_interrupt();
}

// List of functions for helpers

void scan_col_0(void) {
	kbd.scan_col(0);
}
void scan_col_1(void) {
	kbd.scan_col(1);
}
void scan_col_2(void) {
	kbd.scan_col(2);
}
void scan_col_3(void) {
	kbd.scan_col(3);
}
void update_lcd (void) {
	lcd.update();
}
void read_pot1(void){
	pot1.read_A_pin();
}
void read_pot2(void){
	pot2.read_A_pin();
}
void read_pot3(void){
	pot3.read_A_pin();
}



//The setup function is called once at startup of the sketch
void setup()
{
	uint8_t i,ret;
// Add your initialization code here
	DIDR0 = 0x0f; // see page 257 of datasheet, disable digital pin on pin used for ADC
	which_one = 0;
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

//	dcc_control.begin(analog);


}

// The loop function is called in an endless loop
void loop()
{
	uint8_t last,key;
	uint16_t position;
//Add your repeated code here
	top_level_delay = 0;
	delay(200);
	// keypress handling
	last=0;
	key=kbd.get_key_debounced(last);
	if (key != 0) {
		Serial.write(key);
		switch (key) {
			case '0' ... '9': {
				key = key - '0';
				if (lcd.get_pseudo_led(key) == 0) {
					lcd.pseudo_led(key,1);
				} else {
					lcd.pseudo_led(key,0);

				}
				break;
			case 'A':
				aiguillage.set_state(s_droit);
				break;
			case 'B':
				aiguillage.set_state(s_devie);
				break;
			}
			default:
				break;
		}
	}
	// Pot handling
	lcd.clear();
	position = pot1.get();
//	Serial.print(F("Pot1 = "));
//	Serial.println(position);
	if (position > 512) { // "Positive"
		lcd.bar(1,(position - 512) / 13);
	} else {
		lcd.bar(2,(512- position) / 13);
	}
	position = pot2.get();
//	Serial.print(F("Pot1 = "));
//	Serial.println(position);
	if (position > 512) { // "Positive"
		lcd.bar(4,(position - 512) / 13);
	} else {
		lcd.bar(5,(512- position) / 13);
	}
	position = pot3.get();
//	Serial.print(F("Pot1 = "));
//	Serial.println(position);
	if (position > 512) { // "Positive"
		lcd.bar(7,(position - 512) / 13);
	} else {
		lcd.bar(8,(512- position) / 13);
	}


}



#define NB_TASK 9
void (*todo_in_idle[NB_TASK])() = {
		&scan_col_0,
		&scan_col_1,
		&scan_col_2,
		&scan_col_3,
		&update_lcd,
		&read_pot1,
		&read_pot2,
		&read_pot3,
		&run_organizer
};

void yield(void) {
	if (top_level_delay != 0 ) return;
	top_level_delay++;
	if (which_one == NB_TASK) which_one = 0;
	// call idle task ...*
	todo_in_idle[which_one]();
	which_one++;
	top_level_delay--;
}
