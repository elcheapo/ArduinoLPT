/*
 * helper_function.c
 *
 *  Created on: 5 d�c. 2017
 *      Author: florrain
 */


ISR(TIMER1_OVF_vect) {
	dcc_control.timer_overflow_interrupt();
}
#if 0
uint16_t decompte;
ISR(TIMER0_COMPA_vect) {
	if (decompte != 0) decompte --;
}
#endif

uint32_t buzzer_stop;

void buzzer_on(uint16_t ms) {
	DDRD=(1<<DDD3);
	PORTD|=(1<<PORTD3);
	buzzer_stop = millis() + ms;
}
void stop_buzzer(void) {
	if (millis() > buzzer_stop) {
		PORTD &= ~(1<<PORTD3);
	}
}


// List of functions for helpers
// Extract RAM based t_io from PROGMEM
void get_t_io(t_io &io_port, const t_io * flash_tio) {
	io_port.port = (I2c_Port *)pgm_read_word(&flash_tio->port);
	io_port.mask = (uint8_t)pgm_read_byte(&flash_tio->mask);
}
// Occupancy detector : these are quasi bidi I/O, 1 = Not occupied, 0 = occupied
void current_detection (void) {
	uint32_t time_stamp;
	uint8_t temp;
	t_io io_port;
	for (uint8_t i = 0; i<MAX_TRACKS; i++) {
		get_t_io(io_port,tracks[i].sensor);
		temp = io_port.port->read(time_stamp) & io_port.mask;
		if ((temp == 0) && (tracks[i].occupied == 0)) {
			// Loco just got detected on track
			tracks[i].timestamp = time_stamp;
		}
		if (temp ==  0) {
			tracks[i].occupied = 1;
		} else {
			tracks[i].occupied = 0;
		}
	}
}

void i2c_1 (void) {
	i2c_port1.read_i2c();
	i2c_port1.write_i2c();
}
void i2c_2 (void) {
	i2c_port2.read_i2c();
	i2c_port2.write_i2c();
}
void i2c_3 (void) {
	i2c_port3.read_i2c();
	i2c_port3.write_i2c();
}
void i2c_4 (void) {
	i2c_port4.read_i2c();
	i2c_port4.write_i2c();
}
void i2c_5 (void) {
	i2c_port5.read_i2c();
	i2c_port5.write_i2c();
}


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
#if 0
void read_pot2(void){
	pot2.read_A_pin();
}
void read_pot3(void){
	pot3.read_A_pin();
}
#endif
void alarm_current(void) {
	/* ADC step is 5V / 1024 (10 bit resolution) = 4,88mV / Step
	 * Current flows through a 0.22 Ohm resistor so 4.88mV / 0.22 = 22 mA
	 * Alarm at 1.5A level = 1500/22 = 68 rounded up at 70.
	 */
	uint16_t current;
	current=alarm.get();
	if (current > 70) {
		dcc_control.end();
		digitalWrite(A2,LOW);
		digitalWrite(A3,LOW);
		current_alarm = 1;
	}
}
uint8_t radio_packet[32], radio_count,radio_id;

void radio_get_packet_scan() {
	if(radio_get_packet(radio_packet, radio_count, radio_id)) {
		// We received something
		lcd.pseudo_led(9,1);
		if (radio_packet[0] == 2) { // That's a fine packet
			Radiopot1.set_value((uint16_t)((radio_packet[3] << 8)+radio_packet[4]));
			Radiopot2.set_value((uint16_t)((radio_packet[5] << 8)+radio_packet[6]));
//			Serial.write('P');
		}

	}
}

/*
  Read an integer from the keyboard interactively
 */
int8_t getint(uint16_t & value) {
	uint8_t last = 0;
	uint8_t key;
	// Wait until no key is pressed
	do {
		delay(200);
		key = kbd.get_key();
	} while (key != 0);
	Serial.write('o');
	value = 0;
	while (1) {
		lcd.print('\x83');
		do {
			key = kbd.get_key_debounced(last);
			delay(200);
		} while (key == 0);

		switch (key) {
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
		case '0':
			value = (value*10) + (key-'0');
			lcd.write('\b');
			lcd.write(key);
			break;
		case 'D':
			lcd.write('\b');
			lcd.write(' ');
			Serial.println(value);
			return 0;
		case '*':
			return -1;
		case 'B':
			value = value / 10;
			lcd.print(F("\b \b\b"));
			break;
		}
	}
	return (-1);
}

void set_light (const t_signal * light, l_state light_state ) {
	t_io port1,port2;
	get_t_io(port1, &light->red_light);
	get_t_io(port2, &light->green_light);
	if (light_state == l_red) {
		port1.port->clear_mask(port1.mask);
		port2.port->set_mask(port2.mask);

	} else if (light_state == l_green) {
		port1.port->set_mask(port1.mask);
		port2.port->clear_mask(port2.mask);
	} else {
		port1.port->set_mask(port1.mask);
		port2.port->set_mask(port2.mask);
	}
}



void (* const todo_in_idle[])(void) PROGMEM = {
		&scan_col_0,
		&scan_col_1,
		&scan_col_2,
		&scan_col_3,
		&i2c_1,
		&i2c_2,
		&i2c_3,
		&i2c_4,
		&i2c_5,
		&update_lcd,
		&read_pot1,
//		&read_pot2,
//		&read_pot3,
		&run_organizer,
		&alarm_current,
		&stop_buzzer,
		&radio_get_packet_scan,
		&current_detection
};
#define NB_TASK (sizeof(todo_in_idle)/sizeof(void(*)()))

void yield(void) {
	void (* current_todo_in_idle)(void);
	if (top_level_delay != 0 ) return;
	//	Serial.write('i');
	top_level_delay++;
	if (which_one == NB_TASK) which_one = 0;
	// call idle task ...*
//	todo_in_idle[which_one]();
	current_todo_in_idle = (void (*) (void))pgm_read_word(&todo_in_idle[which_one]);
	current_todo_in_idle();
	which_one++;
	top_level_delay--;
//	Serial.println(which_one);
}
