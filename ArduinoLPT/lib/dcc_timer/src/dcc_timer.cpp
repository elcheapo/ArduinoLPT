/*
 * dcc_timer.cpp
 *
 *  Created on: 25 avr. 2012
 *      Author: florrain
 */

#include "Arduino.h"
#include "organizer.h"
#include "dcc_timer.h"


#undef DEBUG

// Constructors ////////////////////////////////////////////////////////////////

DCC_timer::DCC_timer( void ) {
	direct = 1;
	direct_ready = 0;
}
/*
extern DCC_timer dcc_control;

ISR(TIMER1_OVF_vect) {
	dcc_control.timer_overflow_interrupt();
}
*/

inline void DCC_timer::do_send1(void) {
    ICR1 = (F_CPU / 1000000L) * PERIOD_1;
	OCR1A = (F_CPU / 1000000L) * PERIOD_1 / 2;
	OCR1B = (F_CPU / 1000000L) * PERIOD_1 / 2;
}

inline void DCC_timer::do_send0(void) {
    ICR1 = (F_CPU / 1000000L) * PERIOD_0;
    OCR1A = (F_CPU / 1000000L) * PERIOD_0 / 2;
	OCR1B = (F_CPU / 1000000L) * PERIOD_0 / 2;
}


void DCC_timer::timer_overflow_interrupt(void) {
	// Uses timer x in fast PWM / OCRxA = TOP, OCRxB : = toggle on match, OCRxC : inverted output
	switch (_doi_packet.state) {
	case DOI_INTER_PACKET: {
		do_send1();
		/* Insure that we have 5 ms between packets as per DCC 9.2 */
		/* So we wait for 14 (+ 14 or 20) 0/1's before processing the next packet */
		/* 14 * 232�s * 14*112�s = 4.8 ms + packet loading time */
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0) _doi_packet.state = DOI_IDLE;
		if (pkt_abort != 0) {	// in case we need to stop repeating packet
			pkt_ready = 0;
			pkt_abort = 0;
		}
		break;
	}
	case DOI_IDLE: {
		do_send1();
		if (pkt_ready != 0) {
			if (_doi_packet.repeat_ctr >= current_message.repeat) {
				pkt_ready = 0; // DONE processing packet and repeat
				_doi_packet.repeat_ctr = 0;
			} else {
				// send / resend message
				_doi_packet.state = DOI_PREAMBLE;                           // current state
				_doi_packet.ibyte = 0;
				_doi_packet.bitcount = 0;
				_doi_packet.xor_byte = 0;
				if (current_message.type == is_prog)		// TODO: Update logic
					_doi_packet.bitcount = 25;   		// long preamble if service mode
				else
					_doi_packet.bitcount = 14;     	// regular preamble
			}
		}
		break;
	}
	case DOI_PREAMBLE: {
		do_send1();
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0)
			_doi_packet.state = DOI_BSTART;
		break;
	}
	case DOI_BSTART: {
		do_send0();
		if (current_message.size == _doi_packet.ibyte)	{ // message done, goto xor
			_doi_packet.cur_byte = _doi_packet.xor_byte;
//			Serial.println(_doi_packet.cur_byte,16);
			_doi_packet.state = DOI_XOR;
			_doi_packet.bitcount = 8;
		} else { // get next addr or data
			_doi_packet.cur_byte = current_message.dcc[_doi_packet.ibyte++];
//			Serial.print(_doi_packet.cur_byte,16);Serial.write(' ');
			_doi_packet.xor_byte ^= _doi_packet.cur_byte;
			_doi_packet.state = DOI_BYTE;
			_doi_packet.bitcount = 8;
		}
		break;
	}
	case DOI_BYTE:	{
		if (_doi_packet.cur_byte & 0x80) do_send1(); else do_send0();
		_doi_packet.cur_byte <<= 1;
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0)
			_doi_packet.state = DOI_BSTART;
		break;
	}
	case DOI_XOR: {
		if (_doi_packet.cur_byte & 0x80) do_send1(); else do_send0();
		_doi_packet.cur_byte <<= 1;
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0) {
			_doi_packet.state = DOI_LAST_BIT;
		}
		break;
	}
	case DOI_LAST_BIT: {
		do_send1();
		_doi_packet.state = DOI_INTER_PACKET;
		_doi_packet.bitcount = 1;
		_doi_packet.repeat_ctr ++;
		//		xSemaphoreGiveFromISR(ready_for_acknowledge, &xHigherPriorityTaskWoken); // tell the world about it ...
		//		if( xHigherPriorityTaskWoken != pdFALSE ) taskYIELD();
		break;
	}
	default:
		while(1);
		break;
	}
}

void DCC_timer::begin(tmode mode){
	if (mode == digital) {
		_doi_packet.repeat_ctr = 0;
		// Use mode 10 (1010): fast PWM top set in ICR1
		TCCR1A = 0 << WGM10| 1 << WGM11
				| 1 << COM1A0	| 1 << COM1A1		// NonInverted output on OC1A
				| 0 << COM1B0	| 1 << COM1B1;		// inverted output on OC1B
		TCCR1B = 1<<WGM13 | 1 << WGM12
				| (0<<CS12) | (0<<CS11) | (1<<CS10);// no prescaler, source = sys_clk

		// start with 0's
		ICR1 = (F_CPU / 1000000L) * PERIOD_0;
		OCR1A = (F_CPU / 1000000L) * PERIOD_0 / 2;       //Inverted output 58�s = 58*16 = 928 clocks
		OCR1B = (F_CPU / 1000000L) * PERIOD_0 / 2;		 // Non inverted output
		TCNT1 = 0; 										// Synchronize timers
		// Enable Timer Overflow Interrupt
		TIMSK1 = (1<<TOIE1);
		// Set OCRA/B pins as Output
		DDRB |= T1_OCRA|T1_OCRB;
	} else { // Analog
		TCCR1A = 1 << WGM10| 0 << WGM11			// Fast PWM  8 bit 0-FF
				| 0 << COM1A0	| 0 << COM1A1		// PWM signal on OCRA or OCRB
				| 0 << COM1B0	| 0 << COM1B1;

		TCCR1B = 0<<WGM13	| 1 << WGM12
				| (1<<CS12) | (0<<CS11) | (0<<CS10);	//  prescaler / 256, source=16 MHz / 256 = 244 Hz
		TIMSK1 = 0; 				// no interrupt
		OCR1A = 0;
		OCR1B = 0;
		DDRB |= T1_OCRA|T1_OCRB;
		PORTB &= ~(T1_OCRA|T1_OCRB); // start with output OCRA/B deactivated
		TIMSK1 = 0; // No interrupt
	}
}
void DCC_timer::abort_dcc(void){
	pkt_abort = 1;
}

void DCC_timer::send_dcc_packet(message * current){
	// MUST be called when pkt_ready is 0
	if (direct == 0) {
		current_message = *current;
		pkt_ready = 1;
	}
}

void DCC_timer::send_direct_dcc_packet(message * direct) {
	// MUST be called when pkt_ready is 0
	current_message = *direct;
	pkt_ready = 1;
}

void DCC_timer::end(void) {
	TIMSK1 = 0; // disable timer interrupt
	TCCR1A = 0 << WGM10| 0 << WGM11
			| 0 << COM1A0	| 0 << COM1A1		// No output on OCxA
			| 0 << COM1B0	| 0 << COM1B1;		// No output on OCxB
	TCCR1B = 0<<WGM13 | 0 << WGM12
			| (0<<CS12) | (0<<CS11) | (0<<CS10);// timer stopped, no clock
	DDRB |= T1_OCRA|T1_OCRB;
	PORTB &= ~(T1_OCRA|T1_OCRB); // turn off all signals
}

void DCC_timer::analog_set_speed_and_direction(uint16_t speed, tdirection direction) {
	speed = speed >> 1;   // range for speed is 0 - 512
	speed = speed & 0xFF; // limit range to 0 - 255
	OCR1A = 10; // adc measurement at the beginning of the pulse
	if (direction == off) {
		TCCR1A = 1 << WGM10| 0 << WGM11			// PWM Phase correct 8 bit 0-FF
				| 0 << COM1A0	| 0 << COM1A1		// No PWM signal
				| 0 << COM1B0	| 0 << COM1B1;

		TCCR1B = 0<<WGM13	| 1 << WGM12
				| (1<<CS12) | (0<<CS11) | (0<<CS10);	//  prescaler / 256, source=16 MHz / 256 = 244 Hz
		DDRB |= T1_OCRA|T1_OCRB;
		PORTB &= ~(T1_OCRA|T1_OCRB); // OCRA/B set to 0
	} else if (direction == backward) {
		TCCR1A = 1 << WGM10| 0 << WGM11			// PWM Phase correct 8 bit 0-FF
				| 0 << COM1A0	| 0 << COM1A1		// PWM signal non-inverted on OCRB
				| 0 << COM1B0	| 1 << COM1B1;
		TCCR1B = 0<<WGM13	| 1 << WGM12
				| (1<<CS12) | (0<<CS11) | (0<<CS10);	//  prescaler / 256, source=16 MHz / 256 = 244 Hz
		DDRB |= T1_OCRA;
		PORTB &= ~(T1_OCRA); // OCRA set to 0
		OCR1B = speed;
	} else {// forward
		TCCR1A = 1 << WGM10| 0 << WGM11			// PWM Phase correct 8 bit 0-FF
				| 0 << COM1A0	| 1 << COM1A1		// PWM signal non inverted on OCRA
				| 0 << COM1B0	| 0 << COM1B1;

		TCCR1B = 0<<WGM13	| 1 << WGM12
				| (1<<CS12) | (0<<CS11) | (0<<CS10);	//  prescaler / 256, source=16 MHz / 256 = 244 Hz
		DDRB |= T1_OCRB;
		PORTB &= ~(T1_OCRB); // OCRB set to 0
		OCR1A = speed;
	}
}

uint16_t DCC_timer::analog_get_speed(void) {
	if ( (TCCR1A & (0 << COM1B0 | 1 << COM1B1)) == (0 << COM1B0 | 1 << COM1B1))
		return OCR1B * 2;
	else if ( (TCCR1A & (0 << COM1A0 | 1 << COM1A1)) == (0 << COM1A0 | 1 << COM1A1))
		return OCR1A * 2;
	else
		return 0;
}

tdirection DCC_timer::analog_get_direction(void) {
	if ( (TCCR1A & (0 << COM1B0 | 1 << COM1B1)) == (0 << COM1B0 | 1 << COM1B1)) return forward;
	if ( (TCCR1A & (0 << COM1A0 | 1 << COM1A1)) == (0 << COM1A0 | 1 << COM1A1)) return backward;
	return off;
}


