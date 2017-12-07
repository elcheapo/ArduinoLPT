/*
 * dcc_timer.h
 *
 *  Created on: 25 avr. 2012
 *      Author: florrain
 */

#ifndef DCC_TIMER_H_
#define DCC_TIMER_H_

#include "organizer.h"
// define which timers are used for DCC output
#define USE_TIMER1

// define some handy names for the states of the ISR
#define DOI_IDLE     (0)
#define DOI_PREAMBLE (1)
#define DOI_BSTART   (2)
#define DOI_BYTE     (3)
#define DOI_XOR      (4)
#define DOI_INTER_PACKET (5)
#define DOI_LAST_BIT (6)

typedef enum {dcc_off, analog, digital, read_adr} tmode;
typedef enum {off, forward, backward} tdirection;

typedef struct {
	uint8_t state;                            // current state
	uint8_t ibyte;                            // current index of byte in message
	uint8_t bitcount;							// current bit index n char
	uint8_t cur_byte;                         // current byte value
	uint8_t xor_byte;                         // actual check
	uint8_t repeat_ctr;						// current number of repeat
} doi;

/// This are timing definitions from NMRA
#define PERIOD_1   116L                  // 116us for DCC 1 pulse - do not change
//#define PERIOD_1   120L                  // 104us for DCC 1 pulse - do not change
#define PERIOD_0   232L                  // 232us for DCC 0 pulse - do not change
//#define PERIOD_0   200L                  // 200us for DCC 0 pulse - do not change

class DCC_timer {
private:
	doi _doi_packet;
	message current_message;
	volatile uint8_t pkt_abort;
	volatile uint8_t pkt_ready;
	uint8_t direct;
	uint8_t direct_ready;
	void do_send0(void);
	void do_send1(void);

public:
	DCC_timer( void	);
	void begin(tmode mode);
	void end(void);
	void send_dcc_packet(message * current);
	void send_direct_dcc_packet(message * direct);
	void digital_on(void);
	void digital_off(void);
	void set_direct(void);
	void set_queue(void);
	void analog_set_speed_and_direction(uint16_t speed, tdirection direction);
	bool voie_is_on(void);
	uint16_t analog_get_speed(void);
	tdirection analog_get_direction(void);
	void timer_overflow_interrupt(void);
	void abort_dcc(void);
	tmode get_mode(void);
	uint8_t dcc_busy(void);
	uint8_t dcc_queue_busy(void);
};

inline uint8_t DCC_timer::dcc_queue_busy(void) {
	if (direct != 0) return 0;
	return (pkt_ready);
};
inline uint8_t DCC_timer::dcc_busy(void) {return (pkt_ready);};

inline void DCC_timer::set_direct(void) {direct=1;};
inline void DCC_timer::set_queue(void) {direct=0;};

#define T1_OCRA (0x02)
#define T1_OCRB (0x04)

inline void DCC_timer::digital_on(void) {
		PORTB |= T1_OCRA;  // Set OCRA in digital
};
inline void DCC_timer::digital_off(void) {
		PORTB &= ~T1_OCRA;  // Clear OCRA in digital
};

inline tmode DCC_timer::get_mode(void) {
	uint8_t mode = TCCR1A & 0x03;
	if (mode == 0x02) return analog;
	if (mode == 0x03) return digital;
	return dcc_off;
}


#endif /* DCC_TIMER_H_ */
