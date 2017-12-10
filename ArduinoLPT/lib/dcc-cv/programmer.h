/*
 * programmer.h
 *
 *  Created on: 10 janv. 2013
 *      Author: florrain
 */

#ifndef PROGRAMMER_H_
#define PROGRAMMER_H_

typedef enum {VOID, READY, BUSY, REGMODE, CVMODE, SHORT} t_lenz_result;
extern t_lenz_result lenz_result;          // This stores the last programming result

void lprog_send_prog_result(void);
void lprog_read_register(unsigned char regnr);
void lprog_write_register(unsigned char regnr, unsigned char data);

// Note: lenz only allows for 256 cv-regs :-(

extern void lprog_read_cv(unsigned char lenz_cv);
extern void lprog_write_cv(unsigned char lenz_cv, unsigned char data);

//extern bool select_programmer (DCC_timer * _timer, uint8_t _adc_program);
extern uint8_t direct_mode_bit_write(uint16_t cv, uint8_t bitpos, uint8_t mybit);
extern uint8_t direct_mode_bit_verify(uint16_t cv, uint8_t bitpos, uint8_t mybit);
extern uint8_t test_direct_mode(void);
extern uint8_t direct_mode_write(uint16_t cv, uint8_t data);
extern uint8_t direct_mode_verify(uint16_t cv, uint8_t data);
extern uint8_t factory_reset(void);
extern int16_t direct_mode_read(uint16_t cv);
extern int8_t programmer(message * prog_message);
extern bool set_programmer (DCC_timer * _timer, uint8_t _adc_channel);
extern void set_ack_level(uint8_t level);
extern uint8_t get_ack_level(void);
#endif /* PROGRAMMER_H_ */
