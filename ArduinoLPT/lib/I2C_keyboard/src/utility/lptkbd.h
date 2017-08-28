/*
 * keyboard.h
 *
 *  Created on: 12 avr. 2013
 *      Author: florrain
 */

#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include <stdint.h>

class Lptkbd {
  protected:
    volatile uint8_t column[4];

  public:
    Lptkbd(void);
    inline uint8_t get_column(uint8_t column_id);
    int8_t get_key(void);
    int8_t get_key_debounced(uint8_t & last);
    uint8_t get_star_line(void);
    int8_t get_star_line_debounced(uint8_t & last);
};

inline uint8_t Lptkbd::get_column(uint8_t column_id) {return column[column_id & 0x03] & 0x01f;};

#define KEY_F1 0xf1
#define KEY_F2 0xf2
#define KEY_STAR '*'
#define KEY_HASH '#'
#define KEY_ESC '['
#define KEY_ENT ']'
#define KEY_UP 'U'
#define KEY_DOWN 'D'
#define KEY_LEFT '<'
#define KEY_RIGHT '>'

#endif /* KEYBOARD_H_ */
