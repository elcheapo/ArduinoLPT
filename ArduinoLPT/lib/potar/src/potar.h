/*
 * potar.h
 *
 *  Created on: 23 ao�t 2017
 *      Author: florrain
 */

#ifndef LIB_POTAR_SRC_POTAR_H_
#define LIB_POTAR_SRC_POTAR_H_

class Potar {
  private:
    uint8_t pin;
    uint16_t value;

  public:
    Potar (uint8_t _pin);
    Potar (void);
    uint16_t get(void);
    void set_value(uint16_t set_value);
    void read_A_pin();
};


#endif /* LIB_POTAR_SRC_POTAR_H_ */
