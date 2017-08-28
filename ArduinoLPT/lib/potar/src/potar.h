/*
 * potar.h
 *
 *  Created on: 23 août 2017
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
    uint16_t get(void);
    void read_A_pin();
};


#endif /* LIB_POTAR_SRC_POTAR_H_ */
