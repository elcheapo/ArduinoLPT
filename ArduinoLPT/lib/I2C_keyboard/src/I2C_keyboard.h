/*
 * i2c_keyboard.h
 *
 *  Created on: 20 juin 2012
 *      Author: florrain
 */

#ifndef I2C_KEYBOARD_H_
#define I2C_KEYBOARD_H_

#include <stdint.h>
#include "utility/lptkbd.h"

class I2c_Keyboard : public Lptkbd {
  private:
    uint8_t i2c_address;

  public:
    I2c_Keyboard(uint8_t _i2c_address);
    void scan(void);
    void scan_col(uint8_t col);
};


#endif /* I2C_KEYBOARD_H_ */
