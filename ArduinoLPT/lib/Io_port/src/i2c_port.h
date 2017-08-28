/*
 * i2c_port.h
 *
 *  Created on: 20 juin 2012
 *      Author: florrain
 */

#ifndef I2C_PORT_H_
#define I2C_PORT_H_

#include <Wire.h>

class I2c_Port : public Io_Port {
  private:
	uint8_t i2c_address;
	uint8_t current_value;

  protected:

  public:
    I2c_Port(uint8_t i2c_address);
    void write (uint8_t value);
    void clear_mask (uint8_t mask);
    void set_mask (uint8_t mask);
    uint8_t read (void);

};



#endif /* I2C_KEYBOARD_H_ */
