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
	uint8_t input_mask;
	uint8_t modified;
	uint32_t time_stamp;

  protected:

  public:
    I2c_Port(uint8_t i2c_address);
    void write_i2c (void);
    void read_i2c (void);
    void set_input_i2c(void);

    void set_input(uint8_t mask);
    void clear_mask (uint8_t mask);
    void set_mask (uint8_t mask);
    uint8_t read(void);
    uint8_t read(uint32_t & time_stamp);
    void write(uint8_t data);
};

inline uint8_t I2c_Port::read(void) {
	return current_value;
}
inline uint8_t I2c_Port::read(uint32_t & _time_stamp) {
	_time_stamp = time_stamp;
	return current_value;
}
inline void I2c_Port::clear_mask (uint8_t x) {
	current_value = (~(x) & current_value) | input_mask;
	modified = 1;
}
inline void I2c_Port::set_mask(uint8_t x) {
	current_value = x | current_value | input_mask;
	modified = 1;
}
inline void I2c_Port::write(uint8_t data) {
	current_value = data;
	modified = 1;
}


#endif /* I2C_KEYBOARD_H_ */
