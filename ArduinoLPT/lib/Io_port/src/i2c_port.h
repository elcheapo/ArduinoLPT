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
	uint8_t read_value;
	uint8_t write_value;
	uint8_t input_mask;
	uint8_t modified;
	uint8_t disabled;
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

typedef struct {
	I2c_Port * port;
	uint8_t mask;
} t_io;

inline uint8_t I2c_Port::read(void) {
	return read_value;
}
inline uint8_t I2c_Port::read(uint32_t & _time_stamp) {
	_time_stamp = time_stamp;
	return read_value;
}
inline void I2c_Port::clear_mask (uint8_t x) {
	write_value = (~(x) & write_value);
#ifdef DEBUG
	Serial.print(F("Clear: 0x"));
	Serial.print(x,16);
	Serial.print(F(" / 0x"));
	Serial.println(write_value,16);
#endif
	modified = 1;
}
inline void I2c_Port::set_mask(uint8_t x) {
	write_value = x | write_value;
#ifdef DEBUG
	Serial.print(F("Set: 0x"));
	Serial.print(x,16);
	Serial.print(F(" / 0x"));
	Serial.println(write_value,16);
#endif
	modified = 1;
}
inline void I2c_Port::write(uint8_t data) {
	write_value = data;
	modified = 1;
}
inline void I2c_Port::set_input(uint8_t mask) {
	input_mask |= mask;
}

extern void get_t_io(t_io &io_port, const t_io * flash_tio);

#endif /* I2C_KEYBOARD_H_ */
