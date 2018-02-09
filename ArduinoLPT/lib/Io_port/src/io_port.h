/*
 * io_port.h
 *
 *  Created on: Nov 3, 2013
 *      Author: francois
 */

#ifndef IO_PORT_H_
#define IO_PORT_H_


class Io_Port {
  private:

  public:
    virtual void write (uint8_t value);
    virtual void clear_mask (uint8_t mask);
    virtual void set_mask (uint8_t mask);
    virtual uint8_t read(void);
    virtual uint8_t read(uint32_t & time_stamp);
    virtual void set_input(uint8_t mask);
};




#endif /* IO_PORT_H_ */
