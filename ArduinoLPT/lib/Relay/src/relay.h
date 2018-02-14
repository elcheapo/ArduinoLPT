/*
 * relay.h
 *
 *  Created on: Nov 3, 2013
 *      Author: francois
 */

#ifndef RELAY_H_
#define RELAY_H_


#include "io_port.h"
#include "i2c_port.h"

//#define OBJECTS
#define STRUCTS

#ifdef OBJECTS
class Relay {
private:
	class Io_Port * drive;
	uint8_t mask;
	uint8_t inverted; // if <>0 0 = relay on

public:
	Relay(Io_Port * drive, uint8_t mask, uint8_t inverted);
	void relay_on(void);
	void relay_off(void);
};

extern Relay relais[];
#endif

#ifdef STRUCTS

typedef struct {
	const t_io port;
	uint8_t inverted;
} relais_t;

extern const relais_t relais[] PROGMEM;

void relay_on(const relais_t * id);
void relay_off(const relais_t * id);

#endif
#endif /* RELAY_H_ */
