/*
 * aiguillage.h
 *
 *  Created on: Nov 3, 2013
 *      Author: francois
 */

#ifndef AIGUILLAGE_H_
#define AIGUILLAGE_H_

#include "io_port.h"
#include "i2c_port.h"
#include "relay.h"

typedef enum {s_unknown, s_droit, s_devie, s_dontcare} a_state;
typedef enum {t_peco, t_conrad} a_type;

class aiguille {
private:
	const relais_t * act_droit;
	const relais_t * act_devie;
	a_type type;
	bool locked;
protected:
public:
	a_state state;
	aiguille(const relais_t * _act_droit, const relais_t * _act_devie, a_type _type);
	bool set_state(a_state position);
	bool set_state_and_lock(a_state position);
	void unlock(void);
	bool is_locked(void);
	a_state get_state(void);
};

inline a_state aiguille::get_state(void) {
	return state;
}
inline bool aiguille::is_locked(void) {
	return locked;
}
#endif /* AIGUILLAGE_H_ */
