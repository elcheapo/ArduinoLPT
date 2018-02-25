/*
 * config.h
 *
 *  Created on: 4 d�c. 2017
 *      Author: florrain
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#define REGULATOR 0x01
#define REGULATOR_OFF (PORTB |= REGULATOR)
#define REGULATOR_ON (PORTB &= ~(REGULATOR))

#define BUZZER_PIN 3
//Radio defines
#define PIN_CSN 2
#define PIN_CE	4
// LCD display pins
//#define PIN_RST 5
//#define PIN_SS 6
//#define PIN_DC 7
#define PIN_RST 5
#define PIN_SS 6
#define PIN_DC 7

// ANALOG PIN
#define CURRENT_SENSE 0
#define POT 1

#define PROG_TRACK A3
#define MAIN_TRACK A2



#endif /* SRC_CONFIG_H_ */
