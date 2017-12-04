/*
 * nokia5510.h
 *
 *  Created on: Feb 19, 2011
 *      Author: francois
 */

#ifndef NOKIA5510_H_
#define NOKIA5510_H_

#include "Print.h"
#include "display.h"

/*
This Code has extra features
including a XY positioning function on Display
and a Line Draw function on Nokia 3310 LCD
It is modded from the original
http://www.arduino.cc/playground/Code/PCD8544
*/

#define NUM_LINES 6
#define NUM_COL 12

void lcd_reset(void);

class Nokia5510 : public Display {
  private:
	uint8_t pin_ss;
    uint8_t pin_cd;
    uint8_t pin_rst;
//    volatile uint8_t * _lcdcs_port;
    void LcdWrite(uint8_t dc, uint8_t data);
    void gotoXY(uint8_t x, uint8_t y);
    void lcd_print(uint8_t character);


  public:
    Nokia5510(uint8_t _pin_ss, uint8_t pin_cd, uint8_t pin_rst);
    void begin(void);
    void end(void);
    void update(void);
    void low_level_clear(void);
    void lcd_reset(void);

    void normal(void);
    void inverse(void);


};

#define LED_HALF 2
#define LED_ON 1
#define LED_OFF 0


#endif /* NOKIA5510_H_ */
