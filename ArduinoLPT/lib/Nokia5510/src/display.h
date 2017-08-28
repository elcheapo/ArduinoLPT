/*
 * display.h
 *
 *  Created on: 12 avr. 2013
 *      Author: florrain
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#define NUM_LINES 6
#define NUM_COL 12

class Display : public Print {
  private:
  protected:
	uint8_t lcd_update;
    uint8_t led_line;
    uint8_t pseudo_led_value[10];
    uint8_t current_x;
    uint8_t current_y;
    uint8_t lcd_text[NUM_LINES][NUM_COL];

  public:
    Display(uint8_t led);
    size_t write(uint8_t);
    void pseudo_led(uint8_t led,uint8_t on_off);
    void clear(void);
    void clearline(void);
    uint8_t get_pseudo_led (uint8_t no);
    void go(uint8_t x, uint8_t y);
    void bar(uint8_t col, uint8_t heigh, uint8_t max);
    void bar(uint8_t col, uint8_t heigh);
    void menu(const __FlashStringHelper *line1, \
    		const __FlashStringHelper *line2, \
    		const __FlashStringHelper *line3, \
    		const __FlashStringHelper *line4, \
    		const __FlashStringHelper *line5, \
    		const __FlashStringHelper *line6);

};



#endif /* DISPLAY_H_ */
