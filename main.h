#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

uint8_t get_leds();
void set_leds(uint8_t leds);

void send_msg(uint8_t code, uint16_t msg);
uint8_t read_msg(uint8_t *buf);

uint8_t process_msg(uint8_t *msg);

#endif
