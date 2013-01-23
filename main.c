/* Copyright (c) 2013), Vincent Bernardoff <vb@luminar.eu.org>

* Permission to use, copy, modify, and/or distribute this software for
* any purpose with or without fee is hereby granted, provided that the
* above copyright notice and this permission notice appear in all
* copies.

* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
* WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
* AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
* DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
* PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
* TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
* PERFORMANCE OF THIS SOFTWARE.
*/


#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/fuse.h>
#include "uart.h"
#include "gobanproto.h"
#include "main.h"

#define F_CPU 4000000L
#define PHOTODIODES_DELAY 15L

#include <util/delay.h>

// Programming FUSEs

FUSES =
  {
    // Settings for a full swing crystal oscillator, slow startup time
    .low = (FUSE_SUT0 & FUSE_SUT1 & FUSE_CKSEL0 & FUSE_CKSEL1 & FUSE_CKSEL2),
    .high = HFUSE_DEFAULT,
    .extended = EFUSE_DEFAULT,
  };

uint16_t oldstate = 0; // 4x4 goban old state
uint16_t state    = 0; // 4x4 goban state

// Interrupts

// Check board when TIMER0 overflows
ISR(TIMER0_OVF_vect)
{
  oldstate = state; // Backup outdated state into oldstate
  state = 0; // Reseting current state;
  PORTB &= 0xf0; // Put PB[0-3] to 0, do not touch the rest

  PORTB |= 0x08;
  _delay_us(2*PHOTODIODES_DELAY);
  state |= (PINC & 0x0f) << 12;
  PORTB &= ~0x08;

  PORTB = 0x04;
  _delay_us(2*PHOTODIODES_DELAY);
  state |= (PINC & 0x0f) << 8;
  PORTB &= ~0x04;

  PORTB = 0x02;
  _delay_us(2*PHOTODIODES_DELAY);
  state |= (PINC & 0x0f) << 4;
  PORTB &= ~0x02;

  PORTB = 0x01;
  _delay_us(2*PHOTODIODES_DELAY);
  state |= (PIND & 0x0f);
  PORTB &= ~0x01;

  if (oldstate != state) // If state changed, notify endpoint
    send_msg(NEWSTATE, state);
}

// Helper functions

uint8_t get_leds()
{
  uint8_t ret = 0;

  ret |= (PINB & 0x18) << 3;
  ret |= (PINC & 0x18) << 1;
  ret |= (PIND & 0x3c) >> 2;

  return ret;
}

void set_leds(uint8_t leds)
{
  // First put all leds to 0
  PORTB &= ~0x18;
  PORTC &= ~0x18;
  PORTD &= ~0x3c;
  //////////////////////////

  if(leds & 0x80)
    PORTB |= (1 << PORTB5);
  if(leds & 0x40)
    PORTB |= (1 << PORTB4);
  if(leds & 0x20)
    PORTC |= (1 << PORTC5);
  if(leds & 0x10)
    PORTC |= (1 << PORTC4);

  if(leds & 0x08)
    PORTD |= (1 << PORTD5);
  if(leds & 0x04)
    PORTD |= (1 << PORTD4);
  if(leds & 0x02)
    PORTD |= (1 << PORTD3);
  if(leds & 0x01)
    PORTD |= (1 << PORTD2);
}

void send_msg(uint8_t code, uint16_t msg)
{
  uint8_t *msg_ar = (uint8_t*)&msg;
  uart0_putc(code);
  for(int i=0; i<2; i++)
    uart0_putc(msg_ar[i]);
}

uint8_t read_msg(uint8_t *buf)
{
  uint16_t ret;
  uint8_t *ret_data = (uint8_t*)&ret;

  for(int i=0; i<3; i++)
    {
    top:
      ret = uart0_getc();
      if(!ret_data[0]) // If no error
        buf[i] = ret_data[1];
      else // Handling of errors, TODO
        {
          switch (ret)
            {
            case UART_NO_DATA:
              sleep_mode();
              goto top;
            default: /* All other error cases*/
              send_msg(NAK, ret);
              uart0_flush(); // Flushing the receiving buffer, since
                             // we’re unable to continue anyway.
              return ret_data[0];
            }
        }
    }

  return 0;
}

void setup_timers()
{
  TIMSK0 = 1 << TOIE0; // Enable TIMER0 interrupt on overflow
  TCCR0B = (1 << CS00) || (1 << CS02); // TIMER0 frequency = clock/1024 Hz
}

uint8_t process_msg(uint8_t *msg)
{
  switch(msg[0])
    {
    case ACK: break;
    case NAK: break;
    case GETLEDS:
      send_msg(ACK, get_leds());
      break;
    case SETLEDS:
      set_leds(msg[1]);
      send_msg(ACK, 0);
      break;
    case GETSTATE:
      send_msg(ACK, state);
      break;
    default:
      send_msg(NAK, 0);
    }

  return 0;
}

int main()
{
  // Set ports direction
  DDRB = 0x30;
  DDRC = 0x3f;
  DDRD = 0x3c;
  //////////////////////

  uart0_init(UART_BAUD_SELECT(9600, F_CPU)); // Initialize UART.

  setup_timers(); // Setup timers
  sei();          // Globally enable interrupts

  uint8_t cmdbuf[3];
  uint8_t ret;

  // Main loop
  for(;;)
    {
      ret = read_msg(cmdbuf);
      if(!ret)
        process_msg(cmdbuf);
    }
}
