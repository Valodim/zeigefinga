/*
 * Copyright (c) 2013, TU Braunschweig
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/** 
 * \file 
 *      Button sensor routine
 * @author:
 *      Georg von Zengen
 *      Enrico Joerns
 */

#include "lib/sensors.h"
#include "button-sensor2.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define BUTTON_INT_vect   PCINT0_vect
#define BUTTON_ISC        PCIE0
#define BUTTON_INT        PCINT0
#define BUTTON_PORT       PORTA
#define BUTTON_DDR        DDRA
#define BUTION_PIN        PINA
#define BUTTON_P0         PA0
#define BUTTON_P1         PA1
#define BUTTON_REG_EICR   PCICR
#define BUTTON_REG_EIMSK  PCMSK0

#ifndef DEBOUNCE_TIME
#define DEBOUNCE_TIME     CLOCK_SECOND / 40
#endif

const struct sensors_sensor button_sensor2;
static int status(int type);
static struct timer debouncetimer;
/*----------------------------------------------------------------------------*/
ISR(BUTTON_INT_vect) {
  if (timer_expired(&debouncetimer)) {
    timer_reset(&debouncetimer);
    sensors_changed(&button_sensor2);
  }
}
static void
init() {
  BUTTON_DDR &= ~(1 << BUTTON_P0 | 1 << BUTTON_P1);
  BUTTON_PORT |= (1 << BUTTON_P0 | 1 << BUTTON_P1);
  BUTTON_REG_EICR |= (0x1 << BUTTON_ISC); // 1 = any edge
}
/*----------------------------------------------------------------------------*/
static int
value(int type) {
  switch(type) {
    case 0:
      return (1 - ((BUTION_PIN & (1 << BUTTON_P0)) >> BUTTON_P0));
    case 1:
      return (1 - ((BUTION_PIN & (1 << BUTTON_P1)) >> BUTTON_P1));
  }
  return 0;
}
/*----------------------------------------------------------------------------*/
static int
configure(int type, int c) {
  switch (type) {
    case SENSORS_HW_INIT:
      init();
      return 1;
    case SENSORS_ACTIVE:
      // activate sensor
      if (c) {
        if (!status(SENSORS_ACTIVE)) {
          timer_set(&debouncetimer, DEBOUNCE_TIME);
          init();
          BUTTON_REG_EIMSK |= (1 << BUTTON_INT);
          sei();
        }
        // deactivate sensor
      } else {
        BUTTON_REG_EIMSK &= ~(1 << BUTTON_INT);
      }
      return 1;
  }
  return 0;
}
/*----------------------------------------------------------------------------*/
static int
status(int type) {
  switch (type) {
    case SENSORS_ACTIVE:
      return (BUTTON_REG_EIMSK & (1 << BUTTON_INT));
      break;
    case SENSORS_READY:
      return ~(BUTTON_DDR & (1 << BUTTON_P0 | 1 << BUTTON_P1));
  }

  return 0;
}
/*----------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor2, BUTTON_SENSOR2,
        value, configure, status);

