
/*
 * set_nodeid.c
 *
 *  Created on: 13.10.2011
 *      Author: Georg von Zengen 

 *	to set node_id, give NODE_ID=<x> with make
 *
 */

#include "contiki.h"
#ifdef INGA_REVISION
#include <avr/eeprom.h>
#include "settings.h"
#include <util/delay.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "node-id.h"
#include "leds.h"
#include "button-sensor2.h"


#define DEBUG                                   0

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(default_app_process, "Hello world process");
AUTOSTART_PROCESSES(&default_app_process);
/*---------------------------------------------------------------------------*/
static struct etimer timer;
PROCESS_THREAD(default_app_process, ev, data)
{
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor2);

  leds_init();	
  leds_on(2);
  leds_on(1);
  etimer_set(&timer,  CLOCK_SECOND*0.05);
  while (1) {

    PROCESS_YIELD();
    if (ev == sensors_event && data == &button_sensor2) {
      leds_invert(1);
      printf("button interrupt!\n");
    }
    if ( etimer_expired(&timer)){
      etimer_set(&timer,  CLOCK_SECOND);
      printf("state: %d %d\n", button_sensor2.value(0), button_sensor2.value(1));
      leds_invert(2);
    }

  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
