/*
 * Copyright (c) 2014, TU Braunschweig
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
 */

/**
 * @file zeigeaccel.c
 * @author Vincent Breitmoser <v.breitmoser@tu-bs.de>
 */

#include <stdio.h>
#include "contiki.h"
#include "net/rime.h"
#include "acc-sensor.h"
#include "button-sensor.h"
#include "button-sensor2.h"

#include "keycodes.h"

typedef struct {
    int8_t x, y;
    uint8_t button;
    uint8_t modifier;
    uint8_t key;
} buf_xy_t;
static buf_xy_t buf_xy;

/*---------------------------------------------------------------------------*/
PROCESS(accel_process, "accel process");
AUTOSTART_PROCESSES(&accel_process);
/*---------------------------------------------------------------------------*/
static struct etimer timer;
static const struct broadcast_callbacks broadcast_call = {};
static struct broadcast_conn broadcast;
PROCESS_THREAD(accel_process, ev, data)
{

    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);

    broadcast_open(&broadcast, 129, &broadcast_call);

    // just wait shortly to be sure sensor is available
    etimer_set(&timer, CLOCK_SECOND * 0.05);
    PROCESS_YIELD();

    // get pointer to sensor
    static const struct sensors_sensor *acc_sensor; {
        acc_sensor = sensors_find("Acc");

        // activate and check status
        uint8_t status = SENSORS_ACTIVATE(*acc_sensor);
        if (status == 0) {
            printf("Error: Failed to init accelerometer, aborting...\n");
            PROCESS_EXIT();
        }

        // configure
        acc_sensor->configure(ACC_CONF_SENSITIVITY, ACC_2G);
        acc_sensor->configure(ACC_CONF_DATA_RATE, ACC_100HZ);
    }

    // how long to wait for the fifo to fill
    etimer_set(&timer, CLOCK_SECOND * 0.03);

    while (1) {
        PROCESS_YIELD();

        if(ev == PROCESS_EVENT_TIMER && data == &timer) {

            static uint8_t state = 0;
            static struct {
                int16_t x;
                int16_t y;
            } acc_state;

            uint8_t key = 0;
            switch(state) {
                case 0:
                    if(!button_sensor.value(0))
                        break;
                    state = 1;
                    acc_state.x = acc_sensor->value(ACC_X);
                    acc_state.y = acc_sensor->value(ACC_Y);
                case 1:
                    printf("%d\t%d\n", acc_sensor->value(ACC_X), acc_sensor->value(ACC_Y));

                    // still pressed down?
                    if(button_sensor.value(0))
                        break;

                    acc_state.x -= acc_sensor->value(ACC_X);
                    acc_state.y -= acc_sensor->value(ACC_Y);

                    if(acc_state.x < -1000) {
                        printf("left\n");
                        key = HID_KEYBOARD_SC_LEFT_ARROW;
                    } else if(acc_state.x > 1000) {
                        printf("right\n");
                        key = HID_KEYBOARD_SC_RIGHT_ARROW;
                    } else if(acc_state.y < -700) {
                        printf("forward\n");
                        key = HID_KEYBOARD_SC_UP_ARROW;
                    } else if(acc_state.y > 700) {
                        printf("back\n");
                        key = HID_KEYBOARD_SC_DOWN_ARROW;
                    }

                    // check for gestures
                    printf("> %d\t%d\n", acc_state.x, acc_state.y);

                    // reset all values
                    acc_state.x = acc_state.y = 0;
                    state = 0;
            }

            // if any key fell out of that stuff above, send it to the raven
            if(key) {
                buf_xy.x = buf_xy.y = 0;
                buf_xy.modifier = 0;
                buf_xy.key = key;
                packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                broadcast_send(&broadcast);
            }

            // reset timer
            etimer_reset(&timer);
        }
    }

    PROCESS_END();
}

