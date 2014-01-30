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
 * @file gyro-direct-fifo.c
 * @author Vincent Breitmoser <v.breitmoser@tu-bs.de>
 */

#include <stdio.h>
#include "contiki.h"
#include "net/rime.h"
#include "acc-sensor.h"
#include "button-sensor.h"
#include "button-sensor2.h"
#include "l3g4200d.h"
typedef struct {
    int8_t x, y;
    uint8_t button;
    uint8_t modifier;
    uint8_t key;
} buf_xy_t;
static buf_xy_t buf_xy;

/*---------------------------------------------------------------------------*/
PROCESS(gyro_process, "gyro process");
AUTOSTART_PROCESSES(&gyro_process);
/*---------------------------------------------------------------------------*/
static struct etimer timer;
static const struct broadcast_callbacks broadcast_call = {};
static struct broadcast_conn broadcast;
PROCESS_THREAD(gyro_process, ev, data)
{

    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);
    SENSORS_ACTIVATE(button_sensor2);

    broadcast_open(&broadcast, 129, &broadcast_call);

    // just wait shortly to be sure sensor is available
    etimer_set(&timer, CLOCK_SECOND * 0.05);
    PROCESS_YIELD();

    // doing gyro initialization directly because fuck contiki
    if (l3g4200d_init() != 0) {
        printf("Error: Failed to init gyro / step 1, aborting...\n");
        PROCESS_EXIT();
    }

    // L3G4200D_250DPS // L3G4200D_500DPS // L3G4200D_2000DPS
    if(l3g4200d_set_dps(L3G4200D_250DPS) != 0) {
        printf("Error: Failed to init gyro / step 2, aborting...\n");
        PROCESS_EXIT();
    }

    // L3G4200D_ODR_100HZ // L3G4200D_ODR_200HZ // L3G4200D_ODR_400HZ // L3G4200D_ODR_800HZ
    if(l3g4200d_set_data_rate(L3G4200D_ODR_400HZ) != 0) {
        printf("Error: Failed to init gyro / step 3, aborting...\n");
        PROCESS_EXIT();
    }

    // L3G4200D_BYPASS - no fifo, just the single latest value
    // L3G4200D_STREAM - just keep streaming
    // L3G4200D_FIFO - break on fifo overrun
    // l3g4200d_set_fifomode(L3G4200D_BYPASS);
    l3g4200d_set_fifomode(L3G4200D_STREAM);
    l3g4200d_fifo_enable();

    // get pointer to sensor
    static const struct sensors_sensor *acc_sensor;
    acc_sensor = sensors_find("Acc");

    {
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

    int i, num;
    int16_t x, y, z;
    angle_data_t gyro_values[L3G4200D_FIFO_SIZE];

    static uint8_t state = 0;
    static struct {
        uint8_t num;
        int32_t x;
        int32_t y;
    } acc_state;
    acc_state.x = acc_state.y = 0;
    int16_t trust = 0;

    while (1) {
        PROCESS_YIELD();

        if(ev == PROCESS_EVENT_TIMER && data == &timer) {
            x = y = z = 0;

            // get fifo values
            num = l3g4200d_get_angle_fifo(gyro_values);
            // or emulate fifo data from bypass mode
            // gyro_values[0] = l3g4200d_get_angle(); num = 1;

#if 1
            // approach 1: accumulate and divide later
            // smoother movement, lack of a deadzone leads to shaky cursor
            for(i = 0; i < num; i++) {
                x += (gyro_values[i].x * 0.00875);
                y += (gyro_values[i].y * 0.00875);
            }
            x /= num; y /= num; z /= num;
#else
            // approach 2: divide individual values
            // works as a natural deadzone, which makes controls feel less but
            // allows keeping the cursor in position easily
            for(i = 0; i < num; i++) {
                x += (gyro_values[i].x * 0.00875) / num;
                y += (gyro_values[i].y * 0.00875) / num;
                z += (gyro_values[i].z * 0.00875) / num;
            }
#endif
            uint8_t key = 0;
            switch(state) {
                case 0:
                    if(!button_sensor.value(0))
                        break;
                    state = 1;
                    acc_state.num = 0;
                case 1:
                    trust = acc_sensor->value(ACC_Z);
                    printf("%d\t%d\t%d\n", acc_sensor->value(ACC_X), acc_sensor->value(ACC_Y), trust);
                    // roughly, trust the X and Y values if Z is > 0
                    if(trust < -700) {
                        acc_state.x += acc_sensor->value(ACC_X);
                        acc_state.y += acc_sensor->value(ACC_Y);
                        acc_state.num += 1;
                    }
                    // still pressed down?
                    if(button_sensor.value(0))
                        break;

                    acc_state.x /= acc_state.num;
                    acc_state.y /= acc_state.num;

                    // check for gestures
                    printf("> %ld\t%ld\t%d\n", acc_state.x, acc_state.y, acc_state.num);

                    // reset all values
                    acc_state.x = acc_state.y = 0;
                    state = 0;
            }

            if(key) {
                buf_xy.x = buf_xy.y = 0;
                buf_xy.key = key;
                packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                broadcast_send(&broadcast);
            }

#if 0
            if(button_sensor.value(0)) {
                buf_xy.x = z;
                buf_xy.y = -y;
                buf_xy.button = button_sensor2.value(1);
                packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                broadcast_send(&broadcast);
            }
#endif
            // reset timer
            etimer_reset(&timer);
        }
    }

    PROCESS_END();
}

