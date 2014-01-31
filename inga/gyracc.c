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
 * @file zeigefinga.c
 * @author Vincent Breitmoser <v.breitmoser@tu-bs.de>
 */

#include <stdio.h>
#include "contiki.h"
#include "net/rime.h"
#include "dev/acc-sensor.h"
#include "button-sensor2.h"
#include "l3g4200d.h"

#include "keycodes.h"


typedef struct {
    int8_t x, y;
    uint8_t button;
    uint8_t modifier;
    uint8_t key;
} buf_xy_t;
static buf_xy_t buf_xy;

/*---------------------------------------------------------------------------*/
PROCESS(finga_process, "zeigefinga process");
AUTOSTART_PROCESSES(&finga_process);
/*---------------------------------------------------------------------------*/
static struct etimer timer;
static const struct broadcast_callbacks broadcast_call = {};
static struct broadcast_conn broadcast;
PROCESS_THREAD(finga_process, ev, data)
{

    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor2);

    broadcast_open(&broadcast, 129, &broadcast_call);

    // just wait shortly to be sure sensor is available
    etimer_set(&timer, CLOCK_SECOND * 0.05);
    PROCESS_YIELD();

    // initialize accelerometer
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

    // how long to wait for the fifo to fill
    etimer_set(&timer, CLOCK_SECOND * 0.03);

    while (1) {
        PROCESS_YIELD();

        if(ev == PROCESS_EVENT_TIMER && data == &timer) {

            {// --- doubleclick ---
                static uint8_t button_state = 0;
                uint8_t button = 0;
                static struct etimer double_click;

                switch(button_state) {
                    case 0:
                        if(button_sensor2.value(1)){
                            etimer_set(&double_click, CLOCK_SECOND / 1);
                            button_state = 1;
                        }
                        break;
                    case 1:
                        // still pressed down?
                        if(!button_sensor2.value(1))
                              button_state = 2;
                        break;
                    case 2:
                        if(button_sensor2.value(1) && !etimer_expired(&double_click)){
                              button = 1;
                        }
                        button_state = 0;
                        break;
                }
                if(button){
                    buf_xy.button = button;
                    packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                    broadcast_send(&broadcast);
                    buf_xy.button = 0;
                    packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                    broadcast_send(&broadcast);
                }
            }

            { // --- accel ---

                static uint8_t state = 0;
                static struct {
                    int16_t x;
                    int16_t y;
                } acc_state;
                uint8_t key = 0;


                switch(state) {
                    case 0:
                        if(!button_sensor2.value(1))
                            break;
                        state = 1;
                        acc_state.x = acc_sensor->value(ACC_X);
                        acc_state.y = acc_sensor->value(ACC_Y);
                    case 1:
                        // still pressed down?
                        if(button_sensor2.value(1))
                            break;

                        acc_state.x -= acc_sensor->value(ACC_X);
                        acc_state.y -= acc_sensor->value(ACC_Y);

                        // check for gestures
                        if(acc_state.x < -1000){
                            key = HID_KEYBOARD_SC_LEFT_ARROW;
                            printf("left");
                        }
                        else if(acc_state.x > 1000){
                            key = HID_KEYBOARD_SC_RIGHT_ARROW;
                            printf("right");
                        }
                        else if(acc_state.y < -700)
                            key = HID_KEYBOARD_SC_UP_ARROW;
                        else if(acc_state.y > 700)
                            key = HID_KEYBOARD_SC_DOWN_ARROW;

                        // reset all values
                        acc_state.x = acc_state.y = 0;
                        state = 0;
                }

                if(key) {
                    buf_xy.x = buf_xy.y = buf_xy.modifier = 0;
                    buf_xy.key = key;
                    packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                    broadcast_send(&broadcast);
                }

            }

            { // --- gyro ---

                int i, num;
                int16_t x, y, z; x = y = z = 0;
                double x_factor, y_factor;
                angle_data_t gyro_values[L3G4200D_FIFO_SIZE];

                // get fifo values
                num = l3g4200d_get_angle_fifo(gyro_values);
                // or emulate fifo data from bypass mode
                // gyro_values[0] = l3g4200d_get_angle(); num = 1;

                // approach 1: accumulate and divide later
                // smoother movement, lack of a deadzone leads to shaky cursor
                for(i = 0; i < num; i++) {
                    x += (gyro_values[i].x * 0.00875);
                    y += (gyro_values[i].y * 0.00875);
                    z += (gyro_values[i].z * 0.00875);
                }
                x /= num; y /= num; z /= num;
                        
                { // --- acc factor ---

                    static struct {
                        int16_t x;
                        int16_t y;
                        int16_t z;
                    } acc_state;
                    acc_state.x = acc_sensor->value(ACC_X);
                    acc_state.y = acc_sensor->value(ACC_Y);
                    acc_state.z = acc_sensor->value(ACC_Z);
                    
                    double ratio = acc_state.x/acc_state.z;
                    printf("Ratio: %d \n", (int) (ratio*100));

                    if(ratio > 10 ){  // hard left turn
                        x_factor = 10;
                        y_factor = 0.1; 
                                        
                    }
                    if(ratio >= 1 && ratio <=10 ){ // ligth left turn
                        x_factor = 2;
                        y_factor = 0.5;                  
                    }
                    if(ratio >= -1 && ratio <=1 ){ // no turn
                        x_factor = 1;
                        y_factor = 1;                  
                    }
                    if(ratio >= -10 && ratio < -1 ){ // ligth righ turn
                        x_factor = 2;
                        y_factor = 0.5;                  
                    }
                    if(ratio < -10){ // right turn
                        x_factor = 2;
                        y_factor = 0.5;                  
                    }
                }

                if(num > 0) {
                    if(button_sensor2.value(0) && abs(z)+abs(y) > 0) {
                        buf_xy.x = z * x_factor;
                        buf_xy.y = -y * y_factor;
                        buf_xy.button = buf_xy.modifier = buf_xy.key = 0;
                        packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                        broadcast_send(&broadcast);
                    }
                }
            }

            // reset timer
            etimer_reset(&timer);
        }
    }

    PROCESS_END();
}

