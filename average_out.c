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
 * @file acc-example.c
 * @author Enrico Joerns <e.joerns@tu-bs.de>
 */

#include <stdio.h>
#include "contiki.h"
#include "acc-sensor.h"
#include "gyro-sensor.h"
#include "button-sensor.h"
#define TAKT 800

/*---------------------------------------------------------------------------*/
PROCESS(acc_process, "Accelerometer process");
AUTOSTART_PROCESSES(&acc_process);
/*---------------------------------------------------------------------------*/
static struct etimer timer;
PROCESS_THREAD(acc_process, ev, data)
{
    PROCESS_BEGIN();

    // just wait shortly to be sure sensor is available
    etimer_set(&timer, CLOCK_SECOND * 0.05);
    PROCESS_YIELD();

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

    static const struct sensors_sensor *gyro_sensor;
    gyro_sensor = sensors_find("Gyro");

    {
        // get pointer to sensor
        // activate and check status
        uint8_t status = SENSORS_ACTIVATE(*gyro_sensor);
        if (status == 0) {
            printf("Error: Failed to init gyroscope, aborting...\n");
            PROCESS_EXIT();
        }

        // configure
        gyro_sensor->configure(GYRO_CONF_SENSITIVITY, GYRO_250DPS);
        gyro_sensor->configure(GYRO_CONF_DATA_RATE, GYRO_800HZ);
    }

    SENSORS_ACTIVATE(button_sensor);

    etimer_set(&timer, CLOCK_SECOND * (1/TAKT));

    
    static int on;
    static int round = 0;
    static int i;
    static int16_t gx_values[TAKT/100];
    static int16_t gy_values[TAKT/100];
    static int16_t gz_values[TAKT/100];
    static int16_t avgx, avgy, avgz;

    while (1) {
        if (round == TAKT/100){  
            for (i=0; i<(TAKT/100); i++){
                avgx += gx_values[i];
                avgy += gy_values[i];
                avgz += gz_values[i];
            }
            printf("Average Gyro: %d %d %d \n", avgx/(TAKT/100), avgy/(TAKT/100), avgz/(TAKT/100));
            round = 0;
            avgx = 0;
            avgy = 0;
            avgz = 0;
        }
        if(ev == sensors_event && data == &button_sensor) {
            on = !on;
        } else if(ev == PROCESS_EVENT_TIMER) {
            // read and output values
            int16_t ax = acc_sensor->value(ACC_X);
            int16_t ay = acc_sensor->value(ACC_Y);
            int16_t az = acc_sensor->value(ACC_Z);
            int16_t gx = gyro_sensor->value(GYRO_X);
            int16_t gy = gyro_sensor->value(GYRO_Y);
            int16_t gz = gyro_sensor->value(GYRO_Z);
            gx_values[round] = gx;
            gy_values[round] = gy;
            gz_values[round] = gz;
            printf("%d %d %d %d %d %d %d \n", ax, ay, az, gx, gy, gz, on);

            etimer_reset(&timer);
        }
        PROCESS_YIELD();
        // PROCESS_PAUSE();
    round ++;
    }

    // deactivate (never reached here)
    SENSORS_DEACTIVATE(*acc_sensor);
    SENSORS_DEACTIVATE(*gyro_sensor);
    SENSORS_DEACTIVATE(button_sensor);

    PROCESS_END();
}

