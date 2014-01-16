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
#include <math.h>
#include "contiki.h"
#include "acc-sensor.h"
#include "gyro-sensor.h"
#include "button-sensor.h"
 
#define dt 0.01 // 10 ms sample rate!
 
void ComplementaryFilter(short accData[3], short gyrData[3], double *pitch, double *roll)
{
    double pitchAcc, rollAcc;
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0]) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1]) * dt;    // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (1 || forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        if(pitchAcc < 0)
            pitchAcc = (360+pitchAcc);
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        if(rollAcc < 0)
            pitchAcc = (360+rollAcc);
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
} 

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
        gyro_sensor->configure(GYRO_CONF_DATA_RATE, GYRO_100HZ);
    }

    SENSORS_ACTIVATE(button_sensor);

    etimer_set(&timer, CLOCK_SECOND * 0.01);

    static int on;
    static short accData[3], gyroData[3];
    static double pitch = 0, roll = 0;

    while (1) {

        if(ev == sensors_event && data == &button_sensor) {
            on = !on;
        } else if(ev == PROCESS_EVENT_TIMER) {

            accData[0] = acc_sensor->value(ACC_X);
            accData[1] = acc_sensor->value(ACC_Y);
            accData[2] = acc_sensor->value(ACC_Z);
            gyroData[0] = gyro_sensor->value(GYRO_X);
            gyroData[1] = gyro_sensor->value(GYRO_Y);
            gyroData[2] = gyro_sensor->value(GYRO_Z);

            ComplementaryFilter(accData, gyroData, &pitch, &roll);
            printf("%d %d\n", (int) (pitch), (int) (roll));

            // read and output values
            // printf("%d %d %d %d %d %d %d\n",
            //         acc_sensor->value(ACC_X),
            //         acc_sensor->value(ACC_Y),
            //         acc_sensor->value(ACC_Z),
            //         gyro_sensor->value(GYRO_X),
            //         gyro_sensor->value(GYRO_Y),
            //         gyro_sensor->value(GYRO_Z),
            //         on);

            etimer_reset(&timer);
        }

        PROCESS_YIELD();
        // PROCESS_PAUSE();

    }

    // deactivate (never reached here)
    SENSORS_DEACTIVATE(*acc_sensor);
    SENSORS_DEACTIVATE(*gyro_sensor);
    SENSORS_DEACTIVATE(button_sensor);

    PROCESS_END();
}

