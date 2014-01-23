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
#include "l3g4200d.h"
typedef struct {
    uint16_t x, y;
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
    if(l3g4200d_set_data_rate(L3G4200D_ODR_200HZ) != 0) {
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

    int i;
    static angle_data_t gyro_values[L3G4200D_FIFO_SIZE];
    static int16_t x, y, z;
    static int num = 0;

    while (1) {
        // if(ev == PROCESS_EVENT_TIMER)
        {
            num = l3g4200d_get_angle_fifo(gyro_values);
            // gyro_values[0] = l3g4200d_get_angle(); num = 1;
            x = y = z = 0;
            for(i = 0; i < num; i++) {
                // the factor is the raw -> dps transformation, values are
                // hardware and dps mode specific
                x += (gyro_values[i].x * 0.00875) / num;
                y += (gyro_values[i].y * 0.00875) / num;
                z += (gyro_values[i].z * 0.00875) / num;

                // TODO division first? leads to smaller values with some cutoff...
                // x += l3g4200d_raw_to_dps(gyro_values[i].x/num);
                // y += l3g4200d_raw_to_dps(gyro_values[i].y/num);
                // z += l3g4200d_raw_to_dps(gyro_values[i].z/num);
            }
            // TODO or maybe divide accumulated value afterwards?
            // x /= num; y /= num; z /= num;
            if(num > 0) {
                buf_xy.x = -z;
                buf_xy.y = -y;
                if(abs(z) > 1 || abs(y) > 1) {
                    packetbuf_copyfrom(&buf_xy, sizeof(buf_xy_t));
                    broadcast_send(&broadcast);
                }
                printf("%d\t%d\t%d\t%d\n", num, x, y, z);
            }
            // reset timer
            etimer_reset(&timer);
        }
        PROCESS_YIELD();
        // PROCESS_PAUSE();
    }

    PROCESS_END();
}

