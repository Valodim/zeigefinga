#include "contiki.h"
#include "net/rime.h"
#include "dev/button-sensor.h"
#include "acc-sensor.h"
#include "gyro-sensor.h"
#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(rime_unicast_sender, "Rime Unicast Sender");
AUTOSTART_PROCESSES(&rime_unicast_sender);
/*---------------------------------------------------------------------------*/

static int com_counter = 0;
static int button_counter;
static int nodeID;
static struct etimer timer;

static void recv_uc(struct unicast_conn *c, const rimeaddr_t *from) {
  char *datapntr;
  datapntr = packetbuf_dataptr();
  datapntr[15] = '\0';
//  leds_arch_set(com_counter);
  if (com_counter == 0){ // first contact
    nodeID = from->u8[0];
    // printf("nodeID: %d", nodeID);
    com_counter ++;
  }
  else if (from->u8[0] == nodeID){ // you and only you
    com_counter ++;
    // printf("nodeID: %d", nodeID);
  }
  printf("unicast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], datapntr);
}

static struct unicast_conn uc;
static const struct unicast_callbacks unicast_callbacks = {recv_uc}; // List of Callbacks to be called if a message has been received.

PROCESS_THREAD(rime_unicast_sender, ev, data){
  PROCESS_EXITHANDLER(unicast_close(&uc));
  PROCESS_BEGIN();

  // just wait shortly to be sure sensor is available
  etimer_set(&timer, CLOCK_SECOND * 0.05);
  PROCESS_YIELD();

  leds_init();	
  SENSORS_ACTIVATE(button_sensor); // channel = 145

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


  while (1) {
    unicast_open(&uc, 146, &unicast_callbacks);
    rimeaddr_t addr;
    PROCESS_YIELD();
        
    if(ev == sensors_event && data == &button_sensor) {
//    on = !on;
    } else if(ev == PROCESS_EVENT_TIMER) {
        // read and output values
        int16_t ax = acc_sensor->value(ACC_X);
        int16_t ay = acc_sensor->value(ACC_Y);
        int16_t az = acc_sensor->value(ACC_Z);
        int16_t gx = gyro_sensor->value(GYRO_X);
        int16_t gy = gyro_sensor->value(GYRO_Y);
        int16_t gz = gyro_sensor->value(GYRO_Z);
        printf("%d %d %d %d %d %d %d \n", ax, ay, az, gx, gy, gz, on);

        uint16_t buffer [100] ; 
	      int ret = sprintf(buffer, "%d %d %d %d %d %d", ax, ay, ax, gx, gy, gz);
	      packetbuf_copyfrom(buffer, 100); // String + Length to be sennd
/* 
Benutzt man einen größeren Buffer, zb 128, sendet der Inga nichts mehr. Auch bei snprintf funktioniert es nicht. Bei der aktuellen
Größe geht es, allerdings fehlen die hinteren Werte...
Das "ON" habe ich grad rausgenommen, das kommt ja eh nicht mehr an.
*/

	      addr.u8[0] = nodeID; // Address of receiving Node
	      addr.u8[1] = 0;
	      if (!rimeaddr_cmp(&addr, &rimeaddr_node_addr)) {
//	      printf("Message sent\n"); // debug message
	        unicast_send(&uc, &addr);
	      }
        etimer_reset(&timer);
	    }
      button_counter ++;
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

