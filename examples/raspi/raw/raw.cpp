/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example transmits data on hardcoded channel and receives data
 * when not transmitting. Running this sketch on two nodes should allow
 * them to communicate.
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include "lib_crc.h"

#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
       config.h in the lmic library to set it.
#endif

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc 
#define TX_INTERVAL 1000 /* 2000 */

// Dragino Raspberry PI hat (no onboard led)
// see https://github.com/dragino/Lora
#define RF_CS_PIN  RPI_V2_GPIO_P1_22 // Slave Select on GPIO25 so P1 connector pin #22
#define RF_IRQ_PIN RPI_V2_GPIO_P1_07 // IRQ on GPIO4 so P1 connector pin #7
#define RF_RST_PIN RPI_V2_GPIO_P1_11 // Reset on GPIO17 so P1 connector pin #11

// Pin mapping
const lmic_pinmap lmic_pins = { 
    .nss  = RF_CS_PIN,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst  = RF_RST_PIN,
    .dio  = {RF_IRQ_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

#ifndef RF_LED_PIN
#define RF_LED_PIN NOT_A_PIN  
#endif


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) {
}

osjob_t txjob;
osjob_t timeoutjob;
static void tx_func (osjob_t* job);

// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func) {
  u2_t crc16 = 0xffff;  /* seed value */

  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;

  LMIC.frame[LMIC.dataLen] = 0x31;
  crc16 = update_crc_ccitt(crc16, LMIC.frame[LMIC.dataLen++]);
  LMIC.frame[LMIC.dataLen] = 0xFA;
  crc16 = update_crc_ccitt(crc16, LMIC.frame[LMIC.dataLen++]);
  LMIC.frame[LMIC.dataLen] = 0xB6;
  crc16 = update_crc_ccitt(crc16, LMIC.frame[LMIC.dataLen++]);

//  printf("crc16: ");
  while (*str) {
    LMIC.frame[LMIC.dataLen] = *str++;
//    printf("%02x", (u1_t)(LMIC.frame[LMIC.dataLen]));
    crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
    LMIC.dataLen++;
  }
//  printf(" : %x\n", crc16);

  LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
  LMIC.frame[LMIC.dataLen++] = (crc16      ) & 0xFF;

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  Serial.println("TX");
}

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RX /* RADIO_RXON */);
  Serial.println("RX");
}

static void rxtimeout_func(osjob_t *job) {
  digitalWrite(LED_BUILTIN, LOW); // off
}

static void rx_func (osjob_t* job) {

  u2_t crc16 = 0xffff;  /* seed value */
  u1_t i;

  // Blink once to confirm reception and then keep the led on
  digitalWrite(LED_BUILTIN, LOW); // off
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);

  // Reschedule TX so that it should not collide with the other side's
  // next TX
  os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);

  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");

  u4_t address = (LMIC.frame[0] << 16) | (LMIC.frame[1] << 8) | LMIC.frame[2] ;
  if (address == 0x31FAB6)
  {
  
    for (i=0; i<(LMIC.dataLen-2); i++)
    {
      printf("%02x", (u1_t)(LMIC.frame[i]));
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[i]));
    }
    u2_t pkt_crc = (LMIC.frame[i] << 8 | LMIC.frame[i+1]);
    if (crc16 == pkt_crc ) {
      printf(" %04x is valid crc", pkt_crc);
    } else {
      printf(" %04x is wrong crc", pkt_crc);
    }

  } else {
      printf(" %06x is wrong address", address);
  }
  Serial.println();  

  // Restart RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
//  printf("txdone_func\n");
  rx(rx_func);
}

// log text to USART and toggle LED
static void tx_func (osjob_t* job) {
  // say hello
  //tx("Hello, world!", txdone_func);
  tx("\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001\001", txdone_func);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}

// application entry point
int setup() {
  printf("Starting\n");

  // Init GPIO bcm
  if (!bcm2835_init()) {
      fprintf( stderr, "bcm2835_init() Failed\n\n" );
      return 1;
  }
//  printf("Starting 1\n");
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize runtime env
  os_init();
//  printf("Starting 2\n");
  // Set up these settings once, and use them for both TX and RX

#if defined(CFG_eu868)
  // Use a frequency in the g3 which allows 10% duty cycling.
  LMIC.freq = 868400000;
#elif defined(CFG_us915)
  LMIC.freq = 902300000;
#endif

  // Maximum TX power
  LMIC.txpow = 5;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then the interval should be (significantly)
  // lowered to comply with duty cycle limits as well.
  LMIC.datarate =  DR_FSK /*  DR_SF9  */ ;
  //LMIC.datarate =  DR_SF9 ;
  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  printf("Started\n");

  // setup initial job
  os_setCallback(&txjob, tx_func);

  return 0;
}

int main(void) {
  
  if (setup())
    return 1;

  printf("Loop\n");

  while(1) {
    // execute scheduled jobs and events
    os_runloop_once();
  }
}
