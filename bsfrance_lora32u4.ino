/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 * 29 December 2016 Modified by Zaki to cater for RF95 + Arduino UNO
 * 01 Aug 2019      Modified to use with BSFrance LoRa32u4
 *******************************************************************************/

#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <avr/sleep.h>
#include "LowPower.h"

#define VBATPIN A9
#define WAKEUPPIN 3  //Provisioned to put an ON/OFF switch later
#define whiteLED 13

int wakeup_by_pin = 0;
int datasent_count = 2;

   
/********************************* ABP Section : Need to fill these *************************************************************************/
static const PROGMEM u1_t NWKSKEY[16] = { 0x8E, 0xD9, 0xBE, 0xB0, 0xF6, 0x78, 0xD6, 0xDC, 0xFE, 0xC2, 0x2E, 0x71, 0x9C, 0xE1, 0x34, 0xDC };

static const u1_t PROGMEM APPSKEY[16] = { 0xB0, 0xE1, 0x1C, 0x22, 0x3F, 0xAC, 0x5F, 0xC4, 0xC6, 0x92, 0x02, 0x93, 0x47, 0x71, 0x76, 0x10 };

static const u4_t DEVADDR = 0xZZZZZZ ; // <-- Change this address for every node!
   
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/********************************************* ABP Section Ends ***************************************************************************/

static osjob_t sendjob;

/* Schedule TX every this many seconds */
const unsigned TX_INTERVAL = 5;  //every 5sec

/* Pin mapping */
const lmic_pinmap lmic_pins = {
        .nss = 8,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 4,
        .dio = {7, 5, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
    Serial.println();
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            delay(100);
            digitalWrite(whiteLED,LOW);
            if(LMIC.dataLen) {
                 // data received in rx slot after tx
                 Serial.print(F("Received "));
                 Serial.print(LMIC.dataLen);
                 Serial.print(F(" bytes of payload: 0x"));
                 for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 0x10) 
                    {
                        Serial.print(F("0"));
                    }
                    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                    
                    if (i==0) //check the first byte
                    {
                      if (LMIC.frame[LMIC.dataBeg + 0] == 0x00)
                      {
                          Serial.print(F(" Yes!!!! "));
                      }
                    }
                    
                 }
                 Serial.println();
            }

            if(datasent_count == 0)
            {
               datasent_count = 2;
               Serial.println("I am going to sleep now....");
               delay(100);
          
               // Disable USB clock 
               USBCON |= _BV(FRZCLK);
               // Disable USB PLL
               PLLCSR &= ~_BV(PLLE); 
               // Disable USB
               USBCON &= ~_BV(USBE); 
               // Add your wake up source here, example attachInterrupt()

               for(int i=0; i<75; i++)  //sleep for around 10 minutes
               {
                    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
               }
   
               USBDevice.attach(); 
               // In order for PC to have enough time to show the connected COM port
               delay(1000); // In order for PC to have enough time to show the connected COM port
            }
            else
            {
               datasent_count--;
               Serial.print("Need to send : ");
               Serial.print(datasent_count);
               Serial.print(" more time(s) before sleep.");
               delay(100); 
            }
            
            /* Schedule next transmission */
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
                        
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            /* data received in ping slot */
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    
    static uint8_t message[2];
    
    /* Check if there is not a current TX/RX job running */
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /* Prepare upstream data transmission at the next possible time. */          

        Serial.println(" ");  

        charger(1); // to enable charger and measure battery voltage below
        
        float measuredvbat = analogRead(VBATPIN);
        measuredvbat *= 4.2;  // Multiply by 4.2V, our max batt voltage
        measuredvbat /= 1024; // convert to voltage
        Serial.print("VBat: " ); Serial.println(measuredvbat);

        /************** For Battery ************/
        int16_t battery = measuredvbat * 100;  
        message[0] = highByte(battery);
        message[1] = lowByte(battery);
        /****************************************************/
        
        LMIC_setTxData2(1, message, sizeof(message), 0);     
        Serial.println();   
        Serial.println(F("Packet queued"));
        /*Print Freq being used*/
        Serial.print("Transmit on Channel : ");Serial.println(LMIC.txChnl);
        /*Print mV being supplied*/
        digitalWrite(whiteLED,HIGH);
                
    }
    /* Next TX is scheduled after TX_COMPLETE event. */
}

void setup() {
    delay(3000);
    Serial.begin(9600);
    Serial.println(F("Starting"));
    delay(10000);
    
    pinMode(13, OUTPUT);                  
    pinMode(VBATPIN,INPUT);

    pinMode(WAKEUPPIN, INPUT_PULLUP);  
    attachInterrupt(digitalPinToInterrupt(WAKEUPPIN), wakeUpNow, RISING);
        
    LoraInitialization();  // Do all Lora Init Stuff

    /* Start job */
    do_send(&sendjob);
}

void loop() {  
    os_runloop_once();   
}


void LoraInitialization(){
  /* LMIC init */
    os_init();
    /* Reset the MAC state. Session and pending data transfers will be discarded. */
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    /****************** ABP Only uses this section *****************************************/
    /* Set static session parameters. Instead of dynamically establishing a session
       by joining the network, precomputed session parameters are be provided.*/
    #ifdef PROGMEM
    /* On AVR, these values are stored in flash and only copied to RAM
       once. Copy them to a temporary buffer here, LMIC_setSession will
       copy them into a buffer of its own again. */
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    /* If not running an AVR with PROGMEM, just use the arrays directly */
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    /******************* ABP Only Ends ****************************************************/

    /* Disable link check validation */
    LMIC_setLinkCheckMode(0);

    /* Set data rate and transmit power (note: txpow seems to be ignored by the library) */
    LMIC_setDrTxpow(DR_SF7,14);  //lowest Datarate possible in 915MHz region
}


void charger(bool on) {
  DDRB |= (1<<PB0); // set PBO as output

  if(on) {
    PORTB |= (1<<PB0); 
  } else {
    PORTB &= ~(1<<PB0); 
  } 
}

void sleepNow() {  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here  
    sleep_enable();          // enables the sleep bit in the mcucr register  
    attachInterrupt(1,wakeUpNow, RISING); // use interrupt 1 (pin 3) and run function  
    wakeup_by_pin = 0;       // set a global variable to 0 before sleep
    sleep_mode();            // here the device is actually put to sleep!!  
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP  
    sleep_disable();         // first thing after waking from sleep: disable sleep...
    
    if(wakeup_by_pin == 1){  
        Serial.println(F("Woke up by pin ..."));  
        delay(100);  
        detachInterrupt(1);      // disables interrupt 1 on pin 3 so the wakeUpNow code will not be executed during normal running time. 
        LMIC_setStandby();
        LoraInitialization();
    }    
}  

void wakeUpNow() {   
  wakeup_by_pin = 1;
}  
