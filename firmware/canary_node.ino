/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses call-backs on the event and RX so there's no data handling in the main loop!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include "DHT.h"
#include "COS_MQ7.h"
#include <stdlib.h>
#include <string.h>
#define __VERSION__ "v0.1"
//#define __DEBUG__

// BLE Device Pins
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 3
#define ADAFRUITBLE_RST 9

// Setup DHT Humidity and Temperature Sensor
#define DHTPIN 2
#define DHTTYPE DHT11
#define MAX_TEMP 50.0 //deg C
#define MIN_TEMP 0 //deg C
#define MAX_TEMP_INTVALUE 0xffff //deg C

// Setup Pins for VUMeter/
#define VUMETER_RST_PIN    (7)
#define VUMETER_ANALOG_PIN (A0)

// Pin definitions for carbon monixide sensor
#define ACTIVE_MONOX_PIN 6
#define READ_MONOX_PIN A4
#define READ_COPSV_PIN A3
#define ACTIVE_MONOX_LED_PIN A2

// RL = 10.0kOhms
// create CO sensor object
// Last parameter is the duration of initial purge in seconds, negative value sets to default 500 seconds
COS_MQ7 MQ7(ACTIVE_MONOX_LED_PIN, ACTIVE_MONOX_PIN, READ_MONOX_PIN, READ_COPSV_PIN, -1);

// Initialize Global variables.
Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
DHT dht(DHTPIN, DHTTYPE); 
uint16_t vumeter_value = 0;

void setupVUMeter() {
  pinMode(VUMETER_RST_PIN, OUTPUT);
  digitalWrite(VUMETER_RST_PIN,LOW);
  
  // Setup Timer Interrupts on Timer 1
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

uint16_t asb_uartGetTemperature() {
  float t = dht.readTemperature(); // deg C
  #ifdef __DEBUG__
  Serial.println(t);
  #endif

  return ((t-MIN_TEMP)/(MAX_TEMP-MIN_TEMP)) * (MAX_TEMP_INTVALUE);
}

uint16_t asb_uartGetHumidity() {
  float h = dht.readHumidity();
  
  #ifdef __DEBUG__
  Serial.println(h);
  #endif
  
  return (h/100)*0xffff;
}

uint16_t asb_uartGetVU() {
  return vumeter_value;
}

static uint16_t CO_value = 0;
static uint8_t CO_valid = 0;

uint16_t asb_uartGetCO() {
  return CO_value;
}

uint16_t asb_uartGetCH4() {
  return 0xd3ad;
}

/**************************************************************************/
/*!
    This function is called whenever select ACI events happen
*/
/**************************************************************************/
void aciCallback(aci_evt_opcode_t event)
{
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      #ifdef __DEBUG__
      Serial.println(F("Advertising started"));
      #endif
      break;
    case ACI_EVT_CONNECTED:
      #ifdef __DEBUG__
      Serial.println(F("Connected!"));
      #endif
      break;
    case ACI_EVT_DISCONNECTED:
      #ifdef __DEBUG__
      Serial.println(F("Disconnected or advertising timed out"));
      #endif
      break;
    default:
      break;
  }
}

/**************************************************************************/
/*!
    This function is called whenever data arrives on the RX channel
*/
/**************************************************************************/
void rxcallback(uint8_t *buffer, uint8_t len)
{
  static uint8_t txBuf[125];
  static float data[4];
  #ifdef __DEBUG__
  Serial.print(F("Received "));
  Serial.print(len);
  Serial.print(F(" bytes: "));
  for(int i=0; i<len; i++)
   Serial.print((char)buffer[i]); 

  Serial.print(F(" ["));
  
  for(int i=0; i<len; i++)
  {
    Serial.print(" 0x"); Serial.print((char)buffer[i], HEX); 
  }
  Serial.println(F(" ]"));
  #endif
  
  for (uint8_t i = 0; i < len; i++) {
    if (buffer[i] == 'u') {
      sprintf((char*)txBuf, "%04x,%04x,%04x,%04x", 
                            asb_uartGetTemperature(),
                            asb_uartGetHumidity(),
                            asb_uartGetVU(),
                            asb_uartGetCO());      
      CO_valid = 0;
      uart.write(txBuf,strlen((char *)txBuf));
    } else if (buffer[i] == 'a') {
      uart.write((uint8_t *)"a", 1);    
    }
  }
}


/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  setupVUMeter();
  
  #ifdef __DEBUG__
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Callback Echo demo"));
  #endif
  
  uart.setRXcallback(rxcallback);
  uart.setACIcallback(aciCallback);
  uart.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
void loop()
{
  uart.pollACI();
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  static uint8_t counter = 0;
  static uint16_t co_counter = 0;
  static uint16_t vumeter_filter = 0;
  if (co_counter == 510) {
    co_counter = 0;
    MQ7.Power_cycle();
    
      Serial.print(MQ7.Get_state());
  Serial.print(',');
  Serial.print(MQ7.Get_Voltage_reading());
  Serial.print(',');
  Serial.println(MQ7.Get_current_CO_reading());
    // Only record 'good' reading
    if(MQ7.Get_state() == 4) {
      CO_valid = 1;
      CO_value = MQ7.Get_CO_reading();
    }

  } else {
    co_counter++;
  }
  if (counter == 200) {
    vumeter_value = vumeter_filter;
    vumeter_filter = 0;
    digitalWrite(VUMETER_RST_PIN, HIGH);
  } else if (counter < 200) {
    digitalWrite(VUMETER_RST_PIN, LOW);
    vumeter_filter = ((uint16_t)analogRead(VUMETER_ANALOG_PIN) + (uint16_t)vumeter_filter)/2;

  }
  counter++;
}

