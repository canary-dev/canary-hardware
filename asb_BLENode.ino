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

#include "asb_UARTInterface.h"
#include <stdlib.h>
#include <string.h>
#define __VERSION__ "v0.1"
#define __DEBUG__

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 3
#define ADAFRUITBLE_RST 9

#define DHTPIN 2
#define DHTTYPE DHT11
#define MAX_TEMP 50.0 //deg C
#define MIN_TEMP 0 //deg C

#define MAX_TEMP_INTVALUE 0xffff //deg C

// Initialize Global variables.
Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
DHT dht(DHTPIN, DHTTYPE); 


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
  return 0x3f;
}

uint16_t asb_uartGetCO() {
  return 0xb3ef;
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
      
      asb_uartGetHumidity();
      asb_uartGetVU();
      asb_uartGetCO();
      uint8_t *txBuf_ptr = txBuf;
      
      sprintf((char*)txBuf, "%d,%d,%d,%d", 
                            asb_uartGetTemperature(),
                            asb_uartGetHumidity(),
                            asb_uartGetVU(),
                            asb_uartGetCH4());      

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
