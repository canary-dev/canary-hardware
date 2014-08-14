#ifndef __ASB_UARTINTERFACE_H__
#define __ASB_UARTINTERFACE_H__

#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <stdint.h>

typedef enum asb_UARTStates {
  IDLE = 0,  // IDLE state, when the interface is just waiting
  ACTIVE,    // Just activated
  SEND_DATA, // Sending data
  DIAGNOSTIC // Diagnostic state
} asb_UARTStates;
#endif
