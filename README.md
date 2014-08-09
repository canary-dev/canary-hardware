# asb_BLENode

Uses adafruits BLE UART library as a basis and the nRF8001 BLE shield. 

## UART Protocol

| Commands | Response |
| a | a |
| u | %04x,%04x,%04x,%04x,%04x; |

The data response to u is organized (from left to right as):

- temperature
- humidity
- noise level
- CO level
- CH4 level

the units being sent across the link are currently not documented, but will be filled in as we know more about our units (@fixme)

