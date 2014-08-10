# asb_BLENode

Uses adafruits BLE UART library as a basis and the nRF8001 BLE shield. 

## UART Protocol

| Commands | Response |
| a | a |
| u | %d,%d,%d,%d,%d |

The data response to u is organized (from left to right as):

- temperature
-- Scaled (T (deg C) / 50 (deg C)) * 0xffff
- humidity
-- Scaled as (H (percent) / 100 (percent)) * 0xffff)
- noise level
-- Scaled as ?
- CH4 level
-- Scaled as ?


Units and scalings are being documented. Noise and CH4 are prociding difficulties

