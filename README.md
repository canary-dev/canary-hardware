# Canary Hardware Repository

Uses adafruits BLE UART library as a basis and the nRF8001 BLE shield. 

## UART Protocol

| Commands | Response            |
|----------|---------------------|
| a        | a                   |
| u        | %04x,%04x,%04x,%04x |

The data response to u is organized (from left to right as):

- temperature  
  - Scaled (T (deg C) / 50 (deg C)) * 0xffff  
- humidity  
  - Scaled as (H (percent) / 100 (percent)) * 0xffff)  
- noise level  
  - presented as a 16 bit number and a 10 bit fullscale (from the ADC) 
  - convert to dB (full scale): 20*log10(N/1024)
- CH4 level  
  - Scaled as ?  


Units and scalings are being documented. Noise and CH4 are prociding difficulties

