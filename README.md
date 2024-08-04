# GimbalFOC
A gimbal BLDC controller that uses an ATMega328p with a DRV8313PWPR. Communicates over SoftwareSerial with a parent microcontroller such as an Arduino.

## Other Parts
- BLDC: https://shop.iflight.com/gimbal-motors-cat44/ipower-motor-gm6208-150t-brushless-gimbal-motor-pro208
- Battery: Any 4-5S LiPO with an XT60 connector

## Completed
- Initial schema, with a few flaws (listed in todo)
- Intial footprint sourcing

## TODO
- Find capacitor and resistor manufacturers
- Does usb power go to 5v or vcc?
- C_in and C_out of LM2596, working example: https://electronics.stackexchange.com/questions/547010/lm2596s-5-circuit-7-x-v-instead-of-5v-output-voltage-what-did-i-do-wrong
- How much does what inductor I pick matter for the LM2596?
- find digikey sources for JLC parts, or replace parts, to assemble by hand rather than with PCBA service
