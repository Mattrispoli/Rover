## Tx side (BeerBot Folder)

The transmitter side ESP-32 broadcasts direction data to the receiver side ESP-32s.

## Rx side ( BeerBotRx )
Rx side is only configured for forward and backward movement. Separate Rx receivers will be used for turning the front wheels.

PWM is utilized on the Rx side. Rx expects x and y direction values from 0.0 to 1.0.
