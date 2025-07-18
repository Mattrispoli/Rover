## Controller Tx side 

The transmitter side ESP-32 broadcasts direction data to the receiver side ESP-32s.

## Controller Rx side 
Rx side is only configured for forward and backward movement. Separate Rx receivers will be used for turning the front wheels.

PWM is utilized on the Rx side. Rx expects x and y direction values from -1.0 to 1.0.

## Ros Workspace
Publishes directional data from input and sends to subscriber which communicates with the Tx ESP-32.
