# TS_BoatTail_Actuator

First Commit of Boat Tail Actuator

Follows Side Skirt Actuator and Gap Fairing Actuator Controller code.

CAN Standard ID 0x01 - Undistinguished between L | R - Will need to implement individual device ID + Dynamic "First Time" device ID assignment

Command List: HoldMessage = 0x02 |  RetractionMessage = 0x01 | ExtensionMessage = 0x00

Needs LED Controller integration | Diming controlled with PWM signal - ON/OFF can be implemented with simple GPIO output.
