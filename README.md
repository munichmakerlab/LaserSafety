# LaserSafety
Arduino code for the lasercutter safety system

See the control panel:
http://log.munichmakerlab.de/day/2015/04/27

# Sensors
* Flowsensor FS300a
* 2x Dallas 1Wire temperature sensors
* 3x water sensors (open when ok)
* diffence pressure sensor to check on vent

# Programm flow
* Interrupt driven counter to check Flowsensor
* start Dallas Temp reading, set timeout
* check everything else in loop
* timeout read reached, check temp
* if errors, stop machine, change status leds
