RPLidar currently has SDK for Windows/Linux/Mac and is delivered with a UART-to-USB converter in a devkit.
However, it can't be plugged into ESP32 through USB-C as both are USB "devices", while one has to be "host".
This driver provides basic functionality: bring up, scan processing, and tear down, according to state machine and register descriptions specified in "Interface protocol and application notes" by SlamTec.
