# DWD-147kHz-Decoder
The ATmega receives the signal on its RX pin (0) via an interrupt. It can directly be connected to the digital signal pin on the NASA target 147 board (pictures will follow).

This version puts out its messages via the serial line (300 baud) and via a attached CANbus interface. The messages are sent with CAN ID 0x065 (randomly chosen) as multiframe packets (ISO 15765-2).

Currently they are used on a NMEA2000 network.
