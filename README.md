# DWD-147kHz-Decoder

![Screenshot 2024-10-02 at 08 07 50](https://github.com/user-attachments/assets/84171cc3-8754-4afc-b33e-ed932bcb5f7e)

The ATmega receives the signal on its RX pin (0) via an interrupt. It can directly be connected to the digital signal pin (see black wire below) on the NASA target 147 board.

![IMG_5271](https://github.com/user-attachments/assets/468731fd-245c-4392-bd06-803ced2bc289)
![pcb_front](https://github.com/user-attachments/assets/9ae0b2fe-de01-40ea-b6c5-436676522244)



Where 
- red is 5V
- brown is GND
- black is TTL FSK signal
- white is raw audio signal

These wires have been soldered by me.

This version puts out its messages via the serial line (300 baud) and via a attached CANbus interface. The messages are sent with CAN ID 0x065 (randomly chosen) as multiframe packets (ISO 15765-2).

Currently they are used on a NMEA2000 network.

## Building (PlatformIO)
Use <code>PLATFORMIO_BUILD_FLAGS="-DHAS_CAN -DSERIAL_OUT" pio run</code>
