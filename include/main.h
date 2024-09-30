#ifndef _H_MAIN_
#define _H_MAIN_

void send_packet_message(uint8_t* message, int16_t len);
void MCP2515_ISR(int packetSize);
void rpm();

#endif