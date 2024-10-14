#ifndef _H_MAIN_
#define _H_MAIN_

#ifdef HAS_CAN  
void send_packet_message(char* message);
void MCP2515_ISR(int packetSize);
#endif
void signal();

#endif
