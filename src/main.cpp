#ifdef HAS_CAN  
#include <CAN.h>
#endif
#include <Arduino.h>
#include <avr/wdt.h>
#include "main.h"

uint8_t b1_cnt;
uint8_t b2_cnt;

#ifdef HAS_CAN  

#define MCP_CAN_CS_PIN 10
#define CAN_IDS_SIZE 1
#define CAN_ID 0x065
uint16_t CAN_IDS[] = { CAN_ID };
#define CAN_ID_MASK 0x7ff

volatile byte buf[8];
long canId = 0;
boolean has_eng;
boolean has_can;
boolean has_rtr;
bool rtr;
uint8_t len;
uint8_t msg_cnt;

uint8_t message[80]; // two lines on the LCD
#endif

uint8_t ch_cnt;
int RXLED = 17;

const uint8_t STARTER_PIN = 5;
const uint8_t STOPPER_PIN = 6;

#define  F_MID 569
volatile uint8_t  F_OFF;// 2
volatile uint8_t  F_SPAN;// 25

volatile long prevPulseMicros = 0;

volatile uint8_t BIT;

volatile uint16_t f_mid;

uint8_t t1_cnt;

long rpms = 0;

volatile long latestPulseMicros = 0;

volatile long revMicros = 0;

#define MB_SIZE 512

volatile uint16_t mean_buffer[MB_SIZE];
uint16_t mean_buffer2[60];
volatile uint16_t mb_counter;
uint8_t mb2_counter;
uint32_t mean;
uint16_t milsec;

boolean dsp;
byte ti;
uint8_t baudot;
char ch;


void setup() {
  f_mid = F_MID;
  F_OFF = 5;
  F_SPAN = 0x50;
  t1_cnt = 0;
  BIT = 0;
  msg_cnt = 0;
  b1_cnt = 0;
  b2_cnt = 0;
  mb_counter = 0;
  mb2_counter = 0;
  milsec = 0;
  pinMode(STARTER_PIN, OUTPUT);  // Starter Relay
  pinMode(STOPPER_PIN, OUTPUT);  // Stopper Relay

  pinMode(0, INPUT_PULLUP);

  digitalWrite(STARTER_PIN, LOW);
  digitalWrite(RXLED, HIGH);
  digitalWrite(STOPPER_PIN, LOW);
  TXLED0;
#ifdef SERIAL_OUT  
  Serial.begin(300);
#endif
#ifdef HAS_CAN  
  CAN.setPins(MCP_CAN_CS_PIN, 7);
  CAN.setClockFrequency(8E6);


  while (!CAN.begin(250E3)) {
    digitalWrite(RXLED, LOW);
    delay(200);
    digitalWrite(RXLED, HIGH);
    delay(200);
  }
  uint16_t masks[2] = { CAN_ID_MASK, CAN_ID_MASK };

  CAN.setMaskAndFilter(masks, CAN_IDS, CAN_IDS_SIZE);
  CAN.onReceive(MCP2515_ISR);
#endif
  // RPM Pin
  pinMode(0, INPUT);
  cli();  // stop interrupts
  attachInterrupt(digitalPinToInterrupt(0), rpm, RISING);


  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 1000 Hz (16000000/((249+1)*64))
  OCR1A = 249;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  sei();  // allow interrupts
#ifdef SERIAL_OUT   
//  Serial.println("go");
#endif
  wdt_enable(WDTO_1S);
}




ISR(TIMER1_COMPA_vect) {
  milsec++;
  static byte rSq;
  ti++;
  if (rSq == 0 && BIT == 0) {
    rSq = 1;
    ti = 0;
  }  //Trigger Start
  if (rSq == 1 && ti == 10) {
    if (BIT == 0) {
      rSq = 2;
      ti = 0;
    } else {
      rSq = 0;
    }
  }                                                        //STARTBIT
  if (rSq == 2 && ti == 20) { bitWrite(baudot, 0, BIT); }  //DATABITS 1-5
  if (rSq == 2 && ti == 40) { bitWrite(baudot, 1, BIT); }
  if (rSq == 2 && ti == 60) { bitWrite(baudot, 2, BIT); }
  if (rSq == 2 && ti == 80) { bitWrite(baudot, 3, BIT); }
  if (rSq == 2 && ti == 100) {
    bitWrite(baudot, 4, BIT);
    dsp = 1;
  }
  if (rSq == 2 && ti == 120) { rSq = 0; }  //STOPBIT
}



void loop() {


  if (has_eng) {

    if (len == 4) {
      F_OFF = buf[0];
      F_SPAN = buf[1];
    }

    has_eng = false;
  }


  if (milsec >= 1000) { // one second

    mean = 0;
    for(uint16_t i = 0; i < MB_SIZE; i++) {
      mean += mean_buffer[i];
    }
    mean /= MB_SIZE;

    mean_buffer2[mb2_counter++] = mean;

  if (mb2_counter == 60) {  // one minute
    mean = 0;
    mb2_counter = 0;
    for(uint8_t i = 0; i < 60; i++) {
      mean += mean_buffer2[i];
    }
    mean /= 60;
    f_mid = mean;
  #ifdef SERIAL_OUT_DEBUG
    Serial.println();
    Serial.print("f_mid: ");
    Serial.println(f_mid, DEC);
  #endif    
  }


    milsec = 0;
  }

#ifdef HAS_CAN
  if (has_can && has_eng && len == 1) {
    has_can = has_eng = false;
  }
#endif

  //baudot decoding
  static boolean fig;
  digitalWrite(RXLED, BIT);
  if (dsp == 1) {
    ch = '\0';
    if (baudot == B11111) { fig = 0; }  //LETTERS
    else if (baudot == B11011) {
      fig = 1;
    }                                          //FIGURES
    else if (baudot == B01000) { ch = '\r'; }  //CR
    else if (baudot == B00010) {
      ch = '\n';
    }                                         //LF
    else if (baudot == B00100) { ch = ' '; }  //SPACE
    if (fig == 0) {
      if (baudot == B00011) {
        ch = 'A';
      } else if (baudot == B11001) {
        ch = 'B';
      } else if (baudot == B01110) {
        ch = 'C';
      } else if (baudot == B01001) {
        ch = 'D';
      } else if (baudot == B00001) {
        ch = 'E';
      } else if (baudot == B01101) {
        ch = 'F';
      } else if (baudot == B11010) {
        ch = 'G';
      } else if (baudot == B10100) {
        ch = 'H';
      } else if (baudot == B00110) {
        ch = 'I';
      } else if (baudot == B01011) {
        ch = 'J';
      } else if (baudot == B01111) {
        ch = 'K';
      } else if (baudot == B10010) {
        ch = 'L';
      } else if (baudot == B11100) {
        ch = 'M';
      } else if (baudot == B01100) {
        ch = 'N';
      } else if (baudot == B11000) {
        ch = 'O';
      } else if (baudot == B10110) {
        ch = 'P';
      } else if (baudot == B10111) {
        ch = 'Q';
      } else if (baudot == B01010) {
        ch = 'R';
      } else if (baudot == B00101) {
        ch = 'S';
      } else if (baudot == B10000) {
        ch = 'T';
      } else if (baudot == B00111) {
        ch = 'U';
      } else if (baudot == B11110) {
        ch = 'V';
      } else if (baudot == B10011) {
        ch = 'W';
      } else if (baudot == B11101) {
        ch = 'X';
      } else if (baudot == B10101) {
        ch = 'Y';
      } else if (baudot == B10001) {
        ch = 'Z';
      }
    }
    if (fig == 1) {
      if (baudot == B00011) {
        ch = '-';
      } else if (baudot == B11001) {
        ch = '?';
      } else if (baudot == B01110) {
        ch = ':';
      } else if (baudot == B01001) {
      } else if (baudot == B00001) {
        ch = '3';
      } else if (baudot == B01101) {
      } else if (baudot == B11010) {
      } else if (baudot == B10100) {
      } else if (baudot == B00110) {
        ch = '8';
      } else if (baudot == B01011) {
      } else if (baudot == B01111) {
        ch = '(';
      } else if (baudot == B10010) {
        ch = ')';
      } else if (baudot == B11100) {
        ch = '.';
      } else if (baudot == B01100) {
        ch = ',';
      } else if (baudot == B11000) {
        ch = '9';
      } else if (baudot == B10110) {
        ch = '0';
      } else if (baudot == B10111) {
        ch = '1';
      } else if (baudot == B01010) {
        ch = '4';
      } else if (baudot == B00101) {
        ch = '\'';
      } else if (baudot == B10000) {
        ch = '5';
      } else if (baudot == B00111) {
        ch = '7';
      } else if (baudot == B11110) {
        ch = '=';
      } else if (baudot == B10011) {
        ch = '2';
      } else if (baudot == B11101) {
        ch = '/';
      } else if (baudot == B10101) {
        ch = '6';
      } else if (baudot == B10001) {
        ch = '+';
      }
    }
    if (ch != '\0') {
#ifdef HAS_CAN        
      message[msg_cnt++] = ch;
      message[msg_cnt] = '\0';
#endif
      wdt_reset();
#ifdef SERIAL_OUT  
      Serial.write(ch);
#endif
    }
    dsp = 0;

#ifdef HAS_CAN  
    if (ch == '\n') {
      send_packet_message(message, msg_cnt);
      msg_cnt = 0;
    }

#endif  
  }

}


void rpm() {
  
  long nowMicros = micros(); 

  rpms = nowMicros - prevPulseMicros;

  prevPulseMicros = nowMicros;



  if (rpms > (f_mid+F_OFF) && rpms < (f_mid+F_SPAN)) b1_cnt++;
  if (b1_cnt == 5) {
    BIT = 1;
    b1_cnt = 0;
    b2_cnt = 0;
  }

  if (rpms > (f_mid-F_SPAN) && rpms < (f_mid-F_OFF)) b2_cnt++;
  if (b2_cnt == 5) {
    BIT = 0;
    b1_cnt = 0;
    b2_cnt = 0;
  }
  if (b1_cnt || b2_cnt) {
    if (mb_counter >= MB_SIZE) {
      mb_counter = 0;
    }    
    mean_buffer[mb_counter++] = rpms;
  }

}
#ifdef HAS_CAN  
void MCP2515_ISR(int packetSize) {
  len = packetSize;
  rtr = CAN.packetRtr();
  canId = CAN.packetId();
  uint8_t i = 0;
  switch (canId) {
    case CAN_ID:
      while (CAN.available()) {
        buf[i++] = CAN.read();
      }
      has_eng = true;
      has_rtr = rtr;
      break;
    default:
      while (CAN.available()) {
        buf[i++] = CAN.read();
      }
  }
  has_can = true;
}

void send_packet_message(uint8_t* message, int16_t len) {

  uint8_t j = 0;
  CAN.beginPacket(CAN_ID, 8, false);
  CAN.write(j++);  // sequence counter 1st nibble and frame counter in 2nd nibble. Here we do not expect more than 16 frames

  CAN.write(len);

  uint8_t i;
  for (i = 0; i < 6; i++) {
    if (i < len)
      CAN.write(*message++);
    else
      CAN.write(0x00);
  }




  CAN.endPacket();
  len = len - 6;

  if (len <= 0) {
    return;
  }


  uint8_t chunks = len / 7;

  uint8_t rest = len - (chunks * 7);

  for (uint8_t a = 0; a < chunks; a++) {
    CAN.beginPacket(CAN_ID, 8, false);
    CAN.write(j++);


    for (uint8_t i = 0; i < 7; i++) {

      CAN.write(*message++);
    }
    CAN.endPacket();
  }
  if (rest) {
    CAN.beginPacket(CAN_ID, 8, false);
    CAN.write(j++);
    uint8_t i;
    for (i = 0; i < 7; i++) {
      if (i < rest)
        CAN.write(*message++);
      else
        CAN.write(0x00);
    }
    CAN.endPacket();
  }
}
#endif