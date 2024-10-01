#ifdef HAS_CAN  
#include <CAN.h>
#endif
#include <Arduino.h>
#include "main.h"

uint8_t b1_cnt;
uint8_t b2_cnt;

#ifdef HAS_CAN  

#define MCP_CAN_CS_PIN 10
#define CAN_ID_ENG 0x00a
#define CAN_IDS_SIZE 1
uint16_t CAN_IDS[] = { CAN_ID_ENG };
#define CAN_ID_MASK 0x00f

volatile byte buf[8];
#endif

volatile float rpm_counter;
uint8_t ch_cnt;
uint8_t msg_cnt;
int RXLED = 17;

volatile uint8_t g_count;

const uint8_t STARTER_PIN = 5;
const uint8_t STOPPER_PIN = 6;

volatile long prevPulseMicros = 0;

volatile uint8_t BIT;
volatile uint8_t lastBIT;

uint8_t t1_cnt;

uint8_t message[64];

long canId = 0;
boolean has_eng;
boolean has_can;
boolean has_rtr;
bool rtr;
uint8_t len;
volatile uint8_t revolutions = 0;

long rpms = 0;

volatile uint8_t C;

volatile long latestPulseMicros = 0;

volatile long revMicros = 0;


volatile bool printIt = false;
boolean dsp;
byte ti;
uint8_t baudot;
char ch;


void setup() {
  t1_cnt = 0;
  BIT = 0;
  lastBIT = 0;
  C = 0;
  msg_cnt = 0;
  b1_cnt = 0;
  b2_cnt = 0;
  g_count = 0;
  pinMode(STARTER_PIN, OUTPUT);  // Starter Relay
  pinMode(STOPPER_PIN, OUTPUT);  // Stopper Relay

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

  // TIMER 1 for interrupt frequency 2 Hz:
  /**TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 2 Hz increments
  OCR1A = 31249; // = 16000000 / (256 * 2) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 256 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  TCCR1A = 0;

  TCCR1B = 0;

  TCCR1B |= (1 << CS12); //Prescaler 256

  TIMSK1 |= (1 << TOIE1); //enable timer overflow
  */

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
  rpm_counter = 0;
  sei();  // allow interrupts
#ifdef SERIAL_OUT   
  Serial.println("go");
#endif
}




ISR(TIMER1_COMPA_vect) {

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
                                           //C |= (BIT<<7);
                                           //C = C>>1;
  //}
}



void loop() {

  if (has_can && has_eng && len == 1) {
    has_can = has_eng = false;
  }

  //if (g_count>=DEPTH) {

  //Serial.println(rpms, DEC);

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
      message[msg_cnt++] = ch;
      message[msg_cnt] = '\0';
#ifdef SERIAL_OUT  
      Serial.write(ch);
#endif
    }
    dsp = 0;

#ifdef HAS_CAN  
    if (ch == '\n') {
      send_packet_message(message, msg_cnt);
      msg_cnt = 0;
      //digitalWrite(STARTER_PIN, !digitalRead(STARTER_PIN));
      //Serial.print(message);
    }

#endif  
  }




}


void rpm() {
  //rpm_counter++;
  long nowMicros = micros();

  rpms = nowMicros - prevPulseMicros;

  prevPulseMicros = nowMicros;



  if (rpms > 574 && rpms < 628) b1_cnt++;
  if (b1_cnt == 2) {
    BIT = 1;
    b1_cnt = 0;
    b2_cnt = 0;
  }

  if (rpms > 500 && rpms < 570) b2_cnt++;
  if (b2_cnt == 2) {
    BIT = 0;
    b1_cnt = 0;
    b2_cnt = 0;
  }



  if (lastBIT == 1 && BIT == 0) {
    TCNT1 = 0;
    ti = ti / 10;
    ti = ti * 10 + 10;
    lastBIT = BIT;
  }


  //PINB = 1;
}
#ifdef HAS_CAN  
void MCP2515_ISR(int packetSize) {
  len = packetSize;
  rtr = CAN.packetRtr();
  canId = CAN.packetId();
  uint8_t i = 0;
  switch (canId) {
    case CAN_ID_ENG:
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
  CAN.beginPacket(0x065, 8, false);
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
    CAN.beginPacket(0x065, 8, false);
    CAN.write(j++);


    for (uint8_t i = 0; i < 7; i++) {

      CAN.write(*message++);
    }
    CAN.endPacket();
  }
  if (rest) {
    CAN.beginPacket(0x065, 8, false);
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