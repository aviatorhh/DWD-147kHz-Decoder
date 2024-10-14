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
uint16_t CAN_IDS[] = {CAN_ID};
#define CAN_ID_MASK 0x7ff

volatile byte buf[8];
long canId = 0;
boolean has_can;
boolean has_rtr;
bool rtr;
uint8_t len;
uint8_t msg_cnt;

#define MESSAGE_LENGTH 80
char message[MESSAGE_LENGTH];  // two lines on the LCD
#endif

uint8_t ch_cnt;
int RXLED = 17;

const uint8_t STARTER_PIN = 5;
const uint8_t STOPPER_PIN = 6;

#define F_MID 569
volatile uint8_t F_OFF;   // 2
volatile uint8_t F_SPAN;  // 25

volatile long prevPulseMicros = 0;

volatile uint8_t BIT;

volatile uint16_t f_mid;

uint8_t t1_cnt;

long signal_period = 0;

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
    has_can = false;
    msg_cnt = 0;

    CAN.setPins(MCP_CAN_CS_PIN, 7);
    CAN.setClockFrequency(8E6);

    while (!CAN.begin(250E3)) {
        digitalWrite(RXLED, LOW);
        delay(200);
        digitalWrite(RXLED, HIGH);
        delay(200);
    }
    uint16_t masks[2] = {CAN_ID_MASK, CAN_ID_MASK};

    CAN.setMaskAndFilter(masks, CAN_IDS, CAN_IDS_SIZE);
    CAN.onReceive(MCP2515_ISR);
#endif
    // RPM Pin
    pinMode(0, INPUT);
    cli();  // stop interrupts
    attachInterrupt(digitalPinToInterrupt(0), signal, RISING);

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
    wdt_enable(WDTO_2S);
}

ISR(TIMER1_COMPA_vect) {
    milsec++;
    static byte rSq;
    ti++;
    if (rSq == 0 && BIT == 0) {
        rSq = 1;
        ti = 0;
    }  // Trigger Start
    if (rSq == 1 && ti == 10) {
        if (BIT == 0) {
            rSq = 2;
            ti = 0;
        } else {
            rSq = 0;
        }
    }  // STARTBIT
    if (rSq == 2 && ti == 20) {
        bitWrite(baudot, 0, BIT);
    }  // DATABITS 1-5
    if (rSq == 2 && ti == 40) {
        bitWrite(baudot, 1, BIT);
    }
    if (rSq == 2 && ti == 60) {
        bitWrite(baudot, 2, BIT);
    }
    if (rSq == 2 && ti == 80) {
        bitWrite(baudot, 3, BIT);
    }
    if (rSq == 2 && ti == 100) {
        bitWrite(baudot, 4, BIT);
        dsp = 1;
    }
    if (rSq == 2 && ti == 120) {
        rSq = 0;
    }  // STOPBIT
}

void loop() {
#ifdef HAS_CAN
    if (has_can) {
        if (len == 4) {
            F_OFF = buf[0];
            F_SPAN = buf[1];
        }

        has_can = false;
    }
#endif
    if (milsec >= 1000) {  // one second

        mean = 0;
        for (uint16_t i = 0; i < MB_SIZE; i++) {
            mean += mean_buffer[i];
        }
        mean /= MB_SIZE;

        mean_buffer2[mb2_counter++] = mean;

        if (mb2_counter == 60) {  // one minute
            mean = 0;
            mb2_counter = 0;
            for (uint8_t i = 0; i < 60; i++) {
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

    // baudot decoding
    static boolean fig;
    digitalWrite(RXLED, BIT);
    if (dsp == 1) {
        ch = '\0';
        switch (baudot) {
            case B11111:
                fig = 0;  // LETTERS
                break;
            case B11011:
                fig = 1;  // FIGURES
                break;
            case B01000:
                ch = '\r';  // CR
                break;
            case B00010:
                ch = '\n';  // LF
                break;
            case B00100:
                ch = ' ';
                break;
        }

        if (fig == 0) {
            switch (baudot) {
                case B00011:
                    ch = 'A';
                    break;
                case B11001:
                    ch = 'B';
                    break;
                case B01110:
                    ch = 'C';
                    break;
                case B01001:
                    ch = 'D';
                    break;
                case B00001:
                    ch = 'E';
                    break;
                case B01101:
                    ch = 'F';
                    break;
                case B11010:
                    ch = 'G';
                    break;
                case B10100:
                    ch = 'H';
                    break;
                case B00110:
                    ch = 'I';
                    break;
                case B01011:
                    ch = 'J';
                    break;
                case B01111:
                    ch = 'K';
                    break;
                case B10010:
                    ch = 'L';
                    break;
                case B11100:
                    ch = 'M';
                    break;
                case B01100:
                    ch = 'N';
                    break;
                case B11000:
                    ch = 'O';
                    break;
                case B10110:
                    ch = 'P';
                    break;
                case B10111:
                    ch = 'Q';
                    break;
                case B01010:
                    ch = 'R';
                    break;
                case B00101:
                    ch = 'S';
                    break;
                case B10000:
                    ch = 'T';
                    break;
                case B00111:
                    ch = 'U';
                    break;
                case B11110:
                    ch = 'V';
                    break;
                case B10011:
                    ch = 'W';
                    break;
                case B11101:
                    ch = 'X';
                    break;
                case B10101:
                    ch = 'Y';
                    break;
                case B10001:
                    ch = 'Z';
                    break;
            }
        }
        if (fig == 1) {
            switch (baudot) {
                case B00011:
                    ch = '-';
                    break;
                case B11001:
                    ch = '?';
                    break;
                case B01110:
                    ch = ':';
                    break;
                case B00001:
                    ch = '3';
                    break;
                case B00110:
                    ch = '8';
                    break;
                case B01111:
                    ch = '(';
                    break;
                case B10010:
                    ch = ')';
                    break;
                case B11100:
                    ch = '.';
                    break;
                case B01100:
                    ch = ',';
                    break;
                case B11000:
                    ch = '9';
                    break;
                case B10110:
                    ch = '0';
                    break;
                case B10111:
                    ch = '1';
                    break;
                case B01010:
                    ch = '4';
                    break;
                case B00101:
                    ch = '\'';
                    break;
                case B10000:
                    ch = '5';
                    break;
                case B00111:
                    ch = '7';
                    break;
                case B11110:
                    ch = '=';
                    break;
                case B10011:
                    ch = '2';
                    break;
                case B11101:
                    ch = '/';
                    break;
                case B10101:
                    ch = '6';
                    break;
                case B10001:
                    ch = '+';
                    break;
            }
        }
        if (ch != '\0') {
#ifdef HAS_CAN
            message[msg_cnt++] = ch;
            if (ch == '\n' || msg_cnt == MESSAGE_LENGTH) {
                send_packet_message(message);
                msg_cnt = 0;
                memset(message, '\0', sizeof(char) * MESSAGE_LENGTH);
            }            
#endif
            wdt_reset();
#ifdef SERIAL_OUT
            Serial.write(ch);
#endif
        }
        dsp = 0;
    }
}

void signal() {
    long nowMicros = micros();

    signal_period = nowMicros - prevPulseMicros;

    prevPulseMicros = nowMicros;

    if (signal_period > (f_mid + F_OFF) && signal_period < (f_mid + F_SPAN))
        b1_cnt++;
    if (b1_cnt == 5) {
        BIT = 1;
        b1_cnt = 0;
        b2_cnt = 0;
    }

    if (signal_period > (f_mid - F_SPAN) && signal_period < (f_mid - F_OFF))
        b2_cnt++;
    if (b2_cnt == 5) {
        BIT = 0;
        b1_cnt = 0;
        b2_cnt = 0;
    }
    if (b1_cnt || b2_cnt) {
        if (mb_counter >= MB_SIZE) {
            mb_counter = 0;
        }
        mean_buffer[mb_counter++] = signal_period;
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
            has_can = true;
            has_rtr = rtr;
            break;
        default:
            while (CAN.available()) {
                buf[i++] = CAN.read();
            }
    }
}

void send_packet_message(char* message) {
    int16_t len = strlen(message);
    uint8_t j = 0;
    CAN.beginPacket(CAN_ID, 8, false);
    CAN.write(j++);  // sequence counter 1st nibble and frame counter in 2nd
                     // nibble. Here we do not expect more than 16 frames

    CAN.write(len);

    for (uint8_t i = 0; i < 6; i++) {
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
