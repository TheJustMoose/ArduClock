
#include <Wire.h>
#include "DS3231.h"

// TPIC6C595 ctrl
const int REG_DATA = 4;
const int REG_CLK = 5;
const int REG_LATCH = 6;

#define REG_DIR DDRD
#define REG_PORT PORTD
#define ENC_PORT PINC

const int LED1_PIN = 7;
const int LED2_PIN = 8;
const int PWM_PIN = 9;

const int H_BTN_PIN = A3;
const int M_BTN_PIN = 11;
const int Z_BTN_PIN = 2;
const int R_BTN_PIN = 12;

const uint8_t sA = 0x40;
const uint8_t sB = 0x80;
const uint8_t sC = 0x02;
const uint8_t sD = 0x04;
const uint8_t sE = 0x08;
const uint8_t sF = 0x20;
const uint8_t sG = 0x10;
const uint8_t DP = 0x01;

const uint8_t lF = sA | sE | sF | sG;
const uint8_t lU = sB | sC | sD | sE | sF;
const uint8_t lC = sA | sD | sE | sF;
const uint8_t lK = sC | sE | sF | sG;

const int BLANK = 10;

uint8_t SegmentsOf(int digit) {
  switch (digit) {
    case 0: return sA | sB | sC | sD | sE | sF;
    case 1: return sB | sC;
    case 2: return sA | sB | sD | sE | sG;
    case 3: return sA | sB | sC | sD | sG;
    case 4: return sB | sC | sF | sG;
    case 5: return sA | sC | sD | sF | sG;
    case 6: return sA | sC | sD | sE | sF | sG;
    case 7: return sA | sB | sC;
    case 8: return sA | sB | sC | sD | sE | sF | sG;
    case 9: return sA | sB | sC | sD | sF | sG;
    default: return 0;  // BLANK
  }
}

void OutDigit(uint8_t data) {
  // 1us period
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80)
      bitSet(REG_PORT, REG_DATA);
    else
      bitClear(REG_PORT, REG_DATA);
 
    data <<= 1;
 
    bitSet(REG_PORT, REG_CLK);
    bitClear(REG_PORT, REG_CLK);
  }
}

void LatchIt() {
  bitSet(REG_PORT, REG_LATCH);
  bitClear(REG_PORT, REG_LATCH);
}

void setup() {
  // put your setup code here, to run once:
  REG_DIR |= _BV(REG_DATA) | _BV(REG_CLK) | _BV(REG_LATCH);
  bitClear(REG_PORT, REG_LATCH);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  //digitalWrite(PWM_PIN, LOW);
  analogWrite(PWM_PIN, 128);

  Serial.begin(115200);
  Serial.println("Start");
  Wire.begin();
}

bool h_btn_pressed = false;
bool h_in_edit = false;

bool m_btn_pressed = false;
bool m_in_edit = false;

unsigned long btn_time = 0;
unsigned long led_time = 0;
bool updated = false;

int brightness = 127;

/* returns change in encoder state (-1,0,1) */
int8_t read_encoder() {
  static int8_t enc_states[] = {0,1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0};
                            // {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;

  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

void out_time() {
  DateTime now = RTClib::now();
  int h = now.hour();
  int m = now.minute();

  int dig1 = h / 10;
  int dig2 = h % 10;
  int dig3 = m / 10;
  int dig4 = m % 10;

  //bool half = (millis() - btn_time) > 800;

  bool blank = false;//h_in_edit && half;
  OutDigit(SegmentsOf(blank ? BLANK : dig1)); // left
  OutDigit(SegmentsOf(blank ? BLANK : dig2));

  //blank = m_in_edit && half;
  OutDigit(SegmentsOf(blank ? BLANK : dig3));
  OutDigit(SegmentsOf(blank ? BLANK : dig4)); // right

  LatchIt();
}

void loop() {
  static uint8_t counter = 0;      //this variable will be changed by encoder input
  int8_t tmpdata = read_encoder();
  if (tmpdata) {
    if (h_in_edit) {
      DateTime now = RTClib::now();
      int h = now.hour();
      h += tmpdata;
      if (h >= 24)
        h = 0;
      if (h < 0)
        h = 23;

      DS3231 clk;
      clk.setHour(h);
      out_time();

      btn_time = millis();

      Serial.print("H: ");
      Serial.println(h, DEC);
    } else if (m_in_edit) {
      DateTime now = RTClib::now();
      int m = now.minute();
      m += tmpdata;
      if (m >= 60)
        m = 0;
      if (m < 0)
        m = 59;

      DS3231 clk;
      clk.setMinute(m);
      out_time();

      btn_time = millis();

      Serial.print("M: ");
      Serial.println(m, DEC);
    } else {
      brightness += tmpdata*4;
      if (brightness < 0)
        brightness = 0;
      if (brightness > 255)
        brightness = 255;
      analogWrite(PWM_PIN, brightness);
      Serial.println("bright");
    }
  }

  if (digitalRead(H_BTN_PIN) == 0) {
    delay(5);
    if (digitalRead(H_BTN_PIN) == 0 && !h_btn_pressed) {
      h_btn_pressed = true;
      h_in_edit = true;
      m_in_edit = false;
      btn_time = millis();
      Serial.println("H button pressed");
      Serial.println(millis());
    }
  }
  else
    h_btn_pressed = false;

  if (digitalRead(M_BTN_PIN) == 0) {
    delay(5);
    if (digitalRead(M_BTN_PIN) == 0 && !m_btn_pressed) {
      m_btn_pressed = true;
      m_in_edit = true;
      h_in_edit = false;
      btn_time = millis();
      Serial.println("M button pressed");
      Serial.println(millis());
    }
  }
  else
    m_btn_pressed = false;

  if (millis() - btn_time > 6000) {
    if (h_in_edit || m_in_edit) {
      h_in_edit = false;
      m_in_edit = false;
      Serial.println("Stop editing");
    }
  }

  unsigned long d = millis() - led_time;
  if ((d > 900) && !updated) {
    out_time();
    updated = true;
  }

  if (d > 500) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
  }

  if (d >= 1000) {
    updated = false;
    led_time = millis();
    if (!m_in_edit)
      digitalWrite(LED1_PIN, LOW);
    if (!h_in_edit)
      digitalWrite(LED2_PIN, LOW);
    Serial.println("sec");
  }
}
