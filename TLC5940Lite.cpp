/*
 * TLC5940Lite.cpp
 *
 * Copyright (c) 2014, garretlab at gmail.com All rights reserved.
 * This library is based on TLC5940 Library which is based on GNU General Public License Version 3.
 */

#include "TLC5940Lite.h"
#include "Arduino.h"
#include <SPI.h>

void TLC5940Lite::init(uint8_t numTLC5940, int vprgPin, uint8_t initialDC, uint16_t initialGS) {
  needXLAT = 0;

  // The number of TLC5940s
  this->numTLC5940 = numTLC5940;
  dcData = new uint8_t[numTLC5940 * 12];
  gsData = new uint8_t[numTLC5940 * 24];
  
  for (int i = 0; i < numTLC5940 * 16; i++) {
    setDCData(i, initialDC);
    setGSData(i, initialGS);
  }
  
  // VPRG: User defined
  this-> vprgPin = vprgPin;
  pinMode(vprgPin, OUTPUT);

  // XLAT: TIMER1A
  xlatPin = 9;
  pinMode(xlatPin, OUTPUT);

  // BLANK: TIMER1B
  blankPin = 10;
  pinMode(blankPin, OUTPUT);

  // GSCKL: TIMER2B
  gsclkPin = 3;
  pinMode(gsclkPin, OUTPUT);

  // SCLK: SPI SCK
  sclkPin = SCK;
  pinMode(sclkPin, OUTPUT);

  firstGSInputCycle = 1;

  // SPI setup
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}

void TLC5940Lite::begin() {
  sendDCData();
  sendGSData();
  pulsePin(xlatPin);
  beginPWM();
}

void TLC5940Lite::setDCData(uint8_t channel, uint8_t value) {
  uint8_t *dc = dcData + (16 * numTLC5940 - channel - 1) * 3 / 4;
  value &= 0x3f;
  
  switch (channel % 4) {
  case 0:
    *dc = (*dc & 0xc0) | value;
    break;
  case 1:
    *dc = (*dc & 0xf0) | (value >> 2);
    dc++;
    *dc = (*dc & 0x3f) | (value << 6);
    break;
  case 2:
    *dc = (*dc & 0xfc) | (value >> 4);
    dc++;
    *dc = (*dc & 0x0f) | (value << 4);
    break;
  case 3:
    *dc = (*dc & 0x03) | (value << 2);
    break;
  }
}

void TLC5940Lite::sendDCData() {
  disableXLAT();
  clearXLATInterrupt();
  needXLAT = 0;
  digitalWrite(vprgPin, HIGH);
  
  uint8_t *p = dcData;
  for (int i = 0; i < (numTLC5940 * 12); i++) {
    SPI.transfer(*p++);
  }
  
  pulsePin(xlatPin);
  
  digitalWrite(vprgPin, LOW);
  firstGSInputCycle = 1;
}

uint8_t TLC5940Lite::getDCData(uint8_t channel) {
  uint8_t *dc = dcData + (16 * numTLC5940 - channel - 1) * 3 / 4;
  uint8_t value;

  switch (channel % 4) {
  case 0:
    value = *dc & 0x3f;
    break;
  case 1:
    value = ((*dc & 0x0f) << 2) | (*(dc + 1) & 0xc0) >> 6;
    break;
  case 2:
    value = ((*dc & 0x03) << 4) | (*(dc + 1) & 0xf0) >> 4;
    break;
  case 3:
    value = (*dc & 0xfc) >> 2;
    break;
  }  
  
  return value;
}

void TLC5940Lite::setGSData(uint8_t channel, uint16_t value) {
  value &= 0x0fff;
  uint8_t *gs = gsData + (16 * numTLC5940 - channel -1) * 3 / 2;
  
  switch (channel % 2) {
  case 0:
    *gs = (*gs & 0xf0) | (value >> 8);
    gs++;
    *gs = value & 0xff;
    break;
  case 1:
    *gs = value >> 4;
    gs++;
    *gs = (*gs & 0x0f) | (value << 4);
    break;
  }
}

uint8_t TLC5940Lite::sendGSData() {
  if (needXLAT) {
    return 1;
  }

  disableXLAT();

  if (firstGSInputCycle) {
    firstGSInputCycle = 0;
  } else {
    pulsePin(sclkPin);
  }

  uint8_t *p = gsData;
  for (int i = 0; i < (numTLC5940 * 24); i++) {
    SPI.transfer(*p++);
  }

  needXLAT = 1;
  enableXLAT();
  setXLATInterrupt();
  return 0;
}

uint16_t TLC5940Lite::getGSData(uint8_t channel) {
  uint8_t *gs = gsData + (16 * numTLC5940 - channel -1) * 3 / 2;
  uint16_t value;

  switch (channel % 2) {
  case 0:
    value = (uint16_t)(*gs & 0x0f) << 8 | *(gs + 1);
    break;
  case 1:
    value = (uint16_t)(*gs << 4) | (*(gs + 1) & 0xf0)>> 4;
    break;
  }
  
  return value;
}

void TLC5940Lite::enableXLAT() {
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
}

void TLC5940Lite::disableXLAT() {
  TCCR1A = _BV(COM1B1);
}

void TLC5940Lite::setXLATInterrupt() {
  TIFR1 |= _BV(TOV1); 
  TIMSK1 = _BV(TOIE1);
}

void TLC5940Lite::clearXLATInterrupt() {
  TIMSK1 = 0;
}

void TLC5940Lite::pulsePin(int pin) {
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}

void TLC5940Lite::dcModeStart() {
  disableXLAT();
  clearXLATInterrupt();
  needXLAT = 0;
  digitalWrite(vprgPin, HIGH);
}

void TLC5940Lite::dcModeStop(void) {
  digitalWrite(vprgPin, LOW);
  firstGSInputCycle = 1;
}

void TLC5940Lite::beginPWM() {
  TCCR1A = _BV(COM1B1);  
  TCCR1B = _BV(WGM13);
  OCR1A = 1;
  OCR1B = 2;
  ICR1 = 8192;
  
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22);
  OCR2A = 3;
  OCR2B = 0;

  TCCR1B |= _BV(CS10);
  TCCR2B |= _BV(CS20);
}

ISR(TIMER1_OVF_vect) {
  TLC5940Lite::disableXLAT();
  TLC5940Lite::clearXLATInterrupt();
  TLC5940Lite::needXLAT = 0;
}

volatile uint8_t TLC5940Lite::needXLAT;

TLC5940Lite Tlc;

