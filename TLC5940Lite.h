/*
 * TLC5940Lite.h
 *
 * Copyright (c) 2014, garretlab at gmail.com All rights reserved.
 * This library is based on TLC5940 Library which is based on GNU General Public License Version 3.
 */

#ifndef TLC5940LITE_H
#define TLC5940LITE_H

#include <Arduino.h>

class TLC5940Lite {
  public:
    void init(uint8_t numTLC5940, int vprgPin, uint8_t initialDC = 63, uint16_t initialGS = 0);
    void begin();
    
    void setDCData(uint8_t channel, uint8_t value);
    void sendDCData();
    uint8_t getDCData(uint8_t channel);
    
    void setGSData(uint8_t channel, uint16_t value);
    uint8_t sendGSData();
    uint16_t getGSData(uint8_t channel);

    // Need to be public to use in interrupt vector
    static void disableXLAT();
    static void clearXLATInterrupt();
    static volatile uint8_t needXLAT;
    
  private:
    int numTLC5940;
    
    int vprgPin;
    int xlatPin;
    int blankPin;
    int sclkPin;
    int gsclkPin;
    
    int firstGSInputCycle;

    uint8_t *dcData;
    uint8_t *gsData;
    
    void enableXLAT();
    void pulsePin(int pin);
    void setXLATInterrupt();
    void dcModeStart();
    void dcModeStop();
    void beginPWM();
};

extern TLC5940Lite Tlc;

#endif


