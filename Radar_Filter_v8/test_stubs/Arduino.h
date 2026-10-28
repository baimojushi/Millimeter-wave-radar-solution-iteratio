#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstdio>
#define CONFIG_BT_ENABLED 1
#define CONFIG_BLUEDROID_ENABLED 1
#define PI 3.14159265358979323846
#define TWO_PI 6.2831853071795864769
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define SERIAL_8N1 0
class HardwareSerial {
public:
    void begin(unsigned long) {}
    void begin(unsigned long, int, int, int) {}
    void setRxBufferSize(int) {}
    int available() { return 0; }
    int read() { return -1; }
    template<class T> void print(const T&) {}
    template<class T> void println(const T&) {}
    template<class... T> int printf(const char*, T...) { return 0; }
};
extern HardwareSerial Serial, Serial2;
uint32_t millis();
uint32_t micros();
void delay(unsigned long);
int digitalRead(int);
void pinMode(int, int);
