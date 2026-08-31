
#include "ask_hal.h"
#include <Arduino.h>
#include "define_board.h"
ask_t ask433;

#if defined(ARDUINO_MEGA_2560)
  #define RX433_PIN 5
  #define TX433_PIN 4
#elif defined(ARDUINO_STM32)
  #define RX433_PIN PA7
  #define TX433_PIN PA6
#endif
void ask_write_pin_433(bool data) {
  digitalWrite(TX433_PIN, data);
}

bool ask_read_pin_433(void) {
  return digitalRead(RX433_PIN);
}

uint32_t ask_micros_433(void) {
  return micros();
}

void ask_init_rx433(void) {
  pinMode(RX433_PIN, INPUT);
}

void ask_init_tx433(void) {
  pinMode(TX433_PIN, OUTPUT);
}

void ask_delay_ms_433(uint32_t delay_ms) {
  delay(delay_ms);
}

void ask_delay_us_433(uint32_t delay_us) {
  delayMicroseconds(delay_us);
}

ask_t ask315;

#if defined(ARDUINO_MEGA_2560)
  #define RX315_PIN 5
  #define TX315_PIN 4
#elif defined(ARDUINO_STM32)
  #define RX315_PIN PA7
  #define TX315_PIN PA6
#endif

void ask_write_pin_315(bool data) {
  digitalWrite(TX315_PIN, data);
}

bool ask_read_pin_315(void) {
  return digitalRead(RX315_PIN);
}

uint32_t ask_micros_315(void) {
  return micros();
}

void ask_init_rx315(void) {
  pinMode(RX315_PIN, INPUT);
}

void ask_init_tx315(void) {
  pinMode(TX315_PIN, OUTPUT);
}

void ask_delay_ms_315(uint32_t delay_ms) {
  delay(delay_ms);
}

void ask_delay_us_315(uint32_t delay_us) {
  delayMicroseconds(delay_us);
}
