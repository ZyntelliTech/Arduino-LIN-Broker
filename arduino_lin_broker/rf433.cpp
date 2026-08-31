#include "rf433.h"

#include <Arduino.h>

#include "ask.h"
#include "ask_hal.h"

static uint8_t rf433Data[3];

void rf433Begin() {
  ask433.fn_init_rx = ask_init_rx433;
  ask433.fn_init_tx = ask_init_tx433;
  ask433.fn_micros = ask_micros_433;
  ask433.fn_read_pin = ask_read_pin_433;
  ask433.fn_write_pin = ask_write_pin_433;
  ask433.fn_delay_ms = ask_delay_ms_433;
  ask433.fn_delay_us = ask_delay_us_433;
  ask_init(&ask433);
  delay(50);
}

void rf433SendBytes(uint8_t *data, uint8_t len, uint32_t bitTimeUs, uint8_t tryCount) {
  ask_send_bytes(&ask433, data, len, bitTimeUs, tryCount);
}

void rf433SendMessage(uint8_t signalByte) {
  rf433Data[0] = RF433_PREFIX_0;
  rf433Data[1] = RF433_PREFIX_1;
  rf433Data[2] = signalByte;
  rf433SendBytes(rf433Data, 3, RF433_BIT_TIME_US, RF433_TRY_COUNT);
}

void rf433SendMessageBurst(uint8_t signalByte, uint8_t bursts, uint16_t gapMs) {
  for (uint8_t i = 0; i < bursts; i++) {
    rf433SendMessage(signalByte);
    if (i + 1 < bursts) {
      delay(gapMs);
    }
  }
}

void rf433TurnOnChannel() {
  rf433SendMessage(RF433_CHANNEL1_SIGNAL);
}

void rf433TurnOffChannel() {
  rf433SendMessage(RF433_CHANNEL2_SIGNAL);
}

void rf433TriggerChannel1Burst(uint8_t bursts, uint16_t gapMs) {
  rf433SendMessageBurst(RF433_CHANNEL1_SIGNAL, bursts, gapMs);
}

void rf433TriggerChannel2Burst(uint8_t bursts, uint16_t gapMs) {
  rf433SendMessageBurst(RF433_CHANNEL2_SIGNAL, bursts, gapMs);
}
