// 433 MHz RF transmit helpers (ASK library), based on EV1527_Decoder.ino.

#ifndef RF433_H
#define RF433_H

#include <stdint.h>

const uint32_t RF433_BIT_TIME_US    = 1380;
const uint8_t  RF433_TRY_COUNT      = 5;
const uint8_t  RF433_PREFIX_0       = 0x15;
const uint8_t  RF433_PREFIX_1       = 0x55;
const uint8_t  RF433_CHANNEL1_SIGNAL = 0x0C;
const uint8_t  RF433_CHANNEL2_SIGNAL = 0x03;

void rf433Begin();
void rf433SendMessage(uint8_t signalByte);
void rf433SendMessageBurst(uint8_t signalByte, uint8_t bursts = 3, uint16_t gapMs = 35);
void rf433SendBytes(uint8_t *data, uint8_t len, uint32_t bitTimeUs = RF433_BIT_TIME_US, uint8_t tryCount = RF433_TRY_COUNT);
void rf433TurnOnChannel();
void rf433TurnOffChannel();
void rf433TriggerChannel1Burst(uint8_t bursts = 3, uint16_t gapMs = 35);
void rf433TriggerChannel2Burst(uint8_t bursts = 3, uint16_t gapMs = 35);

#endif
