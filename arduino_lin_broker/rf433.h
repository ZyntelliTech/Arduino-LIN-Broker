#ifndef RF433_H
#define RF433_H

#include <stdint.h>

const uint32_t RF433_BIT_TIME_US      = 1380;
const uint8_t  RF433_TRY_COUNT        = 5;

const uint8_t  RF433_PREFIX_0         = 0x15;
const uint8_t  RF433_PREFIX_1         = 0x55;

const uint8_t  RF433_CHANNEL1_SIGNAL  = 0x0C;
const uint8_t  RF433_CHANNEL2_SIGNAL  = 0x03;

void rf433Begin();

void rf433SendMessage(uint8_t signalByte);

void rf433TurnOnChannel();
void rf433TurnOffChannel();

bool rf433Busy();

#endif