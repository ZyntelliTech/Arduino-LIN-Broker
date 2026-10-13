#include "rf433.h"

#include <Arduino.h>
#include <HardwareTimer.h>

// ============================================================
// NON-BLOCKING TIMER-DRIVEN RF433
//
// Blue Pill:
// RF transmitter DATA -> PA6
//
// Existing RF format preserved:
//
// ON:
//   15 55 0C
//
// OFF:
//   15 55 03
//
// Bit time:
//   1380 us
//
// Encoding:
//   0 = HIGH 345 us,  LOW 1035 us
//   1 = HIGH 1035 us, LOW 345 us
//
// Repeats:
//   5
// ============================================================

static const uint32_t RF_SHORT_US    = 345;
static const uint32_t RF_LONG_US     = 1035;
static const uint32_t RF_PREAMBLE_US = 11000;

// Timer is interrupt-disabled while RF is idle. It is re-enabled only
// when a new RF transmission starts.
static const uint32_t RF_IDLE_TIMER_US = 60000;

static HardwareTimer *rfTimer = nullptr;


// ============================================================
// RF TIMER INTERRUPT CONTROL
// ============================================================

static inline void enableRFTimerInterrupt()
{
  TIM2->SR &= ~TIM_SR_UIF;
  TIM2->DIER |= TIM_DIER_UIE;
}

static inline void disableRFTimerInterrupt()
{
  TIM2->DIER &= ~TIM_DIER_UIE;
  TIM2->SR &= ~TIM_SR_UIF;
}


// ============================================================
// RF DATA
// ============================================================

static volatile uint8_t rfData[3];

static volatile uint8_t rfRepeatIndex = 0;
static volatile uint8_t rfByteIndex   = 0;
static volatile int8_t  rfBitIndex    = 7;

static volatile bool rfBusyFlag = false;


// One pending RF command can wait while another is transmitting.
static volatile bool pendingSignalValid = false;
static volatile uint8_t pendingSignal = 0;


// ============================================================
// RF STATE
// ============================================================

enum RFState {
  RF_IDLE,

  RF_INITIAL_HIGH,

  RF_PREAMBLE_LOW,

  RF_BIT_HIGH,

  RF_BIT_LOW,

  RF_END_HIGH
};

static volatile RFState rfState = RF_IDLE;


// ============================================================
// FAST PA6 ACCESS
//
// PA6 is used directly rather than digitalWrite() inside the ISR.
// ============================================================

static inline void rfHigh()
{
  GPIOA->BSRR = (1U << 6);
}


static inline void rfLow()
{
  GPIOA->BRR = (1U << 6);
}


// ============================================================
// TIMER SCHEDULING
//
// IMPORTANT:
//
// TIM2 is configured once for approximately 1 us/count.
//
// We DO NOT call:
//
//   setOverflow()
//   refresh()
//   pause()
//   resume()
//
// from the ISR.
//
// We change ARR/CNT directly instead.
// ============================================================

static inline void scheduleRF(uint32_t us)
{
  // STM32 timer ARR represents period-1.
  TIM2->ARR = us - 1;

  // Restart count from zero.
  TIM2->CNT = 0;

  // Clear any pending update flag.
  TIM2->SR &= ~TIM_SR_UIF;
}


// ============================================================
// CURRENT RF BIT
// ============================================================

static inline bool currentBitIsOne()
{
  return
    (rfData[rfByteIndex] &
    ((uint8_t)1U << rfBitIndex)) != 0;
}


// ============================================================
// LOAD RF MESSAGE
// ============================================================

static inline void loadRFMessage(uint8_t signalByte)
{
  rfData[0] = RF433_PREFIX_0;
  rfData[1] = RF433_PREFIX_1;
  rfData[2] = signalByte;

  rfRepeatIndex = 0;

  rfByteIndex = 0;
  rfBitIndex = 7;
}


// ============================================================
// START RF TRANSMISSION
// ============================================================

static void startTransmission(uint8_t signalByte)
{
  loadRFMessage(signalByte);

  rfBusyFlag = true;

  // Original ask_send_bytes() starts HIGH for approximately
  // 11 ms before the first LOW preamble.
  rfHigh();

  rfState = RF_INITIAL_HIGH;

  scheduleRF(RF_PREAMBLE_US);
  enableRFTimerInterrupt();
}


// ============================================================
// START PENDING TRANSMISSION
//
// Called from ISR after current message is complete.
// ============================================================

static inline void startPendingTransmissionFromISR()
{
  uint8_t signal = pendingSignal;

  pendingSignalValid = false;

  loadRFMessage(signal);

  rfBusyFlag = true;

  rfHigh();

  rfState = RF_INITIAL_HIGH;

  scheduleRF(RF_PREAMBLE_US);
}


// ============================================================
// FINISH RF TRANSMISSION
// ============================================================

static inline void finishTransmissionFromISR()
{
  rfLow();

  if (pendingSignalValid) {

    startPendingTransmissionFromISR();

    return;
  }

  rfBusyFlag = false;

  rfState = RF_IDLE;

  // No reason to keep generating TIM2 interrupts while RF is idle.
  // This removes unnecessary interrupt load from the LIN bridge.
  disableRFTimerInterrupt();
}


// ============================================================
// TIMER ISR
// ============================================================

static void rfTimerISR()
{
  switch (rfState) {

    // ========================================================
    // IDLE
    // ========================================================

    case RF_IDLE:

      // Nothing to do.
      //
      // The timer remains running so we never need to enable /
      // disable the timer from inside this ISR.
      return;


    // ========================================================
    // Initial HIGH finished
    //
    // Original ASK:
    //
    // HIGH 11 ms
    // LOW  11 ms
    // ========================================================

    case RF_INITIAL_HIGH:

      rfLow();

      rfState = RF_PREAMBLE_LOW;

      scheduleRF(RF_PREAMBLE_US);

      return;


    // ========================================================
    // LOW preamble finished
    //
    // Start first bit.
    // ========================================================

    case RF_PREAMBLE_LOW:

      rfByteIndex = 0;
      rfBitIndex = 7;

      rfHigh();

      rfState = RF_BIT_HIGH;

      if (currentBitIsOne()) {

        scheduleRF(RF_LONG_US);

      } else {

        scheduleRF(RF_SHORT_US);
      }

      return;


    // ========================================================
    // HIGH portion of current bit finished.
    // ========================================================

    case RF_BIT_HIGH:

      rfLow();

      rfState = RF_BIT_LOW;

      if (currentBitIsOne()) {

        scheduleRF(RF_SHORT_US);

      } else {

        scheduleRF(RF_LONG_US);
      }

      return;


    // ========================================================
    // LOW portion of current bit finished.
    // Advance to next bit.
    // ========================================================

    case RF_BIT_LOW:

      rfBitIndex--;

      if (rfBitIndex < 0) {

        rfBitIndex = 7;

        rfByteIndex++;
      }


      // ------------------------------------------------------
      // More data bits remain.
      // ------------------------------------------------------

      if (rfByteIndex < 3) {

        rfHigh();

        rfState = RF_BIT_HIGH;

        if (currentBitIsOne()) {

          scheduleRF(RF_LONG_US);

        } else {

          scheduleRF(RF_SHORT_US);
        }

        return;
      }


      // ------------------------------------------------------
      // All 24 bits transmitted.
      //
      // Original ASK ends each repetition:
      //
      // HIGH 345 us
      // LOW
      // ------------------------------------------------------

      rfHigh();

      rfState = RF_END_HIGH;

      scheduleRF(RF_SHORT_US);

      return;


    // ========================================================
    // Final short HIGH pulse finished.
    // ========================================================

    case RF_END_HIGH:

      rfLow();

      rfRepeatIndex++;


      // ------------------------------------------------------
      // Repeat code again.
      // ------------------------------------------------------

      if (rfRepeatIndex < RF433_TRY_COUNT) {

        // Original ASK leaves the pin LOW for the 11 ms
        // preamble before transmitting the next repetition.

        rfState = RF_PREAMBLE_LOW;

        scheduleRF(RF_PREAMBLE_US);

        return;
      }


      // ------------------------------------------------------
      // Completed all 5 repetitions.
      // ------------------------------------------------------

      finishTransmissionFromISR();

      return;


    default:

      rfLow();

      rfBusyFlag = false;

      rfState = RF_IDLE;

      disableRFTimerInterrupt();

      return;
  }
}


// ============================================================
// INITIALIZATION
// ============================================================

void rf433Begin()
{
  // PA6 remains the transmitter DATA pin.
  pinMode(PA6, OUTPUT);

  rfLow();


  // ----------------------------------------------------------
  // Configure TIM2 once.
  //
  // Using MICROSEC_FORMAT establishes a suitable prescaler for
  // microsecond-scale timing.
  // ----------------------------------------------------------

  rfTimer = new HardwareTimer(TIM2);

  rfTimer->setOverflow(
    RF_IDLE_TIMER_US,
    MICROSEC_FORMAT
  );

  rfTimer->attachInterrupt(
    rfTimerISR
  );

  // Force timer configuration ONCE during initialization.
  rfTimer->refresh();

  // Disable ARR preload so changing TIM2->ARR takes effect
  // immediately at each RF transition.
  TIM2->CR1 &= ~TIM_CR1_ARPE;

  rfState = RF_IDLE;

  rfBusyFlag = false;

  pendingSignalValid = false;

  // Start TIM2. The timer counter may remain running, but its update
  // interrupt is disabled while RF is idle.
  rfTimer->resume();
  disableRFTimerInterrupt();

  // ----------------------------------------------------------
  // INTERRUPT PRIORITIES
  //
  // LIN UART receive must always outrank RF waveform timing.
  // Lower numeric value = higher NVIC priority.
  //
  // PA9/PA10  -> USART1 (master side)
  // PA2/PA3   -> USART2 (switch side)
  // TIM2      -> RF waveform
  //
  // A few microseconds of RF edge jitter is harmless; losing a LIN byte is not.
  // ----------------------------------------------------------
  NVIC_SetPriority(USART1_IRQn, 1);
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_SetPriority(TIM2_IRQn, 3);
}


// ============================================================
// RF MESSAGE REQUEST
// ============================================================

void rf433SendMessage(uint8_t signalByte)
{
  noInterrupts();


  // ----------------------------------------------------------
  // An RF transmission is already happening.
  //
  // Queue one command and return immediately.
  // ----------------------------------------------------------

  if (rfBusyFlag) {

    pendingSignal = signalByte;

    pendingSignalValid = true;

    interrupts();

    return;
  }


  // ----------------------------------------------------------
  // Start immediately.
  // ----------------------------------------------------------

  startTransmission(signalByte);

  interrupts();
}


// ============================================================
// CHANNEL COMMANDS
// ============================================================

void rf433TurnOnChannel()
{
  rf433SendMessage(
    RF433_CHANNEL1_SIGNAL
  );
}


void rf433TurnOffChannel()
{
  rf433SendMessage(
    RF433_CHANNEL2_SIGNAL
  );
}


// ============================================================
// STATUS
// ============================================================

bool rf433Busy()
{
  return rfBusyFlag;
}