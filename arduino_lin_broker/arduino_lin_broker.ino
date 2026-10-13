#include "ask_hal.h"
#include "rf433.h"
#include "define_board.h"
#include <EEPROM.h>
#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_STM32)
#include <SoftwareSerial.h>
#endif

// ============================================================
// PROJECT INTERCEPT - BLUE PILL LIN MITM
// ============================================================
//
// AUTO STOP START (ASS) BUTTON:
// - Short press: toggles ASS Button ON/OFF, LED, and corresponding RF signal.
// - 3 sec hold: LED flashes 3x immediately; release before 5 sec to send
//   one factory ASS Button press to the master without toggling exhaust state.
// - 5 sec hold: LED flashes 10x; release before 10 sec to toggle startup mode.
// - 10 sec hold: cancels 5-sec action, reverses RF ON/OFF mapping,
//   flashes LED 20x, and resends current RF state.
//
// STARTUP MODES:
// - Memory: restores last ASS Button ON/OFF state and sends matching RF signal.
// - Default Off: starts ASS Button OFF and sends RF OFF.
// - Mode, ASS Button state, and RF inversion are saved in EEPROM.
//
// SPORT / SPORT PLUS:
// - Their LEDs remain controlled entirely by the factory master.
// - If both are OFF and either turns ON while ASS Button is OFF,
//   ASS Button automatically turns ON and sends RF ON.
// - Turning Sport/Sport Plus OFF does not turn ASS Button OFF.
//
// LIN / LED:
// - Blue Pill modifies ONLY ASS Button LED: 0D Data2 bit 0x40.
// - All other LED commands pass through unchanged from the master.
// - 0D frame validation/locking helps prevent malformed frames from
//   accidentally modifying other LEDs.
// - ASS Button short presses are hidden from the master; only the 3-sec
//   factory function sends the original ASS Button press.
// ============================================================

#if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
const uint8_t PIN_LIN1_SLP = 2;
const uint8_t PIN_LIN1_TX  = 18;
const uint8_t PIN_LIN1_RX  = 19;

const uint8_t PIN_LIN2_SLP = 3;
const uint8_t PIN_LIN2_TX  = 16;
const uint8_t PIN_LIN2_RX  = 17;

#elif defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_STM32)
const uint8_t PIN_LIN1_SLP = PA8;
const uint8_t PIN_LIN1_TX  = PA10;
const uint8_t PIN_LIN1_RX  = PA9; // swapped with PA10
const uint8_t PIN_LIN1_SEL = PB2;

const uint8_t PIN_LIN2_SLP = PA5;
const uint8_t PIN_LIN2_TX  = PA3; 
const uint8_t PIN_LIN2_RX  = PA2; // swapped with PA3
const uint8_t PIN_LIN2_SEL = PB1;
#else
#error "Unsupported board. Select an STM32 Blue Pill or Arduino Mega 2560 board."
#endif

const unsigned long LIN_BAUD = 19200;
const unsigned long PC_BAUD  = 115200;

const unsigned long NEW_FRAME_GAP_US = 800;
const unsigned int LIN_BREAK_US       = 1000;
const unsigned int LIN_BREAK_DELIM_US = 100;

const bool TREAT_MASTER_00_AS_BREAK = true;
const bool SUPPRESS_TX_ECHO = true;
const unsigned long ECHO_MAX_AGE_US = 2000;
const uint8_t ECHO_QUEUE_SIZE = 24;

const bool ENABLE_RF_OUTPUT = true;
const unsigned long RF_IDLE_US = 1500;

bool useEnhancedChecksum = true;

const uint8_t PID_LED_CMD    = 0x0D;
const uint8_t PID_BUTTON_RPT = 0x8E;

const uint8_t BUTTON7_PRESS_DATA1   = 0x80;
const uint8_t BUTTON7_LED_BIT_DATA2 = 0x40;

const unsigned long BUTTON7_FACTORY_HOLD_MS = 3000;
const unsigned long BUTTON7_MODE_HOLD_MS    = 5000;
const unsigned long BUTTON7_INVERT_HOLD_MS  = 10000;
const unsigned long BUTTON7_RELEASE_DEBOUNCE_MS = 200;

// Confirmation flashes use Button 7's LED.
const unsigned long BUTTON7_FLASH_INTERVAL_MS = 150;
const uint8_t FACTORY_FLASH_COUNT = 3;
const uint8_t MODE_FLASH_COUNT    = 10;
const uint8_t INVERT_FLASH_COUNT  = 20;

// EEPROM configuration.
const uint16_t EEPROM_CONFIG_BASE = 0;
const uint8_t EEPROM_MAGIC0 = 0xEC;
const uint8_t EEPROM_MAGIC1 = 0x47;
const uint8_t EEPROM_CONFIG_VERSION = 2;

const bool PRINT_ON_PRESS = false;

#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_STM32)
SoftwareSerial LinMasterSerial(PIN_LIN1_RX, PIN_LIN1_TX);  // PA10 / PA9
SoftwareSerial LinSlaveSerial(PIN_LIN2_RX, PIN_LIN2_TX);   // PA3 / PA2
#else
#define LinMasterSerial Serial1
#define LinSlaveSerial  Serial2
#endif

// -------------------- Echo suppression --------------------

struct EchoEntry {
  uint8_t value;
  unsigned long queuedUs;
};

struct EchoQueue {
  EchoEntry item[ECHO_QUEUE_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
};

EchoQueue echoA = {};
EchoQueue echoB = {};

void clearEchoQueue(EchoQueue &q) {
  q.head = 0;
  q.tail = 0;
  q.count = 0;
}

void expireOldEchoes(EchoQueue &q, unsigned long nowUs) {
  while (q.count > 0) {
    unsigned long age = nowUs - q.item[q.head].queuedUs;
    if (age <= ECHO_MAX_AGE_US) break;

    q.head++;
    if (q.head >= ECHO_QUEUE_SIZE) q.head = 0;
    q.count--;
  }
}

void queueExpectedEcho(EchoQueue &q, uint8_t value) {
  if (!SUPPRESS_TX_ECHO) return;

  unsigned long nowUs = micros();
  expireOldEchoes(q, nowUs);

  if (q.count >= ECHO_QUEUE_SIZE) {
    q.head++;
    if (q.head >= ECHO_QUEUE_SIZE) q.head = 0;
    q.count--;
  }

  q.item[q.tail].value = value;
  q.item[q.tail].queuedUs = nowUs;
  q.tail++;
  if (q.tail >= ECHO_QUEUE_SIZE) q.tail = 0;
  q.count++;
}

bool consumeIfVerifiedEcho(EchoQueue &q, uint8_t receivedByte) {
  if (!SUPPRESS_TX_ECHO) return false;

  unsigned long nowUs = micros();
  expireOldEchoes(q, nowUs);

  if (q.count == 0) return false;

  if (q.item[q.head].value == receivedByte) {
    q.head++;
    if (q.head >= ECHO_QUEUE_SIZE) q.head = 0;
    q.count--;
    return true;
  }

  clearEchoQueue(q);
  return false;
}

// -------------------- Main state --------------------

unsigned long lastRxUsA = 0;
unsigned long lastRxUsB = 0;

// Logical exhaust/Button 7 state.
bool button7LedWanted = false;

// Persistent user settings.
bool memoryMode = false;       // false = Default Off, true = Memory
bool rfCodesInverted = false;

// Button 7 physical hold tracking.
bool button7IsDown = false;
unsigned long button7HoldStartMs = 0;
unsigned long lastButton7SeenMs = 0;

// Hold-action state.
// 5-sec flash is only a preview; the mode change is committed on release if
// the hold ends before 10 sec. Reaching 10 sec cancels the 5-sec action.
bool button7FactoryAckShownThisHold = false;
bool button7ModePreviewShownThisHold = false;
bool button7InvertExecutedThisHold = false;

// A 3-second hold schedules exactly one factory Button 7 response.
// Because hold actions are mutually exclusive, classification happens at
// release; the factory press is therefore synthesized on a subsequent 8E poll.
bool pendingFactoryButton7Pulse = false;

// Button 7 confirmation flash state.
bool button7FlashActive = false;
bool button7FlashOn = false;
uint8_t button7FlashTransitionsRemaining = 0;
unsigned long button7FlashLastChangeMs = 0;

// Factory Sport / Sport Plus LED states from untouched master 0D Data3.
// Blue Pill never modifies these bits.
bool factorySportOn = false;
bool factorySportPlusOn = false;

// RF queue.
bool pendingRFSend = false;
bool pendingRFExplicitStateValid = false;
bool pendingRFExplicitState = false;
bool rf_channel_status = false;

// 8E response capture + rewrite.
uint8_t buttonResponseData[8] = {0};
uint8_t buttonResponseIndex = 0;
bool collectingButtonResponse = false;

bool button8ETransactionActive = false;
uint8_t button8EWireIndex = 255;

bool button8EData1Modified = false;
uint8_t button8EModifiedData1 = 0x00;

// Latest 0D state.
uint8_t lastLedData[8] = {0x00, 0x00, 0x00, 0x00, 0x89, 0xA1, 0xFE, 0xFF};
bool haveLastLedData = false;

// 0D LED modification safety lock.
// Button 7 LED modification is allowed only after several consecutive valid
// factory 0D frames have proven that the parser is aligned.
uint8_t valid0DFrameStreak = 0;
const uint8_t VALID_0D_FRAMES_TO_LOCK = 3;
bool zeroDFrameLocked = false;

// Per-current-0D-frame safety state.
bool current0DEarlyStructureValid = true;
bool current0DButton7Modified = false;

// A->B parser.
enum AState {
  A_WAIT_SYNC_OR_PID,
  A_WAIT_PID,
  A_PASS_THROUGH,
  A_CAPTURE_0D_DATA
};

AState aState = A_WAIT_SYNC_OR_PID;
uint8_t aCurrentPid = 0;
uint8_t ledData[8];
uint8_t ledDataIndex = 0;

// -------------------- RF helpers --------------------

void sendRFLogicalState(bool logicalOn) {
  if (!ENABLE_RF_OUTPUT) return;

  if (!rfCodesInverted) {
    if (logicalOn) rf433TurnOnChannel();
    else rf433TurnOffChannel();
  } else {
    if (logicalOn) rf433TurnOffChannel();
    else rf433TurnOnChannel();
  }

  rf_channel_status = logicalOn;
}

void queueExplicitRFState(bool logicalOn) {
  if (!ENABLE_RF_OUTPUT) return;

  pendingRFExplicitStateValid = true;
  pendingRFExplicitState = logicalOn;
  pendingRFSend = true;
}

void maybeSendQueuedRF() {
  if (!ENABLE_RF_OUTPUT) {
    pendingRFSend = false;
    pendingRFExplicitStateValid = false;
    return;
  }

  if (!pendingRFSend) return;
  if (rf433Busy()) return;

  if (LinMasterSerial.available() > 0) return;
  if (LinSlaveSerial.available() > 0) return;
  if (button8ETransactionActive) return;
  if (aState == A_CAPTURE_0D_DATA) return;

  unsigned long nowUs = micros();

  bool busAIdle =
      (lastRxUsA == 0) || ((nowUs - lastRxUsA) > RF_IDLE_US);
  bool busBIdle =
      (lastRxUsB == 0) || ((nowUs - lastRxUsB) > RF_IDLE_US);

  if (!(busAIdle && busBIdle)) return;

  pendingRFSend = false;

  if (pendingRFExplicitStateValid) {
    bool desiredState = pendingRFExplicitState;
    pendingRFExplicitStateValid = false;
    sendRFLogicalState(desiredState);
  }
}

// -------------------- LIN physical helpers --------------------

void wakeLinTransceivers() {
  pinMode(PIN_LIN1_SLP, OUTPUT);
  pinMode(PIN_LIN2_SLP, OUTPUT);

  digitalWrite(PIN_LIN1_SLP, HIGH);
  digitalWrite(PIN_LIN2_SLP, HIGH);
  delay(5);
}

#if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)

void sendLinBreakOnLinSlaveSerial() {
  LinSlaveSerial.flush();

  UCSR2B &= (uint8_t)~_BV(TXEN2);

  pinMode(PIN_LIN2_TX, OUTPUT);
  digitalWrite(PIN_LIN2_TX, LOW);
  delayMicroseconds(LIN_BREAK_US);
  digitalWrite(PIN_LIN2_TX, HIGH);
  delayMicroseconds(LIN_BREAK_DELIM_US);

  UCSR2B |= _BV(TXEN2);

  clearEchoQueue(echoB);
  aState = A_WAIT_SYNC_OR_PID;
}

void sendLinBreakOnLinMasterSerial() {
  LinMasterSerial.flush();

  UCSR1B &= (uint8_t)~_BV(TXEN1);

  pinMode(PIN_LIN1_TX, OUTPUT);
  digitalWrite(PIN_LIN1_TX, LOW);
  delayMicroseconds(LIN_BREAK_US);
  digitalWrite(PIN_LIN1_TX, HIGH);
  delayMicroseconds(LIN_BREAK_DELIM_US);

  UCSR1B |= _BV(TXEN1);

  clearEchoQueue(echoA);
}

#else

// Keep the STM32 break method from the working version unchanged.
void sendLinBreakOnLinSlaveSerial() {
  LinSlaveSerial.flush();
  LinSlaveSerial.end();
  pinMode(PIN_LIN2_TX, OUTPUT);
  digitalWrite(PIN_LIN2_TX, LOW);
  delayMicroseconds(LIN_BREAK_US);
  digitalWrite(PIN_LIN2_TX, HIGH);
  delayMicroseconds(LIN_BREAK_DELIM_US);
  LinSlaveSerial.begin(LIN_BAUD);

  clearEchoQueue(echoB);
  aState = A_WAIT_SYNC_OR_PID;
}

void sendLinBreakOnLinMasterSerial() {
  LinMasterSerial.flush();
  LinMasterSerial.end();
  pinMode(PIN_LIN1_TX, OUTPUT);
  digitalWrite(PIN_LIN1_TX, LOW);
  delayMicroseconds(LIN_BREAK_US);
  digitalWrite(PIN_LIN1_TX, HIGH);
  delayMicroseconds(LIN_BREAK_DELIM_US);
  LinMasterSerial.begin(LIN_BAUD);

  clearEchoQueue(echoA);
}

#endif

inline void forwardByteAtoB(uint8_t b) {
  LinSlaveSerial.write(b);
  queueExpectedEcho(echoB, b);
}

inline void forwardByteBtoA(uint8_t b) {
  LinMasterSerial.write(b);
  queueExpectedEcho(echoA, b);
}

// -------------------- Checksum --------------------

uint8_t linChecksum(const uint8_t *data, uint8_t len, uint8_t pid, bool enhanced) {
  uint16_t sum = 0;

  if (enhanced) {
    sum += pid;
    if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  }

  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
    if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  }

  return (uint8_t)(~sum);
}

// -------------------- EEPROM --------------------

uint8_t configChecksum(bool memMode, bool inverted, bool buttonState) {
  uint8_t c = EEPROM_MAGIC0 ^ EEPROM_MAGIC1 ^ EEPROM_CONFIG_VERSION;
  c ^= memMode ? 0xA5 : 0x5A;
  c ^= inverted ? 0x3C : 0xC3;
  c ^= buttonState ? 0x96 : 0x69;
  return c;
}

void saveConfigurationToEEPROM() {
  uint16_t a = EEPROM_CONFIG_BASE;

  EEPROM.update(a++, EEPROM_MAGIC0);
  EEPROM.update(a++, EEPROM_MAGIC1);
  EEPROM.update(a++, EEPROM_CONFIG_VERSION);
  EEPROM.update(a++, memoryMode ? 1 : 0);
  EEPROM.update(a++, rfCodesInverted ? 1 : 0);
  EEPROM.update(a++, button7LedWanted ? 1 : 0);
  EEPROM.update(a++, configChecksum(memoryMode, rfCodesInverted, button7LedWanted));
}

void loadConfigurationFromEEPROM() {
  uint16_t a = EEPROM_CONFIG_BASE;

  uint8_t m0 = EEPROM.read(a++);
  uint8_t m1 = EEPROM.read(a++);
  uint8_t version = EEPROM.read(a++);
  bool storedMemoryMode = EEPROM.read(a++) != 0;
  bool storedInverted = EEPROM.read(a++) != 0;
  bool storedButton7State = EEPROM.read(a++) != 0;
  uint8_t storedChecksum = EEPROM.read(a++);

  bool valid =
      m0 == EEPROM_MAGIC0 &&
      m1 == EEPROM_MAGIC1 &&
      version == EEPROM_CONFIG_VERSION &&
      storedChecksum ==
          configChecksum(storedMemoryMode, storedInverted, storedButton7State);

  if (!valid) {
    memoryMode = false;          // Default Off
    rfCodesInverted = false;
    button7LedWanted = false;
    return;
  }

  memoryMode = storedMemoryMode;
  rfCodesInverted = storedInverted;

  if (memoryMode) {
    button7LedWanted = storedButton7State;
  } else {
    button7LedWanted = false;
  }
}

// -------------------- Button 7 helpers --------------------

bool isButton7Payload(const uint8_t *d) {
  return d[0] == 0x00 &&
         d[1] == BUTTON7_PRESS_DATA1 &&
         d[2] == 0x00 &&
         d[3] == 0x00 &&
         d[4] == 0x00 &&
         d[5] == 0x00 &&
         d[6] == 0x00 &&
         d[7] == 0x00;
}

void setButton7LogicalState(bool on, bool sendRF) {
  button7LedWanted = on;

  // Always save the latest state. Default-Off mode simply ignores this state
  // at startup; Memory mode restores it.
  saveConfigurationToEEPROM();

  if (sendRF) {
    queueExplicitRFState(on);
  }
}

void toggleButton7LogicalState() {
  setButton7LogicalState(!button7LedWanted, true);
}

void startButton7Flash(uint8_t flashCount) {
  button7FlashActive = true;
  button7FlashOn = true;
  button7FlashTransitionsRemaining = flashCount * 2 - 1;
  button7FlashLastChangeMs = millis();
}

void updateButton7Flash() {
  if (!button7FlashActive) return;

  unsigned long nowMs = millis();
  if ((nowMs - button7FlashLastChangeMs) < BUTTON7_FLASH_INTERVAL_MS) return;

  button7FlashLastChangeMs = nowMs;
  button7FlashOn = !button7FlashOn;

  if (button7FlashTransitionsRemaining > 0) {
    button7FlashTransitionsRemaining--;
  }

  if (button7FlashTransitionsRemaining == 0) {
    button7FlashActive = false;
    button7FlashOn = false;
  }
}

bool effectiveButton7LedWanted() {
  if (button7FlashActive) return button7FlashOn;
  return button7LedWanted;
}

void handleButton7Response(const uint8_t *data) {
  if (!isButton7Payload(data)) return;

  unsigned long nowMs = millis();
  lastButton7SeenMs = nowMs;

  if (!button7IsDown) {
    button7IsDown = true;
    button7HoldStartMs = nowMs;
    button7FactoryAckShownThisHold = false;
    button7ModePreviewShownThisHold = false;
    button7InvertExecutedThisHold = false;

    #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
    if (PRINT_ON_PRESS) {
      Serial.println(F("Button 7 hold started"));
    }
    #endif
  }
}

void updateButton7HoldProgress() {
  if (!button7IsDown || button7HoldStartMs == 0) return;

  unsigned long heldMs = millis() - button7HoldStartMs;

  // 10-second action executes immediately while still held.
  // It overrides/cancels the 5-second mode action.
  if (!button7InvertExecutedThisHold &&
      heldMs >= BUTTON7_INVERT_HOLD_MS) {

    button7InvertExecutedThisHold = true;

    rfCodesInverted = !rfCodesInverted;
    saveConfigurationToEEPROM();

    // Replace any 10-flash preview with the 20-flash inversion confirmation.
    startButton7Flash(INVERT_FLASH_COUNT);

    // Immediately resend the CURRENT logical exhaust state using the NEW RF
    // mapping so the receiver is synchronized to the inversion change.
    queueExplicitRFState(button7LedWanted);

    #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
    if (PRINT_ON_PRESS) {
      Serial.print(F("Button 7 10 sec: RF inversion = "));
      Serial.println(rfCodesInverted ? F("ON") : F("OFF"));
      Serial.println(F("5-sec mode action cancelled; RF state resent."));
    }
    #endif
    return;
  }

  // At 3 seconds, acknowledge the factory-hold threshold immediately while
  // the button is STILL being held. This is feedback only; the actual factory
  // Button 7 pulse is still scheduled on release if the hold ends before 5 sec.
  if (!button7FactoryAckShownThisHold &&
      !button7ModePreviewShownThisHold &&
      !button7InvertExecutedThisHold &&
      heldMs >= BUTTON7_FACTORY_HOLD_MS) {

    button7FactoryAckShownThisHold = true;
    startButton7Flash(FACTORY_FLASH_COUNT);

    #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
    if (PRINT_ON_PRESS) {
      Serial.println(F("Button 7 3 sec: factory-hold threshold acknowledged."));
    }
    #endif
  }

  // At 5 seconds, provide feedback immediately while the button is still held.
  // Do NOT commit the mode change yet because the user may continue to 10 sec.
  if (!button7ModePreviewShownThisHold &&
      !button7InvertExecutedThisHold &&
      heldMs >= BUTTON7_MODE_HOLD_MS) {

    button7ModePreviewShownThisHold = true;
    startButton7Flash(MODE_FLASH_COUNT);

    #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
    if (PRINT_ON_PRESS) {
      Serial.println(F("Button 7 5 sec: mode-change preview (commit on release before 10 sec)."));
    }
    #endif
  }
}

void classifyAndExecuteButton7Hold(unsigned long heldMs) {
  // If the 10-second action already executed while held, release does nothing.
  if (button7InvertExecutedThisHold) {
    return;
  }

  // 5 to <10 sec: commit the startup-mode change on release.
  // The 10-flash feedback already occurred at the 5-second threshold.
  if (heldMs >= BUTTON7_MODE_HOLD_MS) {
    memoryMode = !memoryMode;
    saveConfigurationToEEPROM();

    #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
    if (PRINT_ON_PRESS) {
      Serial.print(F("Button 7 released after 5 sec: startup mode committed = "));
      Serial.println(memoryMode ? F("MEMORY") : F("DEFAULT OFF"));
    }
    #endif
    return;
  }

  // 3 to <5 sec: factory Button 7 function only.
  if (heldMs >= BUTTON7_FACTORY_HOLD_MS) {
    pendingFactoryButton7Pulse = true;

    #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
    if (PRINT_ON_PRESS) {
      Serial.println(F("Button 7 3 sec: one factory press scheduled"));
    }
    #endif
    return;
  }

  // Short press only.
  toggleButton7LogicalState();

  #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
  if (PRINT_ON_PRESS) {
    Serial.print(F("Button 7 short press: exhaust = "));
    Serial.println(button7LedWanted ? F("ON") : F("OFF"));
  }
  #endif
}

void updateButton7ReleaseTimeout() {
  if (!button7IsDown) return;

  unsigned long nowMs = millis();

  if ((nowMs - lastButton7SeenMs) >= BUTTON7_RELEASE_DEBOUNCE_MS) {
    // Use last genuine Button 7 report rather than the debounce time itself.
    unsigned long heldMs = lastButton7SeenMs - button7HoldStartMs;

    button7IsDown = false;
    button7HoldStartMs = 0;

    classifyAndExecuteButton7Hold(heldMs);

    button7FactoryAckShownThisHold = false;
    button7ModePreviewShownThisHold = false;
    button7InvertExecutedThisHold = false;
  }
}

// -------------------- Sport / Sport Plus --------------------
//
// Sport and Sport Plus LED state is controlled entirely by the master.
// We do not modify Data3. Instead, the untouched master 0D Data3 rising edge
// is used as the trigger for Button 7 auto-ON.

void handleMasterSportStateTransition(bool newSportOn, bool newSportPlusOn) {
  bool bothWereOff = !factorySportOn && !factorySportPlusOn;
  bool eitherTurnedOn =
      (!factorySportOn && newSportOn) ||
      (!factorySportPlusOn && newSportPlusOn);

  if (bothWereOff && eitherTurnedOn && !button7LedWanted) {
    // Only Button 7 is changed. Sport/Sport Plus remain master-controlled.
    setButton7LogicalState(true, true);

    #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
    if (PRINT_ON_PRESS) {
      Serial.println(F("Master Sport/Sport Plus OFF->ON: Button 7 auto-ON."));
    }
    #endif
  }

  factorySportOn = newSportOn;
  factorySportPlusOn = newSportPlusOn;
}

// -------------------- Complete 8E response --------------------

void processCompleteButtonResponse(const uint8_t *data) {
  handleButton7Response(data);
}

void monitorSlaveByte(uint8_t b, bool newFrame) {
  (void)newFrame;

  if (!collectingButtonResponse) return;

  if (buttonResponseIndex < 8) {
    buttonResponseData[buttonResponseIndex++] = b;

    if (buttonResponseIndex == 8) {
      processCompleteButtonResponse(buttonResponseData);
      collectingButtonResponse = false;
    }
  }
}


// -------------------- 0D frame structure guards --------------------

// Known/allowed bit usage from the switch LED map:
//   Data0: PASM 0x30 + blank 0x08 => allowed mask 0x38
//   Data1: currently unused => expected 0x00
//   Data2: spoiler 0x08 + Button 7 0x40 => allowed mask 0x48
//   Data3: Sport 0x01, blank 0x02, PSM 0x04, PSE 0x10,
//          Sport Plus 0x20, blank 0x80 => allowed mask 0xB7
//   Data4..7: fixed 89 A1 FE FF in the observed/master LED frames.
bool early0DByteLooksValid(uint8_t index, uint8_t value) {
  if (index == 0) return (value & (uint8_t)~0x38) == 0;
  if (index == 1) return value == 0x00;
  if (index == 2) return (value & (uint8_t)~0x48) == 0;
  return true;
}

bool full0DStructureLooksValid(const uint8_t *d) {
  if ((d[0] & (uint8_t)~0x38) != 0) return false;
  if (d[1] != 0x00) return false;
  if ((d[2] & (uint8_t)~0x48) != 0) return false;
  if ((d[3] & (uint8_t)~0xB7) != 0) return false;

  if (d[4] != 0x89) return false;
  if (d[5] != 0xA1) return false;
  if (d[6] != 0xFE) return false;
  if (d[7] != 0xFF) return false;

  return true;
}

// -------------------- 0D LED modification --------------------

void applyButton7LedToData(uint8_t *d) {
  if (effectiveButton7LedWanted()) {
    d[2] |= BUTTON7_LED_BIT_DATA2;
  } else {
    d[2] &= (uint8_t)~BUTTON7_LED_BIT_DATA2;
  }
}

void finishStreamed0DFrame(uint8_t receivedChecksum) {
  uint8_t origClassic  = linChecksum(ledData, 8, PID_LED_CMD, false);
  uint8_t origEnhanced = linChecksum(ledData, 8, PID_LED_CMD, true);

  bool checksumValid = false;
  bool frameEnhanced = useEnhancedChecksum;

  if (receivedChecksum == origClassic) {
    checksumValid = true;
    frameEnhanced = false;
  } else if (receivedChecksum == origEnhanced) {
    checksumValid = true;
    frameEnhanced = true;
  }

  bool structureValid =
      current0DEarlyStructureValid &&
      full0DStructureLooksValid(ledData);

  bool factoryFrameTrusted = checksumValid && structureValid;

  if (!factoryFrameTrusted) {
    // Immediately drop parser confidence. Button 7 modification is disabled
    // until several clean factory 0D frames rebuild the lock.
    valid0DFrameStreak = 0;
    zeroDFrameLocked = false;

    // If Data2 was already modified earlier in this streamed frame, DO NOT
    // synthesize a checksum that would legitimize the modified/corrupt data.
    // Forward the master's original checksum. Because Data2 differs, the frame
    // checksum will intentionally fail and the switch should reject the frame.
    //
    // If Data2 was not modified, this is a pure pass-through of the factory
    // bytes/checksum.
    forwardByteAtoB(receivedChecksum);
    return;
  }

  // Trusted frame.
  useEnhancedChecksum = frameEnhanced;

  if (valid0DFrameStreak < 255) valid0DFrameStreak++;
  if (valid0DFrameStreak >= VALID_0D_FRAMES_TO_LOCK) {
    zeroDFrameLocked = true;
  }

  for (uint8_t i = 0; i < 8; i++) {
    lastLedData[i] = ledData[i];
  }
  haveLastLedData = true;

  // Observe Sport/Sport Plus only after the master frame is fully trusted.
  bool newFactorySportOn = (ledData[3] & 0x01) != 0;
  bool newFactorySportPlusOn = (ledData[3] & 0x20) != 0;
  handleMasterSportStateTransition(newFactorySportOn, newFactorySportPlusOn);

  if (current0DButton7Modified) {
    // Recalculate checksum ONLY because the sole intended modification was
    // Button 7's Data2 bit 0x40 on a fully trusted factory frame.
    uint8_t modifiedData[8];
    for (uint8_t i = 0; i < 8; i++) modifiedData[i] = ledData[i];

    applyButton7LedToData(modifiedData);

    uint8_t newChecksum =
        linChecksum(modifiedData, 8, PID_LED_CMD, useEnhancedChecksum);

    forwardByteAtoB(newChecksum);
  } else {
    // No Blue Pill LED change was made. Preserve the factory checksum exactly.
    forwardByteAtoB(receivedChecksum);
  }
}

// -------------------- 8E transaction state --------------------

void resetButton8ETransactionState() {
  button8ETransactionActive = false;
  button8EWireIndex = 255;
  button8EData1Modified = false;
  button8EModifiedData1 = 0;

  collectingButtonResponse = false;
  buttonResponseIndex = 0;
}

void noteMasterPid(uint8_t pid) {
  aCurrentPid = pid;

  resetButton8ETransactionState();

  if (pid == PID_BUTTON_RPT) {
    collectingButtonResponse = true;
    buttonResponseIndex = 0;

    button8ETransactionActive = true;
    button8EWireIndex = 0;
  }
}

// -------------------- Master -> switch --------------------

void processMasterByte(uint8_t b, bool newFrame) {
  if (newFrame) {
    aState = A_WAIT_SYNC_OR_PID;
    ledDataIndex = 0;
  }

  if (aState == A_WAIT_SYNC_OR_PID) {
    forwardByteAtoB(b);

    if (b == 0x55) {
      aState = A_WAIT_PID;
    } else {
      if (b == PID_LED_CMD || b == PID_BUTTON_RPT) {
        noteMasterPid(b);

        if (b == PID_LED_CMD) {
          aState = A_CAPTURE_0D_DATA;
          ledDataIndex = 0;
          current0DEarlyStructureValid = true;
          current0DButton7Modified = false;
        } else {
          aState = A_PASS_THROUGH;
        }
      } else {
        aState = A_PASS_THROUGH;
      }
    }
    return;
  }

  if (aState == A_WAIT_PID) {
    noteMasterPid(b);
    forwardByteAtoB(b);

    if (b == PID_LED_CMD) {
      aState = A_CAPTURE_0D_DATA;
      ledDataIndex = 0;
      current0DEarlyStructureValid = true;
      current0DButton7Modified = false;
    } else {
      aState = A_PASS_THROUGH;
    }
    return;
  }

  if (aState == A_CAPTURE_0D_DATA) {
    if (ledDataIndex < 8) {
      ledData[ledDataIndex] = b;

      uint8_t outByte = b;

      // Continuously verify the early byte positions before Data2 is touched.
      if (!early0DByteLooksValid(ledDataIndex, b)) {
        current0DEarlyStructureValid = false;
      }

      // BLUE PILL LED MODIFICATION POLICY:
      // Modify ONLY Button 7's LED bit: 0D Data2 bit 0x40.
      //
      // The modification is permitted only when:
      //   1) prior 0D traffic has established a stable parser lock, AND
      //   2) Data0/Data1/current Data2 all look structurally valid.
      //
      // All other bytes are always passed through exactly as received.
      if (ledDataIndex == 2 &&
          zeroDFrameLocked &&
          current0DEarlyStructureValid) {

        uint8_t originalData2 = outByte;

        if (effectiveButton7LedWanted()) {
          outByte |= BUTTON7_LED_BIT_DATA2;
        } else {
          outByte &= (uint8_t)~BUTTON7_LED_BIT_DATA2;
        }

        if (outByte != originalData2) {
          current0DButton7Modified = true;
        }
      }

      forwardByteAtoB(outByte);
      ledDataIndex++;
      return;
    }

    finishStreamed0DFrame(b);
    aState = A_PASS_THROUGH;
    return;
  }

  forwardByteAtoB(b);
}

void pumpAtoBOne() {
  if (LinMasterSerial.available() <= 0) return;

  uint8_t b = (uint8_t)LinMasterSerial.read();
  unsigned long now = micros();
  bool newFrame = (lastRxUsA == 0) || ((now - lastRxUsA) > NEW_FRAME_GAP_US);
  lastRxUsA = now;

  if (consumeIfVerifiedEcho(echoA, b)) {
    return;
  }

  if (TREAT_MASTER_00_AS_BREAK && newFrame && b == 0x00) {
    sendLinBreakOnLinSlaveSerial();
    return;
  }

  processMasterByte(b, newFrame);
}

// -------------------- Switch -> master --------------------

void pumpBtoAOne() {
  if (LinSlaveSerial.available() <= 0) return;

  uint8_t b = (uint8_t)LinSlaveSerial.read();
  unsigned long now = micros();
  bool newFrame = (lastRxUsB == 0) || ((now - lastRxUsB) > NEW_FRAME_GAP_US);
  lastRxUsB = now;

  if (consumeIfVerifiedEcho(echoB, b)) {
    return;
  }

  if (button8ETransactionActive) {

    // Data bytes 0..7.
    if (button8EWireIndex < 8) {
      uint8_t outByte = b;

      // Store ORIGINAL byte.
      buttonResponseData[button8EWireIndex] = b;

      if (button8EWireIndex == 1 &&
          buttonResponseData[0] == 0x00) {

        // Physical Button 7 is always hidden from the master.
        if (b == BUTTON7_PRESS_DATA1) {
          outByte = 0x00;
          button8EData1Modified = true;
          button8EModifiedData1 = 0x00;
        }

        // A completed 3-second hold requests one factory Button 7 press.
        // Since the hold duration is classified only on release, synthesize
        // that press on the next response whose Data1 is idle.
        else if (pendingFactoryButton7Pulse && b == 0x00) {
          outByte = BUTTON7_PRESS_DATA1;
          button8EData1Modified = true;
          button8EModifiedData1 = BUTTON7_PRESS_DATA1;
          pendingFactoryButton7Pulse = false;
        }
      }

      monitorSlaveByte(b, newFrame);
      forwardByteBtoA(outByte);

      button8EWireIndex++;
      return;
    }

    // Checksum byte.
    if (button8EWireIndex == 8) {
      uint8_t outChecksum = b;

      if (button8EData1Modified) {
        uint8_t modified[8];

        for (uint8_t i = 0; i < 8; i++) {
          modified[i] = buttonResponseData[i];
        }

        modified[1] = button8EModifiedData1;

        uint8_t originalClassic =
            linChecksum(buttonResponseData, 8, PID_BUTTON_RPT, false);
        uint8_t originalEnhanced =
            linChecksum(buttonResponseData, 8, PID_BUTTON_RPT, true);

        bool responseEnhanced = true;

        if (b == originalClassic) {
          responseEnhanced = false;
        } else if (b == originalEnhanced) {
          responseEnhanced = true;
        }

        outChecksum =
            linChecksum(modified, 8, PID_BUTTON_RPT, responseEnhanced);
      }

      forwardByteBtoA(outChecksum);
      resetButton8ETransactionState();
      return;
    }
  }

  monitorSlaveByte(b, newFrame);
  forwardByteBtoA(b);
}

// -------------------- Optional Mega debug --------------------

#if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
void handlePcSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '?') {
      Serial.print(F("button7LedWanted = "));
      Serial.println(button7LedWanted ? F("ON") : F("OFF"));

      Serial.print(F("startup mode = "));
      Serial.println(memoryMode ? F("MEMORY") : F("DEFAULT OFF"));

      Serial.print(F("rfCodesInverted = "));
      Serial.println(rfCodesInverted ? F("YES") : F("NO"));

      Serial.print(F("factorySportOn = "));
      Serial.println(factorySportOn ? F("YES") : F("NO"));

      Serial.print(F("factorySportPlusOn = "));
      Serial.println(factorySportPlusOn ? F("YES") : F("NO"));

      Serial.print(F("pendingFactoryButton7Pulse = "));
      Serial.println(pendingFactoryButton7Pulse ? F("YES") : F("NO"));

      Serial.print(F("modePreviewShownThisHold = "));
      Serial.println(button7ModePreviewShownThisHold ? F("YES") : F("NO"));

      Serial.print(F("invertExecutedThisHold = "));
      Serial.println(button7InvertExecutedThisHold ? F("YES") : F("NO"));
    }
  }
}
#endif

// -------------------- Setup / loop --------------------

void setup() {
  #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
  Serial.begin(PC_BAUD);
  Serial.println(F("Project Intercept - fixed Button 7 exhaust controller"));
  #endif

  pinMode(PIN_LIN1_SEL, OUTPUT);
  pinMode(PIN_LIN2_SEL, OUTPUT);

  digitalWrite(PIN_LIN1_SEL, HIGH);
  digitalWrite(PIN_LIN2_SEL, HIGH);

  loadConfigurationFromEEPROM();

  wakeLinTransceivers();

#if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
  LinMasterSerial.begin(LIN_BAUD, SERIAL_8N1);
  LinSlaveSerial.begin(LIN_BAUD, SERIAL_8N1);
#else
  LinMasterSerial.begin(LIN_BAUD);
  LinSlaveSerial.begin(LIN_BAUD);
#endif

  rf433Begin();

  // Required startup behavior:
  // Memory mode restores last Button 7 state; Default-Off forces OFF.
  // loadConfigurationFromEEPROM() has already selected button7LedWanted.
  if (ENABLE_RF_OUTPUT) {
    sendRFLogicalState(button7LedWanted);
  } else {
    rf_channel_status = button7LedWanted;
  }

  // In Default-Off mode, persist OFF as the current state as well.
  if (!memoryMode && button7LedWanted) {
    button7LedWanted = false;
    saveConfigurationToEEPROM();
  }
}

void loop() {
  // Preserve the working bridge behavior: service switch/slave first.
  for (uint8_t i = 0; i < 32; i++) {
    bool didWork = false;

    if (LinSlaveSerial.available() > 0) {
      pumpBtoAOne();
      didWork = true;
    }

    if (LinMasterSerial.available() > 0) {
      pumpAtoBOne();
      didWork = true;
    }

    if (!didWork) break;
  }

  updateButton7HoldProgress();
  updateButton7ReleaseTimeout();
  updateButton7Flash();
  maybeSendQueuedRF();

  #if defined(ARDUINO_MEGA_2560) || defined(ARDUINO_AVR_MEGA2560)
  if (Serial.available() > 0) {
    handlePcSerial();
  }
  #endif
}