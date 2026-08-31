#include "ask_hal.h"
#include "rf433.h"
#include "define_board.h"

// ============================================================
//  LIN MITM - RAW FORWARDER + BUTTON 9 FAST PRESS LED INJECTOR
//  Arduino Mega 2560
// ------------------------------------------------------------
//  Bus A = factory master side    Serial1: TX=18 RX=19 SLP=2
//  Bus B = switch/slave side      Serial2: TX=16 RX=17 SLP=3
//
//  Fixes for quick press:
//    1) Detects the button-9 press as soon as Data0..Data2 = 00 40 00
//       after an 8E poll, instead of waiting for all 8 data bytes.
//    2) Stores the latest factory 0D LED command.
//    3) After a toggle, sends an immediate refreshed 0D LED command to the
//       switch side once the bus is briefly idle, so the LED turns on/off
//       without waiting for the next factory LED update.
// ============================================================

// -------------------- Config -------------------------

#if defined(ARDUINO_MEGA_2560)
const uint8_t PIN_LIN1_SLP = 2;
const uint8_t PIN_LIN1_TX  = 18;  // Serial1 TX, master-side transceiver TXD
const uint8_t PIN_LIN1_RX  = 19;  // Serial1 RX, master-side transceiver TXD

const uint8_t PIN_LIN2_SLP = 3;
const uint8_t PIN_LIN2_TX  = 16;  // Serial2 TX, switch-side transceiver TXD
const uint8_t PIN_LIN2_RX  = 17;  // Serial2 RX, master-side transceiver TXD

#elif defined(ARDUINO_STM32)
const uint8_t PIN_LIN1_SLP = PA8;
const uint8_t PIN_LIN1_TX  = PA9;  // Serial1 TX, master-side transceiver TXD
const uint8_t PIN_LIN1_RX  = PA10;  // Serial1 RX, master-side transceiver TXD

const uint8_t PIN_LIN2_SLP = PA5;
const uint8_t PIN_LIN2_TX  = PA2;  // Serial2 TX, switch-side transceiver TXD
const uint8_t PIN_LIN2_RX  = PA3;  // Serial2 RX, master-side transceiver TXD
#endif

const unsigned long LIN_BAUD = 19200;
const unsigned long PC_BAUD  = 115200;

const unsigned long NEW_FRAME_GAP_US = 2500;

const unsigned int LIN_BREAK_US       = 1000;
const unsigned int LIN_BREAK_DELIM_US = 100;

const bool TREAT_MASTER_00_AS_BREAK = true;
const bool SUPPRESS_TX_ECHO = true;

// Start with enhanced; auto-detects from factory 0D checksum.
bool useEnhancedChecksum = true;

const uint8_t PID_LED_CMD    = 0x0D;
const uint8_t PID_BUTTON_RPT = 0x8E;

const uint8_t BLANK_LED_BIT_DATA3 = 0x02;

// Re-arm after the button-9-active report stops.
const unsigned long BUTTON_RELEASE_TIMEOUT_MS = 120;

// Send immediate LED refresh after a toggle once switch-side bus is idle.
// At 19200 baud, 10 bits/byte ~= 520 us. 1500 us gives a small idle window.
const unsigned long IMMEDIATE_LED_IDLE_US = 1500;

// Limit debug prints; serial printing can make LIN timing worse.
const bool PRINT_ON_PRESS = false;

//HardwareSerial Serial1(PIN_LIN1_RX, PIN_LIN1_TX);   // RX, TX
HardwareSerial Serial2(PIN_LIN2_RX, PIN_LIN2_TX);   // RX, TX

// -------------------- State -------------------------
unsigned long lastRxUsA = 0;
unsigned long lastRxUsB = 0;

volatile uint8_t suppressA = 0;
volatile uint8_t suppressB = 0;

bool button9LedWanted = false;
bool button9IsDown = false;
unsigned long lastButton9SeenMs = 0;

bool pendingImmediateLedRefresh = false;

uint8_t lastMasterPidToSwitch = 0x00;
bool expectingButtonResponse = false;

// Latest factory 0D LED frame data, used for immediate refresh.
uint8_t lastLedData[8] = {0x00, 0x00, 0x00, 0x00, 0x89, 0xA1, 0xFE, 0xFF};
bool haveLastLedData = false;
bool rf_channel_status = false;
// -------------------- A->B parser state -------------
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

// -------------------- Rolling B detector ------------
uint8_t bWin3[3] = {0};
uint8_t bWin8[8] = {0};
uint8_t bWin9[9] = {0};
uint8_t bCount = 0;

// -------------------- Helpers ------------------------

void sendRFCommands()
{
  if (rf_channel_status == true) {
    rf433TurnOffChannel();
    rf_channel_status = false;
  } else {
    rf433TurnOnChannel();
    rf_channel_status = true;
  }
}
void wakeLinTransceivers() {
  pinMode(PIN_LIN1_SLP, OUTPUT);
  pinMode(PIN_LIN2_SLP, OUTPUT);

  digitalWrite(PIN_LIN1_SLP, HIGH);
  digitalWrite(PIN_LIN2_SLP, HIGH);
  delay(5);
}

void sendLinBreakOnSerial2() {
  Serial2.flush();
  Serial2.end();
  pinMode(PIN_LIN2_TX, OUTPUT);
  digitalWrite(PIN_LIN2_TX, LOW);
  delayMicroseconds(LIN_BREAK_US);
  digitalWrite(PIN_LIN2_TX, HIGH);
  delayMicroseconds(LIN_BREAK_DELIM_US);
  Serial2.begin(LIN_BAUD, SERIAL_8N1);

  aState = A_WAIT_SYNC_OR_PID;
}

void sendLinBreakOnSerial1() {
  Serial1.flush();
  Serial1.end();
  pinMode(PIN_LIN1_TX, OUTPUT);
  digitalWrite(PIN_LIN1_TX, LOW);
  delayMicroseconds(LIN_BREAK_US);
  digitalWrite(PIN_LIN1_TX, HIGH);
  delayMicroseconds(LIN_BREAK_DELIM_US);
  Serial1.begin(LIN_BAUD, SERIAL_8N1);
}

inline void forwardByteAtoB(uint8_t b) {
  Serial2.write(b);
  if (SUPPRESS_TX_ECHO && suppressB < 250) suppressB++;
}

inline void forwardByteBtoA(uint8_t b) {
  Serial1.write(b);
  if (SUPPRESS_TX_ECHO && suppressA < 250) suppressA++;
}

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

bool isBlankButtonData3Prefix(const uint8_t *d) {
  // Early detection for Button Position 9 blank:
  // Data0=00 Data1=40 Data2=00.
  return d[0] == 0x00 &&
         d[1] == 0x40 &&
         d[2] == 0x00;
}

bool isBlankButtonData8(const uint8_t *d) {
  return d[0] == 0x00 &&
         d[1] == 0x40 &&
         d[2] == 0x00 &&
         d[3] == 0x00 &&
         d[4] == 0x00 &&
         d[5] == 0x00 &&
         d[6] == 0x00 &&
         d[7] == 0x00;
}

bool isBlankButtonPidPlusData9(const uint8_t *d) {
  return d[0] == PID_BUTTON_RPT && isBlankButtonData8(&d[1]);
}

void handleButton9Pressed() {
  unsigned long nowMs = millis();
  lastButton9SeenMs = nowMs;

  if (button9IsDown) {
    return;
  }

  button9IsDown = true;
  button9LedWanted = !button9LedWanted;
  pendingImmediateLedRefresh = true;

  #if defined(ARDUINO_MEGA_2560)
  if (PRINT_ON_PRESS) {
    Serial.print(F("Button 9 press edge. button9LedWanted = "));
    Serial.println(button9LedWanted ? F("ON") : F("OFF"));
  }
  #endif
}

void handleButton9Released() {
  button9IsDown = false;
}

void updateButton9ReleaseTimeout() {
  if (button9IsDown) {
    unsigned long nowMs = millis();
    if ((nowMs - lastButton9SeenMs) > BUTTON_RELEASE_TIMEOUT_MS) {
      handleButton9Released();
    }
  }
}

void resetRollingDetector() {
  for (uint8_t i = 0; i < 3; i++) bWin3[i] = 0;
  for (uint8_t i = 0; i < 8; i++) bWin8[i] = 0;
  for (uint8_t i = 0; i < 9; i++) bWin9[i] = 0;
  bCount = 0;
}

// Rolling/sliding detector. It detects:
//   - early 3-byte prefix after 8E: 00 40 00
//   - full 8-byte data response:   00 40 00 00 00 00 00 00
//   - logged 9-byte form:          8E 00 40 00 00 00 00 00 00
void monitorSlaveByte(uint8_t b, bool newFrame) {
  if (newFrame) {
    resetRollingDetector();
  }

  for (uint8_t i = 0; i < 2; i++) bWin3[i] = bWin3[i + 1];
  bWin3[2] = b;

  for (uint8_t i = 0; i < 7; i++) bWin8[i] = bWin8[i + 1];
  bWin8[7] = b;

  for (uint8_t i = 0; i < 8; i++) bWin9[i] = bWin9[i + 1];
  bWin9[8] = b;

  if (bCount < 9) bCount++;

  if ((lastMasterPidToSwitch == PID_BUTTON_RPT || expectingButtonResponse)) {
    // Fast path: toggle as soon as first 3 data bytes identify the button.
    if (bCount >= 3 && isBlankButtonData3Prefix(bWin3)) {
      handleButton9Pressed();
      expectingButtonResponse = false;
      // Do not reset the detector here; repeated bytes from same response are
      // ignored by button9IsDown anyway.
      return;
    }

    if (bCount >= 8 && isBlankButtonData8(bWin8)) {
      handleButton9Pressed();
      expectingButtonResponse = false;
      return;
    }
  }

  if (bCount >= 9 && isBlankButtonPidPlusData9(bWin9)) {
    handleButton9Pressed();
    expectingButtonResponse = false;
    return;
  }
}

void applyBlankLedToData(uint8_t *d) {
  if (button9LedWanted) {
    d[3] |= BLANK_LED_BIT_DATA3;
  } else {
    d[3] &= (uint8_t)~BLANK_LED_BIT_DATA3;
  }
}

void send0DFrameToSwitch(const uint8_t *data) {
  uint8_t temp[8];
  for (uint8_t i = 0; i < 8; i++) temp[i] = data[i];

  applyBlankLedToData(temp);
  uint8_t cs = linChecksum(temp, 8, PID_LED_CMD, useEnhancedChecksum);

  sendLinBreakOnSerial2();
  forwardByteAtoB(0x55);
  forwardByteAtoB(PID_LED_CMD);
  for (uint8_t i = 0; i < 8; i++) {
    forwardByteAtoB(temp[i]);
  }
  forwardByteAtoB(cs);
  Serial2.flush();
}

void maybeSendImmediateLedRefresh() {
  if (!pendingImmediateLedRefresh || !haveLastLedData) return;

  // Wait for both sides to be briefly idle. This avoids injecting in the
  // middle of a normal frame.
  unsigned long now = micros();
  bool busAIdle = (lastRxUsA == 0) || ((now - lastRxUsA) > IMMEDIATE_LED_IDLE_US);
  bool busBIdle = (lastRxUsB == 0) || ((now - lastRxUsB) > IMMEDIATE_LED_IDLE_US);

  if (busAIdle && busBIdle) {
    pendingImmediateLedRefresh = false;
    send0DFrameToSwitch(lastLedData);

    // Send the RF Command
    sendRFCommands();
  }
}

void sendModified0DFrameToSwitch(uint8_t receivedChecksum) {
  uint8_t originalData[8];
  for (uint8_t i = 0; i < 8; i++) {
    originalData[i] = ledData[i];
    lastLedData[i] = ledData[i];  // store unmodified factory LED frame
  }
  haveLastLedData = true;

  uint8_t origClassic  = linChecksum(originalData, 8, PID_LED_CMD, false);
  uint8_t origEnhanced = linChecksum(originalData, 8, PID_LED_CMD, true);

  if (receivedChecksum == origClassic) {
    useEnhancedChecksum = false;
  } else if (receivedChecksum == origEnhanced) {
    useEnhancedChecksum = true;
  }

  applyBlankLedToData(ledData);
  uint8_t newChecksum = linChecksum(ledData, 8, PID_LED_CMD, useEnhancedChecksum);

  for (uint8_t i = 0; i < 8; i++) {
    forwardByteAtoB(ledData[i]);
  }
  forwardByteAtoB(newChecksum);
}

void noteMasterPid(uint8_t pid) {
  aCurrentPid = pid;
  lastMasterPidToSwitch = pid;

  if (pid == PID_BUTTON_RPT) {
    expectingButtonResponse = true;
    resetRollingDetector();
  }
}

// Process one byte from factory master side.
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
    } else {
      aState = A_PASS_THROUGH;
    }
    return;
  }

  if (aState == A_CAPTURE_0D_DATA) {
    if (ledDataIndex < 8) {
      ledData[ledDataIndex++] = b;
      return;
    } else {
      uint8_t receivedChecksum = b;
      sendModified0DFrameToSwitch(receivedChecksum);
      aState = A_PASS_THROUGH;
      return;
    }
  }

  forwardByteAtoB(b);
}

void pumpAtoBOne() {
  if (Serial1.available() <= 0) return;

  uint8_t b = (uint8_t)Serial1.read();
  unsigned long now = micros();
  bool newFrame = (lastRxUsA == 0) || ((now - lastRxUsA) > NEW_FRAME_GAP_US);
  lastRxUsA = now;

  if (SUPPRESS_TX_ECHO && suppressA > 0) {
    suppressA--;
    return;
  }

  if (TREAT_MASTER_00_AS_BREAK && newFrame && b == 0x00) {
    sendLinBreakOnSerial2();
    return;
  }

  processMasterByte(b, newFrame);
}

void pumpBtoAOne() {
  if (Serial2.available() <= 0) return;

  uint8_t b = (uint8_t)Serial2.read();
  unsigned long now = micros();
  bool newFrame = (lastRxUsB == 0) || ((now - lastRxUsB) > NEW_FRAME_GAP_US);
  lastRxUsB = now;

  if (SUPPRESS_TX_ECHO && suppressB > 0) {
    suppressB--;
    return;
  }

  if (TREAT_MASTER_00_AS_BREAK && newFrame && b == 0x00) {
    sendLinBreakOnSerial1();
    return;
  }

  monitorSlaveByte(b, newFrame);
  forwardByteBtoA(b);
}

#if defined(ARDUINO_MEGA_2560)

void handlePcSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '1') {
      button9LedWanted = true;
      pendingImmediateLedRefresh = true;
      Serial.println(F("Forced button9LedWanted = ON"));
    } else if (c == '0') {
      button9LedWanted = false;
      pendingImmediateLedRefresh = true;
      Serial.println(F("Forced button9LedWanted = OFF"));
    } else if (c == 't') {
      button9LedWanted = !button9LedWanted;
      pendingImmediateLedRefresh = true;
      Serial.print(F("Toggled button9LedWanted = "));
      Serial.println(button9LedWanted ? F("ON") : F("OFF"));
    } else if (c == 'e') {
      useEnhancedChecksum = true;
      Serial.println(F("Checksum mode forced: enhanced"));
    } else if (c == 'c') {
      useEnhancedChecksum = false;
      Serial.println(F("Checksum mode forced: classic"));
    } else if (c == 'a') {
      rf433TurnOnChannel();
      Serial.println(F("RF channel 1 triggered"));
    } else if (c == 'b') {
      rf433TurnOffChannel();
      Serial.println(F("RF channel 2 triggered"));
    } else if (c == '?') {
      Serial.print(F("button9LedWanted = "));
      Serial.println(button9LedWanted ? F("ON") : F("OFF"));
      Serial.print(F("button9IsDown = "));
      Serial.println(button9IsDown ? F("YES") : F("NO"));
      Serial.print(F("pendingImmediateLedRefresh = "));
      Serial.println(pendingImmediateLedRefresh ? F("YES") : F("NO"));
      Serial.print(F("haveLastLedData = "));
      Serial.println(haveLastLedData ? F("YES") : F("NO"));
      Serial.print(F("checksum mode = "));
      Serial.println(useEnhancedChecksum ? F("enhanced") : F("classic"));
      Serial.print(F("lastMasterPidToSwitch = 0x"));
      if (lastMasterPidToSwitch < 16) Serial.print('0');
      Serial.println(lastMasterPidToSwitch, HEX);
    }
  }
}

#endif

void setup() {
  
  #if defined(ARDUINO_MEGA_2560)
  Serial.begin(PC_BAUD);
  Serial.println(F("LIN MITM fast press + immediate 0D LED refresh"));
  Serial.println(F("Serial commands: 1=force ON, 0=force OFF, t=toggle, e=enh, c=classic, a=RF ch1, b=RF ch2, ?=status"));
  #endif

  wakeLinTransceivers();

  Serial1.begin(LIN_BAUD, SERIAL_8N1);
  Serial2.begin(LIN_BAUD, SERIAL_8N1);

  rf433Begin();

  // Initiate the RF channel
  rf433TurnOnChannel();
  rf_channel_status = true;
}

void loop() {
  // Prioritize slave response bytes.
  pumpBtoAOne();
  pumpAtoBOne();

  for (uint8_t i = 0; i < 8; i++) {
    if (Serial2.available() > 0) pumpBtoAOne();
    if (Serial1.available() > 0) pumpAtoBOne();
  }

  updateButton9ReleaseTimeout();
  maybeSendImmediateLedRefresh();

  #if defined(ARDUINO_MEGA_2560)
  if (Serial.available() > 0) {
    handlePcSerial();
  }
  #endif

}
