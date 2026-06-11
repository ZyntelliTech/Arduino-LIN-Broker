// ============================================================
//  LIN bidirectional bridge — two LINTTL3 on Arduino Mega 2560
// ------------------------------------------------------------
//  Bus A: Serial1 — TX→18 RX→19 SLP→2
//  Bus B: Serial2 — TX→16 RX→17 SLP→3
//  PC log: Serial (USB) @ 115200
//
//  Valid frame on Bus A → forwarded to Bus B
//  Valid frame on Bus B → forwarded to Bus A
// ============================================================

enum LinBus : uint8_t { BUS_A = 0, BUS_B = 1 };

// -------------------- Config -------------------------
const uint8_t       PIN_LIN1_SLP   = 2;
const uint8_t       PIN_LIN1_TX    = 18;
const uint8_t       PIN_LIN2_SLP   = 3;
const uint8_t       PIN_LIN2_TX    = 16;
const unsigned long LIN_BAUD       = 19200;
const unsigned long PC_BAUD        = 115200;
const unsigned long FRAME_GAP_US   = 3000;
const uint8_t       MAX_FRAME      = 64;
const unsigned long SLP_WAKE_MS    = 1;
const unsigned long LIN_BREAK_US       = 1300;
const unsigned long LIN_BREAK_DELIM_US = 100;
const unsigned long LIN_ECHO_SUPPRESS_MS = 30;

// -------------------- Per-bus RX state ---------------
struct LinRxState {
  uint8_t  buf[MAX_FRAME];
  uint8_t  len;
  unsigned long lastByteUs;
  unsigned long suppressRxUntilMs;
};

LinRxState linRx[2];

// -------------------- Helpers ------------------------
HardwareSerial& linUart(LinBus bus) {
  return (bus == BUS_A) ? Serial1 : Serial2;
}

uint8_t linTxPin(LinBus bus) {
  return (bus == BUS_A) ? PIN_LIN1_TX : PIN_LIN2_TX;
}

uint8_t linSlpPin(LinBus bus) {
  return (bus == BUS_A) ? PIN_LIN1_SLP : PIN_LIN2_SLP;
}

const __FlashStringHelper* linBusName(LinBus bus) {
  return (bus == BUS_A) ? F("A (Serial1)") : F("B (Serial2)");
}

void printHex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void linTransceiversWake() {
  pinMode(PIN_LIN1_SLP, OUTPUT);
  pinMode(PIN_LIN2_SLP, OUTPUT);
  digitalWrite(PIN_LIN1_SLP, HIGH);
  digitalWrite(PIN_LIN2_SLP, HIGH);
  delay(SLP_WAKE_MS);
}

void linNoteBusTx(LinBus bus) {
  linRx[bus].suppressRxUntilMs = millis() + LIN_ECHO_SUPPRESS_MS;
}

bool linBusRxSuppressed(LinBus bus) {
  return millis() < linRx[bus].suppressRxUntilMs;
}

// -------------------- Checksum / PID ---------------
uint8_t linChecksumSum(uint16_t seed, const uint8_t* data, uint8_t len) {
  uint16_t sum = seed;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
    if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  }
  return (uint8_t)(~sum);
}

uint8_t linChecksumEnhanced(uint8_t pid, const uint8_t* data, uint8_t len) {
  return linChecksumSum(pid, data, len);
}

uint8_t linChecksumClassic(uint8_t pid, const uint8_t* data, uint8_t len) {
  (void)pid;
  return linChecksumSum(0, data, len);
}

uint8_t linBuildPid(uint8_t id) {
  id &= 0x3F;
  uint8_t b0 = id & 1;
  uint8_t b1 = (id >> 1) & 1;
  uint8_t b2 = (id >> 2) & 1;
  uint8_t b3 = (id >> 3) & 1;
  uint8_t b4 = (id >> 4) & 1;
  uint8_t b5 = (id >> 5) & 1;
  uint8_t p0 = (uint8_t)(b0 ^ b1 ^ b2 ^ b4);
  uint8_t p1 = (uint8_t)(~(b1 ^ b3 ^ b4 ^ b5) & 1);
  return (uint8_t)(id | (p0 << 6) | (p1 << 7));
}

bool pidParityOk(uint8_t pid) {
  return linBuildPid(pid & 0x3F) == pid;
}

// -------------------- Master TX on either bus --------
void linBusFlushRx(LinBus bus) {
  HardwareSerial& uart = linUart(bus);
  while (uart.available() > 0) {
    (void)uart.read();
  }
}

void linBusSendBreak(LinBus bus) {
  HardwareSerial& uart = linUart(bus);
  uint8_t txPin = linTxPin(bus);

  uart.flush();
  uart.end();
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);
  delayMicroseconds(LIN_BREAK_US);
  digitalWrite(txPin, HIGH);
  delayMicroseconds(LIN_BREAK_DELIM_US);
  uart.begin(LIN_BAUD, SERIAL_8N1);
  linBusFlushRx(bus);
}

bool linBusSendHeader(LinBus bus, uint8_t id) {
  HardwareSerial& uart = linUart(bus);
  uint8_t pid = linBuildPid(id);

  linBusSendBreak(bus);
  uart.write(0x55);
  uart.write(pid);
  uart.flush();
  linNoteBusTx(bus);
  return true;
}

bool linBusMasterSend(LinBus bus, uint8_t id, const uint8_t* data, uint8_t dataLen,
                      bool enhanced) {
  if (dataLen > 8) return false;

  HardwareSerial& uart = linUart(bus);
  uint8_t pid = linBuildPid(id);

  linBusSendBreak(bus);
  uart.write(0x55);
  uart.write(pid);
  for (uint8_t i = 0; i < dataLen; i++) {
    uart.write(data[i]);
  }
  uart.write(enhanced ? linChecksumEnhanced(pid, data, dataLen)
                      : linChecksumClassic(pid, data, dataLen));
  uart.flush();
  linNoteBusTx(bus);
  return true;
}

bool linForwardFrame(LinBus dest, uint8_t linId, const uint8_t* payload,
                     uint8_t payloadLen, uint8_t pid, uint8_t chk) {
  if (payloadLen > 8) return false;

  uint8_t data[8];
  for (uint8_t i = 0; i < payloadLen; i++) {
    data[i] = payload[i];
  }

  bool classicOk  = (chk == linChecksumClassic(pid, payload, payloadLen));
  bool enhancedOk = (chk == linChecksumEnhanced(pid, payload, payloadLen));

  if (!classicOk && !enhancedOk) {
    return false;
  }

  bool useEnhanced = enhancedOk && !classicOk;
  return linBusMasterSend(dest, linId, data, payloadLen, useEnhanced);
}

// -------------------- Parse / flush ------------------
// Accepts:
//   00 55 PID              header only (master request, no data on wire)
//   55 PID                 header only
//   00 55 PID [data] CHK   full frame with optional break placeholder
//   55 PID [data] CHK      full frame
bool parseLinFrame(const uint8_t* buf, uint8_t len,
                   uint8_t& pid, const uint8_t*& payload,
                   uint8_t& payloadLen, uint8_t& checksum,
                   bool& headerOnly) {
  headerOnly = false;
  payload    = nullptr;
  payloadLen = 0;
  checksum   = 0;

  if (len < 2) return false;

  uint8_t pidIndex;
  if (buf[0] == 0x00 && buf[1] == 0x55) {
    if (len < 3) return false;
    pidIndex = 2;
  } else if (buf[0] == 0x55) {
    pidIndex = 1;
  } else {
    return false;
  }

  pid = buf[pidIndex];

  // Header only: ends right after PID (e.g. 00 55 C1)
  if (len == pidIndex + 1) {
    headerOnly = true;
    return true;
  }

  // Full frame: need at least one checksum byte after PID
  if (len < pidIndex + 2) return false;

  checksum   = buf[len - 1];
  payload    = &buf[pidIndex + 1];
  payloadLen = (uint8_t)(len - (pidIndex + 2));
  return true;
}

void flushFrame(LinBus rxBus, LinBus txBus) {
  LinRxState& rx = linRx[rxBus];
  if (rx.len == 0) return;

  if (linBusRxSuppressed(rxBus)) {
    rx.len = 0;
    return;
  }

  Serial.print(F("--- Bus "));
  Serial.print(linBusName(rxBus));
  Serial.println(F(" RX ---"));
  Serial.print(F("RAW: "));
  for (uint8_t i = 0; i < rx.len; i++) {
    printHex2(rx.buf[i]);
    Serial.print(' ');
  }
  Serial.println();

  uint8_t pid = 0, chk = 0, payloadLen = 0;
  const uint8_t* payload = nullptr;
  bool headerOnly = false;

  if (!parseLinFrame(rx.buf, rx.len, pid, payload, payloadLen, chk, headerOnly)) {
    Serial.println(F("Parse: not a recognized LIN frame"));
    Serial.println();
    rx.len = 0;
    return;
  }

  uint8_t linId = pid & 0x3F;
  Serial.print(F("ID:  0x"));
  printHex2(linId);
  Serial.print(F("  PID: 0x"));
  printHex2(pid);
  Serial.print(F("  parity: "));
  Serial.println(pidParityOk(pid) ? F("OK") : F("FAIL"));

  bool forwarded = false;

  if (headerOnly) {
    Serial.println(F("Type: header only (master request)"));
    if (pidParityOk(pid) && linBusSendHeader(txBus, linId)) {
      forwarded = true;
    }
  } else {
    Serial.print(F("DATA ("));
    Serial.print(payloadLen);
    Serial.print(F("): "));
    for (uint8_t i = 0; i < payloadLen; i++) {
      printHex2(payload[i]);
      Serial.print(' ');
    }
    Serial.println();

    uint8_t chkClassic  = linChecksumClassic(pid, payload, payloadLen);
    uint8_t chkEnhanced = linChecksumEnhanced(pid, payload, payloadLen);

    Serial.print(F("CHK: 0x"));
    printHex2(chk);
    Serial.println();

    bool chkOk = (chk == chkClassic) || (chk == chkEnhanced);
    if (chk == chkClassic) {
      Serial.println(F("Checksum: OK (classic)"));
    } else if (chk == chkEnhanced) {
      Serial.println(F("Checksum: OK (enhanced)"));
    } else {
      Serial.print(F("Checksum: FAIL  classic=0x"));
      printHex2(chkClassic);
      Serial.print(F("  enhanced=0x"));
      printHex2(chkEnhanced);
      Serial.println();
    }

    if (chkOk) {
      forwarded = linForwardFrame(txBus, linId, payload, payloadLen, pid, chk);
    }
  }

  if (forwarded) {
    Serial.print(F("--- Bus "));
    Serial.print(linBusName(txBus));
    Serial.println(F(" TX ---"));
    Serial.print(F("ID:  0x"));
    printHex2(linId);
    if (headerOnly) {
      Serial.println(F("  (header only)"));
    } else {
      Serial.print(F("  DATA ("));
      Serial.print(payloadLen);
      Serial.print(F("): "));
      for (uint8_t i = 0; i < payloadLen; i++) {
        printHex2(payload[i]);
        Serial.print(' ');
      }
      Serial.println();
    }
  } else if (!headerOnly || pidParityOk(pid)) {
    Serial.print(F("Forward to Bus "));
    Serial.print(linBusName(txBus));
    Serial.println(F(" failed"));
  }
  Serial.println();

  rx.len = 0;
}

void pollBusRx(LinBus bus, LinBus forwardTo) {
  LinRxState& rx = linRx[bus];
  HardwareSerial& uart = linUart(bus);

  while (uart.available() > 0) {
    uint8_t b = (uint8_t)uart.read();
    if (rx.len < MAX_FRAME) {
      rx.buf[rx.len++] = b;
    } else {
      flushFrame(bus, forwardTo);
      rx.buf[rx.len++] = b;
    }
    rx.lastByteUs = micros();
  }

  if (rx.len > 0 && (micros() - rx.lastByteUs) > FRAME_GAP_US) {
    flushFrame(bus, forwardTo);
  }
}

// -------------------- Arduino ----------------------
void setup() {
  Serial.begin(PC_BAUD);
  linTransceiversWake();
  Serial1.begin(LIN_BAUD, SERIAL_8N1);
  Serial2.begin(LIN_BAUD, SERIAL_8N1);

  Serial.println(F("LIN bidirectional bridge (Mega + 2x LINTTL3)"));
  Serial.println(F("Bus A <-> Bus B  (Serial1 <-> Serial2)"));
  Serial.println(F("Bus A: RX=19 TX=18 SLP=2"));
  Serial.println(F("Bus B: RX=17 TX=16 SLP=3"));
  Serial.print(F("LIN baud: "));
  Serial.println(LIN_BAUD);
  Serial.println();
}

void loop() {
  pollBusRx(BUS_A, BUS_B);  // Serial1 RX → forward Serial2
  pollBusRx(BUS_B, BUS_A);  // Serial2 RX → forward Serial1
}
