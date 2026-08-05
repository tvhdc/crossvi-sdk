#include "XteinkDetect.h"

#include <Arduino.h>
#include <BoardConfig.h>
#include <Wire.h>

#include <string.h>

namespace freeink {

#if !(FREEINK_DEVICE_X4 || FREEINK_DEVICE_X3)

// Neither Xteink profile is in this build, so there is nothing to fingerprint.
// Probing would also be unsafe here: SDA=20 / SCL=0 are only free pins on the
// Xteink C3 pinout — on an ESP32-S3, GPIO20 is native USB D+ and GPIO0 is the
// boot strap.
XteinkVerdict detectXteinkVerdict(uint8_t* score1, uint8_t* score2) {
  if (score1) *score1 = 0;
  if (score2) *score2 = 0;
  return XteinkVerdict::Inconclusive;
}
bool detectXteinkIsX3() { return false; }
X3DisplayVerdict detectX3DisplayController(uint8_t verBytes[5], uint8_t* flg) {
  if (verBytes) memset(verBytes, 0, 5);
  if (flg) *flg = 0;
  return X3DisplayVerdict::Uc8253Assumed;
}
bool selectXteinkDevice() { return false; }

#else

namespace {

// X3-only peripherals on the secondary I2C bus (SDA=20, SCL=0).
constexpr int X3_I2C_SDA = 20;
constexpr int X3_I2C_SCL = 0;
constexpr uint32_t X3_I2C_FREQ = 400000;

constexpr uint8_t ADDR_BQ27220 = 0x55;  // fuel gauge
constexpr uint8_t ADDR_DS3231 = 0x68;   // RTC
constexpr uint8_t ADDR_QMI8658 = 0x6B;  // IMU
constexpr uint8_t ADDR_QMI8658_ALT = 0x6A;

constexpr uint8_t BQ27220_SOC_REG = 0x2C;
constexpr uint8_t BQ27220_VOLT_REG = 0x08;
constexpr uint8_t DS3231_SEC_REG = 0x00;
constexpr uint8_t QMI8658_WHO_AM_I_REG = 0x00;
constexpr uint8_t QMI8658_WHO_AM_I_VALUE = 0x05;

bool readReg8(uint8_t addr, uint8_t reg, uint8_t* out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) < 1) return false;
  *out = Wire.read();
  return true;
}

bool readReg16LE(uint8_t addr, uint8_t reg, uint16_t* out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, static_cast<uint8_t>(2), static_cast<uint8_t>(true)) < 2) {
    while (Wire.available()) Wire.read();
    return false;
  }
  const uint8_t lo = Wire.read();
  const uint8_t hi = Wire.read();
  *out = (static_cast<uint16_t>(hi) << 8) | lo;
  return true;
}

// Each probe checks not just for an ACK but for a plausible value, so a stray
// pull-up or floating bus can't masquerade as a present chip.
bool probeBq27220() {
  uint16_t soc = 0;
  uint16_t mv = 0;
  if (!readReg16LE(ADDR_BQ27220, BQ27220_SOC_REG, &soc) || soc > 100) return false;
  if (!readReg16LE(ADDR_BQ27220, BQ27220_VOLT_REG, &mv)) return false;
  return mv >= 2500 && mv <= 5000;
}

bool probeDs3231() {
  uint8_t sec = 0;
  if (!readReg8(ADDR_DS3231, DS3231_SEC_REG, &sec)) return false;
  const uint8_t tens = (sec >> 4) & 0x07;
  const uint8_t ones = sec & 0x0F;
  return tens <= 5 && ones <= 9;  // valid BCD seconds
}

bool probeQmi8658() {
  uint8_t who = 0;
  if (readReg8(ADDR_QMI8658, QMI8658_WHO_AM_I_REG, &who) && who == QMI8658_WHO_AM_I_VALUE) return true;
  if (readReg8(ADDR_QMI8658_ALT, QMI8658_WHO_AM_I_REG, &who) && who == QMI8658_WHO_AM_I_VALUE) return true;
  return false;
}

uint8_t runProbePass() {
  Wire.begin(X3_I2C_SDA, X3_I2C_SCL, X3_I2C_FREQ);
  Wire.setTimeOut(6);
  const uint8_t score =
      static_cast<uint8_t>(probeBq27220()) + static_cast<uint8_t>(probeDs3231()) + static_cast<uint8_t>(probeQmi8658());
  Wire.end();
  pinMode(X3_I2C_SDA, INPUT);
  pinMode(X3_I2C_SCL, INPUT);
  return score;
}

}  // namespace

XteinkVerdict detectXteinkVerdict(uint8_t* score1, uint8_t* score2) {
  const uint8_t pass1 = runProbePass();
  delay(2);
  const uint8_t pass2 = runProbePass();
  if (score1) *score1 = pass1;
  if (score2) *score2 = pass2;
  // X3 confirmed only when both passes see at least two of the three chips; the
  // X4 sees zero, so a single stray ACK never flips the result. Anything in
  // between is Inconclusive: callers should run as X4 but may re-probe later.
  if (pass1 >= 2 && pass2 >= 2) return XteinkVerdict::X3Confirmed;
  if (pass1 == 0 && pass2 == 0) return XteinkVerdict::X4Confirmed;
  return XteinkVerdict::Inconclusive;
}

bool detectXteinkIsX3() { return detectXteinkVerdict() == XteinkVerdict::X3Confirmed; }

#if FREEINK_DEVICE_X3

namespace {

// X3 display pins (shared by both X3 controller variants; see XTEINK_X3).
constexpr int8_t EPD_SCLK = 8;
constexpr int8_t EPD_MOSI = 10;  // the controller's bidirectional SDA
constexpr int8_t EPD_CS = 21;
constexpr int8_t EPD_DC = 4;
constexpr int8_t EPD_RST = 5;
constexpr int8_t EPD_BUSY = 6;

// UC8279d read-capable registers (UC8279d_B 0.1 datasheet).
constexpr uint8_t UC8279_CMD_VER = 0x70;  // reserved, CHIP_VER, LUT_VER[23:0]
constexpr uint8_t UC8279_CMD_FLG = 0x71;  // status; BUSY_N (D0) = 1 when idle
constexpr uint8_t UC8279_CMD_RMTP = 0xA2;  // dummy byte, then MTP[0..n]

inline void epdClockDelay() { delayMicroseconds(1); }  // ~500 kHz, timing-safe

void epdWriteByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(EPD_MOSI, (b & 0x80) ? HIGH : LOW);
    epdClockDelay();
    digitalWrite(EPD_SCLK, HIGH);
    epdClockDelay();
    digitalWrite(EPD_SCLK, LOW);
    b <<= 1;
  }
}

uint8_t epdReadByte() {
  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    // The controller shifts the next bit out on the SCL falling edge; sample
    // while the clock is low, then pulse.
    epdClockDelay();
    b = static_cast<uint8_t>((b << 1) | (digitalRead(EPD_MOSI) == HIGH ? 1 : 0));
    digitalWrite(EPD_SCLK, HIGH);
    epdClockDelay();
    digitalWrite(EPD_SCLK, LOW);
  }
  return b;
}

// One command + N-byte half-duplex read: command with DC low, then SDA (our
// MOSI) released to input with DC high while the controller drives the reads.
void epdCmdRead(uint8_t cmd, uint8_t* out, uint8_t len) {
  pinMode(EPD_MOSI, OUTPUT);
  digitalWrite(EPD_DC, LOW);
  digitalWrite(EPD_CS, LOW);
  epdClockDelay();
  epdWriteByte(cmd);
  digitalWrite(EPD_DC, HIGH);
  pinMode(EPD_MOSI, INPUT_PULLUP);
  epdClockDelay();
  for (uint8_t i = 0; i < len; i++) out[i] = epdReadByte();
  digitalWrite(EPD_CS, HIGH);
  pinMode(EPD_MOSI, OUTPUT);
}

// Match on a real FLG status plus a non-uniform VER response. Some shipping
// UC8279 units report CHIP_VER=0x00, so pinning the second byte to a particular
// value rejects valid new X3 panels. A controller that does not answer leaves
// the half-duplex line at a uniform all-low or all-high pattern instead.
bool verIsFloating(const uint8_t ver[5]) {
  for (int i = 1; i < 5; i++) {
    if (ver[i] != ver[0]) return false;
  }
  return true;
}

bool flgIsDriven(uint8_t flg) {
  if (flg == 0x00 || flg == 0xFF) return false;
  return (flg & 0x01) == 0x01;
}

bool matchUc8279(const uint8_t ver[5], uint8_t flg) { return flgIsDriven(flg) && !verIsFloating(ver); }

bool runDisplayProbePass(uint8_t ver[5], uint8_t* flg, uint8_t rstLowMs) {
  pinMode(EPD_CS, OUTPUT);
  digitalWrite(EPD_CS, HIGH);
  pinMode(EPD_SCLK, OUTPUT);
  digitalWrite(EPD_SCLK, LOW);
  pinMode(EPD_DC, OUTPUT);
  digitalWrite(EPD_DC, LOW);
  pinMode(EPD_MOSI, OUTPUT);
  pinMode(EPD_BUSY, INPUT);

  // Screen with a short reset first. Only a potential UC8279 match pays for a
  // second pass using the vendor identification timing (RST low for 50 ms).
  // The panel driver resets the controller again during begin().
  pinMode(EPD_RST, OUTPUT);
  digitalWrite(EPD_RST, HIGH);
  delay(2);
  digitalWrite(EPD_RST, LOW);
  delay(rstLowMs);
  digitalWrite(EPD_RST, HIGH);
  delay(30);

  uint8_t flgByte = 0;
  epdCmdRead(UC8279_CMD_FLG, &flgByte, 1);
  epdCmdRead(UC8279_CMD_VER, ver, 5);
  if (flg) *flg = flgByte;
  return matchUc8279(ver, flgByte);
}

void releaseDisplayPins() {
  // Same convention as the I2C probe: leave everything released. RST_N has an
  // internal pull-up, so INPUT keeps the controller out of reset.
  pinMode(EPD_SCLK, INPUT);
  pinMode(EPD_MOSI, INPUT);
  pinMode(EPD_CS, INPUT_PULLUP);  // don't leave the panel selected
  pinMode(EPD_DC, INPUT);
  pinMode(EPD_RST, INPUT);
}

}  // namespace

X3DisplayVerdict detectX3DisplayController(uint8_t verBytes[5], uint8_t* flg) {
  uint8_t ver1[5] = {0};
  uint8_t ver2[5] = {0};
  uint8_t flg1 = 0;
  bool pass1 = runDisplayProbePass(ver1, &flg1, /*rstLowMs=*/1);
  if (!pass1) {
    // Some new X3 modules only answer reliably after the vendor's 50 ms
    // identification reset. The established UC8253 path pays this once during
    // boot, then remains on its existing driver when the bus still floats.
    delay(2);
    pass1 = runDisplayProbePass(ver1, &flg1, /*rstLowMs=*/50);
  }
  delay(2);
  const bool pass2 = runDisplayProbePass(ver2, nullptr, /*rstLowMs=*/pass1 ? 50 : 1);

  const bool verAgree = memcmp(ver1, ver2, 5) == 0;
  bool confirmed = pass1 && pass2 && verAgree;
  if (!confirmed && flgIsDriven(flg1) && verAgree && verIsFloating(ver1) && ver1[0] == 0xFF) {
    // Field units can report an unreadable all-FF VER even though a real
    // UC8279 is present. RMTP returns one dummy byte followed by the programmed
    // MTP refresh key 0xA5. UC8253 does not implement RMTP, so its released bus
    // cannot satisfy this positive check.
    uint8_t mtp[49] = {0};
    epdCmdRead(UC8279_CMD_RMTP, mtp, sizeof(mtp));
    confirmed = mtp[1] == 0xA5;
  }
  releaseDisplayPins();
  if (verBytes) memcpy(verBytes, pass1 && pass2 ? ver2 : ver1, 5);
  if (flg) *flg = flg1;
  // Confirmed only when both passes match the UC8279 signature AND agree on
  // the VER bytes — a floating bus can't produce the same stable non-trivial
  // pattern twice. Disagreement is Inconclusive (resolve as UC8253, the
  // shipping controller, but don't persist so a flaky boot re-probes).
  if (confirmed) return X3DisplayVerdict::Uc8279Confirmed;
  if (!pass1 && !pass2) return X3DisplayVerdict::Uc8253Assumed;
  return X3DisplayVerdict::Inconclusive;
}

#else  // X4-only build: no X3 profile, nothing to probe.

X3DisplayVerdict detectX3DisplayController(uint8_t verBytes[5], uint8_t* flg) {
  if (verBytes) memset(verBytes, 0, 5);
  if (flg) *flg = 0;
  return X3DisplayVerdict::Uc8253Assumed;
}

#endif  // FREEINK_DEVICE_X3

bool selectXteinkDevice() {
  const bool isX3 = detectXteinkIsX3();
  if (!isX3) {
    BoardConfig::selectDevice(BoardConfig::Board::XteinkX4);
    return false;
  }
  // X3 confirmed: fingerprint which panel controller this production run
  // carries and select the matching sibling profile.
  const bool isUc8279 = detectX3DisplayController() == X3DisplayVerdict::Uc8279Confirmed;
  BoardConfig::selectDevice(isUc8279 ? BoardConfig::Board::XteinkX3Uc8279 : BoardConfig::Board::XteinkX3);
  return true;
}

#endif  // FREEINK_DEVICE_X4 || FREEINK_DEVICE_X3

}  // namespace freeink
