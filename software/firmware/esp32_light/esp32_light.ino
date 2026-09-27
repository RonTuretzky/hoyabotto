// ESP32 light-sensor firmware for the farm paddle: one BH1750 over I2C, one JSON line per reading.
//
// Wiring (ESP-WROOM-32 devkit, verify the delivered board's labels first):
//   BH1750 VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22 (ADDR left floating = 0x23)
// Output at 115200 baud, ~5 Hz:
//   {"seq":812,"t_ms":93410,"lux":412.5,"ok":true}
//   {"seq":813,"t_ms":93610,"lux":null,"ok":false,"err":"i2c"}
// The Mac side (farm/adapters/esp32_serial.py) turns silence or a seq gap into STALE and ok=false into INVALID.
// Flash with Arduino IDE / arduino-cli (board: "ESP32 Dev Module"); no library beyond Wire is required.

#include <Wire.h>

static const uint8_t BH1750_ADDR = 0x23;
static const uint8_t CMD_POWER_ON = 0x01;
static const uint8_t CMD_RESET = 0x07;
static const uint8_t CMD_CONT_HIRES = 0x10;   // 1 lx resolution, 120 ms typ / 180 ms max conversion
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
static const unsigned long PERIOD_MS = 200;     // >= one full conversion, so every sample is a distinct reading

unsigned long seq = 0;
unsigned long nextAt = 0;
bool sensorReady = false;

bool bh1750Write(uint8_t cmd) {
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(cmd);
  return Wire.endTransmission() == 0;
}

bool bh1750Init() {
  if (!bh1750Write(CMD_POWER_ON)) return false;
  delay(10);
  if (!bh1750Write(CMD_RESET)) return false;
  delay(10);
  if (!bh1750Write(CMD_CONT_HIRES)) return false;
  delay(180);
  return true;
}

// Returns true and sets lux on success; false on any bus error.
bool bh1750Read(float &lux) {
  if (Wire.requestFrom((int)BH1750_ADDR, 2) != 2) return false;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  lux = raw / 1.2f;   // datasheet: lux = raw / 1.2 at default MTreg (69)
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  sensorReady = bh1750Init();
  nextAt = millis();
}

void loop() {
  unsigned long now = millis();
  if ((long)(now - nextAt) < 0) { delay(2); return; }
  nextAt += PERIOD_MS;
  seq++;
  float lux = 0;
  bool ok = sensorReady && bh1750Read(lux);
  if (!ok) {
    // try to bring the sensor back without blocking the stream for long
    sensorReady = bh1750Init();
    Serial.print("{\"seq\":"); Serial.print(seq);
    Serial.print(",\"t_ms\":"); Serial.print(now);
    Serial.print(",\"lux\":null,\"ok\":false,\"err\":\""); Serial.print(sensorReady ? "read" : "i2c"); Serial.println("\"}");
    return;
  }
  Serial.print("{\"seq\":"); Serial.print(seq);
  Serial.print(",\"t_ms\":"); Serial.print(now);
  Serial.print(",\"lux\":"); Serial.print(lux, 1);
  Serial.println(",\"ok\":true}");
}
