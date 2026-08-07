/*
  SCADA EV: SCT-013 + ADS1263 + ESP32 + MQTT

  Analog input:
    ADS1263 IN1 positive input
    ADS1263 IN0 negative input

  ADS1263 SPI:
    CS=27, SCLK=18, MISO/DOUT=19, MOSI/DIN=23, DRDY=25, RESET=26

  Current sensor defaults:
    SCT-013-000, 100 A / 50 mA, external burden 22 ohm

  MQTT is standard MQTT 3.1.1 over TCP or TLS. No vendor API is used.
*/

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <math.h>
#include <time.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#warning "Using secrets.example.h; create secrets.h before deployment"
#endif

namespace Pins {
constexpr uint8_t CS = 27;
constexpr uint8_t SCLK = 18;
constexpr uint8_t MISO = 19;
constexpr uint8_t MOSI = 23;
constexpr uint8_t DRDY = 25;
constexpr uint8_t RESET = 26;
}  // namespace Pins

namespace Ads {
constexpr uint32_t SPI_HZ = 1000000;
constexpr uint8_t EXPECTED_ID = 1;

constexpr uint8_t REG_ID = 0x00;
constexpr uint8_t REG_POWER = 0x01;
constexpr uint8_t REG_INTERFACE = 0x02;
constexpr uint8_t REG_MODE0 = 0x03;
constexpr uint8_t REG_MODE1 = 0x04;
constexpr uint8_t REG_MODE2 = 0x05;
constexpr uint8_t REG_INPMUX = 0x06;
constexpr uint8_t REG_REFMUX = 0x0F;

constexpr uint8_t CMD_START1 = 0x08;
constexpr uint8_t CMD_STOP1 = 0x0A;
constexpr uint8_t CMD_RDATA1 = 0x12;
constexpr uint8_t CMD_RREG = 0x20;
constexpr uint8_t CMD_WREG = 0x40;

constexpr uint8_t POWER_INTREF_ON = 0x11;
constexpr uint8_t INTERFACE_STATUS_CHECKSUM = 0x05;
constexpr uint8_t MODE0_DELAY_35_US = 0x03;
constexpr uint8_t MODE1_FIR = 0x84;
constexpr uint8_t MODE2_PGA_BYPASS_2400_SPS = 0x8A;
constexpr uint8_t MODE2_PGA_GAIN1_2400_SPS = 0x0A;
constexpr uint8_t REFMUX_INTERNAL_2V5 = 0x00;
constexpr uint8_t REFMUX_AVDD_AVSS = 0x24;
constexpr uint8_t INPMUX_IN1_IN0 = 0x10;
constexpr uint8_t INPMUX_ANALOG_SUPPLY_MONITOR = 0xCC;
}  // namespace Ads

constexpr float NOMINAL_ANALOG_SUPPLY_V = 5.000f;
constexpr float INTERNAL_REFERENCE_V = 2.500f;

// IN1-IN0 calibration obtained with the SDG1032X after correcting polarity.
// Source: Sine 60 Hz, Low=1.500 V, High=3.000 V, Phase=0.
constexpr float INPUT_CAL_GAIN = 0.9412289f;
constexpr float INPUT_CAL_OFFSET_V = -0.0512069f;

// Keep true for the local SDG1032X calibration test. In this mode WiFi/MQTT
// are disabled and no current value is published as if it were production.
constexpr bool CALIBRATION_MODE = true;
constexpr float CALIBRATION_LOW_V = 1.500f;
constexpr float CALIBRATION_HIGH_V = 3.000f;
constexpr float CALIBRATION_FREQUENCY_HZ = 60.0f;
constexpr uint32_t CALIBRATION_REPORT_INTERVAL_MS = 1000;

constexpr float SCT_PRIMARY_A = 100.0f;
constexpr float SCT_SECONDARY_A = 0.050f;
constexpr float BURDEN_OHM = 22.0f;
constexpr float CURRENT_CAL_GAIN = 1.0f;
constexpr float CHARGING_THRESHOLD_A = 1.0f;

constexpr size_t WINDOW_SAMPLES = 1200;
constexpr uint32_t DRDY_TIMEOUT_US = 100000;
constexpr float MIN_FREQUENCY_RMS_V = 0.005f;
constexpr uint32_t PUBLISH_INTERVAL_MS = 5000;
constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr uint32_t MQTT_RETRY_MS = 5000;
constexpr uint16_t MQTT_KEEPALIVE_S = 60;
constexpr size_t MQTT_PAYLOAD_SIZE = 384;
constexpr size_t MQTT_QUEUE_SIZE = 16;

SPISettings adsSpi(Ads::SPI_HZ, MSBFIRST, SPI_MODE1);
float sampleBuffer[WINDOW_SAMPLES];
float adcFullScaleV = NOMINAL_ANALOG_SUPPLY_V;

WiFiClient plainClient;
WiFiClientSecure secureClient;
Client* mqttSocket = nullptr;

uint32_t crcErrors = 0;
uint32_t statusErrors = 0;
uint32_t drdyTimeouts = 0;
uint32_t sequenceNumber = 0;
uint32_t droppedMessages = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastMqttAttemptMs = 0;
uint32_t lastMqttActivityMs = 0;
uint32_t lastPublishMs = 0;
uint16_t mqttPacketId = 0;

struct Metrics {
  float meanV;
  float rmsAcV;
  float minV;
  float maxV;
  float vppV;
  float frequencyHz;
  float sampleRate;
};

struct PendingMessage {
  char payload[MQTT_PAYLOAD_SIZE];
};

PendingMessage mqttQueue[MQTT_QUEUE_SIZE];
size_t queueHead = 0;
size_t queueCount = 0;

void selectAdc() {
  SPI.beginTransaction(adsSpi);
  digitalWrite(Pins::CS, LOW);
}

void deselectAdc() {
  digitalWrite(Pins::CS, HIGH);
  SPI.endTransaction();
}

void writeCommand(uint8_t command) {
  selectAdc();
  SPI.transfer(command);
  deselectAdc();
}

void writeRegister(uint8_t reg, uint8_t value) {
  selectAdc();
  SPI.transfer(Ads::CMD_WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(value);
  deselectAdc();
}

uint8_t readRegister(uint8_t reg) {
  selectAdc();
  SPI.transfer(Ads::CMD_RREG | reg);
  SPI.transfer(0x00);
  const uint8_t value = SPI.transfer(0x00);
  deselectAdc();
  return value;
}

void hardwareReset() {
  digitalWrite(Pins::RESET, HIGH);
  delay(300);
  digitalWrite(Pins::RESET, LOW);
  delay(300);
  digitalWrite(Pins::RESET, HIGH);
  delay(300);
}

bool waitDrdy(uint32_t timeoutUs) {
  const uint32_t started = micros();
  while (digitalRead(Pins::DRDY) != LOW) {
    if (static_cast<uint32_t>(micros() - started) >= timeoutUs) {
      ++drdyTimeouts;
      return false;
    }
    yield();
  }
  return true;
}

bool verifyRegister(const char* name, uint8_t reg, uint8_t expected) {
  const uint8_t actual = readRegister(reg);
  Serial.printf("[REG] %-9s 0x%02X %s\n", name, actual,
                actual == expected ? "OK" : "ERROR");
  return actual == expected;
}

bool readRawSample(int32_t& raw) {
  if (!waitDrdy(DRDY_TIMEOUT_US)) {
    return false;
  }

  uint8_t data[4];
  selectAdc();
  SPI.transfer(Ads::CMD_RDATA1);
  const uint8_t status = SPI.transfer(0x00);
  for (uint8_t& byte : data) {
    byte = SPI.transfer(0x00);
  }
  const uint8_t checksumReceived = SPI.transfer(0x00);
  deselectAdc();

  if ((status & 0x40) == 0) {
    ++statusErrors;
    return false;
  }

  uint8_t checksumCalculated = 0x9B;
  for (const uint8_t byte : data) {
    checksumCalculated = static_cast<uint8_t>(checksumCalculated + byte);
  }
  if (checksumCalculated != checksumReceived) {
    ++crcErrors;
    return false;
  }

  const uint32_t bits = (static_cast<uint32_t>(data[0]) << 24) |
                        (static_cast<uint32_t>(data[1]) << 16) |
                        (static_cast<uint32_t>(data[2]) << 8) |
                        static_cast<uint32_t>(data[3]);
  raw = static_cast<int32_t>(bits);
  return true;
}

float rawToVolts(int32_t raw, float referenceV) {
  return static_cast<float>(static_cast<double>(raw) * referenceV /
                            2147483648.0);
}

float calibrateInputVoltage(float measuredV) {
  return measuredV * INPUT_CAL_GAIN + INPUT_CAL_OFFSET_V;
}

bool selectConversion(uint8_t mode2, uint8_t refmux, uint8_t inpmux) {
  writeCommand(Ads::CMD_STOP1);
  writeRegister(Ads::REG_MODE2, mode2);
  writeRegister(Ads::REG_REFMUX, refmux);
  writeRegister(Ads::REG_INPMUX, inpmux);
  writeCommand(Ads::CMD_START1);
  delay(20);

  return readRegister(Ads::REG_MODE2) == mode2 &&
         readRegister(Ads::REG_REFMUX) == refmux &&
         readRegister(Ads::REG_INPMUX) == inpmux;
}

bool configureAds1263() {
  hardwareReset();
  const uint8_t idRegister = readRegister(Ads::REG_ID);
  const uint8_t chipId = idRegister >> 5;
  Serial.printf("[ADC] REG_ID=0x%02X, chip ID=%u\n", idRegister, chipId);
  if (chipId != Ads::EXPECTED_ID) {
    return false;
  }

  writeCommand(Ads::CMD_STOP1);
  writeRegister(Ads::REG_POWER, Ads::POWER_INTREF_ON);
  writeRegister(Ads::REG_INTERFACE, Ads::INTERFACE_STATUS_CHECKSUM);
  writeRegister(Ads::REG_MODE0, Ads::MODE0_DELAY_35_US);
  writeRegister(Ads::REG_MODE1, Ads::MODE1_FIR);
  writeRegister(Ads::REG_MODE2, Ads::MODE2_PGA_BYPASS_2400_SPS);
  writeRegister(Ads::REG_REFMUX, Ads::REFMUX_AVDD_AVSS);
  writeRegister(Ads::REG_INPMUX, Ads::INPMUX_IN1_IN0);

  bool ok = true;
  ok &= verifyRegister("POWER", Ads::REG_POWER, Ads::POWER_INTREF_ON);
  ok &= verifyRegister("INTERFACE", Ads::REG_INTERFACE,
                       Ads::INTERFACE_STATUS_CHECKSUM);
  ok &= verifyRegister("MODE0", Ads::REG_MODE0, Ads::MODE0_DELAY_35_US);
  ok &= verifyRegister("MODE1", Ads::REG_MODE1, Ads::MODE1_FIR);
  ok &= verifyRegister("MODE2", Ads::REG_MODE2,
                       Ads::MODE2_PGA_BYPASS_2400_SPS);
  ok &= verifyRegister("REFMUX", Ads::REG_REFMUX, Ads::REFMUX_AVDD_AVSS);
  ok &= verifyRegister("INPMUX", Ads::REG_INPMUX, Ads::INPMUX_IN1_IN0);
  if (!ok) {
    return false;
  }

  writeCommand(Ads::CMD_START1);
  delay(20);
  return true;
}

bool measureAnalogSupply(float& supplyV) {
  if (!selectConversion(Ads::MODE2_PGA_GAIN1_2400_SPS,
                        Ads::REFMUX_INTERNAL_2V5,
                        Ads::INPMUX_ANALOG_SUPPLY_MONITOR)) {
    return false;
  }

  constexpr size_t SUPPLY_SAMPLES = 64;
  double sumMonitorV = 0.0;
  bool ok = true;
  for (size_t i = 0; i < SUPPLY_SAMPLES; ++i) {
    int32_t raw;
    if (!readRawSample(raw)) {
      ok = false;
      break;
    }
    sumMonitorV += rawToVolts(raw, INTERNAL_REFERENCE_V);
  }

  if (ok) {
    supplyV = static_cast<float>(4.0 * sumMonitorV / SUPPLY_SAMPLES);
    ok = supplyV > 4.0f && supplyV < 5.5f;
  }

  const bool restored =
      selectConversion(Ads::MODE2_PGA_BYPASS_2400_SPS,
                       Ads::REFMUX_AVDD_AVSS, Ads::INPMUX_IN1_IN0);
  return ok && restored;
}

Metrics calculateMetrics(const float* samples, size_t count,
                         uint32_t elapsedUs) {
  Metrics result{};
  result.minV = samples[0];
  result.maxV = samples[0];

  double sum = 0.0;
  for (size_t i = 0; i < count; ++i) {
    sum += samples[i];
    result.minV = min(result.minV, samples[i]);
    result.maxV = max(result.maxV, samples[i]);
  }
  result.meanV = static_cast<float>(sum / count);

  double sumSquares = 0.0;
  for (size_t i = 0; i < count; ++i) {
    const double ac = samples[i] - result.meanV;
    sumSquares += ac * ac;
  }
  result.rmsAcV = static_cast<float>(sqrt(sumSquares / count));
  result.vppV = result.maxV - result.minV;
  result.sampleRate = elapsedUs > 0
                          ? static_cast<float>(count - 1) * 1000000.0f /
                                static_cast<float>(elapsedUs)
                          : 0.0f;

  result.frequencyHz = 0.0f;
  if (result.rmsAcV >= MIN_FREQUENCY_RMS_V && result.sampleRate > 0.0f) {
    const float hysteresis = max(0.002f, result.rmsAcV * 0.20f);
    bool armed = false;
    size_t firstCrossing = 0;
    size_t lastCrossing = 0;
    uint16_t crossings = 0;

    for (size_t i = 0; i < count; ++i) {
      const float ac = samples[i] - result.meanV;
      if (ac <= -hysteresis) {
        armed = true;
      } else if (armed && ac >= hysteresis) {
        if (crossings == 0) {
          firstCrossing = i;
        }
        lastCrossing = i;
        ++crossings;
        armed = false;
      }
    }

    if (crossings >= 2 && lastCrossing > firstCrossing) {
      result.frequencyHz = static_cast<float>(crossings - 1) *
                           result.sampleRate /
                           static_cast<float>(lastCrossing - firstCrossing);
    }
  }
  return result;
}

bool runMathSelfTest() {
  constexpr size_t TEST_COUNT = 240;
  constexpr float TEST_SAMPLE_RATE = 2400.0f;
  constexpr float TEST_FREQUENCY = 60.0f;
  constexpr float TEST_OFFSET = 2.5f;
  constexpr float TEST_PEAK = 0.5f;
  float testSamples[TEST_COUNT];

  for (size_t i = 0; i < TEST_COUNT; ++i) {
    testSamples[i] =
        TEST_OFFSET + TEST_PEAK *
                          sinf(2.0f * PI * TEST_FREQUENCY * i /
                               TEST_SAMPLE_RATE);
  }
  const uint32_t elapsedUs = static_cast<uint32_t>(
      (TEST_COUNT - 1) * 1000000.0f / TEST_SAMPLE_RATE);
  const Metrics test = calculateMetrics(testSamples, TEST_COUNT, elapsedUs);
  const float expectedRms = TEST_PEAK / sqrtf(2.0f);
  const bool ok = fabsf(test.meanV - TEST_OFFSET) < 0.001f &&
                  fabsf(test.rmsAcV - expectedRms) < 0.001f &&
                  fabsf(test.frequencyHz - TEST_FREQUENCY) < 0.5f;
  Serial.printf("[SELFTEST] RMS=%.4f f=%.2f Hz -> %s\n", test.rmsAcV,
                test.frequencyHz, ok ? "OK" : "ERROR");
  return ok;
}

bool captureWindow(Metrics& metrics, Metrics& rawMetrics) {
  size_t captured = 0;
  const uint32_t startedUs = micros();
  uint32_t firstSampleUs = 0;
  uint32_t lastSampleUs = 0;

  while (captured < WINDOW_SAMPLES) {
    int32_t raw;
    if (!readRawSample(raw)) {
      if (static_cast<uint32_t>(micros() - startedUs) > 2000000UL) {
        return false;
      }
      continue;
    }

    const uint32_t now = micros();
    if (captured == 0) {
      firstSampleUs = now;
    }
    lastSampleUs = now;
    sampleBuffer[captured++] = rawToVolts(raw, adcFullScaleV);
  }

  const uint32_t elapsedUs =
      static_cast<uint32_t>(lastSampleUs - firstSampleUs);
  rawMetrics = calculateMetrics(sampleBuffer, WINDOW_SAMPLES, elapsedUs);
  for (float& sample : sampleBuffer) {
    sample = calibrateInputVoltage(sample);
  }
  metrics = calculateMetrics(sampleBuffer, WINDOW_SAMPLES, elapsedUs);
  return true;
}

float voltageRmsToCurrent(float rmsV) {
  const float transformerRatio = SCT_PRIMARY_A / SCT_SECONDARY_A;
  return rmsV * transformerRatio / BURDEN_OHM * CURRENT_CAL_GAIN;
}

bool metricsAreValid(const Metrics& m, const Metrics& raw) {
  // IN1-IN0 is bipolar: a negative differential voltage is valid. Only a
  // value close to either differential full-scale limit indicates a rail.
  const bool lowRail = raw.minV < -(adcFullScaleV - 0.05f);
  const bool highRail = raw.maxV > (adcFullScaleV - 0.05f);
  const bool frequencyOk =
      m.rmsAcV < MIN_FREQUENCY_RMS_V ||
      (m.frequencyHz >= 45.0f && m.frequencyHz <= 65.0f);
  const bool finite = isfinite(m.meanV) && isfinite(m.rmsAcV) &&
                      isfinite(m.frequencyHz);
  return !lowRail && !highRail && frequencyOk && finite;
}

void printCalibrationReport(const Metrics& raw, const Metrics& calibrated) {
  Serial.printf(
      "[RAW IN1-IN0] media=%.6f V | AC_RMS=%.6f V | Vpp=%.6f V | "
      "min=%.6f V | max=%.6f V | f=%.2f Hz | Fs=%.1f SPS\n",
      raw.meanV, raw.rmsAcV, raw.vppV, raw.minV, raw.maxV,
      raw.frequencyHz, raw.sampleRate);
  Serial.printf(
      "[CAL ACTUAL] media=%.6f V | AC_RMS=%.6f V | Vpp=%.6f V | "
      "min=%.6f V | max=%.6f V\n",
      calibrated.meanV, calibrated.rmsAcV, calibrated.vppV,
      calibrated.minV, calibrated.maxV);

  const float expectedMean =
      (CALIBRATION_HIGH_V + CALIBRATION_LOW_V) * 0.5f;
  const float expectedPeak =
      (CALIBRATION_HIGH_V - CALIBRATION_LOW_V) * 0.5f;
  const float measuredPeak = raw.rmsAcV * sqrtf(2.0f);
  const float sineShape =
      raw.rmsAcV > 0.0f
          ? raw.vppV / (2.0f * sqrtf(2.0f) * raw.rmsAcV)
          : 0.0f;
  const bool sineIsValid =
      measuredPeak > 0.05f &&
      fabsf(raw.frequencyHz - CALIBRATION_FREQUENCY_HZ) <= 1.0f &&
      sineShape >= 0.90f && sineShape <= 1.10f;

  if (!sineIsValid) {
    Serial.printf(
        "[CAL ESPERA] Aplique seno %.1f Hz, Low=%.3f V, High=%.3f V, "
        "Phase=0 (shape=%.3f).\n",
        CALIBRATION_FREQUENCY_HZ, CALIBRATION_LOW_V,
        CALIBRATION_HIGH_V, sineShape);
    return;
  }

  const float suggestedGain = expectedPeak / measuredPeak;
  const float suggestedOffset = expectedMean - suggestedGain * raw.meanV;
  Serial.printf(
      "[CONST SUGERIDAS] INPUT_CAL_GAIN=%.7ff | "
      "INPUT_CAL_OFFSET_V=%.7ff\n",
      suggestedGain, suggestedOffset);
}

size_t encodeRemainingLength(size_t length, uint8_t out[4]) {
  size_t count = 0;
  do {
    uint8_t byte = length % 128;
    length /= 128;
    if (length > 0) {
      byte |= 0x80;
    }
    out[count++] = byte;
  } while (length > 0 && count < 4);
  return count;
}

bool appendMqttString(uint8_t* buffer, size_t capacity, size_t& position,
                      const char* value) {
  const size_t length = strlen(value);
  if (length > 65535 || position + 2 + length > capacity) {
    return false;
  }
  buffer[position++] = static_cast<uint8_t>(length >> 8);
  buffer[position++] = static_cast<uint8_t>(length & 0xFF);
  memcpy(buffer + position, value, length);
  position += length;
  return true;
}

bool sendMqttPacket(uint8_t header, const uint8_t* body, size_t bodyLength) {
  if (mqttSocket == nullptr || !mqttSocket->connected()) {
    return false;
  }
  uint8_t remaining[4];
  const size_t remainingCount = encodeRemainingLength(bodyLength, remaining);
  if (mqttSocket->write(&header, 1) != 1 ||
      mqttSocket->write(remaining, remainingCount) != remainingCount ||
      (bodyLength > 0 && mqttSocket->write(body, bodyLength) != bodyLength)) {
    mqttSocket->stop();
    return false;
  }
  lastMqttActivityMs = millis();
  return true;
}

bool mqttConnect() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  plainClient.stop();
  secureClient.stop();
  if (SCADA_MQTT_USE_TLS) {
    if (strlen(SCADA_MQTT_CA_CERT) == 0) {
      Serial.println("[MQTT] TLS enabled but CA certificate is empty.");
      return false;
    }
    secureClient.setCACert(SCADA_MQTT_CA_CERT);
    mqttSocket = &secureClient;
  } else {
    mqttSocket = &plainClient;
  }

  if (!mqttSocket->connect(SCADA_MQTT_HOST, SCADA_MQTT_PORT)) {
    Serial.printf("[MQTT] TCP connection failed: %s:%u\n", SCADA_MQTT_HOST,
                  SCADA_MQTT_PORT);
    return false;
  }

  uint8_t body[512];
  size_t position = 0;
  if (!appendMqttString(body, sizeof(body), position, "MQTT")) {
    return false;
  }
  body[position++] = 0x04;
  uint8_t connectFlags = 0x02;
  if (strlen(SCADA_MQTT_USERNAME) > 0) {
    connectFlags |= 0x80;
  }
  if (strlen(SCADA_MQTT_PASSWORD) > 0) {
    connectFlags |= 0x40;
  }
  body[position++] = connectFlags;
  body[position++] = static_cast<uint8_t>(MQTT_KEEPALIVE_S >> 8);
  body[position++] = static_cast<uint8_t>(MQTT_KEEPALIVE_S & 0xFF);

  char clientId[48];
  uint64_t mac = ESP.getEfuseMac();
  snprintf(clientId, sizeof(clientId), "scada-%s-%04X", SCADA_DEVICE_ID,
           static_cast<unsigned int>(mac & 0xFFFF));
  if (!appendMqttString(body, sizeof(body), position, clientId) ||
      (strlen(SCADA_MQTT_USERNAME) > 0 &&
       !appendMqttString(body, sizeof(body), position,
                         SCADA_MQTT_USERNAME)) ||
      (strlen(SCADA_MQTT_PASSWORD) > 0 &&
       !appendMqttString(body, sizeof(body), position,
                         SCADA_MQTT_PASSWORD))) {
    mqttSocket->stop();
    return false;
  }

  if (!sendMqttPacket(0x10, body, position)) {
    return false;
  }

  mqttSocket->setTimeout(3000);
  uint8_t connAck[4];
  if (mqttSocket->readBytes(connAck, sizeof(connAck)) != sizeof(connAck) ||
      connAck[0] != 0x20 || connAck[1] != 0x02 || connAck[3] != 0x00) {
    Serial.println("[MQTT] Broker rejected or did not answer CONNECT.");
    mqttSocket->stop();
    return false;
  }

  Serial.printf("[MQTT] Connected as %s, topic=%s\n", clientId,
                SCADA_MQTT_TOPIC);
  lastMqttActivityMs = millis();
  return true;
}

bool mqttPublish(const char* payload) {
  uint8_t body[MQTT_PAYLOAD_SIZE + 128];
  size_t position = 0;
  if (!appendMqttString(body, sizeof(body), position, SCADA_MQTT_TOPIC)) {
    return false;
  }
  ++mqttPacketId;
  if (mqttPacketId == 0) {
    ++mqttPacketId;
  }
  body[position++] = static_cast<uint8_t>(mqttPacketId >> 8);
  body[position++] = static_cast<uint8_t>(mqttPacketId & 0xFF);
  const size_t payloadLength = strlen(payload);
  if (position + payloadLength > sizeof(body)) {
    return false;
  }
  memcpy(body + position, payload, payloadLength);
  position += payloadLength;
  if (!sendMqttPacket(0x32, body, position)) {
    return false;
  }

  mqttSocket->setTimeout(2000);
  uint8_t pubAck[4];
  if (mqttSocket->readBytes(pubAck, sizeof(pubAck)) != sizeof(pubAck) ||
      pubAck[0] != 0x40 || pubAck[1] != 0x02 ||
      pubAck[2] != static_cast<uint8_t>(mqttPacketId >> 8) ||
      pubAck[3] != static_cast<uint8_t>(mqttPacketId & 0xFF)) {
    Serial.println("[MQTT] Missing or invalid PUBACK; retaining message.");
    mqttSocket->stop();
    return false;
  }
  lastMqttActivityMs = millis();
  return true;
}

void enqueueMessage(const char* payload) {
  if (queueCount == MQTT_QUEUE_SIZE) {
    queueHead = (queueHead + 1) % MQTT_QUEUE_SIZE;
    --queueCount;
    ++droppedMessages;
  }
  const size_t tail = (queueHead + queueCount) % MQTT_QUEUE_SIZE;
  strlcpy(mqttQueue[tail].payload, payload,
          sizeof(mqttQueue[tail].payload));
  ++queueCount;
}

void flushMqttQueue() {
  while (queueCount > 0 && mqttSocket != nullptr &&
         mqttSocket->connected()) {
    if (!mqttPublish(mqttQueue[queueHead].payload)) {
      return;
    }
    queueHead = (queueHead + 1) % MQTT_QUEUE_SIZE;
    --queueCount;
  }
}

void maintainConnections() {
  const uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    if (static_cast<uint32_t>(now - lastWifiAttemptMs) >= WIFI_RETRY_MS) {
      lastWifiAttemptMs = now;
      WiFi.disconnect();
      WiFi.begin(SCADA_WIFI_SSID, SCADA_WIFI_PASSWORD);
      Serial.println("[WIFI] Connecting...");
    }
    return;
  }

  if (mqttSocket == nullptr || !mqttSocket->connected()) {
    if (static_cast<uint32_t>(now - lastMqttAttemptMs) >= MQTT_RETRY_MS) {
      lastMqttAttemptMs = now;
      mqttConnect();
    }
    return;
  }

  flushMqttQueue();
  if (!mqttSocket->connected()) {
    return;
  }

  while (mqttSocket->available() > 0) {
    mqttSocket->read();
  }
  const uint32_t afterFlush = millis();
  if (static_cast<uint32_t>(afterFlush - lastMqttActivityMs) >= 30000UL) {
    sendMqttPacket(0xC0, nullptr, 0);
  }
}

void buildAndQueuePayload(const Metrics& m, const Metrics& raw) {
  const bool valid = metricsAreValid(m, raw);
  const float currentRmsA = valid ? voltageRmsToCurrent(m.rmsAcV) : 0.0f;
  const bool charging = valid && currentRmsA >= CHARGING_THRESHOLD_A;
  const time_t now = time(nullptr);
  char payload[MQTT_PAYLOAD_SIZE];

  const int written = snprintf(
      payload, sizeof(payload),
      "{\"version\":1,\"dispositivo\":\"%s\",\"secuencia\":%lu,"
      "\"timestamp_unix\":%lld,\"corriente_rms_a\":%.4f,"
      "\"senal_rms_v\":%.6f,\"bias_v\":%.6f,\"vpp_v\":%.6f,"
      "\"frecuencia_hz\":%.3f,\"muestreo_sps\":%.1f,"
      "\"valido\":%s,\"cargando\":%s,\"cola\":%u,"
      "\"descartados\":%lu,\"uptime_ms\":%lu}",
      SCADA_DEVICE_ID, static_cast<unsigned long>(++sequenceNumber),
      static_cast<long long>(now > 1700000000 ? now : 0), currentRmsA,
      m.rmsAcV, m.meanV, m.vppV, m.frequencyHz, m.sampleRate,
      valid ? "true" : "false", charging ? "true" : "false",
      static_cast<unsigned int>(queueCount),
      static_cast<unsigned long>(droppedMessages),
      static_cast<unsigned long>(millis()));

  if (written <= 0 || static_cast<size_t>(written) >= sizeof(payload)) {
    Serial.println("[MQTT] Payload overflow; measurement not queued.");
    return;
  }
  enqueueMessage(payload);
  Serial.println(payload);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n[BOOT] SCADA SCT-013 + ADS1263");
  Serial.println("[ADC] Entrada diferencial IN1-IN0");

  pinMode(Pins::CS, OUTPUT);
  pinMode(Pins::RESET, OUTPUT);
  pinMode(Pins::DRDY, INPUT);
  digitalWrite(Pins::CS, HIGH);
  digitalWrite(Pins::RESET, HIGH);
  SPI.begin(Pins::SCLK, Pins::MISO, Pins::MOSI, Pins::CS);

  if (!runMathSelfTest()) {
    Serial.println("[FATAL] Math self-test failed.");
    while (true) {
      delay(1000);
    }
  }

  while (!configureAds1263()) {
    Serial.println("[ADC ERROR] Check power and SPI wiring. Retry in 2 s.");
    delay(2000);
  }

  float measuredSupplyV;
  if (measureAnalogSupply(measuredSupplyV)) {
    adcFullScaleV = measuredSupplyV;
    Serial.printf("[ADC] AVDD-AVSS=%.5f V\n", adcFullScaleV);
  } else {
    Serial.println("[ADC] Supply monitor failed; using 5.000 V.");
  }

  if (CALIBRATION_MODE) {
    WiFi.mode(WIFI_OFF);
    Serial.println(
        "[MODE] CALIBRACION LOCAL: WiFi y MQTT desactivados.");
    Serial.printf(
        "[GEN] Sine %.1f Hz | High=%.3f V | Low=%.3f V | Phase=0\n",
        CALIBRATION_FREQUENCY_HZ, CALIBRATION_HIGH_V,
        CALIBRATION_LOW_V);
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(SCADA_WIFI_SSID, SCADA_WIFI_PASSWORD);
    lastWifiAttemptMs = millis();
    configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
  }
}

void loop() {
  if (!CALIBRATION_MODE) {
    maintainConnections();
  }

  Metrics metrics;
  Metrics rawMetrics;
  if (!captureWindow(metrics, rawMetrics)) {
    Serial.println("[ADC] Capture failed.");
    if (!CALIBRATION_MODE) {
      maintainConnections();
    }
    delay(100);
    return;
  }

  const uint32_t now = millis();
  const uint32_t interval = CALIBRATION_MODE
                                ? CALIBRATION_REPORT_INTERVAL_MS
                                : PUBLISH_INTERVAL_MS;
  if (static_cast<uint32_t>(now - lastPublishMs) >= interval) {
    lastPublishMs = now;
    if (CALIBRATION_MODE) {
      printCalibrationReport(rawMetrics, metrics);
    } else {
      buildAndQueuePayload(metrics, rawMetrics);
    }
  }

  if (!CALIBRATION_MODE) {
    maintainConnections();
  }
  delay(25);
}
