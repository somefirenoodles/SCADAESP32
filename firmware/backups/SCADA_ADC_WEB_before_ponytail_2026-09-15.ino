/*
  SCADA EV: SCT-013 + ADS1263 + ESP32 + local web dashboard

  Analog input:
    Current: ADS1263 IN1 positive, IN0 negative
    Voltage: ADS1263 IN3 positive, IN2 negative

  ADS1263 SPI:
    CS=27, SCLK=18, MISO/DOUT=19, MOSI/DIN=23, DRDY=25, RESET=26

  Current sensor defaults:
    SCT-013 voltage-output model, 100 A / 1 V
    The sensor has an internal burden; external RB must be removed.

  The ESP32 creates a local SoftAP and serves the dashboard at 192.168.4.1.
*/

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
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
// Commissioning speed for reliable point-to-point wiring.
constexpr uint32_t SPI_HZ = 250000;
constexpr uint32_t CS_SETTLE_US = 2;
constexpr uint32_t REGISTER_SETTLE_MS = 1;
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
constexpr uint8_t INPMUX_IN3_IN2 = 0x32;
constexpr uint8_t INPMUX_ANALOG_SUPPLY_MONITOR = 0xCC;
}  // namespace Ads

static_assert(Ads::INPMUX_IN3_IN2 == 0x32,
              "ADS1263 INPMUX must select IN3 positive and IN2 negative");

constexpr float NOMINAL_ANALOG_SUPPLY_V = 5.000f;
constexpr float INTERNAL_REFERENCE_V = 2.500f;

// IN1-IN0 calibration obtained with the SDG1032X after correcting polarity.
// Source: Sine 60 Hz, Low=1.500 V, High=3.000 V, Phase=0.
constexpr float INPUT_CAL_GAIN = 0.9412289f;
constexpr float INPUT_CAL_OFFSET_V = -0.0512069f;

// Keep true for the local SDG1032X calibration test. In this mode WiFi/MQTT
// are disabled and no current value is published as if it were production.
constexpr bool CALIBRATION_MODE = false;
constexpr float CALIBRATION_LOW_V = 1.500f;
constexpr float CALIBRATION_HIGH_V = 3.000f;
constexpr float CALIBRATION_FREQUENCY_HZ = 60.0f;
constexpr uint32_t CALIBRATION_REPORT_INTERVAL_MS = 1000;

// Nominal transfer ratio printed on the voltage-output SCT. The external
// burden resistor RB is not part of this configuration. Re-adjust
// CURRENT_CAL_GAIN against a trusted clamp meter after installation.
constexpr float SCT_RATED_PRIMARY_A = 100.0f;
constexpr float SCT_RATED_OUTPUT_V = 1.0f;
constexpr float CURRENT_CAL_GAIN = 1.0f;
constexpr float CURRENT_MIN_A = 0.25f;
constexpr float CURRENT_MAX_A = 100.0f;
constexpr float CHARGING_THRESHOLD_A = 5.0f;
constexpr float NOMINAL_LINE_V = 208.0f;
constexpr float ASSUMED_POWER_FACTOR = 1.0f;

// Combined scale of the isolated voltage sensor and its conditioning circuit.
// Calibrate against a trusted AC meter before setting this flag to true:
//   VOLTAGE_MAINS_PER_SENSOR_V = reference_mains_rms / sensor_rms
// Never connect mains directly to IN2/IN3.
constexpr bool VOLTAGE_SENSOR_CALIBRATED = false;
constexpr float VOLTAGE_MAINS_PER_SENSOR_V = 1.0f;

constexpr size_t WINDOW_SAMPLES = 1200;
constexpr uint32_t DRDY_TIMEOUT_US = 100000;
constexpr float MIN_FREQUENCY_RMS_V = 0.005f;
constexpr uint32_t PUBLISH_INTERVAL_MS = 5000;
constexpr uint32_t ADC_RETRY_MS = 5000;
constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr uint32_t MQTT_RETRY_MS = 5000;
constexpr uint16_t MQTT_KEEPALIVE_S = 60;
constexpr uint8_t MQTT_PUBLISH_QOS = 0;
constexpr size_t MQTT_PAYLOAD_SIZE = 640;
constexpr size_t MQTT_QUEUE_SIZE = 16;
const IPAddress LOCAL_AP_IP(192, 168, 4, 1);
const IPAddress LOCAL_AP_MASK(255, 255, 255, 0);

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
uint32_t lastAdcAttemptMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastMqttAttemptMs = 0;
uint32_t lastMqttActivityMs = 0;
uint32_t lastPublishMs = 0;
uint32_t lastEnergyUpdateMs = 0;
uint64_t chargingUptimeMs = 0;
double accumulatedEnergyWh = 0.0;
uint16_t mqttPacketId = 0;
bool adcAvailable = false;
bool wifiInfoPrinted = false;

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

WebServer webServer(80);
char latestPayload[MQTT_PAYLOAD_SIZE] = "";
bool latestPayloadReady = false;

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>SCADA EV</title>
  <style>
    :root{color-scheme:light;font-family:Arial,sans-serif;color:#171b20;background:#fff}
    *{box-sizing:border-box}body{margin:0;background:#fff}main{width:min(920px,100%);margin:auto;padding:18px 22px 30px}
    header{display:flex;align-items:center;justify-content:space-between;gap:16px;border-bottom:1px solid #bfc6ce;padding-bottom:10px}
    h1{margin:0;font-size:1.3rem}header p{margin:2px 0 0;color:#525b66;font-size:.875rem}h2{margin:0 0 7px;font-size:.95rem}
    code,pre,dd,.value{font-family:ui-monospace,Consolas,monospace}.status{display:flex;align-items:center;gap:7px;font-size:.875rem}
    .dot{width:9px;height:9px;border-radius:50%;background:#15803d}.dot.error{background:#b42318}section{margin-top:14px;min-width:0}
    .metrics{display:grid;grid-template-columns:repeat(4,1fr);margin:0;border:1px solid #d2d7dd}.metrics div{border-right:1px solid #d2d7dd;padding:9px 12px}.metrics div:last-child{border:0}
    dt{color:#525b66;font-size:.8rem}dd{margin:2px 0 0;font-size:1.18rem;font-weight:650}.grid{display:grid;grid-template-columns:minmax(340px,.85fr) minmax(0,1.15fr);gap:20px;margin-top:16px}
    .grid section{margin:0;border-top:2px solid #171b20;padding-top:8px}table{width:100%;border-collapse:collapse;font-size:.84rem}th,td{border-bottom:1px solid #e0e4e8;padding:6px 7px;text-align:left}th{color:#39434e}.value{text-align:right}
    .json-head{display:flex;align-items:baseline;justify-content:space-between;gap:12px}.json-head span{color:#525b66;font-size:.8rem}
    pre{min-height:330px;max-height:430px;overflow:auto;margin:0;border:1px solid #d2d7dd;background:#f7f8f9;padding:10px 12px;font-size:.82rem;line-height:1.48}
    pre:focus-visible{outline:3px solid #38bdf8;outline-offset:2px}.waiting{color:#525b66}
    @media(max-width:700px){main{padding:14px}header{align-items:flex-start;flex-direction:column;gap:7px}.metrics{grid-template-columns:1fr 1fr}.metrics div:nth-child(2){border-right:0}.metrics div:nth-child(-n+2){border-bottom:1px solid #d2d7dd}.grid{grid-template-columns:1fr}pre{min-height:auto}}
    @media(max-width:420px){.metrics{grid-template-columns:1fr}.metrics div{border-right:0;border-bottom:1px solid #d2d7dd}.metrics div:last-child{border-bottom:0}}
  </style>
</head>
<body>
<main>
  <header>
    <div><h1>SCADA EV</h1></div>
    <div class="status" role="status" aria-live="polite"><span id="dot" class="dot"></span><span id="status">Consultando</span></div>
  </header>
  <section aria-labelledby="summary"><h2 id="summary">Lectura actual</h2>
    <dl class="metrics">
      <div><dt>Corriente</dt><dd id="current">—</dd></div><div><dt>Potencia</dt><dd id="power">—</dd></div>
      <div><dt>Energía</dt><dd id="energy">—</dd></div><div><dt>Estado</dt><dd id="charging">—</dd></div>
    </dl>
  </section>
  <div class="grid">
    <section aria-labelledby="fields"><h2 id="fields">Campos</h2><table><thead><tr><th>Campo</th><th class="value">Valor</th></tr></thead><tbody id="fieldRows"></tbody></table></section>
    <section aria-labelledby="json"><div class="json-head"><h2 id="json"><code>GET /api/latest</code></h2><span id="received"></span></div><pre id="raw" tabindex="0" aria-label="Respuesta JSON">Esperando la primera lectura.</pre></section>
  </div>
</main>
<script>
const spec=[['version','Versión',''],['secuencia','Secuencia',''],['timestamp_unix','Timestamp Unix',''],['estado_carga','Estado de carga',''],['uptime_carga_s','Tiempo de carga',' s'],['corriente_a','Corriente',' A'],['potencia_w','Potencia',' W'],['energia_wh','Energía',' Wh'],['voltaje_usado_v','Voltaje usado',' V'],['potencia_estimada','Potencia estimada','']];
const byId=id=>document.getElementById(id);const show=v=>typeof v==='boolean'?(v?'Sí':'No'):v;
function render(data){byId('current').textContent=Number(data.corriente_a).toFixed(2)+' A';byId('power').textContent=(Number(data.potencia_w)/1000).toFixed(2)+' kW';byId('energy').textContent=(Number(data.energia_wh)/1000).toFixed(2)+' kWh';byId('charging').textContent=data.estado_carga==='C'?'Cargando':'Sin carga';byId('fieldRows').innerHTML=spec.map(([key,label,unit])=>'<tr><td>'+label+'</td><td class="value">'+show(data[key])+unit+'</td></tr>').join('');byId('raw').textContent=JSON.stringify(data,null,2);byId('received').textContent=new Date().toLocaleTimeString();byId('status').textContent='Conectado';byId('dot').className='dot'}
async function update(){try{const response=await fetch('/api/latest',{cache:'no-store'});if(!response.ok)throw new Error('HTTP '+response.status);render(await response.json())}catch(error){byId('status').textContent='Sin datos · '+error.message;byId('dot').className='dot error'}}
update();setInterval(update,5000);
</script>
</body>
</html>
)HTML";

void selectAdc() {
  SPI.beginTransaction(adsSpi);
  digitalWrite(Pins::CS, LOW);
  delayMicroseconds(Ads::CS_SETTLE_US);
}

void deselectAdc() {
  delayMicroseconds(Ads::CS_SETTLE_US);
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
  delay(Ads::REGISTER_SETTLE_MS);
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
  uint8_t status = 0;
  const uint32_t statusStartedUs = micros();
  do {
    SPI.transfer(Ads::CMD_RDATA1);
    status = SPI.transfer(0x00);
    if ((status & 0x40) != 0) {
      break;
    }
    if (static_cast<uint32_t>(micros() - statusStartedUs) >=
        DRDY_TIMEOUT_US) {
      deselectAdc();
      ++statusErrors;
      return false;
    }
    delayMicroseconds(5);
  } while (true);

  for (uint8_t& byte : data) {
    byte = SPI.transfer(0x00);
  }
  const uint8_t checksumReceived = SPI.transfer(0x00);
  deselectAdc();

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

float applyLinearCalibration(float measuredV, float gain, float offsetV) {
  return measuredV * gain + offsetV;
}

float calibrateInputVoltage(float measuredV) {
  return applyLinearCalibration(measuredV, INPUT_CAL_GAIN,
                                INPUT_CAL_OFFSET_V);
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

bool captureWindow(uint8_t inpmux, float gain, float offsetV,
                   Metrics& metrics, Metrics& rawMetrics) {
  if (!selectConversion(Ads::MODE2_PGA_BYPASS_2400_SPS,
                        Ads::REFMUX_AVDD_AVSS, inpmux)) {
    return false;
  }

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
    sample = applyLinearCalibration(sample, gain, offsetV);
  }
  metrics = calculateMetrics(sampleBuffer, WINDOW_SAMPLES, elapsedUs);
  return true;
}

float voltageRmsToCurrent(float rmsV) {
  const float sensorAmpsPerVolt =
      SCT_RATED_PRIMARY_A / SCT_RATED_OUTPUT_V;
  return rmsV * sensorAmpsPerVolt * CURRENT_CAL_GAIN;
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

bool readMqttByte(uint8_t& value, uint32_t deadlineMs) {
  while (static_cast<int32_t>(millis() - deadlineMs) < 0) {
    if (mqttSocket != nullptr && mqttSocket->available() > 0) {
      const int byteRead = mqttSocket->read();
      if (byteRead >= 0) {
        value = static_cast<uint8_t>(byteRead);
        return true;
      }
    }
    if (mqttSocket == nullptr || !mqttSocket->connected()) {
      return false;
    }
    delay(1);
  }
  return false;
}

bool readMqttPacket(uint8_t& header, uint8_t* body, size_t bodyCapacity,
                    size_t& bodyLength, uint32_t timeoutMs) {
  const uint32_t deadlineMs = millis() + timeoutMs;
  if (!readMqttByte(header, deadlineMs)) {
    return false;
  }

  bodyLength = 0;
  size_t multiplier = 1;
  for (uint8_t encodedBytes = 0; encodedBytes < 4; ++encodedBytes) {
    uint8_t encoded = 0;
    if (!readMqttByte(encoded, deadlineMs)) {
      return false;
    }
    bodyLength += static_cast<size_t>(encoded & 0x7F) * multiplier;
    if ((encoded & 0x80) == 0) {
      break;
    }
    if (encodedBytes == 3) {
      return false;
    }
    multiplier *= 128;
  }

  if (bodyLength > bodyCapacity) {
    return false;
  }
  for (size_t index = 0; index < bodyLength; ++index) {
    if (!readMqttByte(body[index], deadlineMs)) {
      return false;
    }
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

  // Use the same hostname/port connection path as the previously verified
  // MQTT build. WiFiClient resolves a numeric IPv4 address internally too.
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

  uint8_t connAckHeader = 0;
  uint8_t connAck[4];
  size_t connAckLength = 0;
  if (!readMqttPacket(connAckHeader, connAck, sizeof(connAck),
                      connAckLength, 3000) ||
      connAckHeader != 0x20 || connAckLength != 2 || connAck[1] != 0x00) {
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
  if (MQTT_PUBLISH_QOS == 1) {
    ++mqttPacketId;
    if (mqttPacketId == 0) {
      ++mqttPacketId;
    }
    body[position++] = static_cast<uint8_t>(mqttPacketId >> 8);
    body[position++] = static_cast<uint8_t>(mqttPacketId & 0xFF);
  }
  const size_t payloadLength = strlen(payload);
  if (position + payloadLength > sizeof(body)) {
    return false;
  }
  memcpy(body + position, payload, payloadLength);
  position += payloadLength;
  const uint8_t publishHeader = MQTT_PUBLISH_QOS == 1 ? 0x32 : 0x30;
  if (!sendMqttPacket(publishHeader, body, position)) {
    return false;
  }
  if (MQTT_PUBLISH_QOS == 0) {
    return true;
  }

  const uint32_t ackDeadlineMs = millis() + 3000;
  while (static_cast<int32_t>(millis() - ackDeadlineMs) < 0) {
    uint8_t responseHeader = 0;
    uint8_t responseBody[8];
    size_t responseLength = 0;
    const uint32_t remainingMs = ackDeadlineMs - millis();
    if (!readMqttPacket(responseHeader, responseBody, sizeof(responseBody),
                        responseLength, remainingMs)) {
      break;
    }

    const uint8_t packetType = responseHeader & 0xF0;
    if (packetType == 0xD0 && responseLength == 0) {
      continue;
    }
    if (packetType == 0x40 && responseLength == 2 &&
        responseBody[0] == static_cast<uint8_t>(mqttPacketId >> 8) &&
        responseBody[1] == static_cast<uint8_t>(mqttPacketId & 0xFF)) {
      lastMqttActivityMs = millis();
      return true;
    }
    Serial.printf("[MQTT] Unexpected packet type=0x%02X length=%u.\n",
                  packetType, static_cast<unsigned int>(responseLength));
  }

  Serial.println("[MQTT] Missing or invalid PUBACK; retaining message.");
  mqttSocket->stop();
  return false;
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
    wifiInfoPrinted = false;
    if (static_cast<uint32_t>(now - lastWifiAttemptMs) >= WIFI_RETRY_MS) {
      lastWifiAttemptMs = now;
      WiFi.disconnect();
      if (strlen(SCADA_WIFI_PASSWORD) == 0) {
        WiFi.begin(SCADA_WIFI_SSID);
      } else {
        WiFi.begin(SCADA_WIFI_SSID, SCADA_WIFI_PASSWORD);
      }
      Serial.println("[WIFI] Connecting...");
    }
    return;
  }

  if (!wifiInfoPrinted) {
    Serial.printf("[WIFI] SSID=%s | RSSI=%d dBm | MAC=%s\n",
                  WiFi.SSID().c_str(), WiFi.RSSI(),
                  WiFi.macAddress().c_str());
    Serial.print("[WIFI] IP=");
    Serial.print(WiFi.localIP());
    Serial.print(" | gateway=");
    Serial.print(WiFi.gatewayIP());
    Serial.print(" | mask=");
    Serial.println(WiFi.subnetMask());
    Serial.printf("[MQTT] Target=%s:%u | topic=%s\n", SCADA_MQTT_HOST,
                  SCADA_MQTT_PORT, SCADA_MQTT_TOPIC);
    wifiInfoPrinted = true;
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

void buildAndQueuePayload(const Metrics& current,
                          const Metrics& rawCurrent,
                          const Metrics& voltage,
                          const Metrics& rawVoltage,
                          bool adcIsAvailable) {
  bool currentValid = adcIsAvailable && metricsAreValid(current, rawCurrent);
  const bool voltageValid =
      adcIsAvailable && metricsAreValid(voltage, rawVoltage);
  float currentRmsA = currentValid ? voltageRmsToCurrent(current.rmsAcV)
                                   : 0.0f;
  if (currentValid && currentRmsA > CURRENT_MAX_A) {
    currentValid = false;
    currentRmsA = 0.0f;
  } else if (currentValid && currentRmsA < CURRENT_MIN_A) {
    currentRmsA = 0.0f;
  }

  const bool charging = currentValid && currentRmsA > CHARGING_THRESHOLD_A;
  const bool mainsVoltageValid =
      voltageValid && VOLTAGE_SENSOR_CALIBRATED;
  const float lineVoltageV = mainsVoltageValid
                                 ? voltage.rmsAcV * VOLTAGE_MAINS_PER_SENSOR_V
                                 : NOMINAL_LINE_V;
  const bool powerEstimated = !mainsVoltageValid;
  const float powerW = currentValid
                           ? lineVoltageV * currentRmsA * ASSUMED_POWER_FACTOR
                           : 0.0f;

  const uint32_t nowMs = millis();
  if (lastEnergyUpdateMs != 0) {
    const uint32_t elapsedMs =
        static_cast<uint32_t>(nowMs - lastEnergyUpdateMs);
    if (currentValid) {
      accumulatedEnergyWh +=
          static_cast<double>(powerW) * elapsedMs / 3600000.0;
    }
    if (charging) {
      chargingUptimeMs += elapsedMs;
    }
  }
  lastEnergyUpdateMs = nowMs;

  const time_t now = time(nullptr);
  char payload[MQTT_PAYLOAD_SIZE];
  const int written = snprintf(
      payload, sizeof(payload),
      "{\"version\":3,\"secuencia\":%lu,\"timestamp_unix\":%lld,"
      "\"estado_carga\":\"%s\",\"uptime_carga_s\":%llu,"
      "\"corriente_a\":%.3f,\"potencia_w\":%.2f,"
      "\"energia_wh\":%.4f,\"voltaje_usado_v\":%.2f,"
      "\"potencia_estimada\":%s}",
      static_cast<unsigned long>(++sequenceNumber),
      static_cast<long long>(now > 1700000000 ? now : 0),
      charging ? "C" : "NC",
      static_cast<unsigned long long>(chargingUptimeMs / 1000ULL),
      currentRmsA, powerW, accumulatedEnergyWh, lineVoltageV,
      powerEstimated ? "true" : "false");

  if (written <= 0 || static_cast<size_t>(written) >= sizeof(payload)) {
    Serial.println("[MQTT] Payload overflow; measurement not queued.");
    return;
  }
  strlcpy(latestPayload, payload, sizeof(latestPayload));
  latestPayloadReady = true;
  Serial.printf(
      "[MEDICION] %s | CARGA=%llu s | I=%.3f A | P=%.2f W | "
      "E=%.4f Wh | V=%.2f V%s\n",
      charging ? "C" : "NC",
      static_cast<unsigned long long>(chargingUptimeMs / 1000ULL),
      currentRmsA, powerW, accumulatedEnergyWh, lineVoltageV,
      powerEstimated ? " (nominal)" : "");
  Serial.print("[JSON] ");
  Serial.println(payload);
}

bool initializeAdc() {
  lastAdcAttemptMs = millis();
  if (!configureAds1263()) {
    Serial.println(
        "[ADC] Not available; MQTT continues with invalid measurements.");
    return false;
  }

  float measuredSupplyV;
  if (measureAnalogSupply(measuredSupplyV)) {
    adcFullScaleV = measuredSupplyV;
    Serial.printf("[ADC] AVDD-AVSS=%.5f V\n", adcFullScaleV);
  } else {
    adcFullScaleV = NOMINAL_ANALOG_SUPPLY_V;
    Serial.println("[ADC] Supply monitor failed; using 5.000 V.");
  }
  Serial.println("[ADC] Online.");
  return true;
}

void startLocalWebServer() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(LOCAL_AP_IP, LOCAL_AP_IP, LOCAL_AP_MASK)) {
    Serial.println("[WEB] Failed to configure SoftAP address.");
  }
  const bool apStarted = strlen(SCADA_AP_PASSWORD) == 0
                             ? WiFi.softAP(SCADA_AP_SSID)
                             : WiFi.softAP(SCADA_AP_SSID,
                                           SCADA_AP_PASSWORD);
  if (!apStarted) {
    Serial.println("[WEB] Failed to start SoftAP.");
    return;
  }

  webServer.on("/", HTTP_GET, []() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
  });
  webServer.on("/api/latest", HTTP_GET, []() {
    webServer.sendHeader("Cache-Control", "no-store");
    if (!latestPayloadReady) {
      webServer.send(503, "application/json",
                     "{\"error\":\"measurement_not_ready\"}");
      return;
    }
    webServer.send(200, "application/json", latestPayload);
  });
  webServer.on("/health", HTTP_GET, []() {
    char health[192];
    snprintf(health, sizeof(health),
             "{\"status\":\"ok\",\"adc_disponible\":%s,"
             "\"medicion_disponible\":%s,\"clientes\":%u,"
             "\"uptime_ms\":%lu}",
             adcAvailable ? "true" : "false",
             latestPayloadReady ? "true" : "false",
             static_cast<unsigned int>(WiFi.softAPgetStationNum()),
             static_cast<unsigned long>(millis()));
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", health);
  });
  webServer.onNotFound([]() {
    webServer.send(404, "application/json", "{\"error\":\"not_found\"}");
  });
  webServer.begin();

  Serial.printf("[WIFI] SoftAP SSID=%s | IP=", SCADA_AP_SSID);
  Serial.println(WiFi.softAPIP());
  Serial.println("[WEB] Dashboard=http://192.168.4.1/");
  Serial.println("[WEB] JSON=http://192.168.4.1/api/latest");
  Serial.println("[WEB] Health=http://192.168.4.1/health");
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n[BOOT] SCADA corriente + voltaje + ADS1263");
  Serial.println("[ADC] Corriente IN1-IN0 | Voltaje IN3-IN2");

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

  adcAvailable = initializeAdc();

  if (CALIBRATION_MODE) {
    WiFi.mode(WIFI_OFF);
    Serial.println(
        "[MODE] CALIBRACION LOCAL: WiFi y MQTT desactivados.");
    Serial.printf(
        "[GEN] Sine %.1f Hz | High=%.3f V | Low=%.3f V | Phase=0\n",
        CALIBRATION_FREQUENCY_HZ, CALIBRATION_HIGH_V,
        CALIBRATION_LOW_V);
  } else {
    startLocalWebServer();
  }
}

void loop() {
  if (!CALIBRATION_MODE) {
    webServer.handleClient();
  }

  const uint32_t beforeCapture = millis();
  if (!adcAvailable &&
      static_cast<uint32_t>(beforeCapture - lastAdcAttemptMs) >=
          ADC_RETRY_MS) {
    adcAvailable = initializeAdc();
  }

  Metrics currentMetrics{};
  Metrics rawCurrentMetrics{};
  Metrics voltageMetrics{};
  Metrics rawVoltageMetrics{};
  if (adcAvailable &&
      !captureWindow(Ads::INPMUX_IN1_IN0, INPUT_CAL_GAIN,
                     INPUT_CAL_OFFSET_V, currentMetrics,
                     rawCurrentMetrics)) {
    Serial.println("[ADC] Current capture failed; entering degraded mode.");
    adcAvailable = false;
    lastAdcAttemptMs = millis();
  }
  if (adcAvailable && !CALIBRATION_MODE &&
      !captureWindow(Ads::INPMUX_IN3_IN2, 1.0f, 0.0f,
                     voltageMetrics, rawVoltageMetrics)) {
    Serial.println("[ADC] Voltage capture failed; entering degraded mode.");
    adcAvailable = false;
    lastAdcAttemptMs = millis();
  }
  if (!CALIBRATION_MODE) {
    webServer.handleClient();
  }

  const uint32_t now = millis();
  const uint32_t interval = CALIBRATION_MODE
                                ? CALIBRATION_REPORT_INTERVAL_MS
                                : PUBLISH_INTERVAL_MS;
  if (static_cast<uint32_t>(now - lastPublishMs) >= interval) {
    lastPublishMs = now;
    if (CALIBRATION_MODE) {
      printCalibrationReport(rawCurrentMetrics, currentMetrics);
    } else {
      buildAndQueuePayload(currentMetrics, rawCurrentMetrics,
                           voltageMetrics, rawVoltageMetrics,
                           adcAvailable);
    }
  }

  if (!CALIBRATION_MODE) {
    webServer.handleClient();
  }
  delay(25);
}
