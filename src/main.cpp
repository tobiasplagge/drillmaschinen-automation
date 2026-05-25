#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_heap_caps.h>

// ---------------------------------------------------------------------------
// Waveshare ESP32-S3-POE-ETH-8DI-8DO configuration
// ---------------------------------------------------------------------------
static const char *AP_SSID = "Drillmaschine-M01";
static const char *AP_PASSWORD = "12345678";

static const char *DEVICE_ID = "waveshare-8di8do-01";
static const char *DEVICE_NAME = "Drillmaschinenüberwachung";
static const char *FIRMWARE_VERSION = "1.3.1";

static constexpr uint8_t CHANNEL_COUNT = 8;
static constexpr uint8_t CHANNEL_NAME_LENGTH = 24;
static constexpr uint8_t CROP_NAME_LENGTH = 24;
static constexpr uint32_t MAIN_SIGNAL_HOLD_MS = 1500;
static constexpr uint32_t GPS_LOG_INTERVAL_MS = 3000;
static constexpr uint32_t GNSS_POLL_INTERVAL_MS = 1000;
static constexpr uint16_t GPS_LOG_TARGET_CAPACITY = 5000;
static constexpr uint16_t MAIN_EVENT_LOG_CAPACITY = 512;
static constexpr uint16_t SENSOR_EVENT_LOG_CAPACITY = 512;

// Waveshare wiki: DI1..DI8 are GPIO4..GPIO11.
static constexpr uint8_t DI_PINS[CHANNEL_COUNT] = {4, 5, 6, 7, 8, 9, 10, 11};

// Waveshare TCA9554PWR expander for DO1..DO8.
static constexpr uint8_t I2C_SDA_PIN = 42;
static constexpr uint8_t I2C_SCL_PIN = 41;
static constexpr uint8_t TCA9554_ADDRESS = 0x20;
static constexpr uint8_t TCA9554_INPUT_REG = 0x00;
static constexpr uint8_t TCA9554_OUTPUT_REG = 0x01;
static constexpr uint8_t TCA9554_CONFIG_REG = 0x03;

// Ebyte EWD108-GN05(485) factory defaults according to the manufacturer:
// Modbus RTU slave address 1, 9600 baud, 8N1. The module exposes GNSS data via
// holding registers and can store an NMEA RMC ASCII sentence in registers. The
// exact register offset can differ by firmware; keep the scan window here easy
// to adjust after the first real-module test.
static constexpr uint8_t GNSS_RS485_RX_PIN = 18;
static constexpr uint8_t GNSS_RS485_TX_PIN = 17;
static constexpr uint8_t GNSS_RS485_DE_RE_PIN = 21;
static constexpr uint8_t GNSS_MODBUS_ADDRESS = 1;
static constexpr uint32_t GNSS_MODBUS_BAUD = 9600;
static constexpr uint16_t GNSS_SCAN_START_REGISTER = 0x0000;
static constexpr uint16_t GNSS_SCAN_REGISTER_COUNT = 96;
static constexpr uint8_t GNSS_MODBUS_FUNCTION_READ_HOLDING = 0x03;

// On this board the digital outputs are controlled through the TCA9554. The
// Waveshare examples initialize all outputs HIGH. Keep this configurable in
// case your wiring expects the opposite logic.
static constexpr bool DO_ACTIVE_HIGH = true;
static constexpr bool INPUT_ACTIVE_HIGH = false;
static constexpr bool MIRROR_RED_TO_OUTPUT = true;

WebServer server(80);
Preferences preferences;
HardwareSerial gnssSerial(1);

struct ChannelState {
  bool inputRaw = false;
  bool active = false;
  bool mainSignal = false;
  bool output = false;
  const char *status = "none";
  uint32_t changes = 0;
  uint32_t lastChangeMs = 0;
  uint32_t activeSinceMs = 0;
  uint32_t mainSignalChanges = 0;
  uint32_t lastMainSignalChangeMs = 0;
};

ChannelState channels[CHANNEL_COUNT];
struct GpsLogEntry {
  uint32_t uptimeMs = 0;
  double latitude = 0;
  double longitude = 0;
  float accuracyM = -1;
  float speedMps = -1;
  float headingDeg = -1;
  uint8_t satellites = 0;
  uint8_t liveMask = 0;
  uint8_t mainMask = 0;
};

struct GnssState {
  bool fix = false;
  bool seen = false;
  double latitude = 0;
  double longitude = 0;
  float accuracyM = -1;
  float speedMps = -1;
  float headingDeg = -1;
  uint8_t satellites = 0;
  uint32_t lastPollMs = 0;
  uint32_t lastFixMs = 0;
  uint32_t pollCount = 0;
  uint32_t okCount = 0;
  uint32_t errorCount = 0;
  char lastError[32] = "not_started";
  char lastSentence[128] = "";
  char rawPreview[96] = "";
};

struct MainSignalEvent {
  uint32_t uptimeMs = 0;
  uint8_t channel = 0;
  bool detected = false;
  bool hasGps = false;
  double latitude = 0;
  double longitude = 0;
  float accuracyM = -1;
  uint8_t liveMask = 0;
  uint8_t mainMask = 0;
  char crop[CROP_NAME_LENGTH] = "";
};

struct SensorTriggerEvent {
  uint32_t startUptimeMs = 0;
  uint32_t endUptimeMs = 0;
  uint32_t durationMs = 0;
  uint8_t channel = 0;
  bool startHasGps = false;
  bool endHasGps = false;
  double startLatitude = 0;
  double startLongitude = 0;
  double endLatitude = 0;
  double endLongitude = 0;
  float startAccuracyM = -1;
  float endAccuracyM = -1;
  uint8_t liveMaskAtStart = 0;
  uint8_t mainMaskAtStart = 0;
  uint8_t liveMaskAtEnd = 0;
  uint8_t mainMaskAtEnd = 0;
  char channelName[CHANNEL_NAME_LENGTH] = "";
  char crop[CROP_NAME_LENGTH] = "";
};

char channelNames[CHANNEL_COUNT][CHANNEL_NAME_LENGTH] = {
    "Kanal 1",
    "Kanal 2",
    "Kanal 3",
    "Kanal 4",
    "Kanal 5",
    "Kanal 6",
    "Kanal 7",
    "Kanal 8",
};
char cropName[CROP_NAME_LENGTH] = "Weizen";
uint8_t tcaOutputState = 0x00;
uint32_t lastDebugMs = 0;
GpsLogEntry *gpsLog = nullptr;
uint16_t gpsLogCapacity = 0;
uint16_t gpsLogHead = 0;
uint16_t gpsLogCount = 0;
uint32_t gpsLogTotal = 0;
uint32_t lastGpsLogMs = 0;
bool lastGpsValid = false;
double lastGpsLatitude = 0;
double lastGpsLongitude = 0;
float lastGpsAccuracyM = -1;
bool recordingActive = false;
GnssState gnss;
MainSignalEvent mainEventLog[MAIN_EVENT_LOG_CAPACITY];
uint16_t mainEventHead = 0;
uint16_t mainEventCount = 0;
uint32_t mainEventTotal = 0;
SensorTriggerEvent sensorEventLog[SENSOR_EVENT_LOG_CAPACITY];
uint16_t sensorEventHead = 0;
uint16_t sensorEventCount = 0;
uint32_t sensorEventTotal = 0;
uint32_t sensorActiveStartMs[CHANNEL_COUNT] = {};
bool sensorActiveStartHasGps[CHANNEL_COUNT] = {};
double sensorActiveStartLatitude[CHANNEL_COUNT] = {};
double sensorActiveStartLongitude[CHANNEL_COUNT] = {};
float sensorActiveStartAccuracyM[CHANNEL_COUNT] = {};
uint8_t sensorActiveStartLiveMask[CHANNEL_COUNT] = {};
uint8_t sensorActiveStartMainMask[CHANNEL_COUNT] = {};

uint8_t liveSignalMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    if (channels[i].active) {
      mask |= 1 << i;
    }
  }
  return mask;
}

uint8_t mainSignalMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    if (channels[i].mainSignal) {
      mask |= 1 << i;
    }
  }
  return mask;
}

void sanitizeChannelName(char *name) {
  name[CHANNEL_NAME_LENGTH - 1] = '\0';
  String cleaned = String(name);
  cleaned.trim();

  if (cleaned.length() == 0) {
    cleaned = "Kanal";
  }

  cleaned.replace("\"", "'");
  cleaned.replace("<", "");
  cleaned.replace(">", "");
  cleaned.toCharArray(name, CHANNEL_NAME_LENGTH);
}

void sanitizeCropName(char *name) {
  name[CROP_NAME_LENGTH - 1] = '\0';
  String cleaned = String(name);
  cleaned.trim();

  if (cleaned.length() == 0) {
    cleaned = "Unbekannt";
  }

  cleaned.replace("\"", "'");
  cleaned.replace("<", "");
  cleaned.replace(">", "");
  cleaned.toCharArray(name, CROP_NAME_LENGTH);
}

void setDefaultChannelName(uint8_t channelIndex) {
  snprintf(channelNames[channelIndex], CHANNEL_NAME_LENGTH, "Kanal %u", channelIndex + 1);
}

void loadChannelNames() {
  preferences.begin("channels", false);

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    char key[6];
    snprintf(key, sizeof(key), "ch%u", i + 1);

    String stored = preferences.getString(key, "");
    if (stored.length() > 0) {
      stored.toCharArray(channelNames[i], CHANNEL_NAME_LENGTH);
      sanitizeChannelName(channelNames[i]);
    } else {
      setDefaultChannelName(i);
    }
  }
}

void loadCropName() {
  String stored = preferences.getString("crop", "");
  if (stored.length() > 0) {
    stored.toCharArray(cropName, CROP_NAME_LENGTH);
    sanitizeCropName(cropName);
  } else {
    sanitizeCropName(cropName);
  }
}

bool saveChannelName(uint8_t channelIndex, const String &name) {
  if (channelIndex >= CHANNEL_COUNT) {
    return false;
  }

  name.toCharArray(channelNames[channelIndex], CHANNEL_NAME_LENGTH);
  sanitizeChannelName(channelNames[channelIndex]);

  char key[6];
  snprintf(key, sizeof(key), "ch%u", channelIndex + 1);
  preferences.putString(key, channelNames[channelIndex]);
  return true;
}

void saveCropName(const String &name) {
  name.toCharArray(cropName, CROP_NAME_LENGTH);
  sanitizeCropName(cropName);
  preferences.putString("crop", cropName);
}

void initGpsLog() {
  gpsLog = static_cast<GpsLogEntry *>(heap_caps_calloc(GPS_LOG_TARGET_CAPACITY, sizeof(GpsLogEntry), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (gpsLog != nullptr) {
    gpsLogCapacity = GPS_LOG_TARGET_CAPACITY;
    Serial.printf("GPS Log im PSRAM bereit: %u Punkte\n", gpsLogCapacity);
    return;
  }

  gpsLog = static_cast<GpsLogEntry *>(heap_caps_calloc(720, sizeof(GpsLogEntry), MALLOC_CAP_8BIT));
  if (gpsLog != nullptr) {
    gpsLogCapacity = 720;
    Serial.println("WARNUNG: GPS Log nutzt internes RAM, nur 720 Punkte");
    return;
  }

  gpsLogCapacity = 0;
  Serial.println("WARNUNG: GPS Log konnte nicht reserviert werden");
}

uint16_t gpsLogOldestIndex() {
  if (gpsLogCapacity == 0 || gpsLogCount < gpsLogCapacity) {
    return 0;
  }
  return gpsLogHead;
}

const GpsLogEntry &gpsLogAt(uint16_t orderedIndex) {
  const uint16_t start = gpsLogOldestIndex();
  return gpsLog[(start + orderedIndex) % gpsLogCapacity];
}

uint16_t mainEventOldestIndex() {
  if (mainEventCount < MAIN_EVENT_LOG_CAPACITY) {
    return 0;
  }
  return mainEventHead;
}

const MainSignalEvent &mainEventAt(uint16_t orderedIndex) {
  const uint16_t start = mainEventOldestIndex();
  return mainEventLog[(start + orderedIndex) % MAIN_EVENT_LOG_CAPACITY];
}

uint16_t sensorEventOldestIndex() {
  if (sensorEventCount < SENSOR_EVENT_LOG_CAPACITY) {
    return 0;
  }
  return sensorEventHead;
}

const SensorTriggerEvent &sensorEventAt(uint16_t orderedIndex) {
  const uint16_t start = sensorEventOldestIndex();
  return sensorEventLog[(start + orderedIndex) % SENSOR_EVENT_LOG_CAPACITY];
}

void appendGpsLog(double latitude, double longitude, float accuracyM, float speedMps, float headingDeg, uint8_t satellites) {
  if (gpsLog == nullptr || gpsLogCapacity == 0) {
    return;
  }

  GpsLogEntry &entry = gpsLog[gpsLogHead];
  entry.uptimeMs = millis();
  entry.latitude = latitude;
  entry.longitude = longitude;
  entry.accuracyM = accuracyM;
  entry.speedMps = speedMps;
  entry.headingDeg = headingDeg;
  entry.satellites = satellites;
  entry.liveMask = liveSignalMask();
  entry.mainMask = mainSignalMask();

  gpsLogHead = (gpsLogHead + 1) % gpsLogCapacity;
  if (gpsLogCount < gpsLogCapacity) {
    gpsLogCount++;
  }
  gpsLogTotal++;
  lastGpsLogMs = entry.uptimeMs;
  lastGpsValid = true;
  lastGpsLatitude = latitude;
  lastGpsLongitude = longitude;
  lastGpsAccuracyM = accuracyM;
}

uint16_t modbusCrc(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void setGnssError(const char *message) {
  strncpy(gnss.lastError, message, sizeof(gnss.lastError) - 1);
  gnss.lastError[sizeof(gnss.lastError) - 1] = '\0';
}

void initGnssRs485() {
  pinMode(GNSS_RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(GNSS_RS485_DE_RE_PIN, LOW);
  gnssSerial.begin(GNSS_MODBUS_BAUD, SERIAL_8N1, GNSS_RS485_RX_PIN, GNSS_RS485_TX_PIN);
  setGnssError("waiting");
  Serial.printf("EWD108-GN05(485) RS485: addr=%u baud=%lu RX=%u TX=%u DE/RE=%u\n",
                GNSS_MODBUS_ADDRESS,
                static_cast<unsigned long>(GNSS_MODBUS_BAUD),
                GNSS_RS485_RX_PIN,
                GNSS_RS485_TX_PIN,
                GNSS_RS485_DE_RE_PIN);
}

bool readModbusHoldingRegisters(uint16_t startRegister, uint16_t registerCount, uint8_t *payload, size_t payloadSize, uint8_t &byteCount) {
  if (registerCount == 0 || registerCount > 120 || payloadSize < static_cast<size_t>(registerCount * 2)) {
    setGnssError("bad_request");
    return false;
  }

  while (gnssSerial.available()) {
    gnssSerial.read();
  }

  uint8_t request[8];
  request[0] = GNSS_MODBUS_ADDRESS;
  request[1] = GNSS_MODBUS_FUNCTION_READ_HOLDING;
  request[2] = highByte(startRegister);
  request[3] = lowByte(startRegister);
  request[4] = highByte(registerCount);
  request[5] = lowByte(registerCount);
  const uint16_t crc = modbusCrc(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  digitalWrite(GNSS_RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(80);
  gnssSerial.write(request, sizeof(request));
  gnssSerial.flush();
  delayMicroseconds(120);
  digitalWrite(GNSS_RS485_DE_RE_PIN, LOW);

  const size_t expectedLength = 5 + static_cast<size_t>(registerCount * 2);
  uint8_t response[245];
  size_t index = 0;
  const uint32_t deadline = millis() + 350;
  while (millis() < deadline && index < expectedLength) {
    while (gnssSerial.available() && index < sizeof(response)) {
      response[index++] = static_cast<uint8_t>(gnssSerial.read());
    }
    delay(1);
  }

  if (index < 5) {
    setGnssError("timeout");
    return false;
  }

  const uint16_t receivedCrc = static_cast<uint16_t>(response[index - 2]) | (static_cast<uint16_t>(response[index - 1]) << 8);
  if (modbusCrc(response, index - 2) != receivedCrc) {
    setGnssError("crc");
    return false;
  }

  if (response[0] != GNSS_MODBUS_ADDRESS) {
    setGnssError("wrong_addr");
    return false;
  }

  if (response[1] & 0x80) {
    setGnssError("exception");
    return false;
  }

  if (response[1] != GNSS_MODBUS_FUNCTION_READ_HOLDING) {
    setGnssError("wrong_func");
    return false;
  }

  byteCount = response[2];
  if (byteCount > payloadSize || index < static_cast<size_t>(byteCount + 5)) {
    setGnssError("bad_len");
    return false;
  }

  memcpy(payload, response + 3, byteCount);
  setGnssError("ok");
  return true;
}

double parseNmeaCoordinate(const String &value, const String &hemisphere) {
  if (value.length() < 4) {
    return 0;
  }

  const int dotIndex = value.indexOf('.');
  const int degreeDigits = (dotIndex > 4) ? dotIndex - 2 : value.length() - 2;
  const double degrees = value.substring(0, degreeDigits).toDouble();
  const double minutes = value.substring(degreeDigits).toDouble();
  double decimal = degrees + (minutes / 60.0);
  if (hemisphere == "S" || hemisphere == "W") {
    decimal = -decimal;
  }
  return decimal;
}

String nmeaField(const String &sentence, uint8_t wantedField) {
  int start = 0;
  uint8_t field = 0;
  for (int i = 0; i <= sentence.length(); i++) {
    if (i == sentence.length() || sentence[i] == ',' || sentence[i] == '*') {
      if (field == wantedField) {
        return sentence.substring(start, i);
      }
      field++;
      start = i + 1;
    }
  }
  return "";
}

bool applyNmeaSentence(const String &sentence) {
  if (!sentence.startsWith("$") || sentence.length() < 10) {
    return false;
  }

  const String type = nmeaField(sentence, 0);
  if (type.endsWith("RMC")) {
    const String valid = nmeaField(sentence, 2);
    if (valid != "A") {
      gnss.fix = false;
      setGnssError("no_fix");
      return false;
    }

    const double lat = parseNmeaCoordinate(nmeaField(sentence, 3), nmeaField(sentence, 4));
    const double lon = parseNmeaCoordinate(nmeaField(sentence, 5), nmeaField(sentence, 6));
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || (lat == 0 && lon == 0)) {
      setGnssError("bad_nmea");
      return false;
    }

    gnss.fix = true;
    gnss.seen = true;
    gnss.latitude = lat;
    gnss.longitude = lon;
    gnss.speedMps = nmeaField(sentence, 7).toFloat() * 0.514444f;
    gnss.headingDeg = nmeaField(sentence, 8).length() > 0 ? nmeaField(sentence, 8).toFloat() : -1.0f;
    gnss.lastFixMs = millis();
    sentence.toCharArray(gnss.lastSentence, sizeof(gnss.lastSentence));
    setGnssError("ok");
    return true;
  }

  if (type.endsWith("GGA")) {
    const int fixQuality = nmeaField(sentence, 6).toInt();
    if (fixQuality <= 0) {
      gnss.fix = false;
      setGnssError("no_fix");
      return false;
    }

    const double lat = parseNmeaCoordinate(nmeaField(sentence, 2), nmeaField(sentence, 3));
    const double lon = parseNmeaCoordinate(nmeaField(sentence, 4), nmeaField(sentence, 5));
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || (lat == 0 && lon == 0)) {
      setGnssError("bad_nmea");
      return false;
    }

    gnss.fix = true;
    gnss.seen = true;
    gnss.latitude = lat;
    gnss.longitude = lon;
    gnss.satellites = static_cast<uint8_t>(constrain(nmeaField(sentence, 7).toInt(), 0, 255));
    gnss.accuracyM = nmeaField(sentence, 8).length() > 0 ? nmeaField(sentence, 8).toFloat() : -1.0f;
    gnss.lastFixMs = millis();
    sentence.toCharArray(gnss.lastSentence, sizeof(gnss.lastSentence));
    setGnssError("ok");
    return true;
  }

  return false;
}

bool scanAsciiForNmea(const char *ascii) {
  String text = String(ascii);
  int start = text.indexOf('$');
  while (start >= 0) {
    int end = text.indexOf('$', start + 1);
    if (end < 0) {
      end = text.length();
    }
    String sentence = text.substring(start, end);
    sentence.trim();
    if (applyNmeaSentence(sentence)) {
      return true;
    }
    start = text.indexOf('$', start + 1);
  }
  return false;
}

bool scanPayloadForNmea(const uint8_t *payload, uint8_t byteCount) {
  char ascii[GNSS_SCAN_REGISTER_COUNT * 2 + 1];
  const uint8_t copyLength = min<uint8_t>(byteCount, sizeof(ascii) - 1);
  for (uint8_t i = 0; i < copyLength; i++) {
    const char c = static_cast<char>(payload[i]);
    ascii[i] = (c >= 32 && c <= 126) ? c : ' ';
  }
  ascii[copyLength] = '\0';
  strncpy(gnss.rawPreview, ascii, sizeof(gnss.rawPreview) - 1);
  gnss.rawPreview[sizeof(gnss.rawPreview) - 1] = '\0';

  if (scanAsciiForNmea(ascii)) {
    return true;
  }

  // Some Modbus devices expose two ASCII bytes per register in swapped order.
  // Try that layout too before declaring the scan window invalid.
  char swapped[GNSS_SCAN_REGISTER_COUNT * 2 + 1];
  for (uint8_t i = 0; i < copyLength; i += 2) {
    if (i + 1 < copyLength) {
      swapped[i] = ascii[i + 1];
      swapped[i + 1] = ascii[i];
    } else {
      swapped[i] = ascii[i];
    }
  }
  swapped[copyLength] = '\0';
  if (scanAsciiForNmea(swapped)) {
    strncpy(gnss.rawPreview, swapped, sizeof(gnss.rawPreview) - 1);
    gnss.rawPreview[sizeof(gnss.rawPreview) - 1] = '\0';
    return true;
  }

  setGnssError("no_nmea");
  return false;
}

void pollGnss() {
  const uint32_t now = millis();
  if (now - gnss.lastPollMs < GNSS_POLL_INTERVAL_MS) {
    return;
  }
  gnss.lastPollMs = now;
  gnss.pollCount++;

  uint8_t payload[GNSS_SCAN_REGISTER_COUNT * 2] = {};
  uint8_t byteCount = 0;
  if (!readModbusHoldingRegisters(GNSS_SCAN_START_REGISTER, GNSS_SCAN_REGISTER_COUNT, payload, sizeof(payload), byteCount)) {
    gnss.errorCount++;
    return;
  }

  if (scanPayloadForNmea(payload, byteCount)) {
    gnss.okCount++;
  } else {
    gnss.errorCount++;
  }

  if (recordingActive && gnss.fix && now - lastGpsLogMs >= GPS_LOG_INTERVAL_MS) {
    appendGpsLog(gnss.latitude, gnss.longitude, gnss.accuracyM, gnss.speedMps, gnss.headingDeg, gnss.satellites);
  }
}

void appendMainSignalEvent(uint8_t channelIndex, bool detected) {
  MainSignalEvent &event = mainEventLog[mainEventHead];
  event.uptimeMs = millis();
  event.channel = channelIndex + 1;
  event.detected = detected;
  event.hasGps = gnss.fix;
  event.latitude = gnss.fix ? gnss.latitude : lastGpsLatitude;
  event.longitude = gnss.fix ? gnss.longitude : lastGpsLongitude;
  event.accuracyM = gnss.fix ? gnss.accuracyM : lastGpsAccuracyM;
  event.liveMask = liveSignalMask();
  event.mainMask = mainSignalMask();
  strncpy(event.crop, cropName, CROP_NAME_LENGTH);
  event.crop[CROP_NAME_LENGTH - 1] = '\0';

  mainEventHead = (mainEventHead + 1) % MAIN_EVENT_LOG_CAPACITY;
  if (mainEventCount < MAIN_EVENT_LOG_CAPACITY) {
    mainEventCount++;
  }
  mainEventTotal++;
}

void startSensorTriggerEvent(uint8_t channelIndex) {
  if (channelIndex >= CHANNEL_COUNT) {
    return;
  }

  sensorActiveStartMs[channelIndex] = millis();
  sensorActiveStartHasGps[channelIndex] = gnss.fix;
  sensorActiveStartLatitude[channelIndex] = gnss.fix ? gnss.latitude : lastGpsLatitude;
  sensorActiveStartLongitude[channelIndex] = gnss.fix ? gnss.longitude : lastGpsLongitude;
  sensorActiveStartAccuracyM[channelIndex] = gnss.fix ? gnss.accuracyM : lastGpsAccuracyM;
  sensorActiveStartLiveMask[channelIndex] = liveSignalMask();
  sensorActiveStartMainMask[channelIndex] = mainSignalMask();
}

void finishSensorTriggerEvent(uint8_t channelIndex) {
  if (channelIndex >= CHANNEL_COUNT || sensorActiveStartMs[channelIndex] == 0) {
    return;
  }

  const uint32_t now = millis();
  SensorTriggerEvent &event = sensorEventLog[sensorEventHead];
  event.startUptimeMs = sensorActiveStartMs[channelIndex];
  event.endUptimeMs = now;
  event.durationMs = now - sensorActiveStartMs[channelIndex];
  event.channel = channelIndex + 1;
  event.startHasGps = sensorActiveStartHasGps[channelIndex];
  event.endHasGps = gnss.fix;
  event.startLatitude = sensorActiveStartLatitude[channelIndex];
  event.startLongitude = sensorActiveStartLongitude[channelIndex];
  event.endLatitude = gnss.fix ? gnss.latitude : lastGpsLatitude;
  event.endLongitude = gnss.fix ? gnss.longitude : lastGpsLongitude;
  event.startAccuracyM = sensorActiveStartAccuracyM[channelIndex];
  event.endAccuracyM = gnss.fix ? gnss.accuracyM : lastGpsAccuracyM;
  event.liveMaskAtStart = sensorActiveStartLiveMask[channelIndex];
  event.mainMaskAtStart = sensorActiveStartMainMask[channelIndex];
  event.liveMaskAtEnd = liveSignalMask();
  event.mainMaskAtEnd = mainSignalMask();
  strncpy(event.channelName, channelNames[channelIndex], CHANNEL_NAME_LENGTH);
  event.channelName[CHANNEL_NAME_LENGTH - 1] = '\0';
  strncpy(event.crop, cropName, CROP_NAME_LENGTH);
  event.crop[CROP_NAME_LENGTH - 1] = '\0';

  sensorEventHead = (sensorEventHead + 1) % SENSOR_EVENT_LOG_CAPACITY;
  if (sensorEventCount < SENSOR_EVENT_LOG_CAPACITY) {
    sensorEventCount++;
  }
  sensorEventTotal++;

  sensorActiveStartMs[channelIndex] = 0;
  sensorActiveStartHasGps[channelIndex] = false;
}

void clearGpsLog() {
  gpsLogHead = 0;
  gpsLogCount = 0;
  gpsLogTotal = 0;
  lastGpsLogMs = 0;
  lastGpsValid = false;
  mainEventHead = 0;
  mainEventCount = 0;
  mainEventTotal = 0;
  sensorEventHead = 0;
  sensorEventCount = 0;
  sensorEventTotal = 0;
  memset(sensorActiveStartMs, 0, sizeof(sensorActiveStartMs));
}

bool tcaWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool tcaRead(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(TCA9554_ADDRESS, static_cast<uint8_t>(1)) != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool setDigitalOutput(uint8_t channelIndex, bool on) {
  if (channelIndex >= CHANNEL_COUNT) {
    return false;
  }

  const bool pinLevel = DO_ACTIVE_HIGH ? on : !on;
  const uint8_t mask = 1 << channelIndex;
  if (pinLevel) {
    tcaOutputState |= mask;
  } else {
    tcaOutputState &= ~mask;
  }

  const bool ok = tcaWrite(TCA9554_OUTPUT_REG, tcaOutputState);
  channels[channelIndex].output = on;
  return ok;
}

void initDigitalOutputs() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(20);

  tcaOutputState = DO_ACTIVE_HIGH ? 0x00 : 0xFF;
  tcaWrite(TCA9554_OUTPUT_REG, tcaOutputState);
  tcaWrite(TCA9554_CONFIG_REG, 0x00); // all 8 expander pins as outputs

  uint8_t readback = 0;
  if (tcaRead(TCA9554_OUTPUT_REG, readback)) {
    Serial.printf("TCA9554 DO expander ok, output register=0x%02X\n", readback);
  } else {
    Serial.println("WARNUNG: TCA9554 DO expander nicht erreichbar");
  }
}

void initDigitalInputs() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    pinMode(DI_PINS[i], INPUT_PULLUP);
    channels[i].inputRaw = digitalRead(DI_PINS[i]) == HIGH;
    channels[i].active = INPUT_ACTIVE_HIGH ? channels[i].inputRaw : !channels[i].inputRaw;
    channels[i].activeSinceMs = channels[i].active ? millis() : 0;
    channels[i].mainSignal = false;
    channels[i].status = "none";
  }
}

void readDigitalInputs() {
  const uint32_t now = millis();

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    const bool raw = digitalRead(DI_PINS[i]) == HIGH;
    const bool active = INPUT_ACTIVE_HIGH ? raw : !raw;

    if (active != channels[i].active) {
      channels[i].changes++;
      channels[i].lastChangeMs = now;
      channels[i].activeSinceMs = active ? now : 0;
    }

    channels[i].inputRaw = raw;
    channels[i].active = active;

    const bool mainSignal = active && channels[i].activeSinceMs > 0 && (now - channels[i].activeSinceMs >= MAIN_SIGNAL_HOLD_MS);
    if (mainSignal != channels[i].mainSignal) {
      channels[i].mainSignal = mainSignal;
      channels[i].mainSignalChanges++;
      channels[i].lastMainSignalChangeMs = now;
      appendMainSignalEvent(i, mainSignal);
      if (mainSignal) {
        startSensorTriggerEvent(i);
      } else {
        finishSensorTriggerEvent(i);
      }
    } else {
      channels[i].mainSignal = mainSignal;
    }
    channels[i].status = mainSignal ? "red" : "none";

    const bool outputOn = MIRROR_RED_TO_OUTPUT && mainSignal;
    if (channels[i].output != outputOn) {
      setDigitalOutput(i, outputOn);
    }
  }
}

String statusJson() {
  JsonDocument doc;
  doc["device_id"] = DEVICE_ID;
  doc["device_name"] = DEVICE_NAME;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["esp_temperature_c"] = temperatureRead();
  doc["crop_name"] = cropName;
  doc["uptime_ms"] = millis();
  doc["wifi_ap_ssid"] = AP_SSID;
  doc["ip"] = WiFi.softAPIP().toString();
  doc["web_url"] = "http://" + WiFi.softAPIP().toString() + "/";
  doc["main_signal_hold_ms"] = MAIN_SIGNAL_HOLD_MS;
  doc["gps_log_interval_ms"] = GPS_LOG_INTERVAL_MS;
  doc["recording_active"] = recordingActive;
  doc["gps_log_count"] = gpsLogCount;
  doc["gps_log_total"] = gpsLogTotal;
  doc["gps_log_capacity"] = gpsLogCapacity;
  doc["last_gps_log_age_ms"] = lastGpsLogMs > 0 ? static_cast<int32_t>(millis() - lastGpsLogMs) : -1;
  doc["main_event_count"] = mainEventCount;
  doc["main_event_total"] = mainEventTotal;
  doc["main_event_capacity"] = MAIN_EVENT_LOG_CAPACITY;
  doc["sensor_event_count"] = sensorEventCount;
  doc["sensor_event_total"] = sensorEventTotal;
  doc["sensor_event_capacity"] = SENSOR_EVENT_LOG_CAPACITY;
  doc["last_gps_valid"] = lastGpsValid;

  JsonObject gnssJson = doc["gnss"].to<JsonObject>();
  gnssJson["source"] = "EWD108-GN05(485)";
  gnssJson["interface"] = "RS485 Modbus RTU";
  gnssJson["modbus_address"] = GNSS_MODBUS_ADDRESS;
  gnssJson["baud"] = GNSS_MODBUS_BAUD;
  gnssJson["fix"] = gnss.fix;
  gnssJson["seen"] = gnss.seen;
  gnssJson["latitude"] = gnss.latitude;
  gnssJson["longitude"] = gnss.longitude;
  gnssJson["accuracy_m"] = gnss.accuracyM;
  gnssJson["speed_mps"] = gnss.speedMps;
  gnssJson["heading_deg"] = gnss.headingDeg;
  gnssJson["satellites"] = gnss.satellites;
  gnssJson["last_fix_age_ms"] = gnss.lastFixMs > 0 ? static_cast<int32_t>(millis() - gnss.lastFixMs) : -1;
  gnssJson["poll_count"] = gnss.pollCount;
  gnssJson["ok_count"] = gnss.okCount;
  gnssJson["error_count"] = gnss.errorCount;
  gnssJson["last_error"] = gnss.lastError;
  gnssJson["last_sentence"] = gnss.lastSentence;
  gnssJson["raw_preview"] = gnss.rawPreview;

  JsonArray array = doc["channels"].to<JsonArray>();
  const uint32_t now = millis();
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    JsonObject ch = array.add<JsonObject>();
    ch["channel"] = i + 1;
    ch["name"] = channelNames[i];
    ch["di_gpio"] = DI_PINS[i];
    ch["input_raw"] = channels[i].inputRaw;
    ch["active"] = channels[i].active;
    ch["live_active"] = channels[i].active;
    ch["main_signal"] = channels[i].mainSignal;
    ch["status"] = channels[i].status;
    ch["output"] = channels[i].output;
    ch["changes"] = channels[i].changes;
    ch["active_ms"] = channels[i].activeSinceMs > 0 && channels[i].active ? static_cast<int32_t>(now - channels[i].activeSinceMs) : 0;
    ch["main_signal_changes"] = channels[i].mainSignalChanges;
    ch["last_change_age_ms"] = channels[i].lastChangeMs > 0 ? static_cast<int32_t>(now - channels[i].lastChangeMs) : -1;
    ch["last_main_signal_change_age_ms"] = channels[i].lastMainSignalChangeMs > 0 ? static_cast<int32_t>(now - channels[i].lastMainSignalChangeMs) : -1;
  }

  String json;
  serializeJson(doc, json);
  return json;
}

String htmlPage() {
  return R"HTML(
<!doctype html>
<html lang="de">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drillmaschinenüberwachung</title>
  <style>
    :root { color-scheme: light dark; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    body { margin: 0; background: #111827; color: #f9fafb; }
    main { width: min(1100px, calc(100% - 28px)); margin: 0 auto; padding: 24px 0 34px; }
    h1 { margin: 0 0 8px; font-size: clamp(1.55rem, 5vw, 2.25rem); }
    .meta { color: #9ca3af; margin-bottom: 18px; }
    .connection { display: flex; flex-wrap: wrap; gap: 8px 14px; align-items: center; margin-bottom: 14px; color: #d1d5db; font-size: .94rem; }
    .connection strong { color: #f9fafb; }
    .connection-dot { width: 13px; height: 13px; border-radius: 50%; background: #22c55e; box-shadow: 0 0 10px #22c55e; display: inline-block; margin-right: 6px; vertical-align: -1px; }
    .connection.offline .connection-dot { background: #ef4444; box-shadow: 0 0 12px #ef4444; }
    .connection.stale .connection-dot { background: #facc15; box-shadow: 0 0 10px #facc15; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }
    .card { border: 1px solid #374151; border-radius: 8px; background: #1f2937; padding: 14px; }
    .top { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
    .name { min-width: 0; font-weight: 800; font-size: 1.08rem; overflow-wrap: anywhere; }
    .channel { color: #9ca3af; font-size: .85rem; margin-top: 2px; }
    .main-state { display: flex; align-items: center; gap: 10px; }
    .dot { width: 40px; height: 40px; border-radius: 50%; background: #facc15; box-shadow: 0 0 14px #facc15; flex: 0 0 auto; }
    .dot.red { background: #ef4444; box-shadow: 0 0 20px #ef4444; }
    .dot.green { background: #22c55e; box-shadow: 0 0 20px #22c55e; }
    .dot.yellow { background: #facc15; box-shadow: 0 0 16px #facc15; }
    .dot.none { background: #facc15; }
    .pill { display: inline-block; min-width: 44px; padding: 2px 7px; border-radius: 999px; background: #374151; color: #d1d5db; font-size: .82rem; font-weight: 800; text-align: center; }
    .pill.on { background: #16a34a; color: #f0fdf4; }
    .pill.main { background: #dc2626; color: #fef2f2; }
    .pill.ok { background: #16a34a; color: #f0fdf4; }
    .pill.pending { background: #ca8a04; color: #fffbeb; }
    .panel { border: 1px solid #374151; border-radius: 8px; background: #1f2937; padding: 14px; margin-bottom: 14px; }
    .panel h2 { margin: 0 0 10px; font-size: 1rem; }
    .actions { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
    .field-row { display: grid; grid-template-columns: minmax(130px, 1fr) auto; gap: 8px; margin-bottom: 12px; }
    input, select { min-width: 0; height: 40px; border-radius: 6px; border: 1px solid #4b5563; background: #111827; color: #f9fafb; padding: 0 10px; font: inherit; }
    button, .link-button { border: 0; border-radius: 6px; background: #2563eb; color: white; padding: 9px 12px; font: inherit; font-weight: 750; text-decoration: none; cursor: pointer; }
    button.secondary, .link-button.secondary { background: #374151; }
    button.danger { background: #991b1b; }
    button:disabled { opacity: .45; cursor: default; }
    .gps-meta { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 8px 14px; margin-top: 12px; color: #d1d5db; font-size: .92rem; }
    .gps-meta strong { color: #f9fafb; }
    details { margin-top: 10px; border-top: 1px solid #374151; padding-top: 10px; }
    summary { color: #d1d5db; cursor: pointer; font-weight: 700; }
    dl { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 10px; margin: 0; font-size: .92rem; }
    dt { color: #9ca3af; }
    dd { margin: 0; text-align: right; font-weight: 650; overflow-wrap: anywhere; }
    .error { min-height: 1.4em; color: #fca5a5; margin-top: 14px; }
  </style>
</head>
<body>
  <main>
    <h1 id="title">Drillmaschinenüberwachung</h1>
    <div class="meta">
      <span id="ip">IP: -</span> · <span id="version">Version: -</span> · <span id="updated">-</span>
    </div>
    <div id="connection" class="connection">
      <span><span class="connection-dot"></span><strong id="connectionState">Verbunden</strong></span>
      <span>Letzter Kontakt: <strong id="lastContact">-</strong></span>
      <span>ESP Temperatur: <strong id="espTemp">-</strong></span>
    </div>
    <section class="panel">
      <h2>Fahrtaufzeichnung</h2>
      <div class="actions">
        <button id="gpsStart" type="button">Aufzeichnen</button>
        <button id="gpsStop" class="secondary" type="button" disabled>Aufzeichnung Stop</button>
        <button id="alarmEnable" class="secondary" type="button">Ton aktivieren</button>
        <button id="alarmAck" class="secondary" type="button">Alarm quittieren</button>
        <a class="link-button secondary" href="/api/gps-log.csv">CSV</a>
        <a class="link-button secondary" href="/api/gps-log.geojson">GeoJSON</a>
        <a class="link-button secondary" href="/api/main-events.csv">Hauptsignale CSV</a>
        <a class="link-button secondary" href="/api/main-events.geojson">Hauptsignale GeoJSON</a>
        <a class="link-button secondary" href="/api/sensor-events.csv">Sensorlog CSV</a>
        <a class="link-button secondary" href="/api/sensor-events.txt">Sensorlog TXT</a>
        <button id="gpsClear" class="danger" type="button">Log löschen</button>
      </div>
      <div class="gps-meta">
        <div>Aufzeichnung: <strong id="gpsStatus">Aus</strong></div>
        <div>GNSS Fix: <strong id="gnssFix">-</strong></div>
        <div>GNSS Quelle: <strong id="gnssSource">-</strong></div>
        <div>Alarm: <strong id="alarmStatus">Aus</strong></div>
        <div>Punkte: <strong id="gpsCount">0</strong></div>
        <div>Hauptsignal-Log: <strong id="mainEventCount">0</strong></div>
        <div>Sensorlog: <strong id="sensorEventCount">0</strong></div>
        <div>Letzter Punkt: <strong id="gpsLast">-</strong></div>
        <div>Position: <strong id="gpsPosition">-</strong></div>
        <div>Genauigkeit: <strong id="gpsAccuracy">-</strong></div>
        <div>Satelliten: <strong id="gpsSatellites">-</strong></div>
        <div>RS485: <strong id="gnssRs485">-</strong></div>
      </div>
    </section>
    <section class="panel">
      <h2>Saat</h2>
      <div class="field-row">
        <input id="cropInput" list="cropSuggestions" maxlength="23" value="Weizen">
        <datalist id="cropSuggestions">
          <option value="Weizen"></option>
          <option value="Gerste"></option>
          <option value="Raps"></option>
          <option value="Senf"></option>
          <option value="Mais"></option>
          <option value="Gras"></option>
          <option value="Kleegras"></option>
          <option value="Zwischenfrucht"></option>
        </datalist>
        <button id="cropSave" type="button">Speichern</button>
      </div>
      <div class="gps-meta">
        <div>Aktuell: <strong id="cropCurrent">-</strong></div>
      </div>
    </section>
    <div id="grid" class="grid"></div>
    <div id="error" class="error"></div>
  </main>
  <script>
    let actionBusy = false;
    let refreshBusy = false;
    let pageActive = true;
    let alarmEnabled = false;
    let audioContext = null;
    let alarmOscillator = null;
    let alarmGain = null;
    let alarmPulseTimer = null;
    let alarmPulsing = false;
    let knownMainSignals = new Map();
    let mainSignalSnapshotReady = false;
    let acknowledgedMainMask = 0;
    let currentMainMask = 0;
    let lastSuccessfulContactMs = 0;
    let connectionTimer = null;
    let renderingGrid = false;
    const openDetailChannels = new Set();

    function escapeHtml(value) {
      return String(value ?? '').replace(/[&<>"']/g, c => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
      }[c]));
    }

    function card(ch) {
      const name = escapeHtml(ch.name || ('Kanal ' + ch.channel));
      const detailsOpen = openDetailChannels.has(String(ch.channel)) ? ' open' : '';
      const view = ch.main_signal
        ? { label: 'Erkannt', pill: 'main', dot: 'red' }
        : (ch.live_active
          ? { label: 'Kein Status', pill: 'pending', dot: 'yellow' }
          : { label: 'OK', pill: 'ok', dot: 'green' });
      return `
        <div class="card" data-channel="${ch.channel}">
          <div class="top">
            <div>
              <div class="name">${name}</div>
              <div class="channel">Kanal ${ch.channel}</div>
            </div>
            <div class="main-state">
              <span class="pill ${view.pill}">${view.label}</span>
              <div class="dot ${view.dot}"></div>
            </div>
          </div>
          <details${detailsOpen}>
            <summary>Details</summary>
            <dl>
              <dt>Live Signal</dt><dd><span class="pill ${ch.live_active ? 'on' : ''}">${ch.live_active ? 'JA' : 'NEIN'}</span></dd>
              <dt>Live Zeit</dt><dd>${ch.active_ms || 0} ms</dd>
              <dt>DI Rohwert</dt><dd>${ch.input_raw ? 'HIGH' : 'LOW'}</dd>
              <dt>DI GPIO</dt><dd>${ch.di_gpio}</dd>
              <dt>Output</dt><dd>${ch.output ? 'Ein' : 'Aus'}</dd>
              <dt>Live Wechsel</dt><dd>${ch.changes}</dd>
              <dt>Haupt Wechsel</dt><dd>${ch.main_signal_changes}</dd>
            </dl>
          </details>
        </div>`;
    }

    function playConfirmTone() {
      if (!alarmEnabled || !audioContext) return;
      const now = audioContext.currentTime;
      const osc = audioContext.createOscillator();
      const gain = audioContext.createGain();
      osc.type = 'square';
      osc.frequency.setValueAtTime(880, now);
      gain.gain.setValueAtTime(0.0001, now);
      gain.gain.exponentialRampToValueAtTime(0.2, now + 0.02);
      gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.22);
      osc.connect(gain);
      gain.connect(audioContext.destination);
      osc.start(now);
      osc.stop(now + 0.24);
    }

    function setAlarmPulseVolume(on) {
      if (!alarmGain || !audioContext) return;
      const now = audioContext.currentTime;
      alarmGain.gain.cancelScheduledValues(now);
      alarmGain.gain.setValueAtTime(alarmGain.gain.value || 0.0001, now);
      alarmGain.gain.exponentialRampToValueAtTime(on ? 0.22 : 0.0001, now + 0.05);
    }

    function startPulsingAlarm() {
      if (!alarmEnabled || !audioContext || alarmPulsing) return;
      alarmPulsing = true;
      alarmOscillator = audioContext.createOscillator();
      alarmGain = audioContext.createGain();
      alarmOscillator.type = 'sawtooth';
      alarmOscillator.frequency.setValueAtTime(620, audioContext.currentTime);
      alarmGain.gain.setValueAtTime(0.0001, audioContext.currentTime);
      alarmOscillator.connect(alarmGain);
      alarmGain.connect(audioContext.destination);
      alarmOscillator.start();

      let on = false;
      setAlarmPulseVolume(true);
      alarmPulseTimer = setInterval(() => {
        on = !on;
        setAlarmPulseVolume(on);
      }, 450);
    }

    function stopPulsingAlarm() {
      if (!alarmPulsing) return;
      alarmPulsing = false;
      if (alarmPulseTimer) {
        clearInterval(alarmPulseTimer);
        alarmPulseTimer = null;
      }
      if (alarmGain && audioContext) {
        setAlarmPulseVolume(false);
      }
      const osc = alarmOscillator;
      setTimeout(() => {
        try { if (osc) osc.stop(); } catch (err) {}
      }, 90);
      alarmOscillator = null;
      alarmGain = null;
    }

    function updateAlarmStatus() {
      const target = document.getElementById('alarmStatus');
      if (!alarmEnabled) {
        target.textContent = 'Nicht aktiviert';
      } else if (alarmPulsing) {
        target.textContent = 'Aktiv';
      } else if (currentMainMask !== 0 && currentMainMask === acknowledgedMainMask) {
        target.textContent = 'Quittiert';
      } else {
        target.textContent = 'Bereit';
      }
    }

    async function enableAlarm() {
      audioContext = audioContext || new (window.AudioContext || window.webkitAudioContext)();
      if (audioContext.state === 'suspended') {
        await audioContext.resume();
      }
      alarmEnabled = true;
      document.getElementById('alarmEnable').textContent = 'Ton aktiv';
      document.getElementById('alarmEnable').disabled = true;
      playConfirmTone();
      updateAlarmStatus();
    }

    function acknowledgeAlarm() {
      acknowledgedMainMask = currentMainMask;
      stopPulsingAlarm();
      updateAlarmStatus();
    }

    function checkMainSignalAlarms(channels) {
      let mask = 0;
      channels.forEach(ch => {
        const previous = knownMainSignals.get(ch.channel);
        const current = Boolean(ch.main_signal);
        if (current) {
          mask |= 1 << (ch.channel - 1);
        }
        knownMainSignals.set(ch.channel, current);
      });
      currentMainMask = mask;
      mainSignalSnapshotReady = true;
      if (currentMainMask === 0) {
        acknowledgedMainMask = 0;
        stopPulsingAlarm();
      } else if ((currentMainMask & ~acknowledgedMainMask) !== 0) {
        startPulsingAlarm();
      } else {
        stopPulsingAlarm();
      }
      updateAlarmStatus();
    }

    function setConnectionState(state, detail) {
      const box = document.getElementById('connection');
      const label = document.getElementById('connectionState');
      box.classList.remove('offline', 'stale');
      if (state === 'offline') {
        box.classList.add('offline');
        label.textContent = 'Offline';
      } else if (state === 'stale') {
        box.classList.add('stale');
        label.textContent = 'Unsicher';
      } else {
        label.textContent = 'Verbunden';
      }
      if (detail) {
        label.textContent += ' - ' + detail;
      }
    }

    function updateLastContact() {
      const target = document.getElementById('lastContact');
      if (!lastSuccessfulContactMs) {
        target.textContent = '-';
        return;
      }
      const ageSeconds = Math.round((Date.now() - lastSuccessfulContactMs) / 1000);
      target.textContent = ageSeconds + ' s';
      if (ageSeconds > 10) {
        setConnectionState('offline');
      } else if (ageSeconds > 4) {
        setConnectionState('stale');
      }
    }

    async function fetchWithTimeout(url, options = {}, timeoutMs = 2500) {
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), timeoutMs);
      try {
        return await fetch(url, {
          cache: 'no-store',
          ...options,
          signal: controller.signal
        });
      } finally {
        clearTimeout(timer);
      }
    }

    async function refresh() {
      if (refreshBusy || !pageActive) return;
      refreshBusy = true;
      try {
        const res = await fetchWithTimeout('/api/status');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        lastSuccessfulContactMs = Date.now();
        setConnectionState('online');
        updateLastContact();
        document.getElementById('title').textContent = data.device_name || data.device_id;
        document.getElementById('ip').textContent = 'IP: ' + data.ip;
        document.getElementById('version').textContent = 'Version: ' + (data.firmware_version || '-');
        document.getElementById('updated').textContent = 'Aktualisiert: ' + new Date().toLocaleTimeString();
        document.getElementById('espTemp').textContent = Number.isFinite(data.esp_temperature_c) ? data.esp_temperature_c.toFixed(1) + ' °C' : '-';
        document.getElementById('cropCurrent').textContent = data.crop_name || '-';
        if (document.activeElement !== document.getElementById('cropInput')) {
          document.getElementById('cropInput').value = data.crop_name || '';
        }
        document.getElementById('gpsCount').textContent = `${data.gps_log_count || 0} / ${data.gps_log_capacity || 0}`;
        document.getElementById('mainEventCount').textContent = `${data.main_event_count || 0} / ${data.main_event_capacity || 0}`;
        document.getElementById('sensorEventCount').textContent = `${data.sensor_event_count || 0} / ${data.sensor_event_capacity || 0}`;
        document.getElementById('gpsLast').textContent = data.last_gps_log_age_ms >= 0 ? data.last_gps_log_age_ms + ' ms' : '-';
        document.getElementById('gpsStatus').textContent = data.recording_active ? 'Aktiv' : 'Aus';
        document.getElementById('gpsStart').disabled = Boolean(data.recording_active);
        document.getElementById('gpsStop').disabled = !data.recording_active;
        const gnss = data.gnss || {};
        document.getElementById('gnssFix').textContent = gnss.fix ? 'Ja' : (gnss.seen ? 'Nein' : 'Nicht empfangen');
        document.getElementById('gnssSource').textContent = gnss.source || '-';
        document.getElementById('gpsPosition').textContent = gnss.fix ? `${Number(gnss.latitude).toFixed(6)}, ${Number(gnss.longitude).toFixed(6)}` : '-';
        document.getElementById('gpsAccuracy').textContent = Number.isFinite(gnss.accuracy_m) && gnss.accuracy_m >= 0 ? `${Number(gnss.accuracy_m).toFixed(1)} m` : '-';
        document.getElementById('gpsSatellites').textContent = gnss.satellites ?? '-';
        document.getElementById('gnssRs485').textContent = `${gnss.last_error || '-'} · OK ${gnss.ok_count || 0} / Fehler ${gnss.error_count || 0}`;
        checkMainSignalAlarms(data.channels || []);
        const grid = document.getElementById('grid');
        renderingGrid = true;
        grid.innerHTML = data.channels.map(card).join('');
        setTimeout(() => { renderingGrid = false; }, 80);
        document.getElementById('error').textContent = '';
      } catch (err) {
        setConnectionState('offline', err.message || 'Keine API');
        updateLastContact();
        document.getElementById('error').textContent = 'Keine Verbindung zur API';
      } finally {
        refreshBusy = false;
      }
    }

    async function setRecording(active) {
      if (actionBusy) return;
      actionBusy = true;
      try {
        const res = await fetchWithTimeout('/api/recording', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ active })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Schalten fehlgeschlagen';
      } finally {
        actionBusy = false;
      }
    }

    function startGps() {
      setRecording(true);
    }

    function stopGps() {
      setRecording(false);
    }

    async function clearGpsLog() {
      try {
        const res = await fetchWithTimeout('/api/gps-log/clear', { method: 'POST' });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Löschen fehlgeschlagen';
      }
    }

    async function saveCrop() {
      const input = document.getElementById('cropInput');
      try {
        const res = await fetchWithTimeout('/api/crop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ crop_name: input.value })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Saat speichern fehlgeschlagen';
      }
    }

    document.getElementById('gpsStart').addEventListener('click', startGps);
    document.getElementById('gpsStop').addEventListener('click', stopGps);
    document.getElementById('gpsClear').addEventListener('click', clearGpsLog);
    document.getElementById('cropSave').addEventListener('click', saveCrop);
    document.getElementById('alarmEnable').addEventListener('click', enableAlarm);
    document.getElementById('alarmAck').addEventListener('click', acknowledgeAlarm);
    window.addEventListener('pagehide', () => {
      pageActive = false;
    });
    document.getElementById('grid').addEventListener('click', event => {
      if (event.target.tagName !== 'SUMMARY') return;
      const details = event.target.closest('details');
      const channel = event.target.closest('.card')?.dataset.channel;
      if (!details || !channel) return;
      setTimeout(() => {
        if (details.open) {
          openDetailChannels.add(channel);
        } else {
          openDetailChannels.delete(channel);
        }
      }, 0);
    });
    document.getElementById('grid').addEventListener('toggle', event => {
      if (event.target.tagName !== 'DETAILS') return;
      if (renderingGrid) return;
      const channel = event.target.closest('.card')?.dataset.channel;
      if (!channel) return;
      if (event.target.open) {
        openDetailChannels.add(channel);
      } else {
        openDetailChannels.delete(channel);
      }
    }, true);
    refresh();
    connectionTimer = setInterval(updateLastContact, 1000);
    setInterval(refresh, 1000);
  </script>
</body>
</html>
)HTML";
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", htmlPage());
}

void handleApiStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", statusJson());
}

void handleApiChannelName() {
  const String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"missing_body\"}");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  const int channel = doc["channel"] | 0;
  const String name = doc["name"] | "";

  if (channel < 1 || channel > CHANNEL_COUNT || name.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"invalid_channel_or_name\"}");
    return;
  }

  saveChannelName(static_cast<uint8_t>(channel - 1), name);

  JsonDocument response;
  response["ok"] = true;
  response["channel"] = channel;
  response["name"] = channelNames[channel - 1];

  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
}

void handleApiCrop() {
  const String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"missing_body\"}");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  const String name = doc["crop_name"] | "";
  if (name.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"invalid_crop_name\"}");
    return;
  }

  saveCropName(name);

  JsonDocument response;
  response["ok"] = true;
  response["crop_name"] = cropName;

  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
}

void handleApiRecording() {
  const String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"missing_body\"}");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  recordingActive = doc["active"] | false;

  JsonDocument response;
  response["ok"] = true;
  response["recording_active"] = recordingActive;
  response["gnss_fix"] = gnss.fix;

  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
}

void handleApiGpsLogCsv() {
  String csv;
  csv.reserve(96 + gpsLogCount * 96);
  csv += "index,uptime_ms,crop_name,latitude,longitude,accuracy_m,speed_mps,heading_deg,satellites,source,live_mask,main_mask\n";

  for (uint16_t i = 0; i < gpsLogCount; i++) {
    const GpsLogEntry &entry = gpsLogAt(i);
    csv += i;
    csv += ",";
    csv += entry.uptimeMs;
    csv += ",";
    csv += cropName;
    csv += ",";
    csv += String(entry.latitude, 7);
    csv += ",";
    csv += String(entry.longitude, 7);
    csv += ",";
    csv += String(entry.accuracyM, 1);
    csv += ",";
    csv += String(entry.speedMps, 2);
    csv += ",";
    csv += String(entry.headingDeg, 1);
    csv += ",";
    csv += static_cast<unsigned int>(entry.satellites);
    csv += ",EWD108-GN05(485),";
    csv += static_cast<unsigned int>(entry.liveMask);
    csv += ",";
    csv += static_cast<unsigned int>(entry.mainMask);
    csv += "\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=gps-log.csv");
  server.send(200, "text/csv; charset=utf-8", csv);
}

void handleApiGpsLogGeoJson() {
  String json;
  json.reserve(128 + gpsLogCount * 190);
  json += "{\"type\":\"FeatureCollection\",\"features\":[";

  for (uint16_t i = 0; i < gpsLogCount; i++) {
    const GpsLogEntry &entry = gpsLogAt(i);
    if (i > 0) {
      json += ",";
    }
    json += "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[";
    json += String(entry.longitude, 7);
    json += ",";
    json += String(entry.latitude, 7);
    json += "]},\"properties\":{\"index\":";
    json += i;
    json += ",\"uptime_ms\":";
    json += entry.uptimeMs;
    json += ",\"crop_name\":\"";
    json += cropName;
    json += "\"";
    json += ",\"accuracy_m\":";
    json += String(entry.accuracyM, 1);
    json += ",\"speed_mps\":";
    json += String(entry.speedMps, 2);
    json += ",\"heading_deg\":";
    json += String(entry.headingDeg, 1);
    json += ",\"satellites\":";
    json += static_cast<unsigned int>(entry.satellites);
    json += ",\"source\":\"EWD108-GN05(485)\"";
    json += ",\"live_mask\":";
    json += static_cast<unsigned int>(entry.liveMask);
    json += ",\"main_mask\":";
    json += static_cast<unsigned int>(entry.mainMask);
    json += "}}";
  }

  json += "]}";
  server.sendHeader("Content-Disposition", "attachment; filename=gps-log.geojson");
  server.send(200, "application/geo+json; charset=utf-8", json);
}

void handleApiMainEventsCsv() {
  String csv;
  csv.reserve(96 + mainEventCount * 100);
  csv += "index,uptime_ms,channel,detected,crop_name,latitude,longitude,accuracy_m,live_mask,main_mask\n";

  for (uint16_t i = 0; i < mainEventCount; i++) {
    const MainSignalEvent &event = mainEventAt(i);
    csv += i;
    csv += ",";
    csv += event.uptimeMs;
    csv += ",";
    csv += event.channel;
    csv += ",";
    csv += event.detected ? "1" : "0";
    csv += ",";
    csv += event.crop;
    csv += ",";
    if (event.hasGps) {
      csv += String(event.latitude, 7);
      csv += ",";
      csv += String(event.longitude, 7);
      csv += ",";
      csv += String(event.accuracyM, 1);
    } else {
      csv += ",,";
    }
    csv += ",";
    csv += static_cast<unsigned int>(event.liveMask);
    csv += ",";
    csv += static_cast<unsigned int>(event.mainMask);
    csv += "\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=hauptsignale.csv");
  server.send(200, "text/csv; charset=utf-8", csv);
}

void handleApiMainEventsGeoJson() {
  String json;
  json.reserve(128 + mainEventCount * 220);
  json += "{\"type\":\"FeatureCollection\",\"features\":[";

  bool first = true;
  for (uint16_t i = 0; i < mainEventCount; i++) {
    const MainSignalEvent &event = mainEventAt(i);
    if (!event.hasGps) {
      continue;
    }
    if (!first) {
      json += ",";
    }
    first = false;

    json += "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[";
    json += String(event.longitude, 7);
    json += ",";
    json += String(event.latitude, 7);
    json += "]},\"properties\":{\"index\":";
    json += i;
    json += ",\"uptime_ms\":";
    json += event.uptimeMs;
    json += ",\"channel\":";
    json += event.channel;
    json += ",\"detected\":";
    json += event.detected ? "true" : "false";
    json += ",\"status\":\"";
    json += event.detected ? "Erkannt" : "OK";
    json += "\",\"crop_name\":\"";
    json += event.crop;
    json += "\",\"accuracy_m\":";
    json += String(event.accuracyM, 1);
    json += ",\"live_mask\":";
    json += static_cast<unsigned int>(event.liveMask);
    json += ",\"main_mask\":";
    json += static_cast<unsigned int>(event.mainMask);
    json += "}}";
  }

  json += "]}";
  server.sendHeader("Content-Disposition", "attachment; filename=hauptsignale.geojson");
  server.send(200, "application/geo+json; charset=utf-8", json);
}

void handleApiSensorEventsCsv() {
  String csv;
  csv.reserve(128 + sensorEventCount * 170);
  csv += "index,start_uptime_ms,end_uptime_ms,duration_ms,duration_s,channel,channel_name,crop_name,start_latitude,start_longitude,start_accuracy_m,end_latitude,end_longitude,end_accuracy_m,live_mask_start,main_mask_start,live_mask_end,main_mask_end\n";

  for (uint16_t i = 0; i < sensorEventCount; i++) {
    const SensorTriggerEvent &event = sensorEventAt(i);
    csv += i;
    csv += ",";
    csv += event.startUptimeMs;
    csv += ",";
    csv += event.endUptimeMs;
    csv += ",";
    csv += event.durationMs;
    csv += ",";
    csv += String(event.durationMs / 1000.0f, 2);
    csv += ",";
    csv += event.channel;
    csv += ",";
    csv += event.channelName;
    csv += ",";
    csv += event.crop;
    csv += ",";
    if (event.startHasGps) {
      csv += String(event.startLatitude, 7);
      csv += ",";
      csv += String(event.startLongitude, 7);
      csv += ",";
      csv += String(event.startAccuracyM, 1);
    } else {
      csv += ",,";
    }
    csv += ",";
    if (event.endHasGps) {
      csv += String(event.endLatitude, 7);
      csv += ",";
      csv += String(event.endLongitude, 7);
      csv += ",";
      csv += String(event.endAccuracyM, 1);
    } else {
      csv += ",,";
    }
    csv += ",";
    csv += static_cast<unsigned int>(event.liveMaskAtStart);
    csv += ",";
    csv += static_cast<unsigned int>(event.mainMaskAtStart);
    csv += ",";
    csv += static_cast<unsigned int>(event.liveMaskAtEnd);
    csv += ",";
    csv += static_cast<unsigned int>(event.mainMaskAtEnd);
    csv += "\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=sensorlog.csv");
  server.send(200, "text/csv; charset=utf-8", csv);
}

void handleApiSensorEventsTxt() {
  String text;
  text.reserve(128 + sensorEventCount * 180);
  text += "Sensorlog Drillmaschinenueberwachung\n";
  text += "SSID: ";
  text += AP_SSID;
  text += "\n";
  text += "Saat: ";
  text += cropName;
  text += "\n\n";

  for (uint16_t i = 0; i < sensorEventCount; i++) {
    const SensorTriggerEvent &event = sensorEventAt(i);
    text += "#";
    text += i + 1;
    text += " | Kanal ";
    text += event.channel;
    text += " (";
    text += event.channelName;
    text += ") | Dauer ";
    text += String(event.durationMs / 1000.0f, 2);
    text += " s | Start ";
    text += event.startUptimeMs;
    text += " ms | Ende ";
    text += event.endUptimeMs;
    text += " ms | Saat ";
    text += event.crop;
    if (event.startHasGps) {
      text += " | GPS Start ";
      text += String(event.startLatitude, 7);
      text += ",";
      text += String(event.startLongitude, 7);
      text += " +/-";
      text += String(event.startAccuracyM, 1);
      text += " m";
    }
    if (event.endHasGps) {
      text += " | GPS Ende ";
      text += String(event.endLatitude, 7);
      text += ",";
      text += String(event.endLongitude, 7);
      text += " +/-";
      text += String(event.endAccuracyM, 1);
      text += " m";
    }
    text += "\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=sensorlog.txt");
  server.send(200, "text/plain; charset=utf-8", text);
}

void handleApiGpsLogClear() {
  clearGpsLog();
  server.send(200, "application/json", "{\"ok\":true}");
}

void startWiFiAccessPoint() {
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(300);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  IPAddress ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gateway, subnet);

  bool started = false;
  for (uint8_t attempt = 1; attempt <= 3 && !started; attempt++) {
    started = WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("WLAN AP Start Versuch %u: %s\n", attempt, started ? "OK" : "FEHLER");
    if (!started) {
      delay(500);
    }
  }

  if (!started) {
    Serial.println("WLAN Access Point konnte nicht gestartet werden");
    return;
  }

  Serial.println("WLAN Access Point gestartet");
  Serial.print("WiFi Mode: ");
  Serial.println(WiFi.getMode());
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Aktive SSID: ");
  Serial.println(WiFi.softAPSSID());
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("Passwort: ");
  Serial.println(AP_PASSWORD);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Waveshare ESP32-S3-POE-ETH-8DI-8DO Drillmaschine");

  loadChannelNames();
  loadCropName();
  initGpsLog();
  initGnssRs485();
  initDigitalInputs();
  startWiFiAccessPoint();
  initDigitalOutputs();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/channel-name", HTTP_POST, handleApiChannelName);
  server.on("/api/crop", HTTP_POST, handleApiCrop);
  server.on("/api/recording", HTTP_POST, handleApiRecording);
  server.on("/api/gps-log.csv", HTTP_GET, handleApiGpsLogCsv);
  server.on("/api/gps-log.geojson", HTTP_GET, handleApiGpsLogGeoJson);
  server.on("/api/main-events.csv", HTTP_GET, handleApiMainEventsCsv);
  server.on("/api/main-events.geojson", HTTP_GET, handleApiMainEventsGeoJson);
  server.on("/api/sensor-events.csv", HTTP_GET, handleApiSensorEventsCsv);
  server.on("/api/sensor-events.txt", HTTP_GET, handleApiSensorEventsTxt);
  server.on("/api/gps-log/clear", HTTP_POST, handleApiGpsLogClear);
  server.onNotFound([]() {
    server.send(404, "application/json", "{\"error\":\"not_found\"}");
  });
  server.begin();

  Serial.println("HTTP Server gestartet");
  Serial.println("Webseite: http://" + WiFi.softAPIP().toString() + "/");
  Serial.println("API:      http://" + WiFi.softAPIP().toString() + "/api/status");
}

void loop() {
  readDigitalInputs();
  pollGnss();
  server.handleClient();

  if (millis() - lastDebugMs > 1000) {
    lastDebugMs = millis();
    Serial.print("LIVE:");
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
      Serial.print(channels[i].active ? " 1" : " 0");
    }
    Serial.print(" MAIN:");
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
      Serial.print(channels[i].mainSignal ? " 1" : " 0");
    }
    Serial.print(" DO:");
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
      Serial.print(channels[i].output ? " 1" : " 0");
    }
    Serial.print(" GNSS:");
    Serial.print(gnss.fix ? " FIX" : " NOFIX");
    Serial.print(" REC:");
    Serial.print(recordingActive ? " 1" : " 0");
    Serial.print(" ERR:");
    Serial.print(gnss.lastError);
    Serial.println();
  }
}
