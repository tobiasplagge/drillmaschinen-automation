#include <Arduino.h>
#include <ArduinoJson.h>
#include <Ethernet.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <base64.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

// ---------------------------------------------------------------------------
// Waveshare ESP32-S3-POE-ETH-8DI-8DO configuration
// ---------------------------------------------------------------------------
static const char *AP_SSID = "Drillmaschine-M01";
static const char *AP_PASSWORD = "12345678";

// Waveshare W5500 Ethernet: INT GPIO12, MOSI GPIO13, MISO GPIO14,
// SCLK GPIO15, CS GPIO16, RST GPIO39.
static constexpr uint8_t ETH_INT_PIN = 12;
static constexpr uint8_t ETH_MOSI_PIN = 13;
static constexpr uint8_t ETH_MISO_PIN = 14;
static constexpr uint8_t ETH_SCLK_PIN = 15;
static constexpr uint8_t ETH_CS_PIN = 16;
static constexpr uint8_t ETH_RST_PIN = 39;
static byte ETH_MAC[6] = {0x02, 0x57, 0x53, 0x33, 0x44, 0x01};
static const IPAddress ETHERNET_IP(192, 168, 4, 10);
static const IPAddress ETHERNET_DNS(192, 168, 4, 1);
static const IPAddress ETHERNET_GATEWAY(192, 168, 4, 1);
static const IPAddress ETHERNET_SUBNET(255, 255, 255, 0);

static const char *DEVICE_ID = "waveshare-8di8do-01";
static const char *DEVICE_NAME = "Drillmaschinenüberwachung";
static const char *FIRMWARE_VERSION = "2.1.0";
static const char *MODULE_ID = "M01";
static const char *DEFAULT_CROP_SUGGESTIONS_JSON = "[\"Weizen\",\"Gerste\",\"Raps\",\"Senf\",\"Mais\",\"Gras\",\"Kleegras\",\"Zwischenfrucht\"]";
// Hikvision HTTP/MJPEG preview. IP, Kanal und Zugangsdaten bei Bedarf anpassen.
// Häufige Varianten sind /ISAPI/Streaming/channels/101/httpPreview für Mainstream
// oder /ISAPI/Streaming/channels/102/httpPreview für Substream.
static const char *HIKVISION_CAMERA_MAIN_STREAM_PATH = "/ISAPI/Streaming/channels/101/httpPreview";
static const char *HIKVISION_CAMERA_SUB_STREAM_PATH = "/ISAPI/Streaming/channels/102/httpPreview";
static const char *CAMERA_PROXY_MAIN_STREAM_URL = "/camera/1/mainstream";
static const char *CAMERA_PROXY_SUB_STREAM_URL = "/camera/1/substream";
static const char *HIKVISION_CAMERA_NAME = "Hikvision Kamera";
static constexpr uint16_t HIKVISION_CAMERA_TEST_TIMEOUT_MS = 1500;
static constexpr uint8_t CAMERA_COUNT = 4;
static constexpr uint8_t CAMERA_HOST_LENGTH = 32;
static constexpr uint8_t CAMERA_USERNAME_LENGTH = 32;
static constexpr uint8_t CAMERA_PASSWORD_LENGTH = 48;

static constexpr uint8_t CHANNEL_COUNT = 8;
static constexpr uint8_t SEED_CHANNEL_COUNT = 6;
static constexpr uint8_t HIDDEN_CHANNEL = 7;
static constexpr uint8_t ROTATION_CHANNEL = 8;
static constexpr uint8_t CHANNEL_NAME_LENGTH = 24;
static constexpr uint8_t CROP_NAME_LENGTH = 24;
static constexpr uint8_t FIELD_NAME_LENGTH = 32;
static constexpr uint8_t TRIP_ID_LENGTH = 48;
static constexpr uint32_t DEFAULT_MAIN_SIGNAL_HOLD_MS = 1500;
static constexpr uint32_t MIN_MAIN_SIGNAL_HOLD_MS = 300;
static constexpr uint32_t MAX_MAIN_SIGNAL_HOLD_MS = 10000;
static constexpr uint32_t SIGNAL_READY_TIMEOUT_MS = 10000;
static constexpr uint32_t RED_SIGNAL_HOLD_MS = 15000;
static constexpr uint32_t ROTATION_PULSE_TIMEOUT_MS = 3000;
static constexpr uint8_t ROTATION_PULSES_PER_REV = 1;
static constexpr uint32_t GPS_LOG_INTERVAL_MS = 3000;
static constexpr uint32_t GNSS_POLL_INTERVAL_MS = 1000;
static constexpr uint32_t GNSS_STALE_MS = 10000;
static constexpr uint32_t GNSS_AUTO_START_HOLD_MS = 3000;
static constexpr uint32_t FILE_FLUSH_INTERVAL_MS = 30000;
static constexpr uint8_t WATCHDOG_TIMEOUT_SECONDS = 8;
static constexpr uint16_t GPS_LOG_TARGET_CAPACITY = 5000;
static constexpr uint16_t MAIN_EVENT_LOG_CAPACITY = 512;
static constexpr uint16_t SENSOR_EVENT_LOG_CAPACITY = 512;
static constexpr uint16_t LIVE_TRACK_MAX_POINTS = 1200;
static constexpr uint16_t LIVE_TRACK_MAX_EVENTS = 128;

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
static constexpr uint32_t GNSS_BAUD_SCAN_RATES[] = {4800, 9600, 19200, 38400, 57600, 115200};
static constexpr uint16_t GNSS_SCAN_START_REGISTER = 0x0005;
static constexpr uint16_t GNSS_SCAN_REGISTER_COUNT = 35;
static constexpr uint8_t GNSS_MODBUS_FUNCTION_READ_HOLDING = 0x03;

// On this board the digital outputs are controlled through the TCA9554. The
// Waveshare examples initialize all outputs HIGH. Keep this configurable in
// case your wiring expects the opposite logic.
static constexpr bool DO_ACTIVE_HIGH = true;
static constexpr bool INPUT_ACTIVE_HIGH = false;
static constexpr bool MIRROR_RED_TO_OUTPUT = true;
static constexpr uint8_t LIGHT_OUTPUT_CHANNEL = 1;
static constexpr uint8_t PNEUMATIC_VALVE_COUNT = 4;
static constexpr uint8_t PNEUMATIC_VALVE_OUTPUTS[PNEUMATIC_VALVE_COUNT] = {2, 3, 4, 5};
static const char *PNEUMATIC_VALVE_LABELS[PNEUMATIC_VALVE_COUNT] = {
    "Sensor 1-6",
    "Sensor 7-12",
    "Sensor 13-18",
    "Sensor 19-24",
};

WebServer server(80);
Preferences preferences;
HardwareSerial gnssSerial(1);
bool ethernetReady = false;
char gnssDirectLine[128] = "";
uint8_t gnssDirectLineLength = 0;

struct ChannelState {
  bool inputRaw = false;
  bool active = false;
  bool mainSignal = false;
  bool latchedAlarm = false;
  bool output = false;
  const char *status = "none";
  uint32_t changes = 0;
  uint32_t detectionCount = 0;
  uint32_t lastChangeMs = 0;
  uint32_t lastDetectionMs = 0;
  uint32_t pulseIntervalMs = 0;
  uint32_t activeSinceMs = 0;
  uint32_t mainSignalChanges = 0;
  uint32_t lastMainSignalChangeMs = 0;
  uint8_t signalQualityPct = 0;
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
  uint32_t lastResponseMs = 0;
  uint32_t lastByteMs = 0;
  uint32_t byteCount = 0;
  uint32_t fixAvailableSinceMs = 0;
  char lastError[32] = "not_started";
  char lastSentence[128] = "";
  char rawPreview[96] = "";
  char rawHexPreview[160] = "";
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
    "Rotation",
};
char cropName[CROP_NAME_LENGTH] = "Weizen";
char fieldName[FIELD_NAME_LENGTH] = "Feld";
char cameraHosts[CAMERA_COUNT][CAMERA_HOST_LENGTH] = {"192.168.4.20", "", "", ""};
char cameraUsernames[CAMERA_COUNT][CAMERA_USERNAME_LENGTH] = {"admin", "", "", ""};
char cameraPasswords[CAMERA_COUNT][CAMERA_PASSWORD_LENGTH] = {"Administrator01", "", "", ""};
char tripId[TRIP_ID_LENGTH] = "-";
char tripGpsPath[64] = "";
char tripSensorPath[64] = "";
char tripMetaPath[64] = "";
uint8_t tcaOutputState = 0x00;
bool doExpanderReady = false;
uint32_t lastDebugMs = 0;
uint32_t bootCounter = 0;
uint32_t tripCounter = 0;
uint32_t mainSignalHoldMs = DEFAULT_MAIN_SIGNAL_HOLD_MS;
const char *resetReason = "unknown";
bool filesystemReady = false;
bool autoStartEnabled = true;
bool autoStartArmed = true;
String persistentGpsBuffer;
uint32_t lastFileFlushMs = 0;
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

void startRecording(const char *source);
void stopRecording(const char *source);

bool isSeedChannel(uint8_t channelIndex) {
  return channelIndex < SEED_CHANNEL_COUNT;
}

bool isHiddenChannel(uint8_t channelIndex) {
  return channelIndex == HIDDEN_CHANNEL - 1;
}

bool isPneumaticValveOutput(uint8_t outputChannel) {
  for (uint8_t i = 0; i < PNEUMATIC_VALVE_COUNT; i++) {
    if (PNEUMATIC_VALVE_OUTPUTS[i] == outputChannel) {
      return true;
    }
  }
  return false;
}

bool isRotationChannel(uint8_t channelIndex) {
  return channelIndex == ROTATION_CHANNEL - 1;
}

bool rotationMoving(uint32_t now) {
  const ChannelState &rotation = channels[ROTATION_CHANNEL - 1];
  return rotation.lastDetectionMs > 0 && now - rotation.lastDetectionMs <= ROTATION_PULSE_TIMEOUT_MS;
}

float rotationRpm(uint32_t now) {
  const ChannelState &rotation = channels[ROTATION_CHANNEL - 1];
  if (!rotationMoving(now) || rotation.pulseIntervalMs == 0 || ROTATION_PULSES_PER_REV == 0) {
    return 0.0f;
  }
  return 60000.0f / (static_cast<float>(rotation.pulseIntervalMs) * static_cast<float>(ROTATION_PULSES_PER_REV));
}

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

void sanitizeFieldName(char *name) {
  name[FIELD_NAME_LENGTH - 1] = '\0';
  String cleaned = String(name);
  cleaned.trim();
  if (cleaned.length() == 0) {
    cleaned = "Feld";
  }
  cleaned.replace("\"", "'");
  cleaned.replace("<", "");
  cleaned.replace(">", "");
  cleaned.replace(",", " ");
  cleaned.toCharArray(name, FIELD_NAME_LENGTH);
}

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    default: return "unknown";
  }
}

bool appendFile(const char *path, const String &content) {
  if (!filesystemReady || content.length() == 0) {
    return false;
  }
  File file = LittleFS.open(path, FILE_APPEND);
  if (!file) {
    Serial.printf("WARNUNG: Datei %s nicht beschreibbar\n", path);
    return false;
  }
  const bool ok = file.print(content) == content.length();
  file.close();
  return ok;
}

void appendSystemEvent(const String &message) {
  String line = String(millis()) + "," + MODULE_ID + "," + message + "\n";
  appendFile("/system-events.log", line);
}

void initFilesystem() {
  filesystemReady = LittleFS.begin(true);
  Serial.printf("LittleFS: %s\n", filesystemReady ? "bereit" : "nicht verfügbar");
  if (!filesystemReady) {
    return;
  }
  if (!LittleFS.exists("/system-events.log")) {
    appendFile("/system-events.log", "uptime_ms,module,event\n");
  }
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

void loadFieldName() {
  String stored = preferences.getString("field", "");
  if (stored.length() > 0) {
    stored.toCharArray(fieldName, FIELD_NAME_LENGTH);
  }
  sanitizeFieldName(fieldName);
}

void sanitizeCameraValue(char *value, size_t maxLength) {
  value[maxLength - 1] = '\0';
  String cleaned(value);
  cleaned.trim();
  cleaned.replace("\r", "");
  cleaned.replace("\n", "");
  cleaned.toCharArray(value, maxLength);
}

void loadCameraSettings() {
  for (uint8_t i = 0; i < CAMERA_COUNT; i++) {
    char key[12];

    snprintf(key, sizeof(key), "cam%u_host", i + 1);
    String stored = preferences.getString(key, "");
    if (stored.length() > 0) {
      stored.toCharArray(cameraHosts[i], CAMERA_HOST_LENGTH);
    }
    sanitizeCameraValue(cameraHosts[i], CAMERA_HOST_LENGTH);

    snprintf(key, sizeof(key), "cam%u_user", i + 1);
    stored = preferences.getString(key, "");
    if (stored.length() > 0) {
      stored.toCharArray(cameraUsernames[i], CAMERA_USERNAME_LENGTH);
    }
    sanitizeCameraValue(cameraUsernames[i], CAMERA_USERNAME_LENGTH);

    snprintf(key, sizeof(key), "cam%u_pass", i + 1);
    stored = preferences.getString(key, "");
    if (stored.length() > 0) {
      stored.toCharArray(cameraPasswords[i], CAMERA_PASSWORD_LENGTH);
    }
    sanitizeCameraValue(cameraPasswords[i], CAMERA_PASSWORD_LENGTH);
  }
}

void loadSensitivity() {
  mainSignalHoldMs = preferences.getUInt("hold_ms", DEFAULT_MAIN_SIGNAL_HOLD_MS);
  mainSignalHoldMs = constrain(mainSignalHoldMs, MIN_MAIN_SIGNAL_HOLD_MS, MAX_MAIN_SIGNAL_HOLD_MS);
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

void saveFieldName(const String &name) {
  name.toCharArray(fieldName, FIELD_NAME_LENGTH);
  sanitizeFieldName(fieldName);
  preferences.putString("field", fieldName);
}

bool cameraConfigured(uint8_t cameraIndex) {
  return cameraIndex < CAMERA_COUNT && strlen(cameraHosts[cameraIndex]) > 0;
}

void saveCameraSettings(uint8_t cameraIndex, const String &host, const String &username, const String &password) {
  if (cameraIndex >= CAMERA_COUNT) {
    return;
  }
  host.toCharArray(cameraHosts[cameraIndex], CAMERA_HOST_LENGTH);
  username.toCharArray(cameraUsernames[cameraIndex], CAMERA_USERNAME_LENGTH);
  password.toCharArray(cameraPasswords[cameraIndex], CAMERA_PASSWORD_LENGTH);
  sanitizeCameraValue(cameraHosts[cameraIndex], CAMERA_HOST_LENGTH);
  sanitizeCameraValue(cameraUsernames[cameraIndex], CAMERA_USERNAME_LENGTH);
  sanitizeCameraValue(cameraPasswords[cameraIndex], CAMERA_PASSWORD_LENGTH);

  char key[12];
  snprintf(key, sizeof(key), "cam%u_host", cameraIndex + 1);
  preferences.putString(key, cameraHosts[cameraIndex]);
  snprintf(key, sizeof(key), "cam%u_user", cameraIndex + 1);
  preferences.putString(key, cameraUsernames[cameraIndex]);
  snprintf(key, sizeof(key), "cam%u_pass", cameraIndex + 1);
  preferences.putString(key, cameraPasswords[cameraIndex]);
}

String cameraAuthHeader(uint8_t cameraIndex) {
  if (cameraIndex >= CAMERA_COUNT) {
    return "";
  }
  return "Basic " + base64::encode(String(cameraUsernames[cameraIndex]) + ":" + String(cameraPasswords[cameraIndex]));
}

void saveSensitivity(uint32_t holdMs) {
  mainSignalHoldMs = constrain(holdMs, MIN_MAIN_SIGNAL_HOLD_MS, MAX_MAIN_SIGNAL_HOLD_MS);
  preferences.putUInt("hold_ms", mainSignalHoldMs);
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

void flushPersistentGpsBuffer() {
  if (persistentGpsBuffer.length() == 0 || tripGpsPath[0] == '\0') {
    return;
  }
  if (appendFile(tripGpsPath, persistentGpsBuffer)) {
    persistentGpsBuffer = "";
    lastFileFlushMs = millis();
  }
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

  if (recordingActive && tripGpsPath[0] != '\0') {
    persistentGpsBuffer += String(entry.uptimeMs) + "," + cropName + "," + fieldName + "," +
                           String(entry.latitude, 7) + "," + String(entry.longitude, 7) + "," +
                           String(entry.accuracyM, 1) + "," + String(entry.speedMps, 2) + "," +
                           String(entry.headingDeg, 1) + "," + String(entry.satellites) + "," +
                           String(entry.liveMask) + "," + String(entry.mainMask) + "\n";
    if (persistentGpsBuffer.length() > 2048 || millis() - lastFileFlushMs >= FILE_FLUSH_INTERVAL_MS) {
      flushPersistentGpsBuffer();
    }
  }
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

void appendGnssRawPreview(char c) {
  const size_t length = strnlen(gnss.rawPreview, sizeof(gnss.rawPreview));
  if (length + 1 < sizeof(gnss.rawPreview)) {
    gnss.rawPreview[length] = (c >= 32 && c <= 126) ? c : '.';
    gnss.rawPreview[length + 1] = '\0';
  } else {
    memmove(gnss.rawPreview, gnss.rawPreview + 1, sizeof(gnss.rawPreview) - 2);
    gnss.rawPreview[sizeof(gnss.rawPreview) - 2] = (c >= 32 && c <= 126) ? c : '.';
    gnss.rawPreview[sizeof(gnss.rawPreview) - 1] = '\0';
  }

  char hexByte[4];
  snprintf(hexByte, sizeof(hexByte), "%02X ", static_cast<uint8_t>(c));
  size_t hexLength = strnlen(gnss.rawHexPreview, sizeof(gnss.rawHexPreview));
  if (hexLength + 3 >= sizeof(gnss.rawHexPreview)) {
    memmove(gnss.rawHexPreview, gnss.rawHexPreview + 3, sizeof(gnss.rawHexPreview) - 4);
    gnss.rawHexPreview[sizeof(gnss.rawHexPreview) - 4] = '\0';
    hexLength = strnlen(gnss.rawHexPreview, sizeof(gnss.rawHexPreview));
  }
  strncat(gnss.rawHexPreview, hexByte, sizeof(gnss.rawHexPreview) - hexLength - 1);
}

void noteGnssReceivedByte(char c) {
  gnss.byteCount++;
  gnss.lastByteMs = millis();
  appendGnssRawPreview(c);
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
    noteGnssReceivedByte(static_cast<char>(gnssSerial.read()));
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
      const char c = static_cast<char>(gnssSerial.read());
      noteGnssReceivedByte(c);
      response[index++] = static_cast<uint8_t>(c);
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

  String normalizedHemisphere = hemisphere;
  normalizedHemisphere.toUpperCase();
  const int dotIndex = value.indexOf('.');
  if (dotIndex < 0 || dotIndex < 3) {
    return 0;
  }
  const int degreeDigits = dotIndex - 2;
  const double degrees = value.substring(0, degreeDigits).toDouble();
  const double minutes = value.substring(degreeDigits).toDouble();
  double decimal = degrees + (minutes / 60.0);
  if (normalizedHemisphere == "S" || normalizedHemisphere == "W") {
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
    if (!valid.equalsIgnoreCase("A")) {
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
    if (gnss.fixAvailableSinceMs == 0) {
      gnss.fixAvailableSinceMs = gnss.lastFixMs;
    }
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
    if (gnss.fixAvailableSinceMs == 0) {
      gnss.fixAvailableSinceMs = gnss.lastFixMs;
    }
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

bool readDirectNmeaFromRs485() {
  bool gotGnssData = false;
  const uint32_t now = millis();

  while (gnssSerial.available()) {
    const char c = static_cast<char>(gnssSerial.read());
    noteGnssReceivedByte(c);

    if (c == '$') {
      gnssDirectLineLength = 0;
      gnssDirectLine[gnssDirectLineLength++] = c;
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (gnssDirectLineLength == 0) {
        continue;
      }

      gnssDirectLine[gnssDirectLineLength] = '\0';
      String sentence = String(gnssDirectLine);
      sentence.trim();
      gnssDirectLineLength = 0;

      if (!sentence.startsWith("$")) {
        continue;
      }

      sentence.toCharArray(gnss.rawPreview, sizeof(gnss.rawPreview));
      gnss.lastResponseMs = now;
      gnss.seen = true;
      gotGnssData = true;

      if (applyNmeaSentence(sentence)) {
        gnss.okCount++;
      } else if (strcmp(gnss.lastError, "ok") == 0) {
        setGnssError("nmea_seen");
      }
      continue;
    }

    if (gnssDirectLineLength == 0) {
      continue;
    }

    if (gnssDirectLineLength < sizeof(gnssDirectLine) - 1 && c >= 32 && c <= 126) {
      gnssDirectLine[gnssDirectLineLength++] = c;
    } else {
      gnssDirectLineLength = 0;
      setGnssError("nmea_overflow");
    }
  }

  return gotGnssData;
}

void pollGnss() {
  const bool directDataSeen = readDirectNmeaFromRs485();
  const uint32_t now = millis();
  if (now - gnss.lastPollMs < GNSS_POLL_INTERVAL_MS) {
    return;
  }
  gnss.lastPollMs = now;
  gnss.pollCount++;

  if (directDataSeen || (gnss.lastByteMs > 0 && now - gnss.lastByteMs < 250)) {
    if (recordingActive && gnss.fix && now - lastGpsLogMs >= GPS_LOG_INTERVAL_MS) {
      appendGpsLog(gnss.latitude, gnss.longitude, gnss.accuracyM, gnss.speedMps, gnss.headingDeg, gnss.satellites);
    }
    if (!directDataSeen && strcmp(gnss.lastError, "ok") != 0) {
      setGnssError("raw_rx");
    }
    return;
  }

  uint8_t payload[GNSS_SCAN_REGISTER_COUNT * 2] = {};
  uint8_t byteCount = 0;
  if (!readModbusHoldingRegisters(GNSS_SCAN_START_REGISTER, GNSS_SCAN_REGISTER_COUNT, payload, sizeof(payload), byteCount)) {
    gnss.errorCount++;
    return;
  }
  gnss.lastResponseMs = now;

  if (scanPayloadForNmea(payload, byteCount)) {
    gnss.okCount++;
  } else {
    gnss.errorCount++;
  }

  if (recordingActive && gnss.fix && now - lastGpsLogMs >= GPS_LOG_INTERVAL_MS) {
    appendGpsLog(gnss.latitude, gnss.longitude, gnss.accuracyM, gnss.speedMps, gnss.headingDeg, gnss.satellites);
  }
}

const char *gnssHealth() {
  const uint32_t now = millis();
  if (gnss.lastResponseMs == 0 || now - gnss.lastResponseMs > GNSS_STALE_MS) {
    if (gnss.lastByteMs > 0 && now - gnss.lastByteMs <= GNSS_STALE_MS) {
      return "invalid_data";
    }
    return "no_rs485";
  }
  if (strcmp(gnss.lastError, "no_nmea") == 0 || strcmp(gnss.lastError, "bad_nmea") == 0 ||
      strcmp(gnss.lastError, "crc") == 0) {
    return "invalid_data";
  }
  if (!gnss.fix || gnss.lastFixMs == 0 || now - gnss.lastFixMs > GNSS_STALE_MS) {
    return "no_fix";
  }
  return "ok";
}

void updateGnssHealthAndRecording() {
  const uint32_t now = millis();
  if (gnss.lastFixMs == 0 || now - gnss.lastFixMs > GNSS_STALE_MS) {
    gnss.fix = false;
    gnss.fixAvailableSinceMs = 0;
    autoStartArmed = true;
  }
  if (autoStartEnabled && autoStartArmed && !recordingActive && gnss.fix &&
      gnss.fixAvailableSinceMs > 0 && now - gnss.fixAvailableSinceMs >= GNSS_AUTO_START_HOLD_MS) {
    startRecording("gnss_auto");
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

  if (recordingActive && tripSensorPath[0] != '\0') {
    String line = String(event.startUptimeMs) + "," + String(event.endUptimeMs) + "," +
                  String(event.durationMs) + "," + String(event.channel) + "," +
                  event.channelName + "," + event.crop + "," + fieldName + ",";
    if (event.startHasGps) {
      line += String(event.startLatitude, 7) + "," + String(event.startLongitude, 7);
    } else {
      line += ",";
    }
    line += ",";
    if (event.endHasGps) {
      line += String(event.endLatitude, 7) + "," + String(event.endLongitude, 7);
    } else {
      line += ",";
    }
    line += "\n";
    appendFile(tripSensorPath, line);
  }

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

void createTripFiles() {
  snprintf(tripGpsPath, sizeof(tripGpsPath), "/trip-%s-gps.csv", tripId);
  snprintf(tripSensorPath, sizeof(tripSensorPath), "/trip-%s-sensor.csv", tripId);
  snprintf(tripMetaPath, sizeof(tripMetaPath), "/trip-%s-meta.txt", tripId);
  if (!filesystemReady) {
    return;
  }
  LittleFS.remove(tripGpsPath);
  LittleFS.remove(tripSensorPath);
  LittleFS.remove(tripMetaPath);
  appendFile(tripGpsPath, "uptime_ms,crop_name,field_name,latitude,longitude,accuracy_m,speed_mps,heading_deg,satellites,live_mask,main_mask\n");
  appendFile(tripSensorPath, "start_uptime_ms,end_uptime_ms,duration_ms,channel,channel_name,crop_name,field_name,start_latitude,start_longitude,end_latitude,end_longitude\n");
  String meta = "trip_id=" + String(tripId) + "\nmodule=" + MODULE_ID + "\nfield=" + fieldName +
                "\ncrop=" + cropName + "\nstart_uptime_ms=" + String(millis()) + "\nstatus=active\n";
  appendFile(tripMetaPath, meta);
}

void startRecording(const char *source) {
  if (recordingActive) {
    return;
  }
  clearGpsLog();
  tripCounter++;
  preferences.putUInt("trip_counter", tripCounter);
  snprintf(tripId, sizeof(tripId), "%s-B%06lu-F%04lu", MODULE_ID,
           static_cast<unsigned long>(bootCounter), static_cast<unsigned long>(tripCounter));
  createTripFiles();
  persistentGpsBuffer = "";
  lastFileFlushMs = millis();
  recordingActive = true;
  autoStartArmed = false;
  appendSystemEvent("recording_start," + String(source) + "," + tripId);
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    if (channels[i].mainSignal) {
      startSensorTriggerEvent(i);
    }
  }
  Serial.printf("Fahrtaufzeichnung gestartet: %s (%s)\n", tripId, source);
}

void stopRecording(const char *source) {
  if (!recordingActive) {
    return;
  }
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    finishSensorTriggerEvent(i);
  }
  flushPersistentGpsBuffer();
  if (tripMetaPath[0] != '\0') {
    appendFile(tripMetaPath, "end_uptime_ms=" + String(millis()) + "\nstatus=completed\n");
  }
  appendSystemEvent("recording_stop," + String(source) + "," + tripId);
  recordingActive = false;
  if (strcmp(source, "manual") == 0) {
    autoStartArmed = false;
  }
  Serial.printf("Fahrtaufzeichnung beendet: %s (%s)\n", tripId, source);
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
  doExpanderReady = ok;
  if (ok) {
    channels[channelIndex].output = on;
  }
  return ok;
}

void initDigitalOutputs() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(20);

  tcaOutputState = DO_ACTIVE_HIGH ? 0x00 : 0xFF;
  tcaWrite(TCA9554_OUTPUT_REG, tcaOutputState);
  tcaWrite(TCA9554_CONFIG_REG, 0x00); // all 8 expander pins as outputs

  uint8_t readback = 0;
  doExpanderReady = tcaRead(TCA9554_OUTPUT_REG, readback);
  if (doExpanderReady) {
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
    channels[i].lastDetectionMs = channels[i].active ? millis() : 0;
    channels[i].signalQualityPct = channels[i].active ? 1 : 0;
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
      if (active) {
        if (channels[i].lastDetectionMs > 0) {
          channels[i].pulseIntervalMs = now - channels[i].lastDetectionMs;
        }
        channels[i].detectionCount++;
        channels[i].lastDetectionMs = now;
      }
      channels[i].lastChangeMs = now;
      channels[i].activeSinceMs = active ? now : 0;
    }

    channels[i].inputRaw = raw;
    channels[i].active = active;

    const bool seedChannel = isSeedChannel(i);
    const bool mainSignal = seedChannel && active && channels[i].activeSinceMs > 0 && (now - channels[i].activeSinceMs >= RED_SIGNAL_HOLD_MS);
    if (seedChannel && active) {
      const uint32_t activeMs = channels[i].activeSinceMs > 0 ? now - channels[i].activeSinceMs : 0;
      channels[i].signalQualityPct = static_cast<uint8_t>(constrain((activeMs * 100UL) / max<uint32_t>(mainSignalHoldMs, 1), 1UL, 100UL));
    } else if (channels[i].lastDetectionMs == 0 || now - channels[i].lastDetectionMs > SIGNAL_READY_TIMEOUT_MS) {
      channels[i].signalQualityPct = 0;
    }
    if (mainSignal != channels[i].mainSignal) {
      channels[i].mainSignal = mainSignal;
      channels[i].mainSignalChanges++;
      channels[i].lastMainSignalChangeMs = now;
      appendMainSignalEvent(i, mainSignal);
      if (mainSignal) {
        channels[i].latchedAlarm = true;
        startSensorTriggerEvent(i);
      } else {
        finishSensorTriggerEvent(i);
      }
    } else {
      channels[i].mainSignal = mainSignal;
    }
    channels[i].status = isRotationChannel(i) ? (rotationMoving(now) ? "rotating" : "stopped") : (mainSignal ? "red" : "none");

    if (seedChannel && i != LIGHT_OUTPUT_CHANNEL - 1 && !isPneumaticValveOutput(i + 1)) {
      const bool outputOn = MIRROR_RED_TO_OUTPUT && mainSignal;
      if (channels[i].output != outputOn) {
        setDigitalOutput(i, outputOn);
      }
    }
  }
}

String statusJson() {
  JsonDocument doc;
  const uint32_t now = millis();
  doc["device_id"] = DEVICE_ID;
  doc["device_name"] = DEVICE_NAME;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["module_id"] = MODULE_ID;
  doc["esp_temperature_c"] = temperatureRead();
  doc["crop_name"] = cropName;
  doc["field_name"] = fieldName;
  doc["trip_id"] = tripId;
  doc["auto_start_enabled"] = autoStartEnabled;
  doc["filesystem_ready"] = filesystemReady;
  doc["filesystem_used_bytes"] = filesystemReady ? LittleFS.usedBytes() : 0;
  doc["filesystem_total_bytes"] = filesystemReady ? LittleFS.totalBytes() : 0;
  doc["boot_counter"] = bootCounter;
  doc["reset_reason"] = resetReason;
  doc["uptime_ms"] = now;
  doc["heap_free_bytes"] = ESP.getFreeHeap();
  doc["heap_total_bytes"] = ESP.getHeapSize();
  doc["heap_min_free_bytes"] = ESP.getMinFreeHeap();
  doc["psram_free_bytes"] = ESP.getFreePsram();
  doc["psram_total_bytes"] = ESP.getPsramSize();
  doc["sketch_size_bytes"] = ESP.getSketchSize();
  doc["sketch_free_space_bytes"] = ESP.getFreeSketchSpace();
  doc["wifi_ap_ssid"] = AP_SSID;
  doc["ip"] = WiFi.softAPIP().toString();
  doc["web_url"] = "http://" + WiFi.softAPIP().toString() + "/";
  doc["ethernet_ready"] = ethernetReady;
  doc["ethernet_ip"] = ethernetReady ? Ethernet.localIP().toString() : "";
  doc["ethernet_link"] = Ethernet.linkStatus() == LinkON;
  doc["main_signal_hold_ms"] = mainSignalHoldMs;
  doc["quality_signal_hold_ms"] = mainSignalHoldMs;
  doc["red_signal_hold_ms"] = RED_SIGNAL_HOLD_MS;
  doc["light_channel"] = LIGHT_OUTPUT_CHANNEL;
  doc["light_on"] = channels[LIGHT_OUTPUT_CHANNEL - 1].output;
  doc["light_switchable"] = doExpanderReady;
  JsonArray valves = doc["pneumatic_valves"].to<JsonArray>();
  for (uint8_t i = 0; i < PNEUMATIC_VALVE_COUNT; i++) {
    const uint8_t outputChannel = PNEUMATIC_VALVE_OUTPUTS[i];
    JsonObject valve = valves.add<JsonObject>();
    valve["index"] = i;
    valve["label"] = PNEUMATIC_VALVE_LABELS[i];
    valve["output_channel"] = outputChannel;
    valve["on"] = channels[outputChannel - 1].output;
    valve["switchable"] = doExpanderReady;
  }
  doc["camera_name"] = HIKVISION_CAMERA_NAME;
  doc["camera_stream_url"] = CAMERA_PROXY_SUB_STREAM_URL;
  doc["camera_main_stream_url"] = CAMERA_PROXY_MAIN_STREAM_URL;
  doc["camera_sub_stream_url"] = CAMERA_PROXY_SUB_STREAM_URL;
  doc["camera_host"] = cameraHosts[0];
  doc["camera_username"] = cameraUsernames[0];
  JsonArray cameras = doc["cameras"].to<JsonArray>();
  for (uint8_t i = 0; i < CAMERA_COUNT; i++) {
    JsonObject camera = cameras.add<JsonObject>();
    camera["index"] = i;
    camera["number"] = i + 1;
    camera["name"] = "Kamera " + String(i + 1);
    camera["configured"] = cameraConfigured(i);
    camera["host"] = cameraHosts[i];
    camera["username"] = cameraUsernames[i];
    camera["sub_stream_url"] = "/camera/" + String(i + 1) + "/substream";
    camera["main_stream_url"] = "/camera/" + String(i + 1) + "/mainstream";
  }
  doc["gps_log_interval_ms"] = GPS_LOG_INTERVAL_MS;
  doc["recording_active"] = recordingActive;
  doc["gps_log_count"] = gpsLogCount;
  doc["gps_log_total"] = gpsLogTotal;
  doc["gps_log_capacity"] = gpsLogCapacity;
  doc["last_gps_log_age_ms"] = lastGpsLogMs > 0 ? static_cast<int32_t>(now - lastGpsLogMs) : -1;
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
  gnssJson["health"] = gnssHealth();
  gnssJson["warning"] = strcmp(gnssHealth(), "ok") != 0;
  gnssJson["seen"] = gnss.seen;
  gnssJson["latitude"] = gnss.latitude;
  gnssJson["longitude"] = gnss.longitude;
  gnssJson["accuracy_m"] = gnss.accuracyM;
  gnssJson["speed_mps"] = gnss.speedMps;
  gnssJson["heading_deg"] = gnss.headingDeg;
  gnssJson["satellites"] = gnss.satellites;
  gnssJson["last_fix_age_ms"] = gnss.lastFixMs > 0 ? static_cast<int32_t>(now - gnss.lastFixMs) : -1;
  gnssJson["last_byte_age_ms"] = gnss.lastByteMs > 0 ? static_cast<int32_t>(now - gnss.lastByteMs) : -1;
  gnssJson["poll_count"] = gnss.pollCount;
  gnssJson["ok_count"] = gnss.okCount;
  gnssJson["error_count"] = gnss.errorCount;
  gnssJson["byte_count"] = gnss.byteCount;
  gnssJson["last_error"] = gnss.lastError;
  gnssJson["last_sentence"] = gnss.lastSentence;
  gnssJson["raw_preview"] = gnss.rawPreview;
  gnssJson["raw_hex_preview"] = gnss.rawHexPreview;

  JsonArray modules = doc["modules"].to<JsonArray>();
  for (uint8_t i = 1; i <= 4; i++) {
    JsonObject module = modules.add<JsonObject>();
    char id[4];
    snprintf(id, sizeof(id), "M%02u", i);
    module["id"] = id;
    module["label"] = "Modul " + String(i);
    module["local"] = i == 1;
    module["online"] = i == 1;
    module["prepared"] = i != 1;
  }

  JsonObject rotationJson = doc["rotation"].to<JsonObject>();
  rotationJson["channel"] = ROTATION_CHANNEL;
  rotationJson["status"] = rotationMoving(now) ? "Dreht" : "Dreht nicht";
  rotationJson["moving"] = rotationMoving(now);
  rotationJson["rpm"] = rotationRpm(now);
  rotationJson["pulse_count"] = channels[ROTATION_CHANNEL - 1].detectionCount;
  rotationJson["pulse_interval_ms"] = channels[ROTATION_CHANNEL - 1].pulseIntervalMs;
  rotationJson["last_pulse_age_ms"] = channels[ROTATION_CHANNEL - 1].lastDetectionMs > 0 ? static_cast<int32_t>(now - channels[ROTATION_CHANNEL - 1].lastDetectionMs) : -1;
  rotationJson["timeout_ms"] = ROTATION_PULSE_TIMEOUT_MS;

  JsonArray array = doc["channels"].to<JsonArray>();
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    JsonObject ch = array.add<JsonObject>();
    ch["channel"] = i + 1;
    ch["name"] = channelNames[i];
    ch["hidden"] = isHiddenChannel(i);
    ch["seed_channel"] = isSeedChannel(i);
    ch["rotation_channel"] = isRotationChannel(i);
    ch["di_gpio"] = DI_PINS[i];
    ch["input_raw"] = channels[i].inputRaw;
    ch["active"] = channels[i].active;
    ch["live_active"] = channels[i].active;
    ch["main_signal"] = channels[i].mainSignal;
    ch["latched_alarm"] = channels[i].latchedAlarm;
    ch["status"] = channels[i].status;
    ch["output"] = channels[i].output;
    ch["changes"] = channels[i].changes;
    ch["detection_count"] = channels[i].detectionCount;
    ch["pulse_interval_ms"] = channels[i].pulseIntervalMs;
    ch["rotation_moving"] = isRotationChannel(i) ? rotationMoving(now) : false;
    ch["rotation_rpm"] = isRotationChannel(i) ? rotationRpm(now) : 0.0f;
    ch["signal_quality_pct"] = channels[i].signalQualityPct;
    ch["last_detection_age_ms"] = channels[i].lastDetectionMs > 0 ? static_cast<int32_t>(now - channels[i].lastDetectionMs) : -1;
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
    * { box-sizing: border-box; }
    main { width: min(1100px, calc(100% - 28px)); margin: 0 auto; padding: 24px 0 34px; }
    h1 { margin: 0 0 8px; font-size: clamp(1.55rem, 5vw, 2.25rem); }
    .meta { color: #9ca3af; margin-bottom: 18px; }
    .connection { display: flex; flex-wrap: wrap; gap: 8px 14px; align-items: center; margin-bottom: 14px; color: #d1d5db; font-size: .94rem; }
    .connection span { min-width: 0; overflow-wrap: anywhere; }
    .connection strong { color: #f9fafb; }
    .connection-dot { width: 13px; height: 13px; border-radius: 50%; background: #22c55e; box-shadow: 0 0 10px #22c55e; display: inline-block; margin-right: 6px; vertical-align: -1px; }
    .connection.offline .connection-dot { background: #ef4444; box-shadow: 0 0 12px #ef4444; }
    .connection.stale .connection-dot { background: #facc15; box-shadow: 0 0 10px #facc15; }
    .connection.camera .connection-dot { background: #22c55e; box-shadow: 0 0 10px #22c55e; animation: cameraPulse 1.1s ease-in-out infinite; }
    @keyframes cameraPulse { 0%, 100% { opacity: .35; transform: scale(.85); } 50% { opacity: 1; transform: scale(1.15); } }
    .warning { border: 1px solid #f59e0b; border-radius: 6px; background: #78350f; color: #fef3c7; padding: 10px 12px; margin-bottom: 14px; font-weight: 750; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 12px; }
    .bar-grid { display: grid; grid-template-columns: repeat(8, minmax(72px, 1fr)); gap: 10px; }
    .bar-card { min-width: 0; border: 1px solid #374151; border-radius: 8px; background: #1f2937; padding: 10px; display: grid; justify-items: center; gap: 8px; }
    .bar-card.latched { border-color: #ef4444; box-shadow: 0 0 0 2px rgba(239,68,68,.85), 0 0 18px rgba(239,68,68,.45); }
    .bar-label { width: 100%; min-height: 2.8em; color: #f9fafb; font-weight: 800; font-size: .86rem; text-align: center; overflow-wrap: anywhere; display: flex; align-items: center; justify-content: center; }
    .bar-channel { color: #9ca3af; font-size: .78rem; }
    .status-bar { position: relative; overflow: hidden; width: min(100%, 42px); height: clamp(140px, 34vh, 360px); border-radius: 6px; background: #f59e0b; border: 1px solid rgba(255,255,255,.18); box-shadow: inset 0 0 0 1px rgba(17,24,39,.45), 0 0 16px rgba(245,158,11,.45); }
    .status-bar.signal { background: #f9fafb; box-shadow: inset 0 0 0 1px rgba(17,24,39,.35), 0 0 18px rgba(249,250,251,.55); }
    .status-bar.stopped { background: #991b1b; box-shadow: inset 0 0 0 1px rgba(17,24,39,.35), 0 0 18px rgba(239,68,68,.55); }
    .bar-fill { position: absolute; inset: auto 0 0; height: var(--level, 0%); min-height: 0; background: #22c55e; box-shadow: 0 0 20px rgba(34,197,94,.8); transition: height .2s ease, background .2s ease; }
    .bar-fill.red { height: 100%; background: #ef4444; box-shadow: 0 0 24px rgba(239,68,68,.9); }
    .bar-fill.yellow { background: #84cc16; box-shadow: 0 0 18px rgba(132,204,22,.75); }
    .status-text { font-size: .8rem; font-weight: 800; color: #d1d5db; text-align: center; min-height: 1.2em; }
    .rotation-line { color: #9ca3af; font-size: .76rem; font-weight: 750; text-align: center; min-height: 1.2em; }
    .rotation-line.on { color: #bbf7d0; }
    .rotation-line.off { color: #fecaca; }
    .camera-panel { margin-top: 14px; }
    .camera-panel summary { cursor: pointer; font-weight: 850; color: #f9fafb; }
    .camera-controls { margin-top: 12px; display: flex; gap: 8px; flex-wrap: wrap; }
    .camera-stream-button { background: #374151; min-width: 96px; }
    .camera-stream-button.active { background: #2563eb; }
    .camera-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 12px; margin-top: 14px; }
    .camera-card { border: 1px solid #374151; border-radius: 8px; padding: 10px; background: #111827; }
    .camera-card h3 { margin: 0; color: #f9fafb; font-size: 1rem; }
    .camera-frame { margin-top: 12px; border: 1px solid #374151; border-radius: 8px; overflow: hidden; background: #020617; aspect-ratio: 16 / 9; display: flex; align-items: center; justify-content: center; }
    .camera-frame img { width: 100%; height: 100%; object-fit: contain; display: block; }
    .camera-meta { margin-top: 8px; color: #9ca3af; font-size: .86rem; overflow-wrap: anywhere; }
    .card { min-width: 0; border: 1px solid #374151; border-radius: 8px; background: #1f2937; padding: 14px; }
    .top { display: flex; align-items: flex-start; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
    .name { min-width: 0; font-weight: 800; font-size: 1.08rem; overflow-wrap: anywhere; }
    .channel { color: #9ca3af; font-size: .85rem; margin-top: 2px; }
    .main-state { display: flex; align-items: center; gap: 10px; }
    .dot { width: 40px; height: 40px; border-radius: 50%; background: #facc15; box-shadow: 0 0 14px #facc15; flex: 0 0 auto; }
    .dot.red { background: #ef4444; box-shadow: 0 0 20px #ef4444; }
    .dot.green { background: #22c55e; box-shadow: 0 0 20px #22c55e; }
    .dot.yellow { background: #facc15; box-shadow: 0 0 16px #facc15; }
    .dot.none { background: #facc15; }
    .pillar { width: 20px; height: 64px; border-radius: 6px; background: #facc15; box-shadow: 0 0 10px #facc15; display: inline-block; margin-right: 12px; }
    .pillar.red { background: #ef4444; box-shadow: 0 0 16px #ef4444; }
    .pillar.green { background: #16a34a; box-shadow: 0 0 16px #16a34a; }
    .pillar.yellow { background: #facc15; box-shadow: 0 0 12px #facc15; }
    .pill { display: inline-block; min-width: 44px; padding: 2px 7px; border-radius: 999px; background: #374151; color: #d1d5db; font-size: .82rem; font-weight: 800; text-align: center; }
    .pill.on { background: #16a34a; color: #f0fdf4; }
    .pill.main { background: #dc2626; color: #fef2f2; }
    .pill.ok { background: #16a34a; color: #f0fdf4; }
    .pill.pending { background: #ca8a04; color: #fffbeb; }
    .panel { min-width: 0; border: 1px solid #374151; border-radius: 8px; background: #1f2937; padding: 14px; margin-bottom: 14px; }
    .panel h2 { margin: 0 0 10px; font-size: 1rem; }
    .actions { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
    .actions > * { max-width: 100%; }
    .tools-details { margin-top: 12px; }
    .archive-list { margin-top: 10px; display: grid; gap: 6px; font-size: .88rem; }
    .archive-list a { color: #bfdbfe; overflow-wrap: anywhere; }
    .crop-chip-row { display: flex; flex-wrap: wrap; gap: 8px; margin: 0 0 12px; }
    .crop-chip { background: #374151; color: #f9fafb; padding: 7px 10px; border-radius: 6px; }
    .crop-chip.active { background: #16a34a; }
    .suggestion-row { display: grid; grid-template-columns: minmax(0, 1fr) auto; gap: 8px; align-items: center; padding: 5px 0; }
    .camera-test-status { margin-top: 8px; color: #d1d5db; font-size: .9rem; overflow-wrap: anywhere; white-space: pre-wrap; }
    .camera-test-status.ok { color: #bbf7d0; }
    .camera-test-status.warn { color: #fde68a; }
    .camera-test-status.fail { color: #fecaca; }
    .rs485-test-panel { display: grid; gap: 10px; }
    .rs485-test-status { border: 1px solid #374151; border-radius: 6px; padding: 9px; color: #d1d5db; background: #111827; overflow-wrap: anywhere; }
    .rs485-test-status.ok { border-color: #16a34a; color: #bbf7d0; }
    .rs485-test-status.warn { border-color: #ca8a04; color: #fde68a; }
    .rs485-test-status.fail { border-color: #dc2626; color: #fecaca; }
    .rs485-raw { margin: 0; min-height: 54px; white-space: pre-wrap; font-size: .85rem; color: #d1d5db; background: #111827; border: 1px solid #374151; border-radius: 6px; padding: 9px; overflow-wrap: anywhere; }
    .camera-settings-grid { display: grid; gap: 12px; }
    .camera-settings-card { border: 1px solid #374151; border-radius: 8px; padding: 10px; background: #111827; }
    .camera-settings-card h4 { margin: 0 0 8px; color: #f9fafb; }
    .module-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 8px; }
    .module { border: 1px solid #4b5563; border-radius: 6px; padding: 9px; color: #9ca3af; }
    .module.online { border-color: #16a34a; color: #dcfce7; }
    .settings-only { display: none; }
    .settings-active .settings-only { display: block; }
    .sensor-table { display: grid; gap: 8px; }
    .sensor-row { display: grid; grid-template-columns: minmax(100px, 1fr) repeat(5, minmax(70px, auto)); gap: 8px; align-items: center; border: 1px solid #374151; border-radius: 6px; padding: 8px; color: #d1d5db; font-size: .88rem; }
    .system-row { grid-template-columns: minmax(130px, 1fr) minmax(120px, auto) minmax(80px, auto); }
    .sensor-row strong { color: #f9fafb; overflow-wrap: anywhere; }
    .field-row { display: grid; grid-template-columns: minmax(130px, 1fr) auto; gap: 8px; margin-bottom: 12px; }
    input, select { min-width: 0; height: 40px; border-radius: 6px; border: 1px solid #4b5563; background: #111827; color: #f9fafb; padding: 0 10px; font: inherit; }
    button, .link-button { border: 0; border-radius: 6px; background: #2563eb; color: white; padding: 9px 12px; font: inherit; font-weight: 750; text-decoration: none; cursor: pointer; }
    button.secondary, .link-button.secondary { background: #374151; }
    button.danger { background: #991b1b; }
    button:disabled { opacity: .45; cursor: default; }
    .gps-meta { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 8px 14px; margin-top: 12px; color: #d1d5db; font-size: .92rem; }
    .gps-meta strong { color: #f9fafb; }
    .gps-meta > div { min-width: 0; overflow-wrap: anywhere; }
    .track-wrap { position: relative; overflow: hidden; min-height: 280px; border: 1px solid #374151; border-radius: 6px; background: #e5e7eb; }
    #trackCanvas { display: block; width: 100%; height: 360px; }
    #topoMap { width: 100%; height: 520px; border: 1px solid #374151; border-radius: 6px; background: #e5e7eb; }
    .tabs { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 14px; border-bottom: 1px solid #374151; padding-bottom: 8px; }
    .tab-button { background: #374151; }
    .tab-button.active { background: #2563eb; }
    .nav-action { background: #4b5563; }
    .nav-action.light-on { background: #16a34a; color: #f0fdf4; }
    .nav-action.light-off { background: #991b1b; color: #fef2f2; }
    .nav-action.light-unknown { background: #f59e0b; color: #111827; }
    .valve-panel { margin-bottom: 14px; }
    .valve-actions { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 8px; }
    .valve-button { background: #374151; }
    .valve-button.active { background: #16a34a; color: #f0fdf4; }
    .hidden { display: none !important; }
    .map-status { margin: 0 0 10px; color: #d1d5db; font-size: .9rem; }
    .track-legend { display: flex; flex-wrap: wrap; gap: 8px 16px; margin-top: 10px; color: #d1d5db; font-size: .88rem; }
    .legend-key { display: inline-flex; align-items: center; gap: 6px; }
    .legend-dot { width: 11px; height: 11px; border-radius: 50%; display: inline-block; background: #16a34a; }
    .legend-dot.route { background: #2563eb; }
    .legend-dot.alert { background: #dc2626; }
    details { margin-top: 10px; border-top: 1px solid #374151; padding-top: 10px; }
    summary { color: #d1d5db; cursor: pointer; font-weight: 700; }
    dl { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 10px; margin: 0; font-size: .92rem; }
    dt { color: #9ca3af; }
    dd { margin: 0; text-align: right; font-weight: 650; overflow-wrap: anywhere; }
    .error { min-height: 1.4em; color: #fca5a5; margin-top: 14px; }
    @media (max-width: 520px) {
      main { width: min(100% - 20px, 1100px); padding-top: 16px; }
      .grid { grid-template-columns: 1fr; }
      .bar-grid { grid-template-columns: repeat(4, minmax(0, 1fr)); }
      .status-bar { width: 34px; height: 180px; }
      .bar-label { font-size: .78rem; }
      .field-row { grid-template-columns: 1fr; }
      .connection { display: grid; grid-template-columns: 1fr 1fr; }
      .actions { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .actions > * { display: flex; min-width: 0; width: 100%; align-items: center; justify-content: center; overflow-wrap: anywhere; text-align: center; }
      .gps-meta { grid-template-columns: 1fr; }
      .module-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .sensor-row { grid-template-columns: 1fr 1fr; }
      #topoMap, #trackCanvas { height: 430px; }
    }
  </style>
</head>
<body>
  <main>
    <h1 id="title">Drillmaschinenüberwachung</h1>
    <div class="meta">
      <span id="ip">IP: -</span> · <span id="version">Version: -</span> · <span id="updated">-</span>
    </div>
    <div id="debugOverlay" class="warning hidden" style="display:none;margin-bottom:10px;font-size:.9rem;"></div>
    <div id="connection" class="connection">
      <span>Kontakt: <span class="connection-dot"></span><strong id="connectionState">OK</strong></span>
      <span>ESP Temperatur: <strong id="espTemp">-</strong></span>
    </div>
    <div id="gnssWarning" class="warning hidden"></div>
    <nav class="tabs" aria-label="Ansicht">
      <button id="monitoringTab" class="tab-button active" type="button">Überwachung</button>
      <button id="mapTab" class="tab-button" type="button">Karte</button>
      <button id="lightOn" class="nav-action light-unknown" type="button">Licht</button>
      <button id="alarmEnable" class="nav-action secondary" type="button">Ton aktivieren</button>
      <button id="alarmAck" class="nav-action secondary" type="button">Alarm quittieren</button>
      <button id="settingsTab" class="tab-button" type="button">Einstellungen</button>
    </nav>
    <section class="panel settings-only">
      <h2>Überblick</h2>
      <details class="tools-details">
        <summary>Dateien und Wartung</summary>
        <div class="actions">
          <a class="link-button secondary" href="/api/gps-log.csv">CSV</a>
          <a class="link-button secondary" href="/api/gps-log.geojson">GeoJSON</a>
          <a class="link-button secondary" href="/api/combined.geojson">Route + Sensoren</a>
          <a class="link-button secondary" href="/api/main-events.csv">Hauptsignale CSV</a>
          <a class="link-button secondary" href="/api/sensor-events.csv">Sensorlog CSV</a>
          <a class="link-button secondary" href="/api/sensor-events.txt">Sensorlog TXT</a>
          <a class="link-button secondary" href="/api/system-events.log">Neustart-Log</a>
          <button id="archiveRefresh" class="secondary" type="button">Archiv anzeigen</button>
          <button id="gpsClear" class="danger" type="button">Live-Log löschen</button>
        </div>
        <div id="archiveList" class="archive-list"></div>
      </details>
      <div class="gps-meta">
        <div>Sensor-Status: <strong id="sensorStatus">-</strong></div>
        <div>Letzter Kontakt: <strong id="lastContactMini">-</strong></div>
        <div>GPS-Signal: <strong id="gnssFix">-</strong></div>
        <div>Alarm: <strong id="alarmStatus">Aus</strong></div>
      </div>
    </section>
    <section id="mapView" class="panel hidden">
      <h2>Live-Fahrt</h2>
      <p id="mapStatus" class="map-status">Topografische Karte wird beim Öffnen geladen.</p>
      <div class="actions"><button id="mapFollow" class="secondary" type="button">Position folgen: Ein</button></div>
      <div class="gps-meta">
        <div>Signalqualität: <strong id="mapGnssQuality">-</strong></div>
        <div>Position: <strong id="mapGnssPosition">-</strong></div>
        <div>Genauigkeit: <strong id="mapGnssAccuracy">-</strong></div>
        <div>RS485: <strong id="mapGnssRs485">-</strong></div>
      </div>
      <div id="topoMap" class="hidden"></div>
      <div id="trackFallback" class="track-wrap">
        <canvas id="trackCanvas"></canvas>
      </div>
      <div class="track-legend">
        <span class="legend-key"><span class="legend-dot"></span>Aktuelle Position</span>
        <span class="legend-key"><span class="legend-dot route"></span>Fahrspur</span>
        <span class="legend-key"><span class="legend-dot alert"></span>Sensor erkannt</span>
        <span id="trackInfo">Warte auf GNSS-Daten</span>
      </div>
    </section>
    <section class="panel settings-only">
      <h2>Saat</h2>
      <div class="field-row">
        <input id="fieldInput" maxlength="31" value="Feld" placeholder="Feldname">
        <button id="fieldSave" type="button">Feld speichern</button>
      </div>
      <div class="field-row">
        <input id="cropInput" list="cropSuggestions" maxlength="23" value="Weizen">
        <datalist id="cropSuggestions"></datalist>
        <button id="cropSave" type="button">Speichern</button>
      </div>
      <div id="cropQuickList" class="crop-chip-row"></div>
      <div class="gps-meta">
        <div>Aktuell: <strong id="cropCurrent">-</strong></div>
        <div>Feld: <strong id="fieldCurrent">-</strong></div>
      </div>
    </section>
    <section id="settingsView" class="panel hidden">
      <h2>Fehleranalyse & Einstellungen</h2>
      <div class="field-row">
        <div style="min-width:0;">
          <h3>Kameras</h3>
          <div id="cameraSettingsGrid" class="camera-settings-grid"></div>
        </div>
      </div>
      <div class="settings-only">
        <h3>Kanaldetails</h3>
        <div id="sensorDetailsGrid" class="sensor-table"></div>
      </div>
      <div class="field-row">
        <div style="min-width:0;">
          <h3>Systemauslastung</h3>
          <div id="systemLoadGrid" class="sensor-table"></div>
        </div>
      </div>
      <div class="field-row">
        <div style="min-width:0;">
          <h3>Empfindlichkeit</h3>
          <div class="field-row">
            <input id="sensitivityInput" type="number" min="300" max="10000" step="100" value="1500">
            <button id="sensitivitySave" type="button">Speichern</button>
          </div>
          <div class="gps-meta">
            <div>Signalpegel 100 % bei: <strong id="sensitivityCurrent">-</strong></div>
            <div>Rot/Alarm ab: <strong id="redSignalCurrent">-</strong></div>
          </div>
        </div>
      </div>
      <div class="field-row">
        <div style="min-width:0;">
          <h3>GNSS Diagnose</h3>
          <div class="rs485-test-panel">
            <div class="actions">
              <button id="rs485Test" class="secondary" type="button">RS485 Test starten</button>
              <button id="rs485BaudScan" class="secondary" type="button">Baudrate scannen</button>
              <button id="rs485AddressScan" class="secondary" type="button">Adressen scannen</button>
              <button id="rs485RegisterScan" class="secondary" type="button">Register scannen</button>
              <span id="rs485TestStatus" class="rs485-test-status">Noch nicht getestet.</span>
            </div>
            <div id="rs485TestGrid" class="sensor-table"></div>
            <strong>Rohdaten ASCII</strong>
            <pre id="rs485RawPreview" class="rs485-raw">Keine Rohdaten.</pre>
            <strong>Rohdaten HEX</strong>
            <pre id="rs485HexPreview" class="rs485-raw">Keine HEX-Daten.</pre>
          </div>
          <pre id="gnssDebug" style="white-space:pre-wrap; font-size:.85rem; color:#d1d5db;"></pre>
        </div>
      </div>
      <div class="field-row">
        <div style="min-width:0;">
          <h3>Saat-Vorschläge verwalten</h3>
          <div style="display:flex;gap:8px;margin-bottom:8px;">
            <input id="cropAddInput" placeholder="Neue Saat hinzufügen" style="flex:1; height:36px;" />
            <button id="cropAddBtn" class="secondary" type="button">Hinzufügen</button>
          </div>
          <div id="cropSuggestionsList" style="color:#d1d5db;font-size:.95rem;"></div>
        </div>
      </div>
      <details>
        <summary>Dateien und Logs</summary>
        <div class="actions">
          <a class="link-button secondary" href="/api/gps-log.csv">CSV</a>
          <a class="link-button secondary" href="/api/gps-log.geojson">GeoJSON</a>
          <a class="link-button secondary" href="/api/combined.geojson">Route + Sensoren</a>
          <a class="link-button secondary" href="/api/main-events.csv">Hauptsignale CSV</a>
          <a class="link-button secondary" href="/api/sensor-events.csv">Sensorlog CSV</a>
          <a class="link-button secondary" href="/api/sensor-events.txt">Sensorlog TXT</a>
          <a class="link-button secondary" href="/api/system-events.log">Neustart-Log</a>
        </div>
      </details>
    </section>
    <section class="panel settings-only">
      <h2>Module</h2>
      <div id="modules" class="module-grid"></div>
    </section>
    <div class="hidden" aria-hidden="true">
      <button id="gpsStart" type="button">Aufzeichnen</button>
      <button id="gpsStop" type="button">Aufzeichnung Stop</button>
      <span id="gpsStatus"></span>
      <span id="tripId"></span>
      <span id="gnssSource"></span>
      <span id="gpsCount"></span>
      <span id="mainEventCount"></span>
      <span id="sensorEventCount"></span>
      <span id="gpsLast"></span>
      <span id="gpsPosition"></span>
      <span id="gpsAccuracy"></span>
      <span id="gpsSatellites"></span>
      <span id="gnssRs485"></span>
      <span id="filesystem"></span>
    </div>
    <div id="grid" class="bar-grid monitoring-view"></div>
    <section class="panel monitoring-view valve-panel">
      <h2>Pneumatikventile</h2>
      <div id="valveActions" class="valve-actions"></div>
    </section>
    <div id="cameraGrid" class="camera-grid monitoring-view"></div>
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
    let systemWarningActive = false;
    let acknowledgedSystemWarning = false;
    let lastSuccessfulContactMs = 0;
    let connectionTimer = null;
    let renderingGrid = false;
    let lightOnState = false;
    let lightSwitchable = false;
    let cameraStreamModes = JSON.parse(localStorage.getItem('cameraStreamModes') || '{}');
    let trackBusy = false;
    let lastTrackData = { points: [], events: [], current: null };
    let topoMap = null;
    let topoRoute = null;
    let topoCurrent = null;
    let topoEvents = null;
    let leafletLoading = false;
    let topoFitted = false;
    let followMap = true;
    let rs485TestRunning = false;
    const openDetailChannels = new Set();

    function escapeHtml(value) {
      var safe = (value !== undefined && value !== null) ? String(value) : '';
      return safe.replace(/[&<>"']/g, function(c) {
        return ({'&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'})[c];
      });
    }

    function visibleChannels(channels) {
      return (channels || []).filter(ch => !ch.hidden && ch.channel !== 7);
    }

    function formatRpm(value) {
      const rpm = Number(value || 0);
      if (!Number.isFinite(rpm) || rpm <= 0) return '0 U/min';
      return rpm >= 10 ? rpm.toFixed(0) + ' U/min' : rpm.toFixed(1) + ' U/min';
    }

    function card(ch, rotation) {
      if (ch.rotation_channel) {
        const name = escapeHtml(ch.name || 'Rotation');
        const moving = Boolean(rotation && rotation.moving);
        const rpm = formatRpm(rotation ? rotation.rpm : ch.rotation_rpm);
        return `
          <div class="bar-card" data-channel="${ch.channel}" aria-label="${name}: ${moving ? 'Dreht' : 'Dreht nicht'}">
            <div class="bar-channel">K${ch.channel}</div>
            <div class="status-bar ${moving ? 'signal' : 'stopped'}" aria-hidden="true">
              <div class="bar-fill" style="--level:${moving ? 100 : 0}%"></div>
            </div>
            <div class="bar-label">${name}</div>
            <div class="status-text">${moving ? 'Dreht' : 'Dreht nicht'}</div>
            <div class="rotation-line ${moving ? 'on' : 'off'}">${rpm}</div>
          </div>`;
      }

      const name = escapeHtml(ch.name || ('Kanal ' + ch.channel));
      const rotationMoving = Boolean(rotation && rotation.moving);
      const rotationText = rotationMoving ? `Dreht · ${formatRpm(rotation.rpm)}` : 'Dreht nicht';
      const quality = Math.max(0, Math.min(100, Number(ch.signal_quality_pct || 0)));
      const recentlyDetected = quality > 0;
      const level = ch.main_signal ? 100 : quality;
      const view = ch.main_signal
        ? { label: 'Dauersignal', fill: 'red' }
        : (recentlyDetected
          ? { label: level + ' %', fill: level >= 70 ? 'yellow' : '' }
          : { label: 'Bereit', fill: '' });
      const alarmClass = ch.latched_alarm ? ' latched' : '';
      return `
        <div class="bar-card${alarmClass}" data-channel="${ch.channel}" aria-label="${name}: ${view.label}">
          <div class="bar-channel">K${ch.channel}</div>
          <div class="status-bar ${recentlyDetected || ch.main_signal ? 'signal' : ''}" aria-hidden="true">
            <div class="bar-fill ${view.fill}" style="--level:${level}%"></div>
          </div>
          <div class="bar-label">${name}</div>
          <div class="status-text">${view.label}</div>
          <div class="rotation-line ${rotationMoving ? 'on' : 'off'}">${rotationText}</div>
        </div>`;
    }

    function sensorDetails(channels) {
      return visibleChannels(channels).map(ch => `
        <div class="sensor-row">
          <strong>${escapeHtml(ch.name || ('Kanal ' + ch.channel))}</strong>
          <span>Kanal ${ch.channel}</span>
          <span>Status: ${ch.rotation_channel ? (ch.rotation_moving ? 'Dreht' : 'Dreht nicht') : (ch.main_signal ? 'Dauersignal' : (ch.live_active ? 'Saat erkannt' : 'Bereit'))}</span>
          <span>Detektionen: ${ch.detection_count || 0}</span>
          <span>Drehzahl: ${ch.rotation_channel ? formatRpm(ch.rotation_rpm) : '-'}</span>
          <span>Qualität: ${ch.signal_quality_pct || 0} %</span>
          <span>Letzte Detektion: ${ch.last_detection_age_ms >= 0 ? ch.last_detection_age_ms + ' ms' : '-'}</span>
          <span>Live: ${ch.live_active ? 'JA' : 'NEIN'}</span>
          <span>Livezeit: ${ch.active_ms || 0} ms</span>
          <span>Haupt: ${ch.main_signal ? 'Erkannt' : 'OK'}</span>
          <span>Quittierung: ${ch.latched_alarm ? 'offen' : 'OK'}</span>
          <span>DI ${ch.di_gpio}: ${ch.input_raw ? 'HIGH' : 'LOW'}</span>
          <span>Output: ${ch.output ? 'Ein' : 'Aus'}</span>
          <span>Livewechsel: ${ch.changes || 0}</span>
          <span>Hauptwechsel: ${ch.main_signal_changes || 0}</span>
          <span>Letzter Livewechsel: ${ch.last_change_age_ms >= 0 ? ch.last_change_age_ms + ' ms' : '-'}</span>
          <span>Letzter Hauptwechsel: ${ch.last_main_signal_change_age_ms >= 0 ? ch.last_main_signal_change_age_ms + ' ms' : '-'}</span>
        </div>
      `).join('');
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
      } else if ((currentMainMask !== 0 && currentMainMask === acknowledgedMainMask) ||
                 (systemWarningActive && acknowledgedSystemWarning)) {
        target.textContent = 'Quittiert';
      } else {
        target.textContent = 'Bereit';
      }
    }

    function updateLightButton() {
      const button = document.getElementById('lightOn');
      button.textContent = 'Licht';
      button.classList.remove('light-on', 'light-off', 'light-unknown');
      if (!lightSwitchable) {
        button.classList.add('light-unknown');
        button.title = 'Lichtausgang nicht schaltbar';
      } else if (lightOnState) {
        button.classList.add('light-on');
        button.title = 'Licht ist eingeschaltet';
      } else {
        button.classList.add('light-off');
        button.title = 'Licht ist ausgeschaltet';
      }
    }

    function updateValveButtons(valves) {
      const container = document.getElementById('valveActions');
      container.innerHTML = (valves || []).map(valve => `
        <button class="valve-button ${valve.on ? 'active' : ''}" type="button" data-channel="${valve.output_channel}" data-on="${valve.on ? '1' : '0'}" ${valve.switchable ? '' : 'disabled'}>
          ${escapeHtml(valve.label || ('Ausgang ' + valve.output_channel))}
        </button>
      `).join('');
      container.querySelectorAll('.valve-button').forEach(button => button.addEventListener('click', () => toggleValve(button)));
    }

    async function toggleValve(button) {
      if (actionBusy) return;
      actionBusy = true;
      const channel = Number(button.dataset.channel);
      const nextState = button.dataset.on !== '1';
      try {
        const res = await fetchWithTimeout('/api/output', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ channel, on: nextState })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('error').textContent = 'Pneumatikventil schalten fehlgeschlagen';
      } finally {
        actionBusy = false;
      }
    }

    function setCameraStreamMode(index, mode) {
      cameraStreamModes[index] = mode === 'main' ? 'main' : 'sub';
      localStorage.setItem('cameraStreamModes', JSON.stringify(cameraStreamModes));
      updateCamera(window.lastStatusData || {});
    }

    function renderCameraSettings(cameras) {
      const container = document.getElementById('cameraSettingsGrid');
      container.innerHTML = cameras.map(camera => `
        <div class="camera-settings-card" data-camera-index="${camera.index}">
          <h4>Kamera ${camera.number}</h4>
          <div class="field-row">
            <input class="cameraHostInput" maxlength="31" placeholder="Kamera-IP" value="${escapeHtml(camera.host || '')}">
            <input class="cameraUserInput" maxlength="31" placeholder="Benutzer" value="${escapeHtml(camera.username || '')}">
          </div>
          <div class="field-row">
            <input class="cameraPasswordInput" type="password" maxlength="47" placeholder="${camera.configured ? 'Passwort gespeichert' : 'Passwort'}">
            <button class="cameraSave" type="button">Speichern</button>
          </div>
          <div class="actions">
            <button class="cameraTest secondary" type="button">Verbindung testen</button>
            <button class="cameraClear secondary" type="button">Ausblenden</button>
          </div>
          <div class="camera-test-status">Noch nicht getestet.</div>
        </div>
      `).join('');
      container.querySelectorAll('.cameraSave').forEach(button => button.addEventListener('click', event => saveCameraSettings(event.currentTarget.closest('.camera-settings-card'))));
      container.querySelectorAll('.cameraTest').forEach(button => button.addEventListener('click', event => testCameraConnection(event.currentTarget.closest('.camera-settings-card'))));
      container.querySelectorAll('.cameraClear').forEach(button => button.addEventListener('click', event => clearCameraSettings(event.currentTarget.closest('.camera-settings-card'))));
    }

    function updateCamera(data) {
      const cameras = data.cameras || [];
      const settingsGrid = document.getElementById('cameraSettingsGrid');
      if (!settingsGrid.contains(document.activeElement)) {
        renderCameraSettings(cameras);
      }
      const configured = cameras.filter(camera => camera.configured);
      const container = document.getElementById('cameraGrid');
      container.innerHTML = configured.length ? configured.map(camera => {
        const mode = cameraStreamModes[camera.index] === 'main' ? 'main' : 'sub';
        const streamUrl = mode === 'main' ? camera.main_stream_url : camera.sub_stream_url;
        return `
          <details class="panel camera-panel camera-card" data-camera-index="${camera.index}" data-camera-stream-url="${escapeHtml(streamUrl)}">
            <summary><span>${escapeHtml(camera.name || ('Kamera ' + camera.number))}</span></summary>
            <div class="camera-controls">
              <button class="camera-stream-button ${mode === 'sub' ? 'active' : ''}" type="button" data-mode="sub">Substream</button>
              <button class="camera-stream-button ${mode === 'main' ? 'active' : ''}" type="button" data-mode="main">Mainstream</button>
            </div>
            <div class="camera-frame"><img alt="Kamerabild" loading="lazy"></div>
            <div class="camera-meta">${escapeHtml(streamUrl)}</div>
          </details>
        `;
      }).join('') : '';

      container.querySelectorAll('.camera-stream-button').forEach(button => button.addEventListener('click', event => {
        const panel = event.currentTarget.closest('.camera-panel');
        setCameraStreamMode(panel.dataset.cameraIndex, event.currentTarget.dataset.mode);
      }));
      container.querySelectorAll('.camera-panel').forEach(panel => {
        panel.addEventListener('toggle', event => {
          syncCameraPanel(event.currentTarget);
          if (event.currentTarget.open) {
            document.getElementById('error').textContent = '';
            hideDebugOverlay();
            updateLastContact();
          } else {
            refresh();
          }
        });
        syncCameraPanel(panel);
      });
    }

    function syncCameraPanel(panel) {
      const image = panel.querySelector('img');
      const streamUrl = panel.dataset.cameraStreamUrl || '';
      if (panel.open && streamUrl && image.getAttribute('src') !== streamUrl) {
        image.src = streamUrl;
      } else if (!panel.open && image.getAttribute('src')) {
        image.removeAttribute('src');
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

    async function acknowledgeAlarm() {
      if (!alarmEnabled) {
        try {
          await enableAlarm();
        } catch (err) {
          showDebug('Tonfreigabe fehlgeschlagen: ' + err.message);
        }
      }
      acknowledgedMainMask = currentMainMask;
      acknowledgedSystemWarning = systemWarningActive;
      stopPulsingAlarm();
      updateAlarmStatus();
      try {
        const res = await fetchWithTimeout('/api/alarm/ack', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: '{}'
        }, 2500);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        refresh();
      } catch (err) {
        showDebug('Quittierung fehlgeschlagen: ' + err.message);
      }
    }

    function checkMainSignalAlarms(channels) {
      let mask = 0;
      channels.forEach(ch => {
        const previous = knownMainSignals.get(ch.channel);
        const current = Boolean(ch.main_signal || ch.latched_alarm);
        if (current) {
          mask |= 1 << (ch.channel - 1);
        }
        knownMainSignals.set(ch.channel, current);
      });
      currentMainMask = mask;
      mainSignalSnapshotReady = true;
      syncAlarm();
    }

    function syncAlarm() {
      if (currentMainMask === 0) {
        acknowledgedMainMask = 0;
      }
      if (!systemWarningActive) {
        acknowledgedSystemWarning = false;
      }
      if ((currentMainMask & ~acknowledgedMainMask) !== 0 || (systemWarningActive && !acknowledgedSystemWarning)) {
        startPulsingAlarm();
      } else {
        stopPulsingAlarm();
      }
      updateAlarmStatus();
    }

    function updateGnssWarning(gnss) {
      const box = document.getElementById('gnssWarning');
      const messages = {
        no_rs485: 'GNSS-Warnung: RS485-GPS-Modul antwortet nicht.',
        no_fix: 'GNSS-Warnung: GPS-Modul erreichbar, aber noch kein Satelliten-Fix.',
        invalid_data: 'GNSS-Warnung: GPS-Daten empfangen, aber nicht auswertbar.'
      };
      const message = messages[gnss.health];
      systemWarningActive = Boolean(message);
      box.textContent = message || '';
      box.classList.toggle('hidden', !message);
      syncAlarm();
    }

    function gnssQualityText(gnss) {
      if (!gnss || !gnss.seen) return 'Keine Daten';
      if (!gnss.fix) return 'Kein Fix';
      const age = Number(gnss.last_fix_age_ms);
      if (Number.isFinite(age) && age > 10000) return 'Fix veraltet';
      const accuracy = Number(gnss.accuracy_m);
      const satellites = Number(gnss.satellites || 0);
      if (Number.isFinite(accuracy) && accuracy >= 0) {
        if (accuracy <= 2) return 'Sehr gut';
        if (accuracy <= 5) return 'Gut';
        if (accuracy <= 10) return 'Mittel';
        return 'Schwach';
      }
      if (satellites >= 10) return 'Sehr gut';
      if (satellites >= 6) return 'Gut';
      return 'Fix aktiv';
    }

    function updateMapGnssStatus(gnss) {
      document.getElementById('mapGnssQuality').textContent = gnssQualityText(gnss);
      document.getElementById('mapGnssPosition').textContent = gnss && gnss.fix
        ? `${Number(gnss.latitude).toFixed(6)}, ${Number(gnss.longitude).toFixed(6)}`
        : '-';
      document.getElementById('mapGnssAccuracy').textContent = gnss && Number.isFinite(gnss.accuracy_m) && gnss.accuracy_m >= 0
        ? `${Number(gnss.accuracy_m).toFixed(1)} m`
        : ((gnss && gnss.satellites) ? `${gnss.satellites} Satelliten` : '-');
      document.getElementById('mapGnssRs485').textContent = gnss
        ? `${gnss.last_error || '-'} · ${gnss.last_byte_age_ms >= 0 ? gnss.last_byte_age_ms + ' ms' : 'kein Byte'}`
        : '-';
    }

    function isCameraStreamOpen() {
      return Array.from(document.querySelectorAll('.camera-panel')).some(panel => {
        const image = panel.querySelector('img');
        return Boolean(panel.open && image && image.getAttribute('src'));
      });
    }

    function hideDebugOverlay() {
      const dbg = document.getElementById('debugOverlay');
      if (!dbg) return;
      dbg.textContent = '';
      dbg.style.display = 'none';
      dbg.classList.add('hidden');
    }

    function setConnectionState(state, detail) {
      const box = document.getElementById('connection');
      const label = document.getElementById('connectionState');
      box.classList.remove('offline', 'stale', 'camera');
      if (state === 'offline') {
        box.classList.add('offline');
        label.textContent = 'Fehler';
      } else if (state === 'stale') {
        box.classList.add('stale');
        label.textContent = '> 10 s';
      } else if (state === 'camera') {
        box.classList.add('camera');
        label.textContent = 'Kamera aktiv';
      } else {
        label.textContent = 'OK';
      }
      if (detail) {
        label.textContent += ' - ' + detail;
      }
    }

    function updateLastContact() {
      const target = document.getElementById('lastContactMini');
      if (isCameraStreamOpen()) {
        target.textContent = 'Kamera';
        setConnectionState('camera');
        return;
      }
      if (!lastSuccessfulContactMs) {
        target.textContent = '-';
        setConnectionState('offline');
        return;
      }
      const ageSeconds = Math.round((Date.now() - lastSuccessfulContactMs) / 1000);
      target.textContent = ageSeconds + ' s';
      if (ageSeconds > 10) {
        setConnectionState('stale');
      } else {
        setConnectionState('online');
      }
    }

    function showMapFallback(message) {
      document.getElementById('mapStatus').textContent = message;
      document.getElementById('topoMap').classList.add('hidden');
      document.getElementById('trackFallback').classList.remove('hidden');
      drawTrack();
    }

    function updateTopoMap(data = lastTrackData) {
      if (!topoMap || !window.L) return;
      const route = (data.points || []).map(point => [Number(point.latitude), Number(point.longitude)]);
      topoRoute.setLatLngs(route);
      topoEvents.clearLayers();
      (data.events || []).forEach(event => {
        L.circleMarker([Number(event.latitude), Number(event.longitude)], {
          radius: 7, color: '#fff', weight: 2, fillColor: '#dc2626', fillOpacity: 1
        }).bindPopup(`Sensor ${event.channel}: Erkannt`).addTo(topoEvents);
      });
      if (data.current && data.current.fix) {
        const current = [Number(data.current.latitude), Number(data.current.longitude)];
        topoCurrent.setLatLng(current).setStyle({ opacity: 1, fillOpacity: 1 });
        if (!topoFitted) {
          if (route.length > 1) topoMap.fitBounds(topoRoute.getBounds(), { padding: [28, 28] });
          else topoMap.setView(current, 17);
          topoFitted = true;
        } else if (followMap) {
          topoMap.panTo(current, { animate: true });
        }
      }
      document.getElementById('mapStatus').textContent = 'OpenTopoMap online · Live-Spur aktiv';
    }

    function createTopoMap() {
      if (topoMap || !window.L) return;
      const target = document.getElementById('topoMap');
      target.classList.remove('hidden');
      document.getElementById('trackFallback').classList.add('hidden');
      topoMap = L.map('topoMap').setView([51.0, 10.0], 6);
      topoMap.on('dragstart zoomstart', () => setMapFollow(false));
      const tiles = L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png', {
        maxZoom: 17,
        attribution: 'Kartendaten: © OpenStreetMap-Mitwirkende, SRTM | Darstellung: © OpenTopoMap (CC-BY-SA)'
      });
      tiles.on('tileerror', () => {
        document.getElementById('mapStatus').textContent = 'Kartenkacheln nicht erreichbar · Raster-Fallback verfügbar';
      });
      tiles.addTo(topoMap);
      topoRoute = L.polyline([], { color: '#2563eb', weight: 4 }).addTo(topoMap);
      topoEvents = L.layerGroup().addTo(topoMap);
      topoCurrent = L.circleMarker([51.0, 10.0], {
        radius: 8, color: '#fff', weight: 2, fillColor: '#16a34a', fillOpacity: 0, opacity: 0
      }).addTo(topoMap);
      setTimeout(() => topoMap.invalidateSize(), 0);
      updateTopoMap();
    }

    function setMapFollow(enabled) {
      followMap = enabled;
      document.getElementById('mapFollow').textContent = 'Position folgen: ' + (followMap ? 'Ein' : 'Aus');
      if (followMap) {
        topoFitted = false;
        updateTopoMap();
      }
    }

    function loadTopoMap() {
      if (topoMap) {
        setTimeout(() => topoMap.invalidateSize(), 0);
        updateTopoMap();
        return;
      }
      if (window.L) {
        createTopoMap();
        return;
      }
      if (leafletLoading) return;
      leafletLoading = true;
      document.getElementById('mapStatus').textContent = 'Lade topografische Online-Karte …';
      const css = document.createElement('link');
      css.rel = 'stylesheet';
      css.href = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';
      document.head.appendChild(css);
      const script = document.createElement('script');
      script.src = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
      script.onload = () => { leafletLoading = false; createTopoMap(); };
      script.onerror = () => { leafletLoading = false; showMapFallback('Kein Internet · lokale Rasteransicht aktiv'); };
      document.head.appendChild(script);
    }

    function selectView(view) {
      const showMap = view === 'map';
      const showSettings = view === 'settings';
      const showMonitoring = !showMap && !showSettings;
      document.body.classList.toggle('settings-active', showSettings);
      document.querySelectorAll('.monitoring-view').forEach(element => element.classList.toggle('hidden', !showMonitoring));
      document.getElementById('mapView').classList.toggle('hidden', !showMap);
      document.getElementById('settingsView').classList.toggle('hidden', !showSettings);
      document.getElementById('monitoringTab').classList.toggle('active', showMonitoring);
      document.getElementById('mapTab').classList.toggle('active', showMap);
      document.getElementById('settingsTab').classList.toggle('active', showSettings);
      if (showMap) {
        loadTopoMap();
        refreshTrack();
      }
      if (showSettings) {
        renderSettings(window.lastStatusData || {});
        try { loadCropSuggestions(); } catch (e) {}
      }
    }

    function drawTrack(data = lastTrackData) {
      const canvas = document.getElementById('trackCanvas');
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const width = Math.max(320, Math.round(rect.width));
      const height = Math.max(280, Math.round(rect.height));
      canvas.width = width * dpr;
      canvas.height = height * dpr;
      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = '#e5e7eb';
      ctx.fillRect(0, 0, width, height);

      const positions = [...(data.points || [])];
      if (data.current && data.current.fix) positions.push(data.current);
      (data.events || []).forEach(event => positions.push(event));
      if (!positions.length) {
        ctx.fillStyle = '#4b5563';
        ctx.font = '600 15px system-ui';
        ctx.textAlign = 'center';
        ctx.fillText('Noch keine GNSS-Position vorhanden', width / 2, height / 2);
        document.getElementById('trackInfo').textContent = 'Warte auf GNSS-Daten';
        return;
      }

      const refLat = positions.reduce((sum, point) => sum + Number(point.latitude), 0) / positions.length;
      const refLon = positions.reduce((sum, point) => sum + Number(point.longitude), 0) / positions.length;
      const metersPerLat = 111320;
      const metersPerLon = 111320 * Math.cos(refLat * Math.PI / 180);
      const local = point => ({ x: (Number(point.longitude) - refLon) * metersPerLon, y: (Number(point.latitude) - refLat) * metersPerLat });
      const localPositions = positions.map(local);
      let minX = Math.min(...localPositions.map(point => point.x));
      let maxX = Math.max(...localPositions.map(point => point.x));
      let minY = Math.min(...localPositions.map(point => point.y));
      let maxY = Math.max(...localPositions.map(point => point.y));
      const minSpan = 60;
      if (maxX - minX < minSpan) { const pad = (minSpan - maxX + minX) / 2; minX -= pad; maxX += pad; }
      if (maxY - minY < minSpan) { const pad = (minSpan - maxY + minY) / 2; minY -= pad; maxY += pad; }
      const margin = 30;
      const scale = Math.min((width - margin * 2) / (maxX - minX), (height - margin * 2) / (maxY - minY));
      const toCanvas = point => {
        const value = local(point);
        return { x: margin + (value.x - minX) * scale, y: height - margin - (value.y - minY) * scale };
      };
      const gridMeters = scale > 5 ? 10 : (scale > 1 ? 25 : 50);
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 1;
      for (let x = Math.floor(minX / gridMeters) * gridMeters; x <= maxX; x += gridMeters) {
        const px = margin + (x - minX) * scale;
        ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, height); ctx.stroke();
      }
      for (let y = Math.floor(minY / gridMeters) * gridMeters; y <= maxY; y += gridMeters) {
        const py = height - margin - (y - minY) * scale;
        ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(width, py); ctx.stroke();
      }

      const route = data.points || [];
      if (route.length > 1) {
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        route.forEach((point, index) => {
          const pos = toCanvas(point);
          if (index === 0) ctx.moveTo(pos.x, pos.y);
          else ctx.lineTo(pos.x, pos.y);
        });
        ctx.stroke();
      }
      (data.events || []).forEach(event => {
        const pos = toCanvas(event);
        ctx.fillStyle = '#dc2626';
        ctx.beginPath(); ctx.arc(pos.x, pos.y, 6, 0, Math.PI * 2); ctx.fill();
      });
      if (data.current && data.current.fix) {
        const pos = toCanvas(data.current);
        ctx.fillStyle = '#16a34a';
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(pos.x, pos.y, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      }
      ctx.fillStyle = '#111827';
      ctx.font = '700 13px system-ui';
      ctx.textAlign = 'center';
      ctx.fillText('N', width - 22, 22);
      ctx.beginPath(); ctx.moveTo(width - 22, 28); ctx.lineTo(width - 28, 40); ctx.lineTo(width - 16, 40); ctx.closePath(); ctx.fill();
      ctx.strokeStyle = '#111827';
      ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(18, height - 18); ctx.lineTo(18 + gridMeters * scale, height - 18); ctx.stroke();
      ctx.fillStyle = '#111827';
      ctx.textAlign = 'left';
      ctx.font = '600 12px system-ui';
      ctx.fillText(`${gridMeters} m`, 18, height - 24);
      document.getElementById('trackInfo').textContent = `${route.length} Spurpunkte · ${(data.events || []).length} Sensor-Marker`;
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

    async function fetchStatusNow() {
      const res = await fetchWithTimeout('/api/status', {}, 2500);
      if (!res.ok) throw new Error('HTTP ' + res.status);
      const data = await res.json();
      window.lastStatusData = data;
      return data;
    }

    async function runRs485Test() {
      if (rs485TestRunning) return;
      const button = document.getElementById('rs485Test');
      const status = document.getElementById('rs485TestStatus');
      rs485TestRunning = true;
      button.disabled = true;
      status.className = 'rs485-test-status warn';
      status.textContent = 'RS485 wird geprüft ...';

      try {
        let data = await fetchStatusNow();
        const startBytes = Number((data.gnss || {}).byte_count || 0);
        updateRs485Diagnostics(data, 0);

        for (let i = 0; i < 4; i++) {
          await new Promise(resolve => setTimeout(resolve, 900));
          data = await fetchStatusNow();
          const delta = Number((data.gnss || {}).byte_count || 0) - startBytes;
          updateRs485Diagnostics(data, Math.max(0, delta));
          status.textContent = `RS485 wird geprüft ... ${i + 1}/4`;
        }

        const gnss = data.gnss || {};
        const byteDelta = Math.max(0, Number(gnss.byte_count || 0) - startBytes);
        const diagnosis = rs485Diagnosis(gnss, byteDelta);
        status.className = `rs485-test-status ${diagnosis.className}`;
        status.textContent = diagnosis.text;
      } catch (err) {
        status.className = 'rs485-test-status fail';
        status.textContent = 'RS485-Test fehlgeschlagen: API nicht erreichbar.';
      } finally {
        rs485TestRunning = false;
        button.disabled = false;
      }
    }

    async function runRs485BaudScan() {
      if (rs485TestRunning) return;
      const button = document.getElementById('rs485BaudScan');
      const status = document.getElementById('rs485TestStatus');
      const grid = document.getElementById('rs485TestGrid');
      const raw = document.getElementById('rs485RawPreview');
      rs485TestRunning = true;
      button.disabled = true;
      status.className = 'rs485-test-status warn';
      status.textContent = 'Baudraten werden gescannt ...';

      try {
        const res = await fetchWithTimeout('/api/rs485-scan', {}, 14000);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        const rows = (data.rates || []).map(item =>
          renderMetric(`${item.baud} Baud`, `${item.bytes || 0} Bytes`, item.preview || '-')
        );
        grid.innerHTML = rows.join('');
        raw.textContent = data.best_preview || 'Keine Rohdaten.';
        const liveSeen = Number(data.live_bytes_before || 0) > 0 || Number(data.live_last_byte_age_before_ms) >= 0;
        if (data.ok) {
          status.className = 'rs485-test-status warn';
          status.textContent = `RS485 empfängt Bytes bei ${data.best_baud} Baud. Wenn das nicht ${data.default_baud} Baud ist, muss die GNSS-Baudrate angepasst werden.`;
        } else if (liveSeen) {
          status.className = 'rs485-test-status warn';
          status.textContent = `Live-Empfang sieht RS485-Bytes, der Baudscan konnte sie aber nicht sicher zuordnen. Rohdaten prüfen: Wenn kein $G... sichtbar ist, passt Datenformat/Baudrate noch nicht.`;
        } else {
          status.className = 'rs485-test-status fail';
          status.textContent = `Keine Bytes auf den getesteten Baudraten. RX GPIO${data.rx_pin}, TX GPIO${data.tx_pin}, RTS GPIO${data.rts_pin}.`;
        }
      } catch (err) {
        status.className = 'rs485-test-status fail';
        status.textContent = 'Baudscan fehlgeschlagen: API nicht erreichbar.';
      } finally {
        rs485TestRunning = false;
        button.disabled = false;
      }
    }

    async function runRs485AddressScan() {
      if (rs485TestRunning) return;
      const button = document.getElementById('rs485AddressScan');
      const status = document.getElementById('rs485TestStatus');
      const grid = document.getElementById('rs485TestGrid');
      rs485TestRunning = true;
      button.disabled = true;
      status.className = 'rs485-test-status warn';
      status.textContent = 'Modbus-Adressen werden gescannt ...';

      try {
        const res = await fetchWithTimeout('/api/rs485-address-scan', {}, 35000);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        const hits = data.hits || [];
        if (hits.length) {
          grid.innerHTML = hits.map(hit =>
            renderMetric(`Adresse ${hit.address}`, `Funktion 0x${Number(hit.function || 0).toString(16).toUpperCase()}`, `${hit.bytes || 0} Bytes · ${hit.duration_ms || 0} ms`)
          ).join('');
          status.className = 'rs485-test-status ok';
          status.textContent = `Modbus-Gerät gefunden bei ${hits.map(hit => hit.address).join(', ')} auf ${data.baud} Baud.`;
        } else {
          grid.innerHTML = [
            renderMetric('Adressscan', `${data.scanned || 0} Adressen`, `${data.duration_ms || 0} ms`),
            renderMetric('RS485 Pins', `RX GPIO${data.rx_pin}`, `TX GPIO${data.tx_pin} · RTS GPIO${data.rts_pin}`)
          ].join('');
          status.className = 'rs485-test-status fail';
          status.textContent = `Keine Modbus-Antwort auf ${data.baud} Baud gefunden.`;
        }
      } catch (err) {
        status.className = 'rs485-test-status fail';
        status.textContent = 'Adressscan fehlgeschlagen: API nicht erreichbar oder Timeout.';
      } finally {
        rs485TestRunning = false;
        button.disabled = false;
      }
    }

    async function runRs485RegisterScan() {
      if (rs485TestRunning) return;
      const button = document.getElementById('rs485RegisterScan');
      const status = document.getElementById('rs485TestStatus');
      const grid = document.getElementById('rs485TestGrid');
      rs485TestRunning = true;
      button.disabled = true;
      status.className = 'rs485-test-status warn';
      status.textContent = 'Modbus-Register werden gescannt ...';

      try {
        const res = await fetchWithTimeout('/api/rs485-register-scan', {}, 35000);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        const registers = data.registers || [];
        if (data.ok) {
          grid.innerHTML = registers.length
            ? registers.map(item =>
                renderMetric(`Reg ${item.register} (${item.hex})`, String(item.value), `${item.value_hex} · ${item.duration_ms || 0} ms`)
              ).join('')
            : renderMetric('Registerscan', `${data.responded || 0} Antworten`, 'Alle gelisteten Werte waren 0');
          status.className = 'rs485-test-status ok';
          status.textContent = `${data.responded || 0} von ${data.scanned || 0} Registern haben geantwortet.`;
        } else {
          grid.innerHTML = renderMetric('Registerscan', `${data.scanned || 0} Register`, 'Keine Antwort');
          status.className = 'rs485-test-status fail';
          status.textContent = `Keine Registerantwort bei Adresse ${data.address} auf ${data.baud} Baud.`;
        }
      } catch (err) {
        status.className = 'rs485-test-status fail';
        status.textContent = 'Registerscan fehlgeschlagen: API nicht erreichbar oder Timeout.';
      } finally {
        rs485TestRunning = false;
        button.disabled = false;
      }
    }

    async function refreshTrack() {
      if (trackBusy || !pageActive) return;
      if (isCameraStreamOpen()) return;
      trackBusy = true;
      try {
        const res = await fetchWithTimeout('/api/track');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        lastTrackData = await res.json();
        drawTrack();
        updateTopoMap();
      } catch (err) {
        document.getElementById('trackInfo').textContent = 'Live-Fahrt nicht erreichbar';
      } finally {
        trackBusy = false;
      }
    }

    async function refresh() {
      if (refreshBusy || !pageActive) return;
      if (isCameraStreamOpen()) {
        document.getElementById('error').textContent = '';
        hideDebugOverlay();
        updateLastContact();
        return;
      }
      refreshBusy = true;
      try {
        const res = await fetchWithTimeout('/api/status');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        window.lastStatusData = data;
        window.mainSignalHoldMs = data.quality_signal_hold_ms || data.main_signal_hold_ms || 1500;
        lastSuccessfulContactMs = Date.now();
        setConnectionState('online');
        updateLastContact();
        document.getElementById('title').textContent = data.device_name || data.device_id;
        document.getElementById('ip').textContent = 'IP: ' + data.ip;
        document.getElementById('version').textContent = 'Version: ' + (data.firmware_version || '-');
        document.getElementById('updated').textContent = 'Aktualisiert: ' + new Date().toLocaleTimeString();
        document.getElementById('espTemp').textContent = Number.isFinite(data.esp_temperature_c) ? data.esp_temperature_c.toFixed(1) + ' °C' : '-';
        document.getElementById('cropCurrent').textContent = data.crop_name || '-';
        document.getElementById('fieldCurrent').textContent = data.field_name || '-';
        document.getElementById('sensitivityCurrent').textContent = (data.quality_signal_hold_ms || data.main_signal_hold_ms || 0) + ' ms';
        document.getElementById('redSignalCurrent').textContent = (data.red_signal_hold_ms || 15000) + ' ms';
        lightOnState = Boolean(data.light_on);
        lightSwitchable = Boolean(data.light_switchable);
        updateLightButton();
        updateValveButtons(data.pneumatic_valves || []);
        updateCamera(data);
        if (document.activeElement !== document.getElementById('sensitivityInput')) {
          document.getElementById('sensitivityInput').value = data.quality_signal_hold_ms || data.main_signal_hold_ms || 1500;
        }
        document.getElementById('tripId').textContent = data.trip_id || '-';
        document.getElementById('filesystem').textContent = data.filesystem_ready
          ? `${Math.round((data.filesystem_used_bytes || 0) / 1024)} / ${Math.round((data.filesystem_total_bytes || 0) / 1024)} KB`
          : 'Nicht verfügbar';
        if (document.activeElement !== document.getElementById('cropInput')) {
          document.getElementById('cropInput').value = data.crop_name || '';
        }
        if (document.activeElement !== document.getElementById('fieldInput')) {
          document.getElementById('fieldInput').value = data.field_name || '';
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
        document.getElementById('gpsSatellites').textContent = (gnss.satellites !== undefined && gnss.satellites !== null) ? gnss.satellites : '-';
        document.getElementById('gnssRs485').textContent = `${gnss.last_error || '-'} · OK ${gnss.ok_count || 0} / Fehler ${gnss.error_count || 0}`;
        updateGnssWarning(gnss);
        updateMapGnssStatus(gnss);
        // minimal overview fields
        const channelsList = data.channels || [];
        const seedChannels = channelsList.filter(c => c.seed_channel);
        const activeCount = seedChannels.filter(c => c.main_signal).length;
        const rotation = data.rotation || {};
        document.getElementById('sensorStatus').textContent = `${activeCount} / ${seedChannels.length} · ${rotation.moving ? 'Dreht' : 'Dreht nicht'}`;
        document.getElementById('lastContactMini').textContent = '0 s';
        // populate settings debug
        try { renderSettings(data); } catch (e) {}
        document.getElementById('modules').innerHTML = (data.modules || []).map(module =>
          `<div class="module ${module.online ? 'online' : ''}"><strong>${escapeHtml(module.id)}</strong><br>${module.online ? 'Lokal online' : 'Vorbereitet'}</div>`
        ).join('');
        checkMainSignalAlarms(data.channels || []);
        const grid = document.getElementById('grid');
        const shownChannels = visibleChannels(data.channels || []);
        renderingGrid = true;
        grid.innerHTML = shownChannels.map(ch => card(ch, rotation)).join('');
        document.getElementById('sensorDetailsGrid').innerHTML = sensorDetails(data.channels || []);
        setTimeout(() => { renderingGrid = false; }, 80);
        document.getElementById('error').textContent = '';
        hideDebugOverlay();
      } catch (err) {
        if (isCameraStreamOpen()) {
          document.getElementById('error').textContent = '';
          hideDebugOverlay();
          updateLastContact();
          return;
        }
        updateLastContact();
        document.getElementById('error').textContent = 'Keine Verbindung zur API';
        try {
          const dbg = document.getElementById('debugOverlay');
          dbg.style.display = 'block';
          dbg.textContent = 'Refresh error: ' + (err.message || String(err));
          dbg.classList.remove('hidden');
        } catch (e) {}
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
      if (!confirm('Live-Log wirklich löschen? Bereits archivierte Fahrtdateien bleiben erhalten.')) return;
      try {
        const res = await fetchWithTimeout('/api/gps-log/clear', { method: 'POST' });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Löschen fehlgeschlagen';
      }
    }

    async function saveField() {
      const input = document.getElementById('fieldInput');
      try {
        const res = await fetchWithTimeout('/api/field', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ field_name: input.value })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Feld speichern fehlgeschlagen';
      }
    }

    async function saveSensitivitySetting() {
      const input = document.getElementById('sensitivityInput');
      const value = Number(input.value);
      try {
        const res = await fetchWithTimeout('/api/sensitivity', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ main_signal_hold_ms: value })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('error').textContent = 'Empfindlichkeit speichern fehlgeschlagen';
      }
    }

    async function testCameraConnection(card) {
      const button = card.querySelector('.cameraTest');
      const target = card.querySelector('.camera-test-status');
      const index = Number(card.dataset.cameraIndex);
      button.disabled = true;
      target.className = 'camera-test-status';
      target.textContent = 'Kamera wird getestet ...';
      try {
        const res = await fetchWithTimeout('/api/camera-test?index=' + encodeURIComponent(index), {}, 12000);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        const items = [data.substream, data.mainstream].filter(Boolean);
        const hasOk = items.some(item => item.stream_ok);
        const hasReachable = items.some(item => item.reachable);
        target.className = 'camera-test-status ' + (hasOk ? 'ok' : hasReachable ? 'warn' : 'fail');
        target.textContent = items.map(item => {
          const label = item.label || 'Stream';
          const iface = item.interface ? ` über ${item.interface}` : '';
          const status = item.reachable
            ? `HTTP ${item.http_code}${item.stream_ok ? ', MJPEG erkannt' : ', kein MJPEG'}`
            : `nicht erreichbar (${item.error || 'Timeout'})`;
          const contentType = item.content_type ? `\n  Typ: ${item.content_type}` : '';
          return `${label}${iface}: ${status}${contentType}`;
        }).join('\n');
      } catch (err) {
        target.className = 'camera-test-status fail';
        target.textContent = `Kameratest fehlgeschlagen (${err.message || 'Timeout'})`;
      } finally {
        button.disabled = false;
      }
    }

    async function saveCameraSettings(card) {
      const index = Number(card.dataset.cameraIndex);
      const host = card.querySelector('.cameraHostInput').value.trim();
      const username = card.querySelector('.cameraUserInput').value.trim();
      const passwordInput = card.querySelector('.cameraPasswordInput');
      const password = passwordInput.value.trim();
      const target = card.querySelector('.camera-test-status');
      if (!host || !username || !password) {
        target.className = 'camera-test-status fail';
        target.textContent = 'Kamera-IP, Benutzer und Passwort eintragen.';
        return;
      }
      try {
        const res = await fetchWithTimeout('/api/camera-settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ index, host, username, password })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        passwordInput.value = '';
        passwordInput.placeholder = 'Passwort gespeichert';
        target.className = 'camera-test-status ok';
        target.textContent = 'Kameraeinstellungen gespeichert.';
        await refresh();
      } catch (err) {
        target.className = 'camera-test-status fail';
        target.textContent = 'Kameraeinstellungen speichern fehlgeschlagen';
      }
    }

    async function clearCameraSettings(card) {
      const index = Number(card.dataset.cameraIndex);
      const target = card.querySelector('.camera-test-status');
      try {
        const res = await fetchWithTimeout('/api/camera-settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ index, host: '', username: '', password: '' })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        target.className = 'camera-test-status ok';
        target.textContent = 'Kamera ausgeblendet.';
        await refresh();
      } catch (err) {
        target.className = 'camera-test-status fail';
        target.textContent = 'Kamera ausblenden fehlgeschlagen';
      }
    }

    async function loadArchive() {
      const target = document.getElementById('archiveList');
      target.textContent = 'Archiv wird geladen …';
      try {
        const res = await fetchWithTimeout('/api/archive');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        target.innerHTML = (data.files || []).length
          ? data.files.map(file => `<a href="/api/archive/download?path=${encodeURIComponent(file.path)}">${escapeHtml(file.name)} (${Math.round(file.size / 1024)} KB)</a>`).join('')
          : 'Noch keine archivierten Fahrtdateien.';
      } catch (err) {
        target.textContent = 'Archiv nicht erreichbar';
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
        await loadCropSuggestions();
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Saat speichern fehlgeschlagen';
      }
    }

    async function selectCrop(name) {
      document.getElementById('cropInput').value = name;
      await saveCrop();
    }

    function formatBytes(bytes) {
      const value = Number(bytes || 0);
      if (value >= 1048576) return (value / 1048576).toFixed(1) + ' MB';
      if (value >= 1024) return Math.round(value / 1024) + ' KB';
      return value + ' B';
    }

    function formatPct(used, total) {
      if (!total) return '-';
      return Math.round((used / total) * 100) + ' %';
    }

    function formatDuration(ms) {
      const totalSeconds = Math.floor(Number(ms || 0) / 1000);
      const hours = Math.floor(totalSeconds / 3600);
      const minutes = Math.floor((totalSeconds % 3600) / 60);
      const seconds = totalSeconds % 60;
      return `${hours} h ${minutes} min ${seconds} s`;
    }

    function renderMetric(label, value, detail = '') {
      return `<div class="sensor-row system-row"><strong>${escapeHtml(label)}</strong><span>${escapeHtml(value)}</span><span>${escapeHtml(detail)}</span></div>`;
    }

    function rs485Diagnosis(gnss, byteDelta = null) {
      const bytes = Number(gnss.byte_count || 0);
      const lastByteAge = Number(gnss.last_byte_age_ms);
      const hasRecentBytes = Number.isFinite(lastByteAge) && lastByteAge >= 0 && lastByteAge <= 10000;
      const hasNewBytes = byteDelta !== null && byteDelta > 0;

      if (gnss.fix) {
        return { className: 'ok', text: `OK: GPS-Fix erkannt. Bytes gesamt: ${bytes}.` };
      }
      if (gnss.seen || gnss.last_sentence) {
        return { className: 'ok', text: `OK: GNSS-Daten erkannt, aber noch kein Fix. Fehler: ${gnss.last_error || '-'}.` };
      }
      if (hasNewBytes || hasRecentBytes || bytes > 0) {
        const deltaText = byteDelta !== null ? ` Neue Bytes im Test: ${byteDelta}.` : '';
        return { className: 'warn', text: `RS485 empfängt Daten, aber noch keinen gültigen GPS-Satz.${deltaText} Fehler: ${gnss.last_error || '-'}.` };
      }
      return { className: 'fail', text: `Keine RS485-Bytes empfangen. Fehler: ${gnss.last_error || '-'}.` };
    }

    function updateRs485Diagnostics(data, byteDelta = null) {
      const gnss = data.gnss || {};
      const grid = document.getElementById('rs485TestGrid');
      const raw = document.getElementById('rs485RawPreview');
      const hex = document.getElementById('rs485HexPreview');
      if (!grid || !raw || !hex) return;

      grid.innerHTML = [
        renderMetric('Schnittstelle', gnss.interface || '-', `${gnss.baud || '-'} Baud · Adresse ${gnss.modbus_address || '-'}`),
        renderMetric('Empfangene Bytes', String(gnss.byte_count || 0), byteDelta !== null ? `+${byteDelta} im Test` : ''),
        renderMetric('Letztes Byte', Number(gnss.last_byte_age_ms) >= 0 ? `${gnss.last_byte_age_ms} ms` : 'Noch nie', ''),
        renderMetric('Letzter Fehler', gnss.last_error || '-', `OK ${gnss.ok_count || 0} / Fehler ${gnss.error_count || 0}`),
        renderMetric('GPS-Satz', gnss.last_sentence ? 'Erkannt' : 'Noch keiner', gnss.fix ? 'Fix vorhanden' : 'Kein Fix')
      ].join('');
      raw.textContent = gnss.raw_preview || 'Keine Rohdaten.';
      hex.textContent = gnss.raw_hex_preview || 'Keine HEX-Daten.';

      if (!rs485TestRunning) {
        const status = document.getElementById('rs485TestStatus');
        const diagnosis = rs485Diagnosis(gnss, byteDelta);
        status.className = `rs485-test-status ${diagnosis.className}`;
        status.textContent = diagnosis.text;
      }
    }

    function renderSettings(data) {
      try {
        const heapTotal = Number(data.heap_total_bytes || 0);
        const heapFree = Number(data.heap_free_bytes || 0);
        const psramTotal = Number(data.psram_total_bytes || 0);
        const psramFree = Number(data.psram_free_bytes || 0);
        const fsTotal = Number(data.filesystem_total_bytes || 0);
        const fsUsed = Number(data.filesystem_used_bytes || 0);
        const sketchSize = Number(data.sketch_size_bytes || 0);
        const sketchFree = Number(data.sketch_free_space_bytes || 0);
        document.getElementById('systemLoadGrid').innerHTML = [
          renderMetric('Heap', formatBytes(heapTotal - heapFree) + ' / ' + formatBytes(heapTotal), formatPct(heapTotal - heapFree, heapTotal)),
          renderMetric('Heap Minimum frei', formatBytes(data.heap_min_free_bytes), ''),
          renderMetric('PSRAM', formatBytes(psramTotal - psramFree) + ' / ' + formatBytes(psramTotal), formatPct(psramTotal - psramFree, psramTotal)),
          renderMetric('LittleFS', data.filesystem_ready ? formatBytes(fsUsed) + ' / ' + formatBytes(fsTotal) : 'Nicht bereit', data.filesystem_ready ? formatPct(fsUsed, fsTotal) : ''),
          renderMetric('Sketch Flash', formatBytes(sketchSize) + ' / ' + formatBytes(sketchSize + sketchFree), formatPct(sketchSize, sketchSize + sketchFree)),
          renderMetric('Uptime', formatDuration(data.uptime_ms), ''),
          renderMetric('Boots / Reset', String(data.boot_counter || 0), data.reset_reason || '-'),
          renderMetric('Ethernet', data.ethernet_ready ? (data.ethernet_link ? 'Link aktiv' : 'Kein Link') : 'Nicht bereit', data.ethernet_ip || '-')
        ].join('');
      } catch (e) {
        document.getElementById('systemLoadGrid').textContent = 'Nicht verfügbar';
      }
      updateRs485Diagnostics(data);
      try {
        const gnss = data.gnss || {};
        const info = {
          health: gnss.health,
          warning: gnss.warning,
          last_error: gnss.last_error,
          last_sentence: gnss.last_sentence,
          raw_preview: gnss.raw_preview,
          raw_hex_preview: gnss.raw_hex_preview,
          byte_count: gnss.byte_count,
          last_byte_age_ms: gnss.last_byte_age_ms,
          poll_count: gnss.poll_count,
          ok_count: gnss.ok_count,
          error_count: gnss.error_count
        };
        document.getElementById('gnssDebug').textContent = JSON.stringify(info, null, 2);
      } catch (e) {
        document.getElementById('gnssDebug').textContent = 'Nicht verfügbar';
      }
    }

    async function loadCropSuggestions() {
      try {
        const res = await fetchWithTimeout('/api/crops');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const list = await res.json();
        const datalist = document.getElementById('cropSuggestions');
        const suggestions = Array.isArray(list) ? list : [];
        datalist.innerHTML = suggestions.map(v => `<option value="${escapeHtml(v)}"></option>`).join('');
        const quick = document.getElementById('cropQuickList');
        const current = document.getElementById('cropInput').value;
        quick.innerHTML = suggestions.map(v =>
          `<button type="button" class="crop-chip ${v === current ? 'active' : ''}" data-name="${escapeHtml(v)}">${escapeHtml(v)}</button>`
        ).join('');
        quick.querySelectorAll('.crop-chip').forEach(btn => btn.addEventListener('click', e => {
          selectCrop(e.currentTarget.getAttribute('data-name'));
        }));
        const container = document.getElementById('cropSuggestionsList');
        container.innerHTML = suggestions.length ? suggestions.map(v => `<div class="suggestion-row"><span>${escapeHtml(v)}</span><button data-name="${escapeHtml(v)}" class="removeCrop secondary">Entfernen</button></div>`).join('') : 'Keine Vorschläge.';
        container.querySelectorAll('.removeCrop').forEach(btn => btn.addEventListener('click', async e => {
          const name = e.currentTarget.getAttribute('data-name');
          try {
            const r = await fetchWithTimeout('/api/crops', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'remove', name }) });
            if (!r.ok) throw new Error('HTTP ' + r.status);
            await loadCropSuggestions();
          } catch (err) { document.getElementById('error').textContent = 'Vorschlag entfernen fehlgeschlagen'; }
        }));
      } catch (err) {
        document.getElementById('cropSuggestionsList').textContent = 'Vorschläge nicht erreichbar';
      }
    }

    document.getElementById('cropAddBtn').addEventListener('click', async () => {
      const input = document.getElementById('cropAddInput');
      const name = input.value.trim();
      if (!name) return;
      try {
        const res = await fetchWithTimeout('/api/crops', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'add', name }) });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        input.value = '';
        await loadCropSuggestions();
        document.getElementById('cropInput').value = name;
        await saveCrop();
      } catch (err) {
        document.getElementById('error').textContent = 'Vorschlag hinzufügen fehlgeschlagen';
      }
    });

    async function toggleLight() {
      if (actionBusy) return;
      actionBusy = true;
      try {
        const nextState = !lightOnState;
        const res = await fetchWithTimeout('/api/output', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ channel: 1, on: nextState })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        lightOnState = nextState;
        lightSwitchable = true;
        updateLightButton();
        await refresh();
      } catch (err) {
        lightSwitchable = false;
        updateLightButton();
        document.getElementById('error').textContent = 'Licht schalten fehlgeschlagen';
      } finally {
        actionBusy = false;
      }
    }

    document.getElementById('gpsStart').addEventListener('click', startGps);
    document.getElementById('gpsStop').addEventListener('click', stopGps);
    document.getElementById('gpsClear').addEventListener('click', clearGpsLog);
    document.getElementById('cropSave').addEventListener('click', saveCrop);
    document.getElementById('fieldSave').addEventListener('click', saveField);
    document.getElementById('sensitivitySave').addEventListener('click', saveSensitivitySetting);
    document.getElementById('archiveRefresh').addEventListener('click', loadArchive);
    document.getElementById('rs485Test').addEventListener('click', runRs485Test);
    document.getElementById('rs485BaudScan').addEventListener('click', runRs485BaudScan);
    document.getElementById('rs485AddressScan').addEventListener('click', runRs485AddressScan);
    document.getElementById('rs485RegisterScan').addEventListener('click', runRs485RegisterScan);
    document.getElementById('alarmEnable').addEventListener('click', enableAlarm);
    document.getElementById('alarmAck').addEventListener('click', acknowledgeAlarm);
    document.getElementById('monitoringTab').addEventListener('click', () => selectView('monitoring'));
    document.getElementById('mapTab').addEventListener('click', () => selectView('map'));
    document.getElementById('settingsTab').addEventListener('click', () => selectView('settings'));
    document.getElementById('lightOn').addEventListener('click', () => toggleLight());
    document.getElementById('mapFollow').addEventListener('click', () => setMapFollow(!followMap));
    window.addEventListener('pagehide', () => {
      pageActive = false;
      document.querySelectorAll('.camera-panel img').forEach(image => image.removeAttribute('src'));
    });
    document.getElementById('grid').addEventListener('click', event => {
      if (event.target.tagName !== 'SUMMARY') return;
      const details = event.target.closest('details');
      const cardEl = event.target.closest('.card');
      const channel = cardEl ? cardEl.dataset.channel : null;
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
      const cardEl = event.target.closest('.card');
      const channel = cardEl ? cardEl.dataset.channel : null;
      if (!channel) return;
      if (event.target.open) {
        openDetailChannels.add(channel);
      } else {
        openDetailChannels.delete(channel);
      }
    }, true);
    refresh();
    loadCropSuggestions();
    refreshTrack();
    window.addEventListener('resize', () => drawTrack());
    connectionTimer = setInterval(updateLastContact, 1000);
    setInterval(refresh, 1000);
    setInterval(refreshTrack, 3000);
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

uint16_t listenForRs485Bytes(uint32_t listenMs, char *preview, size_t previewSize) {
  uint16_t count = 0;
  const uint32_t startMs = millis();
  if (preview != nullptr && previewSize > 0) {
    preview[0] = '\0';
  }

  while (millis() - startMs < listenMs) {
    while (gnssSerial.available()) {
      const char c = static_cast<char>(gnssSerial.read());
      count++;
      noteGnssReceivedByte(c);
      if (preview != nullptr && previewSize > 1) {
        const size_t length = strnlen(preview, previewSize);
        if (length + 1 < previewSize) {
          preview[length] = (c >= 32 && c <= 126) ? c : '.';
          preview[length + 1] = '\0';
        }
      }
    }
    delay(1);
  }

  return count;
}

void restartGnssSerial(uint32_t baud) {
  gnssSerial.end();
  delay(20);
  digitalWrite(GNSS_RS485_DE_RE_PIN, LOW);
  gnssSerial.begin(baud, SERIAL_8N1, GNSS_RS485_RX_PIN, GNSS_RS485_TX_PIN);
  gnssDirectLineLength = 0;
}

void handleApiRs485Scan() {
  JsonDocument doc;
  JsonArray rates = doc["rates"].to<JsonArray>();
  uint32_t bestBaud = 0;
  uint16_t bestCount = 0;
  char bestPreview[64] = "";
  const uint32_t liveBytesBefore = gnss.byteCount;
  const uint32_t liveLastByteAgeBefore = gnss.lastByteMs > 0 ? millis() - gnss.lastByteMs : UINT32_MAX;

  digitalWrite(GNSS_RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < sizeof(GNSS_BAUD_SCAN_RATES) / sizeof(GNSS_BAUD_SCAN_RATES[0]); i++) {
    const uint32_t baud = GNSS_BAUD_SCAN_RATES[i];
    restartGnssSerial(baud);
    listenForRs485Bytes(250, nullptr, 0);

    char preview[64] = "";
    const uint16_t count = listenForRs485Bytes(1600, preview, sizeof(preview));
    JsonObject result = rates.add<JsonObject>();
    result["baud"] = baud;
    result["bytes"] = count;
    result["preview"] = preview;

    if (count > bestCount) {
      bestBaud = baud;
      bestCount = count;
      strncpy(bestPreview, preview, sizeof(bestPreview) - 1);
      bestPreview[sizeof(bestPreview) - 1] = '\0';
    }
  }

  restartGnssSerial(GNSS_MODBUS_BAUD);

  doc["ok"] = bestCount > 0;
  doc["live_bytes_before"] = liveBytesBefore;
  doc["live_bytes_now"] = gnss.byteCount;
  doc["live_last_byte_age_before_ms"] = liveLastByteAgeBefore == UINT32_MAX ? -1 : static_cast<int32_t>(liveLastByteAgeBefore);
  doc["best_baud"] = bestBaud;
  doc["best_bytes"] = bestCount;
  doc["best_preview"] = bestPreview;
  doc["default_baud"] = GNSS_MODBUS_BAUD;
  doc["rx_pin"] = GNSS_RS485_RX_PIN;
  doc["tx_pin"] = GNSS_RS485_TX_PIN;
  doc["rts_pin"] = GNSS_RS485_DE_RE_PIN;

  String json;
  serializeJson(doc, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

bool probeModbusAddress(uint8_t address, uint8_t &functionCode, uint8_t &byteCount, uint32_t &durationMs) {
  const uint32_t startMs = millis();
  functionCode = 0;
  byteCount = 0;

  while (gnssSerial.available()) {
    noteGnssReceivedByte(static_cast<char>(gnssSerial.read()));
  }

  uint8_t request[8];
  request[0] = address;
  request[1] = GNSS_MODBUS_FUNCTION_READ_HOLDING;
  request[2] = highByte(GNSS_SCAN_START_REGISTER);
  request[3] = lowByte(GNSS_SCAN_START_REGISTER);
  request[4] = 0x00;
  request[5] = 0x01;
  const uint16_t crc = modbusCrc(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  digitalWrite(GNSS_RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(80);
  gnssSerial.write(request, sizeof(request));
  gnssSerial.flush();
  delayMicroseconds(120);
  digitalWrite(GNSS_RS485_DE_RE_PIN, LOW);

  uint8_t response[16];
  size_t index = 0;
  const uint32_t deadline = millis() + 90;
  while (millis() < deadline && index < sizeof(response)) {
    while (gnssSerial.available() && index < sizeof(response)) {
      const char c = static_cast<char>(gnssSerial.read());
      noteGnssReceivedByte(c);
      response[index++] = static_cast<uint8_t>(c);
    }
    delay(1);
  }

  durationMs = millis() - startMs;
  if (index < 5) {
    return false;
  }

  const uint16_t receivedCrc = static_cast<uint16_t>(response[index - 2]) | (static_cast<uint16_t>(response[index - 1]) << 8);
  if (modbusCrc(response, index - 2) != receivedCrc || response[0] != address) {
    return false;
  }

  functionCode = response[1];
  byteCount = response[2];
  return true;
}

bool probeModbusRegister(uint16_t startRegister, uint16_t &value, uint32_t &durationMs) {
  const uint32_t startMs = millis();
  value = 0;

  while (gnssSerial.available()) {
    noteGnssReceivedByte(static_cast<char>(gnssSerial.read()));
  }

  uint8_t request[8];
  request[0] = GNSS_MODBUS_ADDRESS;
  request[1] = GNSS_MODBUS_FUNCTION_READ_HOLDING;
  request[2] = highByte(startRegister);
  request[3] = lowByte(startRegister);
  request[4] = 0x00;
  request[5] = 0x01;
  const uint16_t crc = modbusCrc(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  digitalWrite(GNSS_RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(80);
  gnssSerial.write(request, sizeof(request));
  gnssSerial.flush();
  delayMicroseconds(120);
  digitalWrite(GNSS_RS485_DE_RE_PIN, LOW);

  uint8_t response[8];
  size_t index = 0;
  const uint32_t deadline = millis() + 90;
  while (millis() < deadline && index < sizeof(response)) {
    while (gnssSerial.available() && index < sizeof(response)) {
      const char c = static_cast<char>(gnssSerial.read());
      noteGnssReceivedByte(c);
      response[index++] = static_cast<uint8_t>(c);
    }
    delay(1);
  }

  durationMs = millis() - startMs;
  if (index < 7) {
    return false;
  }

  const uint16_t receivedCrc = static_cast<uint16_t>(response[index - 2]) | (static_cast<uint16_t>(response[index - 1]) << 8);
  if (modbusCrc(response, index - 2) != receivedCrc || response[0] != GNSS_MODBUS_ADDRESS ||
      response[1] != GNSS_MODBUS_FUNCTION_READ_HOLDING || response[2] != 2) {
    return false;
  }

  value = (static_cast<uint16_t>(response[3]) << 8) | response[4];
  return true;
}

void handleApiRs485AddressScan() {
  restartGnssSerial(GNSS_MODBUS_BAUD);

  JsonDocument doc;
  JsonArray hits = doc["hits"].to<JsonArray>();
  const uint32_t startMs = millis();
  uint16_t scanned = 0;

  for (uint16_t address = 1; address <= 247; address++) {
    uint8_t functionCode = 0;
    uint8_t byteCount = 0;
    uint32_t durationMs = 0;
    scanned++;
    if (probeModbusAddress(static_cast<uint8_t>(address), functionCode, byteCount, durationMs)) {
      JsonObject hit = hits.add<JsonObject>();
      hit["address"] = address;
      hit["function"] = functionCode;
      hit["bytes"] = byteCount;
      hit["duration_ms"] = durationMs;
    }
    if ((address % 8) == 0) {
      esp_task_wdt_reset();
      server.client().flush();
    }
  }

  restartGnssSerial(GNSS_MODBUS_BAUD);
  doc["ok"] = hits.size() > 0;
  doc["baud"] = GNSS_MODBUS_BAUD;
  doc["scanned"] = scanned;
  doc["duration_ms"] = millis() - startMs;
  doc["rx_pin"] = GNSS_RS485_RX_PIN;
  doc["tx_pin"] = GNSS_RS485_TX_PIN;
  doc["rts_pin"] = GNSS_RS485_DE_RE_PIN;

  String json;
  serializeJson(doc, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleApiRs485RegisterScan() {
  restartGnssSerial(GNSS_MODBUS_BAUD);

  JsonDocument doc;
  JsonArray registers = doc["registers"].to<JsonArray>();
  const uint32_t startMs = millis();
  uint16_t responded = 0;
  uint16_t scanned = 0;

  for (uint16_t reg = 0; reg <= 255; reg++) {
    uint16_t value = 0;
    uint32_t durationMs = 0;
    scanned++;
    if (probeModbusRegister(reg, value, durationMs)) {
      responded++;
      if (reg < 32 || value != 0) {
        JsonObject item = registers.add<JsonObject>();
        item["register"] = reg;
        item["hex"] = "0x" + String(reg, HEX);
        item["value"] = value;
        item["value_hex"] = "0x" + String(value, HEX);
        item["duration_ms"] = durationMs;
      }
    }
    if ((reg % 8) == 0) {
      esp_task_wdt_reset();
    }
  }

  restartGnssSerial(GNSS_MODBUS_BAUD);
  doc["ok"] = responded > 0;
  doc["baud"] = GNSS_MODBUS_BAUD;
  doc["address"] = GNSS_MODBUS_ADDRESS;
  doc["scanned"] = scanned;
  doc["responded"] = responded;
  doc["duration_ms"] = millis() - startMs;

  String json;
  serializeJson(doc, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void addCameraTestResult(JsonObject target, uint8_t cameraIndex, const char *label, const char *path) {
  const uint32_t startMs = millis();
  EthernetClient client;
  client.setTimeout(HIKVISION_CAMERA_TEST_TIMEOUT_MS);

  bool connected = ethernetReady && cameraConfigured(cameraIndex) && client.connect(cameraHosts[cameraIndex], 80);
  int httpCode = -1;
  String statusLine;
  String contentType;
  String error;

  if (connected) {
    client.print("GET ");
    client.print(path);
    client.print(" HTTP/1.1\r\nHost: ");
    client.print(cameraHosts[cameraIndex]);
    client.print("\r\nAuthorization: ");
    client.print(cameraAuthHeader(cameraIndex));
    client.print("\r\nConnection: close\r\n\r\n");

    String line;
    bool firstLine = true;
    bool headersDone = false;
    while (millis() - startMs < HIKVISION_CAMERA_TEST_TIMEOUT_MS && client.connected() && !headersDone) {
      while (client.available() && !headersDone) {
        const char c = static_cast<char>(client.read());
        if (c == '\r') {
          continue;
        }
        if (c == '\n') {
          if (firstLine) {
            statusLine = line;
            const int firstSpace = statusLine.indexOf(' ');
            if (firstSpace >= 0 && statusLine.length() >= firstSpace + 4) {
              httpCode = statusLine.substring(firstSpace + 1, firstSpace + 4).toInt();
            }
            firstLine = false;
          } else if (line.length() == 0) {
            headersDone = true;
          } else {
            String lowerLine = line;
            lowerLine.toLowerCase();
            if (lowerLine.startsWith("content-type:")) {
              contentType = line.substring(line.indexOf(':') + 1);
              contentType.trim();
            }
          }
          line = "";
        } else if (line.length() < 160) {
          line += c;
        }
      }
      if (!headersDone) {
        delay(5);
      }
    }

    if (httpCode < 0) {
      error = "no_http_header";
    }
  } else {
    error = !ethernetReady ? "ethernet_not_ready" : cameraConfigured(cameraIndex) ? "connect_failed" : "camera_not_configured";
  }

  const uint32_t durationMs = millis() - startMs;
  client.stop();

  contentType.toLowerCase();
  const bool reachable = connected && httpCode > 0;
  const bool streamOk = httpCode == 200 &&
                        (contentType.indexOf("multipart") >= 0 || contentType.indexOf("image/jpeg") >= 0);

  target["label"] = label;
  target["interface"] = "ethernet";
  target["reachable"] = reachable;
  target["stream_ok"] = streamOk;
  target["http_code"] = httpCode;
  target["status_line"] = statusLine;
  target["content_type"] = contentType;
  target["duration_ms"] = durationMs;
  if (error.length() > 0) {
    target["error"] = error;
  }
}

void handleApiCameraTest() {
  const int indexArg = server.hasArg("index") ? server.arg("index").toInt() : 0;
  if (indexArg < 0 || indexArg >= CAMERA_COUNT) {
    server.send(400, "application/json", "{\"error\":\"invalid_camera_index\"}");
    return;
  }
  const uint8_t cameraIndex = static_cast<uint8_t>(indexArg);
  JsonDocument doc;
  doc["camera_name"] = "Kamera " + String(cameraIndex + 1);
  doc["camera_index"] = cameraIndex;
  addCameraTestResult(doc["substream"].to<JsonObject>(), cameraIndex, "Substream 102", HIKVISION_CAMERA_SUB_STREAM_PATH);
  addCameraTestResult(doc["mainstream"].to<JsonObject>(), cameraIndex, "Mainstream 101", HIKVISION_CAMERA_MAIN_STREAM_PATH);

  String json;
  serializeJson(doc, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleCameraProxy(uint8_t cameraIndex, const char *cameraPath) {
  if (!ethernetReady) {
    server.send(503, "text/plain; charset=utf-8", "Ethernet nicht bereit");
    return;
  }
  if (!cameraConfigured(cameraIndex)) {
    server.send(404, "text/plain; charset=utf-8", "Kamera nicht konfiguriert");
    return;
  }

  EthernetClient camera;
  camera.setTimeout(HIKVISION_CAMERA_TEST_TIMEOUT_MS);
  if (!camera.connect(cameraHosts[cameraIndex], 80)) {
    server.send(502, "text/plain; charset=utf-8", "Kamera nicht erreichbar");
    return;
  }

  camera.print("GET ");
  camera.print(cameraPath);
  camera.print(" HTTP/1.1\r\nHost: ");
  camera.print(cameraHosts[cameraIndex]);
  camera.print("\r\nAuthorization: ");
  camera.print(cameraAuthHeader(cameraIndex));
  camera.print("\r\nConnection: close\r\n\r\n");

  const uint32_t headerStartMs = millis();
  String line;
  String statusLine;
  String contentType = "multipart/x-mixed-replace";
  int httpCode = -1;
  bool firstLine = true;
  bool headersDone = false;
  while (millis() - headerStartMs < HIKVISION_CAMERA_TEST_TIMEOUT_MS && camera.connected() && !headersDone) {
    while (camera.available() && !headersDone) {
      const char c = static_cast<char>(camera.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        if (firstLine) {
          statusLine = line;
          const int firstSpace = statusLine.indexOf(' ');
          if (firstSpace >= 0 && statusLine.length() >= firstSpace + 4) {
            httpCode = statusLine.substring(firstSpace + 1, firstSpace + 4).toInt();
          }
          firstLine = false;
        } else if (line.length() == 0) {
          headersDone = true;
        } else {
          String lowerLine = line;
          lowerLine.toLowerCase();
          if (lowerLine.startsWith("content-type:")) {
            contentType = line.substring(line.indexOf(':') + 1);
            contentType.trim();
          }
        }
        line = "";
      } else if (line.length() < 200) {
        line += c;
      }
    }
    if (!headersDone) {
      delay(5);
    }
  }

  if (!headersDone || httpCode != 200) {
    camera.stop();
    server.send(502, "text/plain; charset=utf-8", "Kamera Stream Fehler: " + statusLine);
    return;
  }

  WiFiClient browser = server.client();
  browser.print("HTTP/1.1 200 OK\r\nCache-Control: no-store\r\nContent-Type: ");
  browser.print(contentType);
  browser.print("\r\n\r\n");

  uint8_t buffer[1024];
  uint32_t lastDataMs = millis();
  while (browser.connected() && camera.connected()) {
    const int available = camera.available();
    if (available > 0) {
      const size_t count = camera.read(buffer, min(available, static_cast<int>(sizeof(buffer))));
      if (count > 0) {
        browser.write(buffer, count);
        lastDataMs = millis();
      }
    } else if (millis() - lastDataMs > 5000) {
      break;
    } else {
      delay(2);
    }
    esp_task_wdt_reset();
  }

  camera.stop();
  browser.stop();
}

void handleApiAlarmAck() {
  uint8_t cleared = 0;
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    if (channels[i].latchedAlarm) {
      channels[i].latchedAlarm = false;
      cleared++;
    }
  }

  JsonDocument response;
  response["ok"] = true;
  response["cleared"] = cleared;
  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
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

String loadCropSuggestionsJson() {
  Preferences uiPreferences;
  uiPreferences.begin("ui", false);
  String stored = uiPreferences.getString("crop_suggestions", "");
  uiPreferences.end();

  JsonDocument doc;
  if (stored.length() == 0 || deserializeJson(doc, stored) || !doc.is<JsonArray>() || doc.size() == 0) {
    return DEFAULT_CROP_SUGGESTIONS_JSON;
  }
  return stored;
}

String normalizeCropSuggestionName(const String &rawName) {
  char buffer[CROP_NAME_LENGTH];
  rawName.toCharArray(buffer, sizeof(buffer));
  sanitizeCropName(buffer);
  return String(buffer);
}

void handleApiCrops() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", loadCropSuggestionsJson());
}

void handleApiCropsPost() {
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

  const String action = doc["action"] | "";
  const String name = normalizeCropSuggestionName(doc["name"] | "");
  if (name.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"missing_name\"}");
    return;
  }

  String stored = loadCropSuggestionsJson();
  JsonDocument arrDoc;
  if (deserializeJson(arrDoc, stored) || !arrDoc.is<JsonArray>()) {
    deserializeJson(arrDoc, DEFAULT_CROP_SUGGESTIONS_JSON);
  }
  JsonArray arr = arrDoc.as<JsonArray>();

  bool changed = false;
  if (action == "add") {
    bool exists = false;
    for (JsonVariant value : arr) {
      if (String(value.as<const char *>()) == name) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      arr.add(name);
      changed = true;
    }
  } else if (action == "remove") {
    JsonDocument newDoc;
    JsonArray newArr = newDoc.to<JsonArray>();
    for (JsonVariant value : arr) {
      const String existing = String(value.as<const char *>());
      if (existing != name) {
        newArr.add(existing);
      }
    }
    String out;
    serializeJson(newDoc, out);
    Preferences uiPreferences;
    uiPreferences.begin("ui", false);
    uiPreferences.putString("crop_suggestions", out);
    uiPreferences.end();
    server.send(200, "application/json", out);
    return;
  } else {
    server.send(400, "application/json", "{\"error\":\"invalid_action\"}");
    return;
  }

  String out;
  if (changed) {
    serializeJson(arr, out);
    Preferences uiPreferences;
    uiPreferences.begin("ui", false);
    uiPreferences.putString("crop_suggestions", out);
    uiPreferences.end();
  } else {
    out = stored;
  }
  server.send(200, "application/json", out);
}

void handleApiField() {
  const String body = server.arg("plain");
  JsonDocument doc;
  if (body.length() == 0 || deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"invalid_body\"}");
    return;
  }
  const String name = doc["field_name"] | "";
  if (name.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"invalid_field_name\"}");
    return;
  }
  saveFieldName(name);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiSensitivity() {
  const String body = server.arg("plain");
  JsonDocument doc;
  if (body.length() == 0 || deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"invalid_body\"}");
    return;
  }
  const uint32_t holdMs = doc["main_signal_hold_ms"] | 0;
  if (holdMs == 0) {
    server.send(400, "application/json", "{\"error\":\"invalid_sensitivity\"}");
    return;
  }
  saveSensitivity(holdMs);

  JsonDocument response;
  response["ok"] = true;
  response["main_signal_hold_ms"] = mainSignalHoldMs;
  response["quality_signal_hold_ms"] = mainSignalHoldMs;
  response["red_signal_hold_ms"] = RED_SIGNAL_HOLD_MS;
  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
}

void handleApiCameraSettings() {
  const String body = server.arg("plain");
  JsonDocument doc;
  if (body.length() == 0 || deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"invalid_body\"}");
    return;
  }

  const int indexArg = doc["index"] | 0;
  if (indexArg < 0 || indexArg >= CAMERA_COUNT) {
    server.send(400, "application/json", "{\"error\":\"invalid_camera_index\"}");
    return;
  }
  const uint8_t cameraIndex = static_cast<uint8_t>(indexArg);
  const String host = doc["host"] | "";
  const String username = doc["username"] | "";
  const String password = doc["password"] | "";
  if (host.length() > 0 && (username.length() == 0 || password.length() == 0)) {
    server.send(400, "application/json", "{\"error\":\"invalid_camera_settings\"}");
    return;
  }

  saveCameraSettings(cameraIndex, host, username, password);

  JsonDocument response;
  response["ok"] = true;
  response["camera_index"] = cameraIndex;
  response["camera_host"] = cameraHosts[cameraIndex];
  response["camera_username"] = cameraUsernames[cameraIndex];
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

  const bool active = doc["active"] | false;
  if (active) {
    startRecording("manual");
  } else {
    stopRecording("manual");
  }

  JsonDocument response;
  response["ok"] = true;
  response["recording_active"] = recordingActive;
  response["gnss_fix"] = gnss.fix;

  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
}

void handleApiOutput() {
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

  const int channel = doc["channel"] | 1;
  const bool on = doc["on"] | false;
  if (channel < 1 || channel > CHANNEL_COUNT) {
    server.send(400, "application/json", "{\"error\":\"invalid_channel\"}");
    return;
  }

  const bool ok = setDigitalOutput(static_cast<uint8_t>(channel - 1), on);
  JsonDocument resp;
  resp["ok"] = ok;
  resp["channel"] = channel;
  resp["output"] = channels[channel - 1].output;
  resp["switchable"] = doExpanderReady;
  if (channel == LIGHT_OUTPUT_CHANNEL) {
    resp["light_on"] = channels[LIGHT_OUTPUT_CHANNEL - 1].output;
  }
  String out;
  serializeJson(resp, out);
  server.send(ok ? 200 : 500, "application/json", out);
}

void handleApiCombinedGeoJson() {
  String json;
  json.reserve(256 + gpsLogCount * 36 + sensorEventCount * 190);
  json += "{\"type\":\"FeatureCollection\",\"features\":[";
  if (gpsLogCount > 0) {
    json += "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";
    for (uint16_t i = 0; i < gpsLogCount; i++) {
      if (i > 0) json += ",";
      const GpsLogEntry &entry = gpsLogAt(i);
      json += "[" + String(entry.longitude, 7) + "," + String(entry.latitude, 7) + "]";
    }
    json += "]},\"properties\":{\"trip_id\":\"" + String(tripId) + "\",\"type\":\"route\"}}";
  }
  bool needsComma = gpsLogCount > 0;
  for (uint16_t i = 0; i < sensorEventCount; i++) {
    const SensorTriggerEvent &event = sensorEventAt(i);
    if (!event.startHasGps) continue;
    if (needsComma) json += ",";
    needsComma = true;
    json += "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[" +
            String(event.startLongitude, 7) + "," + String(event.startLatitude, 7) +
            "]},\"properties\":{\"type\":\"sensor_event\",\"channel\":" + String(event.channel) +
            ",\"channel_name\":\"" + event.channelName + "\",\"duration_ms\":" +
            String(event.durationMs) + ",\"crop_name\":\"" + event.crop + "\"}}";
  }
  json += "]}";
  server.sendHeader("Content-Disposition", "attachment; filename=fahrt-mit-sensoren.geojson");
  server.send(200, "application/geo+json; charset=utf-8", json);
}

void handleApiArchive() {
  JsonDocument doc;
  JsonArray files = doc["files"].to<JsonArray>();
  if (filesystemReady) {
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
      String name = file.name();
      if (!name.startsWith("/")) {
        name = "/" + name;
      }
      if (name.startsWith("/trip-")) {
        JsonObject item = files.add<JsonObject>();
        item["name"] = name.substring(1);
        item["path"] = name;
        item["size"] = file.size();
      }
      file = root.openNextFile();
    }
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleApiArchiveDownload() {
  flushPersistentGpsBuffer();
  const String path = server.arg("path");
  if (!filesystemReady || !path.startsWith("/trip-") || path.indexOf("..") >= 0 || !LittleFS.exists(path)) {
    server.send(404, "application/json", "{\"error\":\"file_not_found\"}");
    return;
  }
  File file = LittleFS.open(path, FILE_READ);
  server.sendHeader("Content-Disposition", "attachment; filename=" + path.substring(1));
  server.streamFile(file, path.endsWith(".txt") ? "text/plain; charset=utf-8" : "text/csv; charset=utf-8");
  file.close();
}

void handleApiSystemEvents() {
  if (!filesystemReady || !LittleFS.exists("/system-events.log")) {
    server.send(404, "application/json", "{\"error\":\"file_not_found\"}");
    return;
  }
  File file = LittleFS.open("/system-events.log", FILE_READ);
  server.sendHeader("Content-Disposition", "attachment; filename=system-events.log");
  server.streamFile(file, "text/plain; charset=utf-8");
  file.close();
}

void handleApiTrack() {
  String json;
  const uint16_t pointCount = min<uint16_t>(gpsLogCount, LIVE_TRACK_MAX_POINTS);
  const uint16_t eventCount = min<uint16_t>(mainEventCount, LIVE_TRACK_MAX_EVENTS);
  json.reserve(256 + pointCount * 64 + eventCount * 72);
  json += "{\"current\":{\"fix\":";
  json += gnss.fix ? "true" : "false";
  json += ",\"latitude\":";
  json += String(gnss.latitude, 7);
  json += ",\"longitude\":";
  json += String(gnss.longitude, 7);
  json += ",\"main_mask\":";
  json += static_cast<unsigned int>(mainSignalMask());
  json += "},\"points\":[";

  const uint16_t firstPoint = gpsLogCount - pointCount;
  for (uint16_t i = 0; i < pointCount; i++) {
    if (i > 0) {
      json += ",";
    }
    const GpsLogEntry &entry = gpsLogAt(firstPoint + i);
    json += "{\"latitude\":";
    json += String(entry.latitude, 7);
    json += ",\"longitude\":";
    json += String(entry.longitude, 7);
    json += ",\"main_mask\":";
    json += static_cast<unsigned int>(entry.mainMask);
    json += "}";
  }

  json += "],\"events\":[";
  bool firstEvent = true;
  const uint16_t firstEventIndex = mainEventCount - eventCount;
  for (uint16_t i = 0; i < eventCount; i++) {
    const MainSignalEvent &event = mainEventAt(firstEventIndex + i);
    if (!event.detected || !event.hasGps) {
      continue;
    }
    if (!firstEvent) {
      json += ",";
    }
    firstEvent = false;
    json += "{\"latitude\":";
    json += String(event.latitude, 7);
    json += ",\"longitude\":";
    json += String(event.longitude, 7);
    json += ",\"channel\":";
    json += static_cast<unsigned int>(event.channel);
    json += "}";
  }

  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
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
  if (recordingActive) {
    server.send(409, "application/json", "{\"error\":\"recording_active\"}");
    return;
  }
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

void startEthernet() {
  pinMode(ETH_INT_PIN, INPUT_PULLUP);
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);
  delay(50);
  digitalWrite(ETH_RST_PIN, HIGH);
  delay(200);

  SPI.begin(ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Ethernet.init(ETH_CS_PIN);
  Ethernet.begin(ETH_MAC, ETHERNET_IP, ETHERNET_DNS, ETHERNET_GATEWAY, ETHERNET_SUBNET);
  delay(200);

  const EthernetHardwareStatus hardware = Ethernet.hardwareStatus();
  const EthernetLinkStatus link = Ethernet.linkStatus();
  ethernetReady = hardware != EthernetNoHardware;

  Serial.println("Ethernet W5500 initialisiert");
  Serial.print("Hardware: ");
  Serial.println(hardware == EthernetNoHardware ? "nicht gefunden" : "OK");
  Serial.print("Link: ");
  Serial.println(link == LinkON ? "ON" : link == LinkOFF ? "OFF" : "UNKNOWN");
  Serial.print("Ethernet IP: ");
  Serial.println(Ethernet.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Waveshare ESP32-S3-POE-ETH-8DI-8DO Drillmaschine");

  resetReason = resetReasonName(esp_reset_reason());
  loadChannelNames();
  loadCropName();
  loadFieldName();
  loadCameraSettings();
  loadSensitivity();
  bootCounter = preferences.getUInt("boot_counter", 0) + 1;
  tripCounter = preferences.getUInt("trip_counter", 0);
  preferences.putUInt("boot_counter", bootCounter);
  initFilesystem();
  appendSystemEvent("boot," + String(resetReason) + ",firmware=" + FIRMWARE_VERSION);
  initGpsLog();
  initGnssRs485();
  initDigitalInputs();
  startWiFiAccessPoint();
  startEthernet();
  initDigitalOutputs();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/camera/1/substream", HTTP_GET, []() { handleCameraProxy(0, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  server.on("/camera/1/mainstream", HTTP_GET, []() { handleCameraProxy(0, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  server.on("/camera/2/substream", HTTP_GET, []() { handleCameraProxy(1, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  server.on("/camera/2/mainstream", HTTP_GET, []() { handleCameraProxy(1, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  server.on("/camera/3/substream", HTTP_GET, []() { handleCameraProxy(2, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  server.on("/camera/3/mainstream", HTTP_GET, []() { handleCameraProxy(2, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  server.on("/camera/4/substream", HTTP_GET, []() { handleCameraProxy(3, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  server.on("/camera/4/mainstream", HTTP_GET, []() { handleCameraProxy(3, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/rs485-scan", HTTP_GET, handleApiRs485Scan);
  server.on("/api/rs485-address-scan", HTTP_GET, handleApiRs485AddressScan);
  server.on("/api/rs485-register-scan", HTTP_GET, handleApiRs485RegisterScan);
  server.on("/api/camera-test", HTTP_GET, handleApiCameraTest);
  server.on("/api/alarm/ack", HTTP_POST, handleApiAlarmAck);
  server.on("/api/channel-name", HTTP_POST, handleApiChannelName);
  server.on("/api/crop", HTTP_POST, handleApiCrop);
  server.on("/api/field", HTTP_POST, handleApiField);
  server.on("/api/sensitivity", HTTP_POST, handleApiSensitivity);
  server.on("/api/camera-settings", HTTP_POST, handleApiCameraSettings);
  server.on("/api/recording", HTTP_POST, handleApiRecording);
  server.on("/api/output", HTTP_POST, handleApiOutput);
  server.on("/api/track", HTTP_GET, handleApiTrack);
  server.on("/api/gps-log.csv", HTTP_GET, handleApiGpsLogCsv);
  server.on("/api/gps-log.geojson", HTTP_GET, handleApiGpsLogGeoJson);
  server.on("/api/main-events.csv", HTTP_GET, handleApiMainEventsCsv);
  server.on("/api/main-events.geojson", HTTP_GET, handleApiMainEventsGeoJson);
  server.on("/api/sensor-events.csv", HTTP_GET, handleApiSensorEventsCsv);
  server.on("/api/sensor-events.txt", HTTP_GET, handleApiSensorEventsTxt);
  server.on("/api/combined.geojson", HTTP_GET, handleApiCombinedGeoJson);
  server.on("/api/archive", HTTP_GET, handleApiArchive);
  server.on("/api/archive/download", HTTP_GET, handleApiArchiveDownload);
  server.on("/api/system-events.log", HTTP_GET, handleApiSystemEvents);
  server.on("/api/gps-log/clear", HTTP_POST, handleApiGpsLogClear);
  server.on("/api/crops", HTTP_GET, handleApiCrops);
  server.on("/api/crops", HTTP_POST, handleApiCropsPost);
  server.onNotFound([]() {
    server.send(404, "application/json", "{\"error\":\"not_found\"}");
  });
  server.begin();
  esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(nullptr);

  Serial.println("HTTP Server gestartet");
  Serial.println("Webseite: http://" + WiFi.softAPIP().toString() + "/");
  Serial.println("API:      http://" + WiFi.softAPIP().toString() + "/api/status");
}

void loop() {
  readDigitalInputs();
  pollGnss();
  updateGnssHealthAndRecording();
  server.handleClient();
  if (recordingActive && millis() - lastFileFlushMs >= FILE_FLUSH_INTERVAL_MS) {
    flushPersistentGpsBuffer();
  }
  esp_task_wdt_reset();

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
