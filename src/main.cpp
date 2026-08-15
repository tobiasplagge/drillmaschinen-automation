#include <Arduino.h>
#include <ArduinoJson.h>
#include <Ethernet.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <base64.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <mbedtls/sha256.h>

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

static constexpr uint8_t DEVICE_NAME_LENGTH = 64;
static const char *DEVICE_ID_DEFAULT = "Rabe Megadrill 3000-01";
char deviceName[DEVICE_NAME_LENGTH] = "Rabe Megadrill 3000-01";
static const char *FIRMWARE_VERSION = "2.6.5";
static const char *MODULE_ID = "M01";
static const char *DEFAULT_CROP_SUGGESTIONS_JSON = "[\"Weizen\",\"Gerste\",\"Roggen\",\"Hafer\",\"Dinkel\",\"Triticale\",\"Raps\",\"Mais\",\"Senf\",\"Pfeffer\",\"Hirse\",\"Buchweizen\",\"Erbsen\",\"Ackerbohnen\",\"Soja\",\"Sonnenblumen\",\"Lein\",\"Luzerne\",\"Gras\",\"Kleegras\",\"Zwischenfrucht\"]";
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
static constexpr uint8_t UPLOAD_URL_LENGTH = 128;
static constexpr uint8_t UPLOAD_TOKEN_LENGTH = 128;
static constexpr uint8_t UPLOAD_RETRY_MAX = 3;
static constexpr uint32_t UPLOAD_RETRY_DELAY_MS = 60000;
static constexpr uint32_t DEFAULT_MAIN_SIGNAL_HOLD_MS = 1500;
static constexpr uint32_t MIN_MAIN_SIGNAL_HOLD_MS = 300;
static constexpr uint32_t MAX_MAIN_SIGNAL_HOLD_MS = 10000;
static constexpr uint32_t ROTATION_PULSE_TIMEOUT_MS = 3000;
static constexpr uint8_t ROTATION_PULSES_PER_REV = 1;
static constexpr uint32_t GPS_LOG_INTERVAL_MS = 3000;
static constexpr uint32_t GNSS_POLL_INTERVAL_MS = 1000;
static constexpr uint32_t GNSS_STALE_MS = 10000;
static constexpr uint32_t GNSS_AUTO_START_HOLD_MS = 3000;
static constexpr uint32_t DEFAULT_LIFT_AUTO_STOP_DELAY_MS = 600000;
static constexpr uint32_t MIN_LIFT_AUTO_STOP_DELAY_MS = 0;
static constexpr uint32_t MAX_LIFT_AUTO_STOP_DELAY_MS = 3600000;
static constexpr uint32_t FILE_FLUSH_INTERVAL_MS = 30000;
static constexpr uint32_t SESSION_TTL_MS = 8UL * 60UL * 60UL * 1000UL;
static constexpr uint8_t MAX_USERS = 8;
static constexpr uint8_t MAX_SESSIONS = 4;
static constexpr bool AUTHENTICATION_REQUIRED = false;
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
static constexpr bool DEFAULT_DO_ACTIVE_HIGH = false;
static constexpr bool LIGHT_DO_ACTIVE_HIGH = false;
static constexpr bool INPUT_ACTIVE_HIGH = false;
// DO2 and DO3 are currently unused. No sensor state may switch them.
static constexpr bool MIRROR_RED_TO_OUTPUT = false;
static constexpr uint8_t LIGHT_OUTPUT_CHANNEL = 1;
static constexpr uint8_t FAN_OUTPUT_CHANNEL = 4;
static constexpr float FAN_ON_TEMPERATURE_C = 43.0f;
static constexpr float FAN_OFF_TEMPERATURE_C = 41.0f;
static constexpr uint32_t FAN_TEMPERATURE_CHECK_INTERVAL_MS = 2000;
static constexpr uint8_t PNEUMATIC_VALVE_COUNT = 4;
static constexpr uint8_t PNEUMATIC_VALVE_OUTPUTS[PNEUMATIC_VALVE_COUNT] = {8, 7, 6, 5};
static constexpr uint32_t PNEUMATIC_VALVE_PULSE_MS = 5000;
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
  uint32_t unixEpoch = 0;
  uint32_t unixEpochUptimeMs = 0;
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
uint8_t activePneumaticValveIndex = 255;
uint32_t pneumaticValveOffAtMs = 0;
uint32_t lastFanTemperatureCheckMs = 0;
uint32_t lastDebugMs = 0;
uint32_t bootCounter = 0;
uint32_t tripCounter = 0;
uint32_t mainSignalHoldMs = DEFAULT_MAIN_SIGNAL_HOLD_MS;
const char *resetReason = "unknown";
bool filesystemReady = false;
uint32_t lastFsCheckMs = 0;
uint32_t cachedFsUsedBytes = 0;
uint32_t cachedFsTotalBytes = 0;
bool autoStartEnabled = true;
bool autoStartArmed = true;
bool manualAutoStartLock = false;
uint32_t liftAutoStopDelayMs = DEFAULT_LIFT_AUTO_STOP_DELAY_MS;
uint32_t liftRaisedSinceMs = 0;
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
char uploadUrl[UPLOAD_URL_LENGTH] = "";
char uploadToken[UPLOAD_TOKEN_LENGTH] = "";
bool autoUpload = false;
bool pendingUpload = false;
char pendingUploadTripId[TRIP_ID_LENGTH] = "";
uint8_t uploadRetryCount = 0;
uint32_t uploadRetryNextMs = 0;
uint32_t tripStartRealTime = 0;
uint32_t tripEndRealTime = 0;
char tripStartFieldName[FIELD_NAME_LENGTH] = "";
char tripStartCropName[CROP_NAME_LENGTH] = "";
bool liftAutoStopPendingConfirm = false;
char liftAutoStopTripId[TRIP_ID_LENGTH] = "";

enum class UserRole : uint8_t {
  Viewer = 0,
  Operator = 1,
  Admin = 2,
};

struct UserAccount {
  char username[32] = "";
  char salt[33] = "";
  char passwordHash[65] = "";
  UserRole role = UserRole::Viewer;
  bool enabled = true;
};

struct SessionState {
  bool active = false;
  char token[65] = "";
  char deviceBinding[65] = "";
  char username[32] = "";
  UserRole role = UserRole::Viewer;
  uint32_t expiresAtMs = 0;
  IPAddress remoteIp;
  char userAgent[96] = "";
};

UserAccount users[MAX_USERS];
uint8_t userCount = 0;
SessionState sessions[MAX_SESSIONS];
SessionState activeSession;
bool activeSessionValid = false;

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

int8_t pneumaticValveIndexForOutput(uint8_t outputChannel) {
  for (uint8_t i = 0; i < PNEUMATIC_VALVE_COUNT; i++) {
    if (PNEUMATIC_VALVE_OUTPUTS[i] == outputChannel) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

uint8_t tcaBitForOutputChannel(uint8_t outputChannel) {
  // Waveshare terminal labels map directly to the TCA9554 bits:
  // DO1 is bit 0, DO2 is bit 1, ... DO8 is bit 7.
  return outputChannel - 1;
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

String roleToString(UserRole role) {
  switch (role) {
    case UserRole::Admin: return "admin";
    case UserRole::Operator: return "operator";
    case UserRole::Viewer:
    default: return "viewer";
  }
}

UserRole roleFromString(const String &value) {
  String normalized = value;
  normalized.toLowerCase();
  if (normalized == "admin") return UserRole::Admin;
  if (normalized == "operator") return UserRole::Operator;
  return UserRole::Viewer;
}

uint8_t roleRank(UserRole role) {
  switch (role) {
    case UserRole::Admin: return 3;
    case UserRole::Operator: return 2;
    case UserRole::Viewer:
    default: return 1;
  }
}

String randomHex(uint8_t byteCount) {
  static const char digits[] = "0123456789abcdef";
  String out;
  out.reserve(byteCount * 2);
  for (uint8_t i = 0; i < byteCount; i++) {
    const uint8_t value = static_cast<uint8_t>(esp_random() & 0xff);
    out += digits[(value >> 4) & 0x0f];
    out += digits[value & 0x0f];
  }
  return out;
}

String sha256Hex(const String &input) {
  uint8_t hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts_ret(&ctx, 0);
  mbedtls_sha256_update_ret(&ctx, reinterpret_cast<const unsigned char *>(input.c_str()), input.length());
  mbedtls_sha256_finish_ret(&ctx, hash);
  mbedtls_sha256_free(&ctx);

  static const char digits[] = "0123456789abcdef";
  String out;
  out.reserve(64);
  for (uint8_t b : hash) {
    out += digits[(b >> 4) & 0x0f];
    out += digits[b & 0x0f];
  }
  return out;
}

String getRequestHeader(const char *name) {
  return server.header(name);
}

String getCookieValue(const String &cookieHeader, const char *name) {
  const String needle = String(name) + "=";
  int index = cookieHeader.indexOf(needle);
  if (index < 0) {
    return "";
  }
  index += needle.length();
  int end = cookieHeader.indexOf(';', index);
  if (end < 0) {
    end = cookieHeader.length();
  }
  String value = cookieHeader.substring(index, end);
  value.trim();
  return value;
}

String currentDeviceBinding() {
  String binding = getRequestHeader("X-Device-Binding");
  binding.trim();
  return binding;
}

void resetActiveSession() {
  activeSessionValid = false;
  memset(&activeSession, 0, sizeof(activeSession));
}

void clearSessions() {
  for (SessionState &session : sessions) {
    memset(&session, 0, sizeof(session));
  }
  resetActiveSession();
}

int findUserIndex(const String &username) {
  for (uint8_t i = 0; i < userCount; i++) {
    if (username.equalsIgnoreCase(users[i].username)) {
      return i;
    }
  }
  return -1;
}

UserAccount *findUser(const String &username) {
  const int index = findUserIndex(username);
  return index >= 0 ? &users[index] : nullptr;
}

bool userPasswordMatches(const UserAccount &user, const String &password) {
  if (!user.enabled || user.username[0] == '\0') {
    return false;
  }
  if (user.salt[0] == '\0' || user.passwordHash[0] == '\0') {
    return false;
  }
  return sha256Hex(String(user.salt) + ":" + password) == user.passwordHash;
}

void setUserPassword(UserAccount &user, const String &password) {
  String salt = randomHex(16);
  salt.toCharArray(user.salt, sizeof(user.salt));
  String hash = sha256Hex(String(user.salt) + ":" + password);
  hash.toCharArray(user.passwordHash, sizeof(user.passwordHash));
}

void saveUsersToPreferences() {
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (uint8_t i = 0; i < userCount; i++) {
    if (users[i].username[0] == '\0') {
      continue;
    }
    JsonObject item = arr.add<JsonObject>();
    item["username"] = users[i].username;
    item["salt"] = users[i].salt;
    item["password_hash"] = users[i].passwordHash;
    item["role"] = roleToString(users[i].role);
    item["enabled"] = users[i].enabled;
  }
  String json;
  serializeJson(doc, json);
  Preferences authPrefs;
  authPrefs.begin("auth", false);
  authPrefs.putString("users", json);
  authPrefs.end();
}

void loadUsersFromPreferences() {
  Preferences authPrefs;
  authPrefs.begin("auth", false);
  String stored = authPrefs.getString("users", "");
  authPrefs.end();

  userCount = 0;
  memset(users, 0, sizeof(users));

  if (stored.length() > 0) {
    JsonDocument doc;
    if (!deserializeJson(doc, stored) && doc.is<JsonArray>()) {
      for (JsonVariant variant : doc.as<JsonArray>()) {
        if (userCount >= MAX_USERS || !variant.is<JsonObject>()) {
          continue;
        }
        JsonObject obj = variant.as<JsonObject>();
        String username = obj["username"] | "";
        if (username.length() == 0) {
          continue;
        }
        username.toCharArray(users[userCount].username, sizeof(users[userCount].username));
        String salt = obj["salt"] | "";
        salt.toCharArray(users[userCount].salt, sizeof(users[userCount].salt));
        String hash = obj["password_hash"] | "";
        hash.toCharArray(users[userCount].passwordHash, sizeof(users[userCount].passwordHash));
        users[userCount].role = roleFromString(obj["role"] | "viewer");
        users[userCount].enabled = obj["enabled"] | true;
        userCount++;
      }
    }
  }
  clearSessions();
}

bool createOrUpdateUser(const String &username, const String &password, UserRole role, bool enabled) {
  if (username.length() == 0) {
    return false;
  }

  UserAccount *user = findUser(username);
  if (!user) {
    if (userCount >= MAX_USERS) {
      return false;
    }
    user = &users[userCount++];
    memset(user, 0, sizeof(UserAccount));
    username.toCharArray(user->username, sizeof(user->username));
  }

  if (password.length() > 0) {
    setUserPassword(*user, password);
  } else if (user->passwordHash[0] == '\0') {
    return false;
  }
  user->role = role;
  user->enabled = enabled;
  saveUsersToPreferences();
  return true;
}

bool deleteUserAccount(const String &username) {
  const int index = findUserIndex(username);
  if (index < 0) {
    return false;
  }
  for (uint8_t i = index; i + 1 < userCount; i++) {
    users[i] = users[i + 1];
  }
  if (userCount > 0) {
    userCount--;
    memset(&users[userCount], 0, sizeof(UserAccount));
  }
  saveUsersToPreferences();
  return true;
}

SessionState *findSessionByToken(const String &token) {
  if (token.length() == 0) {
    return nullptr;
  }
  for (SessionState &session : sessions) {
    if (session.active && token.equals(session.token)) {
      return &session;
    }
  }
  return nullptr;
}

SessionState *findSessionByRequest() {
  String cookie = getRequestHeader("Cookie");
  String token = getCookieValue(cookie, "pm_session");
  if (token.length() == 0) {
    token = getRequestHeader("X-Session-Token");
  }
  return findSessionByToken(token);
}

String sessionBindingFromRequest() {
  String binding = currentDeviceBinding();
  if (binding.length() == 0) {
    binding = getCookieValue(getRequestHeader("Cookie"), "pm_binding");
  }
  return binding;
}

SessionState *allocateSessionSlot() {
  for (SessionState &session : sessions) {
    if (!session.active) {
      return &session;
    }
  }
  return &sessions[0];
}

SessionState *createSession(const UserAccount &user) {
  SessionState *session = allocateSessionSlot();
  if (!session) {
    return nullptr;
  }
  memset(session, 0, sizeof(SessionState));
  String token = randomHex(32);
  token.toCharArray(session->token, sizeof(session->token));
  String binding = sessionBindingFromRequest();
  if (binding.length() == 0) {
    binding = randomHex(16);
  }
  binding.toCharArray(session->deviceBinding, sizeof(session->deviceBinding));
  strncpy(session->username, user.username, sizeof(session->username) - 1);
  session->role = user.role;
  session->expiresAtMs = millis() + SESSION_TTL_MS;
  session->remoteIp = server.client().remoteIP();
  String ua = getRequestHeader("User-Agent");
  ua.toCharArray(session->userAgent, sizeof(session->userAgent));
  session->active = true;
  return session;
}

void expireSession(SessionState &session) {
  memset(&session, 0, sizeof(session));
}

bool authorize(UserRole minimumRole) {
  resetActiveSession();
  if (!AUTHENTICATION_REQUIRED) {
    activeSession.active = true;
    strncpy(activeSession.username, "Lokal", sizeof(activeSession.username) - 1);
    activeSession.role = UserRole::Admin;
    activeSession.expiresAtMs = millis() + SESSION_TTL_MS;
    activeSessionValid = true;
    return true;
  }
  SessionState *session = findSessionByRequest();
  if (!session) {
    server.send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return false;
  }
  if (millis() > session->expiresAtMs) {
    expireSession(*session);
    server.send(401, "application/json", "{\"error\":\"session_expired\"}");
    return false;
  }

  const String binding = sessionBindingFromRequest();
  if (binding.length() == 0 || binding != session->deviceBinding) {
    server.send(401, "application/json", "{\"error\":\"session_binding_mismatch\"}");
    return false;
  }

  const String ua = getRequestHeader("User-Agent");
  if (ua.length() > 0 && session->userAgent[0] != '\0' && ua != session->userAgent) {
    server.send(401, "application/json", "{\"error\":\"session_agent_mismatch\"}");
    return false;
  }

  if (roleRank(session->role) < roleRank(minimumRole)) {
    server.send(403, "application/json", "{\"error\":\"forbidden\"}");
    return false;
  }

  activeSession = *session;
  activeSessionValid = true;
  return true;
}

void sendSessionCookie(const SessionState &session) {
  server.sendHeader("Set-Cookie", "pm_session=" + String(session.token) + "; Path=/; Max-Age=28800; SameSite=Strict");
  server.sendHeader("Set-Cookie", "pm_binding=" + String(session.deviceBinding) + "; Path=/; Max-Age=28800; SameSite=Strict");
}

template <typename Handler>
void registerProtectedRoute(const char *path, HTTPMethod method, UserRole minimumRole, Handler handler) {
  server.on(path, method, [minimumRole, handler]() {
    if (!authorize(minimumRole)) {
      return;
    }
    handler();
  });
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

void loadLiftAutoStopDelay() {
  liftAutoStopDelayMs = preferences.getUInt("lift_stop_ms", DEFAULT_LIFT_AUTO_STOP_DELAY_MS);
  liftAutoStopDelayMs = constrain(liftAutoStopDelayMs, MIN_LIFT_AUTO_STOP_DELAY_MS, MAX_LIFT_AUTO_STOP_DELAY_MS);
}

void saveLiftAutoStopDelay(uint32_t delayMs) {
  liftAutoStopDelayMs = constrain(delayMs, MIN_LIFT_AUTO_STOP_DELAY_MS, MAX_LIFT_AUTO_STOP_DELAY_MS);
  preferences.putUInt("lift_stop_ms", liftAutoStopDelayMs);
}

void loadUploadConfig() {
  String stored = preferences.getString("upload_url", "");
  stored.toCharArray(uploadUrl, UPLOAD_URL_LENGTH);
  stored = preferences.getString("upload_tok", "");
  stored.toCharArray(uploadToken, UPLOAD_TOKEN_LENGTH);
  autoUpload = preferences.getBool("auto_upload", false);
}

void saveUploadConfig() {
  preferences.putString("upload_url", uploadUrl);
  preferences.putString("upload_tok", uploadToken);
  preferences.putBool("auto_upload", autoUpload);
}

void loadDeviceConfig() {
  String stored = preferences.getString("device_name", "");
  if (stored.length() > 0) {
    stored.toCharArray(deviceName, DEVICE_NAME_LENGTH);
  }
}

void saveDeviceConfig() {
  preferences.putString("device_name", deviceName);
}

// DI7 (GPIO10, HIDDEN_CHANNEL) liest ISO 11786 PIN5: 0V=unten, 10V=oben.
// Opto-Koppler invertiert: GPIO LOW = Eingang aktiv = Gerät oben.
bool liftIsDown() { return !channels[HIDDEN_CHANNEL - 1].active; }
bool liftIsUp()   { return  channels[HIDDEN_CHANNEL - 1].active; }

uint32_t liftAutoStopRemainingMs(uint32_t now) {
  if (!recordingActive || !liftIsUp() || liftRaisedSinceMs == 0) return 0;
  const uint32_t elapsed = now - liftRaisedSinceMs;
  return elapsed >= liftAutoStopDelayMs ? 0 : liftAutoStopDelayMs - elapsed;
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

uint32_t nmeaToEpoch(const String &hhmmss, const String &ddmmyy) {
  if (hhmmss.length() < 6 || ddmmyy.length() < 6) return 0;
  int h  = hhmmss.substring(0, 2).toInt();
  int m  = hhmmss.substring(2, 4).toInt();
  int s  = hhmmss.substring(4, 6).toInt();
  int d  = ddmmyy.substring(0, 2).toInt();
  int mo = ddmmyy.substring(2, 4).toInt();
  int y  = ddmmyy.substring(4, 6).toInt() + 2000;
  if (d < 1 || d > 31 || mo < 1 || mo > 12 || y < 2020) return 0;
  // Civil day → Unix epoch (Howard Hinnant algorithm, UTC)
  if (mo <= 2) { y--; mo += 12; }
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (mo - 3) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int32_t days = (int32_t)(era * 146097 + (int32_t)doe - 719468);
  return (uint32_t)((int64_t)days * 86400 + h * 3600 + m * 60 + s);
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
    {
      uint32_t epoch = nmeaToEpoch(nmeaField(sentence, 1), nmeaField(sentence, 9));
      if (epoch > 0) {
        gnss.unixEpoch = epoch;
        gnss.unixEpochUptimeMs = millis();
      }
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
    if (!manualAutoStartLock) autoStartArmed = true;
  }
  // Nach einem manuellen Stopp darf ein dauerhaftes Hubwerk-unten-Signal die
  // Aufzeichnung nicht erneut starten. Erst Hubwerk oben gibt den Auto-Start
  // für das nächste Absenken wieder frei.
  if (manualAutoStartLock) {
    autoStartArmed = false;
    if (liftIsUp()) {
      manualAutoStartLock = false;
      autoStartArmed = true;
    }
  }
  if (recordingActive) {
    if (liftIsUp()) {
      if (liftRaisedSinceMs == 0) liftRaisedSinceMs = now;
      if (now - liftRaisedSinceMs >= liftAutoStopDelayMs) {
        stopRecording("lift_auto");
        autoStartArmed = true;
        liftRaisedSinceMs = 0;
      }
    } else {
      liftRaisedSinceMs = 0;
    }
    return;
  }
  liftRaisedSinceMs = 0;
  if (autoStartEnabled && autoStartArmed && liftIsDown() && gnss.fix &&
      gnss.fixAvailableSinceMs > 0 && now - gnss.fixAvailableSinceMs >= GNSS_AUTO_START_HOLD_MS) {
    startRecording("lift_gnss_auto");
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
                  String(event.durationMs) + "," + String(event.durationMs / 1000.0f, 2) + "," + String(event.channel) + "," +
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
  appendFile(tripSensorPath, "start_uptime_ms,end_uptime_ms,duration_ms,duration_s,channel,channel_name,crop_name,field_name,start_latitude,start_longitude,end_latitude,end_longitude\n");
  // Write initial metadata as JSON (will be overwritten with final values on stopRecording)
  JsonDocument metaDoc;
  metaDoc["trip_id"] = tripId;
  metaDoc["device_id"] = deviceName;
  metaDoc["module_id"] = MODULE_ID;
  metaDoc["firmware_version"] = FIRMWARE_VERSION;
  metaDoc["field_name"] = fieldName;
  metaDoc["crop_name"] = cropName;
  metaDoc["start_uptime_ms"] = millis();
  metaDoc["start_real_time"] = tripStartRealTime;
  metaDoc["end_real_time"] = 0;
  metaDoc["status"] = "active";
  metaDoc["gps_points"] = 0;
  metaDoc["sensor_events"] = 0;
  metaDoc["main_events"] = 0;
  String metaJson;
  serializeJson(metaDoc, metaJson);
  appendFile(tripMetaPath, metaJson);
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
  strncpy(tripStartFieldName, fieldName, FIELD_NAME_LENGTH - 1);
  tripStartFieldName[FIELD_NAME_LENGTH - 1] = '\0';
  strncpy(tripStartCropName, cropName, CROP_NAME_LENGTH - 1);
  tripStartCropName[CROP_NAME_LENGTH - 1] = '\0';
  tripStartRealTime = gnss.unixEpoch > 0
      ? gnss.unixEpoch + (millis() - gnss.unixEpochUptimeMs) / 1000
      : 0;
  tripEndRealTime = 0;
  createTripFiles();
  persistentGpsBuffer = "";
  lastFileFlushMs = millis();
  recordingActive = true;
  if (strcmp(source, "manual") == 0) manualAutoStartLock = false;
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
  tripEndRealTime = gnss.unixEpoch > 0
      ? gnss.unixEpoch + (millis() - gnss.unixEpochUptimeMs) / 1000
      : 0;
  if (tripMetaPath[0] != '\0' && filesystemReady) {
    LittleFS.remove(tripMetaPath);
    JsonDocument metaDoc;
    metaDoc["trip_id"] = tripId;
    metaDoc["device_id"] = deviceName;
    metaDoc["module_id"] = MODULE_ID;
    metaDoc["firmware_version"] = FIRMWARE_VERSION;
    metaDoc["field_name"] = tripStartFieldName;
    metaDoc["crop_name"] = tripStartCropName;
    metaDoc["start_real_time"] = tripStartRealTime;
    metaDoc["end_real_time"] = tripEndRealTime;
    metaDoc["status"] = "completed";
    metaDoc["gps_points"] = (uint32_t)gpsLogTotal;
    metaDoc["sensor_events"] = (uint32_t)sensorEventTotal;
    metaDoc["main_events"] = (uint32_t)mainEventTotal;
    String metaJson;
    serializeJson(metaDoc, metaJson);
    appendFile(tripMetaPath, metaJson);
  }
  appendSystemEvent("recording_stop," + String(source) + "," + tripId);
  recordingActive = false;
  if (strcmp(source, "lift_auto") == 0) {
    liftAutoStopPendingConfirm = true;
    strncpy(liftAutoStopTripId, tripId, TRIP_ID_LENGTH - 1);
    liftAutoStopTripId[TRIP_ID_LENGTH - 1] = '\0';
  }
  if (strcmp(source, "manual") == 0) {
    manualAutoStartLock = true;
    autoStartArmed = false;
  }
  // Der Browser auf Tablet/Telefon übernimmt den Portal-Upload. Der ESP32
  // stellt dafür nur die lokal gespeicherten Fahrtdateien bereit.
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

  const uint8_t outputChannel = channelIndex + 1;
  const bool activeHigh = (outputChannel == LIGHT_OUTPUT_CHANNEL) ? LIGHT_DO_ACTIVE_HIGH : DEFAULT_DO_ACTIVE_HIGH;
  const bool pinLevel = activeHigh ? on : !on;
  const uint8_t mask = 1 << tcaBitForOutputChannel(outputChannel);
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

bool startPneumaticValve(uint8_t valveIndex) {
  if (valveIndex >= PNEUMATIC_VALVE_COUNT || activePneumaticValveIndex != 255 || !doExpanderReady) {
    return false;
  }

  const uint8_t outputChannel = PNEUMATIC_VALVE_OUTPUTS[valveIndex];
  if (!setDigitalOutput(outputChannel - 1, true)) {
    return false;
  }

  activePneumaticValveIndex = valveIndex;
  pneumaticValveOffAtMs = millis() + PNEUMATIC_VALVE_PULSE_MS;
  return true;
}

void updatePneumaticValves() {
  if (activePneumaticValveIndex >= PNEUMATIC_VALVE_COUNT) {
    return;
  }

  const uint32_t now = millis();
  if (static_cast<int32_t>(now - pneumaticValveOffAtMs) < 0) {
    return;
  }

  const uint8_t outputChannel = PNEUMATIC_VALVE_OUTPUTS[activePneumaticValveIndex];
  setDigitalOutput(outputChannel - 1, false);
  activePneumaticValveIndex = 255;
  pneumaticValveOffAtMs = 0;
}

void updateFanTemperatureControl() {
  const uint32_t now = millis();
  if (now - lastFanTemperatureCheckMs < FAN_TEMPERATURE_CHECK_INTERVAL_MS) {
    return;
  }
  lastFanTemperatureCheckMs = now;
  const float temperatureC = temperatureRead();
  if (!channels[FAN_OUTPUT_CHANNEL - 1].output && temperatureC > FAN_ON_TEMPERATURE_C) {
    setDigitalOutput(FAN_OUTPUT_CHANNEL - 1, true);
  } else if (channels[FAN_OUTPUT_CHANNEL - 1].output && temperatureC <= FAN_OFF_TEMPERATURE_C) {
    setDigitalOutput(FAN_OUTPUT_CHANNEL - 1, false);
  }
}

void initDigitalOutputs() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(20);

  tcaOutputState = 0x00;
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    const uint8_t outputChannel = i + 1;
    const bool activeHigh = (outputChannel == LIGHT_OUTPUT_CHANNEL) ? LIGHT_DO_ACTIVE_HIGH : DEFAULT_DO_ACTIVE_HIGH;
    const bool offPinLevel = activeHigh ? false : true;
    if (offPinLevel) {
      tcaOutputState |= (1 << tcaBitForOutputChannel(outputChannel));
    }
  }
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
      // Jeden Impuls direkt und unabhaengig vom vorherigen Impuls auswerten.
      // Beim Signalabfall darf kein alter Prozentwert stehen bleiben.
      channels[i].activeSinceMs = active ? now : 0;
      if (!active) {
        channels[i].signalQualityPct = 0;
      }
    }

    channels[i].inputRaw = raw;
    channels[i].active = active;

    const bool seedChannel = isSeedChannel(i);
    const bool mainSignal = seedChannel && active && channels[i].activeSinceMs > 0 && (now - channels[i].activeSinceMs >= mainSignalHoldMs);
    if (seedChannel && active) {
      const uint32_t activeMs = channels[i].activeSinceMs > 0 ? now - channels[i].activeSinceMs : 0;
      channels[i].signalQualityPct = static_cast<uint8_t>(constrain((activeMs * 100UL) / max<uint32_t>(mainSignalHoldMs, 1), 1UL, 100UL));
    } else {
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

    if (seedChannel && i != LIGHT_OUTPUT_CHANNEL - 1 && i != FAN_OUTPUT_CHANNEL - 1 && !isPneumaticValveOutput(i + 1)) {
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

  if (filesystemReady && now - lastFsCheckMs >= 10000) {
    cachedFsUsedBytes  = LittleFS.usedBytes();
    cachedFsTotalBytes = LittleFS.totalBytes();
    lastFsCheckMs = now;
  }

  const bool rotMoving      = rotationMoving(now);
  const float rotRpm        = rotationRpm(now);
  const char *gnssHealthStr = gnssHealth();

  doc["device_id"] = deviceName;
  doc["device_name"] = deviceName;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["module_id"] = MODULE_ID;
  doc["esp_temperature_c"] = temperatureRead();
  doc["crop_name"] = cropName;
  doc["field_name"] = fieldName;
  doc["trip_id"] = tripId;
  doc["auto_start_enabled"] = autoStartEnabled;
  doc["filesystem_ready"] = filesystemReady;
  doc["filesystem_used_bytes"] = filesystemReady ? cachedFsUsedBytes : 0;
  doc["filesystem_total_bytes"] = filesystemReady ? cachedFsTotalBytes : 0;
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
  if (activeSessionValid) {
    JsonObject session = doc["session"].to<JsonObject>();
    session["username"] = activeSession.username;
    session["role"] = roleToString(activeSession.role);
    session["expires_in_ms"] = activeSession.expiresAtMs > now ? static_cast<int32_t>(activeSession.expiresAtMs - now) : 0;
  }
  doc["ethernet_ready"] = ethernetReady;
  doc["ethernet_ip"] = ethernetReady ? Ethernet.localIP().toString() : "";
  doc["ethernet_link"] = Ethernet.linkStatus() == LinkON;
  doc["main_signal_hold_ms"] = mainSignalHoldMs;
  doc["quality_signal_hold_ms"] = mainSignalHoldMs;
  doc["red_signal_hold_ms"] = mainSignalHoldMs;
  doc["light_channel"] = LIGHT_OUTPUT_CHANNEL;
  doc["light_on"] = channels[LIGHT_OUTPUT_CHANNEL - 1].output;
  doc["light_switchable"] = doExpanderReady;
  doc["fan_channel"] = FAN_OUTPUT_CHANNEL;
  doc["fan_on"] = channels[FAN_OUTPUT_CHANNEL - 1].output;
  doc["fan_switchable"] = doExpanderReady;
  doc["fan_on_temperature_c"] = FAN_ON_TEMPERATURE_C;
  doc["fan_off_temperature_c"] = FAN_OFF_TEMPERATURE_C;
  doc["do_expander_ready"] = doExpanderReady;
  doc["do_output_register"] = tcaOutputState;
  uint8_t doReadback = 0;
  if (tcaRead(TCA9554_OUTPUT_REG, doReadback)) {
    doc["do_output_readback"] = doReadback;
  }
  JsonArray doMap = doc["do_tca_bit_map"].to<JsonArray>();
  for (uint8_t outputChannel = 1; outputChannel <= CHANNEL_COUNT; outputChannel++) {
    JsonObject item = doMap.add<JsonObject>();
    item["do"] = outputChannel;
    item["tca_bit"] = tcaBitForOutputChannel(outputChannel);
  }
  JsonObject liftJson = doc["lift"].to<JsonObject>();
  liftJson["source"] = "DI7_ISO11786_PIN5";
  liftJson["is_down"] = liftIsDown();
  liftJson["is_up"] = liftIsUp();
  liftJson["raw_high"] = channels[HIDDEN_CHANNEL - 1].inputRaw;
  liftJson["auto_stop_delay_ms"] = liftAutoStopDelayMs;
  liftJson["auto_stop_remaining_ms"] = liftAutoStopRemainingMs(now);
  liftJson["auto_stop_timer_active"] = recordingActive && liftIsUp() && liftRaisedSinceMs > 0;
  liftJson["auto_start_ready"] = autoStartEnabled && autoStartArmed && liftIsDown() && gnss.fix;
  liftJson["auto_start_manual_lock"] = manualAutoStartLock;
  doc["lift_confirm_pending"] = liftAutoStopPendingConfirm;
  if (liftAutoStopPendingConfirm) doc["lift_confirm_trip_id"] = liftAutoStopTripId;
  JsonArray valves = doc["pneumatic_valves"].to<JsonArray>();
  const bool valveBusy = activePneumaticValveIndex < PNEUMATIC_VALVE_COUNT;
  const uint32_t valveRemainingMs =
      valveBusy && static_cast<int32_t>(pneumaticValveOffAtMs - now) > 0 ? pneumaticValveOffAtMs - now : 0;
  for (uint8_t i = 0; i < PNEUMATIC_VALVE_COUNT; i++) {
    const uint8_t outputChannel = PNEUMATIC_VALVE_OUTPUTS[i];
    JsonObject valve = valves.add<JsonObject>();
    valve["index"] = i;
    valve["label"] = PNEUMATIC_VALVE_LABELS[i];
    valve["output_channel"] = outputChannel;
    valve["on"] = channels[outputChannel - 1].output;
    valve["active"] = valveBusy && activePneumaticValveIndex == i;
    valve["locked"] = valveBusy && activePneumaticValveIndex != i;
    valve["remaining_ms"] = valveBusy && activePneumaticValveIndex == i ? valveRemainingMs : 0;
    valve["pulse_ms"] = PNEUMATIC_VALVE_PULSE_MS;
    valve["switchable"] = doExpanderReady && !valveBusy;
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
  gnssJson["health"] = gnssHealthStr;
  gnssJson["warning"] = strcmp(gnssHealthStr, "ok") != 0;
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
  rotationJson["status"] = rotMoving ? "Dreht" : "Dreht nicht";
  rotationJson["moving"] = rotMoving;
  rotationJson["rpm"] = rotRpm;
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
    const bool isRot = isRotationChannel(i);
    ch["rotation_moving"] = isRot ? rotMoving : false;
    ch["rotation_rpm"] = isRot ? rotRpm : 0.0f;
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

const char* htmlPage() {
  static const char PAGE[] = R"HTML(
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
    .settings-box { min-width: 0; border: 1px solid #374151; border-radius: 8px; background: #111827; padding: 14px; margin: 0 0 14px; }
    .settings-box h3 { margin: 0 0 10px; font-size: .98rem; }
    .admin-section { min-width: 0; margin: 0 0 10px; border: 1px solid #374151; border-radius: 8px; background: #111827; padding: 0; }
    .admin-section > summary { padding: 13px 14px; color: #f9fafb; font-size: .98rem; font-weight: 800; list-style-position: inside; }
    .admin-section[open] > summary { border-bottom: 1px solid #374151; }
    .admin-section-content { min-width: 0; padding: 14px; }
    .admin-section-content > .field-row:last-child { margin-bottom: 0; }
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
    .setting-hint { margin: 10px 0 0; padding: 10px 12px; border-radius: 6px; background: #111827; border: 1px solid #374151; color: #d1d5db; font-size: .88rem; line-height: 1.5; }
    .setting-hint strong { color: #f9fafb; }
    .setting-hint ul { margin: 6px 0 0; padding-left: 18px; }
    .setting-hint li { margin-bottom: 4px; }
    .track-wrap { position: relative; overflow: hidden; min-height: 280px; border: 1px solid #374151; border-radius: 6px; background: #e5e7eb; }
    #trackCanvas { display: block; width: 100%; height: 360px; }
    #topoMap { width: 100%; height: 520px; border: 1px solid #374151; border-radius: 6px; background: #e5e7eb; }
    .tabs { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 14px; border-bottom: 1px solid #374151; padding-bottom: 8px; }
    .tab-button { background: #374151; }
    .tab-button.active { background: #2563eb; }
    .nav-action { min-height: 40px; display: inline-flex; align-items: center; justify-content: center; border: 0; border-radius: 6px; background: #4b5563; color: white; padding: 9px 12px; font: inherit; font-weight: 750; white-space: nowrap; box-sizing: border-box; }
    .nav-action.light-on { background: #16a34a; color: #f0fdf4; }
    .nav-action.light-off { background: #991b1b; color: #fef2f2; }
    .nav-action.light-unknown { background: #f59e0b; color: #111827; }
    .nav-action.fan-on { background: #16a34a; color: #f0fdf4; }
    .valve-panel { margin-bottom: 14px; }
    .valve-actions { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 8px; }
    .valve-button { background: #374151; display: grid; gap: 2px; }
    .valve-button.active { background: #16a34a; color: #f0fdf4; }
    .valve-button.active:disabled { opacity: 1; }
    .valve-countdown { font-size: .78rem; font-weight: 650; color: inherit; }
    .hidden { display: none !important; }
    .map-status { margin: 0 0 10px; color: #d1d5db; font-size: .9rem; }
    .track-legend { display: flex; flex-wrap: wrap; gap: 8px 16px; margin-top: 10px; color: #d1d5db; font-size: .88rem; }
    .legend-key { display: inline-flex; align-items: center; gap: 6px; }
    .legend-dot { width: 11px; height: 11px; border-radius: 50%; display: inline-block; background: #16a34a; }
    .legend-dot.route { background: #2563eb; }
    .legend-dot.alert { background: #dc2626; }
    .recording-row { display: flex; align-items: center; justify-content: space-between; gap: 12px; flex-wrap: wrap; }
    .recording-label { font-size: 1.05rem; font-weight: 700; }
    .recording-label.active { color: #16a34a; }
    .recording-trip { font-size: .85rem; color: #9ca3af; margin-top: 2px; }
    .recording-row button { font-size: .97rem; padding: 10px 20px; }
    .modal-overlay { position: fixed; inset: 0; background: rgba(0,0,0,.75); display: flex; align-items: center; justify-content: center; z-index: 1000; padding: 16px; }
    .modal-box { background: #1f2937; border: 1px solid #374151; border-radius: 10px; padding: 22px; max-width: 400px; width: 100%; }
    .modal-box h3 { margin: 0 0 8px; font-size: 1.1rem; }
    .modal-box p { color: #d1d5db; font-size: .92rem; margin: 0 0 16px; line-height: 1.5; }
    .auth-overlay { position: fixed; inset: 0; z-index: 1100; display: flex; align-items: center; justify-content: center; padding: 16px; background: rgba(17,24,39,.94); backdrop-filter: blur(4px); }
    .auth-card { width: min(520px, 100%); border: 1px solid #374151; border-radius: 12px; background: #111827; padding: 22px; box-shadow: 0 24px 70px rgba(0,0,0,.35); }
    .auth-card h2 { margin: 0 0 8px; font-size: 1.35rem; }
    .auth-card p { margin: 0 0 16px; color: #9ca3af; line-height: 1.5; }
    .auth-grid { display: grid; gap: 10px; }
    .auth-grid .field-row { margin: 0; }
    .auth-badge { display: inline-flex; align-items: center; gap: 8px; padding: 4px 10px; border: 1px solid #374151; border-radius: 999px; background: #111827; color: #d1d5db; font-size: .82rem; font-weight: 700; }
    .user-admin-grid { display: grid; grid-template-columns: 1.2fr 1fr .8fr auto; gap: 8px; align-items: center; }
    .user-admin-list { display: grid; gap: 8px; margin-top: 10px; }
    .user-admin-row { display: grid; grid-template-columns: minmax(0, 1fr) 96px 90px auto; gap: 8px; align-items: center; padding: 8px; border: 1px solid #374151; border-radius: 6px; background: #111827; }
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
  <div id="authOverlay" class="auth-overlay hidden">
    <div class="auth-card">
      <h2 id="authTitle">Anmeldung</h2>
      <p id="authHint">Bitte mit einem berechtigten Benutzer anmelden.</p>
      <div class="auth-grid">
        <div class="field-row">
          <input id="authUsername" autocomplete="username" placeholder="Benutzername">
          <input id="authPassword" type="password" autocomplete="current-password" placeholder="Passwort">
        </div>
        <div class="field-row">
          <input id="authBootstrapUsername" autocomplete="username" placeholder="Erster Admin-Benutzer">
          <input id="authBootstrapPassword" type="password" autocomplete="new-password" placeholder="Erstpasswort">
        </div>
        <div class="actions">
          <button id="authLoginBtn" type="button">Anmelden</button>
          <button id="authBootstrapBtn" class="secondary" type="button">Erst-Admin anlegen</button>
        </div>
      </div>
      <div id="authMessage" class="error" style="margin-top:12px;"></div>
    </div>
  </div>
  <main>
    <div style="margin-bottom:8px;"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAANwAAACSCAIAAAChajPKAAAAAXNSR0IArs4c6QAAAERlWElmTU0AKgAAAAgAAYdpAAQAAAABAAAAGgAAAAAAA6ABAAMAAAABAAEAAKACAAQAAAABAAAA3KADAAQAAAABAAAAkgAAAACNoVRNAABAAElEQVR4Ady9BWAkx5U3LmZpRsxMK62WmcneXeOaMWY7zjl84LtL7mJfcsld4OxcLg4YYjtZ22tmWi8zo5ilFbM0GsFoNPr/XlV3TXX3zGi0dr77vn97Pap69ahevS7uKl/HlMPH4ePj6zM15ePrS/8QUB5EfXyn1LivrzOsYgCfgPgFhGEigL8KBECWIvFUQFNEQKj0S0x8pnzxnw/FOblRnEY3hTNXSWHGWAmeGqGCG5dFmFCBMksPgIBoHp0wlqaDcYsp6kNt4CjZcepAMDIsJAGRxAHCxDF0ZFgrnSvCzcJk0g/p7CQkEhYjtUUSFZVaWACCiiMJJiLgJCHOQFNSBJwCUBdqgyHKhdlGoAFbQDi5SyacqS5J6MANBd8TqioBMMc/hwMuSQ+TpHE7rhNP9f53WioggBtlerpHmAmILMzotFQcR8bUputj3kunUuHFo/IQWeMBZIPerWnyQXYVmRUcVJbKX+/15wQzyIVOEou6U8MV7mXCVA1BrjGQRrTss6ocwPzUsPTGMKdhzJzsuAzYjrkyEXGIIBdRXgCIKiQqN4EJBLmQGE+RSAEnK/X1VMuM9NGxVSoNbX2jYaeNyNK1KYYY1W5OCyDZENUZ3MCBADyzsBuLaBlyIHLkrlbjCMZf73OhmktTDLqMCIMbBankSmkKhXWYRg6qhhoDggpwxRYUcaZyN2B8fJ1OieJWCthgOKDybDAEJyOdZnJU1UmGuQgTmqQZx+CyNPmUZCJVkyS4SjgC9hUGZKFcQ8Hc+F6JJAToRTaYVEbQcZOTRFiWLoDeBDhz2IabR/ARATDhOBwiw6kuYHT4xT85SYjmeedoAug54LKghO8hwJ2S+y4hkwwtkUtVgKnTQ4o63wQjmmd15VTBEOxEGAgu9GEKyzgyH2NY5iCHOaYRwuGCP3MyDVduUE22pXTdS+cSjQNdJjFO5BMSy8sP6vlIInkSfskCDK6rLwStCEAPHc5laybMDg/0nXK4zzA0894U5M28ZlZeL6Ef5MnZEHA5wHG4ZswubiV7w03mzMLT58QTW5Y18NFwgWVYyRlk/e8AZF24aqLoNGprtVPqOfbeeEDTEnmMqbbSIbGSVV5PWZBLs2OgA+WF/jpWrKevsOKvjxrRI3qMs3G9WxkSKVMd2niDK5H91YOyGb+UMB0jXiQ64JcSoHtzLovXV6uPUQWXjsjR+EuC5tuTB8iVM3zFE6pROIMgh3BnmZAggBlbQVIFIpy43EcZuubHFSlH4LyJu4bAc4ThutRHpXOqpEJm9tepjVYxnlmZuxOTuZcQA7hLa2jx5ZggnXFA1memxB40gIV5qlzEnD+yxhPhb3jkgY5XCoCvTrAuquMicijQGETrpzoaNWrUnqfwl8NVIRFvEmR4fQAUCogA54a+BaPw1JHnmC5/ddx0ODxVGAEW1yF4iMqoclgmkeFGF5cx/8+EZX10EsndVJDqn2qcSsxp/2mc0mhx8MU/2SGEJEiQ4U6BLCSjMa9xKmGUoqPVRY1ZkhEUQfo6yVkDazUBqf4NlvWRLCkLUcLQxMBNTeJvh0qksYw7GmZblcLwl6kldBMBAx5ZV6QqcrUSRapM6xIoI0wb9p6DsCrUwz/1dVK0lCbP1TTudNCAy3BmB6HpxHIBzBDO14Kz4tRObpRFsCOAUItgf4VHEeOKM3mVRlNVJ1fIOph9cnKS/qefKSyM0ePr5+cX4M+eAH8EdMx1HL6qqIcMehDhjkqGey4aGVMW5A4u4/CwS0ynU6pIituK0nKvlkuGilyjX+oU8kCMJDxa9yXHZUAdmKG6+REi8DJqHU8hEAhuGOjB4+Pj1pFR68jI2NiYxWq1DFmGhoeHRwAYtY1PQEF4ZEhQUHh4WFh4WFR4pNkcFYZgaCggoaGh8FFwFG46U+mqNi7oXICc2EqBqoCZ/TVydu8PM+PsDhuO5zvpcPiJOtIVovBOV4mXA7u8XAmn/Mr1EXkQIgQEASzDWoatA4ODfQODl1pa6xoaG5ubW9s7e3p7h61WuKfdNgEcvCsoPyyQYUnd198vKCAwLDQ4xhydlJSQkZ6Wm5WVm5WZlJhgMkWZoqKCg4KEa8qyXIaNbuESzRvg5bG6PCpv9HGHo68pgeeN08xE0ZngOtWkfp5xoONBN3diPJA4pRlCcH34Ym9fX1tHx4WyinMXSyura7u6e0bHxmCgoMAgOFZgYEBQQEBAYCBC/gEBUMAx6bBP2icm7PjPZpug/+ixowaNiTZnZ2UuKClZOH9OTlZmbEyM2RQFKU7Jil8rAPHuucuXk5CH0BbI3PTJ2riK7L1xNJiSqhq4VsjlxZBxaZ7S29wrsmR0OSyrIiwrA2cUduZZMsSMOHhGdvJX8VCH9Q0MdHZ1nzl34eDR4xcrKrq6elCIwSFBkeHhZpMpLjYmMT4+Lj42xmyOiY42RUagdQ4MCERm7XbUmzY08YODQ2DS0wOf7uzs7urrH7AMD4+OjqH3GRURUViYt3rpkpXLlqanpsTGRMNlVeGX8xdZABl/gd0VhGe+Hqh0SbqozNZDkoxmDMuEvDhc1JRGMgNE5qNJdJugwXIR0REKQytasg4GD7sg9gbknU/39vU3XWrZe+jw3oOHq2vqJuz2yLBwkzkiJTE5Lze7qDC/IDcnLTUV7hgaGjKtWIyB4Isdnd11DQ0VNbVV1bWNl5p7evstFqtjajIrI33TurWb1qzOyc6Mj4v9ki/wtMbRWXha5b8qhJnKhR2UmtJllmR2CONhJUtBY8PK0qf/MQoSEFEqAgJ2cnh67jPEEMwhGr3GhsbmnfsOfLZ7d319c2BgIIYpaSnJJUWzFi+YP3/ObNRqGFQ7JcAWbuzA2eKXVWFOCtSX5VXVJ06fOXOhFB1T1KPjNltKcuLVV2y6dssV2ZmZUVGRHvqacnE4mXoMySRy2AORBzRdki5KPL178z1IR5LTKT3j/ZVShU+QKprBljO/MlwOu1RpWgSXVOj8tbS1wR3ffv/D2roGuGNstLkwP3/V8iVrV67Iz80Wvsg9RihnFGeEcIlEgjTVSdGmHzp6fM/BQ6Xlld09PWPj47Py8269cdsNV2+Ji43V+aUQ51L5/18CnX1Kdwb9yrMNK+NhFY2Tt5DOS85dTSxqUyelFBJMOGza4gS37p7e0+fOb3/j7SPHT0FoQlxsUWHh5o1rN61dk5gQz/novEQSeDlBCOVk6HfuOXj40127L5aWd/X0Tjomn/jHv3/g7jt04ngups2Ld6p4tp+eh1GogIiAnsZ93HuSAOEesh8Y6Y0Q99KnSZHdUedGoJTVMDLSFZgeYSZaosPX1NL61nsf7nj3fYyyzZFRhQW5127ZfPWVm9DDI00Mc5tGbfUKeIwr5JRDsgGmh2689qp1q5Z/vHP3B5/urK6tG7JYAJ9wjMF3AnyDOTNuLtloHoVQonszKNbVIOAlMeRUFq0Vp5AKfTSstKjuY9MQXc5A5zLKxoMWbrkh3yBz8/AUYRqXWB6EjoyOYpbn2Re3Hzh2PMDPLyU5acumDXfcuA1jDmKl+g2CWibamEup7oEuc8orztr6hhNnzq1btSIxMaa043NMeBYmrA7xj9Ixc8lBh6OLetDYQxJnMi0C0LzB0ankOQqD0OQ5+Hr5fFUaXB4f74tEhylHkeX+gYE9Bw7/5o/P1zc2YbJwXknx12675coN69B3NNaOXloGaF5miiuDX5AoFRdo1Ta9rvfky6f/ZtJn7MrC7y5M2hYZlEBY0z0igypztzReKulOoBDkDuErgdOUL3s02moiWjncfKLwPGBq6TQxD2YGf8FcQ8OKUFeWOgQRFYXNISIK5uhEvvHeB8//+ZXBwcHkpIQtG9Y/fN/XMtPTgOlOLjHxIp8eMsXVUH7Zsp9QyZkEkO9Uw+Apu4911Nb/Qem/9w93rM99KDo4xQWyk4xCAkEEtOnOmEslPWQOSci8xNYlAyd/70OyUDkMDv5PPPEEiWUvLtPABVsO579CKSOyEeKC11cBEvXKtMxkTIQxJf6nV177459exmp1ZkbGQ1+7+7GH7sdkuCd3dCODW8xNogKWcYQNhQFd0qZEFgQHhnQM1ltG+hr7T004RpKjisKCTDpkI2cdwpeMzqAokZ8ZYJNe06IrfUqGJ78QLjIFHNmguqgLAgkEI/K3TQSkxL9ukOvJPfIPL/75lTfehjxM9Dz28APXbL4C4cvwSM8ae7CMooxqDSMf6Al9TjS/ubPmN70j9Vi2XJJx+7VFj8eEpEvVlZFuGghna0SSPcqd2u7gnJu+QGWORnneQfx/9KMn1P6MhgKq4B9/REAFePXXSKXPgBs2QFNSLjeHuhyhSHp6++CRL7/6Opxwzuyif/7b725cuxpS4AFOcao+HIJfPCrM+VcHEjiucJ1U04aYUPAmfdLMs+MiMtotVZbR3paBC1a7JSN6XkiAftzDeer1EdabVqRaxDoOXtBBhhsiN2DqYbDjH9yly0L9n3zyCTk+bVgw5QH88n8uCUWBuUydHiiE6VBdOSt5kOQXstWGLMN/2v7aC6+8Bn5zZxf/8O+/v3TRArCkOlJiJZPoBMpRd0pxHGOqESJzk8NcAf6bEJGTEJ57aahs2NbXOnDe7rBnxSwIDgiT8Y1hsgDapBk+njV0kSoZzYMoDSHpRdrplNPgqLw0WwG4OdQk1385U7DnAfzyfxxbJ2NGLaM30hWddGIYlHkYV4rFVZzxcdv7H3/27J9fmbTbsXj9L//wt4vmzwWGopuKRhC9xRgf6UfVUKKRUnlQ0kBJM0JYPeOCCVcAvzxQEL/qppInEyLysFHzcOPLRxq3j09aDQI1AGSKTltRH2dIhfC/PCPSr0Ck11pEVKLpASqm5q8+4waPBLYehzHQOCVsodqdEg26MAr2o5SoAQcyKFdSjcVpZLaCC+cvpPCSEKkqoQ7gKSrnkGuI74cPHjv2mz8+NzE+np2e8fh3v71w3hwqOTfTxZ64q17rRk+RD888WCqxUJSFrbDOKdNw65GGUz6F8atvmP3DmIiMKV/HF7W/KW3fOeUzqUGWIzwsZY0XhwsUJp3rgF+hDL2qqqOI/EipRk5eQAzOoKeRi42lwSmFdAJwDTjIgKzB5KwNOJRdY5GLjGm9kxhKHLziL5AQEGGuDPtFmSpgBGpq637x62c6e7rj42O//61vrFmxDAKlGsJJB6Ag5FBXzJ34upDIoA7uOQqJl1rb9h0+UlPXAEzFOKpFOM85yVu2Fn4/NMCEDXEfVf68dbBC1lPFdWkKEi4QhCbaIhBgfcBIqMfwMi69JE4KtYwIYjA0nNKFdBcgotaADawIQzweci6SDAUJd3Hx6IBcCQA12jjplFoQJTcwOPTMcy9WVNWEh4bff+ft2IlDWKoQnXREDe+SIlkoLAJgw9NU3cjGatipCg9xKheptLw39exLr/zxxb889bvn+PZhUtCQs6Xpty9PvyvAP6TLWvt53VPWiX5ZEy5FZw0X4lS9dPyF5h5IVFJv/xrVE5SKFK2nMh2c8jXNt6DUBmBxInASsWSdFWQSYDpzTsROUip7g9FFsuDJIbxKEECmg4IrAzWi1VcQ22nf//jTT77Yjc25V23acN9dt4Mb+26BFECYP4wnqyQRZ4ygLcswxRQcRaaUKfWVUNVQ/gKR/5NV4mEV05kCQdCnvbMD30sMWoawQd2ZpoboRZma8vf131zwzaKUtfjKorTz0xPNb0z62GWrKgpKMqSgyov9lal4AjA5Mn7VjGpI3ESIiONzy8m0uiLWJmn48SSmA9eCUrEhY9pH8WqutJN0WjqGACvIKuqinIeRswLRvk9AllmxKDFAxnRawUzYhPbsy9ttdtuc4tnfeewRfMsFDwAcZdza3oHPa4hS+/AC04ngKNhdnpyU5Kd6vJaOYjoFZAQdQ64t1wQLm3fecsPf/fDH//j9b0EEqDicAlq7jVh8l8c90m6p7h1s3FX7O4yBUqNKuFQnplz4kgayfXTKSFgzDZIteK5hUqcOgg0vQhb1aBxD8fkyp3TBUbBmAad4Vx6gxaWYrAQ0pspRLU4PRhFSeEBmwkW41hMcQSA9KFd8eIA5oEttbdg3/vV778rOyIBOvn40/Gvv7Pr5r3/b0NTs7+cHEcygjFjVkCIkm8kHL9RnU47cnKzvPfowPrKh7Hj3CG1Fvjgdpye5TOLSBQviYqOXL5yPjyI4EGCyGVeAuSn6Idtff9s2Zp+98paj1meGxlr2NTx/a8nPAv1CmaIuVBLSGYJTYxnuhKohF4zUJONfGdmprVr6sLaXtpL5cCmsptSZzSBfJkN4OnQDvVzehkQOECI8WM2Yc6LVeiQv1xNnznz42edwu82bNmy9YiN3MnDGf1U1dZ/s3O0DB2WnxeKjLz9feCc7zJh44R+G7FydKXzP7ZicBOxiWXlxYcGj99/DPYkjeLaD0FZkTZd1rqrdMQnz2O00psZvbUODOSoK3z1yZC6uubX1k117bOO2Hy3+Vkla2cWOnWdb3p+ffP3shI2qqjreTp+WE5jC7tSRES8/LLjrFPNQrHphU6ym1NHrkHSm10V1yCLK0RRkaIqQ+nD93PERZamiu/grco40Y27xGfbLr72F72Cw3+Lhe78WEhKiZJC1MtmZ6Wgx9xw80tXVlZQQt2ThwoDAAEIQSqoaoLHu7u09fuqM3T6xZMkifDUGcZwVfAUBfMGDjxhdDxWZ1u7yKGcJ5Pgml1f2vf19P/zJz9evXvGdRx+anHS0d3RgG3xCfHxyfHx8TPSozTa7YG5myNcrO46OjA8cano+L2ZpkD9p5foROVKTZbupMNd/vdGcU3JTUEuobUN0ANWoLsTxKotTc27e9Ck1jIwZ8+BkCrLkkeDF9TPyEWKMfiaSjAFdbpGrw8dPHj5+HFXgtmuvmj2rQCYBcnZmxo/+8e/Kq6vxlcyVG9f/0/e/A+fjyvCSEMqCVUtr+9cefQyzNrfffP31W7fypon7JQbLZ85dXDivBKcMyCLksJxHd5nCm4KPGZXeKvUuOm12mrZExxd9YlNE2KMP3IdvJ+fPLbHZbMnJcZOTy0sSt57tfLOm70Blz4F5SVdzfWS5SljkxEWaWxDXU9bcLSpLUKRrPRIpBoBbNjIm5zb96Hta/VDSvDhlsdNSyci6sOxnsmFhLx2mMTo2Nv76O+/j2IrkxMS7br4RjsXaboFIHDDOHR62Bvj74fvrkGB8wE0fcdMf9ZcF8TF3YFZm+pzZxWhVKypr8YU3zAeGaO7xe+zUmf/63R/HbRMkQnmcIQFRA5r2VM4IVoRxlAE3l9lsSk1OXDJ/HqgulpefPX9h94FDre3tiHKVEPD3D9yQ93BEcPTIiPVw08u2yVEhYtqAUNQDpmx8D2heJsk5dUcicIQlNU4Jpb3R24jjzgU9lZJBR2KrZQSIDPBgL54f/J4rK4O7wOGuunIDfE4WwnCIn802PjKKMwV80bKTTPaQ1+gC1LD6lhQV4bCgsxdKcagAl8Jt9N6HnyJ3YWFKNclsIr/zJFlkX9idoJQpZ7bQDyAQHbHh09La1tTcOjo2jjC+5fUPDMzPy09KTEQUZxvg+w0E8KRFzS0wozfpU9d3rGngrKIVT/P465TqAs1YqhKSR0oJTx+Uc6pLEzYBDs8CzM9xFKcEBv4Bhn8etSMq7zVUpTj18UBLSVrZHpAFRzk/2EX/wcefDwwOxMTG3Hz9daQqZVRhQ0HG34bO4ITd39cvMlza36CK1uD7+MwvKY6MiLzU2trV3aswZLnq7uvNy85C/co1gQyVgVCNpKupzqzo0HA0Fj4Zm2AOBySUT28vCcrHgD8tdfniBdFmZTOlcD5U1UvT7wgNjrTZh062vIlPyEUBO2WrgvUQt3FFQ9esdEqr+XLHTKjqDgFwYWcKSwxBqzglMDgSQE77eWD5v5UkKaeYT4VA866u7gNHjmA3yorFiwrzcriOGiszZBxPhWNVcCYaDp8S+ZDtIvtXQV5uUlIiRjzVdXVA5tZDhxJf+QwOWfDhNoBchKqIYKkENApQYWgfBx1AxMZLPrnZWdnpqTjQhWOgnqCxP3uCg4P9/amwSM8pn5zYpdlxi6Ym/ao69w6MtcsKc3z+KzuHzq9kNBEGjuwrAq4PiCZAn6DENcaUcMDfGzUUpxSEgp2cH1WUwJIC0wnxkC6Xlgc0SZgmT9x8+BWqHjt1urW9C93Da7Zc4fxY2+AGGJ5j6xAG3TgLDcxlNRRZkuPgWJXczAy4IMZGhAxsX9+GhqaGpqZzpaWtrdTh86ogFdb0R86sXwBNl6LyowQa9fiGhSqvCgbgxJs9NBpjTTzFcJiRf8j8hOt9p/z7x1oquvcwFJkrA2gVk/KkpF72H2FwIwdZCZ1hkWTUAUDOTSDDAzVOKQtz4Z2yQKGO0iRqDC0SERB66KiFEhzZHZrMyhjmTKAqNMeCzb5DRyYmbBmpyUsXLSTR2kYBEJ5BHJY2PjEe6B+IQ4AIjemo00fIgnPPLp4Fpykrq4QIDu8fHLQOj/b19PUPDQpMOSBzM7qsyCxIAv39c7IyRDcgNzvLFBXBWaVnpMXFK9+e43vc+Lg4WURRwqboqCQfP0dF1y7HFFYdvXo0aLIejNoA0PAUtNywPCrnFNgyB33GWf0qI3B8zg3Iwv00U0JyKQp1XAJFqtBDJ0xG4GGOgDxwXfUaqwTT8gEiNwcwZSa9/f1nzpciCaesxMVEq/zoL3IrcoH+ZE9/P+5fAxeslOA7MgxsQ0NwrGSgxqKMHlSgXTCnJCQktLymZmhoGGe5MI4YmTjgr0oNx5DlH6EY9JFzJEfBGRWwxTpy5y03YYCPiSdArr96c2RkBA9fsWY1BlIIg3PxrAKsBbS0tUMoZtdDQoJjwlKLkzYea3ytc6R2dGI4PFBp9GU1jGFZGVcViaygnlpDq+ZL5BTYLohBAyh/pDpCBdFf5JrXH6KMNE7JUWXWcljDSNVJBhrDRnI5D0Z8LyE663CqyuoaTPL5BwasWbkc+RQ5RCoP9w8MomvY2tZx4MgxP39fjGdf+MsrOdnZ8XExmWmpmenpaKmjzWaykfYpzM+LiTa1t3XU1NUtmDeHdQxwBiW6guxXiyzeOg6WVZWTIKWrp+fg4WNnL5ZilGMbH2cvgF9AoD9GPbASEPz9/HFvJo4XxAuA5hsvALjh9NWF8+auWbEUFefatEcHR7uiQzID/AK1Wlx2jPQ1lpobdl4gchRmUdkUMkO5pDhc45Scg0zsDGvL2QmX2athoawHNIGjEn3Zv3sPHRkdHcFRZiuWLNbxQmVz+NjJIydP4qRJDIawqRYDC3//AHgnJgJR5mFhIckJifiabPXypetWrUxPS5UtlZyUWJife+T4yWf/vH1OcZE5KrKju9vPn5aqL7W1pqYmR4ZH4KxK+CjcXydaE5XyjMOufvyLp77Yu3/CZvfzZ3QgBYL8gJvyhvAE+Cp1LHGqxxUb1v3kh4+nmIvuX/Csv29QgJ/+FFZJlMxx+jAn9I5cn1l9nEvjunuULIuDo2mc0jVTxk5b8xjMpxVJfFyVkFxV6GRxzXVALVe3MWQDk4hlFVVwr7zsbNR33DnYe+SD4ciOt9/bf/gIjuRDNYPD01JTcL5uQlQkHXGGlb2urt72rs5Ll1pxTMW5i2WlFdU4TWXJwvkgpzYe3b6AgJTEJAQOHztx4tQZ9P/g0dg/gaHxq2++iz4DlqrptEp2dCWOfIk2mclHtQ8OogYNNisRVx+fsxcu7N53EHoGBgWQtorzKW4p24GweSrUZZioOr/Yt/+Ga7ZesX5tsH8EdWLYy4CJTGw3gQLQUOagVWSaGCckfrxIJHQAZszWFR+JpRKU2SIvsJ0nWdyN9BgGdY1iuFvIcNkjZTgPc7X0ggx47hD6B4dwrqSfn2/xrPyAAOWgC2QP33f/7vmXdu0/GBoUtGLJolXLluKYydSkJCzPjI2Pod4xm8w4BLCtvfNiecXRk6eqamrf+ejjxqbm++++ffPG9bAN90v0OLE7A95gHx8fRVNLezZoMIxjsU6eORseHo5Do82REbGxsSlJiRlpqXNLZi9eME+chtrW3vHux59hrn71ssVYlkyIj6upa8QkAM29gxPzOdmocjbJMjSaZBZimHgjsGRVXVOLjfQdXd1I545+5sLFi2UVxYWFN1yzRYg2mNA7gKyNSsHLSI1599cVn2kp+QuNvEKiMgSRaXgXcEbayAaFscjmjDNnJTPnYY7Dw9MKcoeAvQs4jAW9PUwrKqwws+Dnh3lpnIOalpx01RWbNq5bDd9CFw0jm537D+BLCfgZ2kGQmKIit129ZfmSRZ/t2vv5rj1HT50atg5janD96lVCSZyyzyzsGxwcuAHwqanB4eHBoaH+vkGMwVtaWy8BdcoHdSR8NC8n+9uPPLh6xTLmcr7nSsvQUuMg/uqaGkyG33T9tWQUGJ0P55mrcUGKOBFRKlFWE6r1B3sdqKMJj3ztrXfoFooAlKNjz/5DZnP0xYqqLZvWYWIBogUbEeClo5YL93SR+OUCkDZDF1TV0DseMsOdBgpR59plTjwrS/l0o5DwSA8chERuLw+YUhLPPRmd12QYvoyNjmEhOyONDmAhOMsLLmd47KH7sNK9YuliTBjtPnh4ZGQkIjwMeyn6B/pR82EnGxa4cap+QX4B/PKxh+9LSox/7e33yiqrf/OH5zCKLykuAiusg+OXjI6sTvlERESgasxElUXHqE4NDVk+2bkL50njNQCKddiKvUWxZvOyxQtBCGVio6NTkhJAn5OZERUVFYblTeExxFf/sOZcuQQFmAqCgkiJbNjjFxIclJWeDp+kPR1TGKEXDltHwJ8NxfQ8eZz7Kbc5LzQOcY09I6gxGx7Jga6qwcpR8j10aOCMimLCPzxy0ycylk6gLpPGCpK0caI7QwCKJBHQJ7M4SkeoSur7+HZ0d6FSDA8L55N5KDSOgEErOl7YZ15aUfnbZ59v6+yG26CmwTAFJYenvrm5rrERxVxaUYXNbD/+weP33H4rKtTfPf/i6bPnXvjLq//6+N/hSHPsGoZKpDd2EI+PN166lJWeBs9ApYueAGa+cRw/8xXFlIEB/qWVldg+h+UZaIKDgK0j16FPuWLZEj4lidl7tlOTHA338PBtQuggsDU2shkNtn392A091HjD3bGgT/7IzIQMYoSUmJBw+8038MziFx9qYo0e+z7xcip2o/fIU0XjsiCcNp8uJOysIPIi1FK5KEotAsWEHkxhNmZ0wozoGohXAiTf0hBLEXd8uG5kR6NKoMGDCkZ1OImfT29vH2a2IyPDcUIzh8NePAB3/HTXvrKKyo7uHipXYk3bclDm3IY09eLrhx4itgb/zx+fx/kZN117TUtL6453P9x3+Gjin7ebIiPLKivhwWBITNHmTk1hxE31E8UcOIU/NycbU578TQBb+BC6jNZRa4wPTR9ipI8OJdWj7Jg7eCdyQaMlKg7fzMx0rHziUomEhHgoxtFADoY43xptM9Tt7+/v7O5GnsgPmBa8NwHpDEJ64JguTGyBJQfiZoqDR0/gaJANa1YmxmPiXbI60ZCxVYPzgISANPnhlpIhLCyqBiVFMbmEx+oMKe4MctFqnGlDKhILar7VhOn/MlS96vq4FxzdiWS2cpfI1FNdTafrwKAF+cHh+JgERxI3Frih7nnx1Tc+2fmFY3IKpYv9PqjY8rLy8vNyTKZI9C/RLausqcW9JKBC7fXex5+ilU9MSHz4vnvOlVXgINPX3nwHPoQ1bhDSKwEPCwiorKnr7R/AMYK4wclkjmRX4wTCdeBkzG3opQK3sTFaGceDo87fePcDONSFi6WhYWHrV69EfQm3RIfJMWn/2q034Wae5kstN1x7FZTEZrrR8TFc14Nu6Le+/hA22iE76OPuPXAILwm6HPA6aILqvL2jEzh0WwpeKuSQTfWzbuokalm0DyfOXcQNKpgEve+u22QH0lWeyBUrRA+W5yg8N5f3a3QTiY9arFxJPtCRkg1BAzO96iJuwNTz4ggCX5+MOEvj1ST0VF4fF3hOEM8Oeoqwa0gQdi1QfcYf5PBiReXREydRCfn5+WMtJyEu7qorN65cugSreRgKYKzQ29tfVVu7c/f+vYcO4eMH+HFnb+/nu/f+4sf/ivryP5/67/GxMYKi2CGJCUMucOcDDoemq+/Q3fT3x5AItSb0naRROaFDAUxTjdnGKMBO+W++1IZzBMJDQ7BpEvUiW+tGlQ338sHud7O5HrPo5ZU1q5YtmT93Nk4qxOJNTFwcDjx67a33Wtta58+d8+1HH8EkA+akMMEOpqgvURdinsuPJl39aFcm1sohGdJhOAgdGOju6sRKOi5H42rwIkfYWARGCNCcD8uRM+p9iCzBH40EmJKXsprK3wolBqcEHQjo1zOqoHcX0Ih1haRD4IJlRFlRrzwSajPdJyZRnfCGj5pU7hZwnZdeeR17edBQosjwuSAqDLgaPz2ay4Wb0kUkebmBwYGffbEHfTuMkc+cP4+p8ms2b/rzjtdR6gFo6MkJlAdq8xYWlSaqHCyHo9QxlxmPCaG42NDgYOwh6uvr7xvox91OnKaoIH/btTRNs2LxQsyVxsRE49o8qET+i/lO5sznzl88j3o0NBj7m7a//mb/wNCq5UuBg4+NcMhvUV7uc//z1LVbNx8/fYaqWHyMMWHD/Ojdt98EC7COBL0PatZJLPrBu/cfsI6Obdm0HlHhkZTm5jGWiBvELwtGQetkCfOCtagpCSj7hDdiwZe8QubnDRmzHDehjK7TUk4yhpX3h9430oJUp/VsqiYoxoobOBVVVdhkhovBALtu65Y7broRsz/oclXV1uNGCHgS3AUz3xgcPPbgfTjYt7KqGuvZtQ2Nx06dgk8sX7Lk7Q8+wsQnZZJYk4XgSRDBIrQ3F+0m6t3Vy5fNys/F3Q5o5dFwDw4NYioezTpXBkdgLsL6pJ8fdkmSfqwnCjZO44EL3qtJ0hwvBnod/LYyZIrVvo6LlZXY04mZJng2sx+5M/oAqcnJnBumC6IiI/jpr0wC/WCNALWp2JEp4LoAeHGX9bIkjZWXjqGHKNUXTBiXxc2ow6fS0oHkqCFNw4RSDRiAaZBkdiKsdWSO74qTW1bkHRDOfmAj7ENDFP0tPKIFx1UgNMHtR8PbuLjYe+64FR6J8t69/+BbH3yMCW2MNOaXzMY5BdijjgXu67duxrIQSh1NYV/fABguX7rw3Y8+VhSDW0Ii2ZQk438y7pRj+dLF8HXMK6GHB56oNfEhBQ5Rx5n+mEgnNF+f6tp6rA2C89ySenjSvDmzaR6HysfX4YMvJqnuVRji40byf+Z6zPswq4B1UYx4MOVeWl6FrRs0TkJ14u+PkdCxk6fxNdLQoOVCWXlIaMg3H34APV16f5hpMG+AfVDc4Yi/m2daBB0dNz4BVUE6BA9RnSxuW53D8JrSCRQvjRu+SgG5SVXA0yLJCOK1MwbAzqXSsnQqWR8f3BSLgsUiDeZQ+NQgepCvv/cBHAU+ittni/B5QWYGMHGHzW+f+xP6mhhzw0AVVdXYcfOP3/s2PGbdyuXPPP+SdYTa4lNnz2GHR0FODg6lQC1G1idvJFkUQGhqCpfeLVu86Hvf+Dpehh1vvYtr83DzCIY3qCwxM7Vg7hxcNIHFdCjQ2tZ+/DR9tzA6Omoym7My0mjYRP0q9EMxya9s74A+2IZBAxe4HRRCpe3ruwwfhpvNK5ctwSjus917qe/K9umhZUf/5OzFi9jBhEzV1tUD+4E7b0e9KApeBIRtSf+v8NFWLl+SsVCSTKMWPeMp+8t0Qjil/DsdhYt08doZAwIbSmm1FClUDaDkUAWisCzDI2Oj4yg8QDCY+OCTz0fHRuGj6LxhroTXoHC1C6XlNPileW5fbEHfd+jotx99GOMe7DA3myItwxZMOtbUN6DuWbNyZVBw8MjoCOGKh/mm3eHANOG3H30Ig+WfPfWbU2fOwvXDI8IjwsLRp8RyEc47qKisQqW7fs0qtNpXrl8LBbBohKXFlORk7i6UL5xxhQq2swvjZX9/X7T79Q0NcHeWBRt6FDhKc/6ckqDgwJdefePQsRPYBkp6U/03ic7xdVs3Y8A+NGzBdD1uyDXxnXVCVQqQEGFbFDyGd2CupmhQ/xoRUnU6vhyBKwnd4JQaEiVBzoba4TByV3npPcaIOZ1WznR3tFyWu1R05uBkmDfBoTy4wxPsUOpoqVHS5LU0BlJG5RN2bAzDZDQ40QMTTFJXjsL4oeaePaj8QI5xLao0KkLQ8AoSdCyA0y4xEsJnZf/8bz87ceosegJLFi1cuXQxPtPGKOfQseNYWoRbY2PR7KJZWA2/67abwRCtsMKfRsrkGvgS4rW33x0aGkLPFYCjJ8/gdh/4Nzy4pb31179/FjU6JHZ0dOJ2R6xaIZukC2XHNzIiAvUxZ4juZoB/IBbxca8etn1wIPulnPJXlwLUAVHyTsX2lTwwmpYVRPC3jkmcXgaoVcOTqoaaknEQLxayL7gzyYJWI4lrRWms6FwjaSjcRpys3KK4SMDaIGawR0dGcU0Tdq8BIyQ45NqtVzz30isYpaI17Ojq5AWD4Uh+TlZdYxMWXdD4oA+6cskSfvtnd3evxWKBQdH046MtOBM2mOGbXZpt4TIpdxQCJDgk5JqtV569cHH3/v0oEnyJ+72/eQT7MnsHBlBtL1ow7+ln/nC+tOz02fPYoHTLtutQF2LPB3iiIk9LwZ4j+mAMPzDZ0ROnEUWrDc619fXVtbXoNcJDMV316a49qNsIGcdcoeVGs664FHyLZiKbLrUCjNEQaPEOYEI0Pzfvrlu28WaBdGWP0xFViPKXm1sHnGlU65GglnzGEy9OBxWYUZ2Y+prSmcJDUqfB6al6JIorfCV8GUt+dWS4y7BORbAmciZBn6TSpyQlB4cEo8eGafBFPvTdNEYbt267/p0PPkHTiE0S5ZXVaCRTU5KxOP43D92Hb8PR/0PlhHH3nTffwIsQRxgMWYYBREWF9UAk4WrlsNAQjGrhGEwDko//UY0mxccX5uU99cwfsPSCZb27br0JS0F/fHF7WUUF1qMfe/h+jKtKn6yyjY9hNv7m669FVYqVTPyLjo6+fusVWOoEE1ZrwdtQOmBPD9yOj2Mo1z5TARh1CYdSazgqS9pu4oM9yx9+thPeiRudgXXg6ImE2Jiyqpobr9mKxS3myeDI+LIfJRNOgDPVRZKM9tcJuyxNZITezq/wgQVcSvLw6rg0hwZI1ZNLrk7FsUHSbDZjrZmfPsrFYbyCdwT/wecwFHj7w4+/8eB9cFYaFycnAYKeJTwPQ2/gN7e0vv/J5wwdpYkPusPQGYWvf/ORB1AHs/qJJrqVF8ThE2WKRJ3XfKkVPoFu4soliyqral9+7XX0ZQ8eO75y2WJsk4sMC+u1jWOUgz4DNnZct/VKHLkBZGzlxOQ5GmI0Q8wviCsyo7Eefw0Vj2SeBWTVQaEwOgPov+IzCdgKXQ2kYO0Hs5s52SaaXecFweiEmUj9GT4aldzQChxRagLihkIPhvK8yJCAgMYp5TSZjmdNtZ+cog+7zDRXUVZUhFmAiER+9By9i8eYTGlJSc3NzVgZ5Nmj4vP1wX5e7ADCxA3GOm+88z66dNjDhsZa3p0OfHgkFlHKKqsCaaHZgUOIcLoLJBfkZefnZhErFw8BUcxIxICZdVKRETgutsZhYYkWrNGTZT+oCafggYvnz8OCDUbVIIyKiMJAG07GLEYCiB0rAMaWIgRijogIe5getMThgzF6REQkBjqYxlJTfRbOn3v+Ytmsgjy8LaKMRarngDt/FQVKujAlBUQwFBCyBkMTEIHjIcByqaHAei6yqIDc5URQiIAHGTyJK8fDnEqmFWEEmO2pdGWeuqic5DKM3ToF+blHT56sq6/Hpt1oE411sGHsvjtu+6eqf8f3WWgTsSsbe3+wrQHLjOmpqdSPnPLBfjOMx3fu2fvJF3vQv4THjI/Z1q5YsXXTRnBAJw7TnB6e3KxMDJrQ7zx05OiVGzY8cs/XSquqMJG0YO7sL/YdQGcAfpaZkU4TQFNTvOcKI8P35s4uSkqIRw8Y043kfORvzAIIAEEVydpwRJQ2nDB8p3CSQlJ8AobklACvVR+8cmlsLl0Gqokz+8t8wskZxDyiAbliOS2CkYiTyG6AXUKUYTkb+qiqkMpONZ8a/5J/Xfifpnog9q5FqlBezPNmF2P4Cc/DIAZ1EqgwbsUdsQvnzsX34JhkgWc0Nl96/uVXTp4+l5uThW8CUZmhbS2trKpraHTYJ9Giwi/RvK5fswIz1eBAumnNLNsOhlqzcsWrb7+H/e1/2fEWZmRuuPbqjevXoFXFNfPbd7xtt9nQJb1yw1oyKRxLYldUkIc1zx1vv9vd00c5JLdUXJG15czXIJoGNgzOADTS8fPHMUkYOeGKcE4oaOVCFElyQDWYDHMXVuo8d8lfOVw2NDXfuszoo3r52lLSpypxGcmtLVgCT9W8mmrxgBdPlbk5BapQXmwlxbPQu8eNi8dPnoZTwr3AEw0Zrl4syM/DsiHmyVEtYXx94Oixo6dOY34RBY2xETCpJmMz2GuXLyvMz1myQL1lxylM5EuVygC4lQc3g7/30cdlVdWYvsEUN1bAu/sGjh4/UVlbg7Z8Pdx2yRLClTwSFsZOtjtvvhGjpYrqahwjg/4l/gNrJKGvoQijN0DpeaJTgRT0X1HdYml0zcplmPNnEKpQ8Cgk7v/Akni4PQWWNkp2584xPTvBwkMAXLhUA46QC8UhVUQ5oqZPKWiBJnuugF9GQCePc2ZqKC8Dzz8XZ0DW1VNCvhORGLIGEWNe+AR2kl9qaaW1OHbICUpxxZKFJUUFBbnZ//qzn2NYDRbAxyzjKDsRBR4Jy2EdEp+e4WOx7zz6ML4sw0qdN8UMVtjE/uDX7ujr6zt55jy6s5jrRkcC+9zGxqmOXDJ//qP330vrfoZCBn8svdx43dVr+lbge7SFc+eEhlF34sjJU7mZWfgODczPl5WjBwJ9YJxK9JV9fIry87BjDUvqVPUyR/RST8o1/jc4iawX9RsYjuaHe43Bm2UcXhjOIpHTQAinY1rKCEKQVP84yZQLQ0llEBkeI8wIAZFLoIHZ9ADv+egwkXPM++CAMtyTXFRYgH0JGHFTydEIAxsXgjGRabWOYtN4QlwsNtfg20IsKGMAi11ChQV5QMYXZ/fdfcfqZUtQAxl9yKg6dwtYG/s5sPCNvT9Y4cQRkjAktiNhURMXiN97x604V5L8XqcuNxkbTKPEnnnhZXQYsHkCk964hKq4MB8f+2LZGh9hYvl0wdwSwLFe39s3gC/FUMVy0UaVPEBk+XLYSKJPVb1GD3fjMBqGehpNoocIXa1MbuuGnpLYo6CpL5wKBh0VvEATcAoIGg3UGHHiueajpeDYOszxibGRcUtBYXpxcba/XyAmxXHVpr8f+9iD5Q9fLT764D34UgdTzWjEd+490NhyCfsQsd8WnTN8x52ZllZSVAhRXtY9HI37B8YcGDmBeVtbBw7ywxFFKSlJmGxi9+OyqsKZRTUzagYwW9TU3IyNRVw0ZlWxSsmRWtva0tUVoKFhK/bqchzeznDLq+yUvy6BRCXhyWGAdarpUgWdHg4y9sjk7qQLJu4CMhPgBOiFuaEzojHTwB01KRrumhR95iU5WjwpwWXQJfbQSP87R1+YdNiCgkKCUDMGhAT7h4QGoc6KjAwzRYSao/DPFHXF+tV8vTErKxNbbDBPXpiXi0oOHgb3gjjF1UhZZMw5BGTmxiIk1ihtkw7sBZ9AGBt8gEWz4NjaE+IzZ1Hq3IXp2PITEBCM9wF9VJxoGuAbhADn5jI7aPqx3o3jjZCKbU3DlmEMyRHGqhIO0eS3WKBGx/mA6BUggJEOt7nO8py5EagpEZcaSEAdsi4qITqDrDgURJ101UOcyO5CujKFU1Kbfxk+rtMA8gQTl5nhgqn4adrOnXrMHbRvtltUNQH+hDuy23obD1d9Cnf0w7whNnb5BgQFhuBr14gwU1RItDkszhweFxOREG9Kxr9YU2JK8qygAPrAijuiHAAMF9VMTI6NTAxYbX2jk4Pjk5ZxO+6GHxydHJ6YtNomx+1TtilfO4wwOTnhsNMXC1ADu32CAoP9fYJ8HdjCiTWmsNCgqBD/iOCA8CC/8GC/iLAgc2igOTggwh8nW6Ei9/Fpam3DPhJ8H4wwuqT1jQ3YW77t6q3Y0TE8MlJeXYOhfVdvb2VVDWyGM6czUlNJVQ8WRLL0uLS0XEAuENRkZMldKyhJQNAFDwaFczkThXtoaSkmYVEUzTdxdJdJ2FmUGaF7fMCEc3etI6MFQzdZUFi7oAUIfN08nGFUWPSNKx6o6bjYN9Q5Nj6C7RS0ZAL3HyJabIuBj2LCKDwo3BwZG2dKToxJSzanJ5rTk2LSE0wp8FekCgkj9qH6gaMdI6VdQw1wynGHZWzSYpscsU1ga9zYpM+Ewwfn/WBBHDs5AgJ9A4P8QwN9QwP9QvxwEpCPbXisBxvLsWcC+3ew4SjQPzjELyLYJyo8MNYcmpZgysbpP7Fh2fEhWThuJTUx8aF77sZBMZCO79fmzZ7Nj+61jo6AHAMm+6Qd90LgW0ZkB2wVU6hmmlEBiQyq1AKgBBQ4jf0YxE2/TE/mNq7xabc+pvot1axMLs3MueX5f0mCwSnlkuC5QIbRidxz4b2GrsrOnraeoba+0c6B4b6RkWEs5qDJo69Y6NVDX5NWCzEHFBYUYQqPTYpLS4nOTI/LTY7OSo3JSohOCwsOH7UPNQwca7Ycb+g72WmtHh7vI1qHP/wQGyWCA0PDg02hAXHBvqYg3wjUx2DvoJp1fNRuGR7r7R9tRC2Lrm1IIFWNEcGxkQGJ4f5xkSFxEYHxEaExYYGxUSHJMSGpmJqH/dGhxPwUJuEn7Y7K6rrExITkxAQcc7Br30GTybR+1XKsH+45eBjVzqY1K/kMPC8Z9BzQY5lwjE04xiOCYtFETF9irNyncTaDwadnK2F4qBElLLdBkMMpoQK9IYKXsc5GwROG9nEBNIJ02ePdBC2fryoG/VFIw2ODA5bePmtX73B7S09TY2t1S09Dj7XNMj6IviAtFKi71tAC4GpYSIfDRoRExUUmp8bk5KbOykosTIuFj6ZP+o50j9a0W8ob+y90jpT1j3SGBSQmRWbHRqYH+WGsPWGbHIMXouM3Pjk85ts3OtE/MjbssPtEhsYkRGSaQ9LjwrLNIUkRQfERAXF+GH5N2ccdoyNAGx+AJ9kmrfBjzEfBKthGjNY8JCgswCc0yD8sCie7hOWG+8XgXRL2gb42x+iYfXBovHvY1osAanT8Wif6UAGvTLkH9a6zLyLIWK1nLD4pnQV1JaVPdh1nNYLWN7zgIzxNMNU5jtMpBYa7gI7SHZpLuEqr/nWJNEOgi7zhmmK77VjVLkwDpcZmm0JjhseG0NdsH2iq7yiv76hs7akftPaj2qMJIx9amKaHZqtpyhrL02FhkfFRSekxefmpc3NTi7MTZsVGJVps7R0jFT3WS9HB6bap0a7hmoGRtv7xVutkx9BYr3V8CIOekMCwuIj0pNDC+JCCxKj8uIgsU1AyasHB8Y7B8XbLWPeQrW1oogMBq61/dGIQ3QCbA6OlCV+6xMcBB8eHRFGhpkjfzOSIouz4RTnmFVGBiUgamRjEAdIDY20WW+fQWKfF3tE/2mYZ6x2bHMQrYXNY0ZG9Nu+JFal3+cpH4Hp0DrkY+MtMvSDvHkHLAyLqHbVrLB0TT04JVPHw4hPRrzjAKmdZMzksZMmtNoB6p2TFgEb8/WMv7zz3RkJUWnZiUU5KYXp8XpI53Tpmae6uqW+rqLh0rq69rLP/0vjEKJvLpHUUWuDGH9p3i24AfYAWFhqZEpuZk1hcnL4oP21OZnwBJjvRMa3tP3ai+dWqnn2WyXZ8khvgGxIbmZYSWZwSVpIcVZQYnh8ZHGe1DXRb6wbGWjutNV3DCLRZbb2oTR2o6agbyrpOdFAHZtNQQ+I+0JDooLQUc0FKxKzooJzYsKzIkHjU632jLWDSO9rQZa3vH2mxjveM2YftPqN2H2zxxKJoaGRwbFRocoZp4dW5/4i7l4WhKMCsoYG4iujt7AUVNzsn1BeBKxHewHSNs6ZPqVNRF/XAXecuHjCRNC3ytAiCv9EogHQOtvzXu48fLf8iKjwqPiYlIy4vN74kO3FWTkpxfFRy50BLfXtF2aVT5c1nmrurUZVCHC3x0ewWqNl/JAC7HeGdfrGRCTnJRbOzFhelLcxLLsG6Zddw7SXLmZr+g/3WrvSouTmxS1Iii2JCsqwTvR2Wqq6R2vbhix1DqE3bRycH7A5MQMLmEAJzsgCroOGOOFcSLpUYlpcUNjvVVJIcWRAWEGuxdfeM1ndZ6tqGKntGGywTHWAyYR+jVUYohd6wb0hMeGJCVG58cGFCZF50aGpCWEF0MK36CLO4DOhKUxd1STItUOdMAh/5nVYfgWwM4GtOfLGBkpguT0ZSj5CvnKFHaZpEFH9p44lfvfcPDe0VbNplKsQ/LMGUlp9aMjdveUHq3KyEwjHbSF1HeXnz6fMNR6taLg6N9GLMQfu68YCeVRjwAhq4s053eGhkemzO7MwlC/NXzUpbYA6P7bRWDY73oG4LCzS1DZd3WCqaBk+3Wi72D18asQ+yc4Jopzj3QzIuZ0d2nsQAKD48N9O8INO8OCEsH72CEftA+1BFq6W83VreM1oHhx63DU/60K4lvDL4ThN6hQfFJoXPSo2Yk2aenRiRH+gT7uPnHx+WCc2NHmB0C6VEWNY09vImcnlU3nB2haPUlDof0kU5IXNdChpN4IqzE+bufXJizDzENcQvHlhMPFxJaPjF2Td///G/Dwz3YK8QHAtegvnsWHNCXsqcOVnLilIX5KWUBAUE1bVXXGg4fqb2QGXb+SFrH27WJp/kfkmMmQSMvMk57TgcPTMpb07G8gU5K4szF6MShVx07I60/OlY06t9Y43QxRdNOqdiOjHdwI4GYfDw8JAYVIqZpsWZpoWpUbMDfcM6hitbh8ubB05f6r/YP3rJNmWld2KKPsEBA/RW/f2DokNS00zzcqKXpZvmRAUkD9t7Okeqmgcq82PWzEnYYPQ/Jln9UTOhxp1/uQ2d8a8uNI1KHgXBVlj7ftIjzowT5SLhxLwmnikjKGck0YFY5o1YBEmLy0XZll86Y3dMoMaij1j9/TA6xlin+tKFxs4qTBsBlJc8e07WkqykQsyo4ziAAWsfLo6gWhPzs8K3WJYw7YhhR+9gZ21bWV1bebcFt9dMmcJiMG8Pl0d/dNw+MjYxCKFo9PkuTITAhRzaxxEVFpcbs3Jhys1LUm6dk3BVWGDMpaELZ9vfP9X25sXOj5oGTg/buuCC2DFM5DSTYMcOofiInNmJW5al3bEw+caUiNm9Y82lXV+can37TMf7wYERC5OvjwiMhkTFBDrrcKhLoELwf/aPx9dDVkXTp5QT5DDypeZbBith8VrAjWiaRUUmr6Ke2vSPINGiugEzJCGUk+iiHDg40veXXU9/dOoV+6SNEMhHqAKiWs8xGRoanp1UtLhg7cKc1WiR0VBWtZw/UbkPg/eGzgp84ohak5TnGYAuIoxNRlOoNUOyUwoXZa9dWrChKH3BhGOkof94Wc9n1X0HBkc64FtouCEGjXVYYFymaX5R0sZs85Kk8EKLrbd54Ez9wLGGvmPdw3XjkyPwQvJiTAxBFqpaPzTWvtEhaXlxq2bFrs+KXgwdGgdP1/Ufaxw40WWpH50YyopZdkvxTzOiaNuo9w+9Zzwj3tP8VTFZcXAJSmGzP758sPllRSssvWUDFwGq8xWX6GbEyaUvCmZ4KzCseWn3f31x6i1anqbOGXtYydAk0NRkeHBkXurs5bOuJwhXxAAAMoFJREFUWJS3DrXmqM16seH4gdKPT1bv7x6k+0HoTQMRFadSedKLRn1FjNMnQwLC8lPmrizZvKxwY3ZCYe9oU3XPvvMdHzf2nUZDHBoQlWqaOzvhimzzitSIYstEb9Pgqeqe/fV9x/tGGiYcNrYsTqZgEuhtgU9ikjIvenVx/Kbs6GUBfsH1A8erevbWEUkTJkdR76ZHz7m+4EeFceumeeWlIuciZmRbTvLX+1U8gOVeVgx29n/yiSc8CwYBI9RjuQTqkDgOxADuDb6O3F1UYevFEC88JCozPr/f0tvUhbVj+lCG6UFVOEJo1icmJzA9VN12sbW3YXJqAsPzwrR5uSmz46KSR8at/cNdWNim1p/lgP2wVp3YYMTuh4XvjoHm2vbSjv5mOG5KdF5u7BLMewdhddsvaGHqTcvT7i5J2BoUEFrTe/hE645Tba/X9hy0jHcBGcuKqlVo/I81oQC/0LzYFSsy7luWekeGeUHrUOmJ1tdOtL1a23MIS5cwI5w2KaLwqoLHixOuYBzcWYjBtRZHjGRO92iJpsP+kulaYbxswNKr5lsRfbm1v5fm0GXw8qh0TCiHvr6NndV/2fPfBy5+RIs6GKvSW0L9CrIJ/UEHDp1CR1JcBtriNSVXz81ajra7rOn0vosfHCr9tIuqTHYkAeeukNIfetDsYnTs8EmPz1lVtHlVyVVFaQuw0NI1Up8SUQzm6C/W9h+o6NrTOVQz6RjntSOtdRItyyUiflOx4dlzErbOTbwqw7SgzVJZ0b2rrOvz1iF0JIgEwySsuSdFFm3O+/78xGuoe3BZD2UZmVYyr2EBn3DZdmmQvtIIysFlZa9s8hVO6kkoqy48IPAME4LrPHsgvZwkpzgDtS47mMHJSiywjg9f6q7DuAepoFUac+KCKOoqfyxRYgRzqacO84LRkRinz8YI3RwRh75pr6UDRaZjS4XIK15UdD6+g5ae2o4yLCChP5poykw1FWLdzzLes6/+mdPtO7AsxOZBaZaS+t3qgzFQYEBoUcKGtVkPYwyEeZ+yri8ONL5wtv3dbmsDupn00SMWKKfsyVHFW/L/dlqPlHiTDHoBjY8Mc1VYOgoZXZfkbRTZdqmJK3p157mrNMCENvryYPhI5f/01IJM4iDhSMkCqtXYFYZA1Qdc6qZDwt6L7MRCu8MOv8QkJYY1qu6gZhUEmmg2hd450FrTXtpn6QwPiciIzy9InZMSk4GhUnt/C5aqqWPK80wq4n9npYPPXu2Oyba+ptq2UkxFRYRFoQ+AWR2bY2TU3j803kmDdMWxqYjwv8N3Mi4sc0nq7WuyHiyMWYfln2Mtrx5uerGp7xQ2brLqkDKHpZ9M86LNud+bm7j1S9WRzKx6czGgzlxffVRbvp75K1NCX4li3jBxh6O3lGetp00ldnpRprDo3JTiQP+g9r5my+gg9omRX9C1CgxVsRp9LzgyPtLYVdXSUw+XS47JxIoOdmkE+gV29F8CIc6h0r1rkMUF4hdNrXV0CEvtbf1NiGJvR3bM4rjwLHhk98ilCcco+g+Ao4KE4KzoRRtyHkMPMio4sbx79/7GP6KCtIx14pAI0p46BvBev1kJ6+CRGPqw2SI5a17UcgZbkaq6DBhwZIDOjpxcRvjyYS5CcHbWlDrZsiQkeUiVMeXwZZDI5C7CVL8oXC+PeVhwBFrk6Ij4AWtPz1CHMvRB7ogdY4kfOCq1eb5wwdq28tEJa2xkIlp/EEaGmXstnX3DXUqbCH0YGZCpRaYY8YK7Y4myrbcB5GO2UWzpyI6dnxQxC9suBydasUsIcoMDQuckXrMx57GShC2j9sETrW/ua/xDQ98JHIHOd6CBGboBocFRC5K3bcp9LNe8nHHX/TCdtTAXIBXBQ5KKMoO/woc4jS6qYTTdu6NVzOvJc9ddE43ky4xoFdIw0SdJHskdQIPtPgJ78X9AwW7z3OTilJhMVF09wx0jaMoxX02dQvVBiAxMdR71MttLu4fa0fpnJuRjzijenNI/3N3R18IcGiTE2EnLeLAmGkD/IWs/Bua9w134JCMrriTdNDciKG5wrAODlqWpd63PeRTrOi2WMvQgjza/3Gdt8qcFQ/JtNOyopLG0vTLzvrVZDyaH08dDl/O48QaDypfDW0dDNuOW0yUwqBHmAUI1JWfnAWnaJF2pTIvvHcJfhyvVZb7YAYSZ86jwmOGRoQFrNybD4YKyPF5sqPMmJm2NXTXY/BYeGpUWm42OaUp01pB1oL2vCU0wGntQKW+I0wMIRu+BHy7CsTV2V7f1NeBcybTY/AzzPMyKY3Pa8vSvYfGwpu/w7rrfnu94f2xiGJ9wEA1VkPaQoKhZCRvW0dDn5sgg5cpvvdGc4vQpzricJSfUQ0jzgnnrGK40EZK9ZaIqBcvzPqViVRXu7V8h2FsCj3haboi518qNFbyvAGAptMUFKSWpsVmBgSGW0QHL2AAWYKhKZf8LTVGJojXvGGhp6qqG46bF5WDqJyMhb9w+2tJbz9aKxJy8MupxZoQzwxx+fwvG9ZiQSo3NyYiZkxY1JyQgsqp33+f1v6ztPohaEVOhmByhd8M/IDVqztL0O9Zk3l8QswaTnUITfUD1H5el7tRBT+YxrvL0iMSqRC2GO8tDDdelKJWgUVWlTyky5o67VgclZmQnICLgkpADhVBvoho+LrkDKBlAxxzknEjAEcAeopTY7PyUkqTodKyDj4wP0dZxXK5Nq89qU0RVIcrKv3+wq66tDKP2jIR8tP5ZibNs9vGG9kqcBUwDcho00R9FNSJS/6En7OOH3R617WVjDitkRYdT5Vfff/xM2zsTU2MQTavevgHxYTnzk69dlXXfouRtqEdJh8t9FDUulxx08ARhq2nYcGGSn02DT9w9ofg/8aMnOIZRA88OCraSD3iSoaapBabGPf/1qLZn0mlSdTnF6Cc3qTgnaRYG2tjJOzY+ah23YLMwlQrNHLFc0nmQ/taxwfqeCsxbZ8UX4nMzrCvippyGjkq7D/Z8QF8Md3h/UlUApLyo4NR+/uOT2ClZMWjtjY1ITjCnoF0O8g3vHavDh5GJ4QVzErcsz7hrSdqt6ZFzPVSQ05rlyyNM4zJq5vR/3Qt2OhK3hp7SGeeY/k88+aQTJoVIxMy8SCL+vyPo3kp6/eCmpvAYtOY5ScXp8bnmsBjskbDZR+FztDOSvuMBCiYi4Vgj9V1l+F4rM542FmHvMD63beym+pJ8naYHCFMvALZEC427SBz2urbq9t6mGFN8RtysNHNJSFBkfFj+ktTbFqbcmGVaRPd3u3lcMHWFydFcIrsEuuLxV4N51IB5JFUBzikhoyIeORjR9ZAvSa5nJ8XBmTxAfZwvogpx+ddJIJL1L64vtqJhiJ2fOgcDGvT/zOExAf5Bk76TGO7gexo+yT4+PlbbWgH5aPdjsC89aRa+tahpLcfkI/kueBJbdfaKT4XSDiDamoR9QEGBQSMTw+GhEbPTF4cGRiRHzso0LUqNnI3Pw0kvvUpC15kFZPtwSkAEbxem8MxeUHpGU1N5iXgpBWjAxyOWHGmNH1BmbpWl+78sY1CQHjnMIbpfEqMMHXQp/w9EkTtUgd0DbRjNtPQ2YiIdWy6wBXPA0me1WbF1Miw07PbVf3P9kvsjQ80Yif9p5y93n3+XfW9OfkmFCBuzURK+KA8JCo0IMmG1M96cnJaQhaFSUnTG7NQlOLnAe1uwYvIefVpM4ueOp+wiHhh5ieaBg8skckr2erpMdQF0lw0XqKxg0G6hjKb1YJlcmiOXwd6HZ6Tj9GxRw/Hlb2xm6xnq6h/uGRrv6x/uwJEsN614pDB1HnKHeZ/nP/t5a08DTh/AIQSY+wzCuTHB4RGhkfjiNjoiFvuPYiLi40xJ8aakiBCTQUUDgF57/O+sPzwo6oLYA7YhaQbkXlSZMyprrosup66d0sjXCDFkzQVgBrkFtRcZlmUYmXtQ0ogsWM30dcdkELZdDo8O4QCj6Mj4ZHMmf6vrOiqwCw79Tnr8cGxGML7jDg2OCA+OCA0KDw4IYVlEJsnb6M//088MC8uYVw9GcG5do945Ro/uzYV0PDOypgfBRi3/tyA6JXVRr7RSSwhvBSzk0kSe2bpJdQOWdVJFy7DLDvO3Wvdue3xpvdBQqw3czLWBJDR14pdAGo9k9pUQmTu6NLcGSRuZKT4p4WroquXK2jUd6EtEhZI8yyI6A5bQmanNX2kUlPGZju1lEUGMSzqjeG8hpKauYhLjDyZNJ2+6bBnkevBIbn/8Op2S9cudPFT7OpXwwluc5F6FXHGUTQAmXFEdN24JkQddKo+6JJQxjQjIshEouLlLYgggVZVig1xZkDHstKmaJvuBMVXF4p4vYpcZwGvvToTOxYxoZCK31G700TF1gwUwNwJ+nU5JUJUe2vAykI0lUt2znWGKFxy5AkaHAETkQZYqTEY9EY/1CPd+wVmXXwHnzCGLxBlLSZYthXXkpAkrS1WKhMqCQm0gqIWgx0HcC4OJ11inK8QThPhr3nsdmlOoqpLTBxXlmRKCmyBwz0igeApwhhzD3UBH03eEPGEpypUzphGjZgO2E+hOBJEKEHFg88zAFHBBJUNEWDASaIDIqQIugAIiIypAKMisKJA5f0Gig4tUAQemCCNVjhrDRghn6O4X+EgS/GVyJwkzosBxwpkyImpEIObOEjWmC1J9gNNxu+nTdHFewDogRUkwE+7UgGPJcedAxwWHmYCQuZNnznV2dS9bvBA3JJBW0oNUHAmOW+WsVuviBfNxhYww9InTZzu6ulcsXURUrKTPnL94qbVtxZJFuDfk9LkLHZ2dWAzBxHNISFBudja/dgmYmI4+cvwUbufE6c6ZGWlka1aQuJfkYnkljnZevngh5HKeLW3t50vLY8wmqCfKAQFcmXji9Jne/oHYmGichp+GU+9VbwMfXK7Y09sHqsUL5+MAaUjA7SfdPT24TQz4EDc8PHzw6InIyAhcoQzMoydPAY6bo3Cd2akz53CF8jx22w0wmy61IF/FBfm4E+Dk6fPdvT1wZCxdIgknpS+YNwfH6x84ciw2xoxrfgDE09PXd+TYSVyOtmj+3OaeurqOi5hpwtdEgQFB6Qk5mQkFwOkd6iptOoZ80zGFAf7RYfGY+ce0qLAGprEuNpzAdruwkHB8E4clK8abv0UI+tZ2lDZ11sxKW5RoSjlVtw8HdSONNiLjwyXHFD7yHLVZKlpPZ8TlYzEWBIPWvjN1hzCTiK+T6zrLLMN9VNSoT2mTAH3hWZg6H4cqQh+v3Jdro/7y8tLcDiG81UN1qJI7XzaB/Nb7H+49dPi///OnuAlL65NEVN/Q9NNf/RrHLj58z52PPnCfcJcd775/4NDRW66/+vHvfZu7y/uffvbhZ7t+8/OfwCnf+fjTXXsPYMSG3OIQ0dSk5Cs2rPna7bfgJmSH3fHCX14rrajYsnH9j3/wOOZhIAWnMj/zwov7Dh7GJcZwa0A4z+1vvvPuR58kJsQ99/SvcHQ+KUS3czb89L9+jYvGcFQVLkjEPQyPf/ebuMIbJfrex59uf+Ot1o5unDQOs+CO2Mceum/d6pUv73jj7IXS5379K+6U3b19P/+f32elJcMp4cQ/+eXTSxbMg1edv1D6418+nZeT+dRPf5yajOtBfU6dO/9vv3j6e19/EOf+v7Rjx5mzF6ldxITmlMMyNPSTH/wTrhz9xW+ewVlF//Gjf17E7gFqam79t188hfMpFy+YV9p84sW9P/ebCMAxmFhnT4hLun7ZvVsX3NHYWfPbD3+MkxZwUCV8I9g3bEH+qge3/AOWmiD0aOXud48/V49dI+MT+PQtMS5187zbrltyH92MSw808Nlf+vH7R1/6/vU/jw6P+/Pup/ss3bTu748rpKZwrOHT33ijpafuqXf/6cYVD8EpsbK1/cCvd51/Z+Psm5fP2rh9z9MN7dUBfrh8FUdC42Qw+9jE6N9s/TGc8jI8Esrwd4mcUniVqNZ19RxwjI+oCQVyb39/W1s7VuGMyIDsPnCwurYOgZ27999z+224Q4mj9Q8MdnV3v/3hJyuWLkGpA9jfP9DR0YHjaxHu6+9vam7Cbe45OdmDg0O4svi3z76Am5oee/gB1J0g7Orp+XzPvjtuvmFOcRHwUVt/secAjg3v6aaTw3m1hypn1779XYTdhcvgr7+Kro5D0o533tt74DDO4ocH19TVnzl/ITCQtoodPHLs5//9W8uQ5Zqtm7Mz03Hf454Dh3A3GZJ6evtbW+nqY4Tx4IoJ1LXm8HCErSOjra1tcF+ELVZrR2fHkGXoT9tf/cHffhcvjMUyfKm5eXDIgtT+vsG2ro5tW7fihnGsrU+M23ArGY407+zsws0pv3/+paf/4yeofW10r2Nbb18/SEbGLK2djQuz1s7JXGYZ6zta8/kLn/9XdvxsHHbQOXApOyV/ReG19gnb2bojn51+PTE67Z5N361pK/39xz/GF8CrZ2/OSZjdM9S6v+yj7fueDg8xbZ5/CxU/1XA++N4NHMYd+PzI0dnbMTxi2bL0JtxBgaVVtLSmcHNNh7VrsAXLB1Djg2Pb8S8rqeDapXcGB4UuyF6XnVA8ONq198IHcRFpa2Zfi7ojLTYHmC4fvATcbXiVBPEMIlxJIaIrNaW+tT7ZJWt3QDg323xI75/8QINh68gnO3ehgU5NTj5XWoobiVctX8rrMPyizOA3v33uT7htBLWj3e7Aq0+tAT67orMjfa+7asvq5ctQYGtWLv/hj3/2ylvvXLlhXW5WVgDu8vQPwGWur7z+1k9/9AOcU/+X199CZYyrQvG6owXhO8oPHz1RU1O/ZOGCiuqaXXv3X3PlJhzhAs6l5dUomZuuvWrdmlXDw9bWtg7cFIHGdPsbb+Mmsu8++gjupsXNodYR6xXr16K6An+cKAS78kt6oB6Wbxx2HPlMdkOtB7Zs8RvwQByhAQXefv/DpQvnb9m0EdnE8cE4hJJyhb1uUz533LStII/OlsFjioysqqv3D0RxTB44evT1d99/+N67oTzY4FhsUKCYcM56Sfay29c+iuoK2d5x4I/nG47MyliAEy6To3NuW/MN8Jmbs/zJHY+cazx4l+Nbn5x+tbG77JrF99676bvhwVE2+xg2z//Px//67pEXVs66EqtKZB960IlgxqYDsX2jokz3bKS3iLwW+1TCYnEpEVYCIsPDT9ceemnn0+EB5m9s/hEOCUNxX7v0LjTyNW0Xd597PyMp/66130bG4PSMljLKH17ryW6hIpAM48O2XNELI5MY0QhBYOiQpaioNPUccA91aWX1vJKi2266DsW/c+9+gQHTwPSzCgrOl1e++OoOqOsf4IcOCVuTh1zaEo5rX9ElwPVvV65bC29ubGw+ceYcihieGxwcnJqYhHuxK2tq0bDu2b8fnbDAoED2pTYJgSd9snsPNizee9ctWVlph0+cbG7F1bMoCr+01CRYfMe7H3z02Rc4737xwnmov9s7u09fuIAe5G03bgMrXN6Nu2xxrRhu3CF28MgA/1PnLuBiG/zDXUzwU3yRQ0ngSVxpQgOL/rjhKS0lEVf3/P6Fl1HfQ0+60ZsWdX1YJeRTXV+PG/jOXyjDxU1wYtw2YbdNpCSlREZE/vm1N3DyPp2ISVehEUN+MKEp0oQPjBJMqWGBUTgCeGzSCsk4HhgfteGOAXxLFB5iRnE4/Oy4wOV09X6sc16z9K7k6MzIEDM+rdw478b0uLzarrKW3gbw5GUOU5DL4x1FZ9p/yjfQVtV+DifYlDWe7exvo+PBALf7NXXU/HnfL20+I49e9YOSrMWgRmFjNZ8JjcJrhvul8TVSTEQiX7si/k6/olzjf/rj8SFlcBC/DgdQ1Ys1KbK7yWEgIcr90tiz5TKA89kXu8dGRjZvXL9504bf/OEF9Pm6H3mQup60KwyfWU3eecsNL/z5lVfeeGvN8qXInmNyAnUGUnHzBwoSroMwjIgTv3Ozs/AdAtpiRCds9pDgQLTd//Pcn15+9fXBQQsK6ebrr/7D8y/j7efvUUNT84kTZ+YUF29cvbqquu7p3z2L1pmPlm6/aVt5RdWps+fQr8BR+OgXon6yWIYG+geLCnJxly3JZAaB0yCfqD9QAaJ6fmH7q7xORFWDSj0YqVSpQ1PaCYQwPBX1+vLFi2GaN9/76M873sTdYeieYocbUnEPFd60P774F9ztDEhqUuLyxYsAsU860FsonlX4mz8+9/sXX8JbAX9R9s4F4PULxLe/ey68PzjSc6D0U7+p0LykOZj5wpnU7ZbGHQefAatTVQeslrGS1BVo+vuGesL8TSno3uEhx5uiM95jUxv6yvDRnE8qZY2lUOWOjJBf2nyGxod+++G/olDwlq4o3FycMR+vvo8j4Fz1sZFJCy7fzU+jL0JlJ8GYk7jjA2I3z7S+KOg4W8UphZ/JwgQqDyAHnrmzPGqIwA05Rz2x9+Dh8PAQeC2KPyEx7vz5suOnzly79UpgoyBRfrPy8h+9754f/cevfvfCy2hAaVsD7itGKr3FirdzI46OjKLaQIPCJE3hMqUNa1eh/vtk52402Fdt3oQ7aHFN4oSNuqR4MKTt6OqYO6foQnk5bhNDY/jZF7tuu2lbSFAw7hj95U9+dPzU2WMnT508c7aqugbjp/WrViAj6EKwrSpkbTBROhOwAHr/9sl5xcXoZsDrcavIx1/swgH6DAea4gYTO8Jo3/0DAk3R5tu2XXfo6IlXdrwxf/48vGx0dThV3ugFOBbMKUlOTkJFjnuh4QQQR58y+vthGHfkxAkM72hHOvVeQIGbA/yDgoNPlu07V3kMe4pRP9606oF52SvKm07j+NWu/uZ3j76As4lHx0evW3H31YvvhCrU7gfAz+k1wAPrQQJczceOe1XYQjwrHZx1wF5ghkFHIQYszF6DPVDjExMFyXSAFq4MBI7dz5aXWNTQVfH+sZe+dd2/wy+RpHoLuYbLllj4FWng7oG5ObHqYfqaUiZUcRSYK4/UokAxVp2AgDsQpzx57kJLRwcM+7vnXkT+0XNHC/jZ7j1br9hATTB9X4VvqX0xqjh6+synX+zBrYa4gob7AeoJMA0KVPTEjUwnzp1HP6+4sBBwMrTDEW0233P7rd96/IcgvPeO2+B5+M4BboKOgc0+sffg0eCQ0DPnLjz+xL/bJjAIDSyvqa2rb5xdVAjfwA30qCM3rlu979CRJ//zVwcOH73p+muSkxJb2ttKKyoxF8MzgkuScRcdWEIdNAuoYmfPKkT109LS9tHOL7jpmTpUm1L20bdyTI2PjaFKfuSeu3/yq6dPnTkfGByInJJN0Cnx9737tpsKcnORC6qAUXfiKrSgYHQBcDXENx968Lv/9C+79x1AecFWoEAFPD5qm5dXMid7GayVmZhfkrk4PCSSbk5xTGQkFNy49IHXDj5T1XIBJxviiwt8qpGZnHux8fjF+uMb59/Ac9HcU3OptzY8wJRsziAtWC2CtwKNLz6Ro286A3zCQqLuXv89dDagGL4DBhoWWHwDJufmLrlv3T/8fMf3Pjv55oKcVevnbkOS+lCtwZgRTxCqcI9/gQX3waPi08Z9qrJ9PTmlO96SJ2pQyCMD/HCzMeobVAYo8jR0xxITP/nsCxTP9ddsxaWtqFRQqp98vvPI8RO4+h0+gfYPDFHY6L09cu898J7Ori7kERxIY3ixn19NXWN0dHRvbz9miy6WlmOucfH8+eDPSpeqE1yhvBH3IcfF4S5bjGZQkGgZUczlldWnz5/PzEjdtHoNjq/A1EV5Vc2h48f3HjqEy9q3v/42pg9v2XYNqj2qp/A4HDFm88Y1a5598eWnn/nDI/ffg2vsm1va3v3o05uvuwZSWFNHd33igkdoZxm2YjaEahpuW9arRBh8JuzjmCWA8jdcd9XJc+c+372fXpIJqreoEfALaGvvjAiPmMTt3ezFQ9FTN2WSPsNYtWLpXbfc+Pwrr6GAcZUpY4jDru1zC5bdvOwhFGZocBiu5wGcXmq/KXNw3Oria2Dkn9V/94vTb60tucYcEbux5MbT5UdeO/AM7hjNTizGh8JvHnyu19K+ZeFdOO51x4Hf4VwabGquaLyIo7VM4XF0nBIKJ9DRbWkJGsHo246SKkibC3IcdZQYlY0DE2/f8M1ff/T3rx35bUHafHyoBAX4g/JSvFKFTP+Xe6SEp77bU3BKycckDA9BjSdyPCYAkxqYJnz2pe1BgYEoY1j4O19/ePXyxbgDJtpk/uaDD8TGRtNrNOXT0dFJk5oHDsMpMSGCrhZXqLgw77GH7n/yF79CK8PbQdaj8sPEymtvv4Nb4voHhuYVF33/sUfNZhPE2SfQ4qEqdURGRPzL338fjoh/AI3hZg87NaMYAGGa5vZt1z360L3UC/D1PX3+wv7DRz7bte+uW2+5UF7x2a49x06eRl3b2NwUEhyEYT7qrfvvvg0zRJgnb+3oDA8N6x8c6BsYxJTT+tUr4G3jE3AtYk4PVcb2MeU2xSnIRd8DYLg3csU7AGaT6RsP3ocradF/5R1EoI2Pjv7698+FR4TbJyYwA//Pf/udgvzcEVyBwnrPuJnvvrvvxP3P0A3vEhhiNOwTOIG7otAvVKoiFIMvvsfF/OQ4znXB7T5r5l63ouzDQ1WffnT6L3eu+fa6km3ljef3Vb73zOdPmEMSRsYHOwYvLchZe/f6b3UPdrx99HkMj8LCwhu6q+bnrMIuegyzMA1nsfU99cHj8HTkEqOy/3hgO05vHbGMoD7DkHNdyTWn6vZ8fu6tv+x+6jvb/jOEdwN8cNTROJ3yikfvHNz19FDCdP/wT2xZOq9/XaGytwDJBt+WkFHerR3tMSZzSlIiKpL4mJiE+PhlixegIsT0yuZN6zHXiBoUjWAEdhiGBMN901KSF86b09IOKtOV69fGRJvBBDN2KKe46JhN69ZglhuLPcBHi4aBQk5GxvXXbHnga3eWFM1CDYSMNrdcwv1cV6xbA75wLLgmNIKzdnX1zCmatXTRQsw9xcfG/X/VnXuMF9UVxyG+WPYBLruwsLgKuqgowgq7KMpDVKzS+qjRptU0NrVpbdO06X/9p2nS/9qmqWlS21pto9XYFyQaa2uVYKWCT1BQER8gW5ZdFnAfsA9A6efcM3Pnzp078/v9EEj7Q/d377nf87jnnrkzc++Z39z5hVtbZ86sYQSqqxsn1fcPHKirq+HegjN4XU01QUYc82bmu+744soVV3PmYnPlkotnn1lfz/HCsdHSPI17DhaSYO/ctauxoWHFVcuwVnQd5nUmPdya8H5ZQrN3/356xIIrS6q8MmzJostRAQzjuajAgVddeQXLQLt2726snzR16pSG+vpJ9fUTa2sXtbdPa2rq2t09v23ewvlthF1dbQ3r87xujP0h9sD6hz8aPTTSNmMJT6tFXpdRGTvE2xyH918yY+HFLR38prD8guHIIHmcVOvGT+Rn3pvqm0cPDRNhvFVt+SW33H7lPazmsCdEhvLgUB939+0XLL9j6bdZXDz88ZHu/TubJk4/s2ZyPa8wqJ3Mnf7C1uW4eujQgbZzF8+aNoefcphyZguvDjrl6Gmt0+ZMqJZtLa5lPxrcO7tlAWuokW0VfmmAWabw3rc2u1OoRqTOZ5Y5hkW74Vzw8QZWPBUdymPGMHKskvBqQVa5dAtEWYaGhwGzQst+4x5ehzAywtmQ+VVbaQJARHJnwAUoa5xyvMhS0amEHWd5K3+3WW1my8TcFRnruCU/fHh3Tw/rKZyU2fZkWuOtxZyDrNnsAYLh4OGemgX/gYFBzppoZ2eSw0aF04v9LOsPDIDkFd50hGUpmhBI0GMtUzJV5mOkwQsFOjuiVeN4MVPDwaEhesEcycu+1SFclbKkjxOQw4L/yAiL1Vgk1/jMvqjmYCAomSNZlICFKYeJk/X+6vHVUHhbD1uFrBqyvqgdUczIkWF+EY63pxFzCDt85FBvfxdTGg9bcumJ/KFDA30H9nGJyXU2LwtkEUc7yI9/kEIvPxs7rpZlHa5yOeH09HVBEbvkkkr82VA3TV6eMrinpmoC53p4WQBGhdhc16T7mayb9g508SYhlqUMm/V0VCB4gpETdcRMry6mrL1vl6H4dK9IF++WfWMzdY+dKpCC/mQEVEwQjXp57ShybRYLCn0aVKmBaFiNyw0oOGAuu9dfqUrQRue+RKbyGMvwjvfxhHitMWsUJSVNCrJXQMQ8tTPmMceSdMlV7ZYBlhWUscDS35700gylELi4ICizrf7I0UNn5s4zL5FjvJUHKzDW1WJhZgCklu1CEA8yscRKiQt5LHF7xd8lupmJp4oVlGLI62yJoPQcUaIbWSMyDNaOTEuW+QRSrBnFOvzux/ENHUY7hxULKW5VSzxFIRbfYX49xPO/RlObS1puls0S28GnPq7f/bYUMKfC0WaOftts5wzTYsmBgg68x16mDR6MgfcUWDM8ulfV7lt26w0KtuyxlFV1zTHlMqT5DvPrnuISzR76JFXVKM80HGA9rHZEQalecn3lmql0ZHniXEywrMqyEeAZ4fFGXPG1lLbGojxsUdV2J2tAHptlSQABUtJYUMJg/aQwTLHmJsYSC8JRu2yRlRXKMLsYkqcdLqcplpETGeX0zsSVw4902ciSicShxr13zylZBLx5g13QhOzi1lh58u2akVBNyYoCY5vUEUqRMsuTJglGAPTSAF2MMlr3JYymwZUsAsyhEhHT0lSOOsoV4pbBuAI9pSqhTKIHo+opEl35Y1SyVY3Rv8ihkDfcLrKisifWWktQ4tpARHrSLQP0YDkh6rA4/DjLetAhFxWRJs3OvWceWkbi6Ji+gf59+/tY1ZvS2MBCDOpYc2GlUNahTOfI0ayrq2V3qGfvXvZsWAyCkRUfFm5Y3SAloqamRse1r7+f/TZdeLLD/P72HSxrzzinRdetYAHG6iM5lORWosXtIFwsnrOixFIOdBa8eM08a7cqn458sGMna+xkXrI9zTYP3WSNk+wklmOam5qqqqK8ceYL1puGhofYNDqn5SzdwMRgJE+cWDelUbL3kYaoAwcPst45zGd09MwJ0ZqR67FkdFxqqIzxbl9CkJiWGei4IfBdbIDX6iyeB0RVQEJu9LEFSzDTQ1wr61tiUsMyB+5aBuQvj//thZde7urqnlDHb1A0QHlv+45VJA63z1cB9/7mQXYU58656Je//T3L17rMTmbx02vW8kAFK/C6GA743l8/+PY773bMb6OMFsLrkT+vfn7Di2+/s40QPL/1XOhkcP7ukT+SVfTm1q1PPbuWjEntvloFgCj54Y9/NvPssxomTbr/oUfJVJo35yLoxBlK16xbv3Xbtk1vbJ578WxST6Bv3LzlgT881tO9Z+2/17eeO4O1TIgksvzkF/exidXT20umqSZW3v/wo5u3vM1OOou405unAmOx9kc//Tnb9B98uPOZtc+T92nMMO7TknEmyHI+cJQDE0weUEdOfZHzNzEuBlgKgov2vgUnc1XMF39jjaFLAyVmdQTh7q3b3h8aYqHbyo8Zom/oKVmmElFEoneCoEU1pcXIsm19PSleKLLHdGdXF2Vmsqk8/SN87OuM7OzsElZT7+rq2vDSK22XzOnq2cPupIrs7d1LdJ4/q1X3PxHY1d29o7PzNJM1oqv9723fTrr79775dTa4yR1GCzD2U3l8hyz3TVveYjndKEmd3NhOfHvru4+teuK2m1c+/tTT1129TDVyYDy79vnvfONuIuy+Bx96eeMbSxbJLgjbQuNOP+2mz17/5D+eWf3EU9/62lcgso3a+Z9d5DVPbZxsVe/c2bloYftb297buPnNjgVtAhsd5Sh67K+rL5p9AfarIutqcaskEnxMvilTNhUZL+NtgsoMSDQEhlGdZ4pRXVojvHCZ6yHh1E8UHQpgb5lUmtnnt7IJFLXnfLkqI0ExEg8XBaWcVzIhqeolGsRC88+Ig0L2F0ctWzj6NLlAwBj75Vvy0EwummSCUZKQkl/AAqEwQRjBFEwrPNJmVIohRgRbLHh/5jktRm30hyDo6+sbGDzIT6UYcdg29tTTRA0SEXDGuPFkjz/8p1U9PIYWuwRLO7t2T96xo6NtrnKtXbdestY//uTF1zbecM1y2MkCIV+JCZKyuf4WjeQ18XjDhpdf297ZyUalkMzHWColMsamN085fPjQvb96gIfCpFfms39/HzvM5IJQ44TLoxRK5yQ+YULtzLNbzpsxw2ZAk+mBpsGBAySa2Nw58qnWPPfC8OjIbTeuVN6h0ZFZ580kf2LV40+xTQUx8oDpu7qNI/nFVzb17uulyThUL7NxKDs0ApGRNv6nayLWeE1CVg2Hx6SDC0FG0VAZfZjket30WwZTtqYunHUeSmyXRVqFHz3085jUolSrnRB0J0TbxLixYxl1LrCk1+GP0qNeGjcE5IdZJR75CJ6L4NqaajFD4jkaAHKSuCgkda2vb5Cda2DkhhwaMSmV5i5nZHTkio72VzZtWvPcvzRTE8zw0FD7vDnXLFtK7jBVLv3WbXip5azpZMj+89m11y5dzKXnDJMywpUA+4pkNF67bDFIMorYPb9p5XUkYr76+hYo+lHnYBRG8mNrN96w4s233qmpreHECgAvtbQ0s/+5+sm/V1dVkVqqTwvRdMrpp3bu6iZzhexM0pxVGnlWpK9XVZ3BzjVTHWl+0LmC5EkyDp4oiwqHHPmEzLdbb7nxu9//Qb15xhJF72//cM/efYs65oteMeaUFcuXMME7gyMDZQ6WYxkFtdD7izS2c0klrTQik9E1EotmSk+lreJ0OUiigygi00U9TC0sW1BG/tJk2aUq3intGmWPxALXrpjB7ri0jbsHNpfJulXA5En1JIAhVg+SxZd3NDZOunnl9YRLdbXcSSDt0nlzpzc18RAjw8aHI+qy9gVXL1kE5rl160kFIiiZpe6+686Nb7zJ/viVl7UDQ35tdc3nPrOiZXozNzEka9q+WNvGjx939bLFcy68kEQQblbYAVdG7o3u/vKXXn19c/eRI3fe/vnWmTOUfm5LC3M/J/errlxEcqfxzlH2wZdefjlPj5BvqTD+8jTIFR0LmpunsplOld41NExafNnClubme756F8+OYSBEzicwRvYYF5MhYB2oBSHzcZxvARGj84VMtcFw5C68RBxxvDsCiore2Mu1YBE8v40OebLysRW0WH95I13krzFjBw8yBEN4jbtmxhJ9ZCGR/0ZVdfMYYfX4KoLso/5+biP00aSDB4d4toEUdNXF3DN44ICmUJCMwd26wgjHvXv3EbeEFGkTZuo5yukb4cw95GEwc3s9xBKCRmMRAPFdU02OsMxTLBFwA835lGtWZl8YIZLxhkbKwLjM1eHnry4OcEXBZbSGEB0Bw1IAUyZPPMLCJErfMYZ7fDSpMXrLjwGeYSe16kwc6I3COk107bFxf+xB6Yoz5WOJ0rwQdIXnyc2jKy+tfI75sCkWrir+r/8WHOSpfh1fR+RHpKvUXNW6hMIyFupgh1CpACDqQxifplOUNyl6oJRc2pK6r8Ktg0qAnsRPX3U1VSiNaCjg8Nq8aoCxPD9bRivQ9bklWpgWxNQcJ+axeBL8anlsBGV5wER8WXg9ASVMmVKxlKLWuM11q4rPcWBGtyEUB0cJUSWaI41BFa7ZWYAn2KuK3Lj7kY4yrsUjZP5XQIsBu6Ym3EZjHksCy5Q8w2nPUpSJoIxXrVwpOTohhw11eLOOdhqjItZYDdk51W3N8h4viteRPAeVry4rwVORFVUSkGVJHBdoK02ybo+gfr20BHNhnMCCw61ErymrKktRufyWTehOKuvgxIwSpbIc7Zx0mFM9bdjq9adAZTamFRyUECQqXpQ6VhVojOUHIHTE60sWVBKQZfn0lFylmYaKnIBhweFWYrApry+u72VFzZm28lhy6XQqGO95dBVEcORZrHETbM3KtBRb8AyFzsda6MJs2cN4Ek5S1Vrj6ZNjxSOVW83rV54qV255mNxBdEVVWuaokLvvcixQ0S7SLVvFEPnYILD0YEElBOUYfH5LUFwlxILQr0RMLrbgqIMHt5e85s4VndNwonskZpupxO2a3xEdeA2CHDstuWB0C5aEwlxhqlWVLqh5xTGqAt2uIsPvbVqsAPLn2gw2RSjHpBRDulJR99OsqVqunIqmxjLAuYpic0oCYuDJ+/bvvhlslMv/Zr4zhZQ1XoQpPoVwKoA9vNMYFRXgna/9icS94jB8JcVmFSmlpEnZLsNou3nMej17cuUE1TvMqfZUxQE5xVxFMcYkMcSVk/JtrbYFT63cfXskqgFSFKmCdWWlgskMndualVzcqvgAJrDwEbQxqzBACch3UFm5dCvVTQdcblGEhtVmDrcSIgPmZUVkQSo1j56v0zXaHpku3AWUQwdjrbAFiK5wgjL18bzvsmk5zwiRYpiBuRiUufpSykzFBWdbgxRO7tCt3GL5WQklO5IaZemUvz6QlVmCIva6vkzg3uGW540sPaF4IpCdtCWKpBSmhw1TTrdNY8PztguIlBlSgB41h79UuLb5QalUHfUgN8pK6nMBrjIEuk3pSqQNvwkmhfMN0ZO7HQtPhaAL2VVcAcRKFqQZSMAFPlGBx+lvuYGj9ofRaVPKwaQ5imoBb3vwtL50zYOGq/41paL8S7owb0L1jp6owQ67NIdsMzSLilTrVwyPv8O6PN4UKDXdJS1llbJaiXMnVLXdotzeWWJQUXErLHbIfWTcVVeXqohbggqFWBLgcaK6fOeJnYXoMrT7/AV3356poSoWBXSmqLEToyGlav0eknicaSlTHNkl7+4dbHHRaMAJfFf+Eec4mWOVCxC1gRE4BkEZlhMnOaPKI0RvHPM96tc9LlvN+ANGj9eEYDLJUPUAVtiJKGQMjJUkFsWUzLfa6VpL2a0aDqMhQ80ICxPwhk7Axyqg4oiM54iwPS4113UuqLBcvi7EuGC9phQDlKreCRqkTfyN+EMgaNmJ0AN6Vdsv1yxLPEEFa0NBNCjGIo0lpvdhmwokhRmEGjOlteTj81u8S16vavmyo2ObjlfBjmM5umIHuGHDr2HGc4aIUDHut7FUDmc+BBzPYch/0W8eC41/RgJ/9aN9o0zBpYBMmkRW/F9EjpUro0XHralvIzehGBapqjFGshoQYVSXqViTFEALVsYwrUVVRVq8AAUqBMXbgqlGjUlTDFOK/FXzkro8qGBrNKYAWbAoN2LUprR8NcaaJAXAKl4LWo71qYy4Fonly0rQJq/q4g1PrMI0qFLTjZgemWF06x9DsdpNyr80GF7pM8X/Aug80u+4Mq+fAAAAAElFTkSuQmCC" alt="Logo" style="max-height:48px; background:#fff; border-radius:8px; padding:4px 10px; display:block;"></div>
    <h1 id="title">Drillmaschinenüberwachung</h1>
    <div class="meta">
      <span id="ip">IP: -</span> · <span id="version">Version: -</span> · <span id="updated">-</span>
      · <span id="authBadge" class="auth-badge">Nicht angemeldet</span>
    </div>
    <div id="debugOverlay" class="warning hidden" style="display:none;margin-bottom:10px;font-size:.9rem;"></div>
    <div id="connection" class="connection">
      <span>Kontakt: <span class="connection-dot"></span><strong id="connectionState">OK</strong></span>
      <span>ESP Temperatur: <strong id="espTemp">-</strong></span>
      <span>Hubwerk: <strong id="liftStatusConn">-</strong></span>
    </div>
    <div id="gnssWarning" class="warning hidden"></div>
    <nav class="tabs" aria-label="Ansicht">
      <button id="monitoringTab" class="tab-button active" type="button">Überwachung</button>
      <button id="mapTab" class="tab-button" type="button">Karte</button>
      <button id="lightOn" class="nav-action light-unknown" type="button">Licht</button>
      <span id="fanStatus" class="nav-action secondary">Lüfter aus</span>
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
        <div>Hubwerk: <strong id="liftStatus">-</strong></div>
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
    <section class="panel monitoring-view">
      <h2>Was wird gedrillt?</h2>
      <div class="field-row">
        <input id="fieldInput" maxlength="31" value="Feld" placeholder="Feldname">
        <button id="fieldSave" type="button">Feld speichern</button>
      </div>
      <div class="field-row">
        <input id="cropInput" list="cropSuggestions" maxlength="23" placeholder="Saat auswählen oder frei eingeben" aria-label="Saat auswählen oder frei eingeben">
        <datalist id="cropSuggestions"></datalist>
        <button id="cropSave" type="button">Saat speichern</button>
      </div>
      <div id="cropQuickList" class="crop-chip-row"></div>
      <div class="gps-meta">
        <div>Aktuell: <strong id="cropCurrent">-</strong></div>
        <div>Feld: <strong id="fieldCurrent">-</strong></div>
      </div>
    </section>
    <section id="settingsView" class="panel hidden">
      <h2>Fehleranalyse & Einstellungen</h2>
      <details id="userAdminPanel" class="admin-section">
        <summary>Benutzerverwaltung</summary>
        <div class="admin-section-content">
        <div class="gps-meta" id="sessionInfo"></div>
        <div class="user-admin-grid" style="margin-top:10px;">
          <input id="userFormUsername" maxlength="31" placeholder="Benutzername">
          <input id="userFormPassword" type="password" placeholder="Passwort">
          <select id="userFormRole">
            <option value="viewer">Viewer</option>
            <option value="operator">Operator</option>
            <option value="admin">Admin</option>
          </select>
          <label style="display:flex;align-items:center;gap:6px;color:#d1d5db;white-space:nowrap;">
            <input id="userFormEnabled" type="checkbox" checked>
            Aktiv
          </label>
        </div>
        <div class="actions" style="margin-top:10px;">
          <button id="userSaveBtn" type="button">Benutzer speichern</button>
          <button id="userReloadBtn" class="secondary" type="button">Neu laden</button>
        </div>
        <div id="userAdminStatus" class="error" style="margin-top:10px;"></div>
        <div id="userList" class="user-admin-list"></div>
        </div>
      </details>
      <details class="admin-section">
        <summary>Kameras</summary>
        <div class="admin-section-content">
          <div id="cameraSettingsGrid" class="camera-settings-grid"></div>
        </div>
      </details>
      <details class="admin-section settings-only">
        <summary>Kanaldetails</summary>
        <div class="admin-section-content">
        <div id="sensorDetailsGrid" class="sensor-table"></div>
        </div>
      </details>
      <details class="admin-section">
        <summary>Systemauslastung</summary>
        <div class="admin-section-content">
          <div id="systemLoadGrid" class="sensor-table"></div>
        </div>
      </details>
      <details class="admin-section" open>
        <summary>Empfindlichkeit</summary>
        <div class="admin-section-content">
          <div class="field-row">
            <input id="sensitivityInput" type="number" min="300" max="10000" step="100" value="1500">
            <button id="sensitivitySave" type="button">Speichern</button>
          </div>
          <div class="gps-meta">
            <div>Signalpegel 100 % bei: <strong id="sensitivityCurrent">-</strong></div>
            <div>Rot/Alarm ab: <strong id="redSignalCurrent">-</strong></div>
          </div>
          <div class="setting-hint">
            <strong>Was stellt man hier ein?</strong> Die Zeit, die der Lichttaster (Sensor) <strong>ununterbrochen</strong> ein Signal melden muss, bevor Alarm (roter Rahmen + Ton, "Dauersignal") ausgelöst wird. Ein Dauersignal bedeutet Verstopfung/Störung, keine normale Kornerkennung.
            <ul>
              <li><strong>Feine, dicht fließende Saat</strong> (z.B. Raps): Wert eher <strong>erhöhen</strong>. Bei feiner Saat kann der Lichtstrahl auch im normalen Betrieb länger am Stück unterbrochen sein &ndash; ein zu niedriger Wert löst dann fälschlich Alarm aus.</li>
              <li><strong>Grobe, einzeln fallende Saat</strong> (z.B. Bohnen, Mais): Wert kann niedrig bleiben &ndash; eine echte Verstopfung wird dann schneller erkannt.</li>
            </ul>
            Hinweis: Diese Einstellung regelt nicht, wie empfindlich der Sensor selbst einzelne Körner erkennt. Erkennt der Sensor feine Saat generell zu selten/schwach, hilft nur der Empfindlichkeits-Regler direkt am Lichttaster bzw. dessen Montageposition &ndash; nicht dieser Wert.
          </div>
        </div>
      </details>
      <details class="admin-section" open>
        <summary>Hubwerk (ISO 11786 PIN 5 · DI7)</summary>
        <div class="admin-section-content">
          <div class="gps-meta">
            <div>Position: <strong id="liftStatusSettings">-</strong></div>
            <div>Auto-Stop nach Ausheben nach: <strong id="liftAutoStopCurrent">-</strong></div>
          </div>
          <div class="field-row" style="margin-top:6px;">
            <input id="liftAutoStopInput" type="number" min="0" max="3600000" step="30000" value="600000">
            <button id="liftAutoStopSave" type="button">Speichern</button>
          </div>
          <div class="gps-meta" style="color:#9ca3af;font-size:.82rem;">0 ms = kein Auto-Stop · Wert in Millisekunden</div>
        </div>
      </details>
      <details class="admin-section">
        <summary>GNSS-Diagnose</summary>
        <div class="admin-section-content">
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
      </details>
      <details id="cropAdminPanel" class="admin-section">
        <summary>Saat-Vorschläge verwalten</summary>
        <div class="admin-section-content">
          <div style="display:flex;gap:8px;margin-bottom:8px;">
            <input id="cropAddInput" placeholder="Neue Saat hinzufügen" style="flex:1; height:36px;" />
            <button id="cropAddBtn" class="secondary" type="button">Hinzufügen</button>
          </div>
          <div id="cropSuggestionsList" style="color:#d1d5db;font-size:.95rem;"></div>
        </div>
      </details>
      <details class="admin-section" open>
        <summary>Gerätename</summary>
        <div class="admin-section-content">
          <div class="field-row" style="margin-bottom:6px;">
            <input id="deviceNameInput" maxlength="63" placeholder="z.B. Rabe Megadrill 3000-01" style="flex:1;" />
            <button id="deviceNameSave" type="button">Speichern</button>
          </div>
          <div class="gps-meta" id="deviceNameStatus"></div>
        </div>
      </details>
      <details class="admin-section">
        <summary>Cloud-Upload (Landmaschinenmanager)</summary>
        <div class="admin-section-content">
          <div class="field-row" style="margin-bottom:6px;">
            <input id="uploadUrlInput" placeholder="https://server/api/trips/upload" style="flex:1; min-width:220px;" />
          </div>
          <div class="field-row" style="margin-bottom:6px;">
            <input id="uploadTokenInput" type="password" placeholder="API-Token (leer lassen = unverändert)" style="flex:1; min-width:220px;" autocomplete="new-password" />
          </div>
          <div class="field-row" style="margin-bottom:6px; align-items:center;">
            <label style="display:flex; align-items:center; gap:6px;">
              <input id="autoUploadInput" type="checkbox" />
              Automatisch nach Fahrtende hochladen
            </label>
          </div>
          <div class="actions">
            <button id="uploadConfigSave" type="button">Speichern</button>
            <button id="uploadConfigTest" class="secondary" type="button">Verbindung testen</button>
            <button id="uploadNowBtn" class="secondary" type="button">Jetzt hochladen</button>
          </div>
          <div class="gps-meta" id="uploadConfigStatus"></div>
        </div>
      </details>
      <details class="admin-section">
        <summary>Dateien und Logs</summary>
        <div class="admin-section-content actions">
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
    <section class="panel monitoring-view">
      <div class="recording-row">
        <div>
          <div class="recording-label" id="recStatusLabel">Aufzeichnung: <span id="gpsStatus">–</span></div>
          <div class="recording-trip">Fahrt-ID: <span id="tripId">–</span></div>
        </div>
        <div style="display:flex; gap:8px;">
          <button id="gpsStart" type="button">&#9654; Start</button>
          <button id="gpsStop" class="danger" type="button">&#9632; Stop</button>
        </div>
      </div>
    </section>
    <div id="liftConfirmModal" class="modal-overlay hidden">
      <div class="modal-box">
        <h3 id="liftConfirmTitle">Fahrt abgeschlossen</h3>
        <p><span id="liftConfirmMessage">Die Aufzeichnung wurde beendet und steht zum Download bereit.</span><br>
           Fahrt: <strong id="liftConfirmTripId">–</strong></p>
        <div class="actions">
          <button id="liftConfirmUploadBtn" type="button">&#8679; Jetzt hochladen</button>
          <button id="liftConfirmCloseBtn" class="secondary" type="button">Schlie&szlig;en</button>
        </div>
        <div id="liftConfirmDownload" class="hidden" style="margin-top:12px;">
          <div style="color:#9ca3af; font-size:.88rem; margin-bottom:6px;">Direkt herunterladen:</div>
          <div class="actions">
            <a id="liftDlGps" class="link-button secondary" href="/api/gps-log.csv">GPS-Daten (CSV)</a>
            <a id="liftDlSensor" class="link-button secondary" href="/api/sensor-events.csv">Sensordaten (CSV)</a>
            <a id="liftDlCombined" class="link-button secondary" href="/api/combined.geojson">Route + Sensoren</a>
          </div>
        </div>
        <div id="liftConfirmStatus" class="gps-meta" style="margin-top:8px;"></div>
      </div>
    </div>
    <div id="leaveConfirmModal" class="modal-overlay hidden">
      <div class="modal-box">
        <h3>Aufzeichnung l&auml;uft</h3>
        <p>Es l&auml;uft gerade eine Fahrtaufzeichnung. Sie l&auml;uft auf dem Ger&auml;t weiter, auch wenn du diese Seite verl&auml;sst &ndash; du verlierst dabei aber die Live-Ansicht (Karte, Kamera, Status), bis du die Seite wieder &ouml;ffnest.</p>
        <div class="actions">
          <button id="leaveConfirmStayBtn" type="button">Auf der Seite bleiben</button>
          <button id="leaveConfirmLeaveBtn" class="secondary" type="button">Trotzdem verlassen</button>
        </div>
      </div>
    </div>
    <div class="hidden" aria-hidden="true">
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
    let liftConfirmShown = false;
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
    let fanOnState = false;
    let fanSwitchable = false;
    let portalUploadUrl = '';
    let portalUploadToken = '';
    let portalAutoUpload = false;
    let browserUploadBusy = false;
    let recordingStateKnown = false;
    let lastRecordingActive = false;
    let leaveGuardArmed = false;

    function isRecordingActive() {
      return Boolean((window.lastStatusData || {}).recording_active);
    }

    function syncLeaveGuard(active) {
      if (active && !leaveGuardArmed) {
        leaveGuardArmed = true;
        history.pushState({ leaveGuard: true }, '', location.href);
      } else if (!active && leaveGuardArmed) {
        leaveGuardArmed = false;
        if (history.state && history.state.leaveGuard) {
          history.back();
        }
      }
    }
    let lastAutoUploadTripId = '';
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
    const authStorageKey = 'pmDeviceBinding';
    const emptyAuthState = { loggedIn: false, username: '', role: '', expiresInMs: 0, bootstrapRequired: false };
    let authState = { ...emptyAuthState };
    let bootCompleted = false;

    function escapeHtml(value) {
      var safe = (value !== undefined && value !== null) ? String(value) : '';
      return safe.replace(/[&<>"']/g, function(c) {
        return ({'&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'})[c];
      });
    }

    function visibleChannels(channels) {
      return (channels || []).filter(ch => !ch.hidden && ch.channel !== 7);
    }

    function generateBindingToken() {
      const bytes = new Uint8Array(16);
      if (window.crypto && window.crypto.getRandomValues) {
        window.crypto.getRandomValues(bytes);
      } else {
        for (let i = 0; i < bytes.length; i++) bytes[i] = Math.floor(Math.random() * 256);
      }
      return Array.from(bytes, b => b.toString(16).padStart(2, '0')).join('');
    }

    function getDeviceBinding() {
      let binding = localStorage.getItem(authStorageKey);
      if (!binding) {
        binding = generateBindingToken();
        localStorage.setItem(authStorageKey, binding);
      }
      return binding;
    }

    function syncAuthBadge() {
      const badge = document.getElementById('authBadge');
      if (!badge) return;
      if (!authState.loggedIn) {
        badge.textContent = 'Nicht angemeldet';
        return;
      }
      badge.textContent = `${authState.username || '-'} · ${authState.role || '-'}`;
    }

    function setAuthOverlay(mode, message) {
      const overlay = document.getElementById('authOverlay');
      const title = document.getElementById('authTitle');
      const hint = document.getElementById('authHint');
      const msg = document.getElementById('authMessage');
      const loginRow = document.getElementById('authUsername').parentElement;
      const bootstrapRow = document.getElementById('authBootstrapUsername').parentElement;
      const bootstrapBtn = document.getElementById('authBootstrapBtn');
      const loginBtn = document.getElementById('authLoginBtn');
      const bootstrapMode = mode === 'bootstrap';
      overlay.classList.remove('hidden');
      title.textContent = bootstrapMode ? 'Erst-Admin anlegen' : 'Anmeldung';
      hint.textContent = bootstrapMode
        ? 'Es existiert noch kein Benutzer. Bitte den ersten Admin anlegen.'
        : 'Bitte mit einem berechtigten Benutzer anmelden.';
      loginRow.classList.toggle('hidden', bootstrapMode);
      bootstrapRow.classList.toggle('hidden', !bootstrapMode);
      loginBtn.style.display = bootstrapMode ? 'none' : 'inline-flex';
      bootstrapBtn.style.display = bootstrapMode ? 'inline-flex' : 'none';
      bootstrapBtn.textContent = 'Erst-Admin anlegen';
      loginBtn.textContent = 'Anmelden';
      msg.textContent = message || '';
    }

    function showAuthOverlay(mode, message) {
      authState.loggedIn = false;
      authState.username = '';
      authState.role = '';
      authState.expiresInMs = 0;
      authState.bootstrapRequired = mode === 'bootstrap';
      setAuthOverlay(mode, message);
      syncAuthBadge();
    }

    function hideAuthOverlay() {
      document.getElementById('authOverlay').classList.add('hidden');
      document.getElementById('authMessage').textContent = '';
    }

    function updateSessionPanel() {
      const sessionInfo = document.getElementById('sessionInfo');
      const userAdminPanel = document.getElementById('userAdminPanel');
      const cropAdminPanel = document.getElementById('cropAdminPanel');
      if (!sessionInfo) return;
      if (!authState.loggedIn) {
        sessionInfo.innerHTML = '<div>Session: <strong>-</strong></div><div>Rolle: <strong>-</strong></div><div>Restlaufzeit: <strong>-</strong></div>';
        if (userAdminPanel) userAdminPanel.classList.add('hidden');
        if (cropAdminPanel) cropAdminPanel.classList.add('hidden');
        return;
      }
      if (userAdminPanel) userAdminPanel.classList.toggle('hidden', authState.role !== 'admin');
      if (cropAdminPanel) cropAdminPanel.classList.toggle('hidden', authState.role !== 'admin');
      sessionInfo.innerHTML = [
        `<div>Session: <strong>${escapeHtml(authState.username || '-')}</strong></div>`,
        `<div>Rolle: <strong>${escapeHtml(authState.role || '-')}</strong></div>`,
        `<div>Restlaufzeit: <strong>${formatDuration(authState.expiresInMs || 0)}</strong></div>`
      ].join('');
    }

    function renderUserRows(list) {
      const container = document.getElementById('userList');
      if (!container) return;
      const users = Array.isArray(list) ? list : [];
      container.innerHTML = users.length ? users.map(user => `
        <div class="user-admin-row">
          <strong>${escapeHtml(user.username || '')}</strong>
          <span>${escapeHtml(user.role || '')}</span>
          <span>${user.enabled ? 'Aktiv' : 'Gesperrt'}</span>
          <button class="secondary" type="button" data-user="${escapeHtml(user.username || '')}">Löschen</button>
        </div>
      `).join('') : '<div style="color:#9ca3af;">Noch keine Benutzer angelegt.</div>';
      container.querySelectorAll('button[data-user]').forEach(button => button.addEventListener('click', async event => {
        const username = event.currentTarget.getAttribute('data-user');
        if (!username || !confirm(`Benutzer "${username}" löschen?`)) return;
        await deleteUser(username);
      }));
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

    function updateChannelGrid(channels, rotation) {
      const grid = document.getElementById('grid');
      const existing = grid.querySelectorAll('.bar-card');
      if (existing.length !== channels.length) {
        renderingGrid = true;
        grid.innerHTML = channels.map(ch => card(ch, rotation)).join('');
        setTimeout(() => { renderingGrid = false; }, 80);
        return;
      }
      channels.forEach((ch, idx) => {
        const cardEl = existing[idx];
        if (ch.rotation_channel) {
          const moving = Boolean(rotation && rotation.moving);
          const rpm = formatRpm(rotation ? rotation.rpm : ch.rotation_rpm);
          cardEl.querySelector('.status-bar').className = 'status-bar ' + (moving ? 'signal' : 'stopped');
          const fill = cardEl.querySelector('.bar-fill');
          fill.className = 'bar-fill';
          fill.style.setProperty('--level', (moving ? 100 : 0) + '%');
          cardEl.querySelector('.status-text').textContent = moving ? 'Dreht' : 'Dreht nicht';
          const rl = cardEl.querySelector('.rotation-line');
          rl.className = 'rotation-line ' + (moving ? 'on' : 'off');
          rl.textContent = rpm;
        } else {
          const quality = Math.max(0, Math.min(100, Number(ch.signal_quality_pct || 0)));
          const recentlyDetected = quality > 0;
          const level = ch.main_signal ? 100 : quality;
          const fillCls = ch.main_signal ? 'red' : (level >= 70 ? 'yellow' : '');
          const label = ch.main_signal ? 'Dauersignal' : (recentlyDetected ? level + ' %' : 'Bereit');
          const rotMoving = Boolean(rotation && rotation.moving);
          cardEl.classList.toggle('latched', Boolean(ch.latched_alarm));
          cardEl.querySelector('.status-bar').className = 'status-bar' + (recentlyDetected || ch.main_signal ? ' signal' : '');
          const fill = cardEl.querySelector('.bar-fill');
          fill.className = 'bar-fill' + (fillCls ? ' ' + fillCls : '');
          fill.style.setProperty('--level', level + '%');
          cardEl.querySelector('.status-text').textContent = label;
          const rl = cardEl.querySelector('.rotation-line');
          rl.className = 'rotation-line ' + (rotMoving ? 'on' : 'off');
          rl.textContent = rotMoving ? `Dreht · ${formatRpm(rotation.rpm)}` : 'Dreht nicht';
        }
      });
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

    function updateFanButton() {
      const status = document.getElementById('fanStatus');
      status.classList.toggle('fan-on', fanOnState);
      status.classList.toggle('secondary', !fanOnState);
      status.textContent = fanOnState ? 'Lüfter an' : 'Lüfter aus';
      status.title = fanSwitchable ? 'Automatik: Ein über 43 °C, Aus bei 41 °C' : 'Lüfterausgang nicht erreichbar';
    }

    function updateValveButtons(valves) {
      const container = document.getElementById('valveActions');
      const list = valves || [];
      const existing = container.querySelectorAll('.valve-button');
      if (existing.length !== list.length) {
        container.innerHTML = list.map(valve => `
          <button class="valve-button ${valve.active ? 'active' : ''}" type="button" data-index="${valve.index}" ${valve.switchable ? '' : 'disabled'}>
            <span>${escapeHtml(valve.label || ('Ausgang ' + valve.output_channel))}</span>
            <span class="valve-countdown">${valve.active ? Math.ceil((valve.remaining_ms || 0) / 1000) + ' s aktiv' : (valve.locked ? 'gesperrt' : '5 s schalten')}</span>
          </button>
        `).join('');
        container.querySelectorAll('.valve-button').forEach(button => button.addEventListener('click', () => pulseValve(button)));
        return;
      }
      list.forEach((valve, idx) => {
        const btn = existing[idx];
        btn.className = 'valve-button' + (valve.active ? ' active' : '');
        btn.disabled = !valve.switchable;
        btn.dataset.index = valve.index;
        const countdown = btn.querySelector('.valve-countdown');
        if (countdown) {
          countdown.textContent = valve.active ? Math.ceil((valve.remaining_ms || 0) / 1000) + ' s aktiv' : (valve.locked ? 'gesperrt' : '5 s schalten');
        }
      });
    }

    async function pulseValve(button) {
      if (actionBusy) return;
      actionBusy = true;
      const index = Number(button.dataset.index);
      try {
        const res = await fetchWithTimeout('/api/valve', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ index })
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
      if (document.body.classList.contains('settings-active') && !settingsGrid.contains(document.activeElement)) {
        renderCameraSettings(cameras);
      }
      const configured = cameras.filter(camera => camera.configured);
      const container = document.getElementById('cameraGrid');
      const cameraConfigKey = configured.map(c => c.index + ':' + (c.host || '')).join('|');
      if (container.dataset.configKey === cameraConfigKey) return;
      container.dataset.configKey = cameraConfigKey;
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
        try { loadDeviceConfig(); } catch (e) {}
        if (authState.role === 'admin') {
          try { loadUploadConfig(); } catch (e) {}
        }
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
      let minX = localPositions.reduce((m, p) => Math.min(m, p.x),  Infinity);
      let maxX = localPositions.reduce((m, p) => Math.max(m, p.x), -Infinity);
      let minY = localPositions.reduce((m, p) => Math.min(m, p.y),  Infinity);
      let maxY = localPositions.reduce((m, p) => Math.max(m, p.y), -Infinity);
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
        const headers = new Headers(options.headers || {});
        headers.set('X-Device-Binding', getDeviceBinding());
        return await fetch(url, {
          cache: 'no-store',
          credentials: 'same-origin',
          ...options,
          headers,
          signal: controller.signal
        });
      } finally {
        clearTimeout(timer);
      }
    }

    async function parseApiError(response) {
      try {
        const data = await response.json();
        return data.error || data.message || ('HTTP ' + response.status);
      } catch (err) {
        return 'HTTP ' + response.status;
      }
    }

    async function loadSession() {
      try {
        const res = await fetchWithTimeout('/api/session', {}, 2500);
        if (!res.ok) {
          const error = await parseApiError(res);
          if (res.status === 409 || error === 'bootstrap_required') {
            authState = { ...emptyAuthState, bootstrapRequired: true };
            showAuthOverlay('bootstrap', 'Bitte den ersten Admin anlegen.');
          } else {
            authState = { ...emptyAuthState };
            showAuthOverlay('login', 'Bitte anmelden.');
          }
          return false;
        }
        const data = await res.json();
        authState = {
          loggedIn: true,
          username: data.username || '',
          role: data.role || '',
          expiresInMs: Number(data.expires_in_ms || 0),
          bootstrapRequired: false
        };
        hideAuthOverlay();
        syncAuthBadge();
        updateSessionPanel();
        if (authState.role === 'admin') {
          await loadUsers();
        }
        return true;
      } catch (err) {
        authState = { ...emptyAuthState };
        showAuthOverlay('login', 'Session nicht verfügbar.');
        return false;
      }
    }

    async function loginWithCredentials(username, password) {
      const payload = { username, password };
      const res = await fetchWithTimeout('/api/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      }, 5000);
      const data = await res.json().catch(() => ({}));
      if (!res.ok) {
        if (res.status === 409 || data.error === 'bootstrap_required') {
          authState.bootstrapRequired = true;
          showAuthOverlay('bootstrap', 'Es existiert noch kein Benutzer.');
          return;
        }
        throw new Error(data.error || ('HTTP ' + res.status));
      }
      authState = {
        loggedIn: true,
        username: data.username || username,
        role: data.role || '',
        expiresInMs: Number(data.expires_in_ms || 0),
        bootstrapRequired: false
      };
      hideAuthOverlay();
      syncAuthBadge();
      updateSessionPanel();
      await refresh();
      if (authState.role === 'admin') {
        await loadUsers();
      }
    }

    async function bootstrapAdmin() {
      const username = document.getElementById('authBootstrapUsername').value.trim();
      const password = document.getElementById('authBootstrapPassword').value;
      if (!username || password.length < 8) {
        throw new Error('Bitte Benutzername und ein Passwort mit mindestens 8 Zeichen angeben.');
      }
      const res = await fetchWithTimeout('/api/bootstrap', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username, password })
      }, 5000);
      const data = await res.json().catch(() => ({}));
      if (!res.ok) {
        throw new Error(data.error || ('HTTP ' + res.status));
      }
      await loginWithCredentials(username, password);
    }

    async function logout() {
      try {
        await fetchWithTimeout('/api/logout', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }, 2500);
      } catch (err) {}
      authState = { ...emptyAuthState };
      showAuthOverlay('login', 'Bitte erneut anmelden.');
      updateSessionPanel();
      syncAuthBadge();
    }

    async function loadUsers() {
      const container = document.getElementById('userList');
      const status = document.getElementById('userAdminStatus');
      if (!container || !status) return;
      if (authState.role !== 'admin') {
        container.innerHTML = '';
        status.textContent = 'Nur Administratoren können Benutzer verwalten.';
        return;
      }
      try {
        const res = await fetchWithTimeout('/api/users', {}, 2500);
        const data = await res.json().catch(() => ({}));
        if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
        renderUserRows(data.users || []);
        status.textContent = '';
      } catch (err) {
        status.textContent = 'Benutzerliste konnte nicht geladen werden: ' + err.message;
      }
    }

    async function saveUser() {
      const status = document.getElementById('userAdminStatus');
      const payload = {
        username: document.getElementById('userFormUsername').value.trim(),
        password: document.getElementById('userFormPassword').value,
        role: document.getElementById('userFormRole').value,
        enabled: document.getElementById('userFormEnabled').checked
      };
      if (!payload.username) {
        status.textContent = 'Bitte einen Benutzernamen angeben.';
        return;
      }
      if (payload.password.length === 0) {
        delete payload.password;
      }
      try {
        const res = await fetchWithTimeout('/api/users', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload)
        }, 5000);
        const data = await res.json().catch(() => ({}));
        if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
        document.getElementById('userFormPassword').value = '';
        status.textContent = 'Benutzer gespeichert.';
        await loadUsers();
      } catch (err) {
        status.textContent = 'Benutzer konnte nicht gespeichert werden: ' + err.message;
      }
    }

    async function deleteUser(username) {
      const status = document.getElementById('userAdminStatus');
      try {
        const res = await fetchWithTimeout('/api/users', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: 'delete', username })
        }, 5000);
        const data = await res.json().catch(() => ({}));
        if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
        status.textContent = 'Benutzer gelöscht.';
        await loadUsers();
      } catch (err) {
        status.textContent = 'Benutzer konnte nicht gelöscht werden: ' + err.message;
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
      if (document.getElementById('mapView').classList.contains('hidden')) return;
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
      if (refreshBusy || !pageActive || !authState.loggedIn) return;
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
        if (data.session) {
          authState.expiresInMs = Number(data.session.expires_in_ms || authState.expiresInMs || 0);
          syncAuthBadge();
          updateSessionPanel();
        }
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
        fanOnState = Boolean(data.fan_on);
        fanSwitchable = Boolean(data.fan_switchable);
        updateFanButton();
        updateValveButtons(data.pneumatic_valves || []);
        updateCamera(data);
        if (document.activeElement !== document.getElementById('sensitivityInput')) {
          document.getElementById('sensitivityInput').value = data.quality_signal_hold_ms || data.main_signal_hold_ms || 1500;
        }
        const liftSettings = data.lift || {};
        document.getElementById('liftStatusSettings').textContent =
          liftSettings.is_down ? 'Unten' : (liftSettings.is_up ? 'Oben' : '-');
        document.getElementById('liftAutoStopCurrent').textContent =
          liftSettings.auto_stop_delay_ms > 0
            ? Math.round(liftSettings.auto_stop_delay_ms / 1000) + ' s'
            : 'Deaktiviert';
        if (document.activeElement !== document.getElementById('liftAutoStopInput')) {
          document.getElementById('liftAutoStopInput').value = liftSettings.auto_stop_delay_ms ?? 600000;
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
        const recActive = Boolean(data.recording_active);
        if (recordingStateKnown && lastRecordingActive && !recActive && portalAutoUpload && data.trip_id && data.trip_id !== lastAutoUploadTripId) {
          lastAutoUploadTripId = data.trip_id;
          uploadTripViaBrowser(data.trip_id, document.getElementById('uploadConfigStatus')).catch(err => {
            showLiftConfirmModal(data.trip_id, 'auto_failed', err.message);
          });
        }
        lastRecordingActive = recActive;
        recordingStateKnown = true;
        syncLeaveGuard(recActive);
        const statusEl = document.getElementById('gpsStatus');
        statusEl.textContent = recActive ? 'Aktiv ●' : 'Gestoppt';
        const labelEl = document.getElementById('recStatusLabel');
        if (labelEl) labelEl.className = 'recording-label' + (recActive ? ' active' : '');
        document.getElementById('gpsStart').disabled = recActive;
        document.getElementById('gpsStop').disabled = !recActive;
        if (data.lift_confirm_pending && !liftConfirmShown) {
          liftConfirmShown = true;
          showLiftConfirmModal(data.lift_confirm_trip_id || '');
        } else if (!data.lift_confirm_pending) {
          liftConfirmShown = false;
        }
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
        const lift = data.lift || {};
        const liftText = lift.is_down ? 'Unten' : (lift.is_up ? 'Oben' : '-');
        const liftTimer = lift.auto_stop_timer_active && lift.auto_stop_remaining_ms > 0
          ? ` · Stop in ${Math.ceil(lift.auto_stop_remaining_ms / 1000)} s` : '';
        const liftPct = lift.is_up ? '100%' : (lift.is_down ? '0%' : '-');
        const liftDisplay = liftText + (liftPct !== '-' ? ' · ' + liftPct : '') + liftTimer;
        ['liftStatus', 'liftStatusConn'].forEach(id => {
          const el = document.getElementById(id);
          if (el) el.textContent = liftDisplay;
        });
        document.getElementById('lastContactMini').textContent = '0 s';
        checkMainSignalAlarms(data.channels || []);
        updateChannelGrid(visibleChannels(data.channels || []), rotation);
        if (document.body.classList.contains('settings-active')) {
          try { renderSettings(data); } catch (e) {}
          document.getElementById('modules').innerHTML = (data.modules || []).map(module =>
            `<div class="module ${module.online ? 'online' : ''}"><strong>${escapeHtml(module.id)}</strong><br>${module.online ? 'Lokal online' : 'Vorbereitet'}</div>`
          ).join('');
          document.getElementById('sensorDetailsGrid').innerHTML = sensorDetails(data.channels || []);
        }
        document.getElementById('error').textContent = '';
        hideDebugOverlay();
      } catch (err) {
        if (err && err.message && /HTTP 401|HTTP 403/.test(err.message)) {
          await logout();
          return;
        }
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
      if (actionBusy) return false;
      actionBusy = true;
      try {
        const res = await fetchWithTimeout('/api/recording', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ active })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
        return true;
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Schalten fehlgeschlagen';
        return false;
      } finally {
        actionBusy = false;
      }
    }

    function startGps() {
      setRecording(true);
    }

    async function stopGps() {
      const stopped = await setRecording(false);
      if (stopped) {
        const tripId = (window.lastStatusData || {}).trip_id || '';
        showLiftConfirmModal(tripId, 'manual');
      }
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

    async function saveLiftAutoStopSetting() {
      const value = Number(document.getElementById('liftAutoStopInput').value);
      try {
        const res = await fetchWithTimeout('/api/lift-autostop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ auto_stop_delay_ms: value })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('error').textContent = 'Hubwerk-Einstellung speichern fehlgeschlagen';
      }
    }

    function playConfirmBeep() {
      try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const beep = (freq, start, dur) => {
          const osc = ctx.createOscillator();
          const gain = ctx.createGain();
          osc.connect(gain);
          gain.connect(ctx.destination);
          osc.frequency.value = freq;
          osc.type = 'sine';
          gain.gain.setValueAtTime(0.4, ctx.currentTime + start);
          gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + start + dur);
          osc.start(ctx.currentTime + start);
          osc.stop(ctx.currentTime + start + dur);
        };
        beep(880, 0.0, 0.18);
        beep(1100, 0.2, 0.18);
        beep(880, 0.4, 0.28);
      } catch (e) {}
    }

    function showLiftConfirmModal(tid, reason = 'automatic', detail = '') {
      document.getElementById('liftConfirmTripId').textContent = tid || '–';
      const title = document.getElementById('liftConfirmTitle');
      const message = document.getElementById('liftConfirmMessage');
      const status = document.getElementById('liftConfirmStatus');
      title.textContent = reason === 'manual' ? 'Aufzeichnung manuell beendet' : 'Aufzeichnung automatisch beendet';
      if (reason === 'auto_failed') {
        message.textContent = 'Keine Internetverbindung zum Portal. Die Fahrt steht zum Download bereit.';
        status.textContent = detail ? 'Upload fehlgeschlagen: ' + detail : '';
      } else if (reason === 'manual') {
        message.textContent = 'Die Fahrt wurde gespeichert und steht zum Download oder Upload bereit.';
        status.textContent = '';
      } else {
        message.textContent = 'Die Fahrt wurde automatisch beendet und steht zum Download oder Upload bereit.';
        status.textContent = '';
      }
      document.getElementById('liftConfirmDownload').classList.remove('hidden');
      document.getElementById('liftConfirmModal').classList.remove('hidden');
      playConfirmBeep();
    }

    async function dismissLiftConfirm() {
      document.getElementById('liftConfirmModal').classList.add('hidden');
      try {
        await fetchWithTimeout('/api/lift-confirm', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
      } catch (e) {}
    }

    async function liftConfirmUpload() {
      document.getElementById('liftConfirmStatus').textContent = 'Upload wird gestartet …';
      document.getElementById('liftConfirmUploadBtn').disabled = true;
      try {
        const tripId = (window.lastStatusData || {}).lift_confirm_trip_id || (window.lastStatusData || {}).trip_id || '';
        await uploadTripViaBrowser(tripId, document.getElementById('liftConfirmStatus'));
        setTimeout(() => dismissLiftConfirm(), 2500);
      } catch (err) {
        document.getElementById('liftConfirmStatus').textContent = 'Upload fehlgeschlagen: ' + err.message + ' – Daten herunterladen:';
        document.getElementById('liftConfirmDownload').classList.remove('hidden');
      }
      document.getElementById('liftConfirmUploadBtn').disabled = false;
      try {
        await fetchWithTimeout('/api/lift-confirm', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
      } catch (e) {}
    }

    async function loadDeviceConfig() {
      try {
        const res = await fetchWithTimeout('/api/device-config');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        if (document.activeElement !== document.getElementById('deviceNameInput')) {
          document.getElementById('deviceNameInput').value = data.device_name || '';
        }
      } catch (err) {
        document.getElementById('deviceNameStatus').textContent = 'Konfiguration konnte nicht geladen werden';
      }
    }

    async function saveDeviceNameSetting() {
      const name = document.getElementById('deviceNameInput').value.trim();
      if (!name) return;
      try {
        const res = await fetchWithTimeout('/api/device-config', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ device_name: name })
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        document.getElementById('deviceNameStatus').textContent = 'Gespeichert.';
        await refresh();
      } catch (err) {
        document.getElementById('deviceNameStatus').textContent = 'Speichern fehlgeschlagen';
      }
    }

    async function loadUploadConfig() {
      try {
        const res = await fetchWithTimeout('/api/upload-config');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        portalUploadUrl = data.upload_url || '';
        portalUploadToken = data.upload_token || '';
        portalAutoUpload = Boolean(data.auto_upload);
        if (document.activeElement !== document.getElementById('uploadUrlInput')) {
          document.getElementById('uploadUrlInput').value = data.upload_url || '';
        }
        document.getElementById('autoUploadInput').checked = Boolean(data.auto_upload);
      } catch (err) {
        document.getElementById('uploadConfigStatus').textContent = 'Konfiguration konnte nicht geladen werden';
      }
    }

    async function saveUploadConfigSetting() {
      const url = document.getElementById('uploadUrlInput').value.trim();
      const token = document.getElementById('uploadTokenInput').value;
      const autoUpload = document.getElementById('autoUploadInput').checked;
      const body = { upload_url: url, auto_upload: autoUpload };
      if (token.length > 0) body.upload_token = token;
      try {
        const res = await fetchWithTimeout('/api/upload-config', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body)
        });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        document.getElementById('uploadTokenInput').value = '';
        portalUploadUrl = url;
        if (token.length > 0) portalUploadToken = token;
        portalAutoUpload = autoUpload;
        document.getElementById('uploadConfigStatus').textContent = 'Gespeichert.';
      } catch (err) {
        document.getElementById('uploadConfigStatus').textContent = 'Speichern fehlgeschlagen';
      }
    }

    async function testUploadConfig() {
      const target = document.getElementById('uploadConfigStatus');
      const button = document.getElementById('uploadConfigTest');
      const configuredUrl = document.getElementById('uploadUrlInput').value.trim() || portalUploadUrl;
      const enteredToken = document.getElementById('uploadTokenInput').value;
      const token = enteredToken || portalUploadToken;
      if (!configuredUrl) {
        target.textContent = 'Test fehlgeschlagen: Portal-URL fehlt.';
        return;
      }
      if (!token) {
        target.textContent = 'Test fehlgeschlagen: API-Token fehlt.';
        return;
      }
      let testUrl;
      try {
        const url = new URL(configuredUrl);
        if (!/\/api\/trips\/upload\/?$/.test(url.pathname)) throw new Error('URL muss mit /api/trips/upload enden');
        url.pathname = url.pathname.replace(/\/api\/trips\/upload\/?$/, '/api/v1/trips');
        url.search = '?limit=1';
        testUrl = url.toString();
      } catch (err) {
        target.textContent = 'Test fehlgeschlagen: ' + err.message;
        return;
      }
      button.disabled = true;
      target.textContent = 'Portal und API-Token werden geprüft ...';
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), 10000);
      try {
        const response = await fetch(testUrl, {
          headers: { 'Authorization': 'Bearer ' + token },
          signal: controller.signal
        });
        const result = await response.json().catch(() => ({}));
        if (response.status === 401 || response.status === 403) throw new Error('API-Token ungültig oder abgelaufen');
        if (!response.ok) throw new Error(result.error || `Portal HTTP ${response.status}`);
        target.textContent = 'Verbindung erfolgreich: Portal erreichbar, API-Token gültig.';
      } catch (err) {
        const message = err.name === 'AbortError'
          ? 'Zeitüberschreitung – keine Internetverbindung oder Portal nicht erreichbar'
          : (err.message || 'Keine Internetverbindung oder Portal nicht erreichbar');
        target.textContent = 'Test fehlgeschlagen: ' + message;
      } finally {
        clearTimeout(timer);
        button.disabled = false;
      }
    }

    async function fetchTripFile(tripId, suffix) {
      const path = `/trip-${tripId}-${suffix}`;
      const res = await fetch('/api/archive/download?path=' + encodeURIComponent(path));
      if (!res.ok) throw new Error(`${suffix} nicht vom Gerät abrufbar`);
      return await res.blob();
    }

    async function uploadTripViaBrowser(tripId, statusTarget) {
      if (browserUploadBusy) throw new Error('Ein Upload läuft bereits');
      if (!tripId || tripId === '-') throw new Error('Keine abgeschlossene Fahrt vorhanden');
      if (!portalUploadUrl) throw new Error('Portal-URL fehlt');
      if (!portalUploadToken) throw new Error('API-Token fehlt');
      browserUploadBusy = true;
      if (statusTarget) statusTarget.textContent = `Tablet lädt Fahrt ${tripId} vom Gerät ...`;
      try {
        const [metadata, gpsCsv, sensorCsv, mainEventsRes] = await Promise.all([
          fetchTripFile(tripId, 'meta.txt'),
          fetchTripFile(tripId, 'gps.csv'),
          fetchTripFile(tripId, 'sensor.csv'),
          fetch('/api/main-events.csv')
        ]);
        if (!mainEventsRes.ok) throw new Error('Hauptsignal-Datei nicht abrufbar');
        const mainCsv = await mainEventsRes.blob();
        const status = window.lastStatusData || {};
        const form = new FormData();
        form.append('trip_id', tripId);
        form.append('device_id', status.device_name || status.device_id || 'ESP32');
        form.append('metadata', metadata, 'meta.json');
        form.append('gps_csv', gpsCsv, 'gps.csv');
        form.append('main_events_csv', mainCsv, 'main-events.csv');
        form.append('sensor_events_csv', sensorCsv, 'sensor-events.csv');
        if (statusTarget) statusTarget.textContent = 'Mobilgerät übermittelt die Fahrt an das Portal ...';
        const portalResponse = await fetch(portalUploadUrl, {
          method: 'POST',
          headers: { 'Authorization': 'Bearer ' + portalUploadToken },
          body: form
        });
        const result = await portalResponse.json().catch(() => ({}));
        if (!portalResponse.ok) throw new Error(result.error || `Portal HTTP ${portalResponse.status}`);
        if (statusTarget) statusTarget.textContent = `Fahrt ${tripId} erfolgreich übermittelt.`;
        return result;
      } finally {
        browserUploadBusy = false;
      }
    }

    async function triggerUploadNow() {
      const target = document.getElementById('uploadConfigStatus');
      try {
        await uploadTripViaBrowser((window.lastStatusData || {}).trip_id || '', target);
      } catch (err) {
        target.textContent = 'Upload fehlgeschlagen: ' + err.message;
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
        const suggestions = Array.isArray(list) ? list : [];
        const cropInput = document.getElementById('cropInput');
        const currentCrop = (window.lastStatusData || {}).crop_name || cropInput.value;
        document.getElementById('cropSuggestions').innerHTML = suggestions.map(v =>
          `<option value="${escapeHtml(v)}"></option>`
        ).join('');
        cropInput.value = currentCrop || '';
        const quick = document.getElementById('cropQuickList');
        const current = cropInput.value;
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
    document.getElementById('liftAutoStopSave').addEventListener('click', saveLiftAutoStopSetting);
    document.getElementById('deviceNameSave').addEventListener('click', saveDeviceNameSetting);
    document.getElementById('uploadConfigSave').addEventListener('click', saveUploadConfigSetting);
    document.getElementById('uploadConfigTest').addEventListener('click', testUploadConfig);
    document.getElementById('uploadNowBtn').addEventListener('click', triggerUploadNow);
    document.getElementById('liftConfirmUploadBtn').addEventListener('click', liftConfirmUpload);
    document.getElementById('liftConfirmCloseBtn').addEventListener('click', dismissLiftConfirm);
    document.getElementById('archiveRefresh').addEventListener('click', loadArchive);
    document.getElementById('authLoginBtn').addEventListener('click', async () => {
      const message = document.getElementById('authMessage');
      message.textContent = '';
      try {
        await loginWithCredentials(
          document.getElementById('authUsername').value.trim(),
          document.getElementById('authPassword').value
        );
      } catch (err) {
        message.textContent = err.message || 'Anmeldung fehlgeschlagen';
      }
    });
    document.getElementById('authBootstrapBtn').addEventListener('click', async () => {
      const message = document.getElementById('authMessage');
      message.textContent = '';
      try {
        await bootstrapAdmin();
      } catch (err) {
        message.textContent = err.message || 'Erst-Admin konnte nicht angelegt werden';
      }
    });
    document.getElementById('userSaveBtn').addEventListener('click', saveUser);
    document.getElementById('userReloadBtn').addEventListener('click', loadUsers);
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
    window.addEventListener('beforeunload', event => {
      if (isRecordingActive()) {
        event.preventDefault();
        event.returnValue = '';
      }
    });
    window.addEventListener('popstate', () => {
      if (!isRecordingActive()) return;
      document.getElementById('leaveConfirmModal').classList.remove('hidden');
      history.pushState({ leaveGuard: true }, '', location.href);
    });
    document.getElementById('leaveConfirmStayBtn').addEventListener('click', () => {
      document.getElementById('leaveConfirmModal').classList.add('hidden');
    });
    document.getElementById('leaveConfirmLeaveBtn').addEventListener('click', () => {
      document.getElementById('leaveConfirmModal').classList.add('hidden');
      leaveGuardArmed = false;
      history.go(-2);
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

    async function bootApp() {
      syncAuthBadge();
      updateSessionPanel();
      const sessionOk = await loadSession();
      if (!sessionOk) {
        return;
      }
      bootCompleted = true;
      await Promise.allSettled([
        refresh(),
        loadCropSuggestions(),
        refreshTrack(),
        loadArchive(),
        loadDeviceConfig()
      ]);
      if (authState.role === 'admin') {
        await loadUploadConfig();
      }
      window.addEventListener('resize', () => drawTrack());
      connectionTimer = setInterval(updateLastContact, 1000);
      setInterval(refresh, 1000);
      setInterval(refreshTrack, 3000);
      if (authState.role === 'admin') {
        await loadUsers();
      }
    }

    bootApp();
  </script>
</body>
</html>
)HTML";
  return PAGE;
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", htmlPage());
}

void handleApiSession() {
  if (!authorize(UserRole::Viewer)) {
    return;
  }
  JsonDocument doc;
  doc["ok"] = true;
  doc["username"] = activeSession.username;
  doc["role"] = roleToString(activeSession.role);
  doc["expires_in_ms"] = activeSession.expiresAtMs > millis() ? static_cast<int32_t>(activeSession.expiresAtMs - millis()) : 0;
  String json;
  serializeJson(doc, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleApiLogin() {
  if (userCount == 0) {
    server.send(409, "application/json", "{\"error\":\"bootstrap_required\"}");
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  const String username = doc["username"] | "";
  const String password = doc["password"] | "";
  if (username.length() == 0 || password.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"missing_credentials\"}");
    return;
  }

  UserAccount *user = findUser(username);
  if (!user || !user->enabled || !userPasswordMatches(*user, password)) {
    server.send(401, "application/json", "{\"error\":\"invalid_credentials\"}");
    return;
  }

  SessionState *session = createSession(*user);
  if (!session) {
    server.send(500, "application/json", "{\"error\":\"session_allocation_failed\"}");
    return;
  }

  sendSessionCookie(*session);
  appendSystemEvent("auth,login," + String(user->username) + "," + roleToString(user->role));
  JsonDocument response;
  response["ok"] = true;
  response["username"] = user->username;
  response["role"] = roleToString(user->role);
  response["expires_in_ms"] = SESSION_TTL_MS;
  String json;
  serializeJson(response, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleApiBootstrap() {
  if (userCount > 0) {
    server.send(409, "application/json", "{\"error\":\"users_already_configured\"}");
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  const String username = doc["username"] | "";
  const String password = doc["password"] | "";
  if (username.length() == 0 || password.length() < 8) {
    server.send(400, "application/json", "{\"error\":\"invalid_bootstrap_data\"}");
    return;
  }

  if (!createOrUpdateUser(username, password, UserRole::Admin, true)) {
    server.send(500, "application/json", "{\"error\":\"bootstrap_failed\"}");
    return;
  }

  appendSystemEvent("auth,bootstrap," + username);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiLogout() {
  if (!authorize(UserRole::Viewer)) {
    return;
  }

  String token = getCookieValue(getRequestHeader("Cookie"), "pm_session");
  SessionState *session = findSessionByToken(token);
  if (session) {
    appendSystemEvent("auth,logout," + String(session->username));
    expireSession(*session);
  }
  server.sendHeader("Set-Cookie", "pm_session=; Path=/; Max-Age=0; SameSite=Strict");
  server.sendHeader("Set-Cookie", "pm_binding=; Path=/; Max-Age=0; SameSite=Strict");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiUsers() {
  if (!authorize(UserRole::Admin)) {
    return;
  }

  if (server.method() == HTTP_GET) {
    JsonDocument doc;
    JsonArray arr = doc["users"].to<JsonArray>();
    for (uint8_t i = 0; i < userCount; i++) {
      if (users[i].username[0] == '\0') {
        continue;
      }
      JsonObject item = arr.add<JsonObject>();
      item["username"] = users[i].username;
      item["role"] = roleToString(users[i].role);
      item["enabled"] = users[i].enabled;
    }
    String json;
    serializeJson(doc, json);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }
  const String action = doc["action"] | "upsert";
  const String username = doc["username"] | "";
  const String password = doc["password"] | "";
  const bool enabled = doc["enabled"] | true;
  const UserRole role = roleFromString(doc["role"] | "viewer");

  if (action == "delete") {
    if (!deleteUserAccount(username)) {
      server.send(404, "application/json", "{\"error\":\"user_not_found\"}");
      return;
    }
    appendSystemEvent("auth,user_delete," + username);
    server.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  if (!createOrUpdateUser(username, password, role, enabled)) {
    server.send(400, "application/json", "{\"error\":\"user_save_failed\"}");
    return;
  }
  appendSystemEvent("auth,user_save," + username + "," + roleToString(role));
  server.send(200, "application/json", "{\"ok\":true}");
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
  response["red_signal_hold_ms"] = mainSignalHoldMs;
  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
}

void handleApiLiftAutoStop() {
  const String body = server.arg("plain");
  JsonDocument doc;
  if (body.length() == 0 || deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"invalid_body\"}");
    return;
  }
  const uint32_t delayMs = doc["auto_stop_delay_ms"] | DEFAULT_LIFT_AUTO_STOP_DELAY_MS;
  saveLiftAutoStopDelay(delayMs);
  JsonDocument response;
  response["ok"] = true;
  response["auto_stop_delay_ms"] = liftAutoStopDelayMs;
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

void handleApiValve() {
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

  const int index = doc["index"] | -1;
  if (index < 0 || index >= PNEUMATIC_VALVE_COUNT) {
    server.send(400, "application/json", "{\"error\":\"invalid_valve\"}");
    return;
  }

  if (activePneumaticValveIndex < PNEUMATIC_VALVE_COUNT) {
    server.send(409, "application/json", "{\"error\":\"valve_busy\"}");
    return;
  }

  const bool ok = startPneumaticValve(static_cast<uint8_t>(index));
  JsonDocument resp;
  resp["ok"] = ok;
  resp["index"] = index;
  resp["output_channel"] = PNEUMATIC_VALVE_OUTPUTS[index];
  resp["pulse_ms"] = PNEUMATIC_VALVE_PULSE_MS;
  resp["active_index"] = activePneumaticValveIndex < PNEUMATIC_VALVE_COUNT ? activePneumaticValveIndex : -1;
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

void handleApiDeviceConfig() {
  if (server.method() == HTTP_GET) {
    JsonDocument doc;
    doc["device_name"] = deviceName;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
    return;
  }
  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }
  if (doc["device_name"].is<const char *>()) {
    String n = doc["device_name"].as<String>();
    n.trim();
    if (n.length() > 0) {
      n.toCharArray(deviceName, DEVICE_NAME_LENGTH);
      saveDeviceConfig();
    }
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiLiftConfirm() {
  liftAutoStopPendingConfirm = false;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiUploadConfig() {
  if (server.method() == HTTP_GET) {
    JsonDocument doc;
    doc["upload_url"]  = uploadUrl;
    doc["upload_token"] = uploadToken;
    doc["auto_upload"] = autoUpload;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
    return;
  }
  // POST
  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }
  if (doc["upload_url"].is<const char *>()) {
    String u = doc["upload_url"].as<String>();
    u.toCharArray(uploadUrl, UPLOAD_URL_LENGTH);
  }
  if (doc["upload_token"].is<const char *>()) {
    String t = doc["upload_token"].as<String>();
    t.toCharArray(uploadToken, UPLOAD_TOKEN_LENGTH);
  }
  if (doc["auto_upload"].is<bool>()) {
    autoUpload = doc["auto_upload"].as<bool>();
  }
  saveUploadConfig();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiUploadNow() {
  const String tid = server.arg("trip_id");
  const char *targetId = tid.length() > 0 ? tid.c_str() : tripId;
  if (recordingActive) {
    server.send(409, "application/json", "{\"error\":\"recording_active\"}");
    return;
  }
  if (uploadUrl[0] == '\0') {
    server.send(400, "application/json", "{\"error\":\"no_upload_url\"}");
    return;
  }
  server.send(202, "application/json", "{\"ok\":true,\"trip_id\":\"" + String(targetId) + "\"}");
  strncpy(pendingUploadTripId, targetId, TRIP_ID_LENGTH - 1);
  pendingUploadTripId[TRIP_ID_LENGTH - 1] = '\0';
  pendingUpload = true;
  uploadRetryCount = 0;
  uploadRetryNextMs = millis() + 500;
}

void handleApiGpsLogClear() {
  if (recordingActive) {
    server.send(409, "application/json", "{\"error\":\"recording_active\"}");
    return;
  }
  clearGpsLog();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------------------------------------------------------------------
// Portal-Upload
// ---------------------------------------------------------------------------

String buildMainEventsCsvString() {
  String csv;
  csv.reserve(96 + mainEventCount * 100);
  csv += "index,uptime_ms,channel,detected,crop_name,latitude,longitude,accuracy_m,live_mask,main_mask\n";
  for (uint16_t i = 0; i < mainEventCount; i++) {
    const MainSignalEvent &ev = mainEventAt(i);
    csv += i; csv += ",";
    csv += ev.uptimeMs; csv += ",";
    csv += ev.channel; csv += ",";
    csv += ev.detected ? "1" : "0"; csv += ",";
    csv += ev.crop; csv += ",";
    if (ev.hasGps) {
      csv += String(ev.latitude, 7); csv += ",";
      csv += String(ev.longitude, 7); csv += ",";
      csv += String(ev.accuracyM, 1);
    } else {
      csv += ",,";
    }
    csv += ","; csv += (unsigned)ev.liveMask;
    csv += ","; csv += (unsigned)ev.mainMask;
    csv += "\n";
  }
  return csv;
}

struct ParsedUploadUrl {
  bool isSSL;
  String host;
  uint16_t port;
  String path;
};

bool parseUploadUrl(const char *url, ParsedUploadUrl &out) {
  String u(url);
  if (u.startsWith("https://")) {
    out.isSSL = true;
    u = u.substring(8);
    out.port = 443;
  } else if (u.startsWith("http://")) {
    out.isSSL = false;
    u = u.substring(7);
    out.port = 80;
  } else {
    return false;
  }
  int slash = u.indexOf('/');
  if (slash < 0) {
    out.path = "/";
    out.host = u;
  } else {
    out.path = u.substring(slash);
    out.host = u.substring(0, slash);
  }
  int colon = out.host.indexOf(':');
  if (colon >= 0) {
    out.port = (uint16_t)out.host.substring(colon + 1).toInt();
    out.host = out.host.substring(0, colon);
  }
  return out.host.length() > 0;
}

static const char *UPLOAD_BOUNDARY = "----FormBoundaryESP32Upload";

String mpTextHeader(const char *name) {
  String h = "--";
  h += UPLOAD_BOUNDARY;
  h += "\r\nContent-Disposition: form-data; name=\"";
  h += name;
  h += "\"\r\n\r\n";
  return h;
}

String mpFileHeader(const char *name, const char *filename, const char *contentType) {
  String h = "--";
  h += UPLOAD_BOUNDARY;
  h += "\r\nContent-Disposition: form-data; name=\"";
  h += name;
  h += "\"; filename=\"";
  h += filename;
  h += "\"\r\nContent-Type: ";
  h += contentType;
  h += "\r\n\r\n";
  return h;
}

bool uploadTripToServer(const char *targetTripId) {
  if (uploadUrl[0] == '\0') {
    appendSystemEvent("upload_skip,no_url," + String(targetTripId));
    return false;
  }

  ParsedUploadUrl parsed;
  if (!parseUploadUrl(uploadUrl, parsed)) {
    appendSystemEvent("upload_fail,bad_url," + String(targetTripId));
    return false;
  }

  char gpsPath[64], sensorPath[64], metaPath[64];
  snprintf(gpsPath, sizeof(gpsPath), "/trip-%s-gps.csv", targetTripId);
  snprintf(sensorPath, sizeof(sensorPath), "/trip-%s-sensor.csv", targetTripId);
  snprintf(metaPath, sizeof(metaPath), "/trip-%s-meta.txt", targetTripId);

  // Read metadata JSON from LittleFS
  String metaJson;
  if (filesystemReady && LittleFS.exists(metaPath)) {
    File mf = LittleFS.open(metaPath, FILE_READ);
    if (mf) { metaJson = mf.readString(); mf.close(); }
  }
  if (metaJson.length() == 0) {
    metaJson = "{\"trip_id\":\"" + String(targetTripId) + "\",\"device_id\":\"" + String(deviceName) + "\"}";
  }

  String mainCsv = buildMainEventsCsvString();

  bool hasGps    = filesystemReady && LittleFS.exists(gpsPath);
  bool hasSensor = filesystemReady && LittleFS.exists(sensorPath);

  size_t gpsFileSize    = 0;
  size_t sensorFileSize = 0;
  if (hasGps)    { File f = LittleFS.open(gpsPath, FILE_READ);    if (f) { gpsFileSize    = f.size(); f.close(); } }
  if (hasSensor) { File f = LittleFS.open(sensorPath, FILE_READ); if (f) { sensorFileSize = f.size(); f.close(); } }

  // Pre-compute Content-Length
  String hTripId   = mpTextHeader("trip_id");
  String hDeviceId = mpTextHeader("device_id");
  String hMeta     = mpFileHeader("metadata", "meta.json", "application/json");
  String hMainCsv  = mpFileHeader("main_events_csv", "main-events.csv", "text/csv");
  String hGpsCsv   = mpFileHeader("gps_csv", "gps.csv", "text/csv");
  String hSensor   = mpFileHeader("sensor_events_csv", "sensor-events.csv", "text/csv");
  String finalBnd  = String("--") + UPLOAD_BOUNDARY + "--\r\n";

  size_t contentLength = 0;
  contentLength += hTripId.length()   + strlen(targetTripId) + 2;
  contentLength += hDeviceId.length() + strlen(deviceName)    + 2;
  contentLength += hMeta.length()     + metaJson.length()    + 2;
  if (mainCsv.length() > 0) contentLength += hMainCsv.length() + mainCsv.length() + 2;
  if (hasGps)               contentLength += hGpsCsv.length()  + gpsFileSize      + 2;
  if (hasSensor)            contentLength += hSensor.length()   + sensorFileSize   + 2;
  contentLength += finalBnd.length();

  // Connect
  WiFiClientSecure wifiSecure;
  WiFiClient wifiPlain;
  EthernetClient ethClient;
  Client *client = nullptr;

  if (parsed.isSSL) {
    wifiSecure.setInsecure();
    if (!wifiSecure.connect(parsed.host.c_str(), parsed.port)) {
      appendSystemEvent("upload_fail,ssl_connect," + String(targetTripId));
      return false;
    }
    client = &wifiSecure;
  } else if (ethernetReady && Ethernet.linkStatus() == LinkON) {
    if (!ethClient.connect(parsed.host.c_str(), parsed.port)) {
      appendSystemEvent("upload_fail,eth_connect," + String(targetTripId));
      return false;
    }
    client = &ethClient;
  } else {
    if (!wifiPlain.connect(parsed.host.c_str(), parsed.port)) {
      appendSystemEvent("upload_fail,wifi_connect," + String(targetTripId));
      return false;
    }
    client = &wifiPlain;
  }

  client->setTimeout(30000);

  // HTTP request headers
  client->print("POST "); client->print(parsed.path); client->print(" HTTP/1.1\r\n");
  client->print("Host: "); client->print(parsed.host); client->print("\r\n");
  if (uploadToken[0] != '\0') {
    client->print("Authorization: Bearer "); client->print(uploadToken); client->print("\r\n");
  }
  client->print("Content-Type: multipart/form-data; boundary=");
  client->print(UPLOAD_BOUNDARY); client->print("\r\n");
  client->print("Content-Length: "); client->print((uint32_t)contentLength); client->print("\r\n");
  client->print("Connection: close\r\n\r\n");

  // Multipart body – text parts
  client->print(hTripId);   client->print(targetTripId); client->print("\r\n");
  client->print(hDeviceId); client->print(deviceName);    client->print("\r\n");
  client->print(hMeta);     client->print(metaJson);     client->print("\r\n");
  if (mainCsv.length() > 0) {
    client->print(hMainCsv); client->print(mainCsv); client->print("\r\n");
  }

  // GPS CSV – streamed from LittleFS
  if (hasGps) {
    client->print(hGpsCsv);
    File f = LittleFS.open(gpsPath, FILE_READ);
    if (f) {
      uint8_t buf[512];
      while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        if (n > 0) client->write(buf, (size_t)n);
        esp_task_wdt_reset();
      }
      f.close();
    }
    client->print("\r\n");
  }

  // Sensor CSV – streamed from LittleFS
  if (hasSensor) {
    client->print(hSensor);
    File f = LittleFS.open(sensorPath, FILE_READ);
    if (f) {
      uint8_t buf[512];
      while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        if (n > 0) client->write(buf, (size_t)n);
        esp_task_wdt_reset();
      }
      f.close();
    }
    client->print("\r\n");
  }

  client->print(finalBnd);

  // Read response status line
  uint32_t deadline = millis() + 15000;
  while (!client->available() && millis() < deadline) {
    delay(10);
    esp_task_wdt_reset();
  }
  String statusLine = client->readStringUntil('\n');
  client->stop();

  int httpStatus = 0;
  if (statusLine.startsWith("HTTP/")) {
    httpStatus = statusLine.substring(9, 12).toInt();
  }

  if (httpStatus == 200 || httpStatus == 201) {
    appendSystemEvent("upload_ok," + String(httpStatus) + "," + String(targetTripId));
    Serial.printf("Upload OK (%d): %s\n", httpStatus, targetTripId);
    return true;
  }
  appendSystemEvent("upload_fail," + String(httpStatus) + "," + String(targetTripId));
  Serial.printf("Upload FEHLER (%d): %s\n", httpStatus, targetTripId);
  return false;
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
  loadLiftAutoStopDelay();
  loadUploadConfig();
  loadDeviceConfig();
  loadUsersFromPreferences();
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

  const char *headerKeys[] = {"Cookie", "User-Agent", "X-Device-Binding", "X-Session-Token"};
  server.collectHeaders(headerKeys, sizeof(headerKeys) / sizeof(headerKeys[0]));

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/session", HTTP_GET, handleApiSession);
  server.on("/api/login", HTTP_POST, handleApiLogin);
  server.on("/api/bootstrap", HTTP_POST, handleApiBootstrap);
  server.on("/api/logout", HTTP_POST, handleApiLogout);
  server.on("/api/users", HTTP_GET, handleApiUsers);
  server.on("/api/users", HTTP_POST, handleApiUsers);
  registerProtectedRoute("/camera/1/substream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(0, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  registerProtectedRoute("/camera/1/mainstream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(0, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  registerProtectedRoute("/camera/2/substream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(1, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  registerProtectedRoute("/camera/2/mainstream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(1, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  registerProtectedRoute("/camera/3/substream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(2, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  registerProtectedRoute("/camera/3/mainstream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(2, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  registerProtectedRoute("/camera/4/substream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(3, HIKVISION_CAMERA_SUB_STREAM_PATH); });
  registerProtectedRoute("/camera/4/mainstream", HTTP_GET, UserRole::Viewer, []() { handleCameraProxy(3, HIKVISION_CAMERA_MAIN_STREAM_PATH); });
  registerProtectedRoute("/api/status", HTTP_GET, UserRole::Viewer, handleApiStatus);
  registerProtectedRoute("/api/rs485-scan", HTTP_GET, UserRole::Operator, handleApiRs485Scan);
  registerProtectedRoute("/api/rs485-address-scan", HTTP_GET, UserRole::Operator, handleApiRs485AddressScan);
  registerProtectedRoute("/api/rs485-register-scan", HTTP_GET, UserRole::Operator, handleApiRs485RegisterScan);
  registerProtectedRoute("/api/camera-test", HTTP_GET, UserRole::Operator, handleApiCameraTest);
  registerProtectedRoute("/api/alarm/ack", HTTP_POST, UserRole::Operator, handleApiAlarmAck);
  registerProtectedRoute("/api/channel-name", HTTP_POST, UserRole::Operator, handleApiChannelName);
  registerProtectedRoute("/api/crop", HTTP_POST, UserRole::Operator, handleApiCrop);
  registerProtectedRoute("/api/field", HTTP_POST, UserRole::Operator, handleApiField);
  registerProtectedRoute("/api/sensitivity", HTTP_POST, UserRole::Operator, handleApiSensitivity);
  registerProtectedRoute("/api/lift-autostop", HTTP_POST, UserRole::Operator, handleApiLiftAutoStop);
  registerProtectedRoute("/api/camera-settings", HTTP_POST, UserRole::Admin, handleApiCameraSettings);
  registerProtectedRoute("/api/recording", HTTP_POST, UserRole::Operator, handleApiRecording);
  registerProtectedRoute("/api/output", HTTP_POST, UserRole::Operator, handleApiOutput);
  registerProtectedRoute("/api/valve", HTTP_POST, UserRole::Operator, handleApiValve);
  registerProtectedRoute("/api/track", HTTP_GET, UserRole::Viewer, handleApiTrack);
  registerProtectedRoute("/api/gps-log.csv", HTTP_GET, UserRole::Viewer, handleApiGpsLogCsv);
  registerProtectedRoute("/api/gps-log.geojson", HTTP_GET, UserRole::Viewer, handleApiGpsLogGeoJson);
  registerProtectedRoute("/api/main-events.csv", HTTP_GET, UserRole::Viewer, handleApiMainEventsCsv);
  registerProtectedRoute("/api/main-events.geojson", HTTP_GET, UserRole::Viewer, handleApiMainEventsGeoJson);
  registerProtectedRoute("/api/sensor-events.csv", HTTP_GET, UserRole::Viewer, handleApiSensorEventsCsv);
  registerProtectedRoute("/api/sensor-events.txt", HTTP_GET, UserRole::Viewer, handleApiSensorEventsTxt);
  registerProtectedRoute("/api/combined.geojson", HTTP_GET, UserRole::Viewer, handleApiCombinedGeoJson);
  registerProtectedRoute("/api/archive", HTTP_GET, UserRole::Viewer, handleApiArchive);
  registerProtectedRoute("/api/archive/download", HTTP_GET, UserRole::Viewer, handleApiArchiveDownload);
  registerProtectedRoute("/api/system-events.log", HTTP_GET, UserRole::Admin, handleApiSystemEvents);
  registerProtectedRoute("/api/gps-log/clear", HTTP_POST, UserRole::Admin, handleApiGpsLogClear);
  registerProtectedRoute("/api/crops", HTTP_GET, UserRole::Viewer, handleApiCrops);
  registerProtectedRoute("/api/crops", HTTP_POST, UserRole::Admin, handleApiCropsPost);
  registerProtectedRoute("/api/device-config", HTTP_GET, UserRole::Viewer, handleApiDeviceConfig);
  registerProtectedRoute("/api/device-config", HTTP_POST, UserRole::Admin, handleApiDeviceConfig);
  registerProtectedRoute("/api/lift-confirm", HTTP_POST, UserRole::Operator, handleApiLiftConfirm);
  registerProtectedRoute("/api/upload-config", HTTP_GET, UserRole::Admin, handleApiUploadConfig);
  registerProtectedRoute("/api/upload-config", HTTP_POST, UserRole::Admin, handleApiUploadConfig);
  registerProtectedRoute("/api/upload-now", HTTP_POST, UserRole::Operator, handleApiUploadNow);
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
  updatePneumaticValves();
  updateFanTemperatureControl();
  server.handleClient();
  if (recordingActive && millis() - lastFileFlushMs >= FILE_FLUSH_INTERVAL_MS) {
    flushPersistentGpsBuffer();
  }
  esp_task_wdt_reset();

  if (pendingUpload && !recordingActive && millis() >= uploadRetryNextMs) {
    pendingUpload = false;
    bool ok = uploadTripToServer(pendingUploadTripId);
    if (!ok && uploadRetryCount < UPLOAD_RETRY_MAX) {
      uploadRetryCount++;
      pendingUpload = true;
      uploadRetryNextMs = millis() + UPLOAD_RETRY_DELAY_MS;
      Serial.printf("Upload fehlgeschlagen, Versuch %u/%u in %lus\n",
                    uploadRetryCount, (uint8_t)UPLOAD_RETRY_MAX, UPLOAD_RETRY_DELAY_MS / 1000UL);
    }
  }

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
