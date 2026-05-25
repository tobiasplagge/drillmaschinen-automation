#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ESPWebServerSecure.hpp>
#include <WiFi.h>
#include <Wire.h>
#include <esp_heap_caps.h>
#include "tls_server_cert_der.h"
#include "tls_server_key_der.h"

// ---------------------------------------------------------------------------
// Waveshare ESP32-S3-POE-ETH-8DI-8DO configuration
// ---------------------------------------------------------------------------
static const char *AP_SSID = "DRILL-8DI8DO";
static const char *AP_PASSWORD = "12345678";

static const char *DEVICE_ID = "waveshare-8di8do-01";
static const char *DEVICE_NAME = "Drillmaschinenüberwachung";
static const char *FIRMWARE_VERSION = "1.2.0";

static constexpr uint8_t CHANNEL_COUNT = 8;
static constexpr uint8_t CHANNEL_NAME_LENGTH = 24;
static constexpr uint8_t CROP_NAME_LENGTH = 24;
static constexpr uint32_t MAIN_SIGNAL_HOLD_MS = 1500;
static constexpr uint32_t GPS_LOG_INTERVAL_MS = 3000;
static constexpr uint16_t GPS_LOG_TARGET_CAPACITY = 5000;
static constexpr uint16_t MAIN_EVENT_LOG_CAPACITY = 512;

// Waveshare wiki: DI1..DI8 are GPIO4..GPIO11.
static constexpr uint8_t DI_PINS[CHANNEL_COUNT] = {4, 5, 6, 7, 8, 9, 10, 11};

// Waveshare TCA9554PWR expander for DO1..DO8.
static constexpr uint8_t I2C_SDA_PIN = 42;
static constexpr uint8_t I2C_SCL_PIN = 41;
static constexpr uint8_t TCA9554_ADDRESS = 0x20;
static constexpr uint8_t TCA9554_INPUT_REG = 0x00;
static constexpr uint8_t TCA9554_OUTPUT_REG = 0x01;
static constexpr uint8_t TCA9554_CONFIG_REG = 0x03;

// On this board the digital outputs are controlled through the TCA9554. The
// Waveshare examples initialize all outputs HIGH. Keep this configurable in
// case your wiring expects the opposite logic.
static constexpr bool DO_ACTIVE_HIGH = true;
static constexpr bool INPUT_ACTIVE_HIGH = false;
static constexpr bool MIRROR_RED_TO_OUTPUT = true;

ESPWebServerSecure server(443);
Preferences preferences;

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
  uint8_t liveMask = 0;
  uint8_t mainMask = 0;
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
MainSignalEvent mainEventLog[MAIN_EVENT_LOG_CAPACITY];
uint16_t mainEventHead = 0;
uint16_t mainEventCount = 0;
uint32_t mainEventTotal = 0;

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

void appendGpsLog(double latitude, double longitude, float accuracyM, float speedMps, float headingDeg) {
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

void appendMainSignalEvent(uint8_t channelIndex, bool detected) {
  MainSignalEvent &event = mainEventLog[mainEventHead];
  event.uptimeMs = millis();
  event.channel = channelIndex + 1;
  event.detected = detected;
  event.hasGps = lastGpsValid;
  event.latitude = lastGpsLatitude;
  event.longitude = lastGpsLongitude;
  event.accuracyM = lastGpsAccuracyM;
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

void clearGpsLog() {
  gpsLogHead = 0;
  gpsLogCount = 0;
  gpsLogTotal = 0;
  lastGpsLogMs = 0;
  lastGpsValid = false;
  mainEventHead = 0;
  mainEventCount = 0;
  mainEventTotal = 0;
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
  doc["web_url"] = "https://" + WiFi.softAPIP().toString() + "/";
  doc["main_signal_hold_ms"] = MAIN_SIGNAL_HOLD_MS;
  doc["gps_log_interval_ms"] = GPS_LOG_INTERVAL_MS;
  doc["gps_log_count"] = gpsLogCount;
  doc["gps_log_total"] = gpsLogTotal;
  doc["gps_log_capacity"] = gpsLogCapacity;
  doc["last_gps_log_age_ms"] = lastGpsLogMs > 0 ? static_cast<int32_t>(millis() - lastGpsLogMs) : -1;
  doc["main_event_count"] = mainEventCount;
  doc["main_event_total"] = mainEventTotal;
  doc["main_event_capacity"] = MAIN_EVENT_LOG_CAPACITY;
  doc["last_gps_valid"] = lastGpsValid;

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
        <button id="gpsClear" class="danger" type="button">Log löschen</button>
      </div>
      <div class="gps-meta">
        <div>Status: <strong id="gpsStatus">Aus</strong></div>
        <div>Alarm: <strong id="alarmStatus">Aus</strong></div>
        <div>Punkte: <strong id="gpsCount">0</strong></div>
        <div>Hauptsignal-Log: <strong id="mainEventCount">0</strong></div>
        <div>Letzter Punkt: <strong id="gpsLast">-</strong></div>
        <div>Genauigkeit: <strong id="gpsAccuracy">-</strong></div>
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
    let gpsWatchId = null;
    let lastGpsPostMs = 0;
    let requestBusy = false;
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
    const gpsLogIntervalMs = 3000;

    function escapeHtml(value) {
      return String(value ?? '').replace(/[&<>"']/g, c => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
      }[c]));
    }

    function card(ch) {
      const name = escapeHtml(ch.name || ('Kanal ' + ch.channel));
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
          <details>
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

    async function refresh() {
      if (requestBusy || gpsWatchId !== null) return;
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
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
        document.getElementById('gpsLast').textContent = data.last_gps_log_age_ms >= 0 ? data.last_gps_log_age_ms + ' ms' : '-';
        checkMainSignalAlarms(data.channels || []);
        document.getElementById('grid').innerHTML = data.channels.map(card).join('');
        document.getElementById('error').textContent = '';
      } catch (err) {
        setConnectionState('offline', err.message || 'Keine API');
        updateLastContact();
        document.getElementById('error').textContent = 'Keine Verbindung zur API';
      }
    }

    async function postGps(position) {
      const now = Date.now();
      if (now - lastGpsPostMs < gpsLogIntervalMs) return;
      lastGpsPostMs = now;

      const coords = position.coords;
      document.getElementById('gpsStatus').textContent = 'Aktiv';
      document.getElementById('gpsAccuracy').textContent = Math.round(coords.accuracy || 0) + ' m';

      try {
        const url = new URL('/api/gps-log', window.location.href);
        url.searchParams.set('latitude', coords.latitude);
        url.searchParams.set('longitude', coords.longitude);
        url.searchParams.set('accuracy_m', coords.accuracy ?? -1);
        url.searchParams.set('speed_mps', coords.speed ?? -1);
        url.searchParams.set('heading_deg', coords.heading ?? -1);

        const res = await fetch(url.href, { method: 'GET', cache: 'no-store' });
        if (!res.ok) {
          const text = await res.text();
          throw new Error('HTTP ' + res.status + (text ? ': ' + text.slice(0, 80) : ''));
        }
        const result = await res.json();
        document.getElementById('gpsStatus').textContent = 'Aktiv - Punkt ' + result.count;
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Senden fehlgeschlagen: ' + (err.message || err);
      }
    }

    function startGps() {
      if (!window.isSecureContext) {
        document.getElementById('gpsStatus').textContent = 'Browser blockiert GPS: HTTPS nicht vertrauenswürdig';
        return;
      }
      if (!navigator.geolocation) {
        document.getElementById('gpsStatus').textContent = 'Nicht verfügbar';
        return;
      }
      if (gpsWatchId !== null) return;

      gpsWatchId = navigator.geolocation.watchPosition(postGps, err => {
        const messages = {
          1: 'GPS Berechtigung verweigert',
          2: 'GPS Position nicht verfügbar',
          3: 'GPS Zeitüberschreitung'
        };
        document.getElementById('gpsStatus').textContent = messages[err.code] || err.message || 'GPS Fehler';
      }, {
        enableHighAccuracy: true,
        maximumAge: 1000,
        timeout: 10000
      });

      document.getElementById('gpsStart').disabled = true;
      document.getElementById('gpsStop').disabled = false;
      document.getElementById('gpsStatus').textContent = 'Warte auf GPS';
    }

    function stopGps() {
      if (gpsWatchId !== null) {
        navigator.geolocation.clearWatch(gpsWatchId);
        gpsWatchId = null;
      }
      lastGpsPostMs = 0;
      document.getElementById('gpsStart').disabled = false;
      document.getElementById('gpsStop').disabled = true;
      document.getElementById('gpsStatus').textContent = 'Aus';
    }

    async function clearGpsLog() {
      try {
        const res = await fetch('/api/gps-log/clear', { method: 'POST' });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        await refresh();
      } catch (err) {
        document.getElementById('gpsStatus').textContent = 'Löschen fehlgeschlagen';
      }
    }

    async function saveCrop() {
      const input = document.getElementById('cropInput');
      try {
        const res = await fetch('/api/crop', {
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
    refresh();
    connectionTimer = setInterval(updateLastContact, 1000);
    setInterval(refresh, 500);
  </script>
</body>
</html>
)HTML";
}

void handleRoot() {
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

void handleApiGpsLogPost() {
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

  const double latitude = doc["latitude"] | 999.0;
  const double longitude = doc["longitude"] | 999.0;
  const float accuracyM = doc["accuracy_m"] | -1.0f;
  const float speedMps = doc["speed_mps"] | -1.0f;
  const float headingDeg = doc["heading_deg"] | -1.0f;

  if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
    server.send(400, "application/json", "{\"error\":\"invalid_coordinates\"}");
    return;
  }

  appendGpsLog(latitude, longitude, accuracyM, speedMps, headingDeg);

  JsonDocument response;
  response["ok"] = true;
  response["count"] = gpsLogCount;
  response["total"] = gpsLogTotal;

  String json;
  serializeJson(response, json);
  server.send(200, "application/json", json);
}

void handleApiGpsLogGet() {
  const double latitude = server.arg("latitude").toDouble();
  const double longitude = server.arg("longitude").toDouble();
  const float accuracyM = server.arg("accuracy_m").length() > 0 ? server.arg("accuracy_m").toFloat() : -1.0f;
  const float speedMps = server.arg("speed_mps").length() > 0 ? server.arg("speed_mps").toFloat() : -1.0f;
  const float headingDeg = server.arg("heading_deg").length() > 0 ? server.arg("heading_deg").toFloat() : -1.0f;

  if (!server.hasArg("latitude") || !server.hasArg("longitude")) {
    server.send(400, "application/json", "{\"error\":\"missing_coordinates\"}");
    return;
  }

  if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
    server.send(400, "application/json", "{\"error\":\"invalid_coordinates\"}");
    return;
  }

  appendGpsLog(latitude, longitude, accuracyM, speedMps, headingDeg);

  JsonDocument response;
  response["ok"] = true;
  response["count"] = gpsLogCount;
  response["total"] = gpsLogTotal;

  String json;
  serializeJson(response, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleApiGpsLogCsv() {
  String csv;
  csv.reserve(96 + gpsLogCount * 96);
  csv += "index,uptime_ms,crop_name,latitude,longitude,accuracy_m,speed_mps,heading_deg,live_mask,main_mask\n";

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
  initDigitalInputs();
  startWiFiAccessPoint();
  initDigitalOutputs();

  server.setServerKeyAndCert(TLS_SERVER_KEY_DER, TLS_SERVER_KEY_DER_len, TLS_SERVER_CERT_DER, TLS_SERVER_CERT_DER_len);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/channel-name", HTTP_POST, handleApiChannelName);
  server.on("/api/crop", HTTP_POST, handleApiCrop);
  server.on("/api/gps-log", HTTP_GET, handleApiGpsLogGet);
  server.on("/api/gps-log", HTTP_POST, handleApiGpsLogPost);
  server.on("/api/gps-log.csv", HTTP_GET, handleApiGpsLogCsv);
  server.on("/api/gps-log.geojson", HTTP_GET, handleApiGpsLogGeoJson);
  server.on("/api/main-events.csv", HTTP_GET, handleApiMainEventsCsv);
  server.on("/api/main-events.geojson", HTTP_GET, handleApiMainEventsGeoJson);
  server.on("/api/gps-log/clear", HTTP_POST, handleApiGpsLogClear);
  server.onNotFound([]() {
    server.send(404, "application/json", "{\"error\":\"not_found\"}");
  });
  server.begin();

  Serial.println("HTTPS Server gestartet");
  Serial.println("Webseite: https://" + WiFi.softAPIP().toString() + "/");
  Serial.println("API:      https://" + WiFi.softAPIP().toString() + "/api/status");
}

void loop() {
  readDigitalInputs();
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
    Serial.println();
  }
}
