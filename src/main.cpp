#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Waveshare ESP32-S3-POE-ETH-8DI-8DO configuration
// ---------------------------------------------------------------------------
static const char *AP_SSID = "DRILL-8DI8DO";
static const char *AP_PASSWORD = "12345678";

static const char *DEVICE_ID = "waveshare-8di8do-01";
static const char *DEVICE_NAME = "Drillmaschine 8DI/8DO";
static const char *FIRMWARE_VERSION = "0.3.0";

static constexpr uint8_t CHANNEL_COUNT = 8;
static constexpr uint8_t CHANNEL_NAME_LENGTH = 24;

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
static constexpr bool INPUT_ACTIVE_HIGH = true;
static constexpr bool MIRROR_RED_TO_OUTPUT = true;

WebServer server(80);
Preferences preferences;

struct ChannelState {
  bool inputRaw = false;
  bool active = false;
  bool output = false;
  const char *status = "none";
  uint32_t changes = 0;
  uint32_t lastChangeMs = 0;
};

ChannelState channels[CHANNEL_COUNT];
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
uint8_t tcaOutputState = 0x00;
uint32_t lastDebugMs = 0;

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
    channels[i].status = channels[i].active ? "red" : "none";
  }
}

void readDigitalInputs() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    const bool raw = digitalRead(DI_PINS[i]) == HIGH;
    const bool active = INPUT_ACTIVE_HIGH ? raw : !raw;

    if (raw != channels[i].inputRaw) {
      channels[i].changes++;
      channels[i].lastChangeMs = millis();
    }

    channels[i].inputRaw = raw;
    channels[i].active = active;
    channels[i].status = active ? "red" : "none";

    const bool outputOn = MIRROR_RED_TO_OUTPUT && active;
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
  doc["uptime_ms"] = millis();
  doc["wifi_ap_ssid"] = AP_SSID;
  doc["ip"] = WiFi.softAPIP().toString();

  JsonArray array = doc["channels"].to<JsonArray>();
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    JsonObject ch = array.add<JsonObject>();
    ch["channel"] = i + 1;
    ch["name"] = channelNames[i];
    ch["di_gpio"] = DI_PINS[i];
    ch["input_raw"] = channels[i].inputRaw;
    ch["active"] = channels[i].active;
    ch["status"] = channels[i].status;
    ch["output"] = channels[i].output;
    ch["changes"] = channels[i].changes;
    ch["last_change_age_ms"] = channels[i].lastChangeMs > 0 ? static_cast<int32_t>(millis() - channels[i].lastChangeMs) : -1;
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
  <title>Drillmaschine 8DI/8DO</title>
  <style>
    :root { color-scheme: light dark; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    body { margin: 0; background: #111827; color: #f9fafb; }
    main { width: min(1100px, calc(100% - 28px)); margin: 0 auto; padding: 24px 0 34px; }
    h1 { margin: 0 0 8px; font-size: clamp(1.55rem, 5vw, 2.25rem); }
    .meta { color: #9ca3af; margin-bottom: 18px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }
    .card { border: 1px solid #374151; border-radius: 8px; background: #1f2937; padding: 14px; }
    .top { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
    .name { min-width: 0; font-weight: 800; font-size: 1.08rem; overflow-wrap: anywhere; }
    .channel { color: #9ca3af; font-size: .85rem; margin-top: 2px; }
    .dot { width: 40px; height: 40px; border-radius: 50%; background: #6b7280; box-shadow: 0 0 14px #6b7280; flex: 0 0 auto; }
    .dot.red { background: #ef4444; box-shadow: 0 0 20px #ef4444; }
    .dot.none { background: #6b7280; }
    dl { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 10px; margin: 0; font-size: .92rem; }
    dt { color: #9ca3af; }
    dd { margin: 0; text-align: right; font-weight: 650; overflow-wrap: anywhere; }
    .error { min-height: 1.4em; color: #fca5a5; margin-top: 14px; }
  </style>
</head>
<body>
  <main>
    <h1 id="title">Drillmaschine 8DI/8DO</h1>
    <div class="meta">
      <span id="ip">IP: -</span> · <span id="version">Version: -</span> · <span id="updated">-</span>
    </div>
    <div id="grid" class="grid"></div>
    <div id="error" class="error"></div>
  </main>
  <script>
    function escapeHtml(value) {
      return String(value ?? '').replace(/[&<>"']/g, c => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
      }[c]));
    }

    function card(ch) {
      const name = escapeHtml(ch.name || ('Kanal ' + ch.channel));
      return `
        <div class="card" data-channel="${ch.channel}">
          <div class="top">
            <div>
              <div class="name">${name}</div>
              <div class="channel">Kanal ${ch.channel}</div>
            </div>
            <div class="dot ${ch.status}"></div>
          </div>
          <dl>
            <dt>Status</dt><dd>${ch.status}</dd>
            <dt>DI aktiv</dt><dd>${ch.active ? 'Ja' : 'Nein'}</dd>
            <dt>DI GPIO</dt><dd>${ch.di_gpio}</dd>
            <dt>Output</dt><dd>${ch.output ? 'Ein' : 'Aus'}</dd>
            <dt>Wechsel</dt><dd>${ch.changes}</dd>
          </dl>
        </div>`;
    }

    async function refresh() {
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        document.getElementById('title').textContent = data.device_name || data.device_id;
        document.getElementById('ip').textContent = 'IP: ' + data.ip;
        document.getElementById('version').textContent = 'Version: ' + (data.firmware_version || '-');
        document.getElementById('updated').textContent = 'Aktualisiert: ' + new Date().toLocaleTimeString();
        document.getElementById('grid').innerHTML = data.channels.map(card).join('');
        document.getElementById('error').textContent = '';
      } catch (err) {
        document.getElementById('error').textContent = 'Keine Verbindung zur API';
      }
    }

    refresh();
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
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"missing_body\"}");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
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
  initDigitalInputs();
  startWiFiAccessPoint();
  initDigitalOutputs();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/channel-name", HTTP_POST, handleApiChannelName);
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
  server.handleClient();

  if (millis() - lastDebugMs > 1000) {
    lastDebugMs = millis();
    Serial.print("DI:");
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
      Serial.print(channels[i].active ? " 1" : " 0");
    }
    Serial.print(" DO:");
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
      Serial.print(channels[i].output ? " 1" : " 0");
    }
    Serial.println();
  }
}
