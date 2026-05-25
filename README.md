# Drillmaschinen-Automation

Firmware-Projekte für die Automatisierung und Steuerung von Drillmaschinen mit verschiedenen ESP32-basierten Hardwareplattformen.

## 📋 Projektübersicht

Dieses Repository enthält zwei Kontrollvarianten:

### 1. **Waveshare ESP32-S3-POE-8DI-8DO** 

Ethernet-basierte Steuerung mit 8 digitalen Eingängen und 8 digitalen Ausgängen.

**Features:**
- ✅ 8 digitale Eingänge (DI1..DI8) auf GPIO4-11
- ✅ 8 digitale Ausgänge (DO1..DO8) via I2C-IO-Expander (TCA9554)
- ✅ Kanalnamen im Webinterface bearbeitbar und im ESP32 gespeichert
- ✅ Drillmaschinenüberwachung mit Live-Status und Hauptsignal je Kanal
- ✅ Webinterface und REST-API über lokalen HTTP-Server
- ✅ Fahrtaufzeichnung mit Ebyte EWD108-GN05(485) per RS485/Modbus RTU
- ✅ Ereignislog für Hauptsignale mit GPS-Koordinaten, sofern verfügbar
- ✅ Alarmton auf Tablet/Smartphone mit Quittierung
- ✅ WLAN Access Point Fallback
- ✅ ArduinoJson Verarbeitung

**WLAN-Zugang:**
```
SSID: Drillmaschine-M01
Passwort: 12345678
Webseite: http://192.168.4.1/
API: http://192.168.4.1/api/status
```

Hinweis zur SSID: `M01` steht fuer `Modul 01`. Wenn spaeter mehrere
Waveshare-Module an der Maschine verwendet werden, koennen sie fortlaufend
benannt werden, z. B. `Drillmaschine-M01`, `Drillmaschine-M02`,
`Drillmaschine-M03` und `Drillmaschine-M04`.

Die Kanalnamen koennen direkt im Webinterface geaendert werden. Nach `OK`
speichert der ESP32 den Namen im internen NVS-Speicher; `/api/status` liefert
ihn pro Kanal als Feld `name` mit.

Die Bearbeitung der Kanalnamen ist im normalen Statusbereich ausgeblendet und
kann ueber die Detailansicht geoeffnet werden. Die Webseite zeigt ausserdem die
Firmware-Version und die interne ESP32-Temperatur an.

### Fahrtaufzeichnung und GNSS

Die Fahrtaufzeichnung nutzt ein Ebyte EWD108-GN05(485) GNSS-Modul am
RS485-Anschluss des Waveshare-Boards. Der ESP32 liest die Position direkt per
Modbus RTU aus; Tablet-/Smartphone-GPS wird nicht mehr verwendet.

Werkseinstellung des EWD108-GN05(485):

- Modbus-Adresse: `1`
- Baudrate: `9600`
- Format: `8N1`
- Versorgung: `5-24 V DC`

Anschluss:

| EWD108-GN05(485) | Waveshare |
|------------------|-----------|
| V+ | 5-24 V Versorgung |
| GND / V- | GND |
| RS485 A | RS485 A |
| RS485 B | RS485 B |
| Antenne | SMA-Antenne nach oben/aussen montieren |

Firmware-Pins fuer den internen RS485-UART:

| Funktion | GPIO |
|----------|------|
| RS485 RX | GPIO18 |
| RS485 TX | GPIO17 |
| RS485 DE/RE | GPIO21 |

Aufgezeichnet werden:

- GPS-Punkte fuer die Fahrspur
- aktuelles Saatgut/Feldfrucht
- Live- und Hauptsignalmaske der 8 Kanaele
- Hauptsignal-Ereignisse mit GPS-Position, wenn zum Zeitpunkt der Stoerung eine Position bekannt ist

Downloads im Webinterface:

- Fahrtaufzeichnung als CSV
- Fahrtaufzeichnung als GeoJSON fuer Google Maps / My Maps
- Hauptsignal-Log als CSV
- Hauptsignal-Log als GeoJSON
- Sensorlog als CSV und TXT mit Start, Ende, Dauer, Kanal und GPS-Bezug

Bekannter Stand: Das EWD108-GN05(485) ist vorbereitet, muss aber am echten
Modul noch gegen die tatsaechliche Registerbelegung getestet werden. Die
Firmware scannt die Modbus-Holding-Register ab Register `0x0000` nach einem
NMEA-RMC/GGA-Satz. Falls das Modul die NMEA-Zeichenkette an einem anderen
Registerbereich ablegt, muessen `GNSS_SCAN_START_REGISTER` und
`GNSS_SCAN_REGISTER_COUNT` oben in `src/main.cpp` angepasst werden. Die
Maschinenueberwachung der Kanaele funktioniert unabhaengig davon weiter.

### Alarmton

Der Alarmton wird im Browser auf dem Tablet oder Smartphone abgespielt. Er muss
einmal ueber `Ton aktivieren` freigeschaltet werden. Wenn ein Kanal den Status
`Erkannt` erreicht, startet ein pulsierender Ton. Mit `Alarm quittieren` wird
der Ton fuer die aktuell anliegenden Stoerungen stummgeschaltet. Neue
Stoerungen loesen den Alarm erneut aus; wenn alle Kanaele wieder `OK` sind,
wird die Quittierung automatisch zurueckgesetzt.

**Hardware:**
- **Board:** Waveshare ESP32-S3-POE-ETH-8DI-8DO
- **Prozessor:** ESP32-S3 (240 MHz Dual-Core)
- **RAM:** 8 MB PSRAM
- **Flash:** 16 MB (QIO OPI Mode)
- **Ethernet:** W5500 über SPI (PoE)
- **I/O Expander:** TCA9554 (I2C 0x20)

**Pinbelegung:**

| Funktion | GPIO | Bemerkung |
|----------|------|-----------|
| DI1 | GPIO4 | Digital Input 1 |
| DI2 | GPIO5 | Digital Input 2 |
| DI3 | GPIO6 | Digital Input 3 |
| DI4 | GPIO7 | Digital Input 4 |
| DI5 | GPIO8 | Digital Input 5 |
| DI6 | GPIO9 | Digital Input 6 |
| DI7 | GPIO10 | Digital Input 7 |
| DI8 | GPIO11 | Digital Input 8 |
| DO1..DO8 | I2C | via TCA9554 I/O-Expander |
| I2C SDA | GPIO42 | Serial Data |
| I2C SCL | GPIO41 | Serial Clock |
| CAN TX | GPIO2 | CAN Bus TX |
| CAN RX | GPIO3 | CAN Bus RX |
| RS485 TX | GPIO17 | RS485 Transmit |
| RS485 RX | GPIO18 | RS485 Receive |
| RS485 RTS | GPIO21 | RS485 Request to Send |

**Logik (src/main.cpp):**
```cpp
DI aktiv, aber kuerzer als 1,5 s → Status Kein Status
DI aktiv ab 1,5 s                → Status Erkannt / Hauptsignal
DI inaktiv                       → Status OK
```

Konfigurierbar via:
```cpp
static constexpr bool DO_ACTIVE_HIGH = true;
static constexpr bool INPUT_ACTIVE_HIGH = false;
static constexpr bool MIRROR_RED_TO_OUTPUT = true;
```

### 2. **ESP32 LD2410 Radar** 

Radar-basierte Präsenzerkennung mit LD2410 Sensor.

**Features:**
- ✅ LD2410 24-GHz-Radarmodul
- ✅ Präsenzerkennung
- ✅ Bewegungserfassung
- ✅ Entfernungsmessung
- ✅ UART/Seriell-Interface

---

## 🚀 Installation & Setup

### Voraussetzungen

- [PlatformIO](https://platformio.org/) CLI oder VS Code Extension
- Python 3.8+
- Git
- USB-zu-UART Adapter (bei Bedarf)

### Projekt klonen

```bash
git clone https://github.com/tobiasplagge/drillmaschinen-automation.git
cd drillmaschinen-automation
```

### Build

```bash
# Waveshare Projekt
cd waveshare_esp32_s3_poe_8di8do_Drillmaschine
platformio run -e waveshare_esp32_s3_poe_8di8do

# LD2410 Projekt
cd ../esp32-ld2410-Drillmaschine
platformio run -e esp32_s3_poe_8di8do
```

### Upload

```bash
cd waveshare_esp32_s3_poe_8di8do_Drillmaschine
platformio run -e waveshare_esp32_s3_poe_8di8do --target upload
```

### Serial Monitor

```bash
platformio device monitor
```

---

## ⚙️ Konfiguration

### platformio.ini - Waveshare Board

```ini
[env:waveshare_esp32_s3_poe_8di8do]
platform = espressif32
board = esp32-s3-devkitm-1
framework = arduino

monitor_speed = 115200
upload_speed = 115200

board_build.flash_size = 16MB
board_build.psram_type = opi
board_build.arduino.memory_type = qio_opi
board_build.f_flash = 80000000L
board_build.flash_mode = qio

build_flags =
  -DARDUINO_USB_CDC_ON_BOOT=1
  -DBOARD_HAS_PSRAM
  -DCORE_DEBUG_LEVEL=3

lib_deps =
  bblanchon/ArduinoJson@^7.4.2
```

### Build-Flags

| Flag | Beschreibung |
|------|-------------|
| `ARDUINO_USB_CDC_ON_BOOT=1` | USB CDC bei Boot aktivieren |
| `BOARD_HAS_PSRAM` | Externes PSRAM aktivieren |
| `CORE_DEBUG_LEVEL=3` | Debug-Level (0-5) |

---

## 📡 API-Referenz

### Status abrufen

```bash
curl http://192.168.4.1/api/status
```

**Response:**
```json
{
  "di": [0, 1, 0, 1, 0, 0, 1, 0],
  "do": [0, 1, 0, 1, 0, 0, 1, 0],
  "timestamp": 1234567890
}
```

Hinweis: Seit die Fahrtaufzeichnung ueber das RS485-GNSS-Modul laeuft, wird
kein Browser-GPS mehr benoetigt. Deshalb nutzt die Firmware wieder HTTP, weil
das auf dem ESP32 stabiler laeuft als ein Self-Signed-HTTPS-Server:

```bash
curl http://192.168.4.1/api/status
```

Weitere Endpunkte:

| Endpoint | Beschreibung |
|----------|--------------|
| `/api/status` | aktueller Kanal-, Alarm-, Temperatur- und Logstatus |
| `/api/crop` | Saatgut/Feldfrucht speichern |
| `/api/recording` | Fahrtaufzeichnung starten/stoppen |
| `/api/gps-log.csv` | Fahrtaufzeichnung als CSV herunterladen |
| `/api/gps-log.geojson` | Fahrtaufzeichnung als GeoJSON herunterladen |
| `/api/main-events.csv` | Hauptsignal-Ereignisse als CSV herunterladen |
| `/api/main-events.geojson` | Hauptsignal-Ereignisse als GeoJSON herunterladen |
| `/api/sensor-events.csv` | abgeschlossene Sensor-Ausloesungen mit Dauer als CSV herunterladen |
| `/api/sensor-events.txt` | abgeschlossene Sensor-Ausloesungen als lesbares Textfile herunterladen |

---

## 📦 Abhängigkeiten

- **ArduinoJson** v7.4.2+ – JSON-Verarbeitung und REST-API

---

## 📁 Projektstruktur

```
drillmaschinen-automation/
├── waveshare_esp32_s3_poe_8di8do_Drillmaschine/
│   ├── src/
│   │   └── main.cpp              # Hauptprogramm Waveshare
│   ├── include/
│   ├── lib/
│   ├── platformio.ini
│   ├── README.md
│   └── RELEASE_NOTES.md
├── esp32-ld2410-Drillmaschine/
│   ├── src/
│   │   └── main.cpp              # Hauptprogramm LD2410
│   ├── include/
│   ├── lib/
│   ├── platformio.ini
│   └── README.md
└── README.md                     # Dieses Dokument
```

---

## 🔧 Entwicklung & Debug

### Debug-Level erhöhen

```ini
[env:waveshare_esp32_s3_poe_8di8do]
build_flags = 
  -DCORE_DEBUG_LEVEL=4
```

**Debug-Level:**
- 0: Keine Ausgabe
- 1: Fehler
- 2: Fehler + Warnungen
- 3: Fehler + Warnungen + Info
- 4: Verbose
- 5: Very Verbose

---

## 🐛 Troubleshooting

### USB wird nicht erkannt

1. Treiber installieren: `pip install esptool`
2. Verfügbare Ports auflisten: `platformio device list`
3. Board zurücksetzen: Boot-Button halten → Reset drücken

### WLAN funktioniert nicht

- Access Point mit SSID `Drillmaschine-M01` suchen
- Passwort: `12345678`
- In der Konsole Fehler überprüfen

### GNSS/Fahrtaufzeichnung funktioniert nicht

- EWD108-GN05(485) mit 5-24 V versorgen
- RS485 A/B pruefen; bei keiner Antwort A/B testweise tauschen
- Werkseinstellung pruefen: Adresse `1`, `9600 8N1`
- Antenne nach oben/aussen montieren
- PPS-LED am EWD108 pruefen: bei Fix blinkt sie laut Hersteller einmal pro Sekunde
- Im Webinterface auf `GNSS Fix`, `RS485` und Fehlerzaehler achten
- Bekannter Stand: Registerbereich muss am echten Modul noch verifiziert werden

### I2C-Fehler (TCA9554)

- GPIO41 (SCL) und GPIO42 (SDA) überprüfen
- I2C-Adresse `0x20` mit `i2cdetect` überprüfen
- Pullup-Widerstände überprüfen (4,7kΩ empfohlen)

### Upload schlägt fehl

- USB-Kabel überprüfen
- Board in Bootloader-Modus versetzen
- ESP32 Tool neu installieren: `pip install esptool --upgrade`

---

## 📚 Weitere Ressourcen

- [PlatformIO Dokumentation](https://docs.platformio.org/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [Waveshare ESP32-S3-POE Wiki](https://www.waveshare.com/wiki/ESP32-S3-POE-ETH-8DI-8DO)
- [LD2410 Präsenzsensor](https://www.seeedstudio.com/LD2410-Human-Presence-Sensor-p-5643.html)
- [ArduinoJson Dokumentation](https://arduinojson.org/)

---

## 📝 Lizenz

MIT License

## 👤 Autor

Tobias Plagge

---

**Zuletzt aktualisiert:** 25. Mai 2026
