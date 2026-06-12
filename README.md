# Drillmaschinenueberwachung M01

Firmware fuer das Waveshare ESP32-S3-POE-ETH-8DI-8DO Board zur
Ueberwachung einer Drillmaschine.

Aktuelle Firmware-Version: **2.1.0**

## Kurzueberblick

Das Modul stellt einen eigenen WLAN-Access-Point bereit, liest acht digitale
Eingaenge ein, schaltet Ausgaenge ueber den TCA9554-I/O-Expander, zeichnet
Fahrten mit einem RS485-GNSS-Modul auf und zeigt ein Hikvision-Kamerabild ueber
den W5500-LAN-Port an.

Die Webseite laeuft lokal auf dem ESP32:

```text
SSID: Drillmaschine-M01
Passwort: 12345678
Webseite: http://192.168.4.1/
API: http://192.168.4.1/api/status
```

## Funktionen

- 8 digitale Eingaenge `DI1..DI8` auf `GPIO4..GPIO11`
- 8 digitale Ausgaenge ueber TCA9554, I2C-Adresse `0x20`
- Kanalnamen im Webinterface editierbar und im NVS gespeichert
- Hauptsignal-Erkennung mit konfigurierbarer Empfindlichkeit
- Alarmton im Tablet-/Smartphone-Browser mit Quittierung
- Lichtausgang direkt auf der Mainpage schaltbar
- Pneumatikventile auf Ausgang `DO2..DO5` direkt auf der Mainpage schaltbar
- Fahrtaufzeichnung mit Ebyte EWD108-GN05(485) per RS485/Modbus RTU
- Auto-Start der Fahrtaufzeichnung nach stabilem GNSS-Fix
- Fahrtarchiv in LittleFS
- CSV-, TXT- und GeoJSON-Downloads
- Live-Kartenansicht mit OpenTopoMap, GNSS-Signalqualitaet und lokalem Canvas-Fallback
- Bis zu vier Hikvision-Kameras ueber W5500-LAN mit Substream/Mainstream-Umschalter
- Kamera-Proxy im ESP32, damit Browser im ESP32-WLAN das LAN-Kamerabild sieht
- Kamera-IP, Benutzer und Passwort im Webinterface einstellbar
- Kamera-Verbindungstest ueber Ethernet
- RS485-/GNSS-Diagnose mit Bytezaehler, Rohdaten, Baud-, Adress- und Registerscan
- Systemauslastung mit Heap, PSRAM, LittleFS, Flash, Uptime und Ethernet-Status
- Boot-Zaehler, Reset-Grund, Systemlog und Task-Watchdog

## Bedienung

### Mainpage

Die Mainpage zeigt:

- Kontaktstatus und ESP32-Temperatur
- Firmware-Version
- Licht-Schalter
- Ton-Aktivierung
- Alarm-Quittierung
- Pneumatikventile `Sensor 1-6`, `Sensor 7-12`, `Sensor 13-18`, `Sensor 19-24`
- Sensoruebersicht
- GNSS-/Aufzeichnungsstatus
- Kamerabereich fuer konfigurierte Kameras

Der Alarmton muss im Browser einmal mit **Ton aktivieren** freigeschaltet
werden. Danach startet er automatisch, wenn ein Hauptsignal in den Zustand
`Erkannt` wechselt. Mit **Alarm quittieren** wird der aktuelle Alarm
stummgeschaltet.

Wenn der Kamerastream offen ist, pausiert die Webseite die zyklischen
API-Refreshes. Der Kontaktstatus zeigt dann **Kamera aktiv** mit gruen
blinkendem Punkt. Dadurch wird der ESP32-Webserver nicht durch parallele
Stream- und API-Verbindungen ueberlastet.

### Einstellungen

Im Reiter **Einstellungen** koennen gesetzt werden:

- Kamera-IP
- Kamera-Benutzer
- Kamera-Passwort
- RS485-/GNSS-Verbindungstests
- Systemauslastung
- Sensor-Empfindlichkeit
- Saat-Vorschlaege
- Kanalnamen in den Kanaldetails

Der Button **Verbindung testen** prueft die jeweilige Kamera ueber den
W5500-LAN-Port.
Eine typische Ausgabe ist:

```text
Substream 102 ueber ethernet: HTTP 200, MJPEG erkannt
Mainstream 101 ueber ethernet: HTTP 200, kein MJPEG
```

Das Passwort wird gespeichert, aber nicht ueber `/api/status` an den Browser
zurueckgegeben. Beim Aendern der Kameradaten muss es neu eingetragen werden.

## Kamera

Standardwerte nach frischem Flash:

```text
IP: 192.168.4.20
Benutzer: admin
Passwort: Administrator01
```

Die erste Hikvision-Kamera haengt direkt am LAN-Port des ESP32/W5500. Bis zu
vier Kameras koennen im Reiter **Einstellungen** angelegt werden. Nicht
konfigurierte Kameras werden auf der Mainpage ausgeblendet.

Der Browser ist im WLAN des ESP32 und kann die Kamera nicht direkt erreichen.
Deshalb stellt die Firmware Proxy-Endpunkte bereit:

```text
/camera/1/substream
/camera/1/mainstream
/camera/2/substream
/camera/2/mainstream
/camera/3/substream
/camera/3/mainstream
/camera/4/substream
/camera/4/mainstream
```

Der Substream sollte auf MJPEG gestellt sein. Mainstream `101` liefert bei
vielen Hikvision-Kameras H.264/H.265 oder XML und kann dann im Browser-`img`
nicht als Livebild angezeigt werden. Der Substream `102` ist der stabile
Standard fuer das Webinterface.

Hikvision-Pfade:

```text
Mainstream: /ISAPI/Streaming/channels/101/httpPreview
Substream:  /ISAPI/Streaming/channels/102/httpPreview
```

## Netzwerk

### WLAN

Der ESP32 stellt einen Access Point bereit:

```text
ESP32 WLAN-IP: 192.168.4.1
SSID:          Drillmaschine-M01
Passwort:      12345678
```

### Ethernet/W5500

Der LAN-Port wird fuer die Kamera verwendet:

```text
ESP32 Ethernet-IP: 192.168.4.10
Kamera-IP:         192.168.4.20
Subnetz:           255.255.255.0
Gateway:           192.168.4.1
```

W5500-Pinbelegung:

| Funktion | GPIO |
|----------|------|
| INT | GPIO12 |
| MOSI | GPIO13 |
| MISO | GPIO14 |
| SCLK | GPIO15 |
| CS | GPIO16 |
| RST | GPIO39 |

## GNSS und Fahrtaufzeichnung

Die Fahrtaufzeichnung nutzt ein Ebyte EWD108-GN05(485) GNSS-Modul am
RS485-Anschluss.

Werkseinstellung:

```text
Modbus-Adresse: 1
Baudrate:       9600
Format:         8N1
Versorgung:     5-24 V DC
```

RS485-Pins:

| Funktion | GPIO |
|----------|------|
| RS485 RX | GPIO18 |
| RS485 TX | GPIO17 |
| RS485 DE/RE | GPIO21 |

Verdrahtung:

| GPS-Modul | Waveshare |
|-----------|-----------|
| A | A |
| B | B |
| 12V - / GND | GND |
| 12V + | externe 12V-Versorgung |

Das GPS-Modul wird extern mit 12 V versorgt. Der Minuspol der 12V-Quelle muss
mit `GND` des Waveshare verbunden sein, damit RS485 einen gemeinsamen Bezug hat.
Bei dem getesteten Modul bleibt `B` auf `B` und `A` auf `A`.

Die Firmware liest den RMC-Datensatz des Ebyte-Moduls aus Holding-Register
`0x0005` ueber `35` Register (`70` Byte). Das entspricht dem
Herstellerbeispiel `01 03 00 05 00 23 14 12`.

Automatik:

- Sobald ein GNSS-Fix mindestens 3 Sekunden stabil ist, startet die
  Fahrtaufzeichnung automatisch.
- Eine manuell gestoppte Fahrt wird erst nach erneutem GNSS-Fix wieder
  automatisch gestartet.
- GPS-Schreibzugriffe werden gepuffert und spaetestens alle 30 Sekunden in
  LittleFS geschrieben.

## Dateien und Logs

Jede Fahrt erhaelt eine ID im Format:

```text
M01-B000012-F0007
```

Im Webinterface unter **Dateien und Wartung** stehen bereit:

- GPS-Log CSV
- GPS-Log GeoJSON
- kombinierte Route und Sensorereignisse GeoJSON
- Hauptsignal-Ereignisse CSV
- Sensorlog CSV
- Sensorlog TXT
- Neustart-Log
- Fahrtarchiv

Das Loeschen des Live-Logs ist waehrend einer aktiven Aufzeichnung gesperrt.
Archivierte Fahrtdateien bleiben dabei erhalten.

## Hardware

| Komponente | Wert |
|------------|------|
| Board | Waveshare ESP32-S3-POE-ETH-8DI-8DO |
| CPU | ESP32-S3, 240 MHz |
| RAM | 8 MB PSRAM |
| Flash | 16 MB |
| Ethernet | W5500 ueber SPI |
| DO-Expander | TCA9554, I2C `0x20` |
| GNSS | Ebyte EWD108-GN05(485) |
| Kamera | Hikvision HTTP/MJPEG Preview |

I/O-Pins:

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
| DO1..DO8 | I2C | via TCA9554 |
| I2C SDA | GPIO42 | TCA9554 |
| I2C SCL | GPIO41 | TCA9554 |
| RS485 TX | GPIO17 | GNSS |
| RS485 RX | GPIO18 | GNSS |
| RS485 DE/RE | GPIO21 | GNSS |
| W5500 INT | GPIO12 | Ethernet |
| W5500 MOSI | GPIO13 | Ethernet |
| W5500 MISO | GPIO14 | Ethernet |
| W5500 SCLK | GPIO15 | Ethernet |
| W5500 CS | GPIO16 | Ethernet |
| W5500 RST | GPIO39 | Ethernet |

## Signal-Logik

```text
DI inaktiv                       -> Status OK
DI aktiv, aber kuerzer als 1,5 s -> Status Kein Status
DI aktiv ab 1,5 s                -> Status Erkannt / Hauptsignal
```

Konfiguration in `src/main.cpp`:

```cpp
static constexpr bool DO_ACTIVE_HIGH = true;
static constexpr bool INPUT_ACTIVE_HIGH = false;
static constexpr bool MIRROR_RED_TO_OUTPUT = true;
static constexpr uint8_t LIGHT_OUTPUT_CHANNEL = 1;
```

`DO2..DO5` sind fuer Pneumatikventile reserviert und werden nicht automatisch
durch Hauptsignale gespiegelt.

## API

Basis:

```text
http://192.168.4.1
```

| Endpoint | Methode | Beschreibung |
|----------|---------|--------------|
| `/api/status` | GET | Gesamtstatus, GNSS, Kamera, Logs, Kanaele |
| `/api/camera-test?index=0` | GET | Kamera-Verbindung ueber Ethernet testen |
| `/api/camera-settings` | POST | Kamera-IP, Benutzer, Passwort je Kamera speichern |
| `/camera/1/substream` bis `/camera/4/substream` | GET | Kamera-Substream ueber ESP32-Proxy |
| `/camera/1/mainstream` bis `/camera/4/mainstream` | GET | Kamera-Mainstream ueber ESP32-Proxy |
| `/api/rs485-scan` | GET | RS485-Baud-/Byte-Test |
| `/api/rs485-address-scan` | GET | Modbus-Adressscan |
| `/api/rs485-register-scan` | GET | Modbus-Registerscan |
| `/api/alarm/ack` | POST | Alarm quittieren |
| `/api/channel-name` | POST | Kanalnamen speichern |
| `/api/crop` | POST | Saat speichern |
| `/api/field` | POST | Feldname speichern |
| `/api/sensitivity` | POST | Hauptsignal-Haltezeit speichern |
| `/api/recording` | POST | Aufzeichnung starten/stoppen |
| `/api/output` | POST | Ausgang schalten |
| `/api/track` | GET | kompakte Live-Fahrt |
| `/api/gps-log.csv` | GET | GPS-Live-Log als CSV |
| `/api/gps-log.geojson` | GET | GPS-Live-Log als GeoJSON |
| `/api/main-events.csv` | GET | Hauptsignal-Ereignisse als CSV |
| `/api/main-events.geojson` | GET | Hauptsignal-Ereignisse als GeoJSON |
| `/api/sensor-events.csv` | GET | Sensorereignisse als CSV |
| `/api/sensor-events.txt` | GET | Sensorereignisse als Text |
| `/api/combined.geojson` | GET | Route und Sensorereignisse kombiniert |
| `/api/archive` | GET | archivierte Fahrtdateien listen |
| `/api/archive/download` | GET | archivierte Datei herunterladen |
| `/api/system-events.log` | GET | Neustart-Log herunterladen |
| `/api/gps-log/clear` | POST | Live-Log loeschen |
| `/api/crops` | GET/POST | Saat-Vorschlaege lesen/aendern |

Beispiele:

```bash
curl http://192.168.4.1/api/status

curl -X POST http://192.168.4.1/api/camera-settings \
  -H 'Content-Type: application/json' \
  -d '{"index":0,"host":"192.168.4.20","username":"admin","password":"Administrator01"}'

curl -X POST http://192.168.4.1/api/sensitivity \
  -H 'Content-Type: application/json' \
  -d '{"main_signal_hold_ms":1500}'
```

## Build und Upload

Voraussetzungen:

- PlatformIO CLI oder VS Code PlatformIO Extension
- Git
- USB-Kabel mit Datenleitungen

Build:

```bash
platformio run -e waveshare_esp32_s3_poe_8di8do
```

Upload:

```bash
platformio run -e waveshare_esp32_s3_poe_8di8do --target upload
```

Serieller Monitor:

```bash
platformio device monitor
```

Lokaler PlatformIO-Pfad auf macOS kann zum Beispiel sein:

```bash
~/.platformio/penv/bin/pio run
```

## PlatformIO-Konfiguration

Wichtige Einstellungen aus `platformio.ini`:

```ini
[env:waveshare_esp32_s3_poe_8di8do]
platform = espressif32
board = esp32-s3-devkitm-1
framework = arduino

board_build.flash_size = 16MB
board_build.psram_type = opi
board_build.arduino.memory_type = qio_opi

lib_deps =
  bblanchon/ArduinoJson@^7.4.2
  arduino-libraries/Ethernet@^2.0.2
```

## Troubleshooting

### Kamera zeigt kein Bild

- In den Einstellungen **Verbindung testen** ausfuehren.
- Substream muss `HTTP 200, MJPEG erkannt` melden.
- Hikvision-Substream `102` auf MJPEG stellen.
- Kamera-IP, Benutzer und Passwort pruefen.
- Kamera muss am W5500-LAN-Port erreichbar sein.
- Wenn Mainstream `kein MJPEG` meldet, ist das fuer viele Hikvision-Kameras
  normal. Substream verwenden.

### Kamera aktiv, aber API pausiert

Das ist beabsichtigt. Der ESP32-Webserver kann den Kamerastream und die
zyklischen API-Refreshes nicht beliebig parallel bedienen. Bei offenem
Kamerabild zeigt die Webseite **Kamera aktiv** und pausiert die API-Abfragen.

### GNSS/Fahrtaufzeichnung funktioniert nicht

- EWD108-GN05(485) mit 5-24 V versorgen.
- RS485 A/B pruefen; beim getesteten Ebyte-Modul ist `B` am GPS-Modul auch `B`
  am Waveshare.
- `12V -` der externen GPS-Versorgung muss mit `GND` des Waveshare verbunden
  sein.
- Modbus-Adresse `1`, `9600 8N1` pruefen.
- Antenne nach oben/aussen montieren.
- PPS-LED am GNSS-Modul pruefen.
- Im Webinterface auf `GNSS Fix`, `RS485` und Fehlerzaehler achten.

### WLAN funktioniert nicht

- Access Point `Drillmaschine-M01` suchen.
- Passwort `12345678` verwenden.
- Seriellen Monitor pruefen.

### I2C-Ausgaenge funktionieren nicht

- GPIO41/GPIO42 pruefen.
- TCA9554-Adresse `0x20` pruefen.
- Pullups auf SDA/SCL pruefen.
- Versorgung und Masseverbindung pruefen.

### Upload schlaegt fehl

- USB-Kabel pruefen.
- Bootloader-Modus verwenden.
- Port in PlatformIO pruefen.
- `platformio device list` ausfuehren.

## Einbauhinweise

Fuer den produktiven Einbau an der Landmaschine empfohlen:

- eigene Sicherung nahe am Abgriff
- Verpolschutz
- TVS-Diode gegen Spannungsspitzen
- Automotive-DC/DC-Wandler
- gemeinsamer sauberer Massepunkt
- geschirmte oder verdrillte Leitungen fuer RS485
- 120-Ohm-RS485-Abschluss nur an den Busenden
- Zugentlastung und vibrationsfeste Steckverbinder

## Projektdateien

```text
.
├── platformio.ini
├── README.md
├── RELEASE_NOTES.md
├── src/
│   └── main.cpp
├── include/
├── lib/
└── test/
```

## Weitere Ressourcen

- PlatformIO Dokumentation: https://docs.platformio.org/
- ESP32 Arduino Core: https://github.com/espressif/arduino-esp32
- Waveshare ESP32-S3-POE-ETH-8DI-8DO Wiki: https://www.waveshare.com/wiki/ESP32-S3-POE-ETH-8DI-8DO
- ArduinoJson: https://arduinojson.org/

## Autor

Tobias Plagge

Letzte Aktualisierung: 7. Juni 2026
