# Waveshare ESP32-S3-POE-ETH-8DI-8DO Drillmaschine

PlatformIO/Arduino-Projekt fuer das Waveshare `ESP32-S3-POE-ETH-8DI-8DO`.

## Funktion

- liest 8 digitale Eingänge `DI1..DI8`
- berechnet je Kanal einen Status
- spiegelt aktive Eingänge auf die 8 digitalen Ausgänge `DO1..DO8`
- stellt eine lokale Webseite bereit
- stellt eine REST-API unter `/api/status` bereit
- startet aktuell einen WLAN Access Point

## WLAN

```text
SSID: DRILL-8DI8DO
Passwort: 12345678
Webseite: http://192.168.4.1/
API: http://192.168.4.1/api/status
```

## Waveshare Pinbelegung

Laut Waveshare-Wiki:

| Funktion | GPIO |
| --- | --- |
| DI1 | GPIO4 |
| DI2 | GPIO5 |
| DI3 | GPIO6 |
| DI4 | GPIO7 |
| DI5 | GPIO8 |
| DI6 | GPIO9 |
| DI7 | GPIO10 |
| DI8 | GPIO11 |
| DO1..DO8 | TCA9554 I/O-Expander, I2C-Adresse `0x20` |
| I2C SDA | GPIO42 |
| I2C SCL | GPIO41 |
| CAN TX | GPIO2 |
| CAN RX | GPIO3 |
| RS485 TX | GPIO17 |
| RS485 RX | GPIO18 |
| RS485 RTS | GPIO21 |

## Logik

Aktuell gilt:

```text
DI aktiv   -> status = red  -> passender DO-Ausgang ein
DI inaktiv -> status = none -> passender DO-Ausgang aus
```

Die Logik kann oben in `src/main.cpp` angepasst werden:

```cpp
static constexpr bool DO_ACTIVE_HIGH = true;
static constexpr bool INPUT_ACTIVE_HIGH = true;
static constexpr bool MIRROR_RED_TO_OUTPUT = true;
```

## Hinweis

Die Ethernet/W5500-Hardware ist auf dem Board vorhanden, die aktuelle Firmware
nutzt aber bewusst zuerst WLAN-AP, weil das fuer Inbetriebnahme und Handy-Test
am einfachsten ist.
