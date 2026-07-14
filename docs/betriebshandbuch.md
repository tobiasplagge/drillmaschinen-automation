# Betriebshandbuch Drillmaschinenueberwachung M01

Dieses Betriebshandbuch beschreibt den Aufbau, die Bedienung, die Anschluesse
und die Fehlersuche fuer das Modul `Drillmaschinenueberwachung M01` auf Basis
des Waveshare ESP32-S3-POE-ETH-8DI-8DO.

Ergaenzender Schaltplan:

```text
docs/di_do_schematic.html
```

## 1. Zweck des Systems

Das System ueberwacht eine Drillmaschine mit mehreren Sensoren. Es erkennt
Stoerungen an den Sensoren, zeigt den Zustand lokal auf einer Webseite an,
schaltet Licht und Pneumatikventile und kann Fahrten mit GNSS-Daten
aufzeichnen.

Der ESP32 arbeitet ohne Cloud. Tablet oder Smartphone verbinden sich direkt mit
dem WLAN des Moduls.

## 2. Hauptkomponenten

| Komponente | Aufgabe |
|------------|---------|
| Waveshare ESP32-S3-POE-ETH-8DI-8DO | Steuergeraet, Webserver, DI/DO, WLAN, LAN |
| E3F-DS30C4 Lichttaster | Sensoren fuer Saat-/Durchflusserkennung an DI1-DI6 |
| EWD108-GN05(485) | GNSS-Modul ueber RS485 fuer Fahrtaufzeichnung |
| Hikvision Kamera | Kamerabild ueber LAN-Port des ESP32 |
| 12V-Hutschienenrelais | Lichtschaltung ueber DO1 |
| Heschen 3V210-08 DC12V | Pneumatikventile ueber DO8, DO7, DO6 und DO5 |
| 24V -> 12V DC/DC-Wandler | Versorgung fuer 12V-Relais und 12V-Ventile |

## 3. Zugang zur Weboberflaeche

Der ESP32 baut einen eigenen WLAN-Hotspot auf.

```text
SSID:      Drillmaschine-M01
Passwort:  12345678
Webseite:  http://192.168.4.1/
API:       http://192.168.4.1/api/status
```

Vorgehen:

1. Tablet oder Smartphone mit `Drillmaschine-M01` verbinden.
2. Browser oeffnen.
3. `http://192.168.4.1/` aufrufen.
4. Auf der Webseite die Sensoren, GNSS-Daten, Kamera und Ausgaenge pruefen.

Hinweis: Wenn das Kamerabild geoeffnet ist, kann die Webseite die API-Abfragen
pausieren. Das ist vorgesehen, damit der ESP32-Webserver nicht durch Kamera und
API gleichzeitig ueberlastet wird.

## 4. Spannungsversorgung

### 4.1 Board und Sensoren

Das Waveshare-Board wird ueber den Eingang `7-36V` versorgt.

| Klemme | Anschluss |
|--------|-----------|
| `7-36V +` | +24 V Maschinenversorgung, abgesichert |
| `7-36V -` | 0 V / Masse |

Die E3F-DS30C4 Sensoren werden ebenfalls mit 24 V betrieben.

### 4.2 12V-Lasten

Das Lichtrelais und die Heschen 3V210-08 Pneumatikventile sind als 12V-Lasten
vorgesehen. Dafuer wird ein DC/DC-Wandler von 24 V auf 12 V eingesetzt.

| Versorgung | Verwendung |
|------------|------------|
| +24 V | Board, Sensoren, DC/DC-Eingang |
| +12 V | Lichtrelais, Pneumatikventile |
| 0 V | gemeinsamer Massepunkt fuer Board, Sensoren, DC/DC und Lasten |

Wichtig: Alle 0-V/GND-Leitungen muessen einen gemeinsamen Bezug haben.

## 5. Digitaleingaenge DI

Die Eingange sind in der Firmware aktiv LOW konfiguriert.

```text
HIGH / offen  = nicht erkannt
LOW / 0 V     = erkannt
```

Das passt zu den E3F-DS30C4 Sensoren in NPN/NO-Verdrahtung.

### 5.1 Sensoranschluss E3F-DS30C4

Typische Aderfarben:

| Sensorader | Anschluss |
|------------|-----------|
| Braun | +24 V |
| Blau | 0 V / GND |
| Schwarz | DI-Eingang |

Wenn der Sensor ausloest, zieht der schwarze Signaldraht den DI-Eingang gegen
0 V. In der Weboberflaeche wird der Kanal dann als erkannt angezeigt.

### 5.2 DI-Klemmenbelegung

| Klemme | GPIO | Funktion |
|--------|------|----------|
| DI1 | GPIO4 | Sensor 1 |
| DI2 | GPIO5 | Sensor 2 |
| DI3 | GPIO6 | Sensor 3 |
| DI4 | GPIO7 | Sensor 4 |
| DI5 | GPIO8 | Sensor 5 |
| DI6 | GPIO9 | Sensor 6 |
| DI7 | GPIO10 | Reserve |
| DI8 | GPIO11 | Hubwerksignal |

DI8 ist fuer das Hubwerksignal vorgesehen. Das Signal muss ebenfalls als
aktive LOW-Schaltung ausgefuehrt werden oder ueber ein Relais/Optokoppler auf
LOW umgesetzt werden.

## 6. Digitalausgaenge DO

Die Ausgaenge werden ueber den TCA9554-Ausgangsexpander des Waveshare-Boards
geschaltet. In der Firmware gilt:

```text
DEFAULT_DO_ACTIVE_HIGH = false, LIGHT_DO_ACTIVE_HIGH = false
```

Damit schalten die Pneumatikventile aktiv-low und bleiben im Ruhezustand aus.
Alle belegten Digitalausgänge werden mit derselben Active-Low-Logik geschaltet.

### 6.1 DO-Klemmenbelegung

| Klemme | Funktion | Last |
|--------|----------|------|
| DO1 | Licht | 12V-Relais fuer Licht |
| DO4 | Lüfter | Automatisch ein über 43 °C, aus bei 41 °C |
| DO8 | Pneumatikventil 1 | Heschen 3V210-08 DC12V |
| DO7 | Pneumatikventil 2 | Heschen 3V210-08 DC12V |
| DO6 | Pneumatikventil 3 | Heschen 3V210-08 DC12V |
| DO5 | Pneumatikventil 4 | Heschen 3V210-08 DC12V |
| DO2 | Reserve | frei |
| DO3 | Reserve | frei |

### 6.2 Licht ueber DO1

DO1 schaltet nicht direkt ein starkes Arbeitslicht, sondern idealerweise ein
12V-Relais oder ein geeignetes MOSFET-/Relaismodul.

Empfohlene Verdrahtung:

```text
+12 V Lastversorgung -> Relaisspule +
Relaisspule -         -> DO1
DO GND                -> 0 V Lastversorgung
Relaiskontakt         -> Lichtversorgung schalten
```

Vor Anschluss pruefen, ob die Relaisspule mit dem Ausgangsstrom des Boards
kompatibel ist.

### 6.3 Pneumatikventile ueber DO8, DO7, DO6 und DO5

Die vier Ventile werden mit 12 V versorgt und ueber DO8, DO7, DO6 und DO5 geschaltet.

Empfohlene Verdrahtung je Ventil:

```text
+12 V Lastversorgung -> Ventilspule +
Ventilspule -         -> DO8/DO7/DO6/DO5
DO GND                -> 0 V Lastversorgung
```

Bei Magnetventilen muss eine Freilaufdiode oder Schutzbeschaltung vorgesehen
werden, falls diese nicht bereits im Ventil/Modul integriert ist.

## 7. GNSS / Fahrtaufzeichnung

Das GNSS-Modul EWD108-GN05(485) wird per RS485 angebunden.

Werkseinstellungen:

```text
Modbus-Adresse: 1
Baudrate:       9600
Format:         8N1
```

### 7.1 RS485-Anschluss

| GNSS-Modul | Waveshare |
|------------|-----------|
| A | RS485 A |
| B | RS485 B |
| GND / 12V - | GND / 0 V |
| Versorgung + | nach Modulvorgabe, z. B. 12 V |

Falls keine Daten empfangen werden, zuerst A/B tauschen oder den
RS485-Test in der Weboberflaeche verwenden.

### 7.2 Fahrtaufzeichnung

Die Fahrtaufzeichnung kann GPS-Punkte und Sensorereignisse speichern. Dateien
koennen im Webinterface heruntergeladen werden.

Typische Downloads:

| Datei | Inhalt |
|-------|--------|
| GPS-Log CSV | Zeit, Position, GNSS-Daten |
| GPS-Log GeoJSON | Fahrspur fuer Kartenprogramme |
| Sensorlog CSV/TXT | Sensorereignisse mit Dauer |
| Combined GeoJSON | Route plus Stoerungsmarker |

## 8. Kamera

Die Hikvision-Kamera wird ueber den LAN-Port des Waveshare-Boards angebunden.
Der ESP32 stellt das Kamerabild ueber Proxy-Endpunkte im WLAN bereit.

Standardwerte:

```text
Kamera-IP: 192.168.4.20
Benutzer:  admin
Passwort:  Administrator01
```

Wichtig fuer das Livebild:

- Substream 102 auf MJPEG stellen.
- Mainstream 101 ist oft H.264/H.265 und wird im Browserbild nicht direkt
  angezeigt.
- Kamera ueber die Einstellungen testen.

## 9. Bedienung im Alltag

### 9.1 Start

1. Maschine einschalten.
2. Warten, bis das WLAN `Drillmaschine-M01` sichtbar ist.
3. Tablet/Smartphone verbinden.
4. Webseite oeffnen.
5. Kontaktstatus und Sensorstatus pruefen.
6. Ton aktivieren, falls akustischer Alarm gewuenscht ist.

### 9.2 Sensoranzeige

Die Hauptseite zeigt die Sensorzustaende. Grundlogik:

| Zustand | Bedeutung |
|---------|-----------|
| OK / gruen | Sensor arbeitet normal |
| Kein Status / gelb oder orange | kein sicherer Sensorimpuls oder Bereitschaft |
| Erkannt / rot | Hauptsignal/Stoerung erkannt |

Die genaue Bewertung haengt von der eingestellten Empfindlichkeit ab.

### 9.3 Alarmton

Der Alarmton wird auf dem Tablet/Smartphone abgespielt. Browser verlangen eine
Benutzeraktion, daher muss der Ton einmal auf der Webseite aktiviert werden.

Mit `Alarm quittieren` wird ein aktueller Alarm stummgeschaltet. Wenn spaeter
ein neuer Fehler auftritt, kann wieder ein Ton ausgelöst werden.

### 9.4 Licht

Der Button `Licht` schaltet DO1.

| Anzeige | Bedeutung |
|---------|-----------|
| gruen | Licht eingeschaltet |
| rot | Licht ausgeschaltet |
| orange | Ausgang nicht schaltbar oder DO-Expander nicht bereit |

### 9.5 Pneumatik

Die Pneumatikventile werden ueber die Webseite geschaltet. Die Zuordnung ist:
Ventil 1 auf DO8, Ventil 2 auf DO7, Ventil 3 auf DO6 und Ventil 4 auf DO5.
Ein Ventil wird jeweils fuer 5 Sekunden eingeschaltet. Waehrend dieser Zeit
laeuft auf der Webseite ein Countdown und die anderen Ventile sind gesperrt.

### 9.6 Luefter

`DO4` steuert den Luefter automatisch anhand der internen ESP-Temperatur. Bei
mehr als `43 °C` wird der Luefter eingeschaltet, bei `41 °C` oder weniger wird
er ausgeschaltet. Die Anzeige `Luefter an` beziehungsweise `Luefter aus` ist
eine reine Statusanzeige und kein Bedienelement.

### 9.7 Fahrtende, Download und Portal-Upload

Wird eine Fahrt manuell beendet, erscheint ein Abschlussdialog mit Fahrt-ID,
Uploadmoeglichkeit und direkten Downloads. Scheitert ein automatischer Upload,
weil das Portal ueber das Mobilgeraet nicht erreichbar ist, bleibt die Fahrt im
ESP32 gespeichert und derselbe Dialog bietet die Dateien zum Download an.

Der Portal-Upload wird vom Browser auf dem Tablet oder Mobiltelefon ausgefuehrt:

1. Fahrtdateien lokal vom ESP32 abrufen.
2. Multipart-Paket mit Metadaten, GPS-, Hauptsignal- und Sensor-CSV bilden.
3. Paket mit Bearer-Token ueber die Internetverbindung des Mobilgeraets senden.

Die Portal-URL muss auf `/api/trips/upload` enden. Mit `Verbindung testen`
werden URL, Erreichbarkeit und Token ohne Fahrt-Upload geprueft. Der automatische
Upload funktioniert nur, solange die Bedienseite geoeffnet ist.

Nach einem manuellen Stopp bleibt der Auto-Start verriegelt, solange das
Hubwerk-unten-Signal dauerhaft anliegt. Erst ein erkanntes `Hubwerk oben` gibt
den Auto-Start fuer das naechste Absenken wieder frei.

## 10. Wartung

Regelmaessig pruefen:

- Steckverbinder auf festen Sitz.
- Sensoren auf Schmutz, Staub und Beschädigung.
- Kabel auf Scheuerstellen und Brueche.
- 24V- und 12V-Sicherungen.
- DC/DC-Wandler auf festen Anschluss.
- Pneumatikventile auf Erwärmung und Schaltgeraeusch.
- Kamera-Stecker und LAN-Kabel.
- GNSS-Antenne und freie Sicht nach oben.

Vor Saisonbeginn:

1. Jeden Sensor einzeln ausloesen und Webanzeige pruefen.
2. Licht schalten.
3. Jedes Pneumatikventil kurz schalten.
4. GNSS-Fix im Freien pruefen.
5. Eine kurze Testfahrt aufzeichnen und GeoJSON/CSV herunterladen.

## 11. Fehlersuche und Reparatur

### 11.1 Webseite nicht erreichbar

Moegliche Ursachen:

- Tablet nicht mit `Drillmaschine-M01` verbunden.
- Falsche Adresse.
- ESP32 nicht gestartet.
- Versorgung fehlt.

Pruefen:

1. WLAN-Liste am Tablet pruefen.
2. `http://192.168.4.1/` aufrufen.
3. Falls vorhanden, seriellen Monitor ueber USB oeffnen.
4. Board spannungslos machen, 10 Sekunden warten, wieder einschalten.

Reparatur:

- Sicherung ersetzen.
- Versorgungskabel pruefen.
- Firmware neu flashen, wenn das Board startet, aber keine Webseite liefert.

### 11.2 `/api/status` funktioniert, Webseite aber nicht richtig

Moegliche Ursachen:

- Browsercache.
- Kamera-Stream blockiert parallele API-Abfragen.
- Alte/inkompatible Browser-Version.

Pruefen:

1. Browsercache loeschen oder privaten Tab nutzen.
2. Kamera-Panel schliessen.
3. Seite neu laden.
4. Direkt `http://192.168.4.1/api/status` testen.

Reparatur:

- Firmware auf die als funktionierend markierte Version flashen.
- Kamerastream auf Substream/MJPEG umstellen.

### 11.3 Ein Sensor zeigt dauerhaft erkannt

Moegliche Ursachen:

- Signaldraht liegt dauerhaft auf GND.
- Sensor falsch eingestellt oder verschmutzt.
- Sensor defekt.
- DI-Eingang falsch verdrahtet.

Pruefen mit Multimeter:

```text
Sensor nicht ausgeloest: DI gegen GND ca. HIGH / mehrere Volt
Sensor ausgeloest:      DI gegen GND ca. 0 V
```

Reparatur:

1. Sensor reinigen.
2. Signaldraht am DI abklemmen.
3. Wenn DI dann inaktiv wird: Sensor/Kabel pruefen.
4. Wenn DI aktiv bleibt: Boardeingang oder Klemmenverdrahtung pruefen.
5. Sensor testweise gegen einen anderen Kanal tauschen.

### 11.4 Sensor reagiert gar nicht

Moegliche Ursachen:

- Keine 24V-Versorgung am Sensor.
- Blau/GND fehlt.
- Schwarz/Signal auf falschem DI.
- Sensorabstand oder Ausrichtung falsch.

Pruefen:

1. Zwischen Braun und Blau ca. 24 V messen.
2. Sensor ausloesen und Schwarz gegen Blau messen.
3. Am Webinterface den richtigen DI beobachten.
4. Sensor testweise an DI1 anschliessen.

Reparatur:

- Sensor neu ausrichten.
- Kabel ersetzen.
- Sensor ersetzen.
- DI-Klemme korrigieren.

### 11.5 Licht schaltet nicht

Moegliche Ursachen:

- DO1 nicht aktiv.
- 12V-Lastversorgung fehlt.
- Relais falsch verdrahtet.
- Lichtkreis/Sicherung defekt.

Pruefen:

1. In der Webseite Licht einschalten.
2. Spannung an Relaisspule messen.
3. Relais-Klicken pruefen.
4. Relaiskontakt und Lichtversorgung messen.

Reparatur:

- 12V-Sicherung oder DC/DC-Wandler pruefen.
- Relais ersetzen.
- Lichtleitung oder Lampe pruefen.

### 11.6 Pneumatikventil schaltet nicht

Moegliche Ursachen:

- DO8, DO7, DO6 oder DO5 falsch zugeordnet.
- Keine 12V-Versorgung.
- Ventilspule defekt.
- Pneumatikdruck fehlt.
- Ausgang ueberlastet.

Pruefen:

1. Ventil in der Webseite schalten.
2. Spannung an Ventilspule messen.
3. Ventil auf Schaltgeraeusch pruefen.
4. Pneumatikdruck pruefen.
5. Ventil testweise direkt kurz mit 12 V pruefen, nur wenn sicher moeglich.

Reparatur:

- Ventil ersetzen.
- Steckverbinder oder Kabel reparieren.
- Bei zu hoher Stromaufnahme Relais/MOSFET-Treiber zwischen DO und Ventil setzen.

### 11.7 GNSS hat keinen Fix

Moegliche Ursachen:

- Antenne ohne freie Sicht.
- Modul nicht versorgt.
- RS485 A/B vertauscht.
- Falsche Baudrate oder Adresse.

Pruefen:

1. GNSS im Freien testen.
2. Versorgung des GNSS-Moduls messen.
3. GND des GNSS mit Board-GND pruefen.
4. RS485-Diagnose in der Webseite starten.
5. A/B testweise tauschen.

Reparatur:

- Antenne besser positionieren.
- RS485-Leitung verdrillt und kurz halten.
- Abschlusswiderstand nur an Busenden einsetzen.
- Modulparameter pruefen.

### 11.8 Kamera zeigt kein Bild

Moegliche Ursachen:

- Kamera nicht am LAN-Port erreichbar.
- Falsche IP.
- Falsches Passwort.
- Substream nicht MJPEG.
- Kamera bekommt keine Versorgung.

Pruefen:

1. Kamera-IP in Einstellungen kontrollieren.
2. Verbindungstest ausfuehren.
3. Substream 102 auf MJPEG stellen.
4. LAN-Kabel und Kamera-Versorgung pruefen.

Reparatur:

- Kamera auf `192.168.4.20` oder eingestellte IP konfigurieren.
- Passwort neu eintragen.
- Kamera neu starten.
- LAN-Kabel ersetzen.

### 11.9 Ausgaenge funktionieren alle nicht

Moegliche Ursachen:

- TCA9554-Expander nicht erreichbar.
- I2C-Leitung gestoert.
- DO COM/GND fehlt.
- Board defekt.

Pruefen:

1. Webstatus: `light_switchable` bzw. Ausgang schaltbar?
2. Seriellen Monitor auf Warnung `TCA9554 DO expander nicht erreichbar` pruefen.
3. DO COM und DO GND messen.
4. Board neu starten.

Reparatur:

- Klemmen und Versorgung pruefen.
- Board ersetzen, wenn I2C-Expander dauerhaft nicht erreichbar ist.

## 12. Firmware neu flashen

Voraussetzungen:

- VS Code mit PlatformIO oder PlatformIO CLI.
- USB-C-Datenkabel.
- Projektordner `waveshare_esp32_s3_poe_8di8do_Drillmaschine`.

Build:

```bash
platformio run -e waveshare_esp32_s3_poe_8di8do
```

Upload:

```bash
platformio run -e waveshare_esp32_s3_poe_8di8do --target upload
```

Monitor:

```bash
platformio device monitor
```

Nach dem Flashen im seriellen Monitor pruefen:

- Firmware-Version.
- SSID.
- IP-Adresse `192.168.4.1`.
- Warnungen zu TCA9554, GNSS oder Ethernet.

## 13. Ersatzteil- und Reparaturstrategie

Empfohlene Ersatzteile auf der Maschine:

- 1 bis 2 E3F-DS30C4 Sensoren.
- 1 Lichtrelais.
- 1 Pneumatikventil 3V210-08 DC12V.
- Sicherungen fuer 24V und 12V.
- Ersatzstecker und Aderendhuelsen.
- kurzes LAN-Kabel fuer Kamera.
- USB-C-Datenkabel.

Schnelle Eingrenzung:

| Fehler | Schneller Test |
|--------|----------------|
| Sensor unklar | Sensor von DI1 auf DI2 tauschen |
| Kabel unklar | DI kurz gegen GND bruecken |
| Ausgang unklar | DO mit kleiner Testlast pruefen |
| Ventil unklar | Ventilspule ohmisch messen |
| Kamera unklar | Verbindungstest im Webinterface |
| GNSS unklar | RS485-Scan und A/B tauschen |

## 14. Sicherheitshinweise

- Vor Arbeiten an der Verdrahtung Maschine spannungsfrei schalten.
- Lastkreise absichern.
- Keine 24 V direkt auf ESP32-GPIOs fuehren.
- Sensor- und Last-GND gemeinsam, aber sauber sternfoermig fuehren.
- Magnetventile und Relais gegen Spannungsspitzen schuetzen.
- Leitungen gegen Scheuern, Zug und Vibration sichern.
- Elektronik gegen Feuchtigkeit und Staub schuetzen.

## 15. Dokumentierte Projektdateien

| Datei | Zweck |
|-------|------|
| `README.md` | technische Projektuebersicht |
| `RELEASE_NOTES.md` | Versionshistorie |
| `src/main.cpp` | Firmware |
| `docs/di_do_schematic.html` | visueller DI/DO-Schaltplan |
| `docs/betriebshandbuch.md` | dieses Betriebshandbuch |

## 16. Kurzcheck vor Einsatz

1. Board startet und WLAN `Drillmaschine-M01` ist sichtbar.
2. Webseite `http://192.168.4.1/` laedt.
3. Alle sechs Sensoren reagieren einzeln.
4. Hubwerksignal an DI8 reagiert.
5. Licht ueber DO1 schaltet.
6. Pneumatikventile DO8, DO7, DO6 und DO5 schalten.
7. GNSS zeigt Fix oder plausiblen Diagnosezustand.
8. Kamera-Verbindungstest ist erfolgreich.
9. Alarmton ist am Tablet aktiviert.
10. Kurze Testfahrt wurde aufgezeichnet und exportiert.
