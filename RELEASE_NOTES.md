# Release Notes

## 1.5.0 - 2026-05-31

### Neu

- Fahrtaufzeichnung startet automatisch, sobald der GNSS-Fix drei Sekunden stabil ist.
- Jede Fahrt erhält eine Fahrt-ID im Format `M01-Bxxxxxx-Fxxxx`.
- Feldname ist neben der Saat im Webinterface editierbar.
- GPS- und Sensorlogs werden als Fahrtdateien dauerhaft in LittleFS archiviert.
- Aktive Sensorereignisse werden beim Stoppen der Aufzeichnung sauber abgeschlossen.
- Kombinierter GeoJSON-Download `/api/combined.geojson` ergänzt.
- Archivierte Dateien können über das Webinterface geladen werden.
- GNSS-Zustände `no_rs485`, `no_fix` und `invalid_data` werden sichtbar unterschieden.
- GNSS-Warnungen werden nach Aktivierung des Tablet-Tons akustisch signalisiert.
- Kartenansicht besitzt einen Schalter `Position folgen`.
- Modulübersicht für `M01` bis `M04` vorbereitet.
- Task-Watchdog, Boot-Zähler und Reset-Grund ergänzt.

### Geändert

- Seltene Downloads und Wartungsaktionen befinden sich in einem ausklappbaren Bereich.
- Löschen des Live-Logs erfordert eine Bestätigung und ist während einer Fahrt gesperrt.
- GPS-Punkte werden für den Flash gepuffert und spätestens alle 30 Sekunden geschrieben.

### Bekannte Einschränkungen

- `M02` bis `M04` sind als spätere Ausbaustufe sichtbar, aber noch nicht über ein zentrales Gateway angebunden.
- Die EWD108-Modbus-Registerbelegung muss weiterhin am realen GNSS-Modul bestätigt werden.

## 1.4.0 - 2026-05-31

### Neu

- Reiter `Überwachung` und `Karte` im Webinterface ergänzt.
- Topografische Online-Karte mit OpenTopoMap im Karten-Reiter ergänzt.
- Fahrspur wird bei fehlendem Internet weiterhin auf einem lokalen Raster dargestellt.
- Aktuelle GNSS-Position wird als grüner Punkt angezeigt.
- Erkannte Hauptsignal-Störungen werden als rote Marker dargestellt.
- Neuer Endpoint `/api/track` für eine kompakte Live-Ansicht.
- Live-Ansicht ist auf 1.200 Spurpunkte und 128 Störungsmarker begrenzt; vollständige Downloads bleiben erhalten.

## 1.3.1 - 2026-05-25

### Neu

- Sensorlog fuer abgeschlossene Hauptsignal-Ausloesungen ergänzt.
- Sensorlog enthaelt Kanal, Kanalname, Saat, Startzeit, Endzeit, Dauer und GPS-Bezug.
- Neue Downloads: `/api/sensor-events.csv` und `/api/sensor-events.txt`.

### Geändert

- WLAN-SSID auf `Drillmaschine-M01` umbenannt.
- `M01` steht fuer `Modul 01`, damit spaeter mehrere Module fortlaufend benannt werden koennen.
- Webserver wieder von HTTPS auf HTTP umgestellt.
- Grund: Browser-GPS wurde entfernt, daher ist HTTPS nicht mehr notwendig.
- HTTP ist auf dem ESP32-S3 stabiler bei Seiten-Refresh und zyklischen API-Abfragen.
- API-Adresse ist wieder `http://192.168.4.1/api/status`.

## 1.3.0 - 2026-05-25

### Neu

- Ebyte EWD108-GN05(485) als GNSS-Quelle vorbereitet.
- RS485/Modbus-RTU-Abfrage direkt im ESP32 ergänzt.
- Fahrtaufzeichnung startet/stoppt jetzt lokal im ESP32; Tablet-/Smartphone-GPS wird nicht mehr verwendet.
- GNSS-Status im Webinterface ergänzt:
  - Fix ja/nein
  - Quelle
  - aktuelle Position
  - Genauigkeit
  - Satelliten
  - RS485-Status und Fehlerzähler
- `/api/status` liefert einen `gnss`-Block mit Modbus-/Fix-/Positionsdaten.
- Neuer Endpoint `/api/recording` zum Starten/Stoppen der Fahrtaufzeichnung.

### Entfernt

- Browser-Geolocation über Tablet/Smartphone.
- `/api/gps-log` als Browser-GPS-Eingang.

### Bekannte Einschränkungen

- Die Modbus-Registerbelegung des echten EWD108-GN05(485) muss noch am Modul verifiziert werden.
- Die Firmware scannt aktuell Holding-Register ab `0x0000` nach NMEA RMC/GGA.
- Falls das Modul NMEA in einem anderen Registerbereich bereitstellt, müssen `GNSS_SCAN_START_REGISTER` und `GNSS_SCAN_REGISTER_COUNT` angepasst werden.

## 1.2.0 - 2026-05-25

### Neu

- Webseite in `Drillmaschinenüberwachung` umbenannt.
- HTTPS-Webserver mit Self-Signed-Zertifikat ergänzt.
- Firmware-Version wird auf der Webseite angezeigt.
- ESP32-Temperatur wird auf der Webseite und in `/api/status` angezeigt.
- Tablet-/Smartphone-Alarmton für Hauptsignal-Störungen ergänzt.
- Alarmton-Quittierung ergänzt:
  - aktuelle Störungen können stummgeschaltet werden
  - neue Störungen lösen den Alarm erneut aus
  - Quittierung wird automatisch zurückgesetzt, wenn alle Kanäle wieder `OK` sind
- Fahrtaufzeichnung mit GPS-Punkten vom Tablet/Smartphone ergänzt.
- Saatgut/Feldfrucht im Webinterface editierbar.
- Hauptsignal-Log mit GPS-Positionen ergänzt, soweit eine GPS-Position verfügbar ist.
- CSV- und GeoJSON-Downloads für Fahrtaufzeichnung und Hauptsignal-Ereignisse ergänzt.
- Verbindungsstatus im Webinterface ergänzt.

### Geändert

- Kanalübersicht zeigt vorrangig Kanalname und Hauptstatus.
- Detailinformationen je Kanal sind ausklappbar.
- Eingangssignale sind für die Waveshare-DI-Eingänge aktiv-low konfiguriert.
- Hauptsignal wird erst nach dauerhaft aktivem Eingang erkannt.
- Web- und API-Adresse ist jetzt `https://192.168.4.1/`.

### Bekannte Einschränkungen

- GPS/Fahrtaufzeichnung ist noch nicht zu 100% stabil.
- Auf manchen Tablets oder Browsern können Fetch-/400-Fehler auftreten.
- GeoJSON-Dateien können leer bleiben, wenn der Browser keine Punkte erfolgreich an den ESP32 überträgt.
- Die Kanalüberwachung funktioniert unabhängig von GPS weiter.

## 0.3.0

- Erste Weboberfläche für Waveshare ESP32-S3-POE-ETH-8DI-8DO.
- 8 digitale Eingänge und 8 digitale Ausgänge.
- Kanalnamen im Webinterface bearbeitbar.
- REST-API `/api/status`.
