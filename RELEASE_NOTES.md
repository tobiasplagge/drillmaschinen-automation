# Release Notes

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
