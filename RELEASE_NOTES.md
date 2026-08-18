# Release Notes

## 2.8.0 - 2026-08-18

### Sensorauswertung

- Umschaltbare Prozentanzeige zwischen bisheriger Impulsdauer und Impulsen pro Sekunde.
- Einstellbarer Referenzwert, wie viele Impulse pro Sekunde 100 % entsprechen.
- Separat einstellbares Messfenster von 100-3000 ms mit Empfehlungen für feine, normale und grobe Saat direkt in der Einstellungsmaske.
- Die einstellbare Dauersignalzeit bleibt unabhängig davon für den Verstopfungsalarm erhalten; 100 % Impulsrate allein lösen keinen Alarm aus.
- Kurze Sensorimpulse werden per Hardware-Interrupt erfasst und dadurch auch während GNSS- oder Webverarbeitung zuverlässig gezählt.
- Sensorberechnung und Webanzeige wurden für eine deutlich schnellere Reaktion beschleunigt.

### Automatische Druckluft

- Frei auswählbare Sensorgruppen 1-6, 7-12, 13-18 und 19-24 für das automatische Durchblasen.
- Nicht ausgewählte Gruppen werden in der automatischen Sequenz übersprungen; die manuelle Ventilsteuerung bleibt vollständig verfügbar.
- Gruppenauswahl und Sensoreinstellungen werden dauerhaft gespeichert.

## 2.7.0 - 2026-08-15

### Automatische Druckluft

- Neue Einstellung "Automatische Druckluft": l&ouml;st in einem einstellbaren Intervall (1-60 Minuten, Standard 15 Minuten) alle 4 Pneumatikventile nacheinander aus, um die Sensoren freizublasen.
- Die Automatisation ist standardm&auml;&szlig;ig deaktiviert und kann in den Einstellungen ein-/ausgeschaltet werden.
- Der Timer l&auml;uft nur w&auml;hrend aktiver Fahrtaufzeichnung und Hubwerk unten (echte Feldarbeit). Bei ausgehobenem Hubwerk oder gestoppter Aufzeichnung bleibt der erreichte Wert eingefroren und l&auml;uft beim n&auml;chsten Mal an derselben Stelle weiter, statt auf 0 zur&uuml;ckzuspringen.
- Neuer API-Endpunkt `POST /api/auto-air` mit `{"enabled": bool, "interval_ms": number}`.

## 2.6.5 - 2026-08-15

### Fahrtaufzeichnung

- Verlassen-Warnung bei aktiver Aufzeichnung ergaenzt: Browser-Zurueck (inkl. Zurueck-Geste auf Tablet/Smartphone) zeigt jetzt einen Bestaetigungsdialog ("Auf der Seite bleiben" / "Trotzdem verlassen"), bevor die Live-Ansicht verlassen wird.
- Zusaetzlich native Browser-Warnung beim Schliessen des Tabs, Neuladen oder Eingabe einer anderen URL (beforeunload), sofern der jeweilige Browser das unterstuetzt.
- Klarstellung: Die Aufzeichnung selbst laeuft weiterhin auf dem Geraet, auch wenn die Seite verlassen wird - die Warnung verhindert nur den versehentlichen Verlust der Live-Ansicht (Karte, Kamera, Status).

## 2.6.4 - 2026-08-15

### Einstellungen

- Erklaerungstext im Bereich "Empfindlichkeit" ergaenzt: erklaert, dass der Wert die Dauer bis zum Dauersignal-Alarm regelt (nicht die Sensorempfindlichkeit selbst) und gibt eine Faustregel fuer feine vs. grobe Saat.

## 2.6.3 - 2026-08-15

### Kanaluebersicht

- Traege Signalqualitaetsanzeige behoben: Jeder noch so kurze Aussetzer am Digitaleingang setzte die Rampenbasis (`activeSinceMs`) zurueck, wodurch die Prozentanzeige nach einem kurzen Wackler nahezu wieder bei 0 % begann, statt an der letzten Position weiterzuwachsen.
- Kurze Aussetzer bis 2 Sekunden (`SIGNAL_GAP_GRACE_MS`) unterbrechen die Rampe jetzt nicht mehr; die Anzeige waechst nach einem kurzen Wackler ohne Ruecksprung weiter.
- Nach einem echten, laengeren Signalausfall (weiterhin `SIGNAL_READY_TIMEOUT_MS` = 10 s) werden Anzeige und Rampenbasis vollstaendig auf 0 zurueckgesetzt, sodass die Anzeige nicht mehr dauerhaft auf einem alten Wert haengen bleiben kann.

## 2.6.2 - 2026-08-15

### Kanaluebersicht

- Alarm (roter Rahmen, Alarmton, `main_signal`/`latched_alarm`) wurde bisher erst nach einer fest verdrahteten Haltezeit von 15 Sekunden ausgeloest, waehrend die Prozentanzeige bereits nach der einstellbaren Haltezeit (maximal 10 Sekunden) 100 % erreichte. Dadurch blieb die Anzeige bei 100 %, ohne dass Alarm oder roter Rahmen ausgeloest wurden.
- Der Alarmzustand wird jetzt anhand derselben einstellbaren Haltezeit (`mainSignalHoldMs`) ausgeloest wie die Prozentanzeige, sodass roter Rahmen und Alarmton exakt beim Erreichen von 100 % erscheinen. Die feste Konstante `RED_SIGNAL_HOLD_MS` wurde entfernt.

## 2.6.1 - 2026-08-07

### Kanaluebersicht

- Fehlerhafte Signalqualitaetsanzeige korrigiert: Kurze Sensorimpulse verschwanden sofort wieder aus der Prozentanzeige, da der Wert beim Abfallen des Eingangs unmittelbar auf `0` zurueckgesetzt wurde.
- Der zuletzt erreichte Qualitaetswert bleibt jetzt bis zu 10 Sekunden nach der letzten Detektion sichtbar (`SIGNAL_READY_TIMEOUT_MS`), analog zum Verhalten vor der letzten Ueberarbeitung der Kanaluebersicht.
- Saateingabe im Dashboard von einem reinen Dropdown auf ein Freitextfeld mit Vorschlagsliste (`datalist`) umgestellt; bestehende Saatvorschlaege bleiben als Auswahlhilfe erhalten, freie Eingabe (bis 23 Zeichen) ist zusaetzlich moeglich.

## 2.6.0 - 2026-08-01

### Release

- Aktuellen Entwicklungsstand als produktive Version freigegeben.

## 2.5.0 - 2026-07-14

### Digitalausgaenge

- TCA9554-Hardwarezuordnung korrigiert: `DO1` entspricht Bit 0 bis `DO8` entspricht Bit 7. Die zuvor irrtuemlich umgekehrte Bitreihenfolge wurde entfernt.
- Ausgangsbelegung festgelegt: `DO1` Licht, `DO2/DO3` Reserve, `DO4` Luefter, `DO5` Ventil 4, `DO6` Ventil 3, `DO7` Ventil 2 und `DO8` Ventil 1.
- Alle belegten Ausgaenge verwenden Active-Low-Logik und werden beim Start ausgeschaltet initialisiert.
- Automatische Sensorspiegelung auf `DO2` und `DO3` deaktiviert; beide Reserveausgaenge bleiben aus.

### Lueftersteuerung

- Temperaturgesteuerten Luefter auf `DO4` ergaenzt.
- Luefter schaltet bei einer ESP-Temperatur ueber `43 °C` ein und bei `41 °C` oder weniger wieder aus.
- Temperatur wird alle zwei Sekunden geprueft; die Hysterese verhindert schnelles Umschalten.
- Manuelle Luefterbedienung entfernt und durch die reine Statusanzeige `Luefter an` beziehungsweise `Luefter aus` ersetzt.
- Statusanzeige optisch an die Navigationsbuttons angepasst.

### Fahrtaufzeichnung und Hubwerk

- Nach einem manuellen Stopp verriegelt die Firmware den automatischen Neustart.
- Ein dauerhaftes Signal `Hubwerk unten` und ein zwischenzeitlich veralteter GNSS-Fix koennen diese Verriegelung nicht aufheben.
- Auto-Start wird erst wieder freigegeben, nachdem mindestens einmal `Hubwerk oben` erkannt wurde; das anschliessende Absenken darf eine neue Fahrt starten.
- Bei manuellem Fahrtende erscheint immer ein Abschlussdialog mit Fahrt-ID, erneutem Upload und Downloads.
- Bei automatisch beendeter Fahrt und fehlgeschlagenem Portal-Upload erscheint ein Hinweis auf die fehlende Portalverbindung mit Download-Angebot.

### Portal- und API-Upload

- Uploadarchitektur auf das vereinbarte Vermittlermodell umgestellt: Der ESP32 stellt die Fahrtdateien lokal bereit, der Browser auf Tablet oder Mobiltelefon uebermittelt sie ueber dessen Internetverbindung an das Portal.
- Direkter automatischer Internet-Upload des ESP32 nach Fahrtende deaktiviert.
- Browser erstellt ein Portal-kompatibles `multipart/form-data` mit `trip_id`, `device_id`, Metadaten, GPS-CSV, Hauptsignal-CSV und Sensorereignis-CSV.
- Automatischer Browser-Upload nach Fahrtende sowie manueller Upload und Upload aus dem Abschlussdialog ergaenzt.
- Sensorarchiv um die Spalte `duration_s` erweitert.
- API-Einrichtung um `Verbindung testen` ergaenzt. Der Test prueft URL-Format, Portal-Erreichbarkeit und Bearer-Token, ohne eine Fahrt hochzuladen.
- Portal-URL muss mit `/api/trips/upload` enden. Fuer den automatischen Upload muss die Bedienseite auf dem Mobilgeraet geoeffnet bleiben.

### Bedienung und Zugriff

- Lokale Anmeldepflicht der ESP32-Weboberflaeche deaktiviert; das Anmeldefenster wird nicht mehr eingeblendet.
- Vorhandene Benutzer-/Sessionimplementierung ist weiterhin im Quellcode enthalten, wird fuer lokale Zugriffe aber umgangen.
- Saat und Feld koennen direkt auf dem Dashboard ausgewaehlt werden. Die Saat steht als Dropdown mit erweiterten Standardkulturen bereit.
- Saat-Vorschlaege koennen im Adminbereich hinzugefuegt und entfernt werden.
- Den Adminbereich in einklappbare Abschnitte fuer Benutzer, Kameras, Kanaele, System, Maschinenparameter, GNSS, Saat, Cloud-Upload und Dateien gegliedert.

### Validierung

- Firmware nach jeder Aenderungsgruppe erfolgreich mit PlatformIO gebaut und auf das angeschlossene ESP32-S3-Board uebertragen.
- Physische Digitalausgangszuordnung nach Wiederherstellung der direkten TCA9554-Bitreihenfolge am Board bestaetigt.

## 2.1.0 - 2026-06-12

### Neu

- Bis zu vier Hikvision-Kameras koennen konfiguriert und auf der Mainpage angezeigt werden.
- Nicht konfigurierte Kameras werden automatisch ausgeblendet.
- Kameraeinstellungen besitzen je Kamera IP-Adresse, Benutzer, Passwort und Verbindungstest.
- Pneumatikventile auf `DO2..DO5` sind auf der Mainpage als `Sensor 1-6`, `Sensor 7-12`, `Sensor 13-18` und `Sensor 19-24` schaltbar.
- Einstellungen zeigen eine Systemauslastung mit Heap, PSRAM, LittleFS, Sketch-Flash, Uptime, Reset-Grund und Ethernet-Status.
- Kartenansicht zeigt GNSS-Signalqualitaet, Position, Genauigkeit/Satelliten und RS485-Status.
- RS485-/GNSS-Diagnose mit Live-Test, Baudscan, Modbus-Adressscan, Registerscan sowie ASCII- und HEX-Rohdatenvorschau.

### Geaendert

- Ebyte EWD108-GN05(485) wird jetzt anhand der Herstellerregister gelesen: RMC ab Holding-Register `0x0005` ueber `35` Register.
- NMEA-RMC-Parser akzeptiert kleine Status- und Richtungsbuchstaben aus dem Ebyte-RMC-Datensatz.
- NMEA-Koordinatenumrechnung korrigiert, damit Werte wie `3046.26769,n` korrekt in Dezimalgrad gewandelt werden.
- Pneumatikventile `DO2..DO5` werden nicht mehr automatisch durch Hauptsignale gespiegelt.
- Kamerastreams starten weiterhin mit Substream; Mainstream kann je Kamera umgeschaltet werden.

### Dokumentation

- GNSS-Verdrahtung dokumentiert: GPS `A` auf Waveshare `A`, GPS `B` auf Waveshare `B`.
- Externe GPS-Versorgung dokumentiert: `12V - / GPS GND` muss mit `GND` des Waveshare verbunden sein.
- Kamera-Proxy-Endpunkte fuer Kamera 1 bis 4 dokumentiert.

## 2.0.0 - 2026-06-07

### Neu

- Hikvision-Kameraeinbindung mit Mainstream-/Substream-Umschaltung ergänzt.
- W5500-Ethernet-Port initialisiert; Kamera wird über den LAN-Port angesprochen.
- Kamera-Stream-Proxy ergänzt, damit das Webinterface den LAN-Kamerastream über den ESP32 anzeigen kann.
- Kamera-Verbindungstest in den Einstellungen ergänzt.
- Kamera-IP, Benutzer und Passwort sind im Webinterface einstellbar und werden persistent gespeichert.
- Tablet-/Smartphone-Ton kann direkt auf der Mainpage aktiviert werden.

### Geändert

- Kamera-Livebild pausiert die zyklischen API-Abfragen, damit der ESP32-Webserver nicht durch parallele Requests blockiert wird.
- Bei aktivem Kamerabild zeigt der Kontaktstatus `Kamera aktiv` mit grün blinkendem Punkt.
- Der Kamera-Test prüft nur noch die relevanten Ethernet-HTTP-Verbindungen.

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
