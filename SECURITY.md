# Security & Compliance Assessment

Stand: 2026-07-08

Dieses Dokument ist eine technische Einschätzung des aktuellen Stands.
Es ist keine Rechtsberatung und ersetzt keine Prüfung durch Datenschutz-,
Steuer- oder Wirtschaftsprüfende.

## Kurzfazit

Der aktuelle Stand ist **nicht abschließend DSGVO- oder GoBD-konform
nachweisbar**.

Es gibt sinnvolle technische Bausteine, aber für eine belastbare Konformität
fehlen noch mehrere Pflicht- und Nachweisbausteine:

- fehlende oder unvollständige Authentifizierung und Rechteverwaltung
- keine belastbar dokumentierte Datenklassifikation
- keine vollständige Lösch-, Aufbewahrungs- und Archivierungslogik
- keine nachweisbar revisionssichere Ablage für steuerlich relevante Daten
- keine dokumentierte Verfahrensdokumentation
- externe Browser-Abhängigkeiten im Webfrontend

## Positive Ansätze

- lokale Ausführung ohne Cloud-Zwang
- Exportfunktionen für CSV, TXT und GeoJSON
- System-Event-Log vorhanden
- Firmware speichert einige Einstellungen lokal
- Kamera-Passwörter werden nicht im Status-JSON ausgegeben

Diese Punkte sind hilfreich, reichen aber allein nicht für DSGVO- oder
GoBD-Konformität.

## DSGVO-Bewertung

### Bereits teilweise erfüllt

- Datenminimierung durch lokale Verarbeitung und begrenzte Speicherung
- technische Grundfunktionen zur Trennung von Bedienoberfläche und Daten
- Teile der Daten sind exportierbar

### Kritische Lücken

1. **Zugriffsschutz**
   - Die HTTP-Endpunkte sind im aktuellen Stand ohne echte Anmeldung und ohne
     Rollenprüfung registriert.
   - Für produktive Nutzung braucht es Authentifizierung, Rollen und
     saubere Berechtigungen.

2. **Datenschutz durch Technikgestaltung**
   - DSGVO Art. 25 verlangt geeignete technische und organisatorische
     Maßnahmen.
   - Dafür fehlen derzeit insbesondere:
     - feingranulare Berechtigungen
     - standardmäßige Minimierung
     - Schutz vor unberechtigtem Export oder Löschen
     - saubere Trennung zwischen Admin- und Benutzerfunktionen

3. **Sicherheit der Verarbeitung**
   - Für Art. 32 DSGVO sind geeignete Sicherheitsmaßnahmen nötig.
   - Der aktuelle Stand zeigt:
     - kein belastbar erkennbares Login-Konzept
     - kein gesichertes Session-Konzept
     - kein verpflichtendes HTTPS im lokalen Betrieb
     - keine dokumentierte Verschlüsselung ruhender Daten

4. **Externe Dienste**
   - Die Kartenansicht lädt aktuell externe Ressourcen nach.
   - Das ist für Datenschutz und Offline-Betrieb ungünstig und sollte lokal
     gehostet werden.

5. **Betroffenenrechte**
   - Es fehlen Prozesse und UI-Funktionen für:
     - Auskunft
     - Berichtigung
     - Löschung
     - Datenportabilität
     - Einschränkung/Widerspruch

6. **Dokumentation**
   - Es fehlen Verzeichnis der Verarbeitungstätigkeiten, TOM-Dokumentation,
     AVV-Übersicht, Löschkonzept und ggf. DSFA.

## GoBD-Bewertung

### Bereits teilweise erfüllt

- strukturierte Exporte für Betriebsdaten
- systemseitige Logdateien
- fortlaufende Fahrt-/Trip-Archivierung

### Kritische Lücken

1. **Unveränderbarkeit**
   - GoBD verlangt, dass Buchungen und Aufzeichnungen vollständig, richtig,
     zeitgerecht und geordnet sind.
   - Im aktuellen Stand werden Archivdateien bei Neuerstellung gelöscht und
     neu geschrieben.
   - Das ist für revisionssichere Archivierung kritisch.

2. **Aufbewahrung**
   - Für aufbewahrungspflichtige Daten braucht es ein klares
     Aufbewahrungs- und Löschkonzept.
   - Das ist derzeit nicht dokumentiert und technisch nicht abgesichert.

3. **Nachvollziehbarkeit**
   - Es fehlt ein manipulationsarmes Audit- und Änderungsprotokoll für
     fachlich relevante Daten.
   - Ein normales Logfile reicht dafür in der Regel nicht.

4. **Verfahrensdokumentation**
   - Für GoBD ist eine vollständige Verfahrensdokumentation zentral.
   - Dazu gehören u. a.:
     - Datenfluss
     - Verantwortlichkeiten
     - Backup/Restore
     - Zugriffsrechte
     - Archivierung
     - Datenüberlassung für Prüfungen

5. **Datenüberlassung**
   - Finanzbehörden müssen Daten in maschinell auswertbarer Form erhalten
     können.
   - CSV/GeoJSON allein decken das nur teilweise ab, wenn steuerlich relevante
     Daten betroffen sind.

## Priorisierte Maßnahmen

### P1 - Sofort wichtig

1. Authentifizierung und Rollenmodell einführen
2. Rechte pro Modul, Aktion und Datensatz erzwingen
3. HTTPS verpflichtend machen
4. Session-Schutz mit Ablauf, Gerätebindung und Server-Logout ergänzen
5. Externe Frontend-Abhängigkeiten lokal hosten
6. Audit-Log für fachlich relevante Änderungen einführen

### P2 - Für Compliance notwendig

1. Lösch- und Aufbewahrungskonzept definieren
2. DSGVO-Dokumentation anlegen:
   - Verarbeitungstätigkeiten
   - TOM
   - AVV-Übersicht
   - Betroffenenrechte
3. GoBD-Verfahrensdokumentation anlegen
4. Archivierung gegen nachträgliche Änderungen absichern
5. Exportformat für Prüfungen definieren

### P3 - Qualität und Nachweis

1. Datenschutzhinweise im System ergänzen
2. Admin-Protokoll mit Zeit, Nutzer, Aktion und Objekt
3. Backup/Restore mit Prüfsummen und Versionsständen
4. Regelmäßige Restore-Tests dokumentieren
5. Datenklassifikation je Modul festlegen

## Technische Hinweise aus dem aktuellen Code

- Es existieren Download-Endpunkte für Log- und Exportdaten.
- Es gibt ein System-Event-Log in LittleFS.
- Trip-Dateien werden bei Neuerstellung gelöscht und neu angelegt.
- Die Kartenansicht lädt externe Skripte und Kartenkacheln.
- Der Webserver läuft aktuell mit offen erreichbaren Endpunkten.

## Relevante Rechtsquellen

- DSGVO, Art. 5, 25, 28, 30, 32:
  https://eur-lex.europa.eu/eli/reg/2016/679/oj/eng
- BfDI: DSGVO / BDSG Überblick:
  https://www.bfdi.bund.de/SharedDocs/Downloads/DE/Broschueren/INFO1.pdf
- BfDI: Datenschutz durch Technikgestaltung:
  https://www.bfdi.bund.de/DE/Fachthemen/Inhalte/Technik/SdT.html
- GoBD, BMF:
  https://www.bundesfinanzministerium.de/Content/DE/Downloads/BMF_Schreiben/Weitere_Steuerthemen/Abgabenordnung/AO-Anwendungserlass/2024-03-11-aenderung-gobd.html
- § 146 AO:
  https://www.gesetze-im-internet.de/ao_1977/__146.html
- § 147 AO:
  https://www.gesetze-im-internet.de/ao_1977/__147.html

## Nächster sinnvoller Schritt

Als nächstes sollte ein konkreter Umsetzungsplan für:

1. Login, Rollen und Session-Schutz
2. Audit-Log und Archivierung
3. lokale Abhängigkeiten statt externer CDN-/Kartenaufrufe
4. Verfahrensdokumentation und Löschkonzept

erstellt werden.
