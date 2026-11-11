# 🦆 QuackSquad – Projektdokumentation: Duckiebot Challenge

[![LaTeX](https://img.shields.io/badge/Document-LaTeX_%2F_KOMA--Script-blue.svg)](https://www.latex-project.org/)
[![BibLaTeX](https://img.shields.io/badge/Bibliography-BibLaTeX_%2F_Biber-success.svg)](https://ctan.org/pkg/biblatex)
[![Institution](https://img.shields.io/badge/Hochschule-Karlsruhe_(HKA)-3a68aa.svg)](https://www.h-ka.de/)
[![Faculty](https://img.shields.io/badge/Fakultät-Maschinenbau_und_Mechatronik-3a68aa.svg)](https://www.h-ka.de/die-hochschule-karlsruhe/fakultaeten/maschinenbau-und-mechatronik)
[![Course](https://img.shields.io/badge/Fach-Robogistics-orange.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Duckietown_DB21J-yellow.svg)](https://www.duckietown.com/)
[![ROS](https://img.shields.io/badge/Middleware-ROS_1-red.svg)](https://www.ros.org/)
[![Repository](https://img.shields.io/badge/Code_Repo-Paulchenn%2FQuackSquad-lightgrey.svg?logo=github)](https://github.com/Paulchenn/QuackSquad.git)

Dieses Repository beinhaltet die vollständige, wissenschaftliche Projektdokumentation (LaTeX-Satz) des Teams **QuackSquad** für die **Duckiebot Challenge** im Rahmen der Master-Lehrveranstaltung **„Robogistics“** an der **Hochschule Karlsruhe (HKA)**.

---

## 📑 Inhaltsverzeichnis

- [Über das Projekt](#-über-das-projekt)
  - [Hintergrund und Zielsetzung](#hintergrund-und-zielsetzung)
  - [Hardware & Software-Plattform](#hardware--software-plattform)
  - [Code-Repository](#code-repository)
- [Herausforderungen & Systemmodule](#-herausforderungen--systemmodule)
  - [Challenge 1: Spurverfolgung (Lane Following)](#challenge-1-spurverfolgung-lane-following)
  - [Challenge 2: Kreuzungsmanagement & Vorfahrt](#challenge-2-kreuzungsmanagement--vorfahrt)
  - [Deep Learning: Objekterkennung (YOLOv8)](#deep-learning-objekterkennung-yolov8)
  - [Challenge 3: Hinderniserkennung & Ausweichen (Duckie Detection)](#challenge-3-hinderniserkennung--ausweichen-duckie-detection)
  - [Challenge 4: Autonomes Parken (Autonomous Parking)](#challenge-4-autonomes-parken-autonomous-parking)
  - [Challenge 5: Kollisionsvermeidung & Mehrbot-System](#challenge-5-kollisionsvermeidung--mehrbot-system)
  - [Entwicklungs- und Kalibrierungstools](#entwicklungs--und-kalibrierungstools)
- [Repository- und Dateistruktur](#-repository--und-dateistruktur)
- [Kompilierung & Bauanleitung](#-kompilierung--bauanleitung)
  - [Voraussetzungen](#voraussetzungen)
  - [Kompilierung via CLI](#kompilierung-via-cli)
  - [Kompilierung in IDEs](#kompilierung-in-ides)
  - [Bereinigen von Build-Artefakten](#bereinigen-von-build-artefakten)
- [Autoren & Betreuung](#-autoren--betreuung)

---

## 📖 Über das Projekt

### Hintergrund und Zielsetzung
Im Modul **Robogistics** entwickeln Studierende autonome Funktionen für mobile Miniaturroboter in der städtischen Modellumgebung **Duckietown**. Die Herausforderungen bilden typische Szenarien des realen autonomen Fahrens ab – von der Spurerkennung über komplexes Kreuzungsverhalten und Objektausweichen bis hin zum koordinierten Fahrbetrieb in Mehrrobotersystemen.

Ziel dieses Projekts war die Konzeption, Implementierung und experimentelle Validierung einer modularen Steuerungssoftware für den Roboter **Duckiebot DB21J**, um alle gestellten Teilaufgaben zuverlässig, reproduzierbar und robust gegen Umweltfluktuationen (z. B. wechselnde Lichtverhältnisse) zu lösen.

### Hardware & Software-Plattform
- **Roboter**: Duckietown Duckiebot DB21J (Differentialantrieb, zwei DC-Motoren mit Hall-Rad-Encodern, Heck-Stützrad)
- **Recheneinheit**: NVIDIA Jetson Nano (4 GB RAM, Ubuntu Linux / JetPack)
- **Sensorik**:
  - Monokulare Front-Fischaugenkamera (160° FoV)
  - Time-of-Flight (ToF) Distanzsensor an der Front
  - 6-Achsen-IMU (Inertial Measurement Unit)
- **Software-Stack**:
  - Middleware: **ROS** (Robot Operating System)
  - Containerisierung: **Docker** & **Duckietown Shell (`dts`)**
  - Bildverarbeitung: **OpenCV**, **YOLOv8n**
  - Textsatz: **LaTeX** (KOMA-Script `scrreprt`, BibLaTeX / Biber, TikZ/PGF)

### Code-Repository
Der funktionale Programmcode (ROS-Nodes, Launchfiles, Docker-Konfigurationen und YOLO-Modellgewichte) ist im zugehörigen GitHub-Repository verfügbar:
👉 **[Paulchenn/QuackSquad](https://github.com/Paulchenn/QuackSquad.git)**

---

## 🎯 Herausforderungen & Systemmodule

Die im Bericht dokumentierte Gesamtlösung gliedert sich in fünf aufeinander aufbauende Challenges sowie übergreifende Infrastruktur- und Bildverarbeitungsmodule:

```
                               ┌───────────────────────────────────┐
                               │       Kamera & ToF-Sensorik       │
                               └─────────────────┬─────────────────┘
                                                 │
          ┌───────────────────────────┬──────────┴───────────┬───────────────────────────┐
          │                           │                      │                           │
┌─────────▼─────────┐       ┌─────────▼─────────┐  ┌─────────▼─────────┐       ┌─────────▼─────────┐
│ CameraReader Node │       │ YOLO Detection    │  │ Intersection Node │       │ Parking Node      │
│  (Fahrspuren &    │       │  (Duckiebots &    │  │  (Haltelinien &   │       │  (Lückensuche &   │
│   Stopplinien)    │       │   Enten-Objekte)  │  │   Vorfahrtslogik) │       │   Parktrajektorie)│
└─────────┬─────────┘       └─────────┬─────────┘  └─────────┬─────────┘       └─────────┬─────────┘
          │                           │                      │                           │
          │         ┌─────────────────┴────────┐             │                           │
          │         │ BypassDuckie Node        │             │                           │
          │         │ (Ausweichtrajektorien)   │             │                           │
          │         └─────────────────┬────────┘             │                           │
          │                           │                      │                           │
          └─────────────────────┐     │     ┌────────────────┘                           │
                                │     │     │                                            │
                               ┌▼─────▼─────▼────────────────┐                           │
                               │   switch_control_node       │◄──────────────────────────┘
                               │  (Multiplexer & Arbitrierung│
                               │   der Steuerbefehle / Twist)│◄─── ToF Collision Avoidance
                               └──────────────┬──────────────┘
                                              │
                               ┌──────────────▼──────────────┐
                               │   Antriebsräder (cmd_vel)   │
                               └─────────────────────────────┘
```

### Challenge 1: Spurverfolgung (Lane Following)
- **Komponente**: `camera_reader_node`
- **Funktionsweise**: Farbraumtransformation des Kamerabildes nach HSV, dynamische Schwellenwert-Segmentierung der weißen (rechten) Begrenzungslinie und gelben (linken) Mittellinie.
- **Extraktion & Glättung**: Canny-Kantendetektion, morphologische Bereinigung, Begrenzung auf relevante Regions of Interest (ROI) sowie ein gleitender Mittelwertfilter zur Berechnung einer ruckfreien Führungskoordinate (`target_x`).

### Challenge 2: Kreuzungsmanagement & Vorfahrt
- **Komponente**: `intersection_handling_node`
- **Funktionsweise**: Zuverlässige Detektion roter Stopplinien im unteren Bilddrittel mit Pixel-Schwellwertfiltern.
- **Entscheidungsfindung**: Zufällige Richtungswahl an Verzweigungen (links, geradeaus, rechts), Zuordnung zu Bildzonen, Ansteuerung der LED-Blinker und Einhaltung der Verkehrsregel *„Rechts vor Links“*.

### Deep Learning: Objekterkennung (YOLOv8)
- **Komponente**: YOLOv8n-Inferenznode
- **Datensatz & Training**: Generierung teamspezifischer Bilddaten in Duckietown, Annotation mit *Label Studio*, Training des ultraleichten Convolutional Neural Networks (CNN) auf Jetson-Nano-Echtzeitbetrieb optimiert.
- **Erkannte Klassen**: Andere Duckiebots, Gummi-Enten (Duckies) und relevante Landmarken.

### Challenge 3: Hinderniserkennung & Ausweichen (Duckie Detection)
- **Komponente**: `bypass_duckie_node`
- **Funktionsweise**: Erkennt im Fahrstreifen platzierte Enten über das neuronale Netz.
- **Trajektorie**: Leitet ein zeit- und geschwindigkeitsbasiertes Ausweichmanöver auf die Gegenfahrbahn ein, passiert das Hindernis sicher und regelt anschließend wieder autonom auf die eigene Fahrspur zurück.

### Challenge 4: Autonomes Parken (Autonomous Parking)
- **Komponente**: `parking_node`
- **Funktionsweise**: Kontinuierliche Abtastung und Identifikation freier Parktaschen anhand von Fahrbahnmarkierungen.
- **Manöver**: Einleitung eines definierten Rückwärtseinparkvorgangs mit anschließender Haltephase und automatischem Wiedereinfädeln in den fließenden Verkehr.

### Challenge 5: Kollisionsvermeidung & Mehrbot-System
- **Komponenten**: `tof_collision_avoidance_node` & visuelle Situationsanalyse
- **Distanzmessung**: Echtzeit-Abstandsmessung über den Time-of-Flight-Sensor; dynamische Geschwindigkeitsreduktion im Bereich von $0{,}2\,\text{m}$ bis $0{,}4\,\text{m}$ sowie Vollbremsung bei unter $0{,}2\,\text{m}$.
- **Mehrbot-Interaktion**: Koordiniertes Fahren im Konvoi und konfliktfreie Kreuzungsdurchquerung im Zusammenspiel mit fremden Duckiebots.

### Entwicklungs- und Kalibrierungstools
- `white_yellow_red_calibration_node`: Interaktive GUI mit OpenCV-Trackbars zur Live-Kalibrierung der HSV-Grenzwerte unter veränderlichen Lichtbedingungen mit direktem Export in `YAML`-Dateien.
- `display_and_save_cameraImg_node`: Zyklische Speicherung und Live-Überwachung von Kamerastreams zur Erstellung von Trainingsdatensätzen.
- `switch_control_node`: Zentrale Arbitrierungsinstanz zur Priorisierung konkurrierender Bewegungsbefehle (`geometry_msgs/Twist`).

---

## 📁 Repository- und Dateistruktur

```text
QuackSquad_Doc/
├── thesis.tex                      # Hauptdokument: Präambel, Paket-Imports, Gliederung
├── bibliography.bib                # Literaturquellen (BibLaTeX-Format)
├── Glossar.bib                     # Glossar- und Abkürzungsressourcen
├── copyAndRename_thesis.bat        # Windows-Hilfsskript zur automatischen Versionierung
├── .gitignore                      # Ignorierte TeX-Build-Artefakte und Output-PDFs
├── README.md                       # Diese Projektdokumentation
│
├── TeXFiles/                       # Modulare Kapitel und Konfigurationen
│   ├── 000_Settings.tex            # Metadaten (Autoren, Titel, CI-Farben, Betreuer)
│   ├── 001_Titlepage.tex           # HKA-konformes Titelblatt
│   ├── 002_NDNotice.tex            # Vertraulichkeits-/Sperrvermerk-Vorlage
│   ├── 004_Acronyms.tex            # Abkürzungsverzeichnis (ROS, ToF, HSV, CNN, ...)
│   ├── 005_Glossar.tex             # Glossardefinitionen
│   ├── 020_Einfuehrung.tex         # Kapitel 1: Einführung, Motivation, Duckietown DB21J
│   ├── 030_Challenge_1.tex         # Kapitel 2: Einleitung Challenge 1
│   ├── 031_LaneFollowing_Node.tex  # Technische Umsetzung Spurverfolgung (camera_reader)
│   ├── 040_Challenge_2.tex         # Kapitel 3: Einleitung Challenge 2
│   ├── 041_IntersectionHandling... # Kreuzungsverhalten & Vorfahrt (Rechts vor Links)
│   ├── 045_Objekterkennung.tex     # YOLOv8-Modellierung, Training & Metriken
│   ├── 050_Challenge_3.tex         # Kapitel 4: Einleitung Challenge 3
│   ├── 051_DuckieDetection_Node... # Hinderniserkennung & Ausweichtrajektorie
│   ├── 060_Challenge_4.tex         # Kapitel 5: Einleitung Challenge 4
│   ├── 061_parking_Node.tex        # Parkmanöver & Parklückenerkennung
│   ├── 070_Challenge_5.tex         # Kapitel 6: Einleitung Challenge 5
│   ├── 071_Collision_Avoidance...  # ToF-Sensorik & Konvoifahrt
│   ├── 080_Tools.tex               # Kapitel 7: HSV-Kalibrierung & Datensatz-Tools
│   ├── 100_Zusammenfassung.tex     # Kapitel 8: Fazit und Ausblick
│   ├── 110_Anhang.tex              # Anhang (optional)
│   └── 120_Declaration.tex         # Ehrenwörtliche Erklärung
│
├── Figures/                        # Bild- und Diagramm-Assets
│   ├── png/                        # Screenshots, Fotos, Plots, Ablaufdiagramme (.png)
│   ├── jpg/                        # Fotomaterial
│   ├── pdf/                        # Vektorgrafiken
│   ├── svg/                        # SVG-Ressourcen
│   ├── tikz/                       # TikZ-Quelldateien
│   ├── tikz2pdf/                   # Standalone-TikZ-PDFs
│   ├── tikzexternalize_cache/      # Cache für TikZ-Kompilierung
│   └── Unterschrift/               # Unterschriften-Scans
│
└── Sonstiges/                      # Zusatzmaterialien
    └── Ablaufdiagramm.drawio        # Editierbare Quelldatei aller Flowcharts
```

---

## 🛠 Kompilierung & Bauanleitung

Das Dokument basiert auf **KOMA-Script (`scrreprt`)** und verwendet moderne Pakete wie **BibLaTeX (Backend: Biber)** und **TikZ Externalize**.

### Voraussetzungen
Stellen Sie sicher, dass eine vollständige TeX-Distribution installiert ist:
- **macOS**: [MacTeX](https://www.tug.org/mactex/) (`brew install --cask mactex-no-gui`)
- **Linux (Ubuntu/Debian)**: `sudo apt install texlive-full biber`
- **Windows**: [MiKTeX](https://miktex.org/) oder [TeX Live](https://www.tug.org/texlive/)

Benötigte Hauptwerkzeuge auf dem `PATH`:
- `pdflatex`
- `biber`
- `latexmk`

### Kompilierung via CLI

#### 🚀 Empfohlene Methode (automatisiert mit `latexmk`)
`latexmk` erkennt Abhängigkeiten und führt automatisch alle erforderlichen TeX- und Biber-Durchläufe aus:

```bash
# Vollständiger Build (PDF-Ausgabe: thesis.pdf)
latexmk -pdf thesis.tex
```

Für kontinuierliche Re-Kompilierung bei Dateiänderungen:
```bash
latexmk -pvc -pdf thesis.tex
```

#### ⚙️ Manuelle Schrittfolge
Falls `latexmk` nicht genutzt wird, ist die kanonische Sequenz zur Auflösung von Querverweisen, Verzeichnissen und Literaturstellen auszuführen:

```bash
pdflatex thesis.tex
biber thesis
pdflatex thesis.tex
pdflatex thesis.tex
```

### Kompilierung in IDEs
- **VS Code**: Empfohlen mit der Erweiterung **[LaTeX Workshop](https://marketplace.visualstudio.com/items?itemName=James-Yu.latex-workshop)**. Die Standard-Build-Toolchain nutzt direkt `latexmk`.
- **TeXstudio / TeXmaker**: Als Standard-Bibliografieprogramm `biber %` (statt `bibtex`) einstellen.

### Bereinigen von Build-Artefakten
Zum Entfernen aller temporären Hilfsdateien (`.aux`, `.bbl`, `.blg`, `.log`, `.toc`, etc.):

```bash
latexmk -c thesis.tex
# oder radikal inklusive des erzeugten PDFs:
latexmk -C thesis.tex
```

---

## 👥 Autoren & Betreuung

### Team QuackSquad (Master Maschinenbau / Mechatronik, HKA)
| Name | Matrikelnummer | Studienschwerpunkt / Rolle |
| :--- | :---: | :--- |
| **Paul Glaser** | 100215 | Robotik & Systemintegration |
| **Julia Kramme** | 100381 | Bildverarbeitung & Algorithmik |
| **Analena Krieger** | 100551 | Sensorik & Regelungstechnik |
| **Tim Schäfer** | 71606 | Objekterkennung & Deep Learning |

### Akademische Betreuung
- **Erstprüfer / Dozent**: **Prof. Dr.-Ing. Christian Wurll**
- **Institution**: **Hochschule Karlsruhe (HKA)** – Fakultät für Maschinenbau und Mechatronik
- **Abgabedatum**: 31. Juli 2025

---

> *Hinweis: Dieses Repository enthält ausschließlich die wissenschaftliche Ausarbeitung und Dokumentation. Den lauffähigen ROS-Programmcode finden Sie unter [github.com/Paulchenn/QuackSquad](https://github.com/Paulchenn/QuackSquad.git).*
