# ESP32 DevKit Parametric Case

Parametrisches OpenSCAD-Gehäuse mit:
- **Snap-Fit** Deckel (keine Schrauben nötig)
- **Compliant Buttons** für Reset/Boot (im Deckel integriert, keine Einzelteile)
- **LED-Fenster** (RGB, CHG, PWR) mit optionaler Membran
- **USB-C Ausschnitt**
- **Belüftungsschlitze**
- **Optionale Montage-Tabs**

## Verwendung

1. `esp32_devkit_case.scad` in OpenSCAD öffnen
2. Im Customizer die Board-Maße eintragen
3. `RENDER_PART` auf `"bottom"` setzen → F6 → Export STL
4. `RENDER_PART` auf `"lid"` setzen → F6 → Export STL
5. Slicen und drucken

## Wichtige Parameter

### Board-Maße (mit Schieblehre messen!)

| Parameter | Beschreibung |
|-----------|--------------|
| `board_len` | Platinenlänge (X, USB-Seite) |
| `board_wid` | Platinenbreite (Y) |
| `board_thick` | PCB-Dicke (meist 1.6mm) |
| `board_top_height` | Höchstes Bauteil oben |
| `board_bottom_height` | Pins/Lötstellen unten |

### Button-Positionen (relativ zur USB-Ecke links unten)

| Parameter | Beschreibung |
|-----------|--------------|
| `btn_reset_pos` | [X, Y] Position Reset-Taster |
| `btn_boot_pos` | [X, Y] Position Boot-Taster |
| `btn_height_above_board` | Höhe der Tasterkappe über PCB |

### LED-Positionen

| Parameter | Beschreibung |
|-----------|--------------|
| `led_rgb_pos` | [X, Y] Position RGB-LED |
| `led_chg_pos` | [X, Y] Position Charge-LED |
| `led_pwr_pos` | [X, Y] Position Power-LED |
| `led_style` | 0=Loch, 1=Membran (0.4mm), 2=Lichtleiter |

### USB-Ports (bis zu 3 Stück)

| Parameter | Beschreibung |
|-----------|--------------|
| `usb_count` | Anzahl der USB-Ports (0-3) |
| `usbN_side` | Seite: 0=vorne(-X), 1=hinten(+X), 2=links(-Y), 3=rechts(+Y) |
| `usbN_position` | Position entlang der Kante (0.0-1.0, 0.5=Mitte) |
| `usbN_width` | Breite des Ausschnitts |
| `usbN_height` | Höhe des Ausschnitts |
| `usbN_z_offset` | Höhe über Platinen-Unterseite |

**Typische USB-Maße:**
- USB-C: 12mm x 7mm
- Micro-USB: 9mm x 4mm
- USB-A: 14mm x 7mm

## Beispiel-Konfigurationen

### Waveshare ESP32-C5-WIFI6-KIT (noch zu messen!)

```openscad
board_len = 52.0;    // TODO: messen
board_wid = 28.0;    // TODO: messen
board_top_height = 8.0;

btn_reset_pos = [3.5, 14.0];  // TODO: messen
btn_boot_pos = [3.5, 7.0];    // TODO: messen

led_rgb_pos = [45.0, 14.0];   // TODO: messen
led_chg_pos = [40.0, 5.0];    // TODO: messen
led_pwr_pos = [40.0, 23.0];   // TODO: messen
```

### ESP32-S3-DevKitC-1 N16R8 (noch zu messen!)

```openscad
board_len = 69.0;    // TODO: messen
board_wid = 25.5;    // TODO: messen
board_top_height = 10.0;

btn_reset_pos = [2.5, 12.75];  // TODO: messen
btn_boot_pos = [66.5, 12.75];  // TODO: messen

led_rgb_pos = [60.0, 20.0];    // TODO: messen (RGB auf GPIO48)
led_chg_pos = [0, 0];          // keine CHG LED
led_pwr_pos = [5.0, 20.0];     // TODO: messen
```

## Druckeinstellungen

| Parameter | Empfehlung |
|-----------|------------|
| Material | PETG (beste Flexibilität für Snaps) |
| Layer | 0.2mm |
| Infill | 20-30% |
| Walls | 3-4 |
| Orientierung | Bottom: Öffnung oben, Lid: Buttons oben (wird im SCAD geflippt) |

## Compliant Button Mechanik

Die Taster sind als einstückige Biegebalken (Living Hinge) ausgeführt:

```
     [Finger Pad]
          │
    ┌─────┴─────┐  ← Deckeloberfläche
    │           │
    │  ═══════╪═│  ← Flex-Arm (0.8-1.2mm dick)
    │         │ │
    │    [Pusher Pin]
    │         │ │
    │    [Board Button]
```

Bei Druck: Der Flex-Arm muss in XY-Ebene liegen (Layer quer zur Biegung).
Dicke `flex_thickness` anpassen: 0.8mm für weiches PETG, 1.2mm für steifes PLA.

## Anpassungen

- **Mehr LEDs?** → `led_window()` Modul duplizieren
- **Keine Buttons?** → `btn_enable = false`
- **Verschraubt statt Snap-Fit?** → Snap-Hooks entfernen, Schraubdome ergänzen
- **Antennen-Ausschnitt?** → Rechteck im Deckel über Antenne subtrahieren
