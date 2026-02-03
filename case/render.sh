#!/bin/bash
#
# OpenSCAD CLI Renderer für ESP32 DevKit Case
# Umgeht den GUI-Crash auf macOS Tahoe
#
# Usage:
#   ./render.sh                    # Rendert bottom + lid mit Defaults
#   ./render.sh bottom             # Nur Bottom
#   ./render.sh lid                # Nur Lid
#   ./render.sh both               # Bottom + Lid (default)
#   ./render.sh -D 'board_len=69'  # Custom Parameter übergeben
#

SCAD_FILE="$(dirname "$0")/esp32_devkit_case.scad"
OUTPUT_DIR="$(dirname "$0")/stl"
OPENSCAD="/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"

# Fallback zu brew-installiertem OpenSCAD
if [ ! -x "$OPENSCAD" ]; then
    OPENSCAD="$(which openscad 2>/dev/null)"
fi

if [ ! -x "$OPENSCAD" ]; then
    echo "Error: OpenSCAD nicht gefunden!"
    echo "Installiere mit: brew install openscad"
    exit 1
fi

# Output-Verzeichnis erstellen
mkdir -p "$OUTPUT_DIR"

# Argumente parsen
PART="both"
EXTRA_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        bottom|lid|both)
            PART="$1"
            shift
            ;;
        -D)
            EXTRA_ARGS="$EXTRA_ARGS -D '$2'"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

render_part() {
    local part=$1
    local output="$OUTPUT_DIR/esp32_case_${part}.stl"

    echo "Rendering $part -> $output"

    "$OPENSCAD" \
        --render \
        -o "$output" \
        -D "RENDER_PART=\"${part}\"" \
        "$SCAD_FILE" 2>&1

    if [ -f "$output" ] && [ -s "$output" ]; then
        echo "  Done: $(ls -lh "$output" | awk '{print $5}')"
    else
        echo "  Error rendering!"
        return 1
    fi
}

echo "=== ESP32 DevKit Case Renderer ==="
echo ""

case $PART in
    bottom)
        render_part "bottom"
        ;;
    lid)
        render_part "lid"
        ;;
    both)
        render_part "bottom"
        render_part "lid"
        ;;
esac

echo ""
echo "STL-Dateien in: $OUTPUT_DIR/"
ls -la "$OUTPUT_DIR"/*.stl 2>/dev/null
