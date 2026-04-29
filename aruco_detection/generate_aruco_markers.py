"""
Generate printable ArUco marker PNGs for ROS bag recording.

Marker ID → sign label mapping (matches detect_aruco.py):
    0 → Stop
    1 → Turn right
    2 → Turn left
    3 → Ahead only
    4 → Roundabout mandatory

Output: aruco_detection/markers/marker_<id>_<label>.png

Usage:
    python3 aruco_detection/generate_aruco_markers.py
"""

from pathlib import Path
import cv2
import numpy as np

MARKER_MAP = {
    0: 'Stop',
    1: 'Turn right',
    2: 'Turn left',
    3: 'Ahead only',
    4: 'Roundabout mandatory',
}

ARUCO_DICT   = cv2.aruco.DICT_4X4_50
MARKER_PX    = 400    # marker image size in pixels (before border/label)
BORDER_PX    = 40     # white quiet-zone border width (recommended: 1+ modules)
LABEL_HEIGHT = 60     # pixels reserved below marker for text label
DPI_NOTE     = 300    # informational only — used to compute print size hint

OUT_DIR = Path(__file__).resolve().parent / 'markers'


def slugify(label):
    return label.lower().replace(' ', '_')


def generate_marker(marker_id, label):
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

    # Draw the raw marker
    marker_img = np.zeros((MARKER_PX, MARKER_PX), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PX, marker_img, 1)

    # Add white quiet-zone border
    canvas_w = MARKER_PX + 2 * BORDER_PX
    canvas_h = MARKER_PX + 2 * BORDER_PX + LABEL_HEIGHT
    canvas   = np.full((canvas_h, canvas_w), 255, dtype=np.uint8)
    canvas[BORDER_PX:BORDER_PX + MARKER_PX, BORDER_PX:BORDER_PX + MARKER_PX] = marker_img

    # Label below the marker
    font       = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.9
    thickness  = 2
    text       = f'ID {marker_id}  —  {label}'
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
    tx = (canvas_w - tw) // 2
    ty = BORDER_PX + MARKER_PX + (LABEL_HEIGHT + th) // 2
    cv2.putText(canvas, text, (tx, ty), font, font_scale, 0, thickness, cv2.LINE_AA)

    # Convert to BGR for imwrite (keeps it a proper PNG)
    out = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    filename = f'marker_{marker_id}_{slugify(label)}.png'
    path     = OUT_DIR / filename
    cv2.imwrite(str(path), out)

    # Print hint: at 300 DPI, MARKER_PX px → physical size in cm
    px_per_cm = DPI_NOTE / 2.54
    size_cm   = MARKER_PX / px_per_cm
    print(f'  [{marker_id}] {label:<25s}  →  {filename}')
    print(f'       Marker area: {size_cm:.1f} × {size_cm:.1f} cm at {DPI_NOTE} DPI '
          f'(full sheet: {(canvas_w/px_per_cm):.1f} × {(canvas_h/px_per_cm):.1f} cm)')


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f'Dictionary : DICT_4X4_50')
    print(f'Output dir : {OUT_DIR}\n')

    for marker_id, label in sorted(MARKER_MAP.items()):
        generate_marker(marker_id, label)

    print(f'\nDone — {len(MARKER_MAP)} markers written to {OUT_DIR}')
    print('\nPrinting tips:')
    print('  • Print at 100% scale (disable "fit to page")')
    print('  • Use the DPI hint above to verify physical size after printing')
    print('  • Laminate or mount on stiff card before attaching to the cylinder')


if __name__ == '__main__':
    main()
