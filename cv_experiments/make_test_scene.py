import cv2
import numpy as np

# Create a white 1280x720 image
image = np.ones((720, 1280, 3), dtype=np.uint8) * 255

# -----------------------------
# DRAW SHELF
# -----------------------------

# Outer shelf frame
cv2.rectangle(image, (150, 150), (950, 600), (160, 160, 160), 6)

# Vertical shelf divisions
for x in [310, 470, 630, 790]:
    cv2.line(image, (x, 150), (x, 600), (180, 180, 180), 4)

# Horizontal shelf rows
for y in [300, 450]:
    cv2.line(image, (150, y), (950, y), (180, 180, 180), 4)

# -----------------------------
# DRAW SHELF NUMBERS
# -----------------------------

labels = ["2", "1", "4", "5", "3"]
x_positions = [230, 390, 550, 710, 870]

for label, x in zip(labels, x_positions):

    cv2.rectangle(
        image,
        (x - 30, 90),
        (x + 30, 140),
        (240, 240, 240),
        -1
    )

    cv2.putText(
        image,
        label,
        (x - 12, 128),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 0, 0),
        3
    )

# -----------------------------
# DEFINE COLORS
# OpenCV uses BGR, not RGB
# -----------------------------

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)

# -----------------------------
# DRAW BOOKS
# -----------------------------

books = [
    ((200, 220), RED),
    ((360, 220), GREEN),
    ((520, 220), BLUE),
    ((680, 220), YELLOW),
    ((840, 220), RED),

    ((200, 370), GREEN),
    ((360, 370), BLUE),
    ((520, 370), YELLOW),
    ((680, 370), RED),
    ((840, 370), GREEN),

    ((200, 520), BLUE),
    ((360, 520), YELLOW),
    ((520, 520), RED),
    ((680, 520), GREEN),
    ((840, 520), BLUE),
]

for (x, y), color in books:

    cv2.rectangle(
        image,
        (x, y),
        (x + 35, y + 70),
        color,
        -1
    )

# -----------------------------
# DRAW RED COLLECTION BIN
# -----------------------------

cv2.rectangle(
    image,
    (1020, 430),
    (1200, 590),
    RED,
    -1
)

# darker red top edge
cv2.rectangle(
    image,
    (1010, 410),
    (1210, 440),
    (0, 0, 180),
    -1
)

cv2.putText(
    image,
    "RED BIN",
    (1040, 650),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.8,
    (0, 0, 0),
    2
)

# -----------------------------
# SAVE IMAGE
# -----------------------------

filename = "competition_test_scene.png"

cv2.imwrite(filename, image)

print(f"Saved {filename}")
