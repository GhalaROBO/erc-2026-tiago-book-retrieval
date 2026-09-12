import cv2
import numpy as np


# --------------------------------------------------
# LOAD TEST IMAGE
# --------------------------------------------------

image = cv2.imread(
    "competition_test_scene.png"
)

if image is None:
    raise RuntimeError(
        "Could not load competition_test_scene.png"
    )


# --------------------------------------------------
# LOAD DIGIT TEMPLATES
# --------------------------------------------------

templates = {}

for digit in range(1, 6):

    template = cv2.imread(
        f"templates/{digit}.png",
        cv2.IMREAD_GRAYSCALE
    )

    if template is None:
        raise RuntimeError(
            f"Missing template for digit {digit}"
        )

    templates[digit] = template


# --------------------------------------------------
# NUMBER LOCATIONS
# --------------------------------------------------

number_boxes = [
    (190, 80, 270, 150),
    (350, 80, 430, 150),
    (510, 80, 590, 150),
    (670, 80, 750, 150),
    (830, 80, 910, 150),
]


# --------------------------------------------------
# EXTRACT DIGIT
# --------------------------------------------------

def extract_digit(roi):

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    _, binary = cv2.threshold(
        gray,
        180,
        255,
        cv2.THRESH_BINARY_INV
    )

    contours, _ = cv2.findContours(
        binary,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    largest = max(
        contours,
        key=cv2.contourArea
    )

    x, y, w, h = cv2.boundingRect(
        largest
    )

    digit = binary[
        y:y + h,
        x:x + w
    ]

    digit = cv2.resize(
        digit,
        (50, 70),
        interpolation=cv2.INTER_NEAREST
    )

    return digit


# --------------------------------------------------
# COMPARE WITH TEMPLATES
# --------------------------------------------------

def recognize_digit(digit_image):

    best_digit = None
    best_score = float("inf")

    for digit, template in templates.items():

        difference = cv2.absdiff(
            digit_image,
            template
        )

        score = np.mean(difference)

        if score < best_score:
            best_score = score
            best_digit = digit

    return best_digit, best_score


# --------------------------------------------------
# RECOGNIZE SHELF LABELS
# --------------------------------------------------

output = image.copy()

detected_order = []

for position, box in enumerate(
    number_boxes,
    start=1
):

    x1, y1, x2, y2 = box

    roi = image[
        y1:y2,
        x1:x2
    ]

    digit_image = extract_digit(roi)

    if digit_image is None:

        print(
            f"Position {position}: "
            "NO DIGIT FOUND"
        )

        detected_order.append(None)

        continue

    digit, score = recognize_digit(
        digit_image
    )

    detected_order.append(digit)

    center_x = (x1 + x2) // 2

    print(
        f"Position {position}: "
        f"digit={digit}, "
        f"x={center_x}, "
        f"score={score:.2f}"
    )

    cv2.rectangle(
        output,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2
    )

    cv2.putText(
        output,
        f"Detected: {digit}",
        (x1 - 10, y1 - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 255),
        2
    )


# --------------------------------------------------
# FINAL RESULT
# --------------------------------------------------

print()
print("==============================")
print("SHELF ORDER")
print("==============================")

print(detected_order)


expected = [2, 1, 4, 5, 3]

print()

if detected_order == expected:

    print("SUCCESS")
    print(
        "Physical shelf arrangement "
        "recognized correctly."
    )

else:

    print("NOT CORRECT YET")
    print("Expected:", expected)
    print("Detected:", detected_order)


cv2.imwrite(
    "template_number_detection.png",
    output
)

print()
print(
    "Saved template_number_detection.png"
)
