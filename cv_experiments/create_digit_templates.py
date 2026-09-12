import cv2
import os


image = cv2.imread("competition_test_scene.png")

if image is None:
    raise RuntimeError(
        "Could not load competition_test_scene.png"
    )


os.makedirs("templates", exist_ok=True)


number_boxes = [
    (190, 80, 270, 150),
    (350, 80, 430, 150),
    (510, 80, 590, 150),
    (670, 80, 750, 150),
    (830, 80, 910, 150),
]


for digit, (x1, y1, x2, y2) in enumerate(
    number_boxes,
    start=1
):

    roi = image[y1:y2, x1:x2]

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
        print(f"ERROR: No contour for digit {digit}")
        continue

    largest = max(
        contours,
        key=cv2.contourArea
    )

    x, y, w, h = cv2.boundingRect(largest)

    digit_image = binary[
        y:y + h,
        x:x + w
    ]

    digit_image = cv2.resize(
        digit_image,
        (50, 70),
        interpolation=cv2.INTER_NEAREST
    )

    filename = f"templates/{digit}.png"

    cv2.imwrite(
        filename,
        digit_image
    )

    print(
        f"Created template for digit {digit}: "
        f"{filename}"
    )


print()
print("All templates created.")
