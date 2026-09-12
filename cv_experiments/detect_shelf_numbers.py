import cv2
import pytesseract


# --------------------------------------------------
# LOAD IMAGE
# --------------------------------------------------

image = cv2.imread("competition_test_scene.png")

if image is None:
    raise RuntimeError(
        "ERROR: Could not load competition_test_scene.png"
    )


# --------------------------------------------------
# MANUALLY DEFINED NUMBER BOXES FOR TEST SCENE
# x1, y1, x2, y2
# --------------------------------------------------

number_boxes = [
    (190, 80, 270, 150),
    (350, 80, 430, 150),
    (510, 80, 590, 150),
    (670, 80, 750, 150),
    (830, 80, 910, 150),
]


# --------------------------------------------------
# OCR CONFIGURATION
# --------------------------------------------------

config = (
    "--psm 10 "
    "-c tessedit_char_whitelist=12345"
)


# --------------------------------------------------
# PROCESS EACH NUMBER SEPARATELY
# --------------------------------------------------

detected_numbers = []

debug_output = image.copy()

for index, (x1, y1, x2, y2) in enumerate(number_boxes):

    roi = image[y1:y2, x1:x2]

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    # Enlarge image to help OCR
    enlarged = cv2.resize(
        gray,
        None,
        fx=4,
        fy=4,
        interpolation=cv2.INTER_CUBIC
    )

    # Automatic threshold
    _, binary = cv2.threshold(
        enlarged,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # Add white border
    binary = cv2.copyMakeBorder(
        binary,
        30,
        30,
        30,
        30,
        cv2.BORDER_CONSTANT,
        value=255
    )

    text = pytesseract.image_to_string(
        binary,
        config=config
    )

    clean_text = "".join(
        char for char in text
        if char in "12345"
    )

    if clean_text:
        detected_digit = int(clean_text[0])
    else:
        detected_digit = None

    detected_numbers.append(detected_digit)

    print(
        f"Label position {index + 1}: "
        f"raw={repr(text)} "
        f"detected={detected_digit}"
    )

    # Save debug image for each label
    cv2.imwrite(
        f"number_{index + 1}_processed.png",
        binary
    )

    # Draw result
    cv2.rectangle(
        debug_output,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2
    )

    if detected_digit is not None:

        cv2.putText(
            debug_output,
            str(detected_digit),
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )


# --------------------------------------------------
# FINAL RESULT
# --------------------------------------------------

print()
print("==============================")
print("DETECTED SHELF NUMBERS")
print("==============================")

print(detected_numbers)

expected = [1, 2, 3, 4, 5]

if detected_numbers == expected:

    print()
    print("SUCCESS")
    print("All shelf numbers detected correctly.")

else:

    print()
    print("NOT PERFECT YET")
    print("Expected:", expected)
    print("Detected:", detected_numbers)


cv2.imwrite(
    "shelf_number_detection.png",
    debug_output
)

print()
print("Saved shelf_number_detection.png")
