import cv2
import numpy as np


# --------------------------------------------------
# LOAD IMAGE
# --------------------------------------------------

image = cv2.imread("competition_test_scene.png")

if image is None:
    raise RuntimeError(
        "ERROR: Could not load competition_test_scene.png"
    )

output = image.copy()

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


# --------------------------------------------------
# HSV COLOR RANGES
# --------------------------------------------------

color_ranges = {

    "GREEN": [
        (
            np.array([35, 100, 100]),
            np.array([85, 255, 255])
        )
    ],

    "BLUE": [
        (
            np.array([90, 100, 100]),
            np.array([130, 255, 255])
        )
    ],

    "YELLOW": [
        (
            np.array([20, 100, 100]),
            np.array([35, 255, 255])
        )
    ],

    "RED": [
        (
            np.array([0, 100, 100]),
            np.array([10, 255, 255])
        ),
        (
            np.array([170, 100, 100]),
            np.array([179, 255, 255])
        )
    ]
}


# --------------------------------------------------
# CREATE MASK
# --------------------------------------------------

def create_mask(hsv_image, ranges):

    final_mask = np.zeros(
        hsv_image.shape[:2],
        dtype=np.uint8
    )

    for lower, upper in ranges:

        current_mask = cv2.inRange(
            hsv_image,
            lower,
            upper
        )

        final_mask = cv2.bitwise_or(
            final_mask,
            current_mask
        )

    return final_mask


# --------------------------------------------------
# CLASSIFY OBJECT
# --------------------------------------------------

def classify_object(color_name, x, y, w, h, area):

    aspect_ratio = w / float(h)

    # Large red object = collection bin
    if (
        color_name == "RED"
        and area > 10000
        and w > 100
        and h > 100
    ):
        return "COLLECTION BIN"

    # Tall, narrow colored object = book
    if (
        area > 1000
        and area < 10000
        and h > w
        and aspect_ratio < 0.8
    ):
        return f"{color_name} BOOK"

    return "UNKNOWN"


# --------------------------------------------------
# PROCESS COLORS
# --------------------------------------------------

detections = []

for color_name, ranges in color_ranges.items():

    mask = create_mask(
        hsv,
        ranges
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    for contour in contours:

        area = cv2.contourArea(contour)

        if area < 500:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        center_x = x + w // 2
        center_y = y + h // 2

        aspect_ratio = w / float(h)

        object_type = classify_object(
            color_name,
            x,
            y,
            w,
            h,
            area
        )

        detection = {
            "type": object_type,
            "color": color_name,
            "center_x": center_x,
            "center_y": center_y,
            "width": w,
            "height": h,
            "area": area,
            "aspect_ratio": aspect_ratio
        }

        detections.append(detection)

        # ------------------------------------------
        # DRAW RESULT
        # ------------------------------------------

        if object_type == "COLLECTION BIN":
            box_color = (255, 0, 255)

        elif "BOOK" in object_type:
            box_color = (0, 180, 0)

        else:
            box_color = (0, 0, 0)

        cv2.rectangle(
            output,
            (x, y),
            (x + w, y + h),
            box_color,
            3
        )

        cv2.circle(
            output,
            (center_x, center_y),
            5,
            (0, 0, 0),
            -1
        )

        cv2.putText(
            output,
            object_type,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            box_color,
            2
        )


# --------------------------------------------------
# PRINT RESULTS
# --------------------------------------------------

print()
print("========================================")
print("PERCEPTION RESULTS")
print("========================================")

book_count = 0
bin_count = 0
unknown_count = 0

for detection in detections:

    print()

    print("Type:", detection["type"])
    print("Color:", detection["color"])

    print(
        "Center:",
        (
            detection["center_x"],
            detection["center_y"]
        )
    )

    print(
        "Size:",
        (
            detection["width"],
            detection["height"]
        )
    )

    print(
        "Area:",
        round(detection["area"], 2)
    )

    print(
        "Aspect ratio:",
        round(detection["aspect_ratio"], 2)
    )

    if "BOOK" in detection["type"]:
        book_count += 1

    elif detection["type"] == "COLLECTION BIN":
        bin_count += 1

    else:
        unknown_count += 1


print()
print("========================================")
print("SUMMARY")
print("========================================")

print("Books detected:", book_count)
print("Collection bins detected:", bin_count)
print("Unknown objects:", unknown_count)


# --------------------------------------------------
# SAVE RESULT
# --------------------------------------------------

cv2.imwrite(
    "classified_objects.png",
    output
)

print()
print("Saved classified_objects.png")
