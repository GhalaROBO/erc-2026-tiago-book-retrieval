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

print("Image loaded successfully.")
print("Image size:", image.shape)


# --------------------------------------------------
# CONVERT BGR IMAGE TO HSV
# --------------------------------------------------

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


# --------------------------------------------------
# DEFINE HSV COLOR RANGES
# --------------------------------------------------

# Red needs two ranges because red wraps around
# the OpenCV hue scale.

lower_red_1 = np.array([0, 100, 100])
upper_red_1 = np.array([10, 255, 255])

lower_red_2 = np.array([170, 100, 100])
upper_red_2 = np.array([179, 255, 255])

lower_green = np.array([35, 100, 100])
upper_green = np.array([85, 255, 255])

lower_blue = np.array([90, 100, 100])
upper_blue = np.array([130, 255, 255])

lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([35, 255, 255])


# --------------------------------------------------
# CREATE COLOR MASKS
# --------------------------------------------------

red_mask_1 = cv2.inRange(
    hsv,
    lower_red_1,
    upper_red_1
)

red_mask_2 = cv2.inRange(
    hsv,
    lower_red_2,
    upper_red_2
)

red_mask = cv2.bitwise_or(
    red_mask_1,
    red_mask_2
)

green_mask = cv2.inRange(
    hsv,
    lower_green,
    upper_green
)

blue_mask = cv2.inRange(
    hsv,
    lower_blue,
    upper_blue
)

yellow_mask = cv2.inRange(
    hsv,
    lower_yellow,
    upper_yellow
)


# --------------------------------------------------
# STORE MASKS
# --------------------------------------------------

masks = {
    "RED": red_mask,
    "GREEN": green_mask,
    "BLUE": blue_mask,
    "YELLOW": yellow_mask
}


# --------------------------------------------------
# COLORS USED FOR DRAWING
# These are BGR values.
# --------------------------------------------------

drawing_colors = {
    "RED": (0, 0, 255),
    "GREEN": (0, 180, 0),
    "BLUE": (255, 0, 0),
    "YELLOW": (0, 180, 180)
}


# --------------------------------------------------
# COPY ORIGINAL IMAGE
# --------------------------------------------------

output = image.copy()


# --------------------------------------------------
# DETECT OBJECTS
# --------------------------------------------------

all_detections = []

for color_name, mask in masks.items():

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    print()
    print("------------------------------")
    print("Searching for:", color_name)
    print("------------------------------")

    object_number = 0

    for contour in contours:

        area = cv2.contourArea(contour)

        # Ignore tiny objects/noise
        if area < 500:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        center_x = x + (w // 2)
        center_y = y + (h // 2)

        object_number += 1

        detection = {
            "color": color_name,
            "x": x,
            "y": y,
            "width": w,
            "height": h,
            "center_x": center_x,
            "center_y": center_y,
            "area": area
        }

        all_detections.append(detection)

        print(
            f"Object {object_number}: "
            f"x={x}, y={y}, "
            f"w={w}, h={h}, "
            f"center=({center_x}, {center_y}), "
            f"area={area:.0f}"
        )

        # Draw bounding box
        cv2.rectangle(
            output,
            (x, y),
            (x + w, y + h),
            drawing_colors[color_name],
            3
        )

        # Draw center
        cv2.circle(
            output,
            (center_x, center_y),
            6,
            (0, 0, 0),
            -1
        )

        # Draw label
        cv2.putText(
            output,
            color_name,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            drawing_colors[color_name],
            2
        )

    print(
        f"Total {color_name} objects detected:",
        object_number
    )


# --------------------------------------------------
# FINAL SUMMARY
# --------------------------------------------------

print()
print("==============================")
print("TOTAL DETECTIONS:", len(all_detections))
print("==============================")


# --------------------------------------------------
# SAVE RESULT
# --------------------------------------------------

cv2.imwrite(
    "detected_colors.png",
    output
)

cv2.imwrite(
    "red_mask.png",
    red_mask
)

cv2.imwrite(
    "green_mask.png",
    green_mask
)

cv2.imwrite(
    "blue_mask.png",
    blue_mask
)

cv2.imwrite(
    "yellow_mask.png",
    yellow_mask
)

print("Saved detected_colors.png")
print("Saved color masks.")
