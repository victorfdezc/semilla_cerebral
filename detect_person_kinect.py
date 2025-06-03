#!/usr/bin/env python3
import freenect
import cv2
import numpy as np

# Initialize the HOG person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Constants for depth scaling and display
MAX_DEPTH_MM = 4000  # ignore anything beyond 4 m for visualization
DEPTH_DISPLAY_SCALE = 255.0 / MAX_DEPTH_MM  # to map mm → 0–255

def get_frames():
    """
    Grab a depth frame (in millimetres) and an RGB frame (in uint8 BGR).
    Returns:
        depth_mm: H×W uint16 array of distance in millimetres
        rgb_bgr: H×W×3 uint8 array in BGR order
    """
    # Get depth in mm
    depth_mm, _ = freenect.sync_get_depth(format=freenect.DEPTH_MM)
    # Get RGB as uint8 RGB; convert to BGR for OpenCV
    rgb, _ = freenect.sync_get_video()
    rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return depth_mm, rgb_bgr

def draw_detections(frame, detections):
    """
    Draw bounding boxes and labels on the frame.
    Args:
        frame: H×W×3 BGR image (modified in-place)
        detections: list of tuples (x, y, w, h, distance_mm)
    """
    for (x, y, w, h, dist) in detections:
        # Bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # Depth label: convert mm → metres with one decimal
        text = f"{dist/1000:.1f} m"
        cv2.putText(
            frame, text,
            (x, y - 10 if y > 20 else y + 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )

def estimate_distance(depth_mm, bbox):
    """
    Estimate a single distance reading inside the bounding box.
    Compute the median of all nonzero pixels in the central region of bbox.
    Args:
        depth_mm: H×W uint16 depth in millimetres
        bbox: (x, y, w, h)
    Returns:
        dist_mm: distance in millimetres (median of ROI), or 0 if no valid pixels
    """
    x, y, w, h = bbox
    # Slightly shrink the ROI to avoid edges
    pad_w = int(0.1 * w)
    pad_h = int(0.1 * h)
    x1 = x + pad_w
    y1 = y + pad_h
    x2 = x + w - pad_w
    y2 = y + h - pad_h
    roi = depth_mm[y1:y2, x1:x2]
    # Mask out invalid (zero) depths
    valid = roi[roi > 0]
    if valid.size == 0:
        return 0
    return int(np.median(valid))

def main():
    cv2.namedWindow("RGB")
    cv2.namedWindow("Depth")

    print("Press ESC in any window to quit.")

    while True:
        depth_mm, rgb_bgr = get_frames()

        # Detect people in the RGB frame
        gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)
        rects, _ = hog.detectMultiScale(
            gray,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05
        )

        detections = []
        for (x, y, w, h) in rects:
            dist_mm = estimate_distance(depth_mm, (x, y, w, h))
            if dist_mm > 0:
                detections.append((x, y, w, h, dist_mm))

        # Draw bounding boxes and distance labels
        draw_detections(rgb_bgr, detections)

        # Prepare a visualisable depth image (8-bit grayscale)
        depth_display = np.clip(depth_mm, 0, MAX_DEPTH_MM).astype(np.float32)
        depth_display = (depth_display * DEPTH_DISPLAY_SCALE).astype(np.uint8)

        # Show side by side
        cv2.imshow("RGB", rgb_bgr)
        cv2.imshow("Depth", depth_display)

        # Exit on ESC
        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break

    cv2.destroyAllWindows()
    freenect.sync_stop()

if __name__ == "__main__":
    main()
