#!/usr/bin/env python3
"""
test_detection.py - Updated to work with unified detection logic
"""

import argparse
import sys
import os

import cv2
import numpy as np
from matplotlib import pyplot as plt

# Allow running from any directory by importing from the same package location
sys.path.insert(0, os.path.dirname(__file__))
from traffic_light_detection import is_red_light_on, cd_red_light_segmentation


def run_test(image_path: str, debug: bool, show_result: bool) -> bool:
    """
    Run detection on a single image.
    Returns True if red light detected, False otherwise.
    """
    img = cv2.imread(image_path)
    if img is None:
        print(f"  [ERROR] Could not load image: {image_path}")
        return False

    # UPDATED: is_red_light_on now returns (status, bbox) 
    # and no longer uses min_red_pixels
    red_on, bbox = is_red_light_on(img, debug=debug)

    status = "🔴 RED LIGHT ON" if red_on else "🟢 NO RED LIGHT"
    bbox_str = f"bbox={bbox}" if bbox else "bbox=None"
    print(f"  {status}  |  {bbox_str}  |  {os.path.basename(image_path)}")

    if show_result:
        display = img.copy()
        if bbox:
            cv2.rectangle(display, bbox[0], bbox[1], (0, 255, 0), 3)
        
        label = "RED DETECTED" if red_on else "NO RED"
        color = (0, 0, 255) if red_on else (0, 200, 0)
        cv2.putText(display, label, (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        
        plt.figure(figsize=(10, 6))
        plt.title(f"{os.path.basename(image_path)} — {label}")
        plt.imshow(cv2.cvtColor(display, cv2.COLOR_BGR2RGB))
        plt.axis("off")
        plt.tight_layout()
        plt.show()

    return red_on


def main():
    parser = argparse.ArgumentParser(description="Test red traffic light detection on images.")
    parser.add_argument("images", nargs="+", help="Path(s) to test images")
    # Note: --threshold is kept in the parser so your commands don't break, 
    # but it is no longer used by the underlying detection logic.
    parser.add_argument(
        "--threshold", "-t", type=int, default=100,
        help="Legacy threshold (no longer used by unified logic)"
    )
    parser.add_argument(
        "--debug", "-d", action="store_true",
        help="Show HSV masks and intermediate images"
    )
    parser.add_argument(
        "--show", "-s", action="store_true",
        help="Display each result image with bounding box overlay"
    )
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"  Traffic Light Detection Test (Unified Logic)")
    print(f"{'='*60}\n")

    results = []
    for image_path in args.images:
        if not os.path.exists(image_path):
            print(f"  [SKIP] File not found: {image_path}")
            continue
        # Removed args.threshold from the call below
        detected = run_test(image_path, args.debug, args.show)
        results.append((image_path, detected))

    print(f"\n{'='*60}")
    print(f"  Summary: {sum(r[1] for r in results)}/{len(results)} images detected as red")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()

