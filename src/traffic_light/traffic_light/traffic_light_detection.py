import cv2
import numpy as np
from matplotlib import pyplot as plt

def image_print(img):
    # Convert to RGB for correct matplotlib display if it's a color image
    if len(img.shape) == 3:
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    else:
        plt.imshow(img, cmap='gray')
    plt.show()

def cd_red_light_segmentation(img, debug=False):
    """
    Detects red region and returns bounding box.
    """

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Stricter HSV ranges but still allowing farther away lights
    # change saturation and value to do so
    lower_red1 = np.array([0,   120,  180])
    upper_red1 = np.array([45,   255,  255]) #try allowing a little orange and even yellow but amp up the saturation and value needed
    lower_red2 = np.array([175, 120,  180])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
    img_filtered = cv2.bitwise_or(mask1, mask2)

    # Morphological cleanup
    kernel = np.ones((5, 5), np.uint8) # Slightly larger kernel for better noise removal
    img_final = cv2.morphologyEx(img_filtered, cv2.MORPH_OPEN, kernel)
    img_final = cv2.morphologyEx(img_final, cv2.MORPH_CLOSE, kernel)

    if debug:
        print("Debugging: Final Red Mask")
        image_print(img_final)

    contours = cv2.findContours(img_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    if len(contours) == 0:
        return None

    biggest_contour = max(contours, key=cv2.contourArea)

    # Filter out small noise
    if cv2.contourArea(biggest_contour) < 10:
        return None

    x, y, w, h = cv2.boundingRect(biggest_contour)
    return ((x, y), (x + w, y + h)) #offset bounding box to match region of interest cropping


def is_red_light_on(img, debug=False):
    """
    Logic updated: Red is only 'ON' if we can actually find a valid bounding box.
    """
    bbox = cd_red_light_segmentation(img, debug=debug)
    
    if bbox is not None:
        return True, bbox
    return False, None


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python traffic_light_detection.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]
    img = cv2.imread(image_path)

    if img is None:
        print(f"Could not load image: {image_path}")
        sys.exit(1)

    # Now we get both the status and the box at the same time
    red_on, bbox = is_red_light_on(img, debug=True)

    print(f"Red light ON: {red_on}")
    print(f"Bounding box: {bbox}")

    if red_on and bbox is not None:
        display = img.copy()
        cv2.rectangle(display, bbox[0], bbox[1], (0, 255, 0), 3)
        image_print(display)
    else:
        print("No red light detected. Showing original image.")
        image_print(img)



