import cv2
import numpy as np
from matplotlib import pyplot as plt

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################


def image_print(img):
    plt.imshow(img)
    plt.show()


def visualize_test():
    img = cv2.imread(
        "src/visual_servoing/visual_servoing/computer_vision/test_images_cone/cone_template.png"
    )
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(img_hsv)
    plt.hist(h.ravel(), 180)
    plt.show()
    plt.hist(s.ravel())
    plt.show()
    plt.hist(v.ravel())
    plt.show()


def get_bounds(percentile=5):
    img = cv2.imread(
        "src/visual_servoing/visual_servoing/computer_vision/test_images_cone/cone_template.png"
    )
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # return the percentile of h s and v as an array of lower bounds and arra y of upper bounds
    h, s, v = cv2.split(img_hsv)
    h = h[h > 0]
    s = s[s > 0]
    v = v[v > 0]
    h_lower = np.percentile(h, percentile)
    h_upper = np.percentile(h, 100 - percentile)
    s_lower = np.percentile(s, percentile)
    s_upper = np.percentile(s, 100 - percentile)
    v_lower = np.percentile(v, percentile)
    v_upper = np.percentile(v, 100 - percentile)
    return (h_lower, s_lower, v_lower), (h_upper, s_upper, v_upper)


def cd_color_segmentation(
    img,
    template,
    debug=False,
):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template: Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
            (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
    bounds = ((5, 210, 110), (30, 255, 255))
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if debug:
        image_print(img_hsv)
    img_filtered = cv2.inRange(img_hsv, bounds[0], bounds[1])
    if debug:
        image_print(img_filtered)
    kernel = np.ones((3, 3), np.uint8)
    img_final = img_filtered
    img_final = cv2.morphologyEx(img_final, cv2.MORPH_CLOSE, kernel)
    if debug:
        image_print(img_final)
    countours = cv2.findContours(img_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[
        0
    ]
    if len(countours) == 0:
        return None
    biggest_box = None
    for contour in countours:
        box = cv2.boundingRect(contour)
        if biggest_box is None or min(box[2] * 1.5, box[3]) > min(
            biggest_box[2] * 1.5, biggest_box[3]
        ):
            biggest_box = box
    bounding_box = (
        (biggest_box[0], biggest_box[1]),
        (biggest_box[0] + biggest_box[2], biggest_box[1] + biggest_box[3]),
    )
    return bounding_box


if __name__ == "__main__":
    # visualize_test()
    # bounds = get_bounds(5)
    bounds = ((5, 190, 192), (30, 255, 255))
    print(bounds)
    img = cv2.imread(
        "/home/racecar/racecar_ws/src/visual_servoing/visual_servoing/computer_vision/test_images_cone/test9.jpg"
    )
    bbox = cd_color_segmentation(img, None, True)
    print(bbox)
    img = cv2.rectangle(img, bbox[0], bbox[1], (0, 255, 0), 1)
    image_print(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
