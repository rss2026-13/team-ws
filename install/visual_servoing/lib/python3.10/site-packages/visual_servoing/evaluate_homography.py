import rosbag2_py
import numpy as np
import cv2
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from vs_msgs.msg import ConeLocation

def get_meters_from_pixels(u, v):
    # --- UPDATE THESE 9 NUMBERS WITH YOUR ACTUAL H MATRIX ---
    H = np.array([[ 2.82539285e-04, -2.13948377e-03, -3.61545803e-01],
 [ 1.73331098e-03, -4.58405936e-04, -5.17824897e-01],
 [ 4.26226589e-04, -8.41180393e-03,  1.00000000e+00]]) 
    
    pixel_point = np.array([u, v, 1.0])
    ground_point = np.dot(H, pixel_point)
    ground_point /= ground_point[2]
    return ground_point[0], ground_point[1]

def process_bag(bag_path, gt_x, gt_y):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    reader.open(storage_options, rosbag2_py.ConverterOptions('', ''))

    calc_xs, calc_ys, errors = [], [], []
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/relative_cone':
            msg = deserialize_message(data, ConeLocation)
            
            calc_xs.append(msg.x_pos)
            calc_ys.append(msg.y_pos)
            errors.append(np.sqrt((msg.x_pos - gt_x)**2 + (msg.y_pos - gt_y)**2))
    if errors:
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        print(f"Number of points: {len(errors)}")
        print(f"Mean Error: {mean_error:.4f} meters")
        print(f"Max Error:  {max_error:.4f} meters")
    else:
        print("No data points were found in the bag for /relative_cone.")


    # --- PLOTTING ---
    plt.figure(figsize=(8, 6))
    plt.scatter(calc_xs, calc_ys, color='blue', label='Calculated Points (Clicks)')
    plt.scatter(gt_x, gt_y, color='red', marker='X', s=100, label='Ground Truth')
    
    plt.title(f"Accuracy Test: Truth at ({gt_x}m, {gt_y}m)")
    plt.xlabel("Distance Forward (x) [m]")
    plt.ylabel("Distance Lateral (y) [m]")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    
    # Save the plot for your report
    plot_name = f"accuracy_plot_{gt_x}m.png"
    plt.savefig(plot_name)
    print(f"Plot saved as {plot_name}")
    plt.show()

# --- CHANGE THESE FOR EACH TEST ---
process_bag('/root/racecar_ws/src/visual_servoing/visual_servoing/rosbags_data/60cmfront17cmright', gt_x=0.6, gt_y=-0.17)
