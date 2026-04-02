import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

def main():
    bag_path = "/home/racecar/racecar_ws/P200_R100_highnoise_T3/P200_R100_highnoise_T3_0.db3"
    
    tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=20))
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    results = []
    pf_message_count = 0
    start_time = None
    end_time = None

    print("Processing bag...")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if start_time is None: start_time = t
        end_time = t

        # 1. Store TF data so we can look up error
        if topic in ['/tf', '/tf_static']:
            msg = deserialize_message(data, TFMessage)
            for transform in msg.transforms:
                tf_buffer.set_transform(transform, "bag_authority")

        # 2. Use the PF Odom topic as clock for our FPS
        if topic == '/pf/pose/odom':
            pf_message_count += 1
            
            try:
                # Calculate error at  moment the PF published
                gt = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                pf = tf_buffer.lookup_transform('map', 'base_link_pf', rclpy.time.Time())
                
                dist = np.sqrt((gt.transform.translation.x - pf.transform.translation.x)**2 + 
                               (gt.transform.translation.y - pf.transform.translation.y)**2)
                results.append([t / 1e9, dist])
            except TransformException:
                continue

    # --- Calculations ---
    total_duration = (end_time - start_time) / 1e9
    # This is your ACTUAL algorithm speed
    true_fps = pf_message_count / total_duration

    data = np.array(results)
    times = data[:, 0] - data[0, 0]
    errors = data[:, 1]

    # 1. Find the peak (relocation point)
    peak_idx = np.argmax(errors)
    peak_time = times[peak_idx]
    
    # 2. Slice data to look after the peak
    post_peak_times = times[peak_idx:]
    post_peak_errors = errors[peak_idx:]

    # 3. Calculate Steady State (last 20% of the entire run)
    steady_state_start_idx = int(len(errors) * 0.8)
    steady_state_error = np.mean(errors[steady_state_start_idx:])

    # 4. Define Convergence: When error first drops below (Steady State + 5cm) 
    # after the peak occurred.
    convergence_threshold = steady_state_error + 0.05
    
    # Look for the first index in the post-peak data that hits the threshold
    conv_indices = np.where(post_peak_errors <= convergence_threshold)[0]
    
    if len(conv_indices) > 0:
        # Time from peak to convergence
        absolute_conv_time = post_peak_times[conv_indices[0]]
        convergence_duration = absolute_conv_time - peak_time
        conv_display_val = f"{convergence_duration:.2f} s (after peak)"
    else:
        absolute_conv_time = None
        convergence_duration = "N/A"
        conv_display_val = "N/A"

    print("\n" + "="*35)
    print(f"ALGORITHM PERFORMANCE REPORT")
    print(f"True Publishing Rate: {true_fps:.2f} Hz")
    print("-" * 35)
    print(f"Steady State Error:  {steady_state_error:.4f} m")
    print(f"Peak Error Detected: {errors[peak_idx]:.2f} m at {peak_time:.2f}s")
    #print(f"Convergence Rate:    {conv_display_val}")
    print("="*35 + "\n")

    # --- Plotting ---
    plt.figure(figsize=(10, 6))
    plt.plot(times, errors, label='Positional Error', color='royalblue')
    
    # Draw the Steady State line
    plt.axhline(y=steady_state_error, color='green', linestyle='--', 
                label=f'Steady State ({steady_state_error:.2f}m)')
    
    # Mark the Peak
    #plt.scatter(peak_time, errors[peak_idx], color='orange', s=100, marker='*', label='Relocation Peak', zorder=6)

    # Mark the Convergence Point
    #if absolute_conv_time:
        #plt.scatter(absolute_conv_time, errors[peak_idx + conv_indices[0]], 
                    #color='red', s=50, label='Converged', zorder=5)

    plt.ylim(0, 2)
    plt.xlim(0,40)

    ax = plt.gca()

    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))

    plt.grid(which='minor', alpha=0.2, linestyle=':')
    plt.grid(which='major', alpha=0.5)

    plt.title(f"Localization Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
