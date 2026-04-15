import numpy as np
import matplotlib.pyplot as plt
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import rosbag2_py
import sys

def read_bag_data(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Could not open bag: {e}")
        return None

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    data = {
        "/trajectory/cte": [],
        "/trajectory/heading_error": [],
        "/trajectory/lookahead": [],
        "/trajectory/steering_angle": []
    }

    while reader.has_next():
        (topic, msg_data, t) = reader.read_next()
        if topic in data:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(msg_data, msg_type)
            data[topic].append([t / 1e9, msg.data])

    for topic in data:
        if data[topic]:
            arr = np.array(data[topic])
            arr[:, 0] -= arr[0, 0] # Normalize time
            data[topic] = arr
        else:
            data[topic] = np.empty((0, 2))
    return data

def plot_metrics(bag_data):
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Pure Pursuit Evaluation', fontsize=14)

    topics = [
        ("/trajectory/cte", "Cross-Track Error (m)", axs[0, 0], 'red'),
        ("/trajectory/heading_error", "Heading Error (rad)", axs[0, 1], 'blue'),
        ("/trajectory/lookahead", "Lookahead Distance (m)", axs[1, 0], 'green'),
        ("/trajectory/steering_angle", "Steering Angle (rad)", axs[1, 1], 'purple')
    ]

    for topic, label, ax, color in topics:
        data_arr = bag_data[topic]
        if data_arr.size > 0:
            ax.plot(data_arr[:, 0], data_arr[:, 1], color=color)
            ax.set_title(label)
            ax.grid(True, alpha=0.3)
            if "cte" in topic:
                rmse = np.sqrt(np.mean(data_arr[:, 1]**2))
                ax.set_xlabel(f"RMSE: {rmse:.4f}")

    plt.tight_layout()
    plt.show()

def main(args=None):
    # You can pass the bag path as an argument or hardcode it here
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        # Update this to your default bag folder
        bag_path = "path_to_your_bag_folder" 

    print(f"Loading bag from: {bag_path}")
    results = read_bag_data(bag_path)
    
    if results:
        plot_metrics(results)

if __name__ == "__main__":
    main()
