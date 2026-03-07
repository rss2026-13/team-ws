# Scripts

## evaluate_rosbag_distances.py

Evaluates wall distance from lidar for each rosbag in `rosbag_data/` using the four branch methods (muktha, tissany, ansisg, jeryl), then writes:

- **muktha_evaluation_stats.csv** – per-bag, per-method: count, mean, std, min, max distance
- **muktha_evaluation_stats.md** – same data as a markdown table

**Requirements:** `numpy`, `scipy`, and `rosbags` (all installed by uv). No ROS 2 needed on macOS; on Linux with ROS 2 sourced it will use `rosbag2_py` instead.

**Run from repo root:**

```bash
uv run scripts/evaluate_rosbag_distances.py
```

On Linux with ROS 2 you can instead source then run:

```bash
source /opt/ros/humble/setup.bash   # optional: use ROS reader
uv run scripts/evaluate_rosbag_distances.py
```

Output: `muktha_evaluation_stats.csv` and `muktha_evaluation_stats.md` in the repo root.
