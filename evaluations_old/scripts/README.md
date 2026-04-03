# Scripts

## evaluate_rosbag_distances.py

Evaluates wall distance from lidar for each rosbag in `rosbag_data/` using the four branch methods (muktha, tissany, ansisg, jeryl), then writes:

- **evaluation_stats.csv** – per-bag, per-method (sorted by method then bag): count, mean, std, min, max distance
- **evaluation_stats.md** – same data as markdown tables, one section per method, plus a short explanation of metrics

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

Output: `evaluation_stats.csv` and `evaluation_stats.md` in the repo root. Progress is logged (bag index, scan count, write steps).

---

## plot_median_runs.py

Overlays **three runs** (run 1, 2, 3) for each **method** and **side** (left/right), plotting distance vs scan index. Uses the same distance logic and tissany_left truncation as the evaluation script (imports from `evaluate_rosbag_distances`).

**Output:** One figure per method in `evaluation_plots/`:

- `muktha_three_runs.png`, `tissany_three_runs.png`, `ansisg_three_runs.png`, `jeryl_three_runs.png` – each has 2 subplots (left, right) with 3 runs overlaid and a 0.6 m desired line.
- `median_of_4_three_runs.png` – 4×2 subplots (one per runner and side), 3 runs overlaid each.

**Run from repo root:**

```bash
uv run scripts/plot_median_runs.py
```

Requires `matplotlib` (added to project dependencies).

---

## plot_by_robot_method.py

Same 3-run overlay layout, but each subplot uses the **method that matches the robot/runner**:
- muktha's 3 left/right runs → **muktha** method distance
- tissany's runs → **tissany** method
- ansis's runs → **ansisg** method
- jeryl's runs → **jeryl** method

**Output:** One figure `evaluation_plots/by_robot_method_three_runs.png` with 4×2 subplots (runner × left/right), each with 3 runs overlaid and a 0.6 m desired line. Subplot titles include the method used (e.g. `muktha left (muktha)`).

**Run from repo root:**

```bash
uv run scripts/plot_by_robot_method.py
```
