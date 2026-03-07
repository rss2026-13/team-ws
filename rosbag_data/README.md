# ROS 2 bag data

This folder holds rosbag recordings (e.g. `straight_inward_*` runs) for the team workspace.

## How to move bags into this folder

**1. Open Terminal and go to the repo root (where you see the `straight_inward_*` folders):**

```bash
cd /Users/muktharamesh/Documents/team-ws
```

**2. Move all bag folders into `rosbag_data/`:**

```bash
mv straight_inward_ansis_left_1 straight_inward_ansis_left_2 straight_inward_ansis_left_3 \
   straight_inward_ansis_right_1 straight_inward_ansis_right_2 straight_inward_ansis_right_3 \
   straight_inward_muktha_left_1 straight_inward_muktha_left_2 straight_inward_muktha_left_3 \
   straight_inward_muktha_right_1 straight_inward_muktha_right_2 straight_inward_muktha_right_3 \
   straight_inward_tissany_left_1 straight_inward_tissany_left_2 straight_inward_tissany_left_3 \
   straight_inward_tissany_right_1 straight_inward_tissany_right_2 straight_inward_tissany_right_3 \
   rosbag_data/
```

If you only have some of these, list the ones you have:

```bash
ls -d straight_inward_*
```

Then move them (replace with your actual names if different):

```bash
mv straight_inward_* rosbag_data/
```

**3. Commit and push to main:**

```bash
git add rosbag_data/
git status
git commit -m "Move rosbag data into rosbag_data/ on main"
git push origin main
```

## Playing back

From the workspace (with ROS 2 sourced):

```bash
ros2 bag play rosbag_data/<bag_folder_name>
```

## Note

- Each bag is a directory containing `metadata.yaml` and one or more `.db3` files.
- If bags are large, consider using Git LFS for `*.db3` and `metadata.yaml`, or store them in shared storage and keep only a manifest or links in the repo.
