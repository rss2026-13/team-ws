# ROS 2 bag data

This folder holds rosbag recordings (e.g. `straight_inward_*` runs) for the team workspace.

## Adding existing bags from repo root to main

If you have bag directories (e.g. `straight_inward_muktha_right_1`, `straight_inward_tissany_left_2`, …) in the repo root, move them here and commit on `main`:

```bash
# From repo root
mv straight_inward_* rosbag_data/
git add rosbag_data/
git status   # confirm all bags are staged
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
