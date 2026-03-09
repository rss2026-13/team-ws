#!/usr/bin/env python3
import numpy as np


def point_dir(wall, dir, margin=0.3):
    p1, p2 = wall
    line_vec = p2 - p1
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return None
    line_vec /= line_len
    dir_vec = np.array(dir)
    dir_vec /= np.linalg.norm(dir_vec)
    det = line_vec[0] * dir_vec[1] - line_vec[1] * dir_vec[0]
    if det == 0:
        return None
    t = (dir_vec[0] * (p1[1] - 0) - dir_vec[1] * (p1[0] - 0)) / det
    u = (line_vec[0] * (p1[1] - 0) - line_vec[1] * (p1[0] - 0)) / det
    if t < -margin or t > line_len + margin or u < 0:
        return None
    return p1 + t * line_vec


def angles_points_from_scan(scan):
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    ranges = np.array(scan.ranges)
    angles = np.linspace(angle_min, angle_max, num=ranges.shape[0])
    points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T
    return angles, points
