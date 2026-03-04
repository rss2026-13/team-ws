#!/usr/bin/env python3
import numpy as np


def segment_points(points, window_size=30, overlap=10):
    segments = []
    for i in range(0, len(points) - window_size + 1, window_size - overlap):
        segment = points[i : i + window_size]
        segments.append(segment)
    return segments


def IEPF(points, D_t):
    start = 0
    end = len(points) - 1
    line_vec = points[end] - points[start]
    line_vec /= np.linalg.norm(line_vec)
    point_vecs = points - points[start]
    projections = point_vecs @ line_vec
    closest_points = np.outer(projections, line_vec) + points[start]
    distances = np.linalg.norm(points - closest_points, axis=1)
    max_index = np.argmax(distances)
    if distances[max_index] > D_t:
        left_segments = IEPF(points[: max_index + 1], D_t)
        right_segments = IEPF(points[max_index:], D_t)
        return left_segments + right_segments
    else:
        return [points]


def fit_segment(points):
    mean = points.mean(axis=0)
    centered = points - mean
    _, _, vh = np.linalg.svd(centered)
    direction = vh[0]

    t = centered @ direction
    t_min, t_max = t.min(), t.max()

    p1 = mean + t_min * direction
    p2 = mean + t_max * direction

    return p1, p2


def detect_walls(scan, side, min_points=2, D_t=0.5, max_dist=5):

    angle_min = scan.angle_min
    angle_max = scan.angle_max
    ranges = np.array(scan.ranges)
    angles = np.linspace(angle_min, angle_max, num=ranges.shape[0])
    # mask = ranges < max_dist
    # ranges = ranges[mask]
    # angles = angles[mask]
    points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T
    clusters = IEPF(np.array(points), D_t=D_t)
    walls = []
    for cluster in clusters:
        if len(cluster) < min_points:
            continue
        wall = fit_segment(cluster)
        walls.append(wall)
    return np.array(walls)
