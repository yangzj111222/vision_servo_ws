"""关键点 → 物理量."""

import numpy as np


def extract_features(keypoints, img_cx):
    """
    keypoints: (4,2) numpy array
    返回: center_x, h_left, h_right, h_avg
    """
    x0, y0 = keypoints[0]   # 左上
    x1, y1 = keypoints[1]   # 右上
    x2, y2 = keypoints[2]   # 右下
    x3, y3 = keypoints[3]   # 左下

    center_x = (x0 + x1 + x2 + x3) / 4.0
    h_left = y3 - y0
    h_right = y2 - y1
    h_avg = (h_left + h_right) / 2.0

    return center_x, h_left, h_right, h_avg


def compute_errors(center_x, h_left, h_right, h_avg, img_cx):
    """计算两个误差量（e_z 由控制节点根据 h_avg_target 算）."""
    e_x = center_x - img_cx
    e_yaw = (h_left - h_right) / max(h_avg, 1.0)
    return e_x, e_yaw