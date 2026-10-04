"""时域滤波：EMA 平滑 + 丢失计数."""

import numpy as np


class TemporalFilter:
    def __init__(self, alpha=0.3, lost_tolerance=10):
        self.alpha = alpha
        self.lost_tolerance = lost_tolerance

        self.f_kps = None
        self.f_center_x = None
        self.f_h_left = None
        self.f_h_right = None
        self.f_h_avg = None

        self.lost_count = 0

    def update_keypoints(self, kps):
        """kps: (4,2) numpy array"""
        if self.f_kps is None:
            self.f_kps = kps.copy()
        else:
            a = self.alpha
            self.f_kps = a * kps + (1 - a) * self.f_kps
        return self.f_kps

    def update_features(self, center_x, h_left, h_right, h_avg):
        a = self.alpha
        if self.f_center_x is None:
            self.f_center_x = center_x
            self.f_h_left = h_left
            self.f_h_right = h_right
            self.f_h_avg = h_avg
        else:
            self.f_center_x = a * center_x + (1 - a) * self.f_center_x
            self.f_h_left   = a * h_left   + (1 - a) * self.f_h_left
            self.f_h_right  = a * h_right  + (1 - a) * self.f_h_right
            self.f_h_avg    = a * h_avg    + (1 - a) * self.f_h_avg

        return (self.f_center_x, self.f_h_left,
                self.f_h_right, self.f_h_avg)

    def on_detection(self, detected):
        if detected:
            self.lost_count = 0
        else:
            self.lost_count += 1

    def is_lost(self):
        return self.lost_count > self.lost_tolerance

    def reset(self):
        self.f_kps = None
        self.f_center_x = None
        self.f_h_left = None
        self.f_h_right = None
        self.f_h_avg = None
        self.lost_count = 0