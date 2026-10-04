"""视觉伺服控制核心逻辑（含接近阶段）."""

import time
import numpy as np
from geometry_msgs.msg import Twist


class AlignController:
    def __init__(self, params):
        self.img_cx = params['img_cx']
        self.h_avg_target = params['h_avg_target']

        self.tol_x = params['tol_x']
        self.tol_yaw = params['tol_yaw']
        self.tol_z = params['tol_z']

        self.dead_x = params['dead_x']
        self.dead_yaw = params['dead_yaw']
        self.dead_z = params['dead_z']

        self.kp_x = params['kp_x']
        self.kp_yaw = params['kp_yaw']
        self.kp_z = params['kp_z']

        self.vmax = params['vmax']
        self.wmax = params['wmax']

        self.aligned_frames = params['aligned_frames']
        self.servo_timeout = params['servo_timeout']

        # ═════════════════════════════════════════════════════════════
        # ★★★ 接近阶段参数（需要现场标定和调整） ★★★
        # ═════════════════════════════════════════════════════════════
        # 相机到夹爪尖端的距离（mm）
        #   = 相机到车头(约 470) + 夹爪长度(约 170)
        #   ⚠️ 换相机位置或换夹爪时，这个值要改
        self.grab_distance_mm = params.get('grab_distance_mm', 640.0)

        # 接近阶段允许的额外余量（mm）
        #   实际距离 > grab_distance + approach_margin 时才进入接近
        #   ⚠️ 调整这个值可以控制"什么时候开始精细对齐"
        #   值越大：允许目标更远就开始精细对齐（但可能对不准）
        #   值越小：要靠近到很近才开始精细对齐（更保守）
        self.approach_margin_mm = params.get('approach_margin_mm', 100.0)

        # 接近阶段的最大线速度（m/s）
        #   ⚠️ 值越大，靠近越快，但可能冲过头
        self.approach_vmax = params.get('approach_vmax', 0.25)

        # 接近阶段的增益（误差米 → 速度）
        #   ⚠️ 值越大，靠近越快，但可能震荡
        self.kp_z_approach = params.get('kp_z_approach', 0.5)

        # ═════════════════════════════════════════════════════════════

        self.state = "IDLE"
        self.phase = "APPROACH"       # ★ 新增：阶段标志
        self.aligned_count = 0
        self.start_time = None
        self.features = None
        self.lost_count = 0

    def start(self):
        self.state = "SERVO"
        self.phase = "APPROACH"       # ★ 重置阶段
        self.aligned_count = 0
        self.start_time = time.time()
        self.features = None
        self.lost_count = 0

    def on_features(self, msg):
        if not msg.valid:
            self.lost_count += 1
            return
        self.lost_count = 0
        self.features = msg

    def compute_cmd(self):
        """返回 (Twist, status_str)"""
        cmd = Twist()

        if self.state == "ALIGNED_STOP":
            return cmd, "ALIGNED_STOP"

        if self.state != "SERVO":
            return cmd, "IDLE"

        # 超时
        if time.time() - self.start_time > self.servo_timeout:
            self.state = "IDLE"
            return cmd, "TIMEOUT"

        # 目标丢失
        if self.lost_count > 30:
            self.state = "IDLE"
            return cmd, "LOST"

        if self.features is None:
            return cmd, "WAIT"

        f = self.features

        # ═════════════════════════════════════════════════════════════
        # 从 h_avg 估算物理距离（mm）
        # ═════════════════════════════════════════════════════════════
        # h_avg 与距离成反比，用标定点反推：
        #   distance = grab_distance × h_avg_target / h_avg
        # ⚠️ h_avg_target 需要在 grab_distance_mm 处标定
        distance_mm = (self.grab_distance_mm * self.h_avg_target
                       / max(f.h_avg, 1.0))

        # ═════════════════════════════════════════════════════════════
        # 阶段切换（带滞后，防抖）
        # ═════════════════════════════════════════════════════════════
        target_distance = self.grab_distance_mm

        if self.phase == "APPROACH":
            # 靠近到阈值以内 → 切到对齐
            if distance_mm < target_distance:
                self.phase = "ALIGN"
        else:  # ALIGN
            # 跑远了（比如被外力推）→ 回到接近
            if distance_mm > target_distance + self.approach_margin_mm:
                self.phase = "APPROACH"

        # ═════════════════════════════════════════════════════════════
        # 阶段 1：APPROACH —— 接近目标
        # ═════════════════════════════════════════════════════════════
        if self.phase == "APPROACH":
            # 距离误差（米）
            e_z_m = (distance_mm - target_distance) / 1000.0

            # 前后速度（比例控制）
            vx = self.kp_z_approach * e_z_m

            # 接近时不做横向平移，专心靠近
            vy = 0.0

            # 偏航修正（边走边转，保持朝向）
            e_yaw = (f.h_left - f.h_right) / max(f.h_avg, 1.0)
            wz = self.kp_yaw * e_yaw if abs(e_yaw) > self.dead_yaw else 0.0

            # 限速
            vx = float(np.clip(vx, -self.approach_vmax, self.approach_vmax))
            wz = float(np.clip(wz, -self.wmax, self.wmax))

            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz

            return cmd, "APPROACH"

        # ═════════════════════════════════════════════════════════════
        # 阶段 2：ALIGN —— 精细对齐（原有逻辑）
        # ═════════════════════════════════════════════════════════════
        e_x = f.center_x - self.img_cx
        e_yaw = (f.h_left - f.h_right) / max(f.h_avg, 1.0)
        e_z_px = self.h_avg_target - f.h_avg   # 像素误差，用于精细对齐

        # 分阶段：先偏航，后平移
        if abs(e_yaw) > self.tol_yaw:
            vx, vy = 0.0, 0.0
            wz = self.kp_yaw * e_yaw if abs(e_yaw) > self.dead_yaw else 0.0
        else:
            wz = 0.0
            vx = self.kp_z * e_z_px if abs(e_z_px) > self.dead_z else 0.0
            vy = self.kp_x * e_x if abs(e_x) > self.dead_x else 0.0

        vx = float(np.clip(vx, -self.vmax, self.vmax))
        vy = float(np.clip(vy, -self.vmax, self.vmax))
        wz = float(np.clip(wz, -self.wmax, self.wmax))

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz

        # 收敛判断
        aligned = (abs(e_x) < self.tol_x and
                   abs(e_yaw) < self.tol_yaw and
                   abs(e_z_px) < self.tol_z)

        if aligned:
            self.aligned_count += 1
        else:
            self.aligned_count = 0

        if self.aligned_count >= self.aligned_frames:
            self.state = "ALIGNED_STOP"
            return Twist(), "ALIGNED"

        return cmd, "SERVO"