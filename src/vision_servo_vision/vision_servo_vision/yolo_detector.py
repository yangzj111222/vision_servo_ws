"""YOLOv11-pose TensorRT 推理封装（带 letterbox 预处理 + 反变换）.

关键设计：
  - letterbox 预处理：保持宽高比，灰边填充（Ultralytics 默认）
  - 反变换：把归一化坐标还原到原图坐标
  - 参数化：关键点数量、输入名、置信度阈值都可通过构造参数调整
"""

import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit


class YOLODetector:
    def __init__(self,
                 engine_path,
                 conf_threshold=0.5,
                 img_size=640,
                 num_kpts=4,
                 input_name="images"):
        """
        Args:
            engine_path:      TensorRT .engine 文件路径
            conf_threshold:   置信度阈值，低于此值认为没检测到
            img_size:         模型输入尺寸（正方形边长），必须和训练时一致
            num_kpts:         关键点数量（你的模型是 4）
            input_name:       输入张量名，用 netron 或打印确认
        """
        self.conf_threshold = float(conf_threshold)
        self.img_size = int(img_size)
        self.num_kpts = int(num_kpts)
        self.input_name = input_name

        # ─────────────────────────────────────────────────
        # ⚠️ 加载 engine
        #   - .engine 文件和 GPU 型号、TensorRT 版本绑定
        #   - 换机器必须重新导出
        # ─────────────────────────────────────────────────
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        if self.engine is None:
            raise RuntimeError(f"Failed to load engine: {engine_path}")

        self.context = self.engine.create_execution_context()

        # ─────────────────────────────────────────────────
        # ⚠️ 打印 engine 信息（调试用，确认后注释掉）
        # ─────────────────────────────────────────────────
        # print("=== Engine Info ===")
        # for i in range(self.engine.num_io_tensors):
        #     name = self.engine.get_tensor_name(i)
        #     mode = self.engine.get_tensor_mode(name)
        #     shape = self.engine.get_tensor_shape(name)
        #     print(f"  {name}: mode={mode}, shape={shape}")
        # print("===================")

        # ─────────────────────────────────────────────────
        # 分配输入显存
        # ─────────────────────────────────────────────────
        self.input_shape = self.engine.get_tensor_shape(self.input_name)
        self.d_input = cuda.mem_alloc(int(np.prod(self.input_shape)) * 4)

        # ─────────────────────────────────────────────────
        # 分配输出显存（可能有多个输出）
        # ─────────────────────────────────────────────────
        self.output_names = []
        self.d_outputs = {}
        self.h_outputs = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
                self.output_names.append(name)
                shape = self.engine.get_tensor_shape(name)
                size = int(np.prod(shape)) * 4
                self.d_outputs[name] = cuda.mem_alloc(size)
                self.h_outputs[name] = np.empty(shape, dtype=np.float32)

        # ─────────────────────────────────────────────────
        # ⚠️ 打印输出名（调试用，确认后注释掉）
        # ─────────────────────────────────────────────────
        # print(f"output_names: {self.output_names}")

        self.stream = cuda.Stream()

        # ──────────────────────────────────联调前必须用 print_engine_info() 确认 engine 形状。───────────────
        # letterbox 参数（每次 infer 时更新）
        # ─────────────────────────────────────────────────
        self._scale = 1.0
        self._pad_x = 0
        self._pad_y = 0

        # 最近一次的置信度（供上层使用）
        self.last_conf = 0.0

    # ==================================================================
    # 预处理：letterbox
    # ==================================================================
    def preprocess(self, img):
        """
        输入: BGR 图像 (H, W, 3)
        输出: (input_tensor, scale, pad_x, pad_y)
            input_tensor: [1, 3, img_size, img_size] float32
            scale:        缩放比例（原图 → letterbox）
            pad_x, pad_y: 左右/上下填充像素
        """
        h, w = img.shape[:2]

        # ─────────────────────────────────────────────────
        # ⚠️ 如果训练时用了 BGR，这里改成不转
        #   Ultralytics 默认 RGB，保持 cvtColor
        # ─────────────────────────────────────────────────
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 等比缩放
        scale = min(self.img_size / h, self.img_size / w)
        nh, nw = int(round(h * scale)), int(round(w * scale))
        resized = cv2.resize(img_rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)

        # 灰边填充（114 是 YOLO 默认填充值）
        canvas = np.full((self.img_size, self.img_size, 3), 114, dtype=np.uint8)
        pad_x = (self.img_size - nw) // 2
        pad_y = (self.img_size - nh) // 2
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized

        # ─────────────────────────────────────────────────
        # ⚠️ 如果训练时归一化方式不同（比如 mean/std），这里改
        #   Ultralytics 默认 /255
        # ─────────────────────────────────────────────────
        img_norm = canvas.astype(np.float32) / 255.0

        # HWC → CHW → NCHW
        img_chw = img_norm.transpose(2, 0, 1)
        input_tensor = np.ascontiguousarray(img_chw[np.newaxis, ...])

        return input_tensor, scale, pad_x, pad_y

    # ==================================================================
    # 推理
    # ==================================================================
    def infer(self, img):
        """
        输入: BGR 图像 (H, W, 3)
        输出: (num_kpts, 2) numpy array（原图坐标系），或 None
        """
        # 预处理
        input_tensor, scale, pad_x, pad_y = self.preprocess(img)

        # 保存 letterbox 参数，供 _parse_output 反变换用
        self._scale = scale
        self._pad_x = pad_x
        self._pad_y = pad_y

        # CPU → GPU
        cuda.memcpy_htod_async(self.d_input, input_tensor, self.stream)

        # 绑定地址
        self.context.set_tensor_address(self.input_name, int(self.d_input))
        for name in self.output_names:
            self.context.set_tensor_address(name, int(self.d_outputs[name]))

        # 异步推理
        self.context.execute_async_v3(stream_handle=self.stream.handle)

        # GPU → CPU
        for name in self.output_names:
            cuda.memcpy_dtoh_async(self.h_outputs[name],
                                   self.d_outputs[name],
                                   self.stream)
        self.stream.synchronize()

        # 解析
        return self._parse_output()

    # ==================================================================
    # 输出解析
    # ==================================================================
    def _parse_output(self):
        """
        解析 YOLOv11-pose 输出.

        假设输出 shape 为 [1, C, N] 或 [1, N, C]，其中：
          C = 4 (box xywh) + 1 (conf) + num_kpts * 3 (x, y, conf)
        """
        if not self.output_names:
            return None

        output = self.h_outputs[self.output_names[0]]

        # ─────────────────────────────────────────────────
        # ⚠️ 打印原始输出形状（调试用，确认后注释掉）
        # ─────────────────────────────────────────────────
        # print(f"raw output shape: {output.shape}")

        # ─────────────────────────────────────────────────
        # 形状统一为 [N, C]
        # ─────────────────────────────────────────────────
        if output.ndim == 3:
            # [1, C, N] 或 [1, N, C]
            if output.shape[1] < output.shape[2]:
                # [1, C, N] → [1, N, C]
                output = output.transpose(0, 2, 1)
            output = output[0]  # [N, C]

        # ─────────────────────────────────────────────────
        # 校验通道数
        # ─────────────────────────────────────────────────
        expected_c = 4 + 1 + self.num_kpts * 3
        if output.shape[1] < expected_c:
            # 通道数不对，可能是别的输出格式
            return None

        # ─────────────────────────────────────────────────
        # 取置信度最高的检测
        # ─────────────────────────────────────────────────
        conf = output[:, 4]
        best_idx = int(np.argmax(conf))
        self.last_conf = float(conf[best_idx])

        if self.last_conf < self.conf_threshold:
            return None

        # ─────────────────────────────────────────────────
        # 提取关键点
        # ─────────────────────────────────────────────────
        kp_start = 5
        kps_flat = output[best_idx, kp_start:kp_start + self.num_kpts * 3]
        kps = kps_flat.reshape(self.num_kpts, 3)  # [num_kpts, 3] = (x, y, conf)

        # YOLO 输出已经是 letterbox 图像素坐标，不要再乘 img_size
        kps_xy = kps[:, :2].copy()

        # ─────────────────────────────────────────────────
        # 反 letterbox：letterbox 坐标 → 原图坐标
        #   orig_x = (letterbox_x - pad_x) / scale
        #   orig_y = (letterbox_y - pad_y) / scale
        # ─────────────────────────────────────────────────
        kps_xy[:, 0] = (kps_xy[:, 0] - self._pad_x) / self._scale
        kps_xy[:, 1] = (kps_xy[:, 1] - self._pad_y) / self._scale

        # ─────────────────────────────────────────────────
        # ⚠️ 打印关键点（调试用，确认后注释掉）
        # ─────────────────────────────────────────────────
        # print(f"keypoints (orig frame):\n{kps_xy}")

        return kps_xy

    # ==================================================================
    # 调试辅助
    # ==================================================================
    def infer_with_conf(self, img):
        """返回 (kps, conf)，kps 或 conf 无效时为 (None, 0.0)."""
        kps = self.infer(img)
        return kps, self.last_conf

    def print_engine_info(self):
        """打印 engine 的所有输入输出信息（调试用）."""
        print("=== Engine Info ===")
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            shape = self.engine.get_tensor_shape(name)
            print(f"  {name}: mode={mode}, shape={shape}")
        print("===================")