Vision Servo — 近距离视觉微调对准系统

基于 HP60C 深度相机 + YOLOv11-Pose + TensorRT 的视觉伺服系统。
机器人在雷达导航到达目标 20~50cm 后，视觉接管底盘，2~3 秒内实现高精度居中与垂直对齐，并触发底盘急停。


一、系统框架

┌──────────────────────────────────────────────────────┐
│  HP60C 相机                                          │
│  /ascamera/AsCamera/rgb0/image (640×480 @ 12Hz)      │
└────────────────────┬─────────────────────────────────┘
                     │
                     ▼
┌──────────────────────────────────────────────────────┐
│  vision_node（视觉节点）                              │
│  YOLOv11-Pose 关键点检测 → EMA 滤波 → 物理量提取       │
│  发布：/vision_servo/target_features                 │
└────────────────────┬─────────────────────────────────┘
                     │
                     ▼
┌──────────────────────────────────────────────────────┐
│  align_controller（控制节点）                         │
│  误差计算 → 分阶段比例控制 → 收敛判断 → 急停           │
│  发布：/cmd_vel_vision, /vision_servo/event          │
└────────────────────┬─────────────────────────────────┘
                     │
                     ▼
┌──────────────────────────────────────────────────────┐
│  底盘 + 决策层                                        │
│  决策层发 /vision_servo/start 触发伺服                │
│  决策层收 /vision_servo/event 知道对准完成             │
└──────────────────────────────────────────────────────┘





┌─────────────────────────────────────────────────────────────┐
│  阶段 1：视觉感知                                            │
│                                                             │
│  HP60C 相机                                                  │
│      │ /ascamera/AsCamera/rgb0/image                        │
│      ▼                                                       │
│  vision_node（YOLOv11 + 滤波）                               │
│      │ /vision_servo/target_features                        │
└──────┼──────────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────┐
│  阶段 2：视觉对准                                            │
│                                                             │
│  align_controller                                            │
│      │ 收到 /vision_servo/start（决策层）                     │
│      │ 30Hz 算速度                                           │
│      │                                                       │
│      ├─ /cmd_vel_vision（Twist）→ 底盘                       │
│      ├─ /vision_servo/stop（Bool）→ 底盘                     │
│      └─ /vision_servo/event（String）→ 决策层/grab_node      │
│           = "VISUAL_ALIGNED"                                │
└──────┼──────────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────┐
│  阶段 3：夹取动作                                            │
│                                                             │
│  grab_node                                                   │
│      │ 收到 /vision_servo/event = "VISUAL_ALIGNED"           │
│      │ 30Hz 跑状态机                                         │
│      │                                                       │
│      ├─ /grab/cmd（GrabCommand）→ 桥接节点 → 下位机           │
│      │                                                       │
│      │ ← /grab/status_feedback（GrabStatus）← 下位机          │
│      │                                                       │
│      └─ /grab/event（GrabEvent）→ 决策层                     │
│           = "GRAB_DONE"                                     │
└─────────────────────────────────────────────────────────────┘ 






核心流程：

相机 → YOLOv11 检测 4 角点 → 算 3 个误差 → 比例控制
→ 连续 5 帧进死区 → 急停 → 通知决策层

核心算法：
误差	        公式	                  含义
e_x	     Center_X - img_cx	             横向偏移
e_yaw	 (H_left - H_right) / H_avg	     偏航
e_z	     H_avg_target - H_avg	         距离

分阶段对齐：先偏航（只转），后平移（前后左右）。


二、工作空间结构

~/vision_servo_ws/src/
├── vision_servo_interfaces/     # 消息接口
├── vision_servo_vision/         # 视觉感知节点
├── vision_servo_control/        # 控制节点
└── vision_servo_bringup/        # 一键启动

依赖关系：

interfaces（消息定义）
    │
    ├──► vision（视觉节点，发布特征）
    │
    └──► control（控制节点，订阅特征）
              │
              ▼
          bringup（同时启动 vision + control）

编译顺序：colcon 自动按依赖排序（interfaces → vision/control → bringup）。


三、四个包逐文件说明
1. vision_servo_interfaces/ — 消息接口

定义节点之间通信的消息格式。
text

vision_servo_interfaces/
├── msg/
│   ├── TargetFeatures.msg      # 视觉特征消息
│   └── AlignStatus.msg         # 对准状态消息
├── CMakeLists.txt              # 声明有哪些 msg
└── package.xml

文件	             作用
TargetFeatures.msg	视觉节点发给控制节点的数据：center_x、h_left、h_right、h_avg、e_yaw、conf、valid

AlignStatus.msg	     控制节点发给决策层的状态：status、time_elapsed、message

CMakeLists.txt	用 rosidl_generate_interfaces() 声明 msg 文件，编译时自动生成 C++/Python 代码

注意：消息接口用 CMake 包（不是 Python 包），改动 .msg 后必须清理重编。


2. vision_servo_vision/ — 视觉感知节点

订阅相机图像，跑 YOLOv11-Pose，输出 4 个关键点对应的物理量。
text             

vision_servo_vision/
├── vision_servo_vision/            # Python 包
│   ├── __init__.py
│   ├── yolo_detector.py            # YOLOv11 + TensorRT 推理封装
│   ├── temporal_filter.py          # EMA 时域滤波
│   ├── feature_extractor.py        # 关键点 → 物理量
│   └── vision_node.py              # ROS2 节点主入口
├── config/
│   ├── params.yaml                 # 节点参数
│   └── camera_intrinsics.yaml      # 相机内参
├── launch/
│   └── vision.launch.py            # 启动文件
├── models/                         # TensorRT 模型（待放）
├── scripts/                        # 辅助脚本（如录数据）
├── resource/                       # ament 资源标记
├── setup.cfg                       # 脚本安装路径
├── setup.py                        # Python 包安装配置
└── package.xml

文件	             作用
yolo_detector.py	加载 TensorRT engine，预处理图像，跑推理，解析 4 个关键点
temporal_filter.py	EMA 滤波：先滤关键点，再滤物理量，防抖
feature_extractor.py	从 4 个关键点算 center_x、h_left、h_right、h_avg
vision_node.py	ROS2 节点：订阅相机，调用上面三个模块，发布 TargetFeatures
params.yaml	模型路径、img_cx（HP60C 是 341.666）、EMA 系数等
camera_intrinsics.yaml	相机内参（从驱动日志提取）
vision.launch.py	启动 vision_node，加载参数
models/	待放 cube_pose.engine（TensorRT 模型）
scripts/	放辅助脚本，比如录数据的 capture.py
resource/vision_servo_vision	空文件，让 ROS2 能找到这个包
setup.cfg	让可执行脚本装到 lib/<pkg>/，否则 ros2 launch 找不到

3. vision_servo_control/ — 控制节点

订阅视觉特征，算误差，发速度指令，判断对准后急停。
text

vision_servo_control/
├── vision_servo_control/           # Python 包
│   ├── __init__.py
│   ├── align_controller.py         # 控制算法（纯逻辑）
│   └── control_node.py             # ROS2 节点主入口
├── config/
│   └── control_params.yaml         # 控制参数
├── launch/
│   └── control.launch.py           # 启动文件
├── resource/
│   └── vision_servo_control        # ament 资源标记
├── setup.cfg
├── setup.py
└── package.xml

文件	作用
align_controller.py	核心算法：算 e_x/e_yaw/e_z，分阶段比例控制，收敛判断。不依赖 ROS，方便单测
control_node.py	ROS2 节点：订阅特征和 start，30Hz 控制循环，发布 cmd_vel 和事件
control_params.yaml	所有增益、阈值、死区、超时
control.launch.py	启动 control_node

关键逻辑：control_loop 里先判断 status，再检查状态，否则 trigger_aligned 永远不会被调用（这是之前修过的 bug）。

4. vision_servo_bringup/ — 一键启动
text

vision_servo_bringup/
├── launch/
│   └── vision_servo.launch.py    # 同时启动 vision + control
├── CMakeLists.txt
└── package.xml

文件	作用
vision_servo.launch.py	同时启动 vision_node 和 align_controller，加载各自参数文件
四、节点与话题
节点
节点	             包	                          作用
vision_node	        vision_servo_vision       	视觉感知，发布特征
align_controller	vision_servo_control	    控制，发布速度
AsCamera	        ascamera	                相机驱动
话题
话题	                          类型	            方向	            说明
/ascamera/AsCamera/rgb0/image  Image	        相机 → vision	      相机图像
/vision_servo/target_features  TargetFeatures	vision → control	 视觉特征
/vision_servo/start	           Empty	        决策层 → control	  触发伺服
/cmd_vel_vision	               Twist	        control → 底盘        速度指令
/vision_servo/event	           String	        control → 决策层	  对准完成/失败
/vision_servo/status	       AlignStatus	    control → 决策层	  状态
/vision_servo/stop	           Bool             control → 底盘	      急停标志


五、快速开始
bash

# 1. 启动相机
source ~/hp60c_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py

# 2. 启动视觉伺服（新终端）
source ~/vision_servo_ws/install/setup.bash
ros2 launch vision_servo_bringup vision_servo.launch.py

# 3. 触发对准（新终端）
ros2 topic pub --once /vision_servo/start std_msgs/msg/Empty

# 4. 看结果
ros2 topic echo /vision_servo/event
ros2 topic echo /cmd_vel_vision

六、编译
bash

cd ~/vision_servo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

注意：

    改 .msg 后必须 rm -rf build install log 再编译

    改 Python 代码不用重编（--symlink-install）

    Python 包必须有 setup.cfg 和 resource/<pkg> 空文件

