std_msgs/Header header

# 动作类型
uint8 ACTION_IDLE = 0
uint8 ACTION_EXTEND = 1
uint8 ACTION_CLOSE = 2
uint8 ACTION_RETRACT = 3
uint8 ACTION_RELEASE = 4
uint8 ACTION_RESET = 5
uint8 action

# 动作参数
float32 extend_length_mm    # 伸长度(mm), 0=默认
float32 close_force_pct     # 夹力(0~100%), 0=默认
float32 speed_pct           # 速度(0~100%), 0=默认

# 超时
float32 timeout_s