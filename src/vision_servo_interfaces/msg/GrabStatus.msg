std_msgs/Header header

uint8 STATE_IDLE = 0
uint8 STATE_EXTENDING = 1
uint8 STATE_CLOSING = 2
uint8 STATE_RETRACTING = 3
uint8 STATE_DONE = 4
uint8 STATE_FAILED = 5
uint8 state

float32 extend_position_mm  # 当前伸长度(反馈)
float32 close_current_a     # 夹爪电流(判断是否夹到)
bool has_object             # 是否夹到物块
string message