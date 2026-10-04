std_msgs/Header header
float32 center_x          # 目标横向中心（像素）
float32 h_left            # 左侧高度（像素）
float32 h_right           # 右侧高度（像素）
float32 h_avg             # 平均高度（像素）
float32 e_yaw             # 偏航误差（归一化）
float32 conf              # 检测置信度
bool valid                # 本帧是否有效