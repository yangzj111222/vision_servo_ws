// ═══════════════════════════════════════════════════════════════
// 夹取节点
//
// 职责：收到"对准完成"事件后，执行夹取动作序列：
//       1. EXTEND   （夹爪伸长）
//       2. CLOSE    （夹住物块）
//       3. RETRACT  （收回）
//       4. 通知决策层夹取完成
//
// 架构：两层分离
//   - GrabNode        ：ROS2 通信层（订阅、发布、定时循环）
//   - GrabController  ：纯状态机（不依赖 ROS，可单测）
// ═══════════════════════════════════════════════════════════════

// 标准库头文件
#include <chrono>      // 时间：std::chrono::steady_clock, 33ms 等
#include <memory>      // 智能指针：std::make_unique, std::make_shared
#include <sstream>     // 字符串流：用于 split() 分割字符串
#include <vector>      // 动态数组：std::vector
#include <string>      // 字符串：std::string

// ROS2 头文件
#include "rclcpp/rclcpp.hpp"                              // ROS2 C++ 客户端库
#include "std_msgs/msg/string.hpp"                        // 标准 String 消息
#include "std_msgs/msg/bool.hpp"                          // 标准 Bool 消息
#include "vision_servo_interfaces/msg/grab_command.hpp"   // 自定义：夹爪指令
#include "vision_servo_interfaces/msg/grab_status.hpp"    // 自定义：夹爪状态
#include "vision_servo_interfaces/msg/grab_event.hpp"     // 自定义：夹爪事件

// 本项目头文件
#include "vision_servo_grab_cpp/grab_controller.hpp"      // 状态机头文件

// 允许直接写 "33ms"，不用写 "std::chrono::milliseconds(33)"
using namespace std::chrono_literals;

// 整个文件放在这个命名空间，避免和其他包的符号冲突
namespace vision_servo_grab_cpp
{

class GrabNode : public rclcpp::Node
{
public:
  // 构造函数：初始化节点、参数、订阅、发布、定时器
  GrabNode()
  : Node("grab_node") 
  {    
    // ─────────────────────────────────────────────────────────
    // 步骤 1：声明并读取参数
    // ─────────────────────────────────────────────────────────
    // 参数会从 YAML 文件读取，如果 YAML 没有就用默认值
    // ⚠️ YAML 里的 key 必须和这里的参数名一致
    // ⚠️ YAML 的顶层 key 必须和节点名 "grab_node" 一致
    // ─────────────────────────────────────────────────────────
    GrabParams params;                                              // 状态机参数结构体

    params.default_extend_mm = declare_parameter("default_extend_mm", 167.0);
    // ↑ 夹爪伸长度(mm)：从收起状态到最前端的距离
    // ⚠️ 需现场用尺子实测

    params.default_close_force_pct = declare_parameter("default_close_force_pct", 60.0);
    // ↑ 夹力(0~100%)：越大夹得越紧

    params.default_speed_pct = declare_parameter("default_speed_pct", 80.0);
    // ↑ 动作速度(0~100%)
    // ⚠️ 太大动作猛会撞，太小会超时

    params.extend_timeout_s = declare_parameter("extend_timeout_s", 3.0);
    // ↑ 伸长阶段的超时(秒)：超过这个时间还没伸长到位就 FAILED

    params.close_timeout_s = declare_parameter("close_timeout_s", 2.0);
    // ↑ 夹住阶段的超时(秒)

    params.retract_timeout_s = declare_parameter("retract_timeout_s", 3.0);
    // ↑ 收回阶段的超时(秒)

    auto_start_ = declare_parameter("auto_start_on_aligned", true);
    // ↑ 是否收到 VISUAL_ALIGNED 后自动触发夹取
    //   true  = 自动（比赛正常用）
    //   false = 需要外部发 /grab/start 才触发（调试用）

    // ─────────────────────────────────────────────────────────
    // 步骤 2：创建状态机
    // ─────────────────────────────────────────────────────────
    // 把参数传给 GrabController，它是纯逻辑类，不依赖 ROS
    // ─────────────────────────────────────────────────────────
    ctrl_ = std::make_unique<GrabController>(params);

    // ─────────────────────────────────────────────────────────
    // 步骤 3：创建订阅者
    // ─────────────────────────────────────────────────────────
    // std::bind 把成员函数绑成"可直接调用的函数"
    //   &GrabNode::onVisionEvent  = 成员函数指针
    //   this                      = 当前对象
    //   std::placeholders::_1     = 占位符，表示回调的第一个参数
    // ─────────────────────────────────────────────────────────

    // 订阅 1：视觉伺服事件（收到 VISUAL_ALIGNED 时触发夹取）
    sub_vision_event_ = create_subscription<std_msgs::msg::String>(
      "/vision_servo/event",     // 话题名
      10,                        // 队列深度
      std::bind(&GrabNode::onVisionEvent, this, std::placeholders::_1));

    // 订阅 2：手动触发（调试试用）
    sub_start_ = create_subscription<std_msgs::msg::Bool>(
      "/grab/start",
      10,
      std::bind(&GrabNode::onGrabStart, this, std::placeholders::_1));

    // 订阅 3：夹爪硬件反馈（伸到哪了、夹到没有）
    sub_feedback_ = create_subscription<vision_servo_interfaces::msg::GrabStatus>(
      "/grab/status_feedback",
      10,
      std::bind(&GrabNode::onFeedback, this, std::placeholders::_1));

    // ─────────────────────────────────────────────────────────
    // 步骤 4：创建发布者
    // ─────────────────────────────────────────────────────────

    // 发布 1：夹爪指令（给下位机/桥接节点）
    pub_cmd_ = create_publisher<vision_servo_interfaces::msg::GrabCommand>(
      "/grab/cmd", 10);

    // 发布 2：夹取事件（给决策层）
    pub_event_ = create_publisher<vision_servo_interfaces::msg::GrabEvent>(
      "/grab/event", 10);

    // 发布 3：夹取状态（给决策层查询用）
    pub_status_ = create_publisher<vision_servo_interfaces::msg::GrabStatus>(
      "/grab/status", 10);

    // ─────────────────────────────────────────────────────────
    // 步骤 5：启动 30 Hz 定时器
    // ─────────────────────────────────────────────────────────
    // 每 33ms 调用一次 loop()
    // 这是节点的"心跳"，负责驱动状态机
    // ─────────────────────────────────────────────────────────
    timer_ = create_wall_timer(33ms, std::bind(&GrabNode::loop, this));

    // ─────────────────────────────────────────────────────────
    // 步骤 6：打印启动日志
    // ─────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Grab node (C++) started");
  }

private:
  // ═══════════════════════════════════════════════════════════
  // 回调函数
  // ═══════════════════════════════════════════════════════════

  // ─────────────────────────────────────────────────────────
  // 回调 1：收到视觉伺服事件
  // ─────────────────────────────────────────────────────────
  // 触发时机：control_node 发 /vision_servo/event
  // 干什么：如果事件是 VISUAL_ALIGNED，就启动夹取状态机
  // ─────────────────────────────────────────────────────────
  void onVisionEvent(const std_msgs::msg::String::SharedPtr msg)
  {
    // 如果不允许自动启动，直接返回
    if (!auto_start_) return;

    // 检查事件内容
    if (msg->data == "VISUAL_ALIGNED") {
      RCLCPP_INFO(get_logger(), "Received VISUAL_ALIGNED, starting grab sequence");
      ctrl_->start();     // 状态机：IDLE → EXTENDING
    }
  }

  // ─────────────────────────────────────────────────────────
  // 回调 2：手动触发夹取
  // ─────────────────────────────────────────────────────────
  // 触发时机：外部发 /grab/start，或者你手动 pub
  // 干什么：和自动触发一样，调 ctrl_->start()
  // ─────────────────────────────────────────────────────────
  void onGrabStart(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {        // data=true 才触发
      RCLCPP_INFO(get_logger(), "Manual grab start");
      ctrl_->start();
    }
  }

  // ─────────────────────────────────────────────────────────
  // 回调 3：收到夹爪硬件反馈
  // ─────────────────────────────────────────────────────────
  // 触发时机：下位机通过桥接节点发 /grab/status_feedback
  // 干什么：更新本地 feedback_，供状态机判断
  // ─────────────────────────────────────────────────────────
  void onFeedback(const vision_servo_interfaces::msg::GrabStatus::SharedPtr msg)
  {
    feedback_.extend_position_mm = msg->extend_position_mm;   // 当前伸到哪了
    feedback_.has_object = msg->has_object;                    // 是否夹到物块
    feedback_.close_current_a = msg->close_current_a;          // 夹爪电流
    has_feedback_ = true;     // 标记：已经收到过反馈
  }

  // ═══════════════════════════════════════════════════════════
  // 核心循环：每 33ms 执行一次
  // ═══════════════════════════════════════════════════════════
  void loop()
  {
    // ─────────────────────────────────────────────────────────
    // 步骤 1：问状态机要指令和事件
    // ─────────────────────────────────────────────────────────
    // 三元运算符：
    //   has_feedback_ ? &feedback_ : nullptr
    //   收到过反馈 → 传指针
    //   没收到过   → 传空指针（表示"还没有反馈"）
    // ─────────────────────────────────────────────────────────
    auto res = ctrl_->update(has_feedback_ ? &feedback_ : nullptr);

    // ─────────────────────────────────────────────────────────
    // 步骤 2：如果有动作，发夹爪指令
    // ─────────────────────────────────────────────────────────
    // res.action 是字符串，空字符串表示"没有动作"
    // 非空（如 "EXTEND:167:80"）→ 要发给夹爪
    // ─────────────────────────────────────────────────────────
    if (!res.action.empty()) {
      publishCmd(res.action);
    }

    // ─────────────────────────────────────────────────────────
    // 步骤 3：如果有事件，发事件
    // ─────────────────────────────────────────────────────────
    // res.event 可能是 "GRAB_DONE" 或 "GRAB_FAILED"
    // 空字符串表示"没有事件"
    // ─────────────────────────────────────────────────────────
    if (!res.event.empty()) {
      publishEvent(res);
    }

    // ─────────────────────────────────────────────────────────
    // 步骤 4：每帧都发状态（决策层随时能查到进度）
    // ─────────────────────────────────────────────────────────
    publishStatus();
  }

  // ═══════════════════════════════════════════════════════════
  // 发布夹爪指令
  // ═══════════════════════════════════════════════════════════
  // 输入：动作字符串，如 "EXTEND:167:80"
  // 输出：发 GrabCommand 到 /grab/cmd
  // ─────────────────────────────────────────────────────────
  void publishCmd(const std::string & action_str)
  {
    vision_servo_interfaces::msg::GrabCommand cmd;   // 创建空消息

    // ─────────────────────────────────────────────────────────
    // 解析字符串，填到消息字段
    // ─────────────────────────────────────────────────────────
    // rfind("前缀", 0) == 0 等价于 "是否以该前缀开头"
    //   （因为 rfind 从位置 0 反向查找，返回 0 说明开头就是它）
    // ─────────────────────────────────────────────────────────

    if (action_str.rfind("EXTEND:", 0) == 0) {
      // "EXTEND:167:80" → parts = ["EXTEND", "167", "80"]
      auto parts = split(action_str, ':');
      cmd.action = vision_servo_interfaces::msg::GrabCommand::ACTION_EXTEND;
      cmd.extend_length_mm = std::stof(parts[1]);    // "167" → 167.0
      cmd.speed_pct = std::stof(parts[2]);           // "80" → 80.0
    } else if (action_str.rfind("CLOSE:", 0) == 0) {
      // "CLOSE:60" → parts = ["CLOSE", "60"]
      auto parts = split(action_str, ':');
      cmd.action = vision_servo_interfaces::msg::GrabCommand::ACTION_CLOSE;
      cmd.close_force_pct = std::stof(parts[1]);     // "60" → 60.0
    } else if (action_str.rfind("RETRACT:", 0) == 0) {
      // "RETRACT:80" → parts = ["RETRACT", "80"]
      auto parts = split(action_str, ':');
      cmd.action = vision_servo_interfaces::msg::GrabCommand::ACTION_RETRACT;
      cmd.speed_pct = std::stof(parts[1]);           // "80" → 80.0
    }

    // 发布消息
    pub_cmd_->publish(cmd);

    // 打印日志（.c_str() 把 std::string 转成 C 风格字符串，供 %s 使用）
    RCLCPP_INFO(get_logger(), "Sent grab cmd: %s", action_str.c_str());
  }

  // ═══════════════════════════════════════════════════════════
  // 发布夹取事件
  // ═══════════════════════════════════════════════════════════
  // 输入：GrabResult（含 event / message / duration）
  // 输出：发 GrabEvent 到 /grab/event
  // ─────────────────────────────────────────────────────────
  void publishEvent(const GrabResult & res)
  {
    vision_servo_interfaces::msg::GrabEvent ev;
    ev.event = res.event;                            // "GRAB_DONE" 或 "GRAB_FAILED"
    ev.message = res.message;                        // 错误信息或空
    ev.duration_s = static_cast<float>(res.duration_s);
    // ↑ static_cast：C++ 显式类型转换（double → float）

    pub_event_->publish(ev);

    // %.2f：浮点数保留 2 位小数
    RCLCPP_INFO(get_logger(), "Grab event: %s (%.2fs)",
                ev.event.c_str(), res.duration_s);
  }

  // ═══════════════════════════════════════════════════════════
  // 发布夹取状态
  // ═══════════════════════════════════════════════════════════
  // 每帧都发（30 Hz）
  // 状态机内部状态 → ROS2 消息字段
  // ─────────────────────────────────────────────────────────
  void publishStatus()
  {
    // 类型别名，省写长类型名
    using GrabStatusMsg = vision_servo_interfaces::msg::GrabStatus;

    GrabStatusMsg st;

    // ─────────────────────────────────────────────────────────
    // 状态映射：内部 enum → 消息字段
    // C++ 的 enum class 不能直接赋值给消息字段，必须显式转换
    // ─────────────────────────────────────────────────────────
    switch (ctrl_->getState()) {
      case GrabState::IDLE:       st.state = GrabStatusMsg::STATE_IDLE; break;
      case GrabState::EXTENDING:  st.state = GrabStatusMsg::STATE_EXTENDING; break;
      case GrabState::CLOSING:    st.state = GrabStatusMsg::STATE_CLOSING; break;
      case GrabState::RETRACTING: st.state = GrabStatusMsg::STATE_RETRACTING; break;
      case GrabState::DONE:       st.state = GrabStatusMsg::STATE_DONE; break;
      case GrabState::FAILED:     st.state = GrabStatusMsg::STATE_FAILED; break;
    }

    st.message = ctrl_->getLastError();    // 错误信息（无错误时为空）

    pub_status_->publish(st);
  }

  // ═══════════════════════════════════════════════════════════
  // 工具函数：字符串分割
  // ═══════════════════════════════════════════════════════════
  // 例：split("EXTEND:167:80", ':') → ["EXTEND", "167", "80"]
  // ─────────────────────────────────────────────────────────
  static std::vector<std::string> split(const std::string & s, char delim)
  {
    std::vector<std::string> out;
    std::stringstream ss(s);        // 把字符串包装成输入流

    std::string item;
    // getline：从流里读一段，遇到分隔符停
    // 成功返回 true，读到末尾返回 false
    while (std::getline(ss, item, delim)) {
      out.push_back(item);          // 把这段加到结果里
    }
    return out;
  }

  // ═══════════════════════════════════════════════════════════
  // 成员变量
  // ═══════════════════════════════════════════════════════════

  // ─── 状态机 ───
  std::unique_ptr<GrabController> ctrl_;    // 智能指针，自动管理内存

  // ─── 参数 ───
  bool auto_start_{true};                   // 是否自动启动（{true} 是 C++11 初始化语法）

  // ─── 订阅者 ───
  // SharedPtr 是 ROS2 的共享指针，必须保留引用，否则会被销毁
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_vision_event_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
  rclcpp::Subscription<vision_servo_interfaces::msg::GrabStatus>::SharedPtr sub_feedback_;

  // ─── 发布者 ───
  rclcpp::Publisher<vision_servo_interfaces::msg::GrabCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<vision_servo_interfaces::msg::GrabEvent>::SharedPtr pub_event_;
  rclcpp::Publisher<vision_servo_interfaces::msg::GrabStatus>::SharedPtr pub_status_;

  // ─── 定时器 ───
  rclcpp::TimerBase::SharedPtr timer_;

  // ─── 反馈状态 ───
  GrabFeedback feedback_;                   // 最新的夹爪反馈
  bool has_feedback_{false};                // 是否收到过反馈
};

}  // namespace vision_servo_grab_cpp

// ═══════════════════════════════════════════════════════════════
// 程序入口
// ═══════════════════════════════════════════════════════════════
int main(int argc, char ** argv)
{
  // 1. 初始化 ROS2（解析命令行参数）
  rclcpp::init(argc, argv);

  // 2. 创建节点并开始 spin
  //    spin = 阻塞，处理所有回调（订阅、定时器），直到 Ctrl+C 或 shutdown
  rclcpp::spin(std::make_shared<vision_servo_grab_cpp::GrabNode>());

  // 3. 关闭 ROS2
  rclcpp::shutdown();

  return 0;
}