#ifndef VISION_SERVO_GRAB_CPP__GRAB_CONTROLLER_HPP_
#define VISION_SERVO_GRAB_CPP__GRAB_CONTROLLER_HPP_

#include <string>
#include <chrono>
#include <memory>
#include <vector>

namespace vision_servo_grab_cpp
{

// ─────────────────────────────────────────────────
// 状态枚举
// ─────────────────────────────────────────────────
enum class GrabState
{
  IDLE,
  EXTENDING,
  CLOSING,
  RETRACTING,
  DONE,
  FAILED
};

// ─────────────────────────────────────────────────
// 参数
// ─────────────────────────────────────────────────
struct GrabParams
{
  double default_extend_mm = 167.0;
  double default_close_force_pct = 60.0;
  double default_speed_pct = 80.0;
  double extend_timeout_s = 3.0;
  double close_timeout_s = 2.0;
  double retract_timeout_s = 3.0;
};

// ─────────────────────────────────────────────────
// 夹爪反馈
// ─────────────────────────────────────────────────
struct GrabFeedback
{
  double extend_position_mm = 0.0;
  bool has_object = false;
  double close_current_a = 0.0;
};

// ─────────────────────────────────────────────────
// update() 的返回值
// ─────────────────────────────────────────────────
struct GrabResult
{
  std::string action;    // 空字符串表示无动作
  std::string event;     // "GRAB_DONE" / "GRAB_FAILED" / 空
  std::string message;
  double duration_s = 0.0;
};

// ─────────────────────────────────────────────────
// 状态机
// ─────────────────────────────────────────────────
class GrabController
{
public:
  explicit GrabController(const GrabParams & params);

  // 触发夹取序列
  void start();

  // 复位到 IDLE
  void reset();

  // 每帧调用，返回 (action, event)
  GrabResult update(const GrabFeedback * feedback);

  // 查询
  GrabState getState() const { return state_; }
  std::string getLastError() const { return last_error_; }

  // 辅助：状态转字符串
  static std::string stateToString(GrabState s);

private:
  GrabParams params_;
  GrabState state_ = GrabState::IDLE;

  std::chrono::steady_clock::time_point state_start_;
  std::chrono::steady_clock::time_point total_start_;

  std::string last_error_;

  double extend_target_mm_ = 0.0;
  double close_force_pct_ = 0.0;
  double speed_pct_ = 0.0;
};

}  // namespace vision_servo_grab_cpp

#endif  // VISION_SERVO_GRAB_CPP__GRAB_CONTROLLER_HPP_