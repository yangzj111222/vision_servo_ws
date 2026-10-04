#include "vision_servo_grab_cpp/grab_controller.hpp"

#include <sstream>
#include <iomanip>

namespace vision_servo_grab_cpp
{

  GrabController::GrabController(const GrabParams &params)
      : params_(params)
  {
  }

  std::string GrabController::stateToString(GrabState s)
  {
    switch (s)
    {
    case GrabState::IDLE:
      return "IDLE";
    case GrabState::EXTENDING:
      return "EXTENDING";
    case GrabState::CLOSING:
      return "CLOSING";
    case GrabState::RETRACTING:
      return "RETRACTING";
    case GrabState::DONE:
      return "DONE";
    case GrabState::FAILED:
      return "FAILED";
    default:
      return "UNKNOWN";
    }
  }

  void GrabController::start()
  {
    state_ = GrabState::EXTENDING;
    state_start_ = std::chrono::steady_clock::now();
    total_start_ = state_start_;
    last_error_.clear();

    extend_target_mm_ = params_.default_extend_mm;
    close_force_pct_ = params_.default_close_force_pct;
    speed_pct_ = params_.default_speed_pct;
  }

  void GrabController::reset()
  {
    state_ = GrabState::IDLE;
    last_error_.clear();
  }

  GrabResult GrabController::update(const GrabFeedback *feedback)
  {
    GrabResult result;

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - state_start_).count();
    double total_elapsed = std::chrono::duration<double>(now - total_start_).count();

    // 辅助 lambda：构造动作
    auto make_action = [&](const std::string &act)
    {
      result.action = act;
      return result;
    };

    // 辅助 lambda：构造失败
    auto make_failed = [&](const std::string &err)
    {
      state_ = GrabState::FAILED;
      last_error_ = err;
      result.event = "GRAB_FAILED";
      result.message = err;
      result.duration_s = total_elapsed;
      return result;
    };

    switch (state_)
    {

    // ─────────────────────────────────────────────
    // 阶段 1：伸长
    // ─────────────────────────────────────────────
    case GrabState::EXTENDING:
    {
      if (elapsed > params_.extend_timeout_s)
      {
        return make_failed("extend timeout");
      }
      // 检查是否到达目标位置
      if (feedback && feedback->extend_position_mm >= extend_target_mm_ * 0.95)
      {
        state_ = GrabState::CLOSING;
        state_start_ = now;
        std::ostringstream oss;
        oss << "CLOSE:" << close_force_pct_;
        return make_action(oss.str());
      }
      // 刚进入伸长阶段，发指令
      if (elapsed < 0.01)
      {
        std::ostringstream oss;
        oss << "EXTEND:" << extend_target_mm_ << ":" << speed_pct_;
        return make_action(oss.str());
      }
      return result;
    }

    // ─────────────────────────────────────────────
    // 阶段 2：夹住
    // ─────────────────────────────────────────────
    case GrabState::CLOSING:
    {
      if (elapsed > params_.close_timeout_s)
      {
        return make_failed("close timeout");
      }
      if (feedback && feedback->has_object)
      {
        state_ = GrabState::RETRACTING;
        state_start_ = now;
        std::ostringstream oss;
        oss << "RETRACT:" << speed_pct_;
        return make_action(oss.str());
      }
      if (elapsed < 0.01)
      {
        std::ostringstream oss;
        oss << "CLOSE:" << close_force_pct_;
        return make_action(oss.str());
      }
      return result;
    }

    // ─────────────────────────────────────────────
    // 阶段 3：收回
    // ─────────────────────────────────────────────
    case GrabState::RETRACTING:
    {
      if (elapsed > params_.retract_timeout_s)
      {
        return make_failed("retract timeout");
      }
      if (feedback && feedback->extend_position_mm <= 5.0)
      {
        state_ = GrabState::DONE;
        result.event = "GRAB_DONE";
        result.message = "grab sequence completed";
        result.duration_s = total_elapsed;
        return result;
      }
      if (elapsed < 0.01)
      {
        std::ostringstream oss;
        oss << "RETRACT:" << speed_pct_;
        return make_action(oss.str());
      }
      return result;
    }

    default:
      return result;
    }
  }

} // namespace vision_servo_grab_cpp