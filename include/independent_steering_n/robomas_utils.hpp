#include <can_plugins2/msg/robomas_frame.hpp>
#include <can_plugins2/msg/robomas_target.hpp>

static std::unique_ptr<can_plugins2::msg::RobomasFrame> get_robomas_frame(const uint8_t motor,const uint8_t mode, const uint8_t temp, const float kp, const float ki, const float kd, const float limitie){
  auto msg = std::make_unique<can_plugins2::msg::RobomasFrame>();
  msg->motor = motor;
  msg->mode = mode;
  msg->temp = temp;
  msg->kp = kp;
  msg->ki = ki;
  msg->kd = kd;
  msg->limitie = limitie;
  return msg;
}

static std::unique_ptr<can_plugins2::msg::RobomasTarget> get_robomas_target(const float target){
  auto msg = std::make_unique<can_plugins2::msg::RobomasTarget>();
  msg->target = target;
  return msg;
}