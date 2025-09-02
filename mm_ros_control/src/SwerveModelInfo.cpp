#include "SwerveModelInfo.hpp"

namespace mm_ros_control {

vector_t getPinocchioJointPosition (const vector_t& state)
{
  vector_t pinocchio_q(25);
  pinocchio_q << state(x_state_ind), state(y_state_ind), state(z_state_ind), state(x_quat_state_ind), state(y_quat_state_ind),
          state(z_quat_state_ind), state(w_quat_state_ind), state(lb_steer_state_ind),
          sin(state(lb_wheel_state_ind)), -cos(state(lb_wheel_state_ind)),  state(lf_steer_state_ind),
          sin(state(lf_wheel_state_ind)), -cos(state(lf_wheel_state_ind)),  state(rb_steer_state_ind),
          sin(state(rb_wheel_state_ind)), -cos(state(rb_wheel_state_ind)),  state(rf_steer_state_ind),
          sin(state(rf_wheel_state_ind)), -cos(state(rf_wheel_state_ind)), state.tail(6);
  return pinocchio_q;
}

}  // namespace mm_ros_control