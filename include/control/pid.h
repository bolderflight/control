/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_CONTROL_PID_H_
#define INCLUDE_CONTROL_PID_H_

namespace controls {

template<typename T>
class Pid {
 public:
  Pid(T kp, T min, T max) : kp_(kp), min_(min), max_(max) {}
  Pid(T kp, T ki, T dt, T min, T max) : kp_(kp), ki_(ki), dt_(dt), min_(min), max_(max) {}
  Pid(T kp, T ki, T kd, T tf, T dt, T min, T max) : kp_(kp), ki_(ki), kd_(kd), tf_(tf), dt_(dt), min_(min), max_(max) {}
  Pid(T kp, T ki, T kd, T tf, T b, T c, T dt, T min, T max) : kp_(kp), ki_(ki), kd_(kd), tf_(tf), b_(b), b_(c), dt_(dt), min_(min), max_(max) {}
  T Run(T ref, T feedback) {
    /* Proportional error */
    ff_prop_err_ = b_ * ref;
    fb_prop_err_ = feedback;
    /* Derivative error */
    if (kd_ != 0) {
      ff_deriv_err_ = c_ * ref;
      fb_deriv_err_ = feedback;
      /* Compute the change in derivative error */
      ff_delta_err_ = ff_deriv_err_ - ff_prev_deriv_err_;
      fb_delta_err_ = fb_deriv_err_ - fb_prev_deriv_err_;
      /* Update the error state */
      if ((tf_ + dt_) != 0) {
        ff_deriv_err_state_ = tf_ * (ff_prev_deriv_err_state_ + ff_delta_err_) / (tf_ + dt_);
        fb_deriv_err_state_ = tf_ * (fb_prev_deriv_err_state_ + fb_delta_err_) / (tf_ + dt_);
      } else {
        ff_deriv_err_state_ = 0;
        fb_deriv_err_state_ = 0;
      }
      /* Store the current values for use in next iteration */
      ff_prev_deriv_err_ = ff_deriv_err_;
      ff_prev_deriv_err_state_ = ff_deriv_err_state_;
      fb_prev_deriv_err_ = fb_deriv_err_;
      fb_prev_deriv_err_state_ = fb_deriv_err_state_;
    }
    /* Integral error */
    if (ki_ != 0) {
      ff_int_err_state_ += (anti_windup_ * dt_ * ref);
      fb_int_err_state_ += (anti_windup_ * dt_ * feedback);
    }
    /* Compute output */
    ff_ = kp_ * ff_prop_err_ + ki_ * ff_int_err_state_ + kd_ * ff_deriv_err_state_;
    fb_ = kp_ * fb_prop_err_ + ki_ * fb_int_err_state_ + kd_ * fb_deriv_err_state_;
    y_ = ff_ - fb_;
    /* Saturation */
    if ((y_ > max_) && (ref - feedback > 0))      {anti_windup_ = 1; y_ = max_;}
    else if ((y_ > max_) && (ref - feedback < 0)) {anti_windup_ = 0; y_ = max_;}
    else if ((y_ < min_) && (ref - feedback > 0)) {anti_windup_ = 0; y_ = min_;}
    else if ((y_ < min_) && (ref - feedback < 0)) {anti_windup_ = 1; y_ = min_;}
    else {anti_windup_ = 1;}
    /* Return the command */
    return y_;
  }

 private:
  /* Control law constants */
  const T kp_ = 0, ki_ = 0, kd_ = 0, tf_ = 0, b_ = 1, c_ = 1, dt_ = 0, min_, max_;
  /* Feedforward and feedback errors */
  T ff_prop_err_ = 0, ff_deriv_err_ = 0;
  T fb_prop_err_ = 0, fb_deriv_err_ = 0;
  /* Previous error values */
  T ff_prev_deriv_err_ = 0;
  T fb_prev_deriv_err_ = 0;
  /* Error states */
  T ff_deriv_err_state_ = 0, ff_int_err_state_ = 0;
  T fb_deriv_err_state_ = 0, fb_int_err_state_ = 0;
  /* Previous error states */
  T ff_prev_deriv_err_state_ = 0;
  T fb_prev_deriv_err_state_ = 0;
  /* Change in error state */
  T ff_delta_err_ = 0;
  T fb_delta_err_ = 0;
  /* Output */
  T ff_, fb_, y_;
  /* Clamping */
  T anti_windup_ = 1;
};

}  // namespace controls

#endif  // INCLUDE_CONTROL_PID_H_
