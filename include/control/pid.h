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
    yp_ = kp_ * (b_ * ref - feedback);
    /* Derivative error */
    yd_ = (kd_ * (c_ * ref - feedback) - dstate_) * tf_;
    /* Compute output */
    y_ = yp_ + istate_ + yd_;
    /* Saturation */
    if ((y_ >= max_) && (ki_ * (ref - feedback) > 0))       {anti_windup_ = 0; y_ = max_;}
    else if ((y_ >= max_) && (ki_ * (ref - feedback) <= 0)) {anti_windup_ = 1; y_ = max_;}
    else if ((y_ <= min_) && (ki_ * (ref - feedback) >= 0)) {anti_windup_ = 1; y_ = min_;}
    else if ((y_ <= min_) && (ki_ * (ref - feedback) < 0))  {anti_windup_ = 0; y_ = min_;}
    else {anti_windup_ = 1;}
    /* Derivative state */
    dstate_ += dt_ * yd_;
    /* Integrator state */
    istate_ += anti_windup_ * dt_ * ki_ * (ref - feedback);
    /* Return the command */
    return y_;
  }

 private:
  /* Control law constants */
  const T kp_ = 0, ki_ = 0, kd_ = 0, tf_ = 0, b_ = 1, c_ = 1, dt_ = 0, min_, max_;
  /* PID responses */
  T yp_ = 0, yd_ = 0;
  /* Derivative state */
  T dstate_ = 0;
  /* Integrator state */
  T istate_ = 0;
  /* Output */
  T y_;
  /* Clamping */
  T anti_windup_ = 1;
};

}  // namespace controls

#endif  // INCLUDE_CONTROL_PID_H_
