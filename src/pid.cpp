/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2023 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#include "pid.h"  // NOLINT

namespace bfs {

float Pid::Run(const float ref, const float feedback) {
  /* Proportional error */
  yp_ = kp_ * (b_ * ref - feedback);
  /* Derivative error */
  yd_ = (kd_ * (c_ * ref - feedback) - dstate_) * n_;
  /* Compute output */
  y_ = yp_ + istate_ + yd_;
  /* Saturation */
  if ((y_ >= max_) && (ki_ * (ref - feedback) > 0)) {
    anti_windup_ = 0;
    y_ = max_;
  } else if ((y_ >= max_) && (ki_ * (ref - feedback) <= 0)) {
    anti_windup_ = 1;
    y_ = max_;
  } else if ((y_ <= min_) && (ki_ * (ref - feedback) >= 0)) {
    anti_windup_ = 1;
    y_ = min_;
  } else if ((y_ <= min_) && (ki_ * (ref - feedback) < 0)) {
    anti_windup_ = 0;
    y_ = min_;
  } else {
    anti_windup_ = 1;
  }
  /* Derivative state */
  dstate_ += dt_ * yd_;
  /* Integrator state */
  istate_ += anti_windup_ * dt_ * ki_ * (ref - feedback);
  /* Return the command */
  return y_;
}

float Pid::Run(const float ref, const float feedback, const float tracking) {
  /* Proportional error */
  yp_ = kp_ * (b_ * ref - feedback);
  /* Derivative error */
  yd_ = (kd_ * (c_ * ref - feedback) - dstate_) * n_;
  /* Compute output */
  y_ = yp_ + istate_ + yd_;
  /* Saturation */
  if ((y_ >= max_) && (ki_ * (ref - feedback) > 0)) {
    anti_windup_ = 0;
    y_ = max_;
  } else if ((y_ >= max_) && (ki_ * (ref - feedback) <= 0)) {
    anti_windup_ = 1;
    y_ = max_;
  } else if ((y_ <= min_) && (ki_ * (ref - feedback) >= 0)) {
    anti_windup_ = 1;
    y_ = min_;
  } else if ((y_ <= min_) && (ki_ * (ref - feedback) < 0)) {
    anti_windup_ = 0;
    y_ = min_;
  } else {
    anti_windup_ = 1;
  }
  /* Derivative state */
  dstate_ += dt_ * yd_;
  /* Integrator state */
  istate_ += anti_windup_ * dt_ * (ki_ * (ref - feedback) + kt_
              * (tracking - y_));
  /* Return the command */
  return y_;
}

void Pid::Reset() {
  dstate_ = d0_;
  istate_ = i0_;
}

}  // namespace bfs
