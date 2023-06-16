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

#ifndef CONTROL_SRC_PID_H_  // NOLINT
#define CONTROL_SRC_PID_H_

#if defined(ARDUINO)
#include <Arduino.h>
#endif

namespace bfs {

class Pid {
 public:
  Pid(const float kp, const float min, const float max)
    : kp_(kp), min_(min), max_(max) {}
  Pid(const float kp, const float ki, const float dt, const float min,
      const float max)
    : kp_(kp), ki_(ki), dt_(dt), min_(min), max_(max) {}
  Pid(const float kp, const float ki, const float dt, const float min,
      const float max, const float kt)
    : kp_(kp), ki_(ki), dt_(dt), min_(min), max_(max), kt_(kt) {}
  Pid(const float kp, const float ki, const float kd, const float N,
      const float dt, const float min, const float max)
    : kp_(kp), ki_(ki), kd_(kd), n_(N), dt_(dt), min_(min), max_(max) {}
  Pid(const float kp, const float ki, const float kd, const float N,
      const float b, const float c, const float dt, const float min,
      const float max)
    : kp_(kp), ki_(ki), kd_(kd), n_(N), b_(b), c_(c), dt_(dt),
      min_(min), max_(max) {}
  Pid(const float kp, const float ki, const float kd, const float N,
      const float b, const float c, const float dt, const float min,
      const float max, const float kt)
    : kp_(kp), ki_(ki), kd_(kd), n_(N), b_(b), c_(c), dt_(dt),
      min_(min), max_(max), kt_(kt) {}
  Pid(const float kp, const float ki, const float kd, const float N,
      const float b, const float c, const float dt, const float min,
      const float max, const float kt, const float d0)
    : kp_(kp), ki_(ki), kd_(kd), n_(N), b_(b), c_(c), dt_(dt),
      min_(min), max_(max), kt_(kt), dstate_(d0), d0_(d0) {}
  Pid(const float kp, const float ki, const float kd, const float N,
      const float b, const float c, const float dt, const float min,
      const float max, const float kt, const float d0, const float i0)
    : kp_(kp), ki_(ki), kd_(kd), n_(N), b_(b), c_(c), dt_(dt), min_(min),
      max_(max), kt_(kt), dstate_(d0), d0_(d0), istate_(i0), i0_(i0) {}
  float Run(const float ref, const float feedback);
  float Run(const float ref, const float feedback, const float tracking);
  void Reset();

 private:
  /* Control law constants */
  const float kp_ = 0, ki_ = 0, kd_ = 0, kt_ = 1, n_ = 0, b_ = 1, c_ = 1,
              dt_ = 0, d0_ = 0, i0_ = 0, min_, max_;
  /* PID responses */
  float yp_ = 0, yd_ = 0;
  /* Derivative state */
  float dstate_ = 0;
  /* Integrator state */
  float istate_ = 0;
  /* Output */
  float y_;
  /* Clamping */
  float anti_windup_ = 1;
};

}  // namespace bfs

#endif  // CONTROL_SRC_PID_H_ NOLINT
