/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_CONTROL_GAIN_H_
#define INCLUDE_CONTROL_GAIN_H_

namespace controls {

template<typename T>
class Gain {
 public:
  Gain(T k, T min, T max) : k_(k), min_(min), max_(max) {}
  T Run(T input) {
    y_ = k_ * input;
    if (y_ > max_) {
      y_ = max_;
    } else if (y_ < min_) {
      y_ = min_;
    }
    return y_;
  }

 private:
  const T k_, min_, max_;
  T y_;
};

}  // namespace controls

#endif  // INCLUDE_CONTROL_GAIN_H_
