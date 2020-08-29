/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "control/control.h"
#include <iostream>

int main() {
  /* Proportional controller with a gain of 2 and limits of +/- 1 */
  controls::Pid<float> pid(2.0f, -1.0f, 1.0f);
  std::cout << pid.Run(3, 1) << std::endl; // 1, saturated

  /* Gain of 2 with limits at -1 and 10 */
  controls::Gain<float> g(2, -1, 10);
  std::cout << g.Run(3) << std::endl; // 6
  std::cout << g.Run(6) << std::endl; // 10, saturated
  std::cout << g.Run(-1) << std::endl; // -1, saturated
}
