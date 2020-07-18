/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "control/pid.h"
#include "control/gain.h"
#include <iostream>

int main() {
  controls::Pid<float> pid(2.0f, -1.0f, 1.0f);
  std::cout << pid.Run(3, 1) << std::endl;

  controls::Gain<float> g(2, -1, 10);
  std::cout << g.Run(3) << std::endl;
}
