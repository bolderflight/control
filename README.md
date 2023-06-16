[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Control
This is a library of control law algorithms. This library is compatible with Arduino and with CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Installation

## Arduino
Use the Arduino Library Manager to install this library or clone to your Arduino/libraries folder. This library is added as:

```C++
#include "control.h"
```

An example Arduino executable is located at *examples/arduino/control_example/control_example.ino*. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other devices.

## CMake
CMake is used to build this library, which is exported as a library target called *control*. The header is added as:

```C++
#include "control.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake ..
make
```

This will build the library and an example executable called *control_example*. The example executable source files are located at *examples/cmake/control_example.cc*.

# Namespace
This library is within the namespace *bfs*

# Gain
The *Gain* class implements an output-limited gain, where an input is multiplied by a constant and the output is saturated at upper and lower limits.

## Methods

**Gain(const float k, const float min, const float max)** Creates a *Gain* object. The gain, minimum, and maximum limit must be specified in the constructor.

```C++
/* A gain of 2 with limits at -1 and 10 */
bfs::Gain g(2, -1, 10);
```

**float Run(const float input)** Computes the output of the block given the input value. This would be the input multiplied by the constant gain, unless the output would exceed a limit, in which case the output is saturated at the limit value.

```C++
std::cout << g.Run(3) << std::endl;
```

# PID
Implements a PID controller. The output is saturated at upper and lower limits and integrators are clamped to avoid windup.

## Methods

**Pid(const float kp, const float min, const float max)** Creates a PID controller with proportional gain, kp, and min / max output limits.

**Pid(const float kp, const float ki, const float dt, const float min, const float max)** Creates a PID controller with proportional, kp, and integrator, ki, gains. In addition to the gains and limits, the sampling time, dt, must be specified.

**Pid(const float kp, const float ki, const float dt, const float min, const float max, const float kt)** Creates a PID controller with proportional, kp, and integrator, ki, gains. In addition to the gains and limits, the sampling time, dt, must be specified. The tracking gain, kt, can also be specified.

**Pid(const float kp, const float ki, const float kd, const float N, const float dt, const float min, const float max)** Creates a PID controller with proportional, kp, integrator, ki, and derivative, kd, gains. In addition to the gains, limits, and sampling time, the filter coefficient of the first order derivative filter, N, must be specified.

**Pid(const float kp, const float ki, const float kd, const float N, const float b, const float c, const float dt, const float min, const float max)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms.

**Pid(const float kp, const float ki, const float kd, const float N, const float b, const float c, const float dt, const float min, const float max, const float kt)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms. The tracking gain, kt, can also be specified.

**Pid(const float kp, const float ki, const float kd, const float N, const float b, const float c, const float dt, const float min, const float max, const float kt, const float d0)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms. The tracking gain, kt, and initial condition of the derivative state, d0, can also be specified.

**Pid(const float kp, const float ki, const float kd, const float N, const float b, const float c, const float dt, const float min, const float max, const float kt, const float d0, const float i0)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms. The tracking gain, kt, initial condition of the derivative state, d0, and initial condition of the integrator state, i0, can also be specified.

```C++
/* PI controller with a 50 Hz sampling frequency and limits of +/-1 */ 
bfs::Pid pid(2.0f, 1.0f, 0.02f, -1.0f, 1.0f);
```

**T Run(T ref, T feedback)** Computes the controller output given a reference command and a feedback value.

```C++
/* Reference command of 3 and a feedback value of 1 */
std::cout << pid.Run(3, 1) << std::endl;
```

**T Run(T ref, T feedback, T tracking)** Computes the controller output given a reference command, feedback value, and tracking value. The tracking value can be used to create transient free transitions between control laws and provide anti-windup protection for cascaded PID controllers.

```C++
/* Reference command of 3, feedback value of 1, and tracking value of 0.5 */
std::cout << pid.Run(3, 1, 0.5) << std::endl;
```

**void Reset()** Resets the derivative and integrator states to their initial values
