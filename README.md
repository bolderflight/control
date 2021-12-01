[![Pipeline](https://gitlab.com/bolderflight/software/control/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/control/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# control
This is a library of control law algorithms. This library is compatible with Arduino ARM and with CMake build systems. It would also be easy to include with other projects, since it is a header only library.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Installation

## Arduino
Use the Arduino Library Manager to install this library or clone to your Arduino/libraries folder. This library is added as:

```C++
#include "control.h"
```

An example Arduino executable is located at *examples/arduino/control_example/control_example.ino*. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other ARM devices. This library is *not* expected to work on AVR devices.

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

## Namespace
This library is within the namespace *bfs*

## Classes

### Gain
The *Gain* class implements an output-limited gain, where an input is multiplied by a constant and the output is saturated at upper and lower limits.

#### Methods

**Gain(T k, T min, T max)** Creates a *Gain* object. The gain, minimum, and maximum limit must be specified in the constructor. This class is templated by type, which also must be specified as a template parameter.

```C++
/* A gain of 2 with limits at -1 and 10 */
bfs::Gain<float> g(2, -1, 10);
```

**T Run(T input)** Computes the output of the block given the input value. This would be the input multiplied by the constant gain, unless the output would exceed a limit, in which case the output is saturated at the limit value.

```C++
std::cout << g.Run(3) << std::endl;
```

### PID
Implements a PID controller. The output is saturated at upper and lower limits and integrators are clamped to avoid windup.

#### Methods

**Pid(T kp, T min, T max)** Creates a PID controller with proportional gain, kp, and min / max output limits. This class is templated by type, which also must be specified as a template parameter.

**Pid(T kp, T ki, T dt, T min, T max)** Creates a PID controller with proportional, kp, and integrator, ki, gains. In addition to the gains and limits, the sampling time, dt, must be specified.

**Pid(T kp, T ki, T dt, T min, T max, T kt)** Creates a PID controller with proportional, kp, and integrator, ki, gains. In addition to the gains and limits, the sampling time, dt, must be specified. The tracking gain, kt, can also be specified.

**Pid(T kp, T ki, T kd, T N, T dt, T min, T max)** Creates a PID controller with proportional, kp, integrator, ki, and derivative, kd, gains. In addition to the gains, limits, and sampling time, the filter coefficient of the first order derivative filter, N, must be specified.

**Pid(T kp, T ki, T kd, T N, T b, T c, T dt, T min, T max)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms.

**Pid(T kp, T ki, T kd, T N, T b, T c, T dt, T min, T max, T kt)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms. The tracking gain, kt, can also be specified.

**Pid(T kp, T ki, T kd, T N, T b, T c, T dt, T min, T max, T kt, T d0)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms. The tracking gain, kt, and initial condition of the derivative state, d0, can also be specified.

**Pid(T kp, T ki, T kd, T N, T b, T c, T dt, T min, T max, T kt, T d0, T i0)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms. The tracking gain, kt, initial condition of the derivative state, d0, and initial condition of the integrator state, i0, can also be specified.

```C++
/* PI controller with a 50 Hz sampling frequency and limits of +/-1 */ 
bfs::Pid<float> pid(2.0f, 1.0f, 0.02f, -1.0f, 1.0f);
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
