# control
This is a library of controllers.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

## Installation
CMake is used to build this library, which is exported as a library target called *control*. The header is added as:

```
#include "control/control.h"
```
Note that you'll need CMake version 3.13 or above; it is recommended to build and install CMake from source, directions are located in the [CMake GitLab repository](https://github.com/Kitware/CMake).

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake ..
make
```

This will build the library and an example executable called *control_example*. The example executable source files are located at *examples/control_example.cc*. This code is built and tested on AARCH64 and AMD64 systems running Linux and AMD64 systems running the Windows Subsystem for Linux (WSL).

## Namespace
This library is within the namespace *controls*

## Classes

### Gain
The *Gain* class implements an output-limited gain, where an input is multiplied by a constant and the output is saturated at upper and lower limits.

#### Methods

**Gain(T k, T min, T max)** Creates a *Gain* object. The gain, minimum, and maximum limit must be specified in the constructor. This class is templated by type, which also must be specified as a template parameter.

```C++
/* A gain of 2 with limits at -1 and 10 */
controls::Gain<float> g(2, -1, 10);
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

**Pid(T kp, T ki, T kd, T tf, T dt, T min, T max)** Creates a PID controller with proportional, kp, integrator, ki, and derivative, kd, gains. In addition to the gains, limits, and sampling time, the time constant of the first order derivative filter, tf, must be specified.

**Pid(T kp, T ki, T kd, T tf, T b, T c, T dt, T min, T max)** Creates a 2-DOF PID controller with setpoint weighting on the proportional, b, and derivative, c, terms.

```C++
/* PI controller with a 50 Hz sampling frequency and limits of +/-1 */ 
controls::Pid<float> pid(2.0f, 1.0f, 0.02f, -1.0f, 1.0f);
```

**T Run(T ref, T feedback)** Computes the controller output given a reference command and a feedback value.

```C++
/* Reference command of 3 and a feedback value of 1 */
std::cout << pid.Run(3, 1) << std::endl;
```
