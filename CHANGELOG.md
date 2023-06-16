# Changelog

## v4.0.0
- Moved away from templates to floats and switched to header and source files

## v3.0.0
- Updated directory structure to support Arduino in addition to CMake builds
- Added an Arduino example
- Updated README

## v2.0.0
- Updated namespace to *bfs*

## v1.1.3
- Updated CONTRIBUTING
- Removed line length exceptions from CI/CD
- Updated gain.h and pid.h to conform to linting

## v1.1.2
- Updated CONTRIBUTING

## v1.1.1
- Updated license to MIT

## v1.1.0
- Added tracking mode for bumpless transition and antiwindup of cascaded controllers. Added kt (tracking gain) setting, derivative initial condition, and integrator initial condition states. Added a Reset method to reset derivative and integrator states to initial conditions.

## v1.0.3
- Modified derivative and integrator state calculations to match Simulink PID block

## v1.0.2
- Made antiwindup clamping depend on the sign of ki, which seems like a good general approach

## v1.0.1
- Antiwindup sign fix

## v1.0.0
- Initial baseline
