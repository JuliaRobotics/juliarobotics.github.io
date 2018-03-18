## JuliaRobotics Mission Statement

JuliaRobotics is a collection of robotics-related packages that focus on robot control, simulation, navigation, and visualization. JuliaRobotics intends to provide more visibility to robotics-related work within in the Julia community. JuliaRobotics does not intend to replace other packages in the Julia ecosystem, but rather improve the communication around combining the variety of open source packages available to open-source robotics. JuliaRobotics intends to leverage the benefits of the Julia language. Lastly, JuliaRobotics has a code of conduct which encourages positive, constructive collaboration, and peer review to improve access and the quality of modern robotic technology.

## Packages

### Visualization

#### Arena.jl

Collection of all 2D and 3D visualizations associated with the [Caesar.jl](http://www.github.com/dehann/Caesar.jl.git) and [RoME.jl](http://www.github.com/dehann/RoME.jl.git) robotic navigation packages.
[![Build Status](https://travis-ci.org/JuliaRobotics/Arena.jl.svg?branch=master)](https://travis-ci.org/dehann/Arena.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/Arena.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/Arena.jl?branch=master)
[![docs](https://img.shields.io/badge/docs-latest-blue.svg)](http://dehann.github.io/Caesar.jl/latest/arena_visualizations.html)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/Arena.jl)

### Machine Vision

#### AprilTags.jl

This package is a ccall wrapper for the [AprilTags](https://april.eecs.umich.edu/software/apriltag.html) library tailored for Julia.
[![Build Status](https://travis-ci.org/JuliaRobotics/AprilTags.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/AprilTags.jl)
[![codecov.io](http://codecov.io/github/JuliaRobotics/AprilTags.jl/coverage.svg?branch=master)](http://codecov.io/github/JuliaRobotics/AprilTags.jl?branch=master)
[![docs](https://img.shields.io/badge/docs-latest-blue.svg)](https://juliarobotics.github.io/AprilTags.jl/latest/)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://juliarobotics.github.io/AprilTags.jl/)

#### SensorFeatureTracking

Algorithms to track features of interest, such as KLT. Please see [documentation](https://JuliaRobotics.github.io/SensorFeatureTracking.jl/latest/), and file issues or make suggestions as you see fit.
**Note** Features in this package are not yet optimized for speed, but the start of implementing machine/computer vision sparse feature functions that are useful to robotics.
[![Build Status](https://travis-ci.org/JuliaRobotics/SensorFeatureTracking.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/SensorFeatureTracking.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/SensorFeatureTracking.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/SensorFeatureTracking.jl?branch=master)
