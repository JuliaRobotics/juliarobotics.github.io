---
layout: default
title: JuliaRobotics
---

## JuliaRobotics Mission Statement

JuliaRobotics is a collection of robotics-related packages that focus on robot control, simulation, navigation, and visualization. JuliaRobotics intends to provide more visibility to robotics-related work within the Julia community. JuliaRobotics does not intend to replace other packages in the Julia ecosystem, but rather to improve the communication around combining the variety of open source packages available for robotics applications. JuliaRobotics intends to leverage the benefits of the Julia language. Lastly, JuliaRobotics has a code of conduct which encourages positive, constructive collaboration, and peer review to improve access and the quality of modern robotics technology.

## Packages

### Visualization

#### Arena.jl

Collection of all 2D and 3D visualizations associated with the [Caesar.jl](https://www.github.com/JuliaRobotics/Caesar.jl.git) and [RoME.jl](https://www.github.com/JuliaRobotics/RoME.jl.git) robotic navigation packages.  This package also relies on [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl.git).

[![Build Status](https://travis-ci.org/JuliaRobotics/Arena.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/Arena.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/Arena.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/Arena.jl?branch=master)
[![docs](https://img.shields.io/badge/docs-latest-blue.svg)](https://dehann.github.io/Caesar.jl/latest/arena_visualizations.html)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/Arena.jl)

#### MeshCatMechanisms.jl

Adds support for visualizing mechanisms and robots from [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/) with [MeshCat.jl](https://github.com/rdeits/MeshCat.jl). All geometries are constructed using [MechanismGeometries.jl](https://github.com/JuliaRobotics/MechanismGeometries.jl).

[![Build Status](https://travis-ci.org/JuliaRobotics/MeshCatMechanisms.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/MeshCatMechanisms.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/MeshCatMechanisms.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/MeshCatMechanisms.jl?branch=master)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/MeshCatMechanisms.jl)

#### MechanismGeometries.jl

Implements several methods of generating or loading geometries associated with a [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/) `Mechanism` in Julia. It is currently used by [MeshCatMechanisms.jl](https://github.com/JuliaRobotics/MeshCatMechanisms.jl) but can also be used independently.

[![Build Status](https://travis-ci.org/JuliaRobotics/MechanismGeometries.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/MechanismGeometries.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/MechanismGeometries.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/MechanismGeometries.jl?branch=master)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/MechanismGeometries.jl)

### Machine Vision

#### AprilTags.jl

This package is a ccall wrapper for the [AprilTags](https://april.eecs.umich.edu/software/apriltag.html) library tailored for Julia.

[![Build Status](https://travis-ci.org/JuliaRobotics/AprilTags.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/AprilTags.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/AprilTags.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/AprilTags.jl?branch=master)
[![docs](https://img.shields.io/badge/docs-latest-blue.svg)](https://juliarobotics.github.io/AprilTags.jl/latest/)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/AprilTags.jl/)

#### SensorFeatureTracking.jl

Algorithms to track features of interest, such as KLT. Please see [documentation](https://JuliaRobotics.github.io/SensorFeatureTracking.jl/latest/), and file issues or make suggestions as you see fit.
**Note** Features in this package are not yet optimized for speed, but the start of implementing machine/computer vision sparse feature functions that are useful to robotics.

[![Build Status](https://travis-ci.org/JuliaRobotics/SensorFeatureTracking.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/SensorFeatureTracking.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/SensorFeatureTracking.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/SensorFeatureTracking.jl?branch=master)

### Dynamics and Simulation

#### RigidBodyDynamics.jl

Julia implementation of various rigid body dynamics and kinematics algorithms.

[![Build Status](https://travis-ci.org/JuliaRobotics/RigidBodyDynamics.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/RigidBodyDynamics.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/RigidBodyDynamics.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/RigidBodyDynamics.jl?branch=master)
[![docs](https://img.shields.io/badge/docs-stable-blue.svg)](https://JuliaRobotics.github.io/RigidBodyDynamics.jl/stable)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/RigidBodyDynamics.jl)

#### RigidBodySim.jl

Simulation and visualization of articulated rigid body systems in Julia.

[![Build Status](https://travis-ci.org/JuliaRobotics/RigidBodySim.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/RigidBodySim.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/RigidBodySim.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/RigidBodySim.jl?branch=master)
[![docs](https://img.shields.io/badge/docs-stable-blue.svg)](https://JuliaRobotics.github.io/RigidBodySim.jl/stable)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/RigidBodySim.jl)

### Calibration

#### MotionCaptureJointCalibration.jl

Kinematic calibration for robots using motion capture data.

[![Build Status](https://travis-ci.org/JuliaRobotics/MotionCaptureJointCalibration.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/MotionCaptureJointCalibration.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/MotionCaptureJointCalibration.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/MotionCaptureJointCalibration.jl?branch=master)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/MotionCaptureJointCalibration.jl)

### SLAM and State-Estimation

#### Caesar.jl

An umbrella package for a Simultaneous Localization and Mapping toolkit that allows for non-Gaussian factor graph based SLAM.  This package relies heavily on RoME.jl, IncrementalInference.jl, KernelDensityEstimate.jl, and others.  Please see Arena.jl for visualization tools.

[![Build Status](https://travis-ci.org/JuliaRobotics/Caesar.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/Caesar.jl)
[![docs](https://img.shields.io/badge/docs-latest-blue.svg)](http://juliarobotics.github.io/Caesar.jl/latest/)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/Caesar.jl)

#### RoME.jl

Robot Motion Estimate package that implements many of the algebraic transforms, utilities, and tools for building local factor graph based SLAM systems. This package, together with IncrementalInference.jl, complete the algorithm known as multimodal iSAM (incremental smoothing and mapping).

[![Build Status](https://travis-ci.org/JuliaRobotics/RoME.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/RoME.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/RoME.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/RoME.jl?branch=master)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/RoME.jl)

### Inference / Optimization

#### IncrementalInference.jl

A nonparametric solution (posterior belief estimate) to the Bayes (Junction) tree refactoring of non-Gaussian factor graph (joint probability) inference problem.

[![Build Status](https://travis-ci.org/JuliaRobotics/IncrementalInference.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/IncrementalInference.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/IncrementalInference.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/IncrementalInference.jl?branch=master)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/IncrementalInference.jl)

#### KernelDensityEstimate.jl

A Julia implementation of Nonparametric Belief Propagation (NBP) where (N x D) dimensional products of infinite functional objects are estimated through an efficient multiscale Gibbs sampling strategy.  See associated KernelDensityEstimatePlotting.jl for visualization tools.

[![Build Status](https://travis-ci.org/JuliaRobotics/KernelDensityEstimate.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/KernelDensityEstimate.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/KernelDensityEstimate.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/KernelDensityEstimate.jl?branch=master)
[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/KernelDensityEstimate.jl)
