# Autonomous Exploration Simulation

A ROS-based autonomous exploration system implementing the frontier-based exploration algorithm from Yamauchi, B., "A frontier-based approach for autonomous exploration", IEEE CIRA, 1997.

## Overview

This package provides a complete simulation environment for testing autonomous exploration algorithms. The robot autonomously navigates and maps unknown environments by identifying and moving towards frontier points (boundaries between known free space and unknown space).

## Features

- **Frontier-based exploration**: Implements the classic Yamauchi algorithm
- **ROS Integration**: Full ROS ecosystem compatibility
- **Simulation Ready**: Works with Gazebo and other ROS simulators
- **Goal Publishing**: Seamless integration with navigation stack

## Topics

- **Goal publishing topic**: `/move_base_simple/goal`

## Demo

See the exploration algorithm in action:

**Demo Video**: https://www.youtube.com/watch?v=MhtYzAHmc9I

## References

Yamauchi, B., "A frontier-based approach for autonomous exploration", IEEE CIRA, 1997.