# Glossary

This glossary defines key terms used throughout the LinkForge documentation, Blender integration, and URDF/XACRO standards.

## Link
A representation of a rigid body in a robot model. Every link must have a unique name. Links contain properties for **Visual**, **Collision**, and **Inertial** data.

## Joint
A connection between two links that defines their relative motion (e.g., revolute, prismatic, fixed, continuous).

## Root Link
The base of the robot's kinematic tree. A robot can only have one root link, and it must not have any parent joint.

## Inertia Tensor
A 3x3 matrix representing the rotational inertia of a rigid body. LinkForge simplifies this by calculating the moments of inertia (`ixx`, `iyy`, `izz`) based on the object's geometry and mass.

## Joint Interface
The configuration of how a joint is controlled in ROS 2. Includes **Command Interfaces** (e.g., Velocity) and **State Interfaces** (e.g., Position, Velocity).

## Sensor
A device that provides feedback from the environment (e.g., Camera, Lidar, IMU). In LinkForge, sensors are specialized Empty objects attached to Links that generate `<sensor>` tags and their associated Gazebo configurations.

## ROS 2 Control
The standard framework for robot control in ROS 2. LinkForge natively supports this by generating the `<ros2_control>` XML block, mapping joint interfaces (Position, Velocity, Effort) to hardware plugins.

## Gazebo
The primary physics simulator for ROS. LinkForge supports both **Gazebo Classic** (v11) and modern **Gazebo Sim** (Harmonic/Ionic) by exporting compatible collision geometry and sensor tags.


## URDF (Unified Robot Description Format)
An XML format used in ROS to describe the physical properties of a robot.

## XACRO (XML Macros)
An XML macro language used to simplify URDF files by allowing variables, math, and macros.

## Actuation Vector
The visual representation in Blender showing the direction of force or movement for a specific joint interface.
