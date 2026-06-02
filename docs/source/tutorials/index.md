# Overview

Tutorials are **learning-oriented** guides that take you by the hand through a series of steps to achieve a specific result.

Start here if you are new to LinkForge or want to learn the fundamental workflows.

::::{grid} 3
:gutter: 3

:::{grid-item-card}  Visual Robot Construction (Blender UI)
:link: building_diff_drive
:link-type: doc

Learn how to build a complete mobile robot visually from scratch in Blender, configuring joints, collision, sensors, and ROS 2 control.
:::

:::{grid-item-card} ️ Programmatic Robot Construction (Python API)
:link: building_diff_drive_programmatic
:link-type: doc

Learn how to build, validate, and export the same differential drive mobile robot using the standalone `linkforge-core` Python library.
:::

:::{grid-item-card} 🤖 Advanced Parametric Manipulator (Python API)
:link: building_parametric_arm
:link-type: doc

Learn how to write a parametric Python script to dynamically generate an N-Joint robotic arm, define MoveIt planning groups, named states, and collision filters (SRDF).
:::

::::

```{toctree}
:maxdepth: 1
:hidden:

building_diff_drive
building_diff_drive_programmatic
building_parametric_arm
```
