# Changelog

All notable changes to LinkForge will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-12-23

> [!NOTE]
> v1.0.0: Initial Production-Ready Release

LinkForge 1.0.0 is the first production-ready release of the professional URDF & XACRO exporter for Blender.

### Features
- **Complete URDF/XACRO Support**: Full bidirectional import and export logic.
- **Sensor Suite**: Comprehensive support for Camera, Depth Camera, LiDAR, IMU, GPS, Contact, and Force/Torque sensors.
- **ROS2 Control Integration**: Native `ros2_control` configuration generation for Gazebo and real hardware interfaces.
- **Automatic Physics**: Automatic calculation of mass and inertia tensors for both primitive shapes and complex meshes.
- **Smart Validation**: Built-in validator to catch errors like disconnected links, missing inertia, or invalid joint limits before export.
- **Round-Trip Fidelity**: Importing a URDF and re-exporting it preserves all properties, including sensor origins and visual/collision geometries.
- **Resilient Parsing**: The parser gracefully handles minor errors (like invalid geometry dimensions) by logging warnings instead of crashing.
- **XACRO Powers**: Automatic macro generation, material extraction, and support for split-file export organization.
- **Viewport Visualization**: Semantic feedback in the viewport for joint axes, limits, and transmission actuation vectors.
- **Modern Standards**: Support for modern Gazebo Sim sensor types (`gpu_lidar`, `navsat`) and ROS 2 conventions.

### Security
- **Path Validation**: Strict validation of mesh paths to prevent path traversal vulnerabilities.
- **XML Hardening**: Protection against XML bomb attacks through depth limits.
- **Input Sanitization**: Numeric constraints to prevent NaN/Inf injection.

---

## Release Notes - v1.0.0

**Installation:**
```bash
# From Blender Extensions
Edit > Preferences > Get Extensions > Search "LinkForge"

# Or manual installation
Download linkforge-1.0.0.zip from releases
Edit > Preferences > Get Extensions > Install from Disk
```

**Documentation:**
- [Architecture Guide](https://linkforge.readthedocs.io/en/latest/explanation/ARCHITECTURE.html)
- [Contributing Guide](CONTRIBUTING.md)

**Contributors:**
- Arouna Patouossa Mounchili ([@arounamounchili](https://github.com/arounamounchili))

[1.0.0]: https://github.com/arounamounchili/linkforge/releases/tag/v1.0.0
