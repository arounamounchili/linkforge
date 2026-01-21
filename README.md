<div align="center">
  <img src="docs/assets/linkforge_logo.png" width="128" alt="LinkForge Logo"/>
</div>

# LinkForge
**Build Simulation-Ready Robots in Blender**

[![Latest Release](https://img.shields.io/github/v/release/arounamounchili/linkforge)](https://github.com/arounamounchili/linkforge/releases/latest)
[![CI](https://github.com/arounamounchili/linkforge/actions/workflows/ci.yml/badge.svg)](https://github.com/arounamounchili/linkforge/actions)
[![Documentation](https://readthedocs.org/projects/linkforge/badge/?version=latest)](https://linkforge.readthedocs.io/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Blender](https://img.shields.io/badge/Blender-4.2%2B-orange.svg)](https://www.blender.org/download/)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-yellow.svg)](https://conventionalcommits.org)

LinkForge is a professional Blender extension for roboticists.

**In robotics, creativity starts in your head — but getting that idea into a simulator usually means hours of writing XML, fixing joint limits, and fighting coordinate systems.**

 LinkForge removes that friction. It acts as a **bridge between creativity and engineering**, allowing you to model your robot in Blender as naturally as sculpting a 3D scene, while guaranteeing the output is rigorous, simulation-ready code.

 1.  **Forge Structure**: Define links, joints, masses, and inertias visually.
 2.  **Perceive Environment**: Attach sensors like LiDAR, IMU, and Depth Cameras.
 3.  **Control Movement**: Configure transmissions and `ros2_control` interfaces.
 4.  **Export Production Code**: Generate clean, validated URDF/XACRO files.

 **From idea → robot → ready for simulation.** All inside Blender.

### 🛠️ Technical Specifications

| Feature | Support | Details |
| :--- | :--- | :--- |
| **Links** | ✅ Full | Visual/Collision Geometry, Materials, Inertial Properties |
| **Joint Types** | ✅ Full | Fixed, Revolute, Continuous, Prismatic, Planar, Floating |
| **Physics** | ✅ Full | Automatic Mass & Inertia (Primitive & Mesh) |
| **Sensors** | ✅ Full | Camera, LiDAR, IMU, GPS, Depth, Contact |
| **Formats** | ✅ Full | URDF 1.0, XACRO (Macros & Properties) |
| **Dynamics** | ✅ Full | Damping, Friction, Safety Limits |
| **Control** | ✅ Full | `ros2_control` & Gazebo Classic/Sim |

## 🚀 Key Features

- **Bidirectional Workflow**: Seamlessly import existing URDF/XACRO files for editing or build complex robot models from scratch using Blender's native tools.
- **Production-Ready Export**: Generates strictly compliant URDF/XACRO files optimized for ROS, ROS 2, and Gazebo. Output is clean, validated, and requires no manual post-processing.
- **Smart Validation (The Safety Net)**: Built-in integrity checker inspects robot topology, physics data, and joint limits. It catches "exploding robot" errors (negative inertias, detached links) *before* you export.
- **ROS2 Control Support**: Automatically generates hardware interface configurations for `ros2_control`, compatible with Gazebo, Webots, Isaac Sim, and physical hardware.
- **Complete Sensor Suite**: Integrated support for Camera, Depth Camera, LiDAR, IMU, GPS, and Force/Torque sensors with configurable noise models.
- **Automatic Physics**: Scientifically accurate calculation of mass properties and inertia tensors for both primitive shapes and complex arbitrary meshes.
- **Advanced XACRO Support**: Intelligent extraction of repeated geometry into macros and shared materials, producing maintainable and modular code.
- **Round-Trip Fidelity**: Import → Edit → Export cycle preserves all data absolute precision, including sensor origins, transmission interfaces, and custom user properties.

## 📦 Installation

**Requirements**: Blender 4.2 or later

### Method 1: Blender Extensions (Recommended)
1. Open Blender → **Edit > Preferences > Get Extensions**
2. Search for **"LinkForge"**
3. Click **Install**

### Method 2: Manual Installation
1. Download the `.zip` package for your platform (e.g., `linkforge-x.x.x-windows-x64.zip`) from [Latest Releases](https://github.com/arounamounchili/linkforge/releases/latest)
2. Open Blender → **Edit > Preferences > Get Extensions**
3. Click dropdown (⌄) → **Install from Disk**
4. Select the downloaded `.zip` file

## 🎯 Quick Start

### Creating a Robot from Scratch

1. **Create Links**
   - Select a mesh → LinkForge panel → **Create Link**
   - Configure mass, inertia, and collision geometry
   - Repeat for all robot parts

2. **Connect with Joints**
   - Select child link → **Create Joint**
   - Choose joint type (Revolute, Prismatic, Continuous, Fixed)
   - Set limits, axis, and dynamics

3. **Add Sensors** (Optional)
   - Select a link → **Add Sensor**
   - Configure sensor type, update rate, and noise parameters

4. **Configure Control** (Optional)
   - Select a joint → **Create Transmission**
   - Configure hardware interfaces (Position, Velocity, or Effort)

5. **Validate & Export**
   - Click **Validate Robot** to check for errors
   - Choose format (URDF/XACRO)
   - Click **Export** → Done!

### Importing Existing URDF

1. **File > Import > URDF (.urdf)**
2. Select your URDF file
3. Edit in Blender
4. Export back to URDF/XACRO



## 📚 Examples

Complete examples in `examples/` directory:

- `roundtrip_test_robot.urdf`: A comprehensive robot containing ALL 6 URDF joint types (fixed, revolute, continuous, prismatic, planar, floating), plus sensors and transmissions. Perfect for testing full roundtrip capabilities.
- `mobile_robot.urdf`: A simple mobile robot base.
- `diff_drive_robot.urdf`: A differential drive robot with wheels.
- `quadruped_robot.urdf`: A 4-legged robot demonstrating complex kinematic chains and multi-link assemblies.


## 📚 Documentation

- **[User Guide](https://linkforge.readthedocs.io/en/latest/tutorials/index.html)** - Comprehensive tutorials and getting started.
- **[API Reference](https://linkforge.readthedocs.io/en/latest/reference/api/index.html)** - Technical reference for developers.
- **[Architecture Guide](https://linkforge.readthedocs.io/en/latest/explanation/ARCHITECTURE.html)** - System design and internals.
- **[CHANGELOG](CHANGELOG.md)** - Version history.
- **Examples**: [examples/](https://github.com/arounamounchili/linkforge/tree/main/examples)

## 💻 Development

### Setup
```bash
# Clone repository
git clone https://github.com/arounamounchili/linkforge.git
cd linkforge

# Install dependencies
uv sync
```

### Testing
```bash
# Run all tests
uv run pytest

# Run with coverage
uv run pytest --cov=linkforge --cov-report=html
```

### Code Quality
```bash
# Format code
uv run ruff format .

# Lint code
uv run ruff check .

# Type check
uv run mypy linkforge

# Install all hooks (code quality and conventional commit messages)
uv run pre-commit install --hook-type pre-commit --hook-type commit-msg
```

### Building & Distribution
To package LinkForge as a Blender extension:
```bash
# General build (automatic wheel bundling)
python3 build_extension.py

# Sync dependencies (update wheels for all platforms)
python3 build_extension.py sync
```
The package will be created in the `dist/` directory.

### Managing Dependencies
LinkForge uses a "Self-Contained" bundling strategy for wheels. To update or add new dependencies:
1. Open `build_extension.py` and update the `DEP_CONFIG` dictionary.
2. Run `python3 build_extension.py sync` to download wheels for Windows, Linux, and Mac (Python 3.10 & 3.11).



## 🎓 Learning Resources

- [Example Files](https://github.com/arounamounchili/linkforge/tree/main/examples) - Sample URDF files
- [Community Forum](https://github.com/arounamounchili/linkforge/discussions) - Ask questions

## 🗺️ Roadmap

- [x] **v1.0.0**: Core URDF/XACRO Export, Sensors, & `ros2_control` basics.
- [x] **v1.1.0**: Enhanced Documentation, Workflow Polish, & Bug Fixes.
- [ ] **v1.2.0**: **Architectural Stability & Precision** (Hexagonal Core, Inertial Accuracy).
- [ ] **v1.3.0**: **MJCF (MuJoCo) Export Support** (High demand for RL).
- [ ] **v1.4.0**: **Kinematic Validation** (MoveIt style checks).


## 🔮 Our Transparency Commitment

We believe in being upfront about what LinkForge does—and what it doesn't do.

1.  **Standard Compliance**: We guarantee 100% compliance with the official URDF/XACRO specifications. Your robot will work in any standard parser.
2.  **The "Wild URDF" Limit**: We aim for lossless Round-Trip (Import → Edit → Export). However, if you import a "Wild" URDF containing non-standard tags, custom XML comments, or parser-specific hacks, **we do not guarantee their preservation**. We clean the code to ensure validity.
3.  **Beta Features**: Advanced features like `ros2_control` are evolving. We commit to vigilance in updating them, but syntax changes in ROS 2 may require extension updates.

## 🤝 Contributing

We welcome contributions! LinkForge is an community-driven project.
- 🙋 Review our [Contributing Guide](CONTRIBUTING.md).
- 🏗️ Check our [Architecture](ARCHITECTURE.md) to understand the internals.
- 💬 Join the conversation on [GitHub Discussions](https://github.com/arounamounchili/linkforge/discussions).

## 📄 License

This project is licensed under the **GNU General Public License v3.0** - see the [LICENSE](LICENSE) file for details.

---

<p align="center">
  <b>Made with ❤️ for roboticists worldwide</b><br/>
  <i>Precision engineering meets creative modeling.</i>
</p>
