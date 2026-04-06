<div align="center">
  <img src="docs/assets/linkforge_logo.png" width="300px" alt="LinkForge Logo"/>
</div>

# LinkForge
**The Linter & Bridge for Robotics**

[![Latest Release](https://img.shields.io/github/v/release/arounamounchili/linkforge)](https://github.com/arounamounchili/linkforge/releases/latest)
[![CI](https://github.com/arounamounchili/linkforge/actions/workflows/ci.yml/badge.svg)](https://github.com/arounamounchili/linkforge/actions)
[![Documentation](https://readthedocs.org/projects/linkforge/badge/?version=latest)](https://linkforge.readthedocs.io/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Blender](https://img.shields.io/badge/Blender-4.2%2B-orange.svg)](https://www.blender.org/download/)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-yellow.svg)](https://conventionalcommits.org)

LinkForge is **The Linter & Bridge for Robotics**, integrating directly into Blender.

It allows you to model your robot as naturally as sculpting a 3D scene, while acting as a strict safety net to guarantee the output is rigorous, simulation-ready code.

 1.  **Forge Structure**: Define links, joints, masses, and inertias visually.
 2.  **Lint & Validate**: Catch structural and physics errors before simulation export.
 3.  **Perceive & Control**: Attach sensors and configure `ros2_control` interfaces.
 4.  **Export Production Code**: Generate hardened URDF/XACRO files.

## 💎 Why LinkForge?

| Feature | Legacy Exporters | LinkForge |
| :--- | :--- | :--- |
| **Architecture** | Monolithic / Tied to one CAD tool | **Hexagonal / Multi-Host & Multi-Target** |
| **Validation** | Post-Export (Fail in Sim) | **Automated Linting (Fail in Editor)** |
| **Physics** | "Close Enough" Mesh Export | **Scientific Inertia & Mass Sanity** |
| **Control** | Manual `ros2_control` XML | **Centralized Dashboard with auto-generation** |
| **Fidelity** | One-way export | **Round-Trip Precision (Import → Edit → Export)** |
| **Sim-to-Real** | Post-Simulation Testing | **Early-Phase Validation & Noise Injection** |

> [!TIP]
> For a deep dive into our long-term technical strategy and the "Digital Twin" philosophy, see **[VISION.md](VISION.md)**.

### 🛠️ Technical Specifications

| Feature | Support | Details |
| :--- | :--- | :--- |
| **Links** | ✅ Full | Visual/Collision Geometry, Materials, Automatic Physics |
| **Joints** | ✅ Full | All 6 types (Fixed, Revolute, etc.) + **Mimic Joints** |
| **Sensors** | ✅ Full | Camera, LiDAR, IMU, GPS, **Contact**, **Force/Torque** |
| **Control** | ✅ Full | `ros2_control` Dashboard & Gazebo Plugin Integration |
| **Validation** | ✅ Pro | **Linter for Robotics** catches structural errors before export |
| **Fidelity** | ✅ Pro | **Round-Trip Precision** for lossless Import/Export |
| **Formats** | ✅ Full | URDF 1.0, XACRO (Macros, Properties, Multi-file), **SRDF (Core API)** |

## 🚀 Key Features

- **Bidirectional Workflow**: Import existing URDF or XACRO files for editing or build complex robot models from scratch using Blender native tools.
- **Production-Ready Export**: Generates strictly compliant URDF and XACRO files optimized for ROS, ROS 2, and Gazebo. Includes **ROS-Agnostic Asset Resolution** for cross-platform editing and **Core API** support for **SRDF** (Semantic Robot Description) generation.
- **Linter for Robotics**: Built-in integrity checker inspects robot topology, physics data, and joint limits. It catches simulation-breaking errors (negative inertias, detached links, circular chains) *before* you export.
- **ROS2 Control Support**: Automatically generates hardware interface configurations for `ros2_control` via a centralized dashboard, compatible with Gazebo and physical hardware.
- **Complete Sensor Suite**: Integrated support for Camera, Depth Camera, LiDAR, IMU, GPS, **Force/Torque**, and **Contact** sensors with configurable noise models.
- **Automatic Physics**: Scientifically accurate calculation of mass properties and inertia tensors for both primitive shapes and complex arbitrary meshes.
- **Advanced XACRO Support**: Intelligent extraction of repeated geometry into macros and shared materials, producing maintainable and modular code.
- **Round-Trip Fidelity**: The Import → Edit → Export cycle preserves all data with **absolute precision**, including sensor origins, transmission interfaces, and custom user properties.
- **Modular Robot Assembly**: High-level **Composer API** (Core library) for assembling robots from modular sub-components, enabling rapid prototyping and **SRDF** generation for complex multi-part systems.

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
   - Select a mesh → **Forge** panel → **Create Link**
   - Configure mass, inertia, and collision geometry in the **Physics** section.
   - Repeat for all robot parts.

2. **Connect with Joints**
   - Select child link → **Forge** panel → **Create Joint**
   - Choose joint type (Revolute, Prismatic, Continuous, Fixed, etc.)
   - Set limits, axis, and dynamics in the **Joint** section.

3. **Add Sensors** (Optional)
   - Select a link → **Perceive** panel → **Add Sensor**
   - Configure sensor properties in the **Sensor** section.

4. **Configure Control** (Optional)
   - Go to the **Control** panel → Enable **Use ROS2 Control**
   - Click `+` to add joints to the Joint Interfaces list.
   - Configure Command/State interfaces (Position, Velocity, Effort).

5. **Validate & Export**
   - Go to the **Validate & Export** panel.
   - Click **Validate Robot** to check for integrity errors.
   - Choose format (URDF/XACRO) and click **Export**.

### Importing Existing URDF

1. Open the **LinkForge** sidebar tab (N-panel in 3D Viewport).
2. In the **Forge** panel, click **Import URDF/XACRO**.
3. Select your file and edit the robot structure normally.
4. Export back via the **Validate & Export** panel.

## 🤖 Examples

Complete examples in `examples/` directory:

- `roundtrip_test_robot.urdf`: A comprehensive robot containing ALL 6 URDF joint types (fixed, revolute, continuous, prismatic, planar, floating), plus sensors. Perfect for testing full roundtrip capabilities.
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
# 1. Install 'just' (Command Runner)
brew install just

# 2. Clone repository
git clone https://github.com/arounamounchili/linkforge.git
cd linkforge

# 3. Install dependencies
just install
```

### Testing
```bash
# Run all tests (Core + Blender)
just test

# Run only core tests
just test-core

# Run with coverage
just coverage
```

### Code Quality
```bash
# Run all checks (Lint + Types)
just check

# Fix linting issues
just fix
```

### Building & Distribution
To package LinkForge as a Blender extension:
```bash
# Build the production-ready .zip
just build
```
The package will be created in the `dist/` directory.

## 🎓 Learning Resources

- [Example Files](https://github.com/arounamounchili/linkforge/tree/main/examples) - Sample URDF files
- [Community Forum](https://github.com/arounamounchili/linkforge/discussions) - Ask questions

## 🗺️ Roadmap

- [x] **v1.0.0**: Core URDF/XACRO Export, Sensors, & `ros2_control` basics.
- [x] **v1.1.0**: Enhanced Documentation, Workflow Polish, & Bug Fixes.
- [x] **v1.2.0**: **Architectural Stability** (Hexagonal Core, Numerical Precision).
- [x] **v1.3.0**: **Performance & Control** (NumPy Acceleration, Depsgraph, & ROS2 Control).
- [/] **v1.4.0**: **Modular Assembly** (SRDF, Composer API, `linkforge_ros`).
- [ ] **v1.5.0**: **High-Fidelity Expansion pt.1** (MJCF/MuJoCo support).
- [ ] **v1.6.0**: **High-Fidelity Expansion pt.2** (SDF/Gazebo support).
- [ ] **v1.7.0**: **Mechanical Debugging** (Real-time IK & Collision Interference Validation).
- [ ] **v2.0.0**: **Intelligence-Driven Rigging** (AI-assisted geometry analysis & Auto-Rigging).

## 🔭 Vision & Future
For a deep dive into our long-term strategy, the **Digital Twin** philosophy, and our technical roadmap for AI and Kinematics, please read our [Project Vision](VISION.md).

## 🤝 Contributing

We welcome contributions! LinkForge is a community-driven project.
- 🙋 Review our [Contributing Guide](CONTRIBUTING.md).
- 🏗️ Check our [Architecture](ARCHITECTURE.md) to understand the internals.
- 💬 Join the conversation on [GitHub Discussions](https://github.com/arounamounchili/linkforge/discussions).

## 📄 License

This project is licensed under the **GNU General Public License v3.0** - see the [LICENSE](LICENSE) file for details.
For third-party component licenses, see [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).

## ✨ Our Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/arounamounchili"><img src="https://avatars.githubusercontent.com/u/55673269?v=4?s=64" width="64px;" alt="arounamounchili"/><br /><sub><b>arounamounchili</b></sub></a><br /><a href="https://github.com/arounamounchili/linkforge/commits?author=arounamounchili" title="Code">💻</a> <a href="#design-arounamounchili" title="Design">🎨</a> <a href="#ideas-arounamounchili" title="Ideas, Planning, & Feedback">🤔</a> <a href="#maintenance-arounamounchili" title="Maintenance">🚧</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/MagnusHanses"><img src="https://avatars.githubusercontent.com/u/115026407?v=4?s=64" width="64px;" alt="MagnusHanses"/><br /><sub><b>MagnusHanses</b></sub></a><br /><a href="https://github.com/arounamounchili/linkforge/issues?q=author%3AMagnusHanses" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/GeorgeKugler"><img src="https://avatars.githubusercontent.com/u/35666712?v=4?s=64" width="64px;" alt="GeKo-8"/><br /><sub><b>GeKo-8</b></sub></a><br /><a href="https://github.com/arounamounchili/linkforge/issues?q=author%3AGeorgeKugler" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/andreas-loeffler"><img src="https://avatars.githubusercontent.com/u/73336148?v=4?s=64" width="64px;" alt="Andreas Loeffler"/><br /><sub><b>Andreas Loeffler</b></sub></a><br /><a href="https://github.com/arounamounchili/linkforge/commits?author=andreas-loeffler" title="Code">💻</a> <a href="https://github.com/arounamounchili/linkforge/commits?author=andreas-loeffler" title="Tests">⚠️</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://www.mec.ed.tum.de/en/iwb/staff/research-team-assembly-technology-and-robotics/mueller-julian/"><img src="https://avatars.githubusercontent.com/u/32650678?v=4?s=64" width="64px;" alt="Julian Müller"/><br /><sub><b>Julian Müller</b></sub></a><br /><a href="#ideas-julianmueller" title="Ideas, Planning, & Feedback">🤔</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://cmp.felk.cvut.cz/~peckama2/"><img src="https://avatars.githubusercontent.com/u/182533?v=4?s=64" width="64px;" alt="Martin Pecka"/><br /><sub><b>Martin Pecka</b></sub></a><br /><a href="https://github.com/arounamounchili/linkforge/issues?q=author%3Apeci1" title="Bug reports">🐛</a> <a href="#ideas-peci1" title="Ideas, Planning, & Feedback">🤔</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/lionelfung7"><img src="https://avatars.githubusercontent.com/u/50703745?v=4?s=64" width="64px;" alt="lionelfung7"/><br /><sub><b>lionelfung7</b></sub></a><br /><a href="https://github.com/arounamounchili/linkforge/issues?q=author%3Alionelfung7" title="Bug reports">🐛</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!

<p align="center">
  <b>Made with ❤️ for roboticists worldwide</b><br/>
  <i>Precision engineering meets creative modeling.</i>
</p>
