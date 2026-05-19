# LinkForge Documentation

Welcome to the official LinkForge documentation. LinkForge is the **LLVM for Robotics**, providing a universal, mathematical Intermediate Representation (IR) and a Blender-based digital twin platform for robot design, simulation, and deployment.

## The LLVM for Robotics Architecture

LinkForge decouples robot definition inputs from physical target configurations by acting as a compiler with standard **Frontends**, a **Universal Intermediate Representation (IR)**, and extensible **Backends**:

```{mermaid}
graph TD
    %% Frontends
    subgraph Frontends [Frontends / Input Compilation]
        A1[🎨 Blender Visual UI]
        A2[⚙️ Programmatic Python API]
        A3[📂 CAD Importers / Parsers]
    end

    %% Core IR
    subgraph Core [LinkForge Middle-End]
        B1[Universal Robot IR]
        B2["Linter & Physicality Optimizer"]
        B1 <-->|Kinematic Audits| B2
    end

    %% Backends
    subgraph Backends [Backends / Target Generation]
        C1[URDF Target]
        C2[XACRO Target]
        C3[SRDF Target]
        C4[Future: MJCF / SDF]
    end

    %% Connections
    Frontends -->|Compile / Ingest| B1
    B1 -->|CodeGen / Export| Backends
```

---

## 🚀 Key Features

LinkForge streamlines robotics modeling with the following capabilities:

- **LLVM for Robotics**: A decoupled compiler architecture featuring frontends, a universal IR, and customizable simulator backends.
- **The .lf Standard (Upcoming)**: A platform-agnostic, metadata-rich file format currently under active design (Phase 2 Roadmap) for universal robot preservation.
- **Dual-Mode Authoring**: Visual 3D editing inside Blender or programmatic Python coding.
- **Production-Ready Export**: Strictly compliant URDF/XACRO files optimized for ROS/Gazebo.
- **ROS2 Control Support**: Automatic hardware interface configuration.
- **Complete Sensor Suite**: Integrated support for LiDAR, IMU, Depth Cameras, and more.
- **Automatic Physics**: Scientific mass properties and inertia tensor calculation.
- **Modular Robot Assembly**: Build and merge robots programmatically with the **Composer API** (v1.4.0).
- **SRDF Generation**: Automatic creation of semantic metadata for complex planning systems (v1.4.0).

---

## 📦 Installation

LinkForge is distributed as two separate, fully integrated packages depending on your workflow:

### 🎨 Blender Extension (Visual UI Editor)
For 3D modelers and roboticists who want to visual-draft digital twins:
* **Prerequisite**: Blender 4.2 or later
1. Open Blender → **Edit > Preferences > Get Extensions**
2. Search for **"LinkForge"**
3. Click **Install**

### ⚙️ Standalone Python Library (`linkforge-core`)
For developer pipelines, automated CI, and procedural robot generation:
* **Prerequisite**: Python >= 3.11
```bash
pip install linkforge-core
```

---

## 🎯 Quick Start

Choose your preferred entry point:

### 🎨 Visual Workflow (Blender UI)
1. **Create Links**: Select a mesh and click **Create Link** in the LinkForge panel.
2. **Connect Joints**: Select a child link and click **Create Joint** to specify constraints.
3. **Validate & Export**: Run the validator in the UI and click **Export URDF/XACRO**.

### ⚙️ Programmatic Workflow (Python API)
Create, validate, and export a complete kinematic robot description programmatically:

```python
from linkforge.core import RobotBuilder, box, cylinder

# Initialize robot builder
builder = RobotBuilder("my_robot")

# Micro-construct links and joints programmatically
builder.link("base_link").visual(box(0.5, 0.5, 0.1)).mass(5.0).root()
builder.link("arm", parent="base_link") \
    .visual(cylinder(0.05, 0.5)) \
    .mass(2.0) \
    .revolute(axis=(0, 0, 1), limits=(-1.57, 1.57)) \
    .commit()

# Export strictly-compliant URDF
urdf_xml = builder.export_urdf()
```

---

::::{grid} 2
:gutter: 3

:::{grid-item-card} 🚀 Tutorials
:link: tutorials/index
:link-type: doc

**Learning-oriented.** Start here if you are new to LinkForge. Step-by-step lessons to build your first robot.
^^^
- [Visual: Building a Diff-Drive Robot](tutorials/building_diff_drive)
- [Programmatic: Building a Diff-Drive Robot](tutorials/building_diff_drive_programmatic)
:::

:::{grid-item-card} 🛠️ How-to Guides
:link: how_to/index
:link-type: doc

**Task-oriented.** Practical guides to help you achieve specific goals or solve problems.
^^^
- [Adding Sensors](how_to/add_sensors)
- [Defining Joints](how_to/index)
:::

:::{grid-item-card} 💡 Explanation
:link: explanation/index
:link-type: doc

**Understanding-oriented.** Deep dives into the architecture, theory, and design of LinkForge.
^^^
- [Architecture Guide](explanation/ARCHITECTURE)
- [Data Model](explanation/data_model)
:::

:::{grid-item-card} 📚 Reference
:link: reference/index
:link-type: doc

**Information-oriented.** Technical descriptions, API documentation, and specifications.
^^^
- [Python API Reference](reference/api/index)
- [URDF Specification](reference/index)
:::

::::

---

```{toctree}
:maxdepth: 2
:hidden:
:caption: Tutorials

tutorials/index
```

```{toctree}
:maxdepth: 2
:hidden:
:caption: How-to Guides

how_to/index
```

```{toctree}
:maxdepth: 2
:hidden:
:caption: Explanation

explanation/index
```

```{toctree}
:maxdepth: 2
:hidden:
:caption: API Reference

reference/index
```

```{toctree}
:maxdepth: 2
:hidden:
:caption: About LinkForge

CHANGELOG
CONTRIBUTING
LICENSE
citation
```

---

:::{note}
**Physics Accuracy**: All inertia calculations use solid-body dynamics formulas to ensure simulation fidelity.
:::

## 👥 Community & Support

- **Found a bug?** Open an issue on our [GitHub Issue Tracker](https://github.com/arounamounchili/linkforge/issues).
- **Have a question?** Join the discussion on [GitHub Discussions](https://github.com/arounamounchili/linkforge/discussions).
- **Want to contribute?** We love PRs! Read our [Contributing Guide](CONTRIBUTING).

## Quick Links

- [GitHub Repository](https://github.com/arounamounchili/linkforge)
- [Issue Tracker](https://github.com/arounamounchili/linkforge/issues)
- [Discussions](https://github.com/arounamounchili/linkforge/discussions)
