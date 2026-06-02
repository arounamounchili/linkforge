# LinkForge Documentation

Welcome to the official LinkForge documentation. LinkForge is **the programmable robot description engine** - a rigorous Intermediate Representation (IR) and Blender-based platform for building, validating, and deploying robot models with scientific precision.

## ️ LinkForge Architecture: IR-Based Compilation

LinkForge decouples robot definition inputs from physical target configurations using a **Frontends → IR → Backends** pipeline:

```{mermaid}
graph LR
    UI["Blender Visual UI"] -->|Compile| IR("Universal Robot IR")
    API["Python Composer API"] -->|Compile| IR
    CAD["URDF / XACRO Parsers"] -->|Ingest| IR

    IR -->|Audit & Verify| Val["Integrity Validator"]

    IR -->|Export| URDF["URDF / XACRO"]
    IR -->|Export| SRDF["SRDF / MoveIt 2"]
    IR -.->|Planned| Future["MJCF / SDF"]

    style Future fill:#f5f5f5,stroke:#aaa,stroke-dasharray:5,color:#888
```

---

##  Key Features

LinkForge streamlines robotics modeling with the following capabilities:

- **IR-Based Compilation**: A decoupled pipeline with multiple frontends (Blender UI, Python API, URDF import), a validated IR, and extensible simulator backends.
- **Dual-Mode Authoring**: Visual 3D editing inside Blender or programmatic Python coding.
- **Production-Ready Export**: Strictly compliant URDF/XACRO files optimized for ROS/Gazebo.
- **ROS2 Control Support**: Automatic hardware interface configuration.
- **Complete Sensor Suite**: Integrated support for LiDAR, IMU, Depth Cameras, and more.
- **Automatic Physics**: Scientific mass properties and inertia tensor calculation.
- **Modular Robot Assembly**: Build and merge robots programmatically with the **Composer API**.
- **SRDF Generation**: Automatic creation of semantic metadata for complex planning systems.

---

##  Installation

LinkForge is distributed as two separate, fully integrated packages depending on your workflow:

###  Blender Extension (Visual UI Editor)
For 3D modelers and roboticists who want to visual-draft digital twins:
* **Prerequisite**: Blender 4.2 or later
1. Open Blender → **Edit > Preferences > Get Extensions**
2. Search for **"LinkForge"**
3. Click **Install**

### ️ Standalone Python Library (`linkforge-core`)
For developer pipelines, automated CI, and procedural robot generation:
* **Prerequisite**: Python >= 3.11
```bash
pip install linkforge-core
```

---

##  Quick Start

Choose your preferred entry point:

###  Visual Workflow (Blender UI)
1. **Create Links**: Select a mesh and click **Create Link** in the LinkForge panel.
2. **Connect Joints**: Select a child link and click **Create Joint** to specify constraints.
3. **Validate & Export**: Run the validator in the UI and click **Export URDF/XACRO**.

### ️ Programmatic Workflow (Python API)
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

:::{grid-item-card}  Tutorials
:link: tutorials/index
:link-type: doc

**Learning-oriented.** Start here if you are new to LinkForge. Step-by-step lessons to build your first robot.
^^^
- [Visual: Building a Diff-Drive Robot](tutorials/building_diff_drive)
- [Programmatic: Building a Diff-Drive Robot](tutorials/building_diff_drive_programmatic)
:::

:::{grid-item-card} ️ How-to Guides
:link: how_to/index
:link-type: doc

**Task-oriented.** Practical guides to help you achieve specific goals or solve problems.
^^^
- [Adding Sensors](how_to/add_sensors)
- [Defining Joints](how_to/index)
:::

:::{grid-item-card}  Explanation
:link: explanation/index
:link-type: doc

**Understanding-oriented.** Deep dives into the architecture, theory, and design of LinkForge.
^^^
- [Architecture Guide](explanation/ARCHITECTURE)
- [Data Model](explanation/data_model)
:::

:::{grid-item-card}  Reference
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
citation
```

---

:::{note}
**Physics Accuracy**: All inertia calculations use solid-body dynamics formulas to ensure simulation fidelity.
:::

##  Community & Support

- **Found a bug?** Open an issue on our [GitHub Issue Tracker](https://github.com/arounamounchili/linkforge/issues).
- **Have a question?** Join the discussion on [GitHub Discussions](https://github.com/arounamounchili/linkforge/discussions).
- **Want to contribute?** We love PRs! Read our [Contributing Guide](CONTRIBUTING).

## Quick Links

- [GitHub Repository](https://github.com/arounamounchili/linkforge)
- [Issue Tracker](https://github.com/arounamounchili/linkforge/issues)
- [Discussions](https://github.com/arounamounchili/linkforge/discussions)
