# The `.lf` Standard: Robotics Intermediate Representation (IR)

**Version**: 1.1
**Status**: Specification Draft
**Target Runtimes**: ROS 2, MuJoCo, Gazebo, Isaac Sim

---

## 1. Overview
The `.lf` (LinkForge) format is the "Source Code" for robotics. It is a high-fidelity, metadata-rich Intermediate Representation (IR) designed to bridge the gap between CAD tools and simulation engines without data loss.

### Design Principles
1.  **Physics is Truth**: Every inertial property must be physically plausible (validated via the triangle inequality).
2.  **Lossless Round-Trips**: All data required for simulation must be syncable back to the visual modeling environment.
3.  **Modular Assembly**: Support for referencing external components via `lf://` URIs.

---

## 2. File Structure
The `.lf` standard uses **JSON** or **YAML** as its primary exchange format.

### 2.1 Top-Level Schema
```json
{
  "format_version": "1.1",
  "units": {
    "length": "meters",
    "mass": "kg",
    "angle": "radians",
    "time": "seconds"
  },
  "metadata": {
    "name": "string",
    "author": "string",
    "license": "string",
    "version": "semver"
  },
  "kinematics": "KinematicsObject",
  "perception": "PerceptionObject",
  "control": "ControlObject",
  "sim_specific": "SimulationObject"
}
```

---

## 3. Core Components

### 3.1 Kinematics (Links & Joints)
Links represent rigid bodies, and Joints represent the kinematic constraints between them.

#### Inertial Properties
LinkForge enforces scientific inertia tensors.
```json
"inertial": {
  "mass": 1.25,
  "origin": [0, 0, 0, 1, 0, 0, 0], // [x, y, z, qw, qx, qy, qz]
  "inertia": {
    "ixx": 0.001, "ixy": 0.0, "ixz": 0.0,
    "iyy": 0.001, "iyz": 0.0,
    "izz": 0.001
  }
}
```

### 3.2 Resource Resolution (`lf://`)
Assets (meshes, materials) should be referenced using cloud-resolvable URIs.
*   `lf://local/parts/wheel.glb`: Resolve from the local project workspace.
*   `lf://registry/sensors/lidar_v3.lf`: Resolve from a global or private registry.

### 3.3 Actuator Curves (AI-Ready)
To support high-fidelity Reinforcement Learning, `.lf` supports torque/effort curves rather than just static limits.
```json
"actuator": {
  "type": "dc_motor",
  "torque_curve": [
    {"rpm": 0, "torque": 5.0},
    {"rpm": 1000, "torque": 4.5}
  ]
}
```

---

## 4. Namespacing & Modular Assembly
When merging robots (e.g., attaching an arm to a torso), LinkForge uses **Prefix Namespacing** to avoid collisions.
*   Sub-robot `arm` link `hand` becomes `arm_hand` in the final IR.

---

## 5. Future: Binary IR
For high-performance loading in large-scale simulation environments (e.g., thousands of robots in Isaac Sim), LinkForge will introduce a **Binary IR** based on **Protocol Buffers (protobuf)**. This will serve as the "Object File" (`.lfo`) to the `.lf` "Source Code."

---

> [!TIP]
> For implementation details, see the `linkforge_core.models` Python module in the source code.
