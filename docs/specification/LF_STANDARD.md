# The `.lf` Standard: Robotics Intermediate Representation (IR)

**Version**: 1.1
**Status**: Specification Draft
**Target Runtimes**: ROS 2, MuJoCo, Gazebo, Isaac Sim

## 1. Overview
The `.lf` (LinkForge) format is an open, typed **Intermediate Representation (IR)** for robotics. Rather than attempting to replace established runtime formats (such as URDF, MJCF, or USD), `.lf` acts as the canonical upstream source of truth that bridges the gap between CAD tools and multi-simulator deployment without data loss.

### Design Principles
1.  **Physics is Truth**: Every inertial property must be physically plausible (positive semi-definite via Sylvester's criterion and satisfying principal moments triangle inequalities).
2.  **Unifying Compiler, Not a Replacement**: `.lf` compiles deterministically into the exact target formats required by runtime environments (URDF/SRDF for ROS 2/MoveIt, MJCF for MuJoCo, USD for Isaac Sim). Downstream stacks remain completely unchanged.
3.  **Lossless Source of Truth**: Preserves design intent, explicit units, actuator dynamics, and coordinate conventions across editing cycles.
4.  **Modular Assembly**: Support for referencing external components via `lf://` URIs with clean prefix-based namespacing.

### 1.1 Compilation Targets

| Target Runtime | Target Format | Status | Role |
| :--- | :--- | :--- | :--- |
| **ROS 2 / Gazebo** | URDF / XACRO | **Production (v1.x)** | Kinematics, visual/collision geometry, ros2_control tags |
| **MoveIt 2** | SRDF | **Production (v1.x)** | Planning groups, named poses, collision disabling matrix |
| **MuJoCo** | MJCF (XML) | *Planned (v2.0)* | Contact dynamics, tendons, actuator torque limits |
| **Isaac Sim** | OpenUSD (USDA/USDC) | *Planned (v2.0)* | RTX rendering, PhysX articulation schemas |

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

## 4. Namespacing & Modular Assembly
When merging robots (e.g., attaching an arm to a torso), LinkForge uses **Prefix Namespacing** to avoid collisions.
*   Sub-robot `arm` link `hand` becomes `arm_hand` in the final IR.

## 5. Future: Binary IR
For high-performance loading in large-scale simulation environments (e.g., thousands of robots in Isaac Sim), LinkForge will introduce a **Binary IR** based on **Protocol Buffers (protobuf)**. This will serve as the "Object File" (`.lfo`) to the `.lf` "Source Code."

> [!TIP]
> For implementation details, see the `linkforge.core.models` Python module in the source code.
