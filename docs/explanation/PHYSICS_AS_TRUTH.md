# Physics as Truth: The LinkForge Design Philosophy

LinkForge is built on the principle that a robot's digital representation should be as physically consistent as the robot itself. This philosophy, "Physics as Truth," governs how we handle data, validation, and transformations within the Intermediate Representation (IR).

## 1. The Core Tenet

In many robotics tools, a robot model is just a collection of strings and numbers. Validity is only checked when the model is loaded into a physics engine or a simulator. If the model is physically nonsensical (e.g., negative mass, invalid inertia tensor), the simulator crashes or produces erratic behavior.

**LinkForge flips this.** We enforce physical validity at the point of creation. If you cannot build a physically stable robot in LinkForge, you cannot export it.

## 2. Implementation Layers

### Layer 1: Atomic Validation (The Guardrails)
Every core model (`Link`, `Joint`, `Sensor`) implements self-validation logic in its `__post_init__` method. These checks are "atomic"—they only care about the internal consistency of that specific object.

- **Links**: Must have non-negative mass. Inertia tensors must satisfy the triangle inequality ($I_{xx} + I_{yy} \ge I_{zz}$, etc.) and must be positive semi-definite.
- **Joints**: Axes must be normalized unit vectors. Limits must be logically ordered (lower < upper). Type-specific constraints (e.g., fixed joints cannot have limits) are strictly enforced.
- **Sensors**: Range resolutions must be positive. Scanning angles must define a valid volume.

### Layer 2: Orchestrated Validation (The Assembly)
The `RobotValidator` runs checks that require knowledge of the entire robot assembly. This is where topological and semantic consistency are verified.

- **Connectivity**: The kinematic graph must be a tree (or a forest). Cycles are detected and flagged.
- **Naming**: Every component must have a unique, sanitized name compatible with downstream formats (URDF/XACRO).
- **Semantics**: Motion planning groups and collision filters must reference existing links and joints.

### Layer 3: Platform Adapters (The Bridge)
When translating from a DCC tool like Blender, LinkForge does not just copy values. It *interprets* them.

- **Auto-Inertia**: LinkForge can automatically calculate mass properties based on mesh geometry, ensuring that the exported inertia tensor is always physically accurate relative to the visual/collision models.
- **Unit Normalization**: All units are strictly SI (meters, kilograms, radians) in the IR, regardless of the source platform's settings.

## 3. Why It Matters

By treating physics as the "source of truth," LinkForge provides several benefits to robotics developers:

1.  **Reduced Debugging**: Catch "exploding robots" in the linter, not after waiting 5 minutes for a simulation to boot.
2.  **Platform Agnostic**: Because the IR is physically sound, it can be exported to Gazebo, MuJoCo, Isaac Sim, or Webots with high confidence that behavior will be consistent.
3.  **Deterministic Refactoring**: When you rename a link or change a joint type, the orchestrated validation ensures that all dependent components (transmissions, sensors, semantic groups) remain valid or are flagged for repair.

## 4. Contributing with Physics in Mind

When adding new features to LinkForge:
- **Ask**: "What are the physical constraints of this feature?"
- **Implement**: Add those constraints to the model's `__post_init__`.
- **Verify**: Add a test case that specifically tries to break those constraints to ensure the guardrails work.
