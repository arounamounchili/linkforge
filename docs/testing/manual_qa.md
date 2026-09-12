# LinkForge Manual QA Protocol & Release Runbook

This protocol defines the **7-Phase Verification Plan** required before every major release and for pull requests modifying viewport interaction, UI dashboards, or export pipelines.

Automated unit tests and headless integration tests cannot fully simulate Blender viewport rendering, operator modal interactivity, undo/redo event stacks, or host window manager states. This runbook serves as the final **Layer of Truth** before publishing.

---

## Test Session Metadata
*Complete this section at the start of each QA session.*

| Field | Value |
| :--- | :--- |
| **QA Engineer / Maintainer** | `[Maintainer Name]` |
| **Test Date** | `YYYY-MM-DD` |
| **Blender Version** | `Blender 4.2 LTS / 4.3 / 4.5+` |
| **Operating System** | `macOS (Apple Silicon / Intel) / Ubuntu 22.04+ / Windows 11` |
| **LinkForge Version** | `v1.5.2` |
| **Build Artifact** | `dist/linkforge-blender-1.5.2-[platform].zip` |
| **Git Commit / Branch** | `[commit-hash] / [branch-name]` |

---

## Phase 1: Environment & Clean Installation (Smoke)

### `TC-INST-01`: Clean Extension Installation
* Verify the packaged extension installs cleanly in a fresh Blender profile.
- [ ] **Action**: Open Blender -> Preferences -> Get Extensions (or Install from Disk) -> Select `.zip` build artifact.
- [ ] **Expected**:
  - LinkForge installs without errors or Python tracebacks in the System Console.
  - Bundled binary wheels (e.g. `PyYAML`) load properly for the host Python architecture.
  - LinkForge tab appears in the 3D Viewport sidebar (`N-Panel`).
  - Add-on Preferences display version `1.5.2` and configuration settings.

### `TC-INST-02`: Base Link Creation
* Verify initial link entity bootstrapping from viewport geometry.
- [ ] **Action**: In an empty scene, create a standard Cube (`Shift+A` -> Mesh -> Cube). Click `Create Link from Mesh` in the LinkForge panel.
- [ ] **Expected**:
  - An Empty object (acting as the Link Root) is created at the mesh origin.
  - The Cube mesh is automatically parented under the Link Empty and named `[Name]_visual`.
  - Object is flagged as `is_robot_link = True`.
  - Link Properties panel activates with default mass (`1.0 kg`) and automatic inertia enabled.

---

## Phase 2: Kinematic Hierarchy & Real-Time Sync Engine

### `TC-KIN-01`: Joint Creation & Auto-Pairing
* Verify joint establishment between parent and child links.
- [ ] **Action**: Create a second Link Empty (`arm_link`). Add a Joint object (`Create Joint`). Select the Joint and click the **Auto (A)** picker.
- [ ] **Expected**:
  - Joint automatically assigns one Link as `Parent` and the other as `Child`.
  - Joint axis gizmo (ARROWS) aligns with the joint transform in the 3D Viewport.

### `TC-KIN-02`: Kinematic Types & Dynamic Limits
* Verify configuration of joint movement constraints.
- [ ] **Action**: Set Joint Type to `REVOLUTE`. Configure limits: `Lower = -1.57 rad`, `Upper = 1.57 rad`, `Effort = 50.0 Nm`, `Velocity = 2.0 rad/s`.
- [ ] **Expected**:
  - Joint properties save without truncation.
  - Limit arc/range gizmo updates interactively in the viewport.

### `TC-KIN-03`: Real-Time Name Synchronization
* Verify bi-directional name synchronization across scene collections.
- [ ] **Action**: In the Blender Outliner, rename the child link from `arm_link` to `manipulator_forearm`.
- [ ] **Expected**:
  - The Joint object's `Child Link` property immediately reflects `manipulator_forearm`.
  - All attached sensors and ROS 2 control dashboard references update simultaneously.
  - Stable model identifiers (`source_name_stored`) prevent accidental reference breakage.

### `TC-KIN-04`: Kinematic Selection Operators
* Verify topological traversal and tree selection utilities.
- [ ] **Action**: Select a mid-chain link and execute selection operators:
  1. `Select Subtree`: Entire downstream kinematic branch is selected.
  2. `Select Parent Joint`: The immediate driving joint object is selected.
  3. `Select Root Link`: Viewport selection jumps directly to the root link empty.
- [ ] **Expected**: Selection sets match the kinematic dependency graph without missing children.

### `TC-KIN-05`: Undo / Redo Event Stack Resilience
* Verify the property and handler engine survives aggressive undo cycles.
- [ ] **Action**: Rename a link, alter a limit, and press `Ctrl+Z` (Undo) multiple times, followed by `Ctrl+Shift+Z` (Redo).
- [ ] **Expected**:
  - Scene states revert and restore cleanly.
  - No stale memory pointers, broken reference errors, or orphan handler warnings in the console.

---

## Phase 3: Collision Geometry & Live Decimation Pipeline

### `TC-COLL-01`: Primitive Collision Shapes
* Verify analytic collision bounding creation.
- [ ] **Action**: For a link, set Collision Type to `Box`, `Cylinder`, and `Sphere`. Click `Generate Collision`.
- [ ] **Expected**:
  - Wireframe preview primitives are generated matching visual bounds.
  - Collision objects receive the `linkforge_geom` property group with `role = COLLISION`.

### `TC-COLL-02`: Mesh Simplification (Decimate Slider)
* Verify real-time modifier synthesis and mesh decimation.
- [ ] **Action**: Select a high-polygon mesh, set collision type to `Mesh (Simplified)`, click `Generate Collision`, and adjust the decimation ratio slider.
- [ ] **Expected**:
  - Modifier updates geometry interactively in the 3D viewport.
  - Face and vertex counts reduce proportionally without freezing the Blender UI.
  - Applying or finalizing the collision bake produces an unlinked, optimized convex/simplified mesh.

---

## Phase 4: Inertia Solvers & 3D Viewport Gizmos

### `TC-PHYS-01`: Scientific Inertia Calculation (Mirtich Divergence)
* Verify automatic inertia tensor calculation from geometry.
- [ ] **Action**: With `Auto-Calculate Inertia` enabled, assign realistic mass (`2.5 kg`) to a link with an irregular 3D mesh.
- [ ] **Expected**:
  - LinkForge calculates principal moments (`ixx`, `iyy`, `izz`) and cross terms (`ixy`, `ixz`, `iyz`) via Mirtich Divergence.
  - Tensor satisfies Sylvester's Criterion (positive semi-definite matrix).

### `TC-PHYS-02`: Manual Inertia Overrides & Physicality Checks
* Verify guardrails when overriding mass and inertia values manually.
- [ ] **Action**: Disable `Auto-Calculate Inertia`. Input physically impossible values (e.g., negative mass, or `ixx + iyy < izz`). Run validation.
- [ ] **Expected**:
  - Validator flags non-physical inertia with high-priority warnings/errors.
  - Triangle inequality violations are clearly described in the Component Browser.

### `TC-PHYS-03`: Real-Time Viewport Gizmos
* Verify 3D inspection overlays for inertial centers and axes.
- [ ] **Action**: Toggle `Show Inertia Gizmos`. Adjust Center of Mass sliders (`origin_xyz`).
- [ ] **Expected**:
  - Inertia equivalent ellipsoid and CoM origin gizmo translate smoothly in the 3D Viewport.

---

## Phase 5: Perception Suite & ROS 2 Control Dashboard

### `TC-SENS-01`: Sensor Configuration & Attachment
* Verify attachment and parameterization of perception hardware.
- [ ] **Action**: Select a link, click `Create Sensor`, and configure:
  1. `Camera`: Resolution (1920x1080), FoV, clipping range.
  2. `LiDAR`: Ray samples (360), min/max range (0.1m - 30m), scan rate.
  3. `IMU` / `Force-Torque`: Noise standard deviations.
- [ ] **Expected**:
  - Sensor Empties spawn with directional visual cones/indicators.
  - Sensor properties map to standard ROS/URDF sensor specifications.

### `TC-CTRL-01`: ROS 2 Control Dashboard
* Verify configuration of transmissions, command interfaces, and state interfaces.
- [ ] **Action**: Open the Control Dashboard. Enable `Use ROS 2 Control`. Add configured joints to the controller list.
- [ ] **Expected**:
  - Joints register with selectable interfaces: `Position`, `Velocity`, `Effort`.
  - PID parameters (`p`, `i`, `d`) accept valid floating-point values.

### `TC-CTRL-02`: Dashboard Maintenance Utilities
* Verify cleanup and pruning of deleted hardware.
- [ ] **Action**: Delete an active joint object directly from the 3D Viewport. Notice the `[Missing]` badge in the dashboard. Click `Prune Missing Joints (🗑️)`.
- [ ] **Expected**:
  - Orphaned joint entry is cleanly removed from the control list.
  - Remaining joint parameters and controller assignments remain intact.

---

## Phase 6: Structural Validation & Security Sandbox

### `TC-VAL-01`: Pre-Flight Component Browser Validation
* Verify comprehensive model validation before compiling.
- [ ] **Action**: Intentionally introduce defects: disconnect a link (orphan link), create a closed kinematic loop, and click `Run Validation`.
- [ ] **Expected**:
  - Component Browser displays color-coded error badges:
    - Red `[ERROR]`: Disconnected/orphan links, closed kinematic cycles.
    - Yellow `[WARNING]`: Sub-optimal inertia ratios, missing collision meshes.
  - Clicking an issue selects and centers the affected object in the Viewport.

### `TC-SEC-01`: Path Traversal Jail & Sandbox Security
* Verify protection against malicious URDF package resource paths.
- [ ] **Action**: Attempt to parse or import an external URDF containing directory traversal paths (e.g., `package://../../../../etc/passwd` or `/root/.ssh/id_rsa`).
- [ ] **Expected**:
  - LinkForge blocks resolution with `RobotSecurityError`.
  - File reading is strictly confined within authorized package roots.

### `TC-XACRO-01`: Secure Expression & Math Resolution
* Verify mathematical property evaluation in XACRO files.
- [ ] **Action**: Import a XACRO file with mathematical property expressions (`${pi / 2}`, `${base_mass * 1.5}`).
- [ ] **Expected**:
  - Mathematical expressions evaluate safely to numerical values.
  - Arbitrary Python code injection (e.g. `__import__('os').system(...)`) is rejected.

---

## Phase 7: Full Round-Trip Export/Import & Metadata Integrity

### `TC-EXP-01`: Multi-Format Compilation
* Verify simultaneous generation of standard robotics assets.
- [ ] **Action**: In the Export panel, select target formats (`URDF`, `XACRO`, `SRDF`), choose mesh export format (`STL` / `OBJ` / `GLB`), enable `Split Files`, and export.
- [ ] **Expected**:
  - Clean export directory is produced:
    - `urdf/`: Synthesized robot XML and XACRO macros.
    - `srdf/`: MoveIt 2 semantic descriptions (planning groups, end-effectors).
    - `meshes/`: Exported visual and collision geometries.
  - XML is well-formed and passes standard ROS `check_urdf` validation.

### `TC-GLTF-01`: glTF 2.0 Custom Properties Preservation
* Verify embeddable metadata for downstream web and game engines.
- [ ] **Action**: Export the robot to `.glb` via Blender's standard exporter with `Custom Properties` checked.
- [ ] **Expected**:
  - The exported file embeds `linkforge`, `linkforge_joint`, `linkforge_sensor`, and `linkforge_geom` metadata dictionaries.
  - Compatible with Godot, Three.js, Unity, and Omniverse glTF loaders.

### `TC-ROUNDTRIP-01`: Full Import Round-Trip Fidelity
* Verify the ultimate test: re-importing the compiled asset into a blank session.
- [ ] **Action**: Open a brand new Blender file (`File -> New -> General`). Import the URDF just exported in `TC-EXP-01`.
- [ ] **Expected**:
  - Full kinematic hierarchy is reconstructed identically.
  - Center of Mass and inertia values match original authoring values.
  - Joint limits, types, and orientations match 1:1.
  - Visual and collision meshes mount at exact coordinates without transform drift.

---

## Release Sign-Off Criteria

A release is authorized for deployment to PyPI, GitHub Releases, and the Blender Extensions Platform **only** when all of the following conditions are met:

- [ ] **Zero Python Tracebacks**: System Console contains no uncaught exceptions or tracebacks during the entire QA workflow.
- [ ] **Session Persistence**: Robot model completely survives a Blender file save (`.blend`) and reload.
- [ ] **Full 7-Phase Completion**: Every critical test case above has passed on the designated release build.
- [ ] **Multi-Platform Verification**: Smoke tests verified on macOS, Linux, and Windows.

| Role | Sign-Off Signature | Date | Decision |
| :--- | :--- | :--- | :--- |
| **Lead Developer** | `@arounamounchili` | `YYYY-MM-DD` | `[PASS / FAIL]` |
| **QA Reviewer** | `[Reviewer]` | `YYYY-MM-DD` | `[PASS / FAIL]` |

---

## Known Architectural Behaviors
*The following are documented behaviors by design, not defects:*
* **Selection Flash**: The 3D Viewport may flash a brief selection outline when real-time name synchronization runs across multiple object collections.
* **Gizmo Redraw Throttle**: In ultra-high-poly scenes (>5M polygons), redraw latency may reach ~50ms during active inertia matrix rotation.
* **Search Case Insensitivity**: The Component Browser search filter operates case-insensitively for ergonomic workflow speed.
