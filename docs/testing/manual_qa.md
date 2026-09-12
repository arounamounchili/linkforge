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
  - LinkForge tabs appear in the 3D Viewport sidebar (`N-Panel`).
  - Add-on Preferences display version `1.5.2` and configuration settings (e.g. "Show Inertia Frames", "Show Joint Gizmos").

### `TC-INST-02`: Base Link Creation
* Verify initial link entity bootstrapping from viewport geometry.
- [ ] **Action**: In an empty scene, create a standard Cube (`Shift+A` -> Mesh -> Cube). In the sidebar (`N-Panel -> LinkForge -> Links`), click `Create Link from Mesh`.
- [ ] **Expected**:
  - In the Outliner: An Empty object (acting as the Link Root) is created at the mesh origin with the visual mesh parented underneath it as `[Name]_visual`.
  - In the Links panel: The "Link Creation" placeholder disappears, and the panel transforms into the active link editor with header **`Link: [Name]`** (chain-link icon).
  - Default **Mass** displays as `1.000 kg` with **Auto-Calculate Inertia** checked.

---

## Phase 2: Kinematic Hierarchy & Real-Time Sync Engine

### `TC-KIN-01`: Joint Creation & Auto-Pairing
* Verify joint establishment between parent and child links.
- [ ] **Action**:
  1. In the Links panel, click `Add Empty Link Frame` to add a second link (e.g., `arm_link`).
  2. Select the child link and click `Create Joint` in the **Joints** panel.
  3. With the Joint object selected, locate the **Connection:** row in the Joints panel and click the **`Detect`** button (icon `AUTO`).
- [ ] **Expected**:
  - The `Parent Link` and `Child Link` fields automatically populate with the respective parent and child links based on spatial proximity.
  - RViz-style RGB Joint axis gizmos (ARROWS) align with the joint transform in the 3D Viewport.

### `TC-KIN-02`: Kinematic Types & Dynamic Limits
* Verify configuration of joint movement constraints.
- [ ] **Action**: Set Joint Type to `REVOLUTE`. Ensure `Has Limits` is checked. Configure limits: `Lower = -1.57 rad`, `Upper = 1.57 rad`, `Effort = 50.0 Nm`, `Velocity = 2.0 rad/s`.
- [ ] **Expected**:
  - Joint properties save without truncation.
  - Limit arc/range gizmo updates interactively in the 3D Viewport.

### `TC-KIN-03`: Real-Time Name Synchronization
* Verify bi-directional name synchronization across scene collections.
- [ ] **Action**: In the Blender Outliner, rename the child link from `arm_link` to `manipulator_forearm`.
- [ ] **Expected**:
  - The Joint object's `Child Link` property immediately reflects `manipulator_forearm`.
  - In the Links panel, the header title updates to `Link: manipulator_forearm`.
  - Child visual elements remain bound to the renamed link empty without broken dependencies or orphan warnings.

### `TC-KIN-04`: Component Browser & Viewport Selection
* Verify scene overview navigation and component focusing.
- [ ] **Action**: In the **Validate & Export** panel, expand the **Component Browser**:
  1. Click on any listed Link or Joint item.
  2. In the search box, type a search filter (e.g., `forearm`).
  3. Clear the search text.
- [ ] **Expected**:
  - Clicking an item immediately selects and activates that object in the 3D Viewport.
  - Typing a search query filters the listed counts and items in real-time.
  - Clearing the search restores the full list.

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
- [ ] **Action**:
  1. Notice that prior to generation, the **Collisions** section displays `No collision geometry` with an info icon (`INFO`).
  2. Click the **`Auto-Generate`** button directly beneath the Collisions box.
  3. In the newly generated collision item row, cycle through the geometry type dropdown (`Box`, `Cylinder`, `Sphere`).
- [ ] **Expected**:
  - In the Links panel: The placeholder disappears, and the new item is listed in the **Collisions** box as `[Name]_collision` with a physics icon (`MOD_PHYSICS`) and a geometry type selector.
  - In the 3D Viewport: A wireframe bounding preview (`display_type = WIRE`) appears immediately overlaid on the mesh.
  - In the Outliner: The collision mesh is parented to the link empty as `[Name]_collision`.

### `TC-COLL-02`: Mesh Simplification (Decimate Slider)
* Verify real-time modifier synthesis and mesh decimation.
- [ ] **Action**:
  1. In the collision item row, set the geometry type dropdown to **`Mesh`**.
  2. Locate the **Collision Quality** percentage slider that appears immediately next to the dropdown.
  3. Drag the slider from `100.0%` down to `20.0%`.
- [ ] **Expected**:
  - A `Decimate` modifier is automatically applied to the collision mesh.
  - The modifier updates geometry interactively in the 3D viewport as the slider moves.
  - Face and vertex counts reduce proportionally without freezing the Blender UI.

---

## Phase 4: Inertia Solvers & 3D Viewport Gizmos

### `TC-PHYS-01`: Scientific Inertia Calculation (Mirtich Divergence)
* Verify automatic inertia tensor calculation from geometry.
- [ ] **Action**: With `Auto-Calculate Inertia` checked, assign realistic mass (`2.5 kg`) to a link with an irregular 3D mesh.
- [ ] **Expected**:
  - LinkForge calculates principal moments (`ixx`, `iyy`, `izz`) and cross terms (`ixy`, `ixz`, `iyz`) via Mirtich Divergence.
  - Tensor satisfies Sylvester's Criterion (positive semi-definite matrix).

### `TC-PHYS-02`: Manual Inertia Overrides & Physicality Checks
* Verify guardrails when overriding mass and inertia values manually.
- [ ] **Action**: Uncheck `Auto-Calculate Inertia`. Input physically impossible values (e.g., negative mass, or `ixx + iyy < izz`). In the Validate & Export panel, click `Run Validation`.
- [ ] **Expected**:
  - Validator flags non-physical inertia with high-priority warnings/errors.
  - Triangle inequality violations are clearly described as actionable cards in the validation results.

### `TC-PHYS-03`: Real-Time Viewport Gizmos
* Verify 3D inspection overlays for inertial centers and axes.
- [ ] **Action**: In LinkForge Preferences, verify `Show Inertia Frames` and `Show Joint Gizmos` are enabled. Adjust Center of Mass sliders (`origin_xyz`).
- [ ] **Expected**:
  - Inertia equivalent ellipsoid and Center of Mass origin gizmo translate smoothly in the 3D Viewport.

---

## Phase 5: Perception Suite & ROS 2 Control Dashboard

### `TC-SENS-01`: Sensor Configuration & Attachment
* Verify attachment and parameterization of perception hardware.
- [ ] **Action**:
  1. Select a link, open the `Perceive` panel, click `Create Sensor`, and configure:
     - `Camera`: Resolution (1920x1080), FoV, clipping range.
     - `LiDAR`: Ray samples (360), min/max range (0.1m - 30m), scan rate.
     - `IMU` / `Force-Torque`: Noise standard deviations.
  2. In the Outliner, rename the sensor's parent link.
- [ ] **Expected**:
  - Sensor Empties spawn with directional visual cones/indicators in the 3D Viewport.
  - Sensor properties map to standard ROS/URDF sensor specifications.
  - The sensor's `Parent Link` attachment field in the Perceive panel immediately updates to the new link name.

### `TC-CTRL-01`: ROS 2 Control Dashboard
* Verify configuration of transmissions, command interfaces, and state interfaces.
- [ ] **Action**:
  1. Open the `Control` panel. Check `Use ROS 2 Control`. Add configured joints to the controller list.
  2. Select and enable interfaces (`Position`, `Velocity`, `Effort`) and set PID values.
  3. In the Outliner, rename an active joint.
- [ ] **Expected**:
  - Joints register with selectable interfaces: `Position`, `Velocity`, `Effort`.
  - The UI list displays active interface badges (e.g. `[P/V/E]`).
  - PID parameters (`p`, `i`, `d`) accept valid floating-point values.
  - The control dashboard list item name updates immediately to reflect the renamed joint.

### `TC-CTRL-02`: Dashboard Maintenance Utilities
* Verify cleanup and pruning of deleted hardware.
- [ ] **Action**: Delete an active joint object directly from the 3D Viewport. Notice the `[Missing]` badge with error icon in the dashboard. Click `Prune Missing Joints (🗑️)`.
- [ ] **Expected**:
  - Orphaned joint entry is cleanly removed from the control list.
  - Remaining joint parameters and controller assignments remain intact.

---

## Phase 6: Structural Validation & Security Sandbox

### `TC-VAL-01`: Pre-Flight Component Browser Validation
* Verify comprehensive model validation before compiling.
- [ ] **Action**: Intentionally introduce defects: disconnect a link (orphan link), create a closed kinematic loop, and click `Run Validation`.
- [ ] **Expected**:
  - Actionable issue cards appear under Robot Validation:
    - Red `CANCEL` icon: Disconnected/orphan links, closed kinematic cycles.
    - Orange `ERROR` icon: Sub-optimal inertia ratios, missing collision meshes.
  - Clicking the `Select` button on any card selects and focuses that affected object directly in the 3D Viewport.

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
- [ ] **Action**: In the **Validate & Export** panel, select target format (`URDF` / `XACRO`), choose mesh format (`STL` / `OBJ` / `GLB`), enable `Split Files`, and click `Export Robot Model`.
- [ ] **Expected**:
  - Clean export directory is produced:
    - `urdf/`: Synthesized robot XML and XACRO macros.
    - `srdf/`: MoveIt 2 semantic descriptions (planning groups, end-effectors) if configured.
    - `meshes/`: Exported visual and collision geometries.
  - XML is well-formed and passes standard ROS `check_urdf` validation.

### `TC-GLTF-01`: glTF 2.0 Custom Properties Preservation
* Verify embeddable metadata for downstream web and game engines.
- [ ] **Action**: Export the robot to `.glb` via Blender's standard exporter (`File -> Export -> glTF 2.0`) with `Custom Properties` checked.
- [ ] **Expected**:
  - The exported file embeds `linkforge`, `linkforge_joint`, `linkforge_sensor`, and `linkforge_geom` metadata dictionaries.
  - Compatible with Godot, Three.js, Unity, and Omniverse glTF loaders.

### `TC-ROUNDTRIP-01`: Full Import Round-Trip Fidelity
* Verify the ultimate test: re-importing the compiled asset into a blank session.
- [ ] **Action**: Open a brand new Blender file (`File -> New -> General`). In the Forge panel, click `Import Robot Model`, and select the exported URDF.
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
