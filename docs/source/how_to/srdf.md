# Defining Semantic Data (SRDF)

SRDF (Semantic Robot Description Format) extends URDF by adding high-level information needed for motion planning in MoveIt 2, such as virtual joints, planning groups, end effectors, named poses, and collision filtering.

While URDF describes **what the robot is**, SRDF describes **how to use it**.

LinkForge lets you configure SRDF either visually in the Blender **Planning** panel or programmatically using the Python `RobotBuilder.semantic` API.

---

## Part 1: Visual Configuration (Blender Planning Panel)

Open the **Planning** panel in the LinkForge N-Panel sidebar (located between **Control** and **Validate & Export**).

:::{note}
**Empty Scene Guard**: If no robot exists in the scene, the panel displays `No robot in scene` and guides you to create links in the **Forge** panel. Once links are present, all planning sections unlock.
:::

### 1. Virtual Joint

A virtual joint attaches your robot's root link to a global reference frame (such as `world` or `odom`).

1. In the **Virtual Joint** header, check the enable box on the right. LinkForge automatically detects the kinematic root link and assigns it to **Child Link** (or select another link object using the eyedropper).
2. Configure **Name** (default: `world_joint`), **Parent Frame** (default: `world`), and **Type**:
   - **Fixed**: Manipulators bolted to a table or floor.
   - **Planar**: Differential-drive or omnidirectional mobile bases (`x`, `y`, `theta`).
   - **Floating**: Humanoids, quadrupeds, drones, or free-floating platforms (6-DOF).

### 2. Planning Groups

Planning groups define kinematic chains, joint sets, or composite subgroups that MoveIt 2 plans for together (e.g., `arm`, `hand`, or `dual_arm`).

1. In the **Planning Groups** section, click **Add (`+`)**.
2. Set the group **Name** and choose a **Mode** using the segmented toggle:
   - **Kinematic Chain**: Select a **Base Link** and **Tip Link**. LinkForge automatically walks the kinematic tree and resolves all active joints between them, displaying a live status row (`✓ Active Joints (n DOF): base_joint → tip_joint` or diagnostics if broken).
   - **Joint Collection**: Click **`+` (Add)** in the `Joints (n)` header to pick specific joints from your robot's kinematic tree (ideal for multi-finger grippers or non-serial mechanisms).
   - **Subgroups**: Click **`+` (Add)** in the `Subgroups (n)` header to composite other existing planning groups into this group (ideal for dual-arm systems or whole-body planners).
3. The UIList row displays a live summary badge: `[Chain: base_link → tip_link]`, `[Collection: n joints]`, or `[Subgroups: n subgroups]`. Use **`▲` / `▼`** to reorder groups or **`-`** to remove a group.

### 3. End Effectors

An end effector associates a gripper or tool group with the manipulator group and link that carries it.

1. In the **End Effectors** section, click **Add (`+`)**.
2. Configure:
   - **Name**: Identifier for the end effector (e.g., `gripper`).
   - **Group**: The planning group representing the end effector itself (searchable dropdown of existing planning groups).
   - **Parent Link**: The physical link where the end effector is mounted (e.g., `wrist_link`).
   - **Parent Group**: The planning group that carries the end effector (e.g., `arm`).
3. The UIList row displays a live status badge (e.g., `[hand @ wrist_link]` or `[Unconfigured]`).

### 4. Group States (Named Poses) with Visual Preview & Viewport Capture

Group states store named joint configurations (such as `home`, `ready`, or `stow`) for a planning group. LinkForge provides live, non-destructive 3D viewport preview and interactive pose capture:

1. In the **Group States** section, click **Add (`+`)**.
2. Set the pose **Name** and select the target **Group** from the searchable dropdown.
3. LinkForge automatically populates a slider for each active joint in the group, displaying its physical limits `[lower, upper]` and clamping values to those bounds.
4. **Live 3D Pose Preview**:
   - Click **Preview Pose** (`HIDE_OFF`) to articulate the robot in the 3D viewport in real time.
   - Adjusting joint sliders immediately articulates the robot's physical links, with full support for revolute, prismatic, continuous, and recursive mimic joint propagation.
   - A live `[Previewing]` badge appears in the poses list. Click **Preview Pose** again or **Rest Pose** (`LOOP_BACK`) to return the robot to its exact rest pose.
5. **Interactive Viewport Capture**:
   - Alternatively, use Blender's native transform tools (Rotate `R` / Grab `G`) to position the robot's joint empties visually in the 3D viewport.
   - Click **Capture Scene Pose** (`CAMERA`): LinkForge extracts the current relative rotations and translations into the group state's joint sliders with one click.
6. **Zero Transform Drift**:
   - All preview articulation uses an in-memory transform cache. Any URDF export or coordinate-frame calculation automatically restores the robot's base rest pose first, ensuring exported origin tags are never corrupted.
7. In the **Active Joints** sub-box:
   - **Sync** (`FILE_REFRESH`): Updates the joint list if you modified the group's joints or kinematic chain.
   - **Reset** (`LOOP_BACK`): Resets all joint sliders to `0.000` (clamped to the nearest valid joint limit).

### 5. Disabled Collisions & Sampled Collision Matrix (ACM)

By default, MoveIt 2 checks for collisions between every pair of links. You can disable pairs that are guaranteed never to collide or that touch by design at a joint to accelerate planning by up to 10×.

- **Generate Matrix (ACM)** (`MOD_PHYSICS`): Runs a Monte Carlo forward-kinematics sampler across the robot's workspace to compute an Allowed Collision Matrix (ACM). LinkForge samples random configurations within joint limits, computes forward-kinematics bounding spheres, and classifies link pairs into MoveIt 2 standard reasons:
  - `Adjacent`: Links connected by a joint.
  - `Default`: Links that collide in the default rest pose.
  - `Never`: Links that can never collide anywhere across the joint limits (disjoint workspaces).
  - `Always`: Links that collide in every sampled valid configuration.
- **Adjacent Only** (`FILE_REFRESH`): Heuristic operator that disables collisions only for directly connected parent–child links.
- **Custom Pairs**: Click **Add (`+`)** to add an individual link pair (`Link 1` and `Link 2`) and assign a reason manually.

### 6. Passive Joints

Unactuated joints (such as rocker-bogie differential pivots, passive casters, suspension linkages, or unpowered mimic fingers) should not be planned for by MoveIt 2 motion planners:

1. Select the joint object in the 3D viewport or Outliner.
2. In the **Joint** panel, check **Passive Joint (MoveIt 2)** (`DRIVER`).
3. LinkForge automatically:
   - Excludes the passive joint from active planning group DOF and pose sliders.
   - Prevents command interfaces from being erroneously assigned to it in `ros2_control`.
   - Exports `<passive_joint name="..."/>` in the companion `.srdf` file.

### 7. Exporting & Importing Companion SRDF

- **Exporting**: Adding any planning group, end effector, group state, or disabled collision automatically enables **Include SRDF** in the **Validate & Export** panel. Clicking **Export Robot** writes both `<robot_name>.urdf` (or `<robot_name>.xacro`) and `<robot_name>.srdf` to the same directory in one click.
- **Automatic Companion Import**: When importing a `.urdf` or `.xacro` file via **Forge > Import Robot Model**, LinkForge automatically searches for a companion `.srdf` file (beside the URDF, or in standard `config/` or `srdf/` directories) and populates the **Planning** panel.
- **Manual SRDF Import**: In the **Planning** panel, click **Import SRDF...** (`IMPORT`) to load any `.srdf` file from disk directly onto your scene's robot. LinkForge matches links and joints to scene objects, preserves unresolved references without data loss, and supports full undo (`Ctrl+Z`).
- **Purge Planning Data**: Click the **Trash** (`TRASH`) icon button next to **Import SRDF...** to clear all planning groups, end effectors, poses, disabled collisions, and virtual joint settings at once.

---

## Part 2: Programmatic Configuration (Python API)

### 1. Planning Groups

In Python, define planning groups using the `group()` method on the `semantic` builder namespace:

```python
from linkforge.core import RobotBuilder

builder = RobotBuilder("my_robot")
# ... (build your robot links and joints)

# Define an arm group using a chain shorthand
builder.semantic.group("arm", base_link="base_link", tip_link="flange")

# Define a gripper group using a list of links or joints
builder.semantic.group("gripper", links=["left_finger", "right_finger", "palm"])
```

::: {tip}
The ``base_link`` and ``tip_link`` arguments are convenience shorthands for defining a single kinematic chain. You can also provide multiple chains using the ``chains=[(base, tip), ...]`` argument.
:::

### 2. Group States (Named Poses)

Group states allow you to save specific joint configurations with meaningful names (e.g., `"home"`, `"stow"`, `"pick"`):

```python
# Add a 'home' pose for the arm
builder.semantic.group_state(
    name="home",
    group="arm",
    values={
        "joint_1": 0.0,
        "joint_2": -1.57,
        "joint_3": 1.57,
    },
)
```

### 3. Disabling Self-Collisions

Optimize planning performance and prevent false positives by disabling specific link pairs:

```python
# Disable collision between specific adjacent links
builder.semantic.disable_collisions("link_1", "link_2", reason="Adjacent")

# Disable all collisions for a set of links (e.g., wheels vs chassis)
builder.robot.disable_all_collisions(["left_wheel", "right_wheel", "chassis"], reason="Never")
```

### 4. Exporting SRDF

When you are ready to use your robot in MoveIt 2, export the semantic description alongside your URDF:

```python
srdf_xml = builder.export_srdf()

with open("robot.srdf", "w") as f:
    f.write(srdf_xml)
```

:::{tip}
The ``export_srdf()`` method automatically validates your groups and states. For example, it will warn you if a group state references a joint that doesn't belong to the specified group.
:::

---

## Semantic Validation Linting

LinkForge includes deep semantic linting in both the Blender **Validate & Export** panel and `validate_robot()`. The validator checks your semantic configuration against the physical kinematic tree and catches:
- Virtual joints referencing missing child links.
- Planning groups containing non-existent links, joints, subgroups, or cyclic subgroup dependencies.
- Kinematic chains (`base_link` -> `tip_link`) that are physically disconnected or broken.
- End effectors referencing unknown groups or parent links outside their parent/end-effector groups.
- Group states referencing unknown joints, joints outside the target group, or joint values exceeding URDF position limits.
- Disabled collision pairs referencing missing links.
