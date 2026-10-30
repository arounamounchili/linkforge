# Defining Semantic Data (SRDF)

SRDF (Semantic Robot Description Format) extends URDF by adding high-level information needed for motion planning in MoveIt 2, such as virtual joints, planning groups, end effectors, named poses, and collision filtering.

While URDF describes **what the robot is**, SRDF describes **how to use it**.

LinkForge lets you configure SRDF either visually in the Blender **Planning** panel or programmatically using the Python `RobotBuilder.semantic` API.

---

## Part 1: Visual Configuration (Blender Planning Panel)

Open the **Planning** panel in the LinkForge N-Panel sidebar (located between **Control** and **Validate & Export**).

### 1. Virtual Joint

A virtual joint attaches your robot's root link to a global reference frame (such as `world` or `odom`).

1. Enable **Virtual Joint** in the **Planning** panel.
2. Click the **Detect Root Link** button to automatically assign the kinematic tree's root link, or pick a link from the scene selector.
3. Configure **Name** (default: `world_joint`), **Parent Frame** (default: `world`), and **Type**:
   - **Fixed**: Manipulators bolted to a table or floor.
   - **Planar**: Differential-drive or omnidirectional mobile bases (`x`, `y`, `theta`).
   - **Floating**: Humanoids, quadrupeds, drones, or free-floating platforms (6-DOF).

### 2. Planning Groups

Planning groups define kinematic chains or joint sets that MoveIt 2 plans for together (e.g., `arm` or `hand`).

1. In the **Planning Groups** section, click **Add (`+`)**.
2. Set the group **Name** and choose a **Mode**:
   - **Chain (Base -> Tip)**: Select a **Base Link** and **Tip Link**. LinkForge automatically walks the kinematic tree and resolves all active joints between them.
   - **Joint Collection**: Click **Add Joint** to pick specific joints from your robot's kinematic tree (ideal for multi-finger grippers or non-serial mechanisms).
3. Check the live **Resolved Active Joints** counter at the bottom of the group box to verify that the kinematic chain is connected.

### 3. End Effectors

An end effector associates a gripper or tool group with the manipulator group and link that carries it.

1. In the **End Effectors** section, click **Add (`+`)**.
2. Configure:
   - **Name**: Identifier for the end effector (e.g., `gripper`).
   - **Group**: The planning group representing the end effector itself (e.g., `hand`).
   - **Parent Link**: The link where the end effector is mounted (e.g., `wrist_link`).
   - **Parent Group**: The planning group that carries the end effector (e.g., `arm`).

### 4. Group States (Named Poses)

Group states store named joint configurations (such as `home`, `ready`, or `stow`) for a planning group.

1. In the **Group States (Poses)** section, click **Add (`+`)**.
2. Set the pose **Name** and select the target **Group**.
3. LinkForge automatically populates a slider for each active joint in the group, displaying its physical limits `[lower, upper]` and clamping values to those bounds.
4. Use **Sync** (`FILE_REFRESH`) if you modify the group's topology, or **Reset (`0.0`)** (`LOOP_BACK`) to reset all joint positions to zero (clamped to valid limits).

### 5. Disabled Collisions

By default, MoveIt 2 checks for collisions between every pair of links. You can disable pairs that are guaranteed never to collide or that touch by design at a joint.

- **Auto-Disable Adjacent Links**: Click **Auto-Disable Adjacent Links** to automatically walk the kinematic tree and add every parent–child link pair with reason `Adjacent`.
- **Custom Pairs**: Click **Add (`+`)** to add a specific link pair (`Link 1` and `Link 2`) and select a standard MoveIt 2 **Reason** (`Adjacent`, `Never`, `Default`, `Always`, or `User`).

### 6. Exporting & Importing Companion SRDF

- **Exporting**: Adding any planning group, end effector, group state, or disabled collision automatically enables **Include SRDF** in the **Validate & Export** panel. Clicking **Export Robot** writes both `<robot>.urdf` (or `<robot>.xacro`) and `<robot>.srdf` to the same directory in one click.
- **Importing**: When importing a `.urdf` or `.xacro` file, LinkForge automatically detects any companion `.srdf` file in the same directory and populates the **Planning** panel.

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
