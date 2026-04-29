# Defining Semantic Data (SRDF)

SRDF (Semantic Robot Description Format) extends URDF by adding high-level information needed for motion planning, such as planning groups, named poses, and collision filtering.

While URDF describes **what the robot is**, SRDF describes **how to use it**.

---

## 1. Planning Groups

A Planning Group is a collection of links and joints that are planned together (e.g., an "arm" or a "gripper"). In LinkForge, you define these using the ``group()`` method.

```python
from linkforge_core.composer import RobotBuilder

builder = RobotBuilder("my_robot")
# ... (build your robot links and joints)

# Define an arm group using a chain shorthand
builder.group("arm", base_link="base_link", tip_link="flange")

# Define a gripper group using a list of links
builder.group("gripper", links=["left_finger", "right_finger", "palm"])
```

::: {tip}
The ``base_link`` and ``tip_link`` arguments are convenience shorthands for defining a single kinematic chain. You can also provide multiple chains using the ``chains=[(base, tip), ...]`` argument.
:::

## 2. Group States (Named Poses)

Group states allow you to save specific joint configurations with meaningful names (e.g., "home", "stow", "pick").

```python
# Add a 'home' pose for the arm
builder.group_state(
    name="home",
    group="arm",
    joint_values={
        "joint_1": 0.0,
        "joint_2": -1.57,
        "joint_3": 1.57,
    }
)
```

## 3. Disabling Self-Collisions

By default, motion planners check for collisions between all pairs of links. You can optimize performance and prevent false positives (e.g., adjacent links that are allowed to touch) by disabling specific pairs.

```python
# Disable collision between specific adjacent links
builder.disable_collisions("link_1", "link_2", reason="Adjacent")

# Disable all collisions for a set of links (e.g., wheels vs chassis)
builder.disable_all_collisions(["left_wheel", "right_wheel", "chassis"], reason="Never")
```

## 4. Exporting SRDF

When you are ready to use your robot in MoveIt or other planners, export the semantic description along with your URDF.

```python
srdf_xml = builder.export_srdf()

with open("robot.srdf", "w") as f:
    f.write(srdf_xml)
```

:::{tip}
The ``export_srdf()`` method automatically validates your groups and states. For example, it will warn you if a group state references a joint that doesn't belong to the specified group.
:::
