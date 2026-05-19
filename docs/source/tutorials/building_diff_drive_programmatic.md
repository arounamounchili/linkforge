# Tutorial: Building a Differential Drive Robot (Programmatic)

In this tutorial, you will configure a complete differential drive mobile robot programmatically in Python using the standalone `linkforge-core` library. You will learn how to define links, joints, a LiDAR sensor, and `ros2_control` configurations, then export everything to standardized URDF and SRDF files.

## What You Will Learn
- How to initialize `RobotBuilder`.
- How to construct **Links** and **Joints** programmatically.
- How to configure joint limits and axes.
- How to attach a **LiDAR Sensor**.
- How to define **Control Interfaces** using `ros2_control`.
- How to **Validate** and **Export** URDF and SRDF data.

---

## 🌳 Kinematic Tree

The structure of the robot we will construct is identical to the one modeled visually in Blender:

```mermaid
graph TD
    base_link[base_link] -->|continuous| left_wheel[left_wheel]
    base_link -->|continuous| right_wheel[right_wheel]
    base_link -->|fixed| lidar_link[lidar_link]
```

---

## Step 1: Install `linkforge-core`

Before writing code, ensure you have Python (version 3.11 or later) installed. Run:

```bash
pip install linkforge-core
```

---

## Step 2: Initialize `RobotBuilder`

The primary entry point for constructing robots programmatically is the `RobotBuilder`. Initialize it with a robot name:

```python
from linkforge.core import RobotBuilder

# Initialize a new robot builder named 'diff_drive'
builder = RobotBuilder("diff_drive")
```

---

## Step 3: Create the Base Link

Next, define the base link (chassis) of our mobile robot. We will shape it using a solid box primitive and set its mass. Since it is the root of the robot's kinematic chain, we will chain `.root()`:

```python
from linkforge.core import box

# Create a box primitive for the chassis (dimensions: 0.4m x 0.3m x 0.1m)
chassis_geom = box(0.4, 0.3, 0.1)

# Stage the base_link
builder.link("base_link") \
    .visual(chassis_geom) \
    .collision() \
    .mass(5.0) \
    .root()
```

::: {tip}
Calling `.collision()` without arguments automatically infers and creates an optimized collision mesh matching your visual geometry. Chaining `.mass(5.0)` automatically computes the physical inertia tensor for the link based on its geometry and mass.
:::

---

## Step 4: Create the Wheels & Configure Control

Now, add the left and right wheels. These will be continuous (unlimited rotation) joints. We will also configure `ros2_control` command and state interfaces on each wheel joint so they can be driven by a velocity controller:

```python
from linkforge.core import cylinder

# Create a cylinder primitive for the wheels (radius: 0.1m, length: 0.05m)
wheel_geom = cylinder(radius=0.1, length=0.05)

# 1. Left Wheel
builder.link("left_wheel", parent="base_link") \
    .visual(wheel_geom, rpy=(1.57, 0, 0)) \
    .collision() \
    .mass(1.0) \
    .continuous(
        axis=(0, 1, 0),
        xyz=(0.1, 0.15, 0.0),
        rpy=(0, 0, 0),
    ) \
    .ros2_control(
        command_interfaces=["velocity"],
        state_interfaces=["position", "velocity"],
    ) \
    .commit()

# 2. Right Wheel
builder.link("right_wheel", parent="base_link") \
    .visual(wheel_geom, rpy=(1.57, 0, 0)) \
    .collision() \
    .mass(1.0) \
    .continuous(
        axis=(0, 1, 0),
        xyz=(0.1, -0.15, 0.0),
        rpy=(0, 0, 0),
    ) \
    .ros2_control(
        command_interfaces=["velocity"],
        state_interfaces=["position", "velocity"],
    ) \
    .commit()
```

---

## Step 5: Create a Lidar Sensor

Finally, add a LiDAR link on top of the base. We will connect it with a fixed joint, and attach a pre-configured LiDAR sensor using the `.lidar()` helper method:

```python
# Create a small cylinder for the LiDAR visual representation
lidar_geom = cylinder(radius=0.05, length=0.06)

# Stage and build the lidar link
builder.link("lidar_link", parent="base_link") \
    .visual(lidar_geom) \
    .collision() \
    .mass(0.2) \
    .fixed(xyz=(0.15, 0.0, 0.08)) \
    .lidar(
        name="chassis_laser",
        range_min=0.1,
        range_max=10.0,
        samples=360,
    ) \
    .commit()
```

---

## Step 6: Validate & Export

We are ready to validate our kinematic model and export both the physical robot description (URDF) and semantic description (SRDF):

```python
# Validate the assembled robot using the built-in validation engine
from linkforge.core import validate_robot

result = validate_robot(builder.robot)

if result.is_valid:
    print("✓ Robot is physically and structurally valid!")
else:
    print("✗ Validation failed:")
    for error in result.errors:
        print(f"  - {error.message}")
    exit(1)

# Export strictly-compliant URDF XML
urdf_xml = builder.export_urdf()
with open("diff_drive.urdf", "w") as f:
    f.write(urdf_xml)
print("✓ Successfully exported diff_drive.urdf!")

# Export SRDF XML for MoveIt motion planning
srdf_xml = builder.export_srdf()
with open("diff_drive.srdf", "w") as f:
    f.write(srdf_xml)
print("✓ Successfully exported diff_drive.srdf!")
```

---

## 完整 Python 脚本

Here is the complete, self-contained Python script to build, validate, and export the robot:

```python
from linkforge.core import RobotBuilder, box, cylinder, validate_robot

def build_robot():
    # Initialize builder
    builder = RobotBuilder("diff_drive")

    # 1. Base link
    builder.link("base_link") \
        .visual(box(0.4, 0.3, 0.1)) \
        .collision() \
        .mass(5.0) \
        .root()

    # 2. Left wheel
    builder.link("left_wheel", parent="base_link") \
        .visual(cylinder(0.1, 0.05), rpy=(1.57, 0, 0)) \
        .collision() \
        .mass(1.0) \
        .continuous(axis=(0, 1, 0), xyz=(0.1, 0.15, 0)) \
        .ros2_control(
            command_interfaces=["velocity"],
            state_interfaces=["position", "velocity"]
        ) \
        .commit()

    # 3. Right wheel
    builder.link("right_wheel", parent="base_link") \
        .visual(cylinder(0.1, 0.05), rpy=(1.57, 0, 0)) \
        .collision() \
        .mass(1.0) \
        .continuous(axis=(0, 1, 0), xyz=(0.1, -0.15, 0)) \
        .ros2_control(
            command_interfaces=["velocity"],
            state_interfaces=["position", "velocity"]
        ) \
        .commit()

    # 4. LiDAR Link & Sensor
    builder.link("lidar_link", parent="base_link") \
        .visual(cylinder(0.05, 0.06)) \
        .collision() \
        .mass(0.2) \
        .fixed(xyz=(0.15, 0, 0.08)) \
        .lidar(name="chassis_laser", range_min=0.1, range_max=10.0, samples=360) \
        .commit()

    # Validate
    result = validate_robot(builder.robot)
    if not result.is_valid:
        print("Validation errors:")
        for err in result.errors:
            print(f"  - {err.message}")
        return

    # Export
    with open("diff_drive.urdf", "w") as f:
        f.write(builder.export_urdf())
    with open("diff_drive.srdf", "w") as f:
        f.write(builder.export_srdf())
    print("✓ Assembled and exported Robot URDF & SRDF successfully!")

if __name__ == "__main__":
    build_robot()
```
