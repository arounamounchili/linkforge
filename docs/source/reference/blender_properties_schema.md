# Blender Properties Schema

LinkForge stores all robot data as structured Blender custom properties on scene objects.
This page is the authoritative reference for every property key, its type, and its default value.

## Why This Matters — Using LinkForge Data in External Tools

Because all LinkForge data is stored as standard Blender custom properties, you can export it
to any glTF-compatible tool (Godot, Unity, custom pipelines) **without writing a URDF importer**:

1. In Blender, go to **File → Export → glTF 2.0**.
2. Under **Data**, enable **Custom Properties**.
3. All LinkForge robot data (mass, joints, limits, sensors…) will be embedded as JSON metadata
   inside the `.glb` file and readable in Godot via `node.get_meta("linkforge")`.

```{important}
These property keys are **stable**. They will not be renamed without a major version bump and
a migration note in the [CHANGELOG](../CHANGELOG.md).
```

---

## Property Group Keys

Each group is registered on `bpy.types.Object` under a fixed attribute name.
When exported to glTF, this attribute name becomes the JSON metadata key.

| glTF Metadata Key | Blender Object Type | Purpose |
|---|---|---|
| `linkforge` | Link Empty | Link identification, physics, inertia |
| `linkforge_joint` | Joint Empty (ARROWS) | Joint type, axis, limits, mimic |
| `linkforge_sensor` | Sensor Empty | Camera, LIDAR, IMU, GPS, Contact, FT |
| `linkforge_transmission` | Transmission Empty | ROS 2 transmission / gear ratios |

> **Note:** `linkforge_control` and `linkforge_robot` are stored on the **Scene**, not on
> individual objects. They are not exported to glTF Custom Properties.

---

## `linkforge` — Link Properties

Stored on **Link Empty** objects (the parent empty of visual and collision meshes).

### Identification

| Key | Type | Default | Description |
|---|---|---|---|
| `is_robot_link` | `bool` | `false` | `true` if this object is a robot link |
| `source_name_stored` | `str` | `""` | Stable robot-model name, immune to Blender `.001` suffixing |

### Physics

| Key | Type | Default | Description |
|---|---|---|---|
| `mass` | `float` | `1.0` | Link mass in kilograms |
| `use_auto_inertia` | `bool` | `true` | When `true`, inertia is computed from geometry automatically |
| `inertia_ixx` | `float` | `1.0` | Moment of inertia — X axis (kg·m²) |
| `inertia_iyy` | `float` | `1.0` | Moment of inertia — Y axis (kg·m²) |
| `inertia_izz` | `float` | `1.0` | Moment of inertia — Z axis (kg·m²) |
| `inertia_ixy` | `float` | `0.0` | Cross product of inertia XY (kg·m²) |
| `inertia_ixz` | `float` | `0.0` | Cross product of inertia XZ (kg·m²) |
| `inertia_iyz` | `float` | `0.0` | Cross product of inertia YZ (kg·m²) |
| `inertia_origin_xyz` | `float[3]` | `[0, 0, 0]` | Center-of-mass position offset from link frame (meters) |
| `inertia_origin_rpy` | `float[3]` | `[0, 0, 0]` | Center-of-mass orientation from link frame (radians, XYZ Euler) |

> **Tip:** `inertia_ixx / iyy / izz` are only read by LinkForge when `use_auto_inertia` is `false`.

### Collision

| Key | Type | Default | Description |
|---|---|---|---|
| `collision_type` | `str` | `"auto"` | Collision shape. One of: `"auto"`, `"box"`, `"sphere"`, `"cylinder"`, `"mesh"` |
| `collision_quality` | `float` | `50.0` | Mesh simplification level (1–100%). Only used when `collision_type` is `"mesh"` |

### Material

| Key | Type | Default | Description |
|---|---|---|---|
| `use_material` | `bool` | `false` | When `true`, exports the Blender material color as a URDF `<material>` tag |

### Advanced Simulation (Gazebo)

Only exported when `use_simulation_props` is `true`.

| Key | Type | Default | Description |
|---|---|---|---|
| `use_simulation_props` | `bool` | `false` | Enable Gazebo-specific `<gazebo>` tag export |
| `gravity` | `bool` | `true` | Whether this link is affected by gravity |
| `self_collide` | `bool` | `false` | Whether this link collides with other links in the same robot |
| `mu` | `float` | `1.0` | Static (Coulomb) friction coefficient |
| `mu2` | `float` | `1.0` | Dynamic friction coefficient |
| `kp` | `float` | `1e12` | Contact stiffness (N/m) |
| `kd` | `float` | `1.0` | Contact damping (N·s/m) |

---

## `linkforge_joint` — Joint Properties

Stored on **Joint Empty** objects (ARROWS empties with colored axes).

### Identification

| Key | Type | Default | Description |
|---|---|---|---|
| `is_robot_joint` | `bool` | `false` | `true` if this object is a robot joint |
| `source_name_stored` | `str` | `""` | Stable robot-model name, immune to Blender `.001` suffixing |

### Kinematics

| Key | Type | Default | Description |
|---|---|---|---|
| `joint_type` | `str` | `"revolute"` | One of: `"revolute"`, `"continuous"`, `"prismatic"`, `"fixed"`, `"floating"`, `"planar"` |
| `parent_link` | `Object pointer` | `None` | Reference to the parent link object |
| `child_link` | `Object pointer` | `None` | Reference to the child link object |
| `axis` | `str` | `"Z"` | Motion axis. One of: `"X"`, `"Y"`, `"Z"`, `"CUSTOM"` |
| `custom_axis_x` | `float` | `0.0` | Custom axis X component (auto-normalized to unit vector) |
| `custom_axis_y` | `float` | `0.0` | Custom axis Y component (auto-normalized to unit vector) |
| `custom_axis_z` | `float` | `1.0` | Custom axis Z component (auto-normalized to unit vector) |

> **Note:** `custom_axis_*` are only read when `axis` is `"CUSTOM"`.

### Limits

| Key | Type | Default | Description |
|---|---|---|---|
| `use_limits` | `bool` | `false` | Enable joint position limits |
| `limit_lower` | `float` | `-3.14159` | Lower position limit — radians for revolute/continuous, meters for prismatic |
| `limit_upper` | `float` | `+3.14159` | Upper position limit — radians for revolute/continuous, meters for prismatic |
| `limit_effort` | `float` | `10.0` | Maximum force or torque the actuator can apply (N or N·m) |
| `limit_velocity` | `float` | `1.0` | Maximum joint velocity (rad/s or m/s) |

### Dynamics

| Key | Type | Default | Description |
|---|---|---|---|
| `use_dynamics` | `bool` | `false` | Include dynamics in the exported URDF |
| `dynamics_damping` | `float` | `0.0` | Viscous damping — resistance to motion |
| `dynamics_friction` | `float` | `0.0` | Static friction — resistance to starting motion |

### Mimic

| Key | Type | Default | Description |
|---|---|---|---|
| `use_mimic` | `bool` | `false` | When `true`, this joint copies another joint's movement |
| `mimic_joint` | `Object pointer` | `None` | The joint to copy movement from |
| `mimic_multiplier` | `float` | `1.0` | Scale factor applied to the mimic joint's position |
| `mimic_offset` | `float` | `0.0` | Constant offset added after applying the multiplier |

### Safety Controller

| Key | Type | Default | Description |
|---|---|---|---|
| `use_safety_controller` | `bool` | `false` | Include a safety controller in the exported URDF |
| `safety_soft_lower_limit` | `float` | `0.0` | Soft lower position bound |
| `safety_soft_upper_limit` | `float` | `0.0` | Soft upper position bound |
| `safety_k_position` | `float` | `0.0` | Position gain for the safety controller |
| `safety_k_velocity` | `float` | `0.0` | Velocity gain for the safety controller |

### Calibration

| Key | Type | Default | Description |
|---|---|---|---|
| `use_calibration` | `bool` | `false` | Include joint calibration data in the exported URDF |
| `use_calibration_rising` | `bool` | `false` | Include a rising-edge reference position |
| `calibration_rising` | `float` | `0.0` | Rising-edge reference position (when enabled) |
| `use_calibration_falling` | `bool` | `false` | Include a falling-edge reference position |
| `calibration_falling` | `float` | `0.0` | Falling-edge reference position (when enabled) |

---

## `linkforge_sensor` — Sensor Properties

Stored on **Sensor Empty** objects.

### Identification & Common

| Key | Type | Default | Description |
|---|---|---|---|
| `is_robot_sensor` | `bool` | `false` | `true` if this object is a robot sensor |
| `source_name_stored` | `str` | `""` | Stable robot-model name |
| `sensor_type` | `str` | `"camera"` | One of: `"camera"`, `"depth_camera"`, `"lidar"`, `"gpu_lidar"`, `"imu"`, `"gps"`, `"contact"`, `"force_torque"` |
| `attached_link` | `Object pointer` | `None` | The link this sensor is physically mounted on |
| `update_rate` | `float` | `30.0` | Data output frequency (Hz) |
| `always_on` | `bool` | `true` | Keep the sensor active even when the robot is idle |
| `visualize` | `bool` | `false` | Render sensor geometry in the simulator viewport |
| `topic_name` | `str` | `""` | ROS 2 topic name for the sensor data stream |

### Camera / Depth Camera

| Key | Type | Default | Description |
|---|---|---|---|
| `camera_horizontal_fov` | `float` | `1.047` | Horizontal field of view in radians (≈ 60°) |
| `camera_width` | `int` | `640` | Image width in pixels |
| `camera_height` | `int` | `480` | Image height in pixels |
| `camera_near_clip` | `float` | `0.1` | Near clipping distance (meters) |
| `camera_far_clip` | `float` | `100.0` | Far clipping distance (meters) |
| `camera_format` | `str` | `"R8G8B8"` | Pixel format. One of: `"R8G8B8"`, `"R16G16B16"`, `"L8"`, `"L16"`, `"BAYER_RGGB8"`, `"BAYER_BGGR8"` |

### LIDAR / GPU LIDAR

| Key | Type | Default | Description |
|---|---|---|---|
| `lidar_horizontal_samples` | `int` | `360` | Number of horizontal scan rays |
| `lidar_horizontal_min_angle` | `float` | `-3.14159` | Start angle of the horizontal scan (radians) |
| `lidar_horizontal_max_angle` | `float` | `+3.14159` | End angle of the horizontal scan (radians) |
| `lidar_vertical_samples` | `int` | `1` | Number of vertical scan layers (set to `1` for 2D LIDAR) |
| `lidar_vertical_min_angle` | `float` | `0.0` | Start angle of the vertical scan (radians) |
| `lidar_vertical_max_angle` | `float` | `0.0` | End angle of the vertical scan (radians) |
| `lidar_range_min` | `float` | `0.08` | Minimum detectable range (meters) |
| `lidar_range_max` | `float` | `10.0` | Maximum detectable range (meters) |
| `lidar_range_resolution` | `float` | `0.01` | Range measurement precision (meters) |

### Contact Sensor

| Key | Type | Default | Description |
|---|---|---|---|
| `contact_collision` | `str` | `""` | Collision element name to monitor. Defaults to `<linkname>_collision` if empty |

### Noise Model

| Key | Type | Default | Description |
|---|---|---|---|
| `use_noise` | `bool` | `false` | Add a noise model to simulate realistic sensor imprecision |
| `noise_type` | `str` | `"gaussian"` | Noise distribution. One of: `"gaussian"`, `"gaussian_quantized"` |
| `noise_mean` | `float` | `0.0` | Mean of the noise distribution |
| `noise_stddev` | `float` | `0.0` | Standard deviation of the noise distribution |

### Gazebo Plugin

| Key | Type | Default | Description |
|---|---|---|---|
| `use_gazebo_plugin` | `bool` | `false` | Attach a Gazebo plugin to this sensor |
| `plugin_filename` | `str` | `""` | Plugin `.so` filename (e.g. `libgazebo_ros_camera.so`) |

---

## `linkforge_transmission` — Transmission Properties

Stored on **Transmission Empty** objects. Used for ROS 2 `<transmission>` tags.

### Identification

| Key | Type | Default | Description |
|---|---|---|---|
| `is_robot_transmission` | `bool` | `false` | `true` if this object is a robot transmission |
| `source_name_stored` | `str` | `""` | Stable robot-model name |
| `transmission_type` | `str` | `"simple"` | One of: `"simple"`, `"differential"`, `"four_bar_linkage"`, `"custom"` |
| `custom_type` | `str` | `""` | Custom type identifier (only used when `transmission_type` is `"custom"`) |

### Joints & Actuators

| Key | Type | Default | Description |
|---|---|---|---|
| `joint_name` | `Object pointer` | `None` | Controlled joint (for `"simple"` transmissions) |
| `joint1_name` | `Object pointer` | `None` | First joint (for `"differential"` transmissions) |
| `joint2_name` | `Object pointer` | `None` | Second joint (for `"differential"` transmissions) |
| `hardware_interface` | `str` | `"position"` | ROS 2 Control interface. One of: `"position"`, `"velocity"`, `"effort"` |
| `mechanical_reduction` | `float` | `1.0` | Gear reduction ratio (actuator / joint) |
| `offset` | `float` | `0.0` | Joint position offset (radians or meters) |
| `use_custom_actuator_name` | `bool` | `false` | Use a manually specified actuator name |
| `actuator_name` | `str` | `""` | Actuator name (when `use_custom_actuator_name` is `true`) |
| `actuator1_name` | `str` | `""` | First actuator name (for `"differential"` transmissions) |
| `actuator2_name` | `str` | `""` | Second actuator name (for `"differential"` transmissions) |

---

## Raw ID Properties (Visual & Collision Mesh Children)

These are plain Blender ID properties set directly on child mesh objects
using `object["key"] = value`. They are visible in glTF exports as metadata.

| Key | Type | Description |
|---|---|---|
| `collision_geometry_type` | `str` | Resolved shape: `"box"`, `"sphere"`, `"cylinder"`, or `"mesh"` |
| `imported_from_source` | `bool` | `true` if this object was created by importing a URDF |
| `source_geometry_type` | `str` | Original geometry type from the imported URDF |
| `source_name` | `str` | Original object name from the imported URDF |
