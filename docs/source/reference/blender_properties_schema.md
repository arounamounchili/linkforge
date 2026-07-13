# Blender Properties Schema

LinkForge stores all robot data as structured Blender custom properties on scene objects.
This page documents the full schema for all property groups, their keys, types, and default
values.

This schema is the stable foundation of LinkForge. External tools (e.g. Godot, Unity, custom
pipelines) can access this data by enabling **Custom Properties** in Blender's glTF exporter
(`File → Export → glTF 2.0 → Data → Custom Properties`), which embeds all properties as
JSON metadata inside the `.glb` file.

```{important}
These property keys are considered **stable**. They will not be renamed without a major
version bump and a clear migration note in the [CHANGELOG](../CHANGELOG.md).
```

---

## Property Group Keys

Each property group is attached to a Blender `Object` under a fixed attribute name:

| Attribute on `bpy.types.Object` | Applies To | Purpose |
|---|---|---|
| `linkforge` | Link Empty | Robot link physics and identification |
| `linkforge_joint` | Joint Empty | Joint kinematics and limits |
| `linkforge_sensor` | Sensor Empty | Sensor configuration |
| `linkforge_transmission` | Joint Empty | ROS 2 transmission data |
| `linkforge_control` | Robot root | ROS 2 Control hardware interface |

---

## `linkforge` — Link Properties

Stored on **Link Empty** objects (parent of visual/collision meshes).

### Identification

| Key | Type | Default | Description |
|---|---|---|---|
| `is_robot_link` | `bool` | `false` | Marks this object as a robot link |
| `source_name_stored` | `str` | `""` | Persistent robot-model name (immune to Blender `.001` suffixing) |

### Physics

| Key | Type | Default | Description |
|---|---|---|---|
| `mass` | `float` | `1.0` | Mass in kilograms |
| `use_auto_inertia` | `bool` | `true` | Auto-calculate inertia from geometry |
| `inertia_ixx` | `float` | `1.0` | Moment of inertia around X axis (kg·m²) |
| `inertia_iyy` | `float` | `1.0` | Moment of inertia around Y axis (kg·m²) |
| `inertia_izz` | `float` | `1.0` | Moment of inertia around Z axis (kg·m²) |
| `inertia_ixy` | `float` | `0.0` | Product of inertia XY (kg·m²) |
| `inertia_ixz` | `float` | `0.0` | Product of inertia XZ (kg·m²) |
| `inertia_iyz` | `float` | `0.0` | Product of inertia YZ (kg·m²) |
| `inertia_origin_xyz` | `float[3]` | `[0,0,0]` | Center of mass offset (meters) |
| `inertia_origin_rpy` | `float[3]` | `[0,0,0]` | Center of mass rotation (radians, XYZ) |

### Collision

| Key | Type | Default | Description |
|---|---|---|---|
| `collision_type` | `str` (enum) | `"auto"` | Collision shape: `"auto"`, `"box"`, `"sphere"`, `"cylinder"`, `"mesh"` |
| `collision_quality` | `float` | `50.0` | Mesh simplification percentage (1–100%) |

### Material

| Key | Type | Default | Description |
|---|---|---|---|
| `use_material` | `bool` | `false` | Export Blender material color to URDF |

### Advanced Simulation (Gazebo)

| Key | Type | Default | Description |
|---|---|---|---|
| `use_simulation_props` | `bool` | `false` | Include Gazebo-specific simulation properties |
| `self_collide` | `bool` | `false` | Allow self-collision with other links |
| `gravity` | `bool` | `true` | Affected by gravity |
| `mu` | `float` | `1.0` | Static (Coulomb) friction coefficient |
| `mu2` | `float` | `1.0` | Dynamic friction coefficient |
| `kp` | `float` | `1e12` | Contact stiffness (N/m) |
| `kd` | `float` | `1.0` | Contact damping (N·s/m) |

---

## `linkforge_joint` — Joint Properties

Stored on **Joint Empty** objects (colored ARROWS empties).

### Identification

| Key | Type | Default | Description |
|---|---|---|---|
| `is_robot_joint` | `bool` | `false` | Marks this object as a robot joint |
| `source_name_stored` | `str` | `""` | Persistent robot-model name |

### Kinematics

| Key | Type | Default | Description |
|---|---|---|---|
| `joint_type` | `str` (enum) | `"revolute"` | `"revolute"`, `"continuous"`, `"prismatic"`, `"fixed"`, `"floating"`, `"planar"` |
| `parent_link` | `Object ref` | `None` | The parent link object |
| `child_link` | `Object ref` | `None` | The child link object |
| `axis` | `str` (enum) | `"Z"` | Joint motion axis: `"X"`, `"Y"`, `"Z"`, `"CUSTOM"` |
| `custom_axis_x` | `float` | `0.0` | Custom axis X component (normalized) |
| `custom_axis_y` | `float` | `0.0` | Custom axis Y component (normalized) |
| `custom_axis_z` | `float` | `1.0` | Custom axis Z component (normalized) |

### Limits

| Key | Type | Default | Description |
|---|---|---|---|
| `use_limits` | `bool` | `false` | Enable position limits |
| `limit_lower` | `float` | `-π` | Lower position limit (rad or m) |
| `limit_upper` | `float` | `+π` | Upper position limit (rad or m) |
| `limit_effort` | `float` | `10.0` | Maximum force/torque (N or N·m) |
| `limit_velocity` | `float` | `1.0` | Maximum velocity (rad/s or m/s) |

### Dynamics

| Key | Type | Default | Description |
|---|---|---|---|
| `use_dynamics` | `bool` | `false` | Enable friction/damping |
| `dynamics_damping` | `float` | `0.0` | Resistance to motion |
| `dynamics_friction` | `float` | `0.0` | Static friction |

### Mimic

| Key | Type | Default | Description |
|---|---|---|---|
| `use_mimic` | `bool` | `false` | Copy another joint's movement |
| `mimic_joint` | `Object ref` | `None` | Joint to copy from |
| `mimic_multiplier` | `float` | `1.0` | Movement scale factor |
| `mimic_offset` | `float` | `0.0` | Position offset after multiplier |

---

## `linkforge_sensor` — Sensor Properties

Stored on **Sensor Empty** objects.

### Identification

| Key | Type | Default | Description |
|---|---|---|---|
| `is_robot_sensor` | `bool` | `false` | Marks this object as a robot sensor |
| `source_name_stored` | `str` | `""` | Persistent robot-model name |
| `sensor_type` | `str` (enum) | `"camera"` | `"camera"`, `"depth_camera"`, `"lidar"`, `"gpu_lidar"`, `"imu"`, `"gps"`, `"contact"`, `"force_torque"` |
| `attached_link` | `Object ref` | `None` | The link this sensor is mounted on |

### Common

| Key | Type | Default | Description |
|---|---|---|---|
| `update_rate` | `float` | `30.0` | Sensor update frequency (Hz) |
| `always_on` | `bool` | `true` | Keep sensor active at all times |
| `visualize` | `bool` | `false` | Visualize sensor in simulator |
| `topic_name` | `str` | `""` | ROS topic name for sensor data |

### Camera / Depth Camera

| Key | Type | Default | Description |
|---|---|---|---|
| `camera_horizontal_fov` | `float` | `1.047` | Horizontal field of view (radians) |
| `camera_width` | `int` | `640` | Image width (pixels) |
| `camera_height` | `int` | `480` | Image height (pixels) |
| `camera_near_clip` | `float` | `0.1` | Near clipping distance (m) |
| `camera_far_clip` | `float` | `100.0` | Far clipping distance (m) |
| `camera_format` | `str` (enum) | `"R8G8B8"` | Pixel format |

### LIDAR / GPU LIDAR

| Key | Type | Default | Description |
|---|---|---|---|
| `lidar_horizontal_samples` | `int` | `360` | Horizontal scan samples |
| `lidar_horizontal_min_angle` | `float` | `-π` | Min horizontal angle (radians) |
| `lidar_horizontal_max_angle` | `float` | `+π` | Max horizontal angle (radians) |
| `lidar_vertical_samples` | `int` | `1` | Vertical scan samples (1 = 2D) |
| `lidar_vertical_min_angle` | `float` | `0.0` | Min vertical angle (radians) |
| `lidar_vertical_max_angle` | `float` | `0.0` | Max vertical angle (radians) |
| `lidar_range_min` | `float` | `0.08` | Minimum range (m) |
| `lidar_range_max` | `float` | `10.0` | Maximum range (m) |
| `lidar_range_resolution` | `float` | `0.01` | Range resolution (m) |

### Noise

| Key | Type | Default | Description |
|---|---|---|---|
| `use_noise` | `bool` | `false` | Add noise model to sensor output |
| `noise_type` | `str` (enum) | `"gaussian"` | `"gaussian"` or `"gaussian_quantized"` |
| `noise_mean` | `float` | `0.0` | Noise mean |
| `noise_stddev` | `float` | `0.0` | Noise standard deviation |

---

## Raw ID Properties (Mesh children)

These are plain Blender ID properties (not PropertyGroups) set directly on visual and
collision mesh objects using `object["key"] = value`:

| Key | Type | Description |
|---|---|---|
| `collision_geometry_type` | `str` | Shape type: `"box"`, `"sphere"`, `"cylinder"`, `"mesh"` |
| `imported_from_source` | `bool` | `true` if object was created by importing a URDF |
| `source_geometry_type` | `str` | Original geometry type from imported URDF |
| `source_name` | `str` | Original name from imported URDF |
