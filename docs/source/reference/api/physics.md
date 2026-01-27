# Physics Calculations

Inertia tensor calculations for various geometries.

## Inertia Calculations

```{eval-rst}
.. automodule:: linkforge_core.physics.inertia
   :members:
   :undoc-members:
   :show-inheritance:
```

## Usage Examples

### Box Inertia

```python
from linkforge_core.physics.inertia import calculate_box_inertia
from linkforge_core.models.geometry import Box, Vector3

box = Box(size=Vector3(1.0, 0.5, 0.3))
inertia = calculate_box_inertia(box, mass=10.0)

print(f"Ixx: {inertia.ixx}")
print(f"Iyy: {inertia.iyy}")
print(f"Izz: {inertia.izz}")
```

### Cylinder Inertia

```python
from linkforge_core.physics.inertia import calculate_cylinder_inertia
from linkforge_core.models.geometry import Cylinder

cylinder = Cylinder(radius=0.1, length=0.5)
inertia = calculate_cylinder_inertia(cylinder, mass=5.0)
```

### Sphere Inertia

```python
from linkforge_core.physics.inertia import calculate_sphere_inertia
from linkforge_core.models.geometry import Sphere

sphere = Sphere(radius=0.2)
inertia = calculate_sphere_inertia(sphere, mass=3.0)
```

### Mesh Inertia

```python
from linkforge_core.physics.inertia import calculate_mesh_inertia
from linkforge_core.models.geometry import Mesh
from pathlib import Path

mesh = Mesh(filepath=Path("robot_part.stl"))
inertia = calculate_mesh_inertia(mesh, mass=2.5)
# Calculates inertia from mesh triangles
```

## Formulas

### Box

For a box with dimensions (x, y, z) and mass m:

```
Ixx = (m/12) * (y² + z²)
Iyy = (m/12) * (x² + z²)
Izz = (m/12) * (x² + y²)
```

### Cylinder

For a cylinder with radius r, length l, and mass m (axis along Z):

```
Ixx = Iyy = (m/12) * (3r² + l²)
Izz = (m/2) * r²
```

### Sphere

For a sphere with radius r and mass m:

```
Ixx = Iyy = Izz = (2/5) * m * r²
```
