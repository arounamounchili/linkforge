"""Physics and inertial property calculations.

This sub-package implements the mathematical engines for calculating
mass, center of mass, and inertia tensors. It supports both standard
geometric primitives and complex mesh data, ensuring physically accurate
robot simulations.
"""

from .inertia import (
    calculate_box_inertia,
    calculate_cylinder_inertia,
    calculate_inertia,
    calculate_mesh_inertia_approximation,
    calculate_mesh_inertia_from_triangles,
    calculate_sphere_inertia,
)
from .mesh_validation import validate_mesh_topology

__all__ = [
    # Inertia
    "calculate_inertia",
    "calculate_box_inertia",
    "calculate_cylinder_inertia",
    "calculate_sphere_inertia",
    "calculate_mesh_inertia_approximation",
    "calculate_mesh_inertia_from_triangles",
    # Validation
    "validate_mesh_topology",
]
