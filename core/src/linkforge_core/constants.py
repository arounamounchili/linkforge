"""Central constants and configuration defaults for the LinkForge ecosystem.

This module provides industry-standard physical constants, XML/XACRO
namespace registries, and sanity limits used during robot validation
to ensure simulation stability.

Attributes:
    DEFAULT_FRICTION_MU: Standard static friction coefficient.
    DEFAULT_CONTACT_KP: High-fidelity contact stiffness for hard surfaces.
    XACRO_URIS: Set of supported XACRO namespace identifiers.
    MAX_REASONABLE_FLOAT: Guardrail to prevent simulation-breaking overflows.
    EPSILON: Small value for floating point comparisons.
    MIN_REASONABLE_MASS: Minimum mass to prevent singular matrices.
    MIN_REASONABLE_INERTIA: Minimum inertia to prevent solver crashes.
    DEFAULT_JOINT_DAMPING: Default damping for kinematic joints.
"""

from __future__ import annotations

# Physics Defaults (Simulation)
# ----------------------------

# Default static friction coefficient (Coulomb)
DEFAULT_FRICTION_MU = 1.0

# Default dynamic friction coefficient (Coulomb)
DEFAULT_FRICTION_MU2 = 1.0

# Default contact stiffness (N/m)
# 1e12 is the industry standard for 'hard' contact in Gazebo/GZ
DEFAULT_CONTACT_KP = 1e12

# Default contact damping (N s/m)
DEFAULT_CONTACT_KD = 1.0

# Default gravity inclusion
DEFAULT_GRAVITY = True

# Default self-collision inclusion
DEFAULT_SELF_COLLIDE = False


# XML and XACRO Namespaces
# ----------------------------

# Official XACRO namespace URIs
XACRO_URIS = {
    "http://www.ros.org/wiki/xacro",
    "http://wiki.ros.org/xacro",
    "http://ros.org/xacro",
}

# Standard prefix for internal structural processing
XACRO_PREFIX = "xacro:"


# Validation Limits (Sanity Checks)
# ----------------------------

# Maximum absolute value allowed for floats in robot models
# 1e18 is safe for stiffness (kp) while preventing simulation-breaking overflows
MAX_REASONABLE_FLOAT = 1e18

# Maximum absolute value allowed for integers (IDs, sample counts, etc.)
MAX_REASONABLE_INT = 1000000

# Numerical Stability (Guardrails)
# ----------------------------

# Small value for floating point comparisons
EPSILON = 1e-9

# Minimum mass in kg to prevent singular matrices in dynamics solvers
MIN_REASONABLE_MASS = 1e-6

# Minimum inertia diagonal value to prevent zero-inertia crashes
MIN_REASONABLE_INERTIA = 1e-9


# Joint Dynamics Defaults
# ----------------------------

# Default joint damping (N s / m or N m s / rad)
DEFAULT_JOINT_DAMPING = 0.0

# Default joint friction (N or N m)
DEFAULT_JOINT_FRICTION = 0.0
