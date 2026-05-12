"""Central constants for the LinkForge project."""

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
