"""Robot validation and security utilities.

This sub-package provides the logic for ensuring robot models are structurally
sound and safe for export. It includes checks for kinematic connectivity,
physics validity, and semantic validation (e.g. SRDF collision matrices),
as well as security validation for external assets.
"""

from __future__ import annotations

from .checks import (
    DuplicateNameCheck,
    GeometryCheck,
    HasLinksCheck,
    JointReferenceCheck,
    MassPropertiesCheck,
    MimicChainCheck,
    Ros2ControlCheck,
    SemanticCheck,
    TreeStructureCheck,
    ValidationCheck,
)
from .result import Severity, ValidationIssue, ValidationResult
from .security import (
    find_sandbox_root,
    is_suspicious_location,
    validate_mesh_path,
    validate_package_uri,
)
from .validator import RobotValidator

__all__ = [
    # Abstract base
    "ValidationCheck",
    # Concrete checks
    "HasLinksCheck",
    "DuplicateNameCheck",
    "JointReferenceCheck",
    "TreeStructureCheck",
    "MassPropertiesCheck",
    "GeometryCheck",
    "Ros2ControlCheck",
    "MimicChainCheck",
    # Result types
    "ValidationResult",
    "ValidationIssue",
    "Severity",
    # Orchestrator
    "RobotValidator",
    # Security helpers
    "is_suspicious_location",
    "validate_mesh_path",
    "validate_package_uri",
    "find_sandbox_root",
    "SemanticCheck",
]
