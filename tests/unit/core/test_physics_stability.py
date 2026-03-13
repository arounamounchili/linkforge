import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import InertiaTensor


def test_inertia_numerical_stability_epsilon() -> None:
    """Verify that inertia tensor allows minor violations within 1e-9 tolerance for real-world jitter."""
    # Case: ixx + iyy = izz - 1e-10 (should PASS due to epsilon)
    tensor = InertiaTensor(
        ixx=1.0,
        ixy=0.0,
        ixz=0.0,
        iyy=1.0,
        iyz=0.0,
        izz=2.0 + 1e-10,
    )
    assert tensor.izz == 2.0 + 1e-10


def test_inertia_validation_rejection_threshold() -> None:
    """Verify that inertia tensor correctly rejects massive violations beyond the permitted epsilon."""
    # Case: ixx + iyy = izz - 1e-8 (should FAIL)
    with pytest.raises(RobotModelError, match="Inertia tensor violates triangle inequality"):
        InertiaTensor(
            ixx=1.0,
            ixy=0.0,
            ixz=0.0,
            iyy=1.0,
            iyz=0.0,
            izz=2.0 + 1e-8,
        )
