import pytest
from linkforge.core import (
    Inertial,
    InertiaTensor,
    Link,
    Robot,
    RobotModelError,
    ValidationResult,
)
from linkforge.core.constants import (
    EPSILON,
    MIN_REASONABLE_INERTIA,
    MIN_REASONABLE_MASS,
)
from linkforge.core.validation import MassPropertiesCheck


def test_inertia_numerical_stability_epsilon() -> None:
    """Verify that inertia tensor allows minor violations within EPSILON tolerance."""
    # Case: ixx + iyy = izz - (EPSILON / 10.0) (should PASS)
    small_violation = EPSILON / 10.0
    tensor = InertiaTensor(
        ixx=1.0,
        ixy=0.0,
        ixz=0.0,
        iyy=1.0,
        iyz=0.0,
        izz=2.0 + small_violation,
    )
    assert tensor.izz == 2.0 + small_violation


def test_inertia_validation_rejection_threshold() -> None:
    """Verify that inertia tensor correctly rejects massive violations beyond EPSILON."""
    # Case: ixx + iyy = izz - (EPSILON * 10.0) (should FAIL)
    large_violation = EPSILON * 10.0
    with pytest.raises(RobotModelError):
        InertiaTensor(
            ixx=1.0,
            ixy=0.0,
            ixz=0.0,
            iyy=1.0,
            iyz=0.0,
            izz=2.0 + large_violation,
        )


def test_mass_stability_guardrail():
    """Verify that mass below MIN_REASONABLE_MASS triggers a critical error."""
    dangerously_low_mass = MIN_REASONABLE_MASS / 2.0
    link = Link(name="danger_link", inertial=Inertial(mass=dangerously_low_mass))
    robot = Robot(name="test_robot", links=[link])

    result = ValidationResult()
    check = MassPropertiesCheck()
    check.run(robot, result)

    assert not result.is_valid
    assert any("Critical low mass" in error.title for error in result.errors)
    assert any("near-zero mass" in error.message for error in result.errors)


def test_inertia_stability_guardrail():
    """Verify that inertia below MIN_REASONABLE_INERTIA triggers a critical error."""
    low_val = MIN_REASONABLE_INERTIA / 2.0
    low_inertia = InertiaTensor(ixx=low_val, ixy=0, ixz=0, iyy=low_val, iyz=0, izz=low_val)

    link = Link(name="jitter_link", inertial=Inertial(mass=0.1, inertia=low_inertia))
    robot = Robot(name="test_robot", links=[link])

    result = ValidationResult()
    check = MassPropertiesCheck()
    check.run(robot, result)

    assert not result.is_valid
    assert any("Critical low inertia" in error.title for error in result.errors)


def test_mass_warning_vs_error():
    """Verify that low (but safe) mass only triggers a warning."""
    safe_low_mass = 0.005
    link = Link(name="warning_link", inertial=Inertial(mass=safe_low_mass))
    robot = Robot(name="test_robot", links=[link])

    result = ValidationResult()
    check = MassPropertiesCheck()
    check.run(robot, result)

    assert result.is_valid
    assert result.has_warnings
    assert any("Very low mass" in warn.title for warn in result.warnings)
