"""Unit tests for validation edge cases and base logic coverage."""

from pathlib import Path

import pytest
from linkforge.core.base import RobotGenerator, RobotParser
from linkforge.core.exceptions import RobotModelError
from linkforge.core.models.link import Inertial, Link
from linkforge.core.models.robot import Robot
from linkforge.core.validation.checks import (
    GeometryCheck,
    MassPropertiesCheck,
    SemanticCheck,
    TreeStructureCheck,
    ValidationCheck,
)
from linkforge.core.validation.result import ValidationResult


class DummyGenerator(RobotGenerator[str]):
    def generate(self, robot: Robot, **kwargs) -> str:
        super().generate(robot, **kwargs)  # type: ignore
        return "done"


class DummyParser(RobotParser[str]):
    def parse(self, filepath: Path, **kwargs) -> str:
        super().parse(filepath, **kwargs)  # type: ignore
        return "done"

    def parse_string(self, content: str, **kwargs) -> str:
        super().parse_string(content, **kwargs)  # type: ignore
        return "done"


class DummyCheck(ValidationCheck):
    def run(self, robot: Robot, result: ValidationResult) -> None:
        super().run(robot, result)  # type: ignore


def test_base_abstract_methods_coverage():
    """Call abstract methods to ensure 100% coverage of base.py pass statements."""
    robot = Robot(name="test")
    gen = DummyGenerator()
    assert gen.generate(robot) == "done"

    parser = DummyParser()
    assert parser.parse(Path("test")) == "done"
    assert parser.parse_string("test") == "done"

    check = DummyCheck()
    check.run(robot, ValidationResult())  # type: ignore


def test_check_root_model_error_coverage(mocker):
    """Cover RobotModelError path in TreeStructureCheck._check_root."""
    check = TreeStructureCheck()
    robot = Robot(name="test_robot")
    result = ValidationResult()

    # We need to make sure HasLinksCheck doesn't stop us
    robot.add_link(Link(name="link1"))

    # Mock robot.root_link to raise RobotModelError
    # Note: root_link is a property, but Robot.root_link might be tricky to patch on instance
    # if it's already computed.
    mocker.patch(
        "linkforge.core.models.robot.Robot.root_link",
        new_callable=mocker.PropertyMock,
        side_effect=RobotModelError("Model fail"),
    )

    # This calls _check_root internally
    check.run(robot, result)

    assert any(
        "Kinematic graph error" in err.title or "Kinematic error" in err.title
        for err in result.errors
    )


def test_geometry_check_warnings():
    """Cover missing visual/collision warnings in GeometryCheck."""
    check = GeometryCheck()
    robot = Robot(name="test_robot")
    # Add a link with no visuals/collisions
    link = Link(name="empty_link")
    robot.add_link(link)

    result = ValidationResult()
    check.run(robot, result)

    assert any("No visual geometry" in w.title for w in result.warnings)
    assert any("No collision geometry" in w.title for w in result.warnings)


def test_semantic_check_no_semantic():
    """Cover early return in SemanticCheck if no semantic model."""
    check = SemanticCheck()
    # Create robot with default semantic (which is empty)
    robot = Robot(name="test_robot")
    # Force semantic to None to hit the branch if applicable,
    # but wait, SemanticCheck checks 'if not robot.semantic:'
    # In Robot, semantic is field(default_factory=SemanticRobotDescription)
    # SemanticRobotDescription is always truthy unless we mock it.

    mocker_semantic = pytest.importorskip("linkforge.core.models.srdf").SemanticRobotDescription
    # Actually if it's a dataclass it might be truthy.

    robot.semantic = None  # type: ignore
    result = ValidationResult()
    check.run(robot, result)
    assert not result.errors


def test_mass_properties_critical_low_mass():
    """Cover critical low mass error path."""
    check = MassPropertiesCheck()
    robot = Robot(name="test_robot")
    link = Link(name="light_link", inertial=Inertial(mass=1e-12))  # Below MIN_REASONABLE_MASS
    robot.add_link(link)

    result = ValidationResult()
    check.run(robot, result)
    assert any("Critical low mass" in err.title for err in result.errors)


class FailingCheck(ValidationCheck):
    def run(self, robot: Robot, result: ValidationResult) -> None:
        raise ValueError("Simulated failure")


def test_robot_validator_check_exception() -> None:
    """Verify that RobotValidator catches exceptions in validation checks."""
    from linkforge.core.validation.validator import RobotValidator

    robot = Robot(name="test_robot")
    validator = RobotValidator(checks=[FailingCheck()])
    result = validator.validate(robot)
    assert any(
        "Validation check FailingCheck failed: Simulated failure" in err.message
        for err in result.errors
    )


def test_robot_validator_reindex_exception(mocker) -> None:
    """Verify that RobotValidator catches indexing errors from _reindex."""
    from linkforge.core.exceptions import RobotValidationError, ValidationErrorCode
    from linkforge.core.validation.validator import RobotValidator

    robot = Robot(name="test_robot")
    mocker.patch.object(
        robot,
        "_reindex",
        side_effect=RobotValidationError(
            ValidationErrorCode.DUPLICATE_NAME, "Duplicate name detected", value="link1"
        ),
    )
    validator = RobotValidator(checks=[])
    result = validator.validate(robot)
    assert any("Duplicate name detected" in err.message for err in result.errors)
