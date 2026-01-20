"""Tests for transmission models."""

from __future__ import annotations

import pytest

from linkforge.core.models import (
    HardwareInterface,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
)


class TestTransmissionJoint:
    """Tests for TransmissionJoint model."""

    def test_default_joint(self):
        """Test creating a transmission joint with defaults."""
        joint = TransmissionJoint(name="joint1")
        assert joint.name == "joint1"
        assert joint.hardware_interfaces == ["position"]
        assert joint.mechanical_reduction == 1.0
        assert joint.offset == 0.0

    def test_custom_joint(self):
        """Test creating a transmission joint with custom parameters."""
        joint = TransmissionJoint(
            name="joint1",
            hardware_interfaces=["position", "velocity"],
            mechanical_reduction=50.0,
            offset=0.1,
        )
        assert joint.name == "joint1"
        assert "position" in joint.hardware_interfaces
        assert "velocity" in joint.hardware_interfaces
        assert joint.mechanical_reduction == 50.0
        assert joint.offset == pytest.approx(0.1)

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(ValueError, match="Transmission joint name cannot be empty"):
            TransmissionJoint(name="")

    def test_empty_interfaces(self):
        """Test that empty interfaces raises error."""
        with pytest.raises(ValueError, match="must have at least one hardware interface"):
            TransmissionJoint(name="joint1", hardware_interfaces=[])

    def test_zero_reduction(self):
        """Test that zero mechanical reduction raises error."""
        with pytest.raises(ValueError, match="mechanical reduction cannot be zero"):
            TransmissionJoint(name="joint1", mechanical_reduction=0.0)


class TestTransmissionActuator:
    """Tests for TransmissionActuator model."""

    def test_default_actuator(self):
        """Test creating a transmission actuator with defaults."""
        actuator = TransmissionActuator(name="motor1")
        assert actuator.name == "motor1"
        assert actuator.hardware_interfaces == ["position"]
        assert actuator.mechanical_reduction == 1.0
        assert actuator.offset == 0.0

    def test_custom_actuator(self):
        """Test creating a transmission actuator with custom parameters."""
        actuator = TransmissionActuator(
            name="motor1",
            hardware_interfaces=["effort"],
            mechanical_reduction=100.0,
            offset=-0.05,
        )
        assert actuator.name == "motor1"
        assert "effort" in actuator.hardware_interfaces
        assert actuator.mechanical_reduction == 100.0
        assert actuator.offset == pytest.approx(-0.05)

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(ValueError, match="Transmission actuator name cannot be empty"):
            TransmissionActuator(name="")

    def test_empty_interfaces(self):
        """Test that empty interfaces raises error."""
        with pytest.raises(ValueError, match="must have at least one hardware interface"):
            TransmissionActuator(name="motor1", hardware_interfaces=[])

    def test_zero_reduction(self):
        """Test that zero mechanical reduction raises error."""
        with pytest.raises(ValueError, match="mechanical reduction cannot be zero"):
            TransmissionActuator(name="motor1", mechanical_reduction=0.0)


class TestTransmission:
    """Tests for Transmission model."""

    def test_simple_transmission(self):
        """Test creating a simple transmission."""
        joint = TransmissionJoint(name="joint1", mechanical_reduction=50.0)
        actuator = TransmissionActuator(name="motor1")
        trans = Transmission(
            name="trans1",
            type=TransmissionType.SIMPLE.value,
            joints=[joint],
            actuators=[actuator],
        )
        assert trans.name == "trans1"
        assert trans.type == TransmissionType.SIMPLE.value
        assert len(trans.joints) == 1
        assert len(trans.actuators) == 1

    def test_differential_transmission(self):
        """Test creating a differential transmission."""
        joint1 = TransmissionJoint(name="joint1")
        joint2 = TransmissionJoint(name="joint2")
        actuator1 = TransmissionActuator(name="motor1")
        actuator2 = TransmissionActuator(name="motor2")
        trans = Transmission(
            name="trans1",
            type=TransmissionType.DIFFERENTIAL.value,
            joints=[joint1, joint2],
            actuators=[actuator1, actuator2],
        )
        assert trans.name == "trans1"
        assert len(trans.joints) == 2
        assert len(trans.actuators) == 2

    def test_create_simple_factory(self):
        """Test creating a simple transmission with factory method."""
        trans = Transmission.create_simple(
            name="trans1",
            joint_name="shoulder_joint",
            mechanical_reduction=100.0,
            hardware_interface="effort",
        )
        assert trans.name == "trans1"
        assert trans.type == TransmissionType.SIMPLE.value
        assert len(trans.joints) == 1
        assert trans.joints[0].name == "shoulder_joint"
        assert trans.joints[0].mechanical_reduction == 100.0
        assert trans.joints[0].hardware_interfaces == ["effort"]
        assert len(trans.actuators) == 1
        assert trans.actuators[0].name == "shoulder_joint_motor"

    def test_create_simple_with_custom_actuator_name(self):
        """Test creating simple transmission with custom actuator name."""
        trans = Transmission.create_simple(
            name="trans1",
            joint_name="elbow_joint",
            actuator_name="elbow_motor",
        )
        assert trans.actuators[0].name == "elbow_motor"

    def test_create_differential_factory(self):
        """Test creating a differential transmission with factory method."""
        trans = Transmission.create_differential(
            name="diff_trans",
            joint1_name="left_wheel_joint",
            joint2_name="right_wheel_joint",
            mechanical_reduction=50.0,
            hardware_interface="velocity",
        )
        assert trans.name == "diff_trans"
        assert trans.type == TransmissionType.DIFFERENTIAL.value
        assert len(trans.joints) == 2
        assert trans.joints[0].name == "left_wheel_joint"
        assert trans.joints[1].name == "right_wheel_joint"
        assert trans.joints[0].mechanical_reduction == 50.0
        assert trans.joints[0].hardware_interfaces == ["velocity"]
        assert len(trans.actuators) == 2
        assert trans.actuators[0].name == "left_wheel_joint_motor"
        assert trans.actuators[1].name == "right_wheel_joint_motor"

    def test_create_differential_with_custom_actuator_names(self):
        """Test creating differential transmission with custom actuator names."""
        trans = Transmission.create_differential(
            name="diff_trans",
            joint1_name="joint1",
            joint2_name="joint2",
            actuator1_name="motor_left",
            actuator2_name="motor_right",
        )
        assert trans.actuators[0].name == "motor_left"
        assert trans.actuators[1].name == "motor_right"

    def test_transmission_with_parameters(self):
        """Test transmission with additional parameters."""
        joint = TransmissionJoint(name="joint1")
        trans = Transmission(
            name="trans1",
            type=TransmissionType.CUSTOM.value,
            joints=[joint],
            parameters={"param1": "value1", "param2": "42"},
        )
        assert trans.parameters["param1"] == "value1"
        assert trans.parameters["param2"] == "42"

    def test_empty_name(self):
        """Test that empty name raises error."""
        joint = TransmissionJoint(name="joint1")
        with pytest.raises(ValueError, match="Transmission name cannot be empty"):
            Transmission(
                name="",
                type=TransmissionType.SIMPLE.value,
                joints=[joint],
            )

    def test_empty_type(self):
        """Test that empty type raises error."""
        joint = TransmissionJoint(name="joint1")
        with pytest.raises(ValueError, match="Transmission type cannot be empty"):
            Transmission(
                name="trans1",
                type="",
                joints=[joint],
            )

    def test_invalid_name(self):
        """Test that invalid name raises error."""
        joint = TransmissionJoint(name="joint1")
        with pytest.raises(ValueError, match="contains invalid characters"):
            Transmission(
                name="trans@1",
                type=TransmissionType.SIMPLE.value,
                joints=[joint],
            )

    def test_no_joints(self):
        """Test that transmission without joints raises error."""
        with pytest.raises(ValueError, match="must have at least one joint"):
            Transmission(
                name="trans1",
                type=TransmissionType.SIMPLE.value,
                joints=[],
            )

    def test_duplicate_joint_names(self):
        """Test that duplicate joint names raise error."""
        joint1 = TransmissionJoint(name="joint1")
        joint2 = TransmissionJoint(name="joint1")  # Duplicate
        with pytest.raises(ValueError, match="duplicate joint names"):
            Transmission(
                name="trans1",
                type=TransmissionType.DIFFERENTIAL.value,
                joints=[joint1, joint2],
            )

    def test_duplicate_actuator_names(self):
        """Test that duplicate actuator names raise error."""
        joint = TransmissionJoint(name="joint1")
        actuator1 = TransmissionActuator(name="motor1")
        actuator2 = TransmissionActuator(name="motor1")  # Duplicate
        with pytest.raises(ValueError, match="duplicate actuator names"):
            Transmission(
                name="trans1",
                type=TransmissionType.SIMPLE.value,
                joints=[joint],
                actuators=[actuator1, actuator2],
            )


class TestTransmissionType:
    """Tests for TransmissionType enum."""

    def test_enum_values(self):
        """Test that enum has expected values."""
        assert TransmissionType.SIMPLE.value == "transmission_interface/SimpleTransmission"
        assert (
            TransmissionType.DIFFERENTIAL.value == "transmission_interface/DifferentialTransmission"
        )
        assert (
            TransmissionType.FOUR_BAR_LINKAGE.value
            == "transmission_interface/FourBarLinkageTransmission"
        )
        assert TransmissionType.CUSTOM.value == "custom"


class TestHardwareInterface:
    """Tests for HardwareInterface enum."""

    def test_enum_values(self):
        """Test that enum has expected values."""
        assert HardwareInterface.POSITION.value == "hardware_interface/PositionJointInterface"
        assert HardwareInterface.VELOCITY.value == "hardware_interface/VelocityJointInterface"
        assert HardwareInterface.EFFORT.value == "hardware_interface/EffortJointInterface"
        assert HardwareInterface.COMMAND_POSITION.value == "position"
        assert HardwareInterface.COMMAND_VELOCITY.value == "velocity"
        assert HardwareInterface.COMMAND_EFFORT.value == "effort"
