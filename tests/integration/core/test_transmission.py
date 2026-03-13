"""Comprehensive tests for all transmission types and configurations."""

import xml.etree.ElementTree as ET

from linkforge_core import URDFGenerator
from linkforge_core.models import (
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    Vector3,
)
from linkforge_core.parsers.urdf_parser import URDFParser


def test_simple_transmission():
    """Test SIMPLE transmission type."""
    robot = Robot(name="test_robot")
    robot.add_link(Link(name="base_link"))
    robot.add_link(Link(name="link1"))
    robot.add_joint(
        Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="link1",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=100, velocity=1.0),
        )
    )

    # Add SIMPLE transmission
    trans = Transmission(
        name="trans1",
        type="transmission_interface/SimpleTransmission",
        joints=[
            TransmissionJoint(
                name="joint1",
                hardware_interfaces=["effort"],
                mechanical_reduction=1.0,
                offset=0.0,
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1", hardware_interfaces=["effort"], mechanical_reduction=1.0
            )
        ],
    )
    robot.add_transmission(trans)

    # Export
    generator = URDFGenerator()
    urdf = generator.generate(robot)

    # Parse and verify
    root = ET.fromstring(urdf)
    trans_elem = root.find("transmission")
    assert trans_elem is not None
    assert trans_elem.get("name") == "trans1"
    assert trans_elem.findtext("type") == "transmission_interface/SimpleTransmission"

    # Verify joint
    joint_elem = trans_elem.find("joint")
    assert joint_elem.get("name") == "joint1"
    hw_iface = joint_elem.find("hardwareInterface")
    assert hw_iface.text == "effort"


def test_differential_transmission():
    """Test DIFFERENTIAL transmission type."""
    robot = Robot(name="test_robot")
    robot.add_link(Link(name="base_link"))
    robot.add_link(Link(name="left_wheel"))
    robot.add_link(Link(name="right_wheel"))
    robot.add_joint(
        Joint(
            name="left_wheel_joint",
            type=JointType.CONTINUOUS,
            parent="base_link",
            child="left_wheel",
            axis=Vector3(1.0, 0.0, 0.0),
        )
    )
    robot.add_joint(
        Joint(
            name="right_wheel_joint",
            type=JointType.CONTINUOUS,
            parent="base_link",
            child="right_wheel",
            axis=Vector3(1.0, 0.0, 0.0),
        )
    )

    # Add DIFFERENTIAL transmission
    trans = Transmission(
        name="diff_trans",
        type="transmission_interface/DifferentialTransmission",
        joints=[
            TransmissionJoint(name="left_wheel_joint", hardware_interfaces=["velocity"]),
            TransmissionJoint(name="right_wheel_joint", hardware_interfaces=["velocity"]),
        ],
        actuators=[
            TransmissionActuator(name="left_motor", hardware_interfaces=["velocity"]),
            TransmissionActuator(name="right_motor", hardware_interfaces=["velocity"]),
        ],
    )
    robot.add_transmission(trans)

    # Export
    generator = URDFGenerator()
    urdf = generator.generate(robot)

    # Parse and verify
    root = ET.fromstring(urdf)
    trans_elem = root.find("transmission")
    assert trans_elem is not None
    assert trans_elem.get("name") == "diff_trans"
    assert trans_elem.findtext("type") == "transmission_interface/DifferentialTransmission"

    # Verify 2 joints
    joints = trans_elem.findall("joint")
    assert len(joints) == 2
    assert joints[0].get("name") == "left_wheel_joint"
    assert joints[1].get("name") == "right_wheel_joint"

    # Verify 2 actuators
    actuators = trans_elem.findall("actuator")
    assert len(actuators) == 2


def test_four_bar_linkage_transmission():
    """Test FOUR_BAR_LINKAGE transmission type."""
    robot = Robot(name="test_robot")
    robot.add_link(Link(name="base_link"))
    robot.add_link(Link(name="link1"))
    robot.add_joint(
        Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="link1",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=100, velocity=1.0),
        )
    )

    # Add FOUR_BAR_LINKAGE transmission
    trans = Transmission(
        name="four_bar_trans",
        type="transmission_interface/FourBarLinkageTransmission",
        joints=[
            TransmissionJoint(
                name="joint1",
                hardware_interfaces=["position"],
                mechanical_reduction=2.0,  # Different reduction ratio
                offset=0.1,  # Non-zero offset
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1",
                hardware_interfaces=["position"],
                mechanical_reduction=2.0,
            )
        ],
    )
    robot.add_transmission(trans)

    # Export
    generator = URDFGenerator()
    urdf = generator.generate(robot)

    # Parse and verify
    root = ET.fromstring(urdf)
    trans_elem = root.find("transmission")
    assert trans_elem is not None
    assert trans_elem.get("name") == "four_bar_trans"
    assert trans_elem.findtext("type") == "transmission_interface/FourBarLinkageTransmission"

    # Verify mechanical reduction and offset
    joint_elem = trans_elem.find("joint")
    reduction_elem = joint_elem.find("mechanicalReduction")
    assert reduction_elem is not None
    assert float(reduction_elem.text) == 2.0

    offset_elem = joint_elem.find("offset")
    assert offset_elem is not None
    assert float(offset_elem.text) == 0.1


def test_custom_transmission():
    """Test CUSTOM transmission type."""
    robot = Robot(name="test_robot")
    robot.add_link(Link(name="base_link"))
    robot.add_link(Link(name="link1"))
    robot.add_joint(
        Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="link1",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=100, velocity=1.0),
        )
    )

    # Add CUSTOM transmission
    trans = Transmission(
        name="custom_trans",
        type="my_package/MyCustomTransmission",
        joints=[TransmissionJoint(name="joint1", hardware_interfaces=["effort"])],
        actuators=[TransmissionActuator(name="motor1", hardware_interfaces=["effort"])],
    )
    robot.add_transmission(trans)

    # Export
    generator = URDFGenerator()
    urdf = generator.generate(robot)

    # Parse and verify
    root = ET.fromstring(urdf)
    trans_elem = root.find("transmission")
    assert trans_elem is not None
    assert trans_elem.get("name") == "custom_trans"
    assert trans_elem.findtext("type") == "my_package/MyCustomTransmission"


def test_all_hardware_interfaces():
    """Test all hardware interface types (position, velocity, effort)."""
    robot = Robot(name="test_robot")
    robot.add_link(Link(name="base_link"))

    for i, interface in enumerate(["position", "velocity", "effort"], 1):
        robot.add_link(Link(name=f"link{i}"))
        robot.add_joint(
            Joint(
                name=f"joint{i}",
                type=JointType.REVOLUTE,
                parent="base_link",
                child=f"link{i}",
                axis=Vector3(1.0, 0.0, 0.0),
                limits=JointLimits(lower=-1.57, upper=1.57, effort=100, velocity=1.0),
            )
        )

        trans = Transmission(
            name=f"trans{i}",
            type="transmission_interface/SimpleTransmission",
            joints=[TransmissionJoint(name=f"joint{i}", hardware_interfaces=[interface])],
            actuators=[TransmissionActuator(name=f"motor{i}", hardware_interfaces=[interface])],
        )
        robot.add_transmission(trans)

    # Export
    generator = URDFGenerator()
    urdf = generator.generate(robot)

    # Parse and verify all interfaces
    root = ET.fromstring(urdf)
    transmissions = root.findall("transmission")
    assert len(transmissions) == 3

    for i, interface in enumerate(["position", "velocity", "effort"]):
        trans_elem = transmissions[i]
        joint_elem = trans_elem.find("joint")
        hw_iface = joint_elem.find("hardwareInterface")
        assert hw_iface.text == interface


def test_transmission_roundtrip():
    """Test transmission round-trip (import → export → import)."""
    original_urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"/>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
      </joint>
      <transmission name="trans1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint1">
          <hardwareInterface>effort</hardwareInterface>
          <mechanicalReduction>2.5</mechanicalReduction>
          <offset>0.2</offset>
        </joint>
        <actuator name="motor1">
          <mechanicalReduction>2.5</mechanicalReduction>
        </actuator>
      </transmission>
    </robot>
    """

    # Import
    robot1 = URDFParser().parse_string(original_urdf)
    assert len(robot1.transmissions) == 1
    trans1 = robot1.transmissions[0]
    assert trans1.name == "trans1"
    assert trans1.joints[0].mechanical_reduction == 2.5
    assert trans1.joints[0].offset == 0.2

    # Export
    generator = URDFGenerator()
    exported_urdf = generator.generate(robot1)

    # Re-import
    robot2 = URDFParser().parse_string(exported_urdf)
    assert len(robot2.transmissions) == 1
    trans2 = robot2.transmissions[0]
    assert trans2.name == "trans1"
    assert trans2.type == "transmission_interface/SimpleTransmission"
    assert trans2.joints[0].name == "joint1"
    assert trans2.joints[0].mechanical_reduction == 2.5
    assert trans2.joints[0].offset == 0.2
    assert trans2.actuators[0].name == "motor1"


def test_ros1_hardware_interface_normalization():
    """Test that ROS 1 hardware interface names are normalized to ROS 2."""
    urdf_with_ros1_interfaces = """<?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"/>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
      </joint>
      <transmission name="trans1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint1">
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor1">
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </actuator>
      </transmission>
    </robot>
    """

    # Import (should normalize)
    robot = URDFParser().parse_string(urdf_with_ros1_interfaces)
    trans = robot.transmissions[0]
    assert trans.joints[0].hardware_interfaces == ["effort"]  # Normalized!
    assert trans.actuators[0].hardware_interfaces == ["effort"]  # Normalized!

    # Export (should use short form)
    generator = URDFGenerator()
    exported_urdf = generator.generate(robot)

    root = ET.fromstring(exported_urdf)
    trans_elem = root.find("transmission")
    joint_elem = trans_elem.find("joint")
    hw_iface = joint_elem.find("hardwareInterface")
    assert hw_iface.text == "effort"  # Short form!
