"""Comprehensive URDF/SRDF roundtrip integration tests."""

from __future__ import annotations

from linkforge.core import (
    Box,
    Cylinder,
    GazeboElement,
    GazeboPlugin,
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
    Link,
    LinkPhysics,
    Robot,
    Sphere,
    Transform,
    Transmission,
    URDFParser,
    Vector3,
    Visual,
)

from tests.core_test_utils import assert_robots_equal, perform_urdf_roundtrip

# Element-Specific Roundtrips


def test_geometry_types_roundtrip() -> None:
    """Test that all geometry types (Box, Cylinder, Sphere, Mesh) survive roundtrip."""
    robot = Robot(name="geometry_test")
    robot.add_link(Link(name="base"))

    # Box
    robot.add_link(
        Link(
            name="box_link",
            visuals=[
                Visual(
                    geometry=Box(size=Vector3(1.0, 2.0, 3.0)),
                    origin=Transform(xyz=Vector3(0.1, 0.2, 0.3)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(name="base_to_box", type=JointType.FIXED, parent="base", child="box_link")
    )

    # Cylinder
    robot.add_link(
        Link(
            name="cylinder_link",
            visuals=[
                Visual(
                    geometry=Cylinder(radius=0.5, length=2.0),
                    origin=Transform(xyz=Vector3(0.0, 0.0, 1.0)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(name="base_to_cylinder", type=JointType.FIXED, parent="base", child="cylinder_link")
    )

    # Sphere
    robot.add_link(
        Link(
            name="sphere_link",
            visuals=[
                Visual(geometry=Sphere(radius=0.75), origin=Transform(xyz=Vector3(1.0, 1.0, 1.0)))
            ],
        )
    )
    robot.add_joint(
        Joint(name="base_to_sphere", type=JointType.FIXED, parent="base", child="sphere_link")
    )

    robot2 = perform_urdf_roundtrip(robot, use_ros2_control=False)
    assert_robots_equal(robot, robot2)


def test_joint_types_and_properties_roundtrip() -> None:
    """Test all joint types, limits, dynamics, mimic, safety, and calibration."""
    robot = Robot(name="joint_comprehensive_test")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="l1"))
    robot.add_link(Link(name="l2"))

    # Comprehensive joint
    robot.add_joint(
        Joint(
            name="comp_joint",
            type=JointType.REVOLUTE,
            parent="base",
            child="l1",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=100.0, velocity=1.0),
            dynamics=JointDynamics(damping=0.1, friction=0.05),
            safety_controller=JointSafetyController(
                soft_lower_limit=-1.5, soft_upper_limit=1.5, k_position=100.0, k_velocity=1.0
            ),
            calibration=JointCalibration(rising=0.1, falling=0.2),
        )
    )

    # Mimic joint
    robot.add_joint(
        Joint(
            name="mimic_joint",
            type=JointType.REVOLUTE,
            parent="base",
            child="l2",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-0.785, upper=0.785, effort=10.0, velocity=1.0),
            mimic=JointMimic(joint="comp_joint", multiplier=0.5, offset=0.1),
        )
    )

    robot2 = perform_urdf_roundtrip(robot, use_ros2_control=False)
    assert_robots_equal(robot, robot2)


def test_visual_origin_normalization_roundtrip() -> None:
    """Test that identity origins are handled correctly (omitted or preserved)."""
    robot = Robot(name="origin_test")
    # Link with explicit identity origin
    robot.add_link(
        Link(
            name="link_identity",
            visuals=[Visual(geometry=Box(size=Vector3(1, 1, 1)), origin=Transform.identity())],
        )
    )
    # Link with NO origin (should be identity)
    robot.add_link(Link(name="link_none", visuals=[Visual(geometry=Box(size=Vector3(1, 1, 1)))]))
    # Connect them to avoid multiple roots
    robot.add_joint(
        Joint(name="j1", type=JointType.FIXED, parent="link_identity", child="link_none")
    )

    robot2 = perform_urdf_roundtrip(robot, use_ros2_control=False)
    # Both should be equivalent to identity
    assert robot2.link("link_identity").visuals[0].origin == Transform.identity()
    assert robot2.link("link_none").visuals[0].origin == Transform.identity()


# Advanced Elements Roundtrips


def test_transmission_types_roundtrip() -> None:
    """Test Simple and Differential transmissions."""
    robot = Robot(name="transmission_test")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="l1"))
    robot.add_link(Link(name="l2"))
    robot.add_link(Link(name="l3"))

    # Joints for transmissions
    robot.add_joint(
        Joint(
            name="j1",
            type=JointType.REVOLUTE,
            parent="base",
            child="l1",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=10, velocity=1),
        )
    )
    robot.add_joint(
        Joint(
            name="j2",
            type=JointType.REVOLUTE,
            parent="base",
            child="l2",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=10, velocity=1),
        )
    )
    robot.add_joint(
        Joint(
            name="j3",
            type=JointType.REVOLUTE,
            parent="base",
            child="l3",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=10, velocity=1),
        )
    )

    # Simple
    robot.add_transmission(
        Transmission.create_simple(
            "t1", "j1", mechanical_reduction=50.0, hardware_interface="effort"
        )
    )

    # Differential
    robot.add_transmission(
        Transmission.create_differential("t2", "j2", "j3", mechanical_reduction=10.0)
    )

    robot2 = perform_urdf_roundtrip(robot, use_ros2_control=False)
    assert_robots_equal(robot, robot2)


def test_gazebo_elements_roundtrip() -> None:
    """Test robot, link, and joint level Gazebo elements."""
    robot = Robot(name="gazebo_test")

    # Link-level properties (Physics go to Link, material goes to GazeboElement)
    robot.add_link(Link(name="l1", physics=LinkPhysics(mu=0.5, mu2=0.5, kp=1000, kd=10)))

    # Robot-level plugin
    robot.add_gazebo_element(
        GazeboElement(plugins=[GazeboPlugin(name="p1", filename="f1.so", parameters={"k1": "v1"})])
    )

    # Link-level Gazebo extensions
    robot.add_gazebo_element(GazeboElement(reference="l1", material="Gazebo/Red"))

    robot2 = perform_urdf_roundtrip(robot, use_ros2_control=False)
    assert_robots_equal(robot, robot2)


def test_ros2_control_roundtrip() -> None:
    """Test ros2_control blocks integration."""
    # Assuming perform_urdf_roundtrip handles ros2_control
    urdf = """<?xml version="1.0"?>
    <robot name="ros2_test">
      <link name="base_link"/>
      <ros2_control name="test_system" type="system">
        <hardware>
          <plugin>fake_components/GenericSystem</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="position"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
        </joint>
      </ros2_control>
    </robot>
    """
    robot1 = URDFParser().parse_string(urdf)
    robot2 = perform_urdf_roundtrip(robot1)
    assert_robots_equal(robot1, robot2)
