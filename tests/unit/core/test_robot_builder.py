import pytest
from linkforge_core.composer.robot_builder import RobotBuilder, box, cylinder, mesh, sphere
from linkforge_core.exceptions import RobotModelError, RobotValidationError
from linkforge_core.models.joint import JointType
from linkforge_core.models.link import Link
from linkforge_core.models.robot import Robot
from linkforge_core.models.sensor import SensorType


class TestRobotBuilder:
    def test_builder_creation(self) -> None:
        """Test basic builder creation with name and existing robot."""
        # By name
        builder = RobotBuilder("my_robot")
        assert builder.robot.name == "my_robot"

        # By existing robot
        existing = Robot(name="existing")
        builder2 = RobotBuilder(robot=existing)
        assert builder2.robot.name == "existing"

    def test_material_registration(self) -> None:
        """Test global material registration."""
        builder = RobotBuilder("mat_test")
        builder.material("red", color=(1, 0, 0, 1))
        assert "red" in builder.robot.materials
        material = builder.robot.materials["red"]
        assert material.color is not None
        assert material.color.r == 1.0
        assert material.color.a == 1.0

    def test_link_chaining_and_root(self) -> None:
        """Test the hierarchical link chaining API."""
        robot = (
            RobotBuilder("chain_test")
            .link("base")
            .visual(box(0.1, 0.1, 0.1))
            .mass(1.0)
            .root()
            .link("arm1", parent="base")
            .revolute(axis=(0, 0, 1), limits=(-1, 1))
            .visual(cylinder(0.05, 0.5))
            .build()
        )

        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert robot.joint("base_to_arm1").type == JointType.REVOLUTE
        assert robot.graph.get_root_links() == ["base"]

    def test_child_chaining(self) -> None:
        """Test the .child() shortcut for kinematic chaining."""
        robot = (
            RobotBuilder("child_test")
            .link("base")
            .visual(box(0.1, 0.1, 0.1))
            .child("link1")
            .fixed()
            .visual(box(0.05, 0.05, 0.05))
            .build()
        )
        assert len(robot.links) == 2
        assert robot.get_joint("base_to_link1") is not None

    def test_collision_inference(self) -> None:
        """Test that collision() clones visual geometry if none provided."""
        builder = RobotBuilder("inf_test")
        builder.link("l1").visual(box(1, 2, 3), xyz=(0.1, 0.2, 0.3)).collision().root()

        link = builder.robot.link("l1")
        assert len(link.collisions) == 1

        from linkforge_core.models.geometry import Box

        geom = link.collisions[0].geometry
        assert isinstance(geom, Box)
        assert geom.size.x == 1.0
        assert link.collisions[0].origin.xyz.x == 0.1

    def test_collision_inference_error(self) -> None:
        """Test error when inferring collision without visuals."""
        builder = RobotBuilder("err_test")
        with pytest.raises(RobotValidationError, match="Cannot infer collision geometry"):
            builder.link("l1").collision()

    def test_automatic_inertia_calculation(self) -> None:
        """Test automatic inertia calculation from geometry."""
        # Box: 1x1x1, mass 12 -> Ixx = 1/12 * 12 * (1^2 + 1^2) = 2.0
        robot = RobotBuilder("phys_test").link("box").visual(box(1, 1, 1)).mass(12.0).root().build()
        link = robot.link("box")
        assert link.inertial is not None
        assert link.inertial.inertia.ixx == pytest.approx(2.0)
        assert link.inertial.inertia.izz == pytest.approx(2.0)

    def test_manual_inertia(self) -> None:
        """Test manual inertia tensor override."""
        robot = (
            RobotBuilder("manual_phys")
            .link("l1")
            .mass(1.0)
            .inertia(ixx=10, iyy=10, izz=10)
            .root()
            .build()
        )
        link = robot.link("l1")
        assert link.inertial is not None
        assert link.inertial.inertia.ixx == 10.0

    def test_joint_configurations(self) -> None:
        """Test different joint type configurations."""
        builder = RobotBuilder("joint_test")
        builder.link("base").root()

        # Continuous
        builder.link("l1", parent="base").continuous(axis=(0, 0, 1)).commit()
        assert builder.robot.joint("base_to_l1").type == JointType.CONTINUOUS

        # Revolute with limits
        builder.link("l2", parent="base").revolute(axis=(1, 0, 0), limits=(-0.5, 0.5)).commit()
        j2 = builder.robot.joint("base_to_l2")
        assert j2.type == JointType.REVOLUTE
        assert j2.limits is not None
        assert j2.limits.lower == -0.5

    def test_transmission_registration(self) -> None:
        """Test that transmissions are correctly registered."""
        robot = (
            RobotBuilder("trans_test")
            .link("base")
            .root()
            .link("arm", parent="base")
            .revolute(axis=(0, 0, 1), limits=(-1, 1))
            .transmission(reduction=100.0, interface="effort", name="my_trans")
            .build()
        )
        assert len(robot.transmissions) == 1
        trans = robot.transmissions[0]
        assert trans.name == "my_trans"
        assert trans.joints[0].mechanical_reduction == 100.0
        assert trans.joints[0].hardware_interfaces == ["effort"]

    def test_srdf_helpers(self) -> None:
        """Test SRDF helpers in RobotBuilder."""
        builder = RobotBuilder("srdf_test")
        builder.link("l1").root().link("l2", parent="l1").commit()

        builder.group("my_group", links=["l1", "l2"])
        builder.disable_collisions("l1", "l2", reason="Adjacent")

        assert len(builder.robot.semantic.groups) == 1
        assert len(builder.robot.semantic.disabled_collisions) == 1

    def test_root_validation(self) -> None:
        """Test that root() fails if there is a parent."""
        builder = RobotBuilder("root_err")
        with pytest.raises(RobotValidationError, match="has a parent"):
            builder.link("l1", parent="world").root()

    def test_attach_merge(self) -> None:
        """Test the attach (merge) functionality."""
        sub = Robot(name="sub")
        sub.add_link(Link(name="sub_root"))

        builder = RobotBuilder("main")
        builder.link("base").root()
        builder.attach(sub, at_link="base", joint_name="conn", prefix="p_")

        assert builder.robot.get_link("p_sub_root") is not None
        assert builder.robot.get_joint("p_conn") is not None

    def test_geometry_helpers(self) -> None:
        """Test the geometry helper functions."""
        from linkforge_core.models.geometry import Box, Cylinder, Mesh, Sphere

        b = box(1, 2, 3)
        assert isinstance(b, Box)
        assert b.size.x == 1

        c = cylinder(0.1, 0.5)
        assert isinstance(c, Cylinder)
        assert c.radius == 0.1

        s = sphere(0.5)
        assert isinstance(s, Sphere)
        assert s.radius == 0.5

        m = mesh("file://test.stl", scale=(2, 2, 2))
        assert isinstance(m, Mesh)
        assert m.resource == "file://test.stl"
        assert m.scale.x == 2.0

    def test_builder_errors(self) -> None:
        """Test builder initialization errors."""
        with pytest.raises(RobotModelError, match="Either name or robot must be provided"):
            RobotBuilder()

    def test_export_shortcuts(self) -> None:
        """Test URDF/SRDF export shortcuts."""
        builder = RobotBuilder("export_test")
        builder.link("base").root()
        assert '<robot name="export_test"' in builder.export_urdf()
        assert '<robot name="export_test"' in builder.export_srdf()

    def test_explicit_transforms_and_origins(self) -> None:
        """Test explicit transforms in visual/collision and joint origin."""
        builder = RobotBuilder("trans_test")
        builder.link("base").root()

        # Test at_origin and explicit collision transform
        builder.link("l1", parent="base").at_origin(xyz=(1, 0, 0), rpy=(0, 0, 1.57)).visual(
            box(0.1, 0.1, 0.1)
        ).collision(box(0.1, 0.1, 0.1), xyz=(0, 0, 0.1)).commit()

        joint = builder.robot.joint("base_to_l1")
        assert joint.origin.xyz.x == 1.0
        assert joint.origin.rpy.z == 1.57

        link = builder.robot.link("l1")
        assert link.collisions[0].origin.xyz.z == 0.1

    def test_physics_fallback_and_manual_origin(self) -> None:
        """Test fallback to zero inertia and manual inertial origin."""
        # No geometry -> Zero inertia (uses epsilon 1e-6 in IR)
        robot = RobotBuilder("fallback").link("l1").mass(1.0).root().build()
        link = robot.link("l1")
        assert link.inertial is not None
        assert link.inertial.inertia.ixx == 1e-6

        # Manual inertial origin
        robot2 = (
            RobotBuilder("origin")
            .link("l1")
            .visual(box(1, 1, 1))
            .mass(1.0, origin_xyz=(0, 0, 10))
            .root()
            .build()
        )
        link2 = robot2.link("l1")
        assert link2.inertial is not None
        assert link2.inertial.origin.xyz.z == 10.0

    def test_visual_with_material_object(self) -> None:
        """Test passing a Material object to visual()."""
        from linkforge_core.models.material import Color, Material

        mat = Material(name="blue", color=Color(0, 0, 1, 1))
        builder = RobotBuilder("mat_obj")
        builder.link("l1").visual(box(1, 1, 1), material=mat).root()
        vis = builder.robot.link("l1").visuals[0]
        assert vis.material is not None
        assert vis.material.name == "blue"

    def test_collision_only_physics(self) -> None:
        """Test inertia calculation when only collision geometry exists."""
        robot = (
            RobotBuilder("coll_phys").link("l1").collision(box(1, 1, 1)).mass(12.0).root().build()
        )
        link = robot.link("l1")
        assert link.inertial is not None
        assert link.inertial.inertia.ixx == pytest.approx(2.0)

    def test_inertia_priority_collision(self) -> None:
        """Verify inertia calculation prioritizes collision over visual geometry."""
        robot = (
            RobotBuilder("priority_test")
            .link("l1")
            .visual(box(1, 1, 1))
            .collision(box(0.1, 0.1, 0.1))
            .mass(12.0)
            .root()
            .build()
        )
        link = robot.link("l1")
        assert link.inertial is not None
        # Expect 0.02 (collision-based) rather than 2.0 (visual-based)
        assert link.inertial.inertia.ixx == pytest.approx(0.02)

    def test_explicit_joint_naming(self) -> None:
        """Test providing explicit names for all joint types."""
        builder = RobotBuilder("name_test")
        builder.link("base").root()

        builder.link("l1", parent="base").fixed(name="custom_fixed").commit()
        builder.link("l2", parent="base").revolute(
            axis=(0, 0, 1), limits=(0, 1), name="custom_rev"
        ).commit()
        builder.link("l3", parent="base").continuous(axis=(0, 0, 1), name="custom_cont").commit()

        assert builder.robot.get_joint("custom_fixed") is not None
        assert builder.robot.get_joint("custom_rev") is not None
        assert builder.robot.get_joint("custom_cont") is not None

    def test_partial_transforms(self) -> None:
        """Test providing only rpy in collision or mass origin."""
        # Collision with only rpy
        builder = RobotBuilder("partial_trans")
        builder.link("l1").visual(box(1, 1, 1)).collision(box(1, 1, 1), rpy=(0, 1.57, 0)).commit()
        assert builder.robot.link("l1").collisions[0].origin.rpy.y == 1.57

        # Mass with only rpy
        builder2 = RobotBuilder("partial_mass")
        builder2.link("l1").visual(box(1, 1, 1)).mass(1.0, origin_rpy=(0, 0, 1.57)).commit()
        link2 = builder2.robot.link("l1")
        assert link2.inertial is not None
        assert link2.inertial.origin.rpy.z == 1.57

    def test_full_collision_transform(self) -> None:
        """Test providing both xyz and rpy in collision."""
        builder = RobotBuilder("full_trans")
        builder.link("l1").visual(box(1, 1, 1)).collision(
            box(1, 1, 1), xyz=(1, 1, 1), rpy=(1, 1, 1)
        ).commit()
        coll = builder.robot.link("l1").collisions[0]
        assert coll.origin.xyz.x == 1.0
        assert coll.origin.rpy.x == 1.0

    def test_direct_inertia_in_mass(self) -> None:
        """Test passing an InertiaTensor directly to the mass() method."""
        from linkforge_core.models.link import InertiaTensor

        tensor = InertiaTensor(ixx=5, iyy=5, izz=5, ixy=0, ixz=0, iyz=0)
        builder = RobotBuilder("direct_inertia")
        builder.link("l1").mass(1.0, inertia=tensor).root()
        link = builder.robot.link("l1")
        assert link.inertial is not None
        assert link.inertial.inertia.ixx == 5.0

    def test_sensors_and_ros2_control(self) -> None:
        """Test adding sensors and ros2_control to links."""
        builder = RobotBuilder("rob")
        from linkforge_core.models.sensor import Sensor, SensorType

        custom_sensor = Sensor(name="custom", type=SensorType.CONTACT, link_name="base")

        builder.ros2_control("test_sys", "test/Plugin")
        builder.link("base").sensor(custom_sensor).root()

        builder.link("camera_link", parent="base").fixed().camera(
            "my_cam", fov=1.5, width=1280, height=720, xyz=(0, 0, 0.1)
        ).ros2_control(
            command_interfaces=["position"], state_interfaces=["position", "velocity"]
        ).commit()

        builder.link("imu_link", parent="base").fixed().imu("my_imu", update_rate=200.0).gps(
            "my_gps"
        ).commit()

        robot = builder.build()
        assert len(robot.sensors) == 4
        assert any(s.name == "my_imu" for s in robot.sensors)
        assert any(s.name == "my_gps" for s in robot.sensors)
        assert any(s.name == "custom" for s in robot.sensors)
        cam = next(s for s in robot.sensors if s.name == "my_cam")
        assert cam.type == SensorType.CAMERA
        assert cam.camera_info is not None
        assert cam.camera_info.width == 1280

        assert len(robot._ros2_controls) == 1
        assert robot._ros2_controls[0].joints[0].name == "base_to_camera_link"
        assert "position" in robot._ros2_controls[0].joints[0].command_interfaces

    def test_advanced_srdf_helpers(self) -> None:
        """Test SRDF helpers like group_state and end_effector."""
        builder = RobotBuilder("srdf")
        builder.link("base").root().link("tool", parent="base").commit()

        builder.group("arm", links=["base", "tool"])
        builder.group_state("home", "arm", {"base_to_tool": 0.0})
        builder.end_effector("gripper", "arm", "tool")
        builder.virtual_joint("world_joint", "base", "world", "fixed")

        robot = builder.build()
        assert len(robot.semantic.group_states) == 1
        assert robot.semantic.group_states[0].name == "home"
        assert len(robot.semantic.end_effectors) == 1
        assert robot.semantic.end_effectors[0].name == "gripper"
        assert len(robot.semantic.virtual_joints) == 1
        assert robot.semantic.virtual_joints[0].name == "world_joint"

    def test_lidar_and_multi_control(self) -> None:
        """Test lidar sensor and adding joints to an existing ros2_control system."""
        builder = RobotBuilder("multi")
        builder.ros2_control("multi_sys", "hardware/Plugin")
        builder.link("base").root()
        builder.link("l1", parent="base").revolute(axis=(0, 0, 1), limits=(0, 1)).ros2_control(
            ["pos"], ["pos"]
        ).commit()
        # Second joint should use the existing system
        builder.link("l2", parent="l1").revolute(axis=(0, 0, 1), limits=(0, 1)).ros2_control(
            ["pos"], ["pos"]
        ).lidar("laser").commit()

        robot = builder.build()
        assert len(robot._ros2_controls) == 1
        assert len(robot._ros2_controls[0].joints) == 2
        assert any(s.type == SensorType.LIDAR for s in robot.sensors)

    def test_double_commit_guard(self) -> None:
        """Test that calling _commit twice doesn't cause issues."""
        builder = RobotBuilder("guard")
        lb = builder.link("base")
        lb.root()
        # Internal _commit is already called by root()
        # Calling it again manually should be a no-op due to self._committed
        lb._commit()
        assert len(builder.robot.links) == 1

    def test_all_joint_types_and_origins(self) -> None:
        """Test all joint types (including continuous and prismatic) and their origins."""
        builder = RobotBuilder("all_joints")
        builder.link("base").root()

        # Continuous
        builder.link("c", parent="base").continuous(axis=(0, 0, 1), xyz=(1, 0, 0)).commit()
        # Prismatic with name and both xyz/rpy
        builder.link("p", parent="c").prismatic(
            axis=(1, 0, 0), limits=(0, 0.5), name="c_to_p", xyz=(0, 0, 0), rpy=(0.1, 0, 0)
        ).commit()
        # Prismatic without name or origin (False branches)
        builder.link("p2", parent="p").prismatic(axis=(0, 1, 0), limits=(0, 0.1)).commit()
        # Fixed with origin
        builder.link("f", parent="p").fixed(xyz=(0, 0, 1), rpy=(0, 0, 1.57)).commit()
        # Revolute with origin
        builder.link("r", parent="f").revolute(
            axis=(0, 1, 0), limits=(-1, 1), xyz=(0.1, 0.1, 0.1)
        ).commit()

        robot = builder.build()
        assert len(robot.joints) == 5
        assert robot.joint("base_to_c").type == JointType.CONTINUOUS
        assert robot.joint("c_to_p").type == JointType.PRISMATIC
        assert robot.joint("p_to_f").origin.xyz.z == 1.0
        assert robot.joint("p_to_f").origin.rpy.z == 1.57
