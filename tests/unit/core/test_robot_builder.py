import pytest
from linkforge_core.composer import RobotBuilder, box, cylinder, mesh, sphere
from linkforge_core.exceptions import RobotModelError, RobotValidationError
from linkforge_core.models.geometry import GeometryType
from linkforge_core.models.joint import Joint, JointType
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

        # Error: neither
        with pytest.raises(RobotModelError, match="Either name or robot"):
            RobotBuilder()

    def test_build_no_validation(self) -> None:
        """Test building without validation (should pass even if invalid)."""
        builder = RobotBuilder("invalid")
        # No links, but validate=False
        builder.build(validate=False)

    def test_material_registration(self) -> None:
        """Test global material registration."""
        builder = RobotBuilder("mat_test")
        builder.material("red", color=(1, 0, 0, 1))
        assert "red" in builder.robot.materials
        material = builder.robot.materials["red"]
        assert material.color is not None
        assert material.color.r == 1.0
        assert material.color.a == 1.0

        # Attach to visual to hit existing material branch
        builder.link("l1").visual(box(1, 1, 1), material="red").root()
        link = builder.robot.link("l1")
        assert len(link.visuals) > 0
        visual = link.visuals[0]
        from linkforge_core.models.material import Material

        assert isinstance(visual.material, Material)
        assert visual.material.name == "red"

    def test_link_chaining_and_root(self) -> None:
        """Test the basic root link and child chaining logic."""
        builder = RobotBuilder("chain_test")

        # Define root
        builder.link("base").root()
        assert builder.robot.has_link("base")
        root = builder.robot.root_link
        assert root is not None
        assert root.name == "base"

        # Add child
        builder.link("link1", parent="base").revolute(axis=(0, 0, 1), limits=(-1, 1)).commit()
        assert builder.robot.has_link("link1")
        assert builder.robot.has_joint("base_to_link1")

        # Add another child using parent param
        builder.link("link2", parent="link1").fixed().commit()
        assert builder.robot.has_link("link2")
        assert builder.robot.has_joint("link1_to_link2")

    def test_child_chaining(self) -> None:
        """Test deep chaining with .child()."""
        builder = RobotBuilder("deep_chain")
        builder.link("base_child").root()

        # Using .child() on a fresh link works because the parent was just staged
        (builder.link("l1_child", parent="base_child").fixed().child("l2_child").fixed().commit())

        assert builder.robot.has_link("l2_child")
        assert len(builder.robot.links) == 3

    def test_collision_inference(self) -> None:
        """Test that collision() can infer geometry from visual."""
        builder = RobotBuilder("coll_test")
        builder.link("l1_coll").visual(box(1, 1, 1)).collision().root()

        link = builder.robot.link("l1_coll")
        assert len(link.visuals) == 1
        assert len(link.collisions) == 1
        assert link.collisions[0].geometry.type == GeometryType.BOX

    def test_collision_inference_error(self) -> None:
        """Test collision inference error when no visual exists."""
        builder = RobotBuilder("coll_err")
        # Match against actual error message "Cannot infer collision geometry"
        with pytest.raises(RobotValidationError, match="Cannot infer collision"):
            builder.link("l1_no_vis").collision()

    def test_automatic_inertia_calculation(self) -> None:
        """Test that inertia is calculated from collision geometry."""
        builder = RobotBuilder("inertia_test")
        builder.link("l1_inertia").visual(box(1, 1, 1)).collision().mass(10.0).root()

        link = builder.robot.link("l1_inertia")
        assert link.inertial is not None
        assert link.mass == 10.0
        assert link.inertia.ixx > 0

    def test_manual_inertia(self) -> None:
        """Test manual inertia entry."""
        builder = RobotBuilder("manual_inertia")
        builder.link("l1_manual").mass(10.0).inertia(ixx=1, iyy=1, izz=1).root()

        link = builder.robot.link("l1_manual")
        assert link.inertia.ixx == 1.0

    def test_joint_configurations(self) -> None:
        """Test different joint types and origins."""
        builder = RobotBuilder("joints")
        builder.link("base_joint").root()

        # Revolute
        builder.link("l1_joint", parent="base_joint").revolute(
            axis=(1, 0, 0), limits=(0, 3.14), xyz=(1, 0, 0)
        ).commit()
        # Continuous
        builder.link("l2_joint", parent="l1_joint").continuous(
            axis=(0, 1, 0), rpy=(1.57, 0, 0)
        ).commit()
        # Prismatic
        builder.link("l3_joint", parent="l2_joint").prismatic(
            axis=(0, 0, 1), limits=(0, 0.5)
        ).commit()
        # Fixed
        builder.link("l4_joint", parent="l3_joint").fixed(xyz=(0, 0, 1)).commit()

        assert builder.robot.joint("base_joint_to_l1_joint").type == JointType.REVOLUTE

    def test_transmission_registration(self) -> None:
        """Test that transmission() correctly registers on the joint."""
        builder = RobotBuilder("trans")
        builder.link("base_trans").root()
        builder.link("l1_trans", parent="base_trans").revolute(
            axis=(0, 0, 1), limits=(0, 1)
        ).transmission(reduction=50.0).commit()

        assert len(builder.robot.transmissions) == 1
        assert builder.robot.transmissions[0].joints[0].mechanical_reduction == 50.0

    def test_srdf_helpers(self) -> None:
        """Test semantic (SRDF) builder methods."""
        builder = RobotBuilder("srdf")
        builder.link("base_srdf").root()
        builder.link("l1_srdf", parent="base_srdf").fixed().commit()

        builder.semantic.group("arm", links=["base_srdf", "l1_srdf"])
        builder.semantic.group_state("home", "arm", {"base_srdf_to_l1_srdf": 0.0})
        builder.semantic.end_effector("tool", group="arm", parent_link="l1_srdf")
        builder.semantic.passive_joint("base_srdf_to_l1_srdf")
        builder.semantic.virtual_joint("vj", child_link="base_srdf")
        builder.semantic.disable_collisions("base_srdf", "l1_srdf")

        s = builder.robot.semantic
        assert len(s.groups) == 1
        assert len(s.group_states) == 1
        assert len(s.end_effectors) == 1
        assert len(s.passive_joints) == 1
        assert len(s.virtual_joints) == 1
        assert len(s.disabled_collisions) == 1

    def test_root_validation(self) -> None:
        """Test that build() fails if no root is defined."""
        builder = RobotBuilder("no_root")
        # Test error when build is called on empty robot
        with pytest.raises(RobotValidationError, match="No root link found"):
            builder.build()

        # Robot with disconnected links (multiple roots)
        builder2 = RobotBuilder("multiroot")
        builder2.link("l1_m").root()
        builder2.link("l2_m").root()
        with pytest.raises(RobotValidationError, match="Multiple root links"):
            builder2.build(validate=True)

    def test_invalid_root_call(self) -> None:
        """Test that calling root() on a link with parent fails."""
        builder = RobotBuilder("invalid_root")
        builder.link("base_ir").root()
        with pytest.raises(RobotValidationError, match="cannot be root"):
            builder.link("child_ir", parent="base_ir").root()

    def test_inertia_fallbacks(self) -> None:
        """Test inertia calculation fallbacks."""
        # Visual-only fallback
        b1 = RobotBuilder("v_fallback_f")
        b1.link("l_vf_f").visual(box(1, 1, 1)).mass(1.0).root()
        assert b1.robot.link("l_vf_f").inertia.ixx > 0

        # No geometry fallback
        b2 = RobotBuilder("no_fallback_f")
        b2.link("l_nf_f").mass(1.0).root()
        assert b2.robot.link("l_nf_f").inertia.ixx == 1e-6

    def test_attach_merge(self) -> None:
        """Test attaching another builder/robot."""
        b1 = RobotBuilder("main")
        b1.link("base_attach").root()

        b2 = RobotBuilder("sub")
        b2.link("sub_base").root()
        b2.link("tool", parent="sub_base").fixed().commit()

        # Attach with empty prefix
        b1.attach(b2, at_link="base_attach", joint_name="attachment", xyz=(0, 0, 1))

        assert b1.robot.has_link("sub_base")
        assert b1.robot.has_link("tool")
        assert b1.robot.has_joint("attachment")

    def test_semantic_subgroups(self) -> None:
        """Test defining planning groups with subgroups."""
        builder = RobotBuilder("semantic_bot")
        builder.link("base").root()

        builder.semantic.group("arm", links=["base"])
        builder.semantic.group("manipulator", subgroups=["arm"])

        robot = builder.build()
        groups = {g.name: g for g in robot.semantic.groups}
        assert "arm" in groups
        assert "manipulator" in groups
        assert "arm" in groups["manipulator"].subgroups

    def test_geometry_helpers(self) -> None:
        """Test box, cylinder, etc. factory functions."""
        b = box(1, 2, 3)
        assert b.type == GeometryType.BOX
        assert b.size.x == 1

        c = cylinder(0.5, 1.0)
        assert c.type == GeometryType.CYLINDER

        s = sphere(0.2)
        assert s.type == GeometryType.SPHERE

        m = mesh("package://test.stl")
        assert m.type == GeometryType.MESH

    def test_builder_errors(self) -> None:
        """Test common builder error states."""
        builder = RobotBuilder("err_robot")

        # Invalid link lookup (using link with invalid parent)
        # It should fail during commit() because parent "non_existent" is missing
        with pytest.raises(RobotValidationError, match="Parent link"):
            builder.link("l_err", parent="non_existent").fixed().commit()

    def test_export_shortcuts(self) -> None:
        """Test export_urdf and export_srdf shortcuts."""
        builder = RobotBuilder("export_robot")
        builder.link("base_exp").root()

        urdf = builder.export_urdf()
        assert '<robot name="export_robot"' in urdf

        srdf = builder.export_srdf()
        assert '<robot name="export_robot"' in srdf

    def test_explicit_transforms_and_origins(self) -> None:
        """Test at_origin() and coordinate frame consistency."""
        builder = RobotBuilder("transforms")
        builder.link("base_tr").root()

        builder.link("l1_tr", parent="base_tr").at_origin(
            xyz=(1, 2, 3), rpy=(0, 0.1, 0)
        ).fixed().commit()
        j = builder.robot.joint("base_tr_to_l1_tr")
        assert j.origin.xyz.x == 1.0
        assert j.origin.rpy.y == 0.1

    def test_physics_fallback_and_manual_origin(self) -> None:
        """Test center-of-mass origin setting."""
        builder = RobotBuilder("physics")
        builder.link("l1_ph").mass(5.0, origin_xyz=(0, 0, 0.5)).root()

        link = builder.robot.link("l1_ph")
        assert link.inertial_origin.xyz.z == 0.5

    def test_visual_with_material_object(self) -> None:
        """Test passing a Material object to visual()."""
        builder = RobotBuilder("mat_obj")
        from linkforge_core.models.material import Color, Material

        mat = Material(name="custom", color=Color(0, 1, 0, 1))

        builder.link("l1_mat").visual(box(1, 1, 1), material=mat).root()
        link = builder.robot.link("l1_mat")
        assert len(link.visuals) > 0
        visual = link.visuals[0]
        from linkforge_core.models.material import Material

        assert isinstance(visual.material, Material)
        assert visual.material.name == "custom"

    def test_collision_only_physics(self) -> None:
        """Test that physics can be calculated even if only collision exists."""
        builder = RobotBuilder("coll_physics")
        builder.link("l1_cp").collision(box(1, 1, 1)).mass(1.0).root()
        assert builder.robot.link("l1_cp").inertial is not None

    def test_inertia_priority_collision(self) -> None:
        """Test that inertia calculation prefers collision over visual if both exist."""
        builder = RobotBuilder("priority")
        # Visual is a small box, Collision is a large box
        builder.link("l1_pr").visual(box(0.1, 0.1, 0.1)).collision(box(10, 10, 10)).mass(1.0).root()

        # High inertia means it used the large box
        assert builder.robot.link("l1_pr").inertia.ixx > 1.0

    def test_explicit_joint_naming(self) -> None:
        """Test providing a custom name to a joint method."""
        builder = RobotBuilder("custom_joint")
        builder.link("base_cj").root()
        builder.link("l1_cj", parent="base_cj").fixed(name="my_special_joint").commit()
        assert builder.robot.has_joint("my_special_joint")

    def test_partial_transforms(self) -> None:
        """Test providing only xyz or only rpy."""
        builder = RobotBuilder("partial")
        builder.link("base_pt").root()
        builder.link("l1_pt", parent="base_pt").fixed(xyz=(1, 0, 0)).commit()
        assert builder.robot.joint("base_pt_to_l1_pt").origin.xyz.x == 1.0
        assert builder.robot.joint("base_pt_to_l1_pt").origin.rpy.x == 0.0

    def test_full_collision_transform(self) -> None:
        """Test explicit transform for collision geometry."""
        builder = RobotBuilder("coll_trans")
        builder.link("l1_ct").collision(box(1, 1, 1), xyz=(0, 0, 1), rpy=(1.57, 0, 0)).root()
        c = builder.robot.link("l1_ct").collisions[0]
        assert c.origin.xyz.z == 1.0
        assert c.origin.rpy.x == 1.57

    def test_direct_inertia_in_mass(self) -> None:
        """Test passing InertiaTensor directly to mass()."""
        from linkforge_core.models.link import InertiaTensor

        builder = RobotBuilder("direct_inertia")
        it = InertiaTensor(ixx=2, iyy=2, izz=2, ixy=0, ixz=0, iyz=0)
        builder.link("l1_di").mass(1.0, inertia=it).root()
        assert builder.robot.link("l1_di").inertia.iyy == 2.0

    def test_sensors_and_ros2_control(self) -> None:
        """Test adding sensors and ros2_control to links."""
        builder = RobotBuilder("sensors")
        builder.ros2_control("sys", "plugin")
        builder.link("base_sens").root()

        (
            builder.link("l1_sens", parent="base_sens")
            .revolute(axis=(0, 0, 1), limits=(-1, 1))
            .ros2_control(command_interfaces=["pos"], state_interfaces=["pos"])
            .camera("cam")
            .commit()
        )

        assert len(builder.robot.sensors) == 1
        assert len(builder.robot.ros2_controls[0].joints) == 1

    def test_advanced_srdf_helpers(self) -> None:
        """Test more complex SRDF helper scenarios."""
        builder = RobotBuilder("adv_srdf")
        builder.link("base_as").root()
        builder.link("l1_as", parent="base_as").fixed().commit()

        # Test disable_all_collisions through Robot model indirectly
        builder.robot.disable_all_collisions(["base_as", "l1_as"])
        assert len(builder.robot.semantic.disabled_collisions) == 1

    def test_all_sensor_types(self) -> None:
        """Test all sensor helper methods (IMU, GPS, FT, Contact)."""
        builder = RobotBuilder("all_sensors")
        builder.link("base_ast").root()
        (
            builder.link("l1_ast", parent="base_ast")
            .fixed()
            .imu("my_imu")
            .gps("my_gps")
            .force_torque("my_ft")
            .contact("my_contact", collision="l1_collision")
            .commit()
        )
        assert len(builder.robot.sensors) == 4
        types = [s.type for s in builder.robot.sensors]
        assert SensorType.IMU in types
        assert SensorType.GPS in types
        assert SensorType.FORCE_TORQUE in types
        assert SensorType.CONTACT in types

        # Check sub-info existence
        ft_sensor = next(s for s in builder.robot.sensors if s.type == SensorType.FORCE_TORQUE)
        assert ft_sensor.force_torque_info is not None
        contact_sensor = next(s for s in builder.robot.sensors if s.type == SensorType.CONTACT)
        assert contact_sensor.contact_info is not None

    def test_lidar_and_multi_control(self) -> None:
        """Test lidar sensor and multiple control interfaces."""
        builder = RobotBuilder("multi")
        builder.ros2_control("sys", "plugin")
        builder.link("base_l").root()
        builder.link("l1_l", parent="base_l").revolute(axis=(0, 0, 1), limits=(-1, 1)).lidar(
            "scan"
        ).commit()

        assert builder.robot.sensors[0].type == SensorType.LIDAR

    def test_kinematic_validation_errors(self) -> None:
        """Test that build() correctly identifies complex kinematic errors."""
        # Cycle detection (no root)
        builder = RobotBuilder("cycle")
        builder.robot.add_link(Link("l1_kv"))
        builder.robot.add_link(Link("l2_kv"))
        builder.robot.add_joint(Joint("j1_kv", JointType.FIXED, "l1_kv", "l2_kv"))
        builder.robot.add_joint(Joint("j2_kv", JointType.FIXED, "l2_kv", "l1_kv"))

        with pytest.raises(RobotValidationError, match="(cyclic|NO_ROOT)"):
            builder.build(validate=True)

        # Cycle with a root (Root -> L1 -> L2 -> L1)
        builder3 = RobotBuilder("cycle_with_root")
        builder3.link("root_kv").root()
        builder3.link("l1_kv_2", parent="root_kv").fixed().commit()
        builder3.link("l2_kv_2", parent="l1_kv_2").fixed().commit()
        builder3.robot.add_joint(Joint("cycle_joint", JointType.FIXED, "l2_kv_2", "l1_kv_2"))

        with pytest.raises(RobotValidationError, match="contains a cycle"):
            builder3.build(validate=True)

    def test_advanced_joint_properties(self) -> None:
        """Test mimic, safety, dynamics, and calibration properties."""
        builder = RobotBuilder("adv_joint")
        builder.link("base_aj").root()

        (
            builder.link("l1_aj", parent="base_aj")
            .revolute(axis=(0, 0, 1), limits=(-1, 1))
            .mimic("other_joint", multiplier=2.0)
            .dynamics(damping=0.5, friction=0.1)
            .safety(soft_lower=-0.9, k_velocity=10.0)
            .calibration(rising=0.1)
            .physics(self_collide=True)
            .commit()
        )

        j = builder.robot.joint("base_aj_to_l1_aj")
        assert j.mimic is not None
        assert j.mimic.joint == "other_joint"
        assert j.dynamics is not None
        assert j.dynamics.damping == 0.5
        assert j.safety_controller is not None
        assert j.safety_controller.soft_lower_limit == -0.9
        assert j.calibration is not None
        assert j.calibration.rising == 0.1

        link = builder.robot.link("l1_aj")
        assert link.physics.self_collide is True
        assert link.physics.gravity is True  # Default

    def test_export_validation(self) -> None:
        """Test validation during export to ensures integrity of generated URDF/SRDF."""
        builder = RobotBuilder("export_val")
        builder.link("base_ev").root()

        # Should pass
        builder.export_urdf(validate=True)
        builder.export_srdf(validate=True)

        # Should fail with cycle (base -> l1 -> base)
        builder.link("l1_ev", parent="base_ev").fixed().commit()
        builder.robot.add_joint(Joint("cycle_ev", JointType.FIXED, "l1_ev", "base_ev"))
        with pytest.raises(RobotValidationError):
            builder.export_urdf(validate=True)
        with pytest.raises(RobotValidationError):
            builder.export_srdf(validate=True)

    def test_builder_edge_cases(self) -> None:
        """Verify remaining builder edge cases and branching logic."""
        # LinkBuilder.build() (line 1037-1038)
        b1 = RobotBuilder("b1")
        robot = b1.link("base_b1").build()
        assert robot.name == "b1"

        # LinkBuilder.sensor() (line 989-990)
        b2 = RobotBuilder("b2")
        from linkforge_core.models.sensor import IMUInfo, Sensor

        s = Sensor(name="raw_s", type=SensorType.IMU, link_name="placeholder", imu_info=IMUInfo())
        b2.link("l_raw").sensor(s).root()
        assert len(b2.robot.sensors) == 1

        # Double commit (line 1043)
        b3 = RobotBuilder("b3")
        l3 = b3.link("l_dc")
        l3.root()
        l3._commit()  # Should return immediately due to _committed flag

        # Inertial origin not overwritten (line 1064 jump)
        b4 = RobotBuilder("b4")
        b4.link("l_origin").collision(box(1, 1, 1)).mass(1.0, origin_xyz=(0, 0, 1)).root()
        assert b4.robot.link("l_origin").inertial_origin.xyz.z == 1.0

        # Export jumps (validate=False)
        b5 = RobotBuilder("b5")
        b5.link("base").root()
        b5.export_urdf(validate=False)
        b5.export_srdf(validate=False)

    def test_ros2_control_no_system_error(self) -> None:
        """Verify error when ros2_control is added without a defined system."""
        builder = RobotBuilder("no_ctrl_sys")
        builder.link("base_ctrl").root()

        with pytest.raises(RobotValidationError, match="no global system exists"):
            builder.link("arm_ctrl", parent="base_ctrl").revolute(
                axis=(0, 0, 1), limits=(-1, 1)
            ).ros2_control(command_interfaces=["pos"], state_interfaces=["pos"]).commit()

    def test_missing_material_creation(self) -> None:
        """Test that a material string not in global materials raises an error."""
        builder = RobotBuilder("miss_mat")
        with pytest.raises(RobotValidationError, match="not found"):
            builder.link("l1_mm").visual(box(1, 1, 1), material="unknown_mat").root()

    def test_collision_inference_with_origin(self) -> None:
        """Test collision inference adding a custom origin offset."""
        builder = RobotBuilder("coll_inf")
        builder.link("l1_ci").visual(box(1, 1, 1)).collision(xyz=(0, 0, 1), rpy=(1.57, 0, 0)).root()
        c = builder.robot.link("l1_ci").collisions[0]
        assert c.origin.xyz.z == 1.0
        assert c.origin.rpy.x == 1.57

    def test_continuous_with_limits(self) -> None:
        """Test continuous joint with effort/velocity limits."""
        builder = RobotBuilder("cont_lim")
        builder.link("base").root()
        builder.link("l1", parent="base").continuous(
            axis=(0, 0, 1), effort=10.0, velocity=5.0
        ).commit()
        limits = builder.robot.joint("base_to_l1").limits
        from linkforge_core.models.joint import JointLimits

        assert isinstance(limits, JointLimits)
        assert limits.effort == 10.0
        assert limits.velocity == 5.0

    def test_inertia_warnings(self, caplog) -> None:
        """Test logging warnings when multiple visuals/collisions are used for inertia."""
        builder = RobotBuilder("warn")
        # Multiple collisions
        builder.link("l1_wc").collision(box(1, 1, 1)).collision(box(2, 2, 2)).mass(1.0).root()
        assert "multiple collisions" in caplog.text

        caplog.clear()
        # Multiple visuals
        builder.link("l1_wv").visual(box(1, 1, 1)).visual(box(2, 2, 2)).mass(1.0).root()
        assert "multiple visuals" in caplog.text

    def test_ros2_control_named_system_not_found(self) -> None:
        """Test error when specific ros2_control system name is not found."""
        builder = RobotBuilder("named_ctrl_err")
        builder.ros2_control("sys1", "plugin")
        builder.link("base").root()
        with pytest.raises(RobotValidationError, match="was not found"):
            builder.link("l1", parent="base").revolute(axis=(0, 0, 1), limits=(-1, 1)).ros2_control(
                ["pos"], ["pos"], system_name="wrong_sys"
            ).commit()

    def test_attach_empty_component(self) -> None:
        """Test attaching a component with no root link."""
        builder = RobotBuilder("main")
        builder.link("base").root()
        empty = RobotBuilder("empty")
        # root_link property raises NO_ROOT before attach() checks for None
        with pytest.raises(RobotValidationError, match="No root link"):
            _ = empty.robot.root_link
            builder.attach(empty, at_link="base")

    def test_ros2_control_named_system_found(self) -> None:
        """Test successfully assigning to a specific ros2_control system by name."""
        builder = RobotBuilder("named_ctrl")
        builder.ros2_control("sys1", "plugin1")
        builder.ros2_control("sys2", "plugin2")
        builder.link("base").root()

        # Link to second system explicitly
        (
            builder.link("l1", parent="base")
            .revolute(axis=(0, 0, 1), limits=(-1, 1))
            .ros2_control(["pos"], ["pos"], system_name="sys2")
            .commit()
        )

        # Verify it attached to the correct one
        assert len(builder.robot.ros2_controls[0].joints) == 0
        system2 = builder.robot.ros2_controls[1]
        assert len(system2.joints) == 1
        joint_ctrl = system2.joints[0]
        assert joint_ctrl.name == "base_to_l1"

    def test_attach_with_collision_disable(self) -> None:
        """Test that attach(..., disable_collision=True) correctly adds semantic data."""
        b1 = RobotBuilder("main")
        b1.link("base").root()

        b2 = RobotBuilder("gripper")
        b2.link("gripper_base").root()

        # Attach with collision disable
        b1.attach(b2, at_link="base", disable_collision=True, reason="Adjacent")

        assert len(b1.robot.semantic.disabled_collisions) == 1
        dc = b1.robot.semantic.disabled_collisions[0]
        assert dc.link1 == "base"
        assert dc.link2 == "gripper_base"
        assert dc.reason == "Adjacent"

        # Test with prefix
        b3 = RobotBuilder("prefixed")
        b3.link("root").root()
        b1.attach(b3, at_link="base", prefix="p1_", disable_collision=True)

        assert len(b1.robot.semantic.disabled_collisions) == 2
        dc2 = b1.robot.semantic.disabled_collisions[1]
        assert dc2.link1 == "base"
        assert dc2.link2 == "p1_root"
