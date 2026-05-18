import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge.core import (
    Box,
    Collision,
    Color,
    Cylinder,
    Inertial,
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
    Link,
    Material,
    Mesh,
    Robot,
    RobotGeneratorError,
    Sphere,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    Vector3,
    Visual,
    XACROGenerator,
)
from linkforge.core.generators.xacro_generator import XACRO_URI


class TestXacroGenerator:
    @pytest.fixture
    def empty_robot(self):
        return Robot(name="test_robot")

    @pytest.fixture
    def simple_robot(self):
        robot = Robot(name="simple_robot")
        # Add basic link and joint to make it a valid robot model
        link_base = Link(name="base_link", inertial=Inertial(mass=1.0))
        link_child = Link(name="child_link", inertial=Inertial(mass=1.0))
        joint = Joint(
            name="base_to_child",
            type=JointType.FIXED,
            parent="base_link",
            child="child_link",
            origin=Transform(xyz=Vector3(1, 2, 3), rpy=Vector3(0.1, 0.2, 0.3)),
        )
        robot.add_link(link_base)
        robot.add_link(link_child)
        robot.add_joint(joint)
        return robot

    def test_generator_instantiation(self):
        """Test instantiation with various options."""
        gen1 = XACROGenerator()
        assert gen1.advanced_mode is True
        assert gen1.extract_materials is True
        assert gen1.extract_dimensions is True
        assert gen1.generate_macros is False
        assert gen1.split_files is False

        gen2 = XACROGenerator(advanced_mode=False)
        assert gen2.advanced_mode is False
        assert gen2.extract_materials is False
        assert gen2.extract_dimensions is False

        gen3 = XACROGenerator(
            generate_macros=True,
            split_files=True,
            output_path=Path("out.xacro"),
        )
        assert gen3.generate_macros is True
        assert gen3.split_files is True
        assert gen3.output_path == Path("out.xacro")

    def test_validation_failure(self, empty_robot):
        """Test that generator raises RobotGeneratorError if robot is invalid."""
        gen = XACROGenerator()
        # Empty robot is invalid because it has no links/joints (validation requires at least one link)
        with pytest.raises(RobotGeneratorError) as exc:
            gen.generate(empty_robot, validate=True)
        assert "Robot validation failed" in str(exc.value)

    def test_material_extraction(self, simple_robot):
        """Test extracting material colors as XACRO properties."""
        mat_red = Material(name="My-Red", color=Color(1.0, 0.0, 0.0, 1.0))
        mat_blue = Material(name="My-Blue", color=Color(0.0, 0.0, 1.0, 1.0))
        mat_texture = Material(name="My-Texture", texture="texture.png")

        # Assign materials to visuals
        visual1 = Visual(name="v1", geometry=Box(size=Vector3(1, 1, 1)), material=mat_red)
        visual2 = Visual(name="v2", geometry=Box(size=Vector3(1, 1, 1)), material=mat_blue)
        visual3 = Visual(name="v3", geometry=Box(size=Vector3(1, 1, 1)), material=mat_texture)

        simple_robot.links[0].add_visual(visual1)
        simple_robot.links[0].add_visual(visual2)
        simple_robot.links[1].add_visual(visual3)

        gen = XACROGenerator(extract_materials=True)
        xml_str = gen.generate(simple_robot)
        root = ET.fromstring(xml_str)

        # Check property elements
        ns = {"xacro": XACRO_URI}
        properties = root.findall(".//xacro:property", ns)

        # Verify property names are sanitized and assigned values
        prop_names = {p.get("name"): p.get("value") for p in properties}
        assert "my_red" in prop_names
        assert prop_names["my_red"] == "1 0 0 1"
        assert "my_blue" in prop_names
        assert prop_names["my_blue"] == "0 0 1 1"
        # Texture-only material shouldn't be extracted as property
        assert "my_texture" not in prop_names

        # Verify global materials reference these properties
        global_materials = root.findall("./material")
        assert len(global_materials) == 3
        mat_dict = {m.get("name"): m for m in global_materials}
        assert "My-Red" in mat_dict
        rgba_color = mat_dict["My-Red"].find("color").get("rgba")
        assert rgba_color == "${my_red}"

        # Verify standard URDF fallback when material extraction is disabled
        gen_std = XACROGenerator(extract_materials=False)
        xml_std = gen_std.generate(simple_robot)
        root_std = ET.fromstring(xml_std)
        mat_dict_std = {m.get("name"): m for m in root_std.findall("./material")}
        assert mat_dict_std["My-Red"].find("color").get("rgba") == "1 0 0 1"

    def test_dimensions_extraction(self, simple_robot):
        """Test extracting common dimensions as properties."""
        # 1. Box dimensions
        visual1 = Visual(geometry=Box(size=Vector3(0.5, 0.2, 0.8)))
        visual2 = Visual(geometry=Box(size=Vector3(0.5, 0.2, 0.8)))

        # 2. Cylinder dimensions
        visual3 = Visual(geometry=Cylinder(radius=0.05, length=0.2))
        visual4 = Visual(geometry=Cylinder(radius=0.05, length=0.2))

        # 3. Sphere dimensions
        visual5 = Visual(geometry=Sphere(radius=0.03))
        visual6 = Visual(geometry=Sphere(radius=0.03))

        # Add prefix name matching triggers (wheel, leg, ball)
        link1 = Link(name="left_wheel", inertial=Inertial(mass=1.0))
        link2 = Link(name="right_wheel", inertial=Inertial(mass=1.0))
        link1.add_visual(visual3)
        link2.add_visual(visual4)

        link3 = Link(name="left_leg", inertial=Inertial(mass=1.0))
        link4 = Link(name="right_leg", inertial=Inertial(mass=1.0))
        link3.add_visual(visual1)
        link4.add_visual(visual2)

        link5 = Link(name="ball_a", inertial=Inertial(mass=1.0))
        link6 = Link(name="ball_b", inertial=Inertial(mass=1.0))
        link5.add_visual(visual5)
        link6.add_visual(visual6)

        simple_robot.add_link(link1)
        simple_robot.add_link(link2)
        simple_robot.add_link(link3)
        simple_robot.add_link(link4)
        simple_robot.add_link(link5)
        simple_robot.add_link(link6)

        gen = XACROGenerator(extract_dimensions=True)
        xml_str = gen.generate(simple_robot, validate=False)
        root = ET.fromstring(xml_str)

        ns = {"xacro": XACRO_URI}
        properties = root.findall(".//xacro:property", ns)
        prop_names = {p.get("name"): p.get("value") for p in properties}

        # Cylinder radius & length suffix/prefix logic
        assert "wheel_radius" in prop_names
        assert prop_names["wheel_radius"] == "0.05"
        assert "wheel_length" in prop_names
        assert prop_names["wheel_length"] == "0.2"

        # Box height, width, depth
        assert "leg_width" in prop_names
        assert prop_names["leg_width"] == "0.5"
        assert "leg_depth" in prop_names
        assert prop_names["leg_depth"] == "0.2"
        assert "leg_height" in prop_names
        assert prop_names["leg_height"] == "0.8"

        # Sphere radius
        assert "ball_radius" in prop_names
        assert prop_names["ball_radius"] == "0.03"

    def test_dimensions_extraction_no_common_prefix(self, simple_robot):
        """Test fallback dimension property naming when there is no common prefix."""
        visual1 = Visual(geometry=Cylinder(radius=0.15, length=0.45))
        visual2 = Visual(geometry=Cylinder(radius=0.15, length=0.45))

        # Use non-matching link names
        link1 = Link(name="apple", inertial=Inertial(mass=1.0))
        link2 = Link(name="banana", inertial=Inertial(mass=1.0))
        link1.add_visual(visual1)
        link2.add_visual(visual2)

        simple_robot.add_link(link1)
        simple_robot.add_link(link2)

        gen = XACROGenerator(extract_dimensions=True)
        xml_str = gen.generate(simple_robot, validate=False)
        root = ET.fromstring(xml_str)

        ns = {"xacro": XACRO_URI}
        properties = root.findall(".//xacro:property", ns)
        prop_names = {p.get("name"): p.get("value") for p in properties}

        # Falls back to cylinder_radius/cylinder_length
        assert "cylinder_radius" in prop_names
        assert prop_names["cylinder_radius"] == "0.15"
        assert "cylinder_length" in prop_names
        assert prop_names["cylinder_length"] == "0.45"

    def test_macro_generation_identical_patterns(self, simple_robot):
        """Test auto-macro generation when links and joints have identical structures."""
        # Define identical links with visual and collision details
        geom = Cylinder(radius=0.1, length=0.5)
        visual = Visual(
            name="v", geometry=geom, material=Material(name="Blue", color=Color(0, 0, 1, 1))
        )

        link_fl = Link(name="fl_wheel", inertial=Inertial(mass=1.0))
        link_fl.add_visual(visual)

        link_fr = Link(name="fr_wheel", inertial=Inertial(mass=1.0))
        link_fr.add_visual(visual)

        # Define identical joints
        joint_fl = Joint(
            name="fl_wheel_joint",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="fl_wheel",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=5.0),
            dynamics=JointDynamics(damping=0.1, friction=0.05),
            mimic=JointMimic(joint="other_joint", multiplier=1.0, offset=0.0),
            safety_controller=JointSafetyController(
                soft_lower_limit=-3.0, soft_upper_limit=3.0, k_position=1.0, k_velocity=2.0
            ),
            calibration=JointCalibration(rising=0.1, falling=0.2),
            origin=Transform(xyz=Vector3(0.5, 0.5, 0), rpy=Vector3(0, 0, 0)),
        )

        joint_fr = Joint(
            name="fr_wheel_joint",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="fr_wheel",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=5.0),
            dynamics=JointDynamics(damping=0.1, friction=0.05),
            mimic=JointMimic(joint="other_joint", multiplier=1.0, offset=0.0),
            safety_controller=JointSafetyController(
                soft_lower_limit=-3.0, soft_upper_limit=3.0, k_position=1.0, k_velocity=2.0
            ),
            calibration=JointCalibration(rising=0.1, falling=0.2),
            origin=Transform(xyz=Vector3(0.5, -0.5, 0), rpy=Vector3(0, 0, 0)),
        )

        simple_robot.add_link(link_fl)
        simple_robot.add_link(link_fr)
        simple_robot.add_joint(joint_fl)
        simple_robot.add_joint(joint_fr)

        gen = XACROGenerator(generate_macros=True)
        xml_str = gen.generate(simple_robot, validate=False)
        root = ET.fromstring(xml_str)

        ns = {"xacro": XACRO_URI}

        # Verify macro definition was generated
        macros = root.findall(".//xacro:macro", ns)
        assert len(macros) == 1
        macro = macros[0]
        assert macro.get("name").startswith("cylinder_")
        assert macro.get("params") == "name parent xyz rpy"

        # Verify macro calls (fl_wheel and fr_wheel)
        macro_name = macro.get("name")
        calls = root.findall(f".//xacro:{macro_name}", ns)
        assert len(calls) == 2

        call_names = {c.get("name"): c for c in calls}
        assert "fl_wheel" in call_names
        assert call_names["fl_wheel"].get("parent") == "base_link"
        assert call_names["fl_wheel"].get("xyz") == "0.5 0.5 0"

        assert "fr_wheel" in call_names
        assert call_names["fr_wheel"].get("parent") == "base_link"
        assert call_names["fr_wheel"].get("xyz") == "0.5 -0.5 0"

    def test_mesh_geometry_scale_and_path(self, simple_robot):
        """Test mesh geometry export path, scaling, and output directory."""
        visual = Visual(
            geometry=Mesh(
                resource="package://my_pack/meshes/base.dae",
                scale=Vector3(2.0, 3.0, 4.0),
            )
        )
        simple_robot.links[0].add_visual(visual)

        gen = XACROGenerator(output_path=Path("/tmp/robot.xacro"))
        xml_str = gen.generate(simple_robot, validate=False)
        root = ET.fromstring(xml_str)

        mesh_elem = root.find(".//mesh")
        assert mesh_elem is not None
        assert mesh_elem.get("filename") == "package://my_pack/meshes/base.dae"
        assert mesh_elem.get("scale") == "2 3 4"

    def test_split_files_generation(self, simple_robot, tmp_path):
        """Test modular split files writing options."""
        # Set up properties and macros triggers
        mat = Material(name="Custom-Red", color=Color(1.0, 0, 0, 1.0))
        visual = Visual(geometry=Cylinder(radius=0.1, length=0.5), material=mat)

        link_fl = Link(name="fl_wheel", inertial=Inertial(mass=1.0))
        link_fl.add_visual(visual)

        link_fr = Link(name="fr_wheel", inertial=Inertial(mass=1.0))
        link_fr.add_visual(visual)

        joint_fl = Joint(
            name="fl_wheel_joint",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="fl_wheel",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=5.0),
            origin=Transform(xyz=Vector3(0.5, 0.5, 0)),
        )
        joint_fr = Joint(
            name="fr_wheel_joint",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="fr_wheel",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=5.0),
            origin=Transform(xyz=Vector3(0.5, -0.5, 0)),
        )

        simple_robot.add_link(link_fl)
        simple_robot.add_link(link_fr)
        simple_robot.add_joint(joint_fl)
        simple_robot.add_joint(joint_fr)

        # Transmission/ROS2 Control triggers
        trans = Transmission(
            name="trans_fl",
            type="TransmissionType",
            joints=(TransmissionJoint(name="fl_wheel_joint", hardware_interfaces=["position"]),),
            actuators=(TransmissionActuator(name="act_fl", hardware_interfaces=["position"]),),
        )
        simple_robot.add_transmission(trans)

        main_file = tmp_path / "my_robot.xacro"

        gen = XACROGenerator(
            generate_macros=True,
            split_files=True,
            extract_materials=True,
            extract_dimensions=True,
            use_ros2_control=True,
        )
        gen.write(simple_robot, main_file, validate=False)

        # Confirm all separate modular xacro files are created
        prop_file = tmp_path / "simple_robot_properties.xacro"
        mac_file = tmp_path / "simple_robot_macros.xacro"
        ctrl_file = tmp_path / "simple_robot_ros2_control.xacro"

        assert main_file.exists()
        assert prop_file.exists()
        assert mac_file.exists()
        assert ctrl_file.exists()

        # Check main file contains the includes
        main_xml = ET.fromstring(main_file.read_text())
        ns = {"xacro": XACRO_URI}
        includes = main_xml.findall("xacro:include", ns)
        include_filenames = {inc.get("filename") for inc in includes}

        assert "simple_robot_properties.xacro" in include_filenames
        assert "simple_robot_macros.xacro" in include_filenames
        assert "simple_robot_ros2_control.xacro" in include_filenames

        # Verify property file contains extracted material
        prop_xml = ET.fromstring(prop_file.read_text())
        props = prop_xml.findall("xacro:property", ns)
        prop_names = [p.get("name") for p in props]
        assert "custom_red" in prop_names

        # Verify macros file contains macro definition
        mac_xml = ET.fromstring(mac_file.read_text())
        macros = mac_xml.findall("xacro:macro", ns)
        assert len(macros) == 1
        assert macros[0].get("name").startswith("cylinder_")

        # Verify control file contains transmission
        ctrl_xml = ET.fromstring(ctrl_file.read_text())
        assert ctrl_xml.find("ros2_control") is not None

    def test_write_single_file(self, simple_robot, tmp_path):
        """Test write method saves robot description to a single XACRO file."""
        generator = XACROGenerator(split_files=False)
        filepath = tmp_path / "robot.xacro"
        generator.write(simple_robot, filepath, validate=False)
        assert filepath.exists()
        content = filepath.read_text()
        assert "robot" in content
        assert "simple_robot" in content

    def test_split_files_generation_empty(self, empty_robot, tmp_path):
        """Test split file generation on a robot with no properties, macros, or control elements."""
        # Add basic link so it has at least some content
        empty_robot.add_link(Link(name="base_link"))
        generator = XACROGenerator(
            split_files=True,
            extract_materials=False,
            extract_dimensions=False,
            generate_macros=False,
        )
        main_filepath = tmp_path / "empty_robot.xacro"
        generator.write(empty_robot, main_filepath, validate=False)

        assert main_filepath.exists()
        # Ensure no auxiliary files were written
        assert not (tmp_path / "empty_robot_properties.xacro").exists()
        assert not (tmp_path / "empty_robot_macros.xacro").exists()
        assert not (tmp_path / "empty_robot_ros2_control.xacro").exists()

    def test_macro_signature_exhaustive(self):
        """Test _get_macro_signature covers all geometry, origin, and joint types."""
        from linkforge.core import Collision

        generator = XACROGenerator()

        # 1. No visuals returns None
        link_no_vis = Link(name="l")
        joint = Joint(name="j", type=JointType.FIXED, parent="p", child="c")
        assert generator._get_macro_signature(link_no_vis, joint) is None

        # 2. Exhaustive visuals and collisions
        link = Link(name="l")

        # Visual sphere
        vis_sphere = Visual(
            geometry=Sphere(radius=1.5),
            origin=Transform(xyz=Vector3(1, 2, 3), rpy=Vector3(0.1, 0.2, 0.3)),
        )
        link.add_visual(vis_sphere)

        # Visual cylinder
        vis_cyl = Visual(geometry=Cylinder(radius=0.5, length=2.0))
        link.add_visual(vis_cyl)

        # Visual mesh with material
        vis_mesh = Visual(
            geometry=Mesh(resource="package://test/mesh.stl"),
            material=Material(name="custom_blue", color=Color(0, 0, 1, 1)),
        )
        link.add_visual(vis_mesh)

        # Collision box
        coll_box = Collision(
            geometry=Box(size=Vector3(1, 2, 3)),
            origin=Transform(xyz=Vector3(4, 5, 6), rpy=Vector3(0.4, 0.5, 0.6)),
        )
        link.add_collision(coll_box)

        # Collision sphere
        coll_sphere = Collision(geometry=Sphere(radius=2.5))
        link.add_collision(coll_sphere)

        # Collision cylinder
        coll_cyl = Collision(geometry=Cylinder(radius=1.2, length=3.4))
        link.add_collision(coll_cyl)

        # Collision mesh
        coll_mesh = Collision(geometry=Mesh(resource="package://test/col_mesh.dae"))
        link.add_collision(coll_mesh)

        # Joint details
        joint_full = Joint(
            name="j",
            type=JointType.REVOLUTE,
            parent="p",
            child="c",
            axis=Vector3(1, 0, 0),
            limits=JointLimits(effort=10.0, velocity=5.0, lower=-1.0, upper=1.0),
            dynamics=JointDynamics(damping=0.5, friction=0.2),
        )

        signature = generator._get_macro_signature(link, joint_full)
        assert signature is not None
        assert "v_sphere" in signature
        assert "v_cylinder" in signature
        assert "v_mesh" in signature
        assert "custom_blue" in signature
        assert "c_box" in signature
        assert "c_sphere" in signature
        assert "c_cylinder" in signature
        assert "c_mesh" in signature
        assert "a_1.000_0.000_0.000" in signature
        assert "l_10.000_5.000" in signature

    def test_macro_definition_and_call_edge_cases(self):
        """Test _generate_macro_definition and _generate_macro_call with None joint returns None."""
        generator = XACROGenerator()
        parent = ET.Element("robot")
        link = Link(name="l")

        # None joint returns early
        assert generator._generate_macro_definition(parent, "sig", [(link, None)]) is None
        assert generator._generate_macro_call(parent, "sig", link, None) is None

    def test_add_geometry_element_fallback_and_mesh_unit_scale(self):
        """Test fallback geometry parsing and mesh geometry with default unit scale."""
        generator = XACROGenerator()
        parent = ET.Element("geometry")

        # Test unregistered geometry fallback
        generator._add_geometry_element(object(), parent)
        assert len(parent) == 1
        assert parent[0].tag == "geometry"
        assert len(parent[0]) == 0

        # Test mesh with 1.0 scale (default)
        mesh_geom = Mesh(resource="package://test/mesh.stl", scale=Vector3(1.0, 1.0, 1.0))
        generator._add_geometry_element(mesh_geom, parent)
        mesh_elem = parent.find(".//mesh")
        assert mesh_elem is not None
        assert "scale" not in mesh_elem.attrib

    def test_get_common_base_name_empty(self):
        """Test _find_common_prefix returns empty string with empty names list."""
        generator = XACROGenerator()
        assert generator._find_common_prefix([]) == ""

    def test_generator_flags(self, empty_robot):
        """Test generator with custom flags disabled."""
        generator = XACROGenerator(
            extract_materials=False,
            extract_dimensions=False,
            generate_macros=False,
        )
        xml = generator.generate(empty_robot, validate=False)
        assert xml is not None

    def test_global_material_without_property(self):
        """Test global material generation with color but not in properties."""
        generator = XACROGenerator(extract_materials=False)
        robot = Robot(name="test")
        test_link = Link(name="l")
        mat = Material(name="red", color=Color(1, 0, 0, 1))
        test_link.add_visual(Visual(geometry=Box(size=Vector3(1, 1, 1)), material=mat))
        robot.add_link(test_link)

        # Run macro signatures check with material in global dict
        generator.global_materials = generator._collect_materials(robot)
        root = ET.Element("robot")
        generator._add_material_element(root, mat)

        mat_elem = root.find("material")
        assert mat_elem is not None
        assert mat_elem.get("name") == "red"
        color_elem = mat_elem.find("color")
        assert color_elem is not None
        assert color_elem.get("rgba") == "1 0 0 1"

    def test_macro_generation_safety_checks_dynamic(self):
        """Test safety guards in _add_link_to_xml when joint is dynamically modified/deleted."""
        generator = XACROGenerator(generate_macros=True)
        robot = Robot(name="test")
        l1 = Link(name="l1")
        l1.add_visual(Visual(geometry=Sphere(radius=1.0)))
        l2 = Link(name="l2")
        l2.add_visual(Visual(geometry=Sphere(radius=1.0)))
        robot.add_link(l1)
        robot.add_link(l2)

        joint = Joint(name="j", type=JointType.FIXED, parent="l1", child="l2")
        robot.add_joint(joint)

        # First generate works and identifies macros
        generator.macro_groups = generator._identify_macro_groups(robot)
        generator.links_in_macros = set()
        for group in generator.macro_groups.values():
            for link, _ in group:
                generator.links_in_macros.add(link.name)

        generator._current_robot = robot

        # Dynamically clear joints to trigger 'if not joint' in _add_link_to_xml
        robot.joints = ()
        parent = ET.Element("robot")
        generator._add_link_to_xml(parent, l2)
        # Should fall back to standard Link Generation
        assert parent.find("link") is not None
        assert parent.find("link").get("name") == "l2"

    def test_xacro_generator_box_visual_and_inline_material(self):
        """Test Box visual inside macro signature and inline materials export."""
        generator = XACROGenerator()
        generator.global_materials = {}  # Ensure material is not global
        generator.links_in_macros = set()
        robot = Robot(name="test")
        generator._current_robot = robot

        # 1. Box visual inside _get_macro_signature
        link = Link(name="l1")
        link.add_visual(Visual(geometry=Box(size=Vector3(1, 2, 3))))
        joint = Joint(name="j1", type=JointType.FIXED, parent="p", child="l1")
        sig = generator._get_macro_signature(link, joint)
        assert sig is not None
        assert "v_box" in sig
        assert "1.000_2.000_3.000" in sig

        # 2. Inline material is serialized inline rather than referenced
        link.add_visual(
            Visual(
                geometry=Sphere(radius=1.0),
                material=Material(name="inline_color", color=Color(0.5, 0.5, 0.5, 1.0)),
            )
        )
        parent = ET.Element("robot")
        generator._add_link_to_xml(parent, link)
        link_elem = parent.find("link")
        assert link_elem is not None
        visual_elem = link_elem.findall("visual")[1]
        mat_elem = visual_elem.find("material")
        assert mat_elem is not None
        assert mat_elem.get("name") == "inline_color"
        assert mat_elem.find("color") is not None

    def test_xacro_generator_use_ros2_control_false(self, tmp_path):
        """Test split write with use_ros2_control=False."""
        generator = XACROGenerator(use_ros2_control=False)
        robot = Robot(name="simple_robot")
        robot.add_link(Link(name="base"))

        main_filepath = tmp_path / "simple_robot.xacro"
        generator.write(robot, main_filepath, validate=False)
        assert main_filepath.exists()
        assert not (tmp_path / "simple_robot_ros2_control.xacro").exists()

    def test_xacro_generator_link_in_macro_no_joint_edge_cases(self):
        """Test macro generation edge cases for orphan links in macros."""
        generator = XACROGenerator(generate_macros=True)
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        # Artificially populate links_in_macros for an orphan link
        generator.links_in_macros = {"l1"}
        generator._current_robot = robot
        parent = ET.Element("robot")
        generator._add_link_to_xml(parent, l1)
        # Should generate standard link as fallback since parent/child joint is missing
        assert parent.find("link") is not None

    def test_xacro_generator_unique_dimensions(self):
        """Test that single/unique link dimension is not grouped in macro signature checks."""
        from linkforge.core.generators.xacro_generator import XACRO_URI

        generator = XACROGenerator(extract_dimensions=True)
        robot = Robot(name="test")

        # Link 1 has unique dimensions, Link 2 has different unique dimensions
        l1 = Link(name="l1")
        l1.add_visual(Visual(geometry=Box(size=Vector3(1.11, 2.22, 3.33))))
        l2 = Link(name="l2")
        l2.add_visual(Visual(geometry=Box(size=Vector3(4.44, 5.55, 6.66))))
        robot.add_link(l1)
        robot.add_link(l2)

        parent = generator.generate_robot_element(robot, validate=False)
        # Verify that unique properties were not extracted (they only extract for 2+ instances)
        properties = parent.findall(f"{{{XACRO_URI}}}property")
        assert not any("1.11" in str(p.get("value")) for p in properties)

    def test_xacro_generator_gazebo_no_plugin(self, tmp_path):
        """Test multi-file split write with a gazebo element that lacks plugin tags."""
        from linkforge.core import GazeboElement

        generator = XACROGenerator()
        robot = Robot(name="gz_robot")
        robot.add_link(Link(name="base"))

        # Gazebo element without plugin
        robot.gazebo_elements = [
            GazeboElement(reference="base", properties={"self_collide": "true"})
        ]

        main_filepath = tmp_path / "gz_robot.xacro"
        generator.write(robot, main_filepath, validate=False)
        assert main_filepath.exists()
        # Should NOT write a control file since there were no plugins in gazebo
        assert not (tmp_path / "gz_robot_ros2_control.xacro").exists()

    def test_mixed_geometry_split_files_and_macro_grouping_edge_cases(self, mocker, tmp_path):
        """Verify xacro generation with mixed geometry types (Box, Mesh, custom fallback),
        partial joint limits (lower-only, upper-only), split-file output, and macro
        grouping resilience when a signature cannot be resolved during generation.
        """
        from dataclasses import dataclass

        from linkforge.core import GazeboElement, GazeboPlugin
        from linkforge.core.models.geometry import Box, GeometryType, Mesh
        from linkforge.core.models.joint import JointLimits
        from linkforge.core.models.material import Color, Material

        @dataclass(frozen=True)
        class CustomGeometry:
            @property
            def type(self):
                return GeometryType.BOX

        robot = Robot(name="comprehensive_robot")
        base = Link(name="base")
        link1 = Link(name="link1")
        link2 = Link(name="link2")
        link3 = Link(name="link3")
        link4 = Link(name="link4")

        # Visual/Collision with Box (has origin & material) and Mesh (no origin)
        mat_red = Material(name="My-Red", color=Color(1.0, 0.0, 0.0, 1.0))

        # Link 1 Visuals
        vis_mesh = Visual(geometry=Mesh(resource="package://test/mesh.stl"))
        vis_box = Visual(
            geometry=Box(size=Vector3(1, 1, 1)),
            origin=Transform(xyz=Vector3(1, 2, 3), rpy=Vector3(0, 0, 0)),
            material=mat_red,
        )
        vis_custom = Visual(geometry=CustomGeometry(), origin=None)

        link1.add_visual(vis_mesh)
        link1.add_visual(vis_box)
        link1.add_visual(vis_custom)

        # Link 1 Collisions
        coll_mesh = Collision(geometry=Mesh(resource="package://test/col_mesh.dae"))
        coll_box = Collision(
            geometry=Box(size=Vector3(1, 1, 1)),
            origin=Transform(xyz=Vector3(1, 2, 3), rpy=Vector3(0, 0, 0)),
        )
        coll_custom = Collision(geometry=CustomGeometry(), origin=None)

        link1.add_collision(coll_mesh)
        link1.add_collision(coll_box)
        link1.add_collision(coll_custom)

        # Link 2 visuals/collisions (identical to Link 1 for macro grouping)
        link2.add_visual(vis_mesh)
        link2.add_visual(vis_box)
        link2.add_visual(vis_custom)
        link2.add_collision(coll_mesh)
        link2.add_collision(coll_box)
        link2.add_collision(coll_custom)

        # Link 3 and Link 4 Visuals (standard Box, no origin)
        vis_box_simple = Visual(geometry=Box(size=Vector3(1, 1, 1)))
        link3.add_visual(vis_box_simple)
        link4.add_visual(vis_box_simple)

        robot.add_link(base)
        robot.add_link(link1)
        robot.add_link(link2)
        robot.add_link(link3)
        robot.add_link(link4)

        # Joints: revolute type
        # Joint 1 & 2 have limits with lower only
        j1 = Joint(
            name="j1",
            type=JointType.REVOLUTE,
            parent="base",
            child="link1",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=10.0, velocity=1.0, lower=-1.5, upper=None),
        )
        j2 = Joint(
            name="j2",
            type=JointType.REVOLUTE,
            parent="base",
            child="link2",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=10.0, velocity=1.0, lower=-1.5, upper=None),
        )

        # Joint 3 & 4 have limits with upper only
        j3 = Joint(
            name="j3",
            type=JointType.REVOLUTE,
            parent="base",
            child="link3",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=10.0, velocity=1.0, lower=None, upper=1.5),
        )
        j4 = Joint(
            name="j4",
            type=JointType.REVOLUTE,
            parent="base",
            child="link4",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=10.0, velocity=1.0, lower=None, upper=1.5),
        )

        robot.add_joint(j1)
        robot.add_joint(j2)
        robot.add_joint(j3)
        robot.add_joint(j4)

        # Gazebo element WITH a plugin AND one WITHOUT a plugin
        robot.gazebo_elements = [
            GazeboElement(reference="base", properties={"self_collide": "true"}),
            GazeboElement(
                reference="link1", plugins=[GazeboPlugin(name="gz_plugin", filename="libgz.so")]
            ),
        ]

        # Patch _get_macro_signature to exercise 224->229 (sig exists but not in macro_groups)
        original_get_sig = XACROGenerator._get_macro_signature

        def mock_get_sig(self, link, joint):
            if getattr(self, "_in_generation_phase", False):
                return "invalid_sig_not_in_macro_groups"
            return original_get_sig(self, link, joint)

        mocker.patch.object(XACROGenerator, "_get_macro_signature", mock_get_sig)

        original_identify = XACROGenerator._identify_macro_groups

        def mock_identify(self, robot):
            res = original_identify(self, robot)
            self._in_generation_phase = True
            return res

        mocker.patch.object(XACROGenerator, "_identify_macro_groups", mock_identify)

        # Run generator in split files mode
        gen = XACROGenerator(generate_macros=True, split_files=True)
        main_filepath = tmp_path / "comprehensive_robot.xacro"
        gen.write(robot, main_filepath, validate=False)
        assert main_filepath.exists()
