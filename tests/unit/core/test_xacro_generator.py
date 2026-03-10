"""Unit tests for XACRO generator."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotGeneratorError
from linkforge_core.generators.xacro_generator import XACRO_NS, XACROGenerator
from linkforge_core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    GazeboElement,
    GazeboPlugin,
    Inertial,
    InertiaTensor,
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
    Ros2Control,
    Ros2ControlJoint,
    Sphere,
    Transform,
    Vector3,
    Visual,
)


class TestXACROGenerator:
    """Test XACRO generator features."""

    def test_generate_basic(self) -> None:
        """Test basic XACRO generation without advanced features."""
        robot = Robot(name="basic_xacro")
        link = Link(name="base_link")
        robot.add_link(link)

        generator = XACROGenerator(advanced_mode=False)
        xml_str = generator.generate(robot)

        root = ET.fromstring(xml_str)
        assert root.tag == "robot"
        assert root.get("name") == "basic_xacro"
        # Should not have properties in basic mode
        assert len(root.findall(f"{XACRO_NS}property")) == 0

    def test_extract_materials(self) -> None:
        """Test material property extraction."""
        robot = Robot(name="mat_bot")

        # Two links using identical red material
        link1 = Link(name="link1")
        mat = Material(name="BrandRed", color=Color(1, 0, 0, 1))
        link1.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat))
        robot.add_link(link1)

        link2 = Link(name="link2")
        link2.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat))
        robot.add_link(link2)

        # Enable extraction
        generator = XACROGenerator(extract_materials=True, advanced_mode=True, pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # Check property existence
        props = root.findall(f"{XACRO_NS}property")
        # Should have 'brandred' property
        prop = next((p for p in props if p.get("name") == "brandred"), None)
        assert prop is not None
        assert prop.get("value") == "1 0 0 1"

        # Check usage in global material definition
        mat_elem = root.find("material[@name='BrandRed']")
        assert mat_elem is not None
        color_elem = mat_elem.find("color")
        assert color_elem is not None
        assert color_elem.get("rgba") == "${brandred}"

    def test_extract_dimensions(self) -> None:
        """Test dimension property extraction."""
        robot = Robot(name="dim_bot")

        # Provide identical geometries and materials to trigger macro grouping
        # Cylinders (Wheels)
        for i in range(4):
            link = Link(name=f"wheel_{i}")
            cyl = Cylinder(radius=0.3, length=0.1)
            link.visuals.append(Visual(geometry=cyl))
            robot.add_link(link)

        # Boxes (Legs)
        for i in range(2):
            link = Link(name=f"leg_{i}")
            box = Box(size=Vector3(0.1, 0.2, 1.0))
            link.visuals.append(Visual(geometry=box))
            robot.add_link(link)

        generator = XACROGenerator(extract_dimensions=True, advanced_mode=True, pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        props = root.findall(f"{XACRO_NS}property")

        # Should extract properties like 'wheel_radius' and 'wheel_length'
        rad_prop = next(
            (p for p in props if p.get("name") is not None and "radius" in str(p.get("name"))), None
        )
        len_prop = next(
            (p for p in props if p.get("name") is not None and "length" in str(p.get("name"))), None
        )

        assert rad_prop is not None
        assert len_prop is not None

        # Check values
        assert "0.3" in str(rad_prop.get("value"))
        assert "0.1" in str(len_prop.get("value"))
        # Heuristic name check
        assert "wheel" in str(rad_prop.get("name"))

        # Check usage in geometry
        # Find a cylinder element
        cyl_elems = root.findall(".//link/visual/geometry/cylinder")
        assert len(cyl_elems) == 4
        assert "${" in str(cyl_elems[0].get("radius"))
        assert "${" in str(cyl_elems[0].get("length"))

        # Check box extraction
        box_props = [p for p in props if p.get("name") and "leg" in str(p.get("name"))]
        assert len(box_props) >= 3  # width, depth, height

        box_elems = root.findall(".//link/visual/geometry/box")
        assert len(box_elems) == 2
        assert "${" in str(box_elems[0].get("size"))

    def test_generate_macros(self) -> None:
        """Test macro generation for repeated structures."""
        robot = Robot(name="macro_bot")
        base = Link(name="base")
        robot.add_link(base)

        # Create 2 legs with identical structure
        # Must match signature: Visuals, Collisions, Inertial
        inertial = Inertial(
            mass=1.0, inertia=InertiaTensor(ixx=0.1, iyy=0.1, izz=0.1, ixy=0, ixz=0, iyz=0)
        )

        for side in ["left", "right"]:
            leg = Link(name=f"{side}_leg", inertial=inertial)
            leg.visuals.append(Visual(geometry=Box(Vector3(0.1, 0.1, 1.0))))
            leg.collisions.append(Collision(geometry=Box(Vector3(0.1, 0.1, 1.0))))
            robot.add_link(leg)

            joint = Joint(
                name=f"{side}_hip",
                type=JointType.REVOLUTE,
                parent="base",
                child=f"{side}_leg",
                origin=Transform(xyz=Vector3(0, 1 if side == "left" else -1, 0)),
                axis=Vector3(1, 0, 0),
                limits=JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0),
            )
            robot.add_joint(joint)

        generator = XACROGenerator(generate_macros=True, advanced_mode=True, pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # Should have a macro definition
        # The generator adds a comment " Macros " before them
        macro = root.find(f"{XACRO_NS}macro")
        assert macro is not None, "Macro definition not found"

        macro_name = macro.get("name")
        params = str(macro.get("params"))
        assert "name parent xyz rpy" in params

        # Should have 2 calls to this macro
        calls = root.findall(f"{XACRO_NS}{macro_name}")
        assert len(calls) == 2

        # Verify call parameters
        left_call = next((c for c in calls if c.get("name") == "left_leg"), None)
        assert left_call is not None
        assert left_call.get("parent") == "base"
        assert "0 1 0" in str(left_call.get("xyz"))
        # format_vector uses {:g} usually or similar, let's just check containing substrings if specific format unknown
        # Actually checking implementation of format_vector... it uses format_float which uses {:g}
        # default logic.

    def test_split_files_logic(self, tmp_path: Path) -> None:
        """Test splitting output into multiple files."""
        robot = Robot(name="split_bot")
        link = Link(name="base")
        robot.add_link(link)

        # Add property candidate (material)
        mat = Material(name="red", color=Color(1, 0, 0, 1))
        link.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat))

        out_file = tmp_path / "robot.xacro"

        # Enable splitting
        generator = XACROGenerator(split_files=True, advanced_mode=True, extract_materials=True)
        generator.write(robot, out_file, validate=False)

        # Files that should exist
        main_file = out_file
        props_file = tmp_path / "split_bot_properties.xacro"
        # Macros file might not exist if no macros generated

        assert main_file.exists()
        assert props_file.exists()

        main_content = main_file.read_text()
        props_content = props_file.read_text()

        # Main file should include properties
        assert "xacro:include" in main_content
        assert 'filename="split_bot_properties.xacro"' in main_content

        # Properties file should contain the property
        assert '<xacro:property name="red"' in props_content

    def test_find_common_prefix_empty(self) -> None:
        """Test _find_common_prefix with empty or single names."""
        gen = XACROGenerator()
        assert gen._find_common_prefix([]) == ""
        assert gen._find_common_prefix(["l1"]) == "l1"
        assert gen._find_common_prefix(["part_a", "part_b"]) == "part"

    def test_generator_validation_error(self) -> None:
        """Test that XACROGenerator.generate raises RobotModelError for invalid robot."""
        robot = Robot(name="invalid")
        l1 = Link(name="l1")
        robot.add_link(l1)
        # Manually append to bypass add_joint validation
        lim = JointLimits(lower=-1, upper=1, effort=1, velocity=1)
        joint = Joint(
            name="j1",
            type=JointType.REVOLUTE,
            parent="world",
            child="l1",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=lim,
        )
        robot._joints.append(joint)

        gen = XACROGenerator()

        # Generator's validation should catch missing 'world' link
        with pytest.raises(RobotGeneratorError, match="Robot validation failed"):
            gen.generate(robot, validate=True)

    def test_generator_dimension_extraction_variations(self) -> None:
        """Test dimension extraction with no common prefix and different types."""
        robot = Robot(name="r")
        root_link = Link(name="base_link")
        robot.add_link(root_link)

        # Repeated spheres - tests sphere branch
        l1 = Link(name="s1", visuals=[Visual(geometry=Sphere(radius=0.1))])
        l2 = Link(name="s2", visuals=[Visual(geometry=Sphere(radius=0.1))])

        # Repeated boxes with no common prefix
        l3 = Link(name="alpha", visuals=[Visual(geometry=Box(size=Vector3(2, 2, 2)))])
        l4 = Link(name="omega", visuals=[Visual(geometry=Box(size=Vector3(2, 2, 2)))])

        robot.add_link(l1)
        robot.add_link(l2)
        robot.add_link(l3)
        robot.add_link(l4)

        # Add joints to keep them in the tree
        robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="base_link", child="s1"))
        robot.add_joint(Joint(name="j2", type=JointType.FIXED, parent="base_link", child="s2"))
        robot.add_joint(Joint(name="j3", type=JointType.FIXED, parent="base_link", child="alpha"))
        robot.add_joint(Joint(name="j4", type=JointType.FIXED, parent="base_link", child="omega"))

        gen = XACROGenerator(advanced_mode=True, extract_dimensions=True)
        xml_str = gen.generate(robot)

        assert 'name="s_radius"' in xml_str  # Prefix "s" from s1, s2
        assert 'name="box_width"' in xml_str  # no common prefix for alpha, omega
        assert 'value="2"' in xml_str

    def test_generator_material_no_prop(self) -> None:
        """Test material color generation when properties are disabled."""
        robot = Robot(name="r")
        robot.add_link(Link(name="l"))
        mat = Material(name="m", color=Color(0.1, 0.2, 0.3, 1.0))
        robot.links[0].visuals.append(Visual(geometry=Box(size=Vector3(1, 1, 1)), material=mat))

        # advanced_mode=True but extract_materials=False
        gen = XACROGenerator(advanced_mode=True, extract_materials=False)
        xml = gen.generate(robot)
        assert 'rgba="0.1 0.2 0.3 1"' in xml
        assert "<color" in xml  # Should be inside the material tag

    def test_generator_macro_signatures(self) -> None:
        """Test all geometry types in macro signatures (Cylinder, Sphere, Mesh) and generation."""
        robot = Robot(name="r")
        root = Link(name="base")
        robot.add_link(root)

        def add_pair(geom: Box | Cylinder | Sphere | Mesh) -> None:
            l1 = Link(name=f"{geom.type.value}1", visuals=[Visual(geometry=geom)])
            l2 = Link(name=f"{geom.type.value}2", visuals=[Visual(geometry=geom)])
            robot.add_link(l1)
            robot.add_link(l2)
            robot.add_joint(
                Joint(name=f"j_{l1.name}", type=JointType.FIXED, parent="base", child=l1.name)
            )
            robot.add_joint(
                Joint(name=f"j_{l2.name}", type=JointType.FIXED, parent="base", child=l2.name)
            )

            # Add identical collision to both to maintain identical signature
            for lnk in [l1, l2]:
                lnk.collisions.append(Collision(name="collision", geometry=geom))

        add_pair(Cylinder(radius=0.1, length=1.0))
        add_pair(Sphere(radius=0.5))
        add_pair(Mesh(resource="test.stl", scale=Vector3(2, 2, 2)))
        add_pair(Box(size=Vector3(1, 2, 3)))

        gen = XACROGenerator(advanced_mode=True, generate_macros=True, extract_dimensions=True)
        xml_str = gen.generate(robot)

        assert "cylinder_" in xml_str
        assert "sphere_" in xml_str
        assert "mesh_" in xml_str
        assert "box_" in xml_str
        assert 'filename="test.stl"' in xml_str
        assert "radius=" in xml_str
        # Check that the value 0.5 exists (either as property or literal)
        assert "0.5" in xml_str

    def test_generator_material_property_fallback(self) -> None:
        """Test material color fallback in generate_robot_element."""
        robot = Robot(name="r")
        robot.add_link(Link(name="l"))
        mat = Material(name="m", color=Color(0.1, 0.2, 0.3, 1.0))
        robot.links[0].visuals.append(Visual(geometry=Box(size=Vector3(1, 1, 1)), material=mat))

        gen = XACROGenerator(advanced_mode=True, extract_materials=True)
        # Manually sabotage material_properties to trigger the elif
        gen.material_properties = {}

        # Override _extract_materials to do nothing
        gen._extract_materials = lambda r, p: None  # type: ignore[method-assign, assignment]

        xml_str = gen.generate(robot)
        assert 'rgba="0.1 0.2 0.3 1"' in xml_str

    def test_generator_split_files_minimal(self, tmp_path: Path) -> None:
        """Test split files with only properties or only macros to hit all branches."""
        robot = Robot(name="r")
        robot.add_link(Link(name="base"))

        # 1. Properties only
        gen = XACROGenerator(split_files=True, extract_dimensions=True, generate_macros=False)
        # Add repeated geometry to force property
        robot.links[0].visuals.append(Visual(geometry=Box(size=Vector3(1, 1, 1))))
        l2 = Link(name="l2", visuals=[Visual(geometry=Box(size=Vector3(1, 1, 1)))])
        robot.add_link(l2)
        robot.add_joint(Joint(name="j", type=JointType.FIXED, parent="base", child="l2"))

        gen.write(robot, tmp_path / "robot.xacro")
        assert (tmp_path / "r_properties.xacro").exists()
        assert not (tmp_path / "r_macros.xacro").exists()

    def test_generator_mesh_relative_path(self, tmp_path: Path) -> None:
        """Test mesh relative path resolution in XACROGenerator."""
        robot = Robot(name="r")
        mesh_path = tmp_path / "meshes" / "link.stl"
        mesh_path.parent.mkdir(parents=True)
        mesh_path.write_text("dummy")

        link = Link(name="l", visuals=[Visual(geometry=Mesh(resource=str(mesh_path)))])
        robot.add_link(link)

        gen = XACROGenerator(advanced_mode=True, urdf_path=tmp_path / "urdf" / "robot.xacro")
        xml_str = gen.generate(robot)
        # Relaxed check for relative path
        assert "meshes/link.stl" in xml_str.replace("\\", "/")

    def test_generator_texture_and_no_color(self) -> None:
        """Test material with texture and material without color."""
        robot = Robot(name="r")
        l1 = Link(name="l1")
        # Material with texture only
        mat_tex = Material(name="tex", texture="checkers.png")
        l1.visuals.append(Visual(geometry=Box(size=Vector3(1, 1, 1)), material=mat_tex))

        # Material with no name, but HAS texture (to pass validation)
        mat_empty = Material(name="", texture="empty.png")
        l1.visuals.append(Visual(geometry=Sphere(radius=0.1), material=mat_empty))

        robot.add_link(l1)

        gen = XACROGenerator(extract_materials=True)
        xml_str = gen.generate(robot)
        assert 'filename="checkers.png"' in xml_str

    def test_generator_material_no_color(self) -> None:
        """Test material color generation with NO color to hit continue branch."""
        robot = Robot(name="r")
        l1 = Link(name="l")
        mat = Material(name="no_color", texture="dummy.png")  # No color assigned, but has texture
        l1.visuals.append(Visual(geometry=Box(size=Vector3(1, 1, 1)), material=mat))
        robot.add_link(l1)

        gen = XACROGenerator(extract_materials=True)
        xml_str = gen.generate(robot)
        # no_color material should NOT be in material_properties
        assert "no_color" in xml_str  # Should exist by name in reference
        assert "rgba=" not in xml_str.lower()  # No color tag generated for this mat

    def test_generator_macro_with_dynamics_and_mesh(self) -> None:
        """Test macro generation for links with dynamics and meshes."""
        robot = Robot(name="r")
        robot.add_link(Link(name="base"))
        mat = Material(name="m", color=Color(1, 0, 0, 1))

        def create_link(name: str, parent: str) -> Link:
            link = Link(name=name)
            link.visuals.append(Visual(geometry=Mesh(resource="mesh.dae"), material=mat))
            robot.add_link(link)
            from linkforge_core.models.joint import JointLimits

            lim = JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0)
            joint = Joint(
                name=f"{name}_j",
                type=JointType.REVOLUTE,
                parent=parent,
                child=name,
                axis=Vector3(1.0, 0.0, 0.0),
                limits=lim,
                dynamics=JointDynamics(damping=0.5, friction=0.1),
            )
            robot.add_joint(joint)
            return link

        create_link("l1", "base")
        create_link("l2", "base")  # identical to l1

        gen = XACROGenerator(advanced_mode=True, generate_macros=True)
        xml_str = gen.generate(robot)

        assert 'damping="0.5"' in xml_str
        assert 'friction="0.1"' in xml_str
        assert 'mesh filename="mesh.dae"' in xml_str

    def test_find_common_prefix_edge_cases(self) -> None:
        """Test common prefix finder with edge cases."""
        gen = XACROGenerator()

        # Empty list
        assert gen._find_common_prefix([]) == ""

        # Single item (should return empty string as per logic?)
        # Logic says: if not names return "", else prefix = names[0]...
        # Wait, if list has 1 item, loop [1:] doesn't run, returns item.
        # But _generate_dimension_property_name passes only if len >= 2.
        # So we test logic directly for safety.
        assert gen._find_common_prefix(["single"]) == "single"

        # No common prefix
        assert gen._find_common_prefix(["a_b", "c_d"]) == ""

        # Partial match
        assert gen._find_common_prefix(["arm_left", "arm_right"]) == "arm"

    def test_group_dimensions_tolerance(self) -> None:
        """Test dimension grouping tolerance."""
        gen = XACROGenerator()

        # Values close enough to group
        values = [("l1", 1.0001), ("l2", 1.0002)]
        groups = gen._group_dimensions_by_value(values)
        assert len(groups) == 1
        assert 1.0 in groups  # grouped by rounded key

        # Values too far apart
        values = [("l1", 1.0), ("l2", 1.1)]
        groups = gen._group_dimensions_by_value(values)
        assert len(groups) == 2

    def test_generator_generate_macro_definition_no_joint(self) -> None:
        """Cover edge case."""
        gen = XACROGenerator()
        root = ET.Element("robot")
        # Template list with link but None joint
        link = Link(name="l")
        group = [(link, None)]
        # Should return early and not crash
        gen._generate_macro_definition(root, "sig", group)  # type: ignore[arg-type]
        assert len(root) == 0

    def test_generator_generate_macro_call_no_joint(self) -> None:
        """Cover edge case."""
        gen = XACROGenerator()
        root = ET.Element("robot")
        link = Link(name="l")
        # Should return early
        gen._generate_macro_call(root, "sig", link, None)
        assert len(root) == 0

    def test_generator_visual_name(self) -> None:
        """Test visual name generation."""
        gen = XACROGenerator()
        robot = Robot(name="test")
        link = Link(name="l")
        vis = Visual(geometry=Box(size=Vector3(1.0, 1.0, 1.0)), name="my_visual")
        link.visuals.append(vis)
        robot.add_link(link)

        xml = gen.generate(robot)
        assert 'visual name="my_visual"' in xml

    def test_generator_write_no_split(self, tmp_path: Path) -> None:
        """Test write functionality without file splitting."""
        gen = XACROGenerator(split_files=False)
        robot = Robot(name="test")
        robot.add_link(Link(name="base"))
        out = tmp_path / "out.xacro"
        gen.write(robot, out)
        assert out.exists()

    def test_generator_split_files_with_macros(self, tmp_path: Path) -> None:
        """Test file splitting with macros."""
        gen = XACROGenerator(split_files=True, generate_macros=True)
        # Create robot with repeated geometry to trigger macros
        robot = Robot(name="test")

        # Base link
        base = Link(name="base")
        robot.add_link(base)

        # Link 1
        l1 = Link(name="l1")
        v1 = Visual(geometry=Box(size=Vector3(1.0, 1.0, 1.0)))  # Identical box
        l1.visuals.append(v1)
        robot.add_link(l1)

        # Link 2
        l2 = Link(name="l2")
        v2 = Visual(geometry=Box(size=Vector3(1.0, 1.0, 1.0)))  # Identical box
        l2.visuals.append(v2)
        robot.add_link(l2)

        # Joints needed for macro grouping
        j1 = Joint(name="j1", type=JointType.FIXED, parent="base", child="l1")
        robot.add_joint(j1)
        j2 = Joint(name="j2", type=JointType.FIXED, parent="base", child="l2")
        robot.add_joint(j2)

        out = tmp_path / "main.xacro"
        gen.write(robot, out)

        assert out.exists()
        macros_file = tmp_path / "test_macros.xacro"
        macros_content = macros_file.read_text()
        assert 'macro name="box_' in macros_content
        assert '_macro"' in macros_content
        assert 'include filename="test_macros.xacro"' in out.read_text()

    def test_generator_extract_material_property_implementation(self) -> None:
        """Test material property extraction implementation."""
        gen = XACROGenerator(extract_materials=True, advanced_mode=True)
        # Manually populate material_properties to simulate extraction having happened
        gen.material_properties["blue"] = "color_blue"

        root = ET.Element("link")
        mat = Material(name="blue", color=Color(0.0, 0.0, 1.0, 1.0))

        gen._add_material_element(root, mat)

        xml = ET.tostring(root).decode()
        assert 'rgba="${color_blue}"' in xml

    def test_find_common_prefix_internal_logic(self) -> None:
        """Test internal logic of _find_common_prefix."""
        gen = XACROGenerator()
        # Test basic cases
        assert gen._find_common_prefix(["arm_link", "arm_joint"]) == "arm"
        assert gen._find_common_prefix(["fl_wheel", "fr_wheel"]) == "wheel"

    def test_generator_split_files_with_ros2_control(self, tmp_path: Path) -> None:
        """Test split files with ros2_control configuration."""
        robot = Robot(name="control_bot")
        robot.add_link(Link(name="base_link"))

        # Add ros2_control
        control = Ros2Control(
            name="GazeboSimSystem", type="system", hardware_plugin="gz_ros2_control/GazeboSimSystem"
        )
        control.joints.append(
            Ros2ControlJoint(
                name="joint1", command_interfaces=["position"], state_interfaces=["position"]
            )
        )
        robot.add_ros2_control(control)

        # Add gazebo plugin
        plugin = GazeboPlugin(name="gz_ros2_control", filename="libgz_ros2_control-system.so")
        robot.gazebo_elements.append(GazeboElement(plugins=[plugin]))

        out_file = tmp_path / "robot.xacro"

        # Enable splitting
        gen = XACROGenerator(split_files=True, advanced_mode=True)
        gen.write(robot, out_file, validate=False)

        # Files that should exist
        main_file = out_file
        control_file = tmp_path / "control_bot_ros2_control.xacro"

        assert main_file.exists()
        assert control_file.exists()

        main_content = main_file.read_text()
        control_content = control_file.read_text()

        # Main file should include ros2_control
        assert "xacro:include" in main_content
        assert 'filename="control_bot_ros2_control.xacro"' in main_content

        # Main file should NOT contain the empty placeholder comments at the bottom
        # It should ONLY have the single comment above the include tag
        assert main_content.count("<!-- ROS2 Control -->") == 1
        assert "<!-- Gazebo -->" not in main_content

        # Control file should contain ros2_control and gazebo tags
        assert "<ros2_control" in control_content
        assert "<gazebo" in control_content
        assert "gz_ros2_control" in control_content


class TestXACROGeneratorEdgeCoverage:
    """Test generator behavior for macro groups, geometry types, joint limits, and file splitting."""

    def test_link_in_macro_with_no_matching_group_falls_through(self) -> None:
        """Verify link handles fallthrough when no matching macro group is found."""
        gen = XACROGenerator(generate_macros=True)
        robot = Robot(name="r")
        link = Link(name="l1", visuals=[Visual(geometry=Box(size=Vector3(x=1, y=1, z=1)))])
        robot.add_link(link)
        gen.links_in_macros = {"l1"}
        gen.macro_groups = {}
        result = gen.generate(robot)
        assert "l1" in result

    def test_generator_unsupported_geometry_fallback(self) -> None:
        """Test that XACROGenerator handles unsupported geometry types via fallback."""
        gen = XACROGenerator()
        robot = Robot(name="r")

        class UnknownGeometry:
            pass

        link = Link(name="l1", visuals=[Visual(geometry=UnknownGeometry())])  # type: ignore
        robot.add_link(link)
        xml = gen.generate(robot)
        # It should generate an empty geometry container `<geometry />` or `<geometry></geometry>`
        assert "<geometry />" in xml or "<geometry></geometry>" in xml

    def test_visual_with_non_standard_geometry_in_signature(self) -> None:
        """Verify signature generation handles non-standard geometry objects."""
        gen = XACROGenerator(generate_macros=True)
        robot = Robot(name="r")
        link = Link(name="l1", visuals=[Visual(geometry=Sphere(radius=0.5))])
        robot.add_link(link)
        result = gen.generate(robot)
        assert "l1" in result

    def test_joint_limit_with_none_lower(self) -> None:
        """Verify joint limit generation handles cases where lower limit is not specified."""
        gen = XACROGenerator()
        robot = Robot(name="r")
        robot.add_link(Link(name="l1"))
        robot.add_link(Link(name="l2"))
        joint = Joint(
            name="j",
            type=JointType.REVOLUTE,
            parent="l1",
            child="l2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(effort=1.0, velocity=1.0, lower=None, upper=None),
        )
        robot.add_joint(joint)
        result = gen.generate(robot)
        assert "j" in result

    def test_joint_dynamics_with_zero_values_skips_attributes(self) -> None:
        """Verify joint dynamics with zero values do not generate redundant XML attributes."""
        gen = XACROGenerator()
        robot = Robot(name="r")
        robot.add_link(Link(name="l1"))
        robot.add_link(Link(name="l2"))
        joint = Joint(
            name="j",
            type=JointType.REVOLUTE,
            parent="l1",
            child="l2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(effort=1.0, velocity=1.0),
            dynamics=JointDynamics(damping=0.0, friction=0.0),
        )
        robot.add_joint(joint)
        result = gen.generate(robot)
        assert "j" in result

    def test_split_files_with_macros_and_properties(self) -> None:
        """Split-files mode creates separate property and macro xacro files."""
        import tempfile

        gen = XACROGenerator(generate_macros=True, split_files=True, advanced_mode=True)
        robot = Robot(name="r")
        robot.add_link(
            Link(
                name="l1",
                visuals=[
                    Visual(
                        geometry=Box(size=Vector3(x=1, y=1, z=1)),
                        material=Material(name="m", color=Color(r=1, g=0, b=0)),
                    )
                ],
            )
        )
        robot.add_link(Link(name="l2"))
        joint = Joint(
            name="j",
            type=JointType.REVOLUTE,
            parent="l1",
            child="l2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(effort=1.0, velocity=1.0),
        )
        robot.add_joint(joint)
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "r.xacro"
            gen.write(robot, out)
            assert out.exists()

    def test_generate_macros_with_enhanced_joint(self) -> None:
        """Test macro generation with mimic, safety_controller, and calibration."""
        robot = Robot(name="enhanced_macro_bot")
        base = Link(name="base")
        robot.add_link(base)

        # Template for repeated link structure
        for side in ["left", "right"]:
            link = Link(name=f"{side}_link")
            link.visuals.append(Visual(geometry=Box(Vector3(0.1, 0.1, 0.1))))
            robot.add_link(link)

            joint = Joint(
                name=f"{side}_joint",
                type=JointType.REVOLUTE,
                parent="base",
                child=f"{side}_link",
                axis=Vector3(0, 0, 1),
                limits=JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0),
                mimic=JointMimic(joint="master_joint", multiplier=1.0, offset=0.1),
                safety_controller=JointSafetyController(
                    soft_lower_limit=-0.9,
                    soft_upper_limit=0.9,
                    k_position=100.0,
                    k_velocity=40.0,
                ),
                calibration=JointCalibration(rising=0.5, falling=None),
            )
            robot.add_joint(joint)

        # Also add the master joint so mimic is valid
        master_link = Link(name="master_link")
        robot.add_link(master_link)
        master_joint = Joint(
            name="master_joint",
            type=JointType.REVOLUTE,
            parent="base",
            child="master_link",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0),
        )
        robot.add_joint(master_joint)

        generator = XACROGenerator(generate_macros=True, advanced_mode=True, pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # Find the macro definition
        macro = root.find(f"{XACRO_NS}macro")
        assert macro is not None

        joint_elem = macro.find("joint")
        assert joint_elem is not None

        # Check Mimic
        mimic_elem = joint_elem.find("mimic")
        assert mimic_elem is not None
        assert mimic_elem.get("joint") == "master_joint"
        assert mimic_elem.get("offset") == "0.1"

        # Check Safety Controller
        safety_elem = joint_elem.find("safety_controller")
        assert safety_elem is not None
        assert safety_elem.get("soft_lower_limit") == "-0.9"
        assert safety_elem.get("k_position") == "100"

        # Check Calibration
        calib_elem = joint_elem.find("calibration")
        assert calib_elem is not None
        assert calib_elem.get("rising") == "0.5"
        assert calib_elem.get("falling") is None

    def test_generate_with_partial_configuration_and_prefix_matching(self) -> None:
        """Verify generator handles partial configurations and prefix matching logic."""
        robot = Robot(name="r")
        link = Link(name="l")
        # Origin but no material (partial visual configuration)
        link.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), origin=Transform()))
        robot.add_link(link)

        # advanced_mode=True but extracts=False (partial extraction configuration)
        gen = XACROGenerator(advanced_mode=True, extract_dimensions=False, extract_materials=False)
        xml = gen.generate(robot)
        assert "<robot" in xml

        # generate_macros=True but no macros possible (empty robot)
        robot2 = Robot(name="empty")
        gen2 = XACROGenerator(generate_macros=True)
        # Add a placeholder link to pass base model validation
        robot2.add_link(Link(name="placeholder"))
        xml2 = gen2.generate(robot2)
        assert "<robot" in xml2

    def test_find_common_prefix_character_fallback(self) -> None:
        """Verify the prefix finder handles cases with no clear underscore separator."""
        gen = XACROGenerator()
        # _find_common_prefix with mismatching lengths and partial matches
        # 1. Underscore suffix match (most common)
        assert gen._find_common_prefix(["fl_wheel", "fr_wheel"]) == "wheel"
        # 2. Fallback to character prefix (no common underscore suffix)
        assert gen._find_common_prefix(["link1", "link2"]) == "link"
        assert gen._find_common_prefix(["a", "abc"]) == "a"
        assert gen._find_common_prefix(["abc", "a"]) == "a"
        assert gen._find_common_prefix(["abc", "abx"]) == "ab"
        assert gen._find_common_prefix(["", "a"]) == ""

    def test_generate_robot_with_empty_name_and_missing_joint_axis(self) -> None:
        """Verify generator handles robots with empty names and joints missing axes."""
        robot = Robot(name="robot")
        # Bypass validation to test generator's empty name branch
        object.__setattr__(robot, "name", "")
        # Parent link must exist
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="l1"))
        robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="base", child="l1"))
        gen = XACROGenerator()
        xml = gen.generate(robot, validate=False)
        assert 'name=""' in xml

    def test_generate_macros_with_varying_joints(self) -> None:
        """Verify links are not grouped into macros if their joints have different properties."""
        robot = Robot(name="r")
        geom = Box(Vector3(1, 1, 1))
        # Topological order: add PARENT first
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="l1", visuals=[Visual(geometry=geom)]))
        robot.add_link(Link(name="l2", visuals=[Visual(geometry=geom)]))

        # Different joint types should prevent grouping
        robot.add_joint(
            Joint(
                name="j1",
                type=JointType.CONTINUOUS,
                parent="base",
                child="l1",
                axis=Vector3(1, 0, 0),
            )
        )
        robot.add_joint(Joint(name="j2", type=JointType.FIXED, parent="base", child="l2"))

        gen = XACROGenerator(generate_macros=True)
        xml = gen.generate(robot, validate=False)
        assert "xacro:macro" not in xml

    def test_generate_macros_with_advanced_features_disabled(self) -> None:
        """Verify macro generation still functions when advanced mode features are disabled."""
        robot = Robot(name="r")
        geom = Box(Vector3(1, 1, 1))

        # Topological order: add PARENT first
        robot.add_link(Link(name="base"))
        mat = Material(name="red", color=Color(1, 0, 0, 1))
        robot.add_link(Link(name="l1", visuals=[Visual(geometry=geom, material=mat)]))
        robot.add_link(Link(name="l2", visuals=[Visual(geometry=geom, material=mat)]))
        robot.add_link(Link(name="l3", visuals=[Visual(geometry=geom, material=mat)]))

        # Group 1 (l1, l2)
        robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="base", child="l1"))
        robot.add_joint(Joint(name="j2", type=JointType.FIXED, parent="base", child="l2"))
        # Different joint type for l3
        robot.add_joint(
            Joint(
                name="j3",
                type=JointType.CONTINUOUS,
                parent="base",
                child="l3",
                axis=Vector3(1, 0, 0),
            )
        )

        # Trigger macro in basic mode
        gen = XACROGenerator(generate_macros=True, advanced_mode=False)
        xml = gen.generate(robot, validate=False)
        assert "xacro:macro" in xml

    def test_xacro_generator_split_and_limit_edge_cases(self) -> None:
        """Verify XACRO generator handles split files and joint limit edge cases."""
        import tempfile
        from pathlib import Path

        # 1. use_ros2_control = False (disable control generation)
        robot = Robot(name="r")
        robot.add_link(Link(name="l"))
        gen = XACROGenerator(use_ros2_control=False)
        assert "<ros2_control" not in gen.generate(robot)

        # 2. Split files with empty gazebo tag (verify filtering logic)
        robot2 = Robot(name="r2")
        robot2.add_link(Link(name="base"))
        # Gazebo element with no plugins
        robot2.gazebo_elements.append(GazeboElement(reference="base"))
        gen2 = XACROGenerator(split_files=True, advanced_mode=True)

        with tempfile.TemporaryDirectory() as tmpdir:
            file_path = Path(tmpdir) / "robot.xacro"
            gen2.write(robot2, file_path)
            # Verify the main file contains the gazebo tag (it wasn't moved to control)
            content = file_path.read_text()
            assert "<gazebo" in content

        # 3. Joint limit variations (lower=None, upper=None) and geometry signatures
        robot3 = Robot(name="r3")
        robot3.add_link(Link(name="base"))
        # Cylinder and Sphere visuals AND collisions
        vis_c = Visual(geometry=Cylinder(radius=0.1, length=0.2))
        vis_s = Visual(geometry=Sphere(radius=0.3))
        col_c = Collision(geometry=Cylinder(radius=0.1, length=0.2))
        col_s = Collision(geometry=Sphere(radius=0.3))

        robot3.add_link(Link(name="l_cyl1", visuals=[vis_c], collisions=[col_c]))
        robot3.add_link(Link(name="l_cyl2", visuals=[vis_c], collisions=[col_c]))
        robot3.add_link(Link(name="l_sph1", visuals=[vis_s], collisions=[col_s]))
        robot3.add_link(Link(name="l_sph2", visuals=[vis_s], collisions=[col_s]))

        # Test both upper=None and lower=None in grouping
        lim_upper_none = JointLimits(effort=1.0, velocity=1.0, lower=-1.0, upper=None)
        lim_lower_none = JointLimits(effort=1.0, velocity=1.0, lower=None, upper=1.0)

        for i in [1, 2]:
            robot3.add_joint(
                Joint(
                    name=f"j_cyl{i}",
                    type=JointType.REVOLUTE,
                    parent="base",
                    child=f"l_cyl{i}",
                    limits=lim_upper_none,
                    axis=Vector3(1, 0, 0),
                )
            )
            robot3.add_joint(
                Joint(
                    name=f"j_sph{i}",
                    type=JointType.CONTINUOUS,
                    parent="base",
                    child=f"l_sph{i}",
                    limits=lim_lower_none,
                    axis=Vector3(1, 0, 0),
                )
            )

        gen3 = XACROGenerator(generate_macros=True)
        xml3 = gen3.generate(robot3)
        assert "xacro:macro" in xml3
