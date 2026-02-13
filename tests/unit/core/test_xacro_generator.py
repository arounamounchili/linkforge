"""Unit tests for XACRO generator."""

import xml.etree.ElementTree as ET

from linkforge_core.generators.xacro_generator import XACRO_NS, XACROGenerator
from linkforge_core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Material,
    Robot,
    Transform,
    Vector3,
    Visual,
)


class TestXACROGenerator:
    """Test XACRO generator features."""

    def test_generate_basic(self):
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

    def test_extract_materials(self):
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
        assert color_elem.get("rgba") == "${brandred}"

    def test_extract_dimensions(self):
        """Test dimension property extraction."""
        robot = Robot(name="dim_bot")

        # 4 sets of identical geometry to trigger extraction
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
        rad_prop = next((p for p in props if "radius" in p.get("name")), None)
        len_prop = next((p for p in props if "length" in p.get("name")), None)

        assert rad_prop is not None
        assert len_prop is not None

        # Check values
        assert "0.3" in rad_prop.get("value")
        assert "0.1" in len_prop.get("value")
        # Heuristic name check
        assert "wheel" in rad_prop.get("name")

        # Check usage in geometry
        # Find a cylinder element
        # Logic: iterate through all links, find visual, find geometry, find cylinder
        # We can just search with xpath
        cyl_elems = root.findall(".//link/visual/geometry/cylinder")
        assert len(cyl_elems) == 4
        assert "${" in cyl_elems[0].get("radius")
        assert "${" in cyl_elems[0].get("length")

        # Check box extraction
        box_props = [p for p in props if "leg" in p.get("name")]
        assert len(box_props) >= 3  # width, depth, height

        box_elems = root.findall(".//link/visual/geometry/box")
        assert len(box_elems) == 2
        assert "${" in box_elems[0].get("size")

    def test_generate_macros(self):
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
        params = macro.get("params")
        assert "name parent xyz rpy" in params

        # Should have 2 calls to this macro
        calls = root.findall(f"{XACRO_NS}{macro_name}")
        assert len(calls) == 2

        # Verify call parameters
        left_call = next((c for c in calls if c.get("name") == "left_leg"), None)
        assert left_call is not None
        assert left_call.get("parent") == "base"
        assert "0 1 0" in left_call.get("xyz")
        # format_vector uses {:g} usually or similar, let's just check containing substrings if specific format unknown
        # Actually checking implementation of format_vector... it uses format_float which uses {:g}
        # default logic.

    def test_split_files_logic(self, tmp_path):
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
