"""Unit tests for XACRO generator."""

import xml.etree.ElementTree as ET

import pytest
from linkforge_core.generators.xacro_generator import XACRO_NS, XACROGenerator
from linkforge_core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointDynamics,
    JointLimits,
    JointType,
    Link,
    Material,
    Mesh,
    Robot,
    Sphere,
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

    def test_find_common_prefix_empty(self):
        """Test _find_common_prefix with empty or single names."""
        gen = XACROGenerator()
        assert gen._find_common_prefix([]) == ""
        assert gen._find_common_prefix(["l1"]) == "l1"
        assert gen._find_common_prefix(["part_a", "part_b"]) == "part"

    def test_generator_validation_error(self):
        """Test that XACROGenerator.generate raises ValueError for invalid robot."""
        robot = Robot(name="invalid")
        l1 = Link(name="l1")
        robot.add_link(l1)
        # Manually append to bypass add_joint validation
        lim = JointLimits(lower=-1, upper=1, effort=1, velocity=1)
        joint = Joint(name="j1", type=JointType.REVOLUTE, parent="world", child="l1", limits=lim)
        robot._joints.append(joint)

        gen = XACROGenerator()
        # Generator's validation should catch missing 'world' link
        with pytest.raises(ValueError, match="Robot validation failed"):
            gen.generate(robot, validate=True)

    def test_generator_dimension_extraction_variations(self):
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

    def test_generator_material_no_prop(self):
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

    def test_generator_macro_signatures(self):
        """Test all geometry types in macro signatures (Cylinder, Sphere, Mesh) and generation."""
        robot = Robot(name="r")
        root = Link(name="base")
        robot.add_link(root)

        def add_pair(geom):
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
        add_pair(Mesh(filepath="model://test.stl", scale=Vector3(2, 2, 2)))
        add_pair(Box(size=Vector3(1, 2, 3)))

        gen = XACROGenerator(advanced_mode=True, generate_macros=True, extract_dimensions=True)
        xml_str = gen.generate(robot)

        assert "cylinder_" in xml_str
        assert "sphere_" in xml_str
        assert "mesh_" in xml_str
        assert "box_" in xml_str
        assert 'filename="model://test.stl"' in xml_str
        assert "radius=" in xml_str
        # Check that the value 0.5 exists (either as property or literal)
        assert "0.5" in xml_str

    def test_generator_material_property_fallback(self):
        """Test material color fallback in generate_robot_element."""
        robot = Robot(name="r")
        robot.add_link(Link(name="l"))
        mat = Material(name="m", color=Color(0.1, 0.2, 0.3, 1.0))
        robot.links[0].visuals.append(Visual(geometry=Box(size=Vector3(1, 1, 1)), material=mat))

        gen = XACROGenerator(advanced_mode=True, extract_materials=True)
        # Manually sabotage material_properties to trigger the elif
        gen.material_properties = {}

        # Override _extract_materials to do nothing
        gen._extract_materials = lambda r, p: None

        xml_str = gen.generate(robot)
        assert 'rgba="0.1 0.2 0.3 1"' in xml_str

    def test_generator_split_files_minimal(self, tmp_path):
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

    def test_generator_mesh_relative_path(self, tmp_path):
        """Test mesh relative path resolution in XACROGenerator."""
        robot = Robot(name="r")
        mesh_path = tmp_path / "meshes" / "link.stl"
        mesh_path.parent.mkdir(parents=True)
        mesh_path.write_text("dummy")

        link = Link(name="l", visuals=[Visual(geometry=Mesh(filepath=mesh_path))])
        robot.add_link(link)

        gen = XACROGenerator(advanced_mode=True, urdf_path=tmp_path / "urdf" / "robot.xacro")
        xml_str = gen.generate(robot)
        # Relaxed check for relative path
        assert "meshes/link.stl" in xml_str.replace("\\", "/")

    def test_generator_texture_and_no_color(self):
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

    def test_generator_material_no_color(self):
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

    def test_generator_macro_with_dynamics_and_mesh(self):
        """Test macro generation for links with dynamics and meshes."""
        robot = Robot(name="r")
        robot.add_link(Link(name="base"))
        mat = Material(name="m", color=Color(1, 0, 0, 1))

        def create_link(name, parent):
            link = Link(name=name)
            link.visuals.append(Visual(geometry=Mesh(filepath="model://mesh.dae"), material=mat))
            robot.add_link(link)
            from linkforge_core.models.joint import JointLimits

            lim = JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0)
            joint = Joint(
                name=f"{name}_j", type=JointType.REVOLUTE, parent=parent, child=name, limits=lim
            )
            # dynamics can be assigned because it's None by default or we can pass it
            object.__setattr__(joint, "dynamics", JointDynamics(damping=0.5, friction=0.1))
            robot.add_joint(joint)
            return link

        create_link("l1", "base")
        create_link("l2", "base")  # identical to l1

        gen = XACROGenerator(advanced_mode=True, generate_macros=True)
        xml_str = gen.generate(robot)

        assert 'damping="0.5"' in xml_str
        assert 'friction="0.1"' in xml_str
        assert 'mesh filename="model://mesh.dae"' in xml_str

    def test_find_common_prefix_edge_cases(self):
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

    def test_group_dimensions_tolerance(self):
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

    def test_generator_generate_macro_definition_no_joint(self):
        """Cover line 534 in xacro_generator.py."""
        gen = XACROGenerator()
        root = ET.Element("robot")
        # Template list with link but None joint
        link = Link(name="l")
        group = [(link, None)]  # type: ignore
        # Should return early and not crash
        gen._generate_macro_definition(root, "sig", group)
        assert len(root) == 0

    def test_generator_generate_macro_call_no_joint(self):
        """Cover line 611 in xacro_generator.py."""
        gen = XACROGenerator()
        root = ET.Element("robot")
        link = Link(name="l")
        # Should return early
        gen._generate_macro_call(root, "sig", link, None)
        assert len(root) == 0

    def test_generator_visual_name(self):
        """Cover line 635 in xacro_generator.py."""
        gen = XACROGenerator()
        robot = Robot(name="test")
        link = Link(name="l")
        vis = Visual(geometry=Box(size=Vector3(1.0, 1.0, 1.0)), name="my_visual")
        link.visuals.append(vis)
        robot.add_link(link)

        xml = gen.generate(robot)
        assert 'visual name="my_visual"' in xml

    def test_generator_write_no_split(self, tmp_path):
        """Cover line 759 in xacro_generator.py."""
        gen = XACROGenerator(split_files=False)
        robot = Robot(name="test")
        robot.add_link(Link(name="base"))
        out = tmp_path / "out.xacro"
        gen.write(robot, out)
        assert out.exists()

    def test_generator_split_files_with_macros(self, tmp_path):
        """Cover lines 790-841 in xacro_generator.py."""
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

    def test_generator_extract_material_property_implementation(self):
        """Cover lines 670-671 in xacro_generator.py - Implementation."""
        gen = XACROGenerator(extract_materials=True, advanced_mode=True)
        # Manually populate material_properties to simulate extraction having happened
        gen.material_properties["blue"] = "color_blue"

        root = ET.Element("link")
        mat = Material(name="blue", color=Color(0.0, 0.0, 1.0, 1.0))

        gen._add_material_element(root, mat)

        xml = ET.tostring(root).decode()
        assert 'rgba="${color_blue}"' in xml

    def test_find_common_prefix_internal_logic(self):
        """Ensure _find_common_prefix logic is fully covered."""
        gen = XACROGenerator()
        # Test basic cases
        assert gen._find_common_prefix(["arm_link", "arm_joint"]) == "arm"
        assert gen._find_common_prefix(["fl_wheel", "fr_wheel"]) == "wheel"
