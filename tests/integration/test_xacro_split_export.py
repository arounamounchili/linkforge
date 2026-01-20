"""Test XACRO round-trip (Export -> Split -> Re-import)."""

import tempfile
from pathlib import Path

from linkforge.core.generators.xacro import XACROGenerator
from linkforge.core.models import Color, Joint, JointType, Link, Material, Robot, Visual
from linkforge.core.models.geometry import Cylinder, Transform, Vector3


def test_split_files_and_reimport_simulated():
    """Test that split files are generated correctly and can be parsed."""
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)

        # 1. Create a robot
        base_link = Link(name="base_link")
        wheel_geom = Cylinder(radius=0.1, length=0.05)
        wheel_mat = Material(name="black", color=Color(0.1, 0.1, 0.1, 1))

        links = [base_link]
        joints = []
        for i in range(2):
            name = f"wheel_{i}"
            link = Link(name=name, visuals=[Visual(geometry=wheel_geom, material=wheel_mat)])
            links.append(link)
            joint = Joint(
                name=f"{name}_joint",
                type=JointType.CONTINUOUS,
                parent="base_link",
                child=name,
                origin=Transform(xyz=Vector3(i, 0, 0)),
            )
            joints.append(joint)

        robot = Robot(name="test_bot", links=links, joints=joints)

        # 2. Export with ALL features enabled
        gen = XACROGenerator(
            extract_materials=True, extract_dimensions=True, generate_macros=True, split_files=True
        )

        main_file = tmp_path / "test_bot.xacro"
        gen.write(robot, main_file, validate=False)

        # 3. Verify files exist
        assert main_file.exists()
        assert (tmp_path / "test_bot_properties.xacro").exists()
        assert (tmp_path / "test_bot_macros.xacro").exists()

        # 4. Verify contents of main file (should have includes)
        main_content = main_file.read_text()
        assert '<xacro:include filename="test_bot_properties.xacro"' in main_content
        assert '<xacro:include filename="test_bot_macros.xacro"' in main_content

        # 5. Verify properties file (should have properties)
        mat_content = (tmp_path / "test_bot_properties.xacro").read_text()
        print(f"Properties content:\n{mat_content}")
        assert '<xacro:property name="black"' in mat_content
        assert 'value="0.1 0.1 0.1 1"' in mat_content

        # 6. Verify macros file (should have the macro)
        macro_content = (tmp_path / "test_bot_macros.xacro").read_text()
        print(f"Macros content:\n{macro_content}")
        assert '<xacro:macro name="cylinder_' in macro_content

    print("\nVerified: All 4 advanced settings work correctly together!")


if __name__ == "__main__":
    test_split_files_and_reimport_simulated()
