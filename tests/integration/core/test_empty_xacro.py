from linkforge_core.parsers import XACROParser


def test_parse_macro_only_xacro(tmp_path):
    """Verify that a XACRO file with only a macro definition results in an empty robot."""
    xacro_content = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="test_macro" params="name">
        <link name="${name}_link"/>
    </xacro:macro>
</robot>
"""
    xacro_file = tmp_path / "macro_only.xacro"
    xacro_file.write_text(xacro_content)

    parser = XACROParser()
    robot = parser.parse(xacro_file)

    # In this case, the robot name is not specified in the <robot> tag,
    # and the macro is never called, so no links are created.
    assert robot.name == "macro_only"
    assert len(robot.links) == 0
    assert len(robot.joints) == 0


def test_parse_macro_with_name_no_call(tmp_path):
    """Verify that a named robot with only a macro definition results in an empty robot."""
    xacro_content = """<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="test_macro" params="name">
        <link name="${name}_link"/>
    </xacro:macro>
</robot>
"""
    xacro_file = tmp_path / "named_macro.xacro"
    xacro_file.write_text(xacro_content)

    parser = XACROParser()
    robot = parser.parse(xacro_file)

    assert robot.name == "my_robot"
    assert len(robot.links) == 0
