import xml.etree.ElementTree as ET

import yaml
from linkforge_core.parsers.xacro_parser import XacroResolver


def test_xacro_namespaced_load_yaml(tmp_path) -> None:
    # 1. Create a mock package structure
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    (pkg_dir / "package.xml").write_text("<package><name>franka_description</name></package>")

    data_dir = pkg_dir / "robots" / "fp3"
    data_dir.mkdir(parents=True)

    yaml_content = {"mass": 1.23, "inertia": {"ixx": 0.1}}
    yaml_file = data_dir / "inertials.yaml"
    yaml_file.write_text(yaml.dump(yaml_content))

    # 2. Create a XACRO file that uses xacro.load_yaml
    xacro_content = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    <xacro:property name="props" value="${xacro.load_yaml('$(find franka_description)/robots/fp3/inertials.yaml')}"/>
    <link name="base_link">
        <inertial>
            <mass value="${props['mass']}"/>
        </inertial>
    </link>
</robot>
"""
    xacro_file = pkg_dir / "robot.xacro"
    xacro_file.write_text(xacro_content)

    # 3. Resolve
    resolver = XacroResolver(start_dir=pkg_dir)
    urdf_string = resolver.resolve_file(xacro_file)

    # 4. Verify
    root = ET.fromstring(urdf_string)
    mass_elem = root.find(".//mass")
    assert mass_elem is not None
    assert mass_elem.get("value") == "1.23"


def test_unknown_macro_warning(caplog) -> None:
    xacro_content = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    <xacro:unknown_macro params="foo"/>
</robot>
"""
    # Simply resolving should trigger the logger warning
    resolver = XacroResolver()
    # We use a dummy root since we don't want to write a file
    root = ET.fromstring(xacro_content)
    resolver.resolve_element(root)

    assert "Unknown macro or tag: 'unknown_macro'" in caplog.text


def test_xacro_namespaced_load_json(tmp_path) -> None:
    # Create a mock json file
    json_dir = tmp_path / "data"
    json_dir.mkdir()
    json_file = json_dir / "params.json"
    import json

    json_content = {"length": 0.5, "radius": 0.1}
    json_file.write_text(json.dumps(json_content))

    xacro_content = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    <xacro:property name="config" value="${{xacro.load_json('{json_file}')}}"/>
    <link name="link1">
        <visual>
            <geometry>
                <cylinder length="${{config['length']}}" radius="${{config['radius']}}"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
    resolver = XacroResolver()
    root = ET.fromstring(xacro_content)
    resolved_root = resolver.resolve_element(root)

    cyl = resolved_root.find(".//cylinder")
    assert cyl.get("length") == "0.5"
    assert cyl.get("radius") == "0.1"


def test_xacro_nested_property_evaluation() -> None:
    # Test that nested properties derived from load_yaml work in expressions
    xacro_content = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    <xacro:property name="robot_data" value="${dict(arm=dict(length=1.0))}"/>
    <xacro:property name="calculated" value="${robot_data['arm']['length'] * 2}"/>
    <link name="test_link" length="${calculated}"/>
</robot>
"""
    resolver = XacroResolver()
    root = ET.fromstring(xacro_content)
    resolved_root = resolver.resolve_element(root)

    link = resolved_root.find(".//link")
    assert link.get("length") == "2.0"


def test_xacro_circular_include(tmp_path) -> None:
    # A includes B, B includes A
    file_a = tmp_path / "a.xacro"
    file_b = tmp_path / "b.xacro"

    file_a.write_text(f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    <xacro:include filename="{file_b}"/>
</robot>
""")
    file_b.write_text(f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    <xacro:include filename="{file_a}"/>
</robot>
""")

    resolver = XacroResolver()
    import pytest
    from linkforge_core.base import RobotParserError

    with pytest.raises(RobotParserError) as excinfo:
        resolver.resolve_file(file_a)

    assert "XACRO error: Recursion depth exceeded:" in str(excinfo.value)
    assert "a.xacro" in str(excinfo.value)


def test_xacro_recursion_limit() -> None:
    # Macro M1 calls M1
    xacro_content = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    <xacro:macro name="loop">
        <xacro:loop/>
    </xacro:macro>
    <xacro:loop/>
</robot>
"""
    resolver = XacroResolver()
    resolver.max_depth = 10  # Set low for test

    import pytest
    from linkforge_core.base import RobotParserError

    with pytest.raises(RobotParserError) as excinfo:
        root = ET.fromstring(xacro_content)
        resolver.resolve_element(root)

    assert "XACRO error: Recursion depth exceeded: 10" in str(excinfo.value)
    assert "10" in str(excinfo.value)
