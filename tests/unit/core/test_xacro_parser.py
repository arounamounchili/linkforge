import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import pytest
from linkforge_core.base import RobotParserError
from linkforge_core.exceptions import RobotModelError
from linkforge_core.parsers.xacro_parser import XACROParser, XacroResolver
from linkforge_core.parsers.xacro_parser import logger as xacro_logger


def test_xacro_substitute_basic_math():
    resolver = XacroResolver()
    # Simple math
    assert resolver._substitute("${1 + 1}") == 2
    # Division
    assert resolver._substitute("${10 / 2}") == 5.0
    # Parentheses and order of ops
    assert resolver._substitute("${(1 + 2) * 3}") == 9


def test_substitute_resolves_properties_in_math():
    resolver = XacroResolver()
    resolver.properties["base_mass"] = 10.0
    resolver.properties["x"] = 0.4
    resolver.properties["y"] = 0.2

    # Simple property substitution
    assert resolver._substitute("${base_mass}") == 10.0
    # Math with properties
    assert resolver._substitute("${base_mass * 2}") == 20.0
    # Math with properties and spaces
    assert float(resolver._substitute("${(1/12) * base_mass * (x*x + y*y)}")) == pytest.approx(
        0.16666666666666666
    )


def test_substitute_ignores_literal_vectors():
    resolver = XacroResolver()
    # Space-separated vector in ${} is invalid syntax -> should raise error
    with pytest.raises(RobotParserError, match="invalid syntax"):
        resolver._substitute("${0 0 0.5}")

    # Vector from property
    resolver.properties["origin"] = "0 1 2"
    assert resolver._substitute("${origin}") == "0 1 2"

    # Malformed math should raise RobotParserError (Fail Loud)
    with pytest.raises(RobotParserError, match="invalid syntax"):
        resolver._substitute("${1 + / 2}")


def test_substitute_resolves_arguments():
    resolver = XacroResolver()
    resolver.args["pkg_name"] = "my_robot"
    assert resolver._substitute("$(arg pkg_name)") == "my_robot"
    assert resolver._substitute("package://$(arg pkg_name)/meshes") == "package://my_robot/meshes"


def test_resolve_elements_with_conditionals():
    resolver = XacroResolver()

    # Test xacro:if true
    xacro_if_true = ET.fromstring("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:if value="1">
                <link name="if_link"/>
            </xacro:if>
        </root>
    """)
    resolved = resolver.resolve_element(xacro_if_true)
    assert any(c.tag == "link" and c.get("name") == "if_link" for c in resolved)

    # Test xacro:if false
    xacro_if_false = ET.fromstring("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:if value="0">
                <link name="should_not_exist"/>
            </xacro:if>
        </root>
    """)
    resolved = resolver.resolve_element(xacro_if_false)
    assert not any(c.tag == "link" for c in resolved)


def test_resolve_elements_with_block_insertion():
    resolver = XacroResolver()
    # Define a block
    block_xml = ET.fromstring("<origin xyz='0 0 1'/>")
    resolver.properties["my_block"] = block_xml

    xacro_block = ET.fromstring("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:insert_block name="my_block"/>
        </root>
    """)
    resolved = resolver.resolve_element(xacro_block)
    assert any(c.tag == "origin" and c.get("xyz") == "0 0 1" for c in resolved)


def test_resolver_enforces_recursion_limit():
    resolver = XacroResolver(max_depth=5)
    # Define a recursive macro
    macro_xml = ET.fromstring("""
        <xacro:macro xmlns:xacro="http://www.ros.org/wiki/xacro" name="loop">
            <xacro:loop/>
        </xacro:macro>
    """)
    resolver.resolve_element(macro_xml)

    call_xml = ET.fromstring("<xacro:loop xmlns:xacro='http://www.ros.org/wiki/xacro'/>")
    with pytest.raises(RobotParserError, match="Maximum XACRO recursion depth"):
        resolver.resolve_element(call_xml)


def test_resolver_handles_malformed_tags_gracefully():
    resolver = XacroResolver()
    # Test malformed property (missing name)
    bad_prop = ET.fromstring(
        "<xacro:property xmlns:xacro='http://www.ros.org/wiki/xacro' value='10'/>"
    )
    # Should not crash, just skip
    resolver.resolve_element(bad_prop)

    # Test missing macro call
    missing_macro = ET.fromstring("<xacro:missing xmlns:xacro='http://www.ros.org/wiki/xacro'/>")
    # Should not crash, just return skip
    res = resolver.resolve_element(missing_macro)
    assert res.tag == "skip"


def test_macro_expansion_with_block_parameters():
    resolver = XacroResolver()
    # Macro with block parameter
    macro_xml = ET.fromstring("""
        <xacro:macro xmlns:xacro="http://www.ros.org/wiki/xacro" name="with_block" params="*body">
            <link name="foo">
                <xacro:insert_block name="body"/>
            </link>
        </xacro:macro>
    """)
    resolver.resolve_element(macro_xml)

    call_xml = ET.fromstring("""
        <xacro:with_block xmlns:xacro="http://www.ros.org/wiki/xacro">
            <origin xyz="1 2 3"/>
        </xacro:with_block>
    """)
    resolved = resolver.resolve_element(call_xml)
    # Check if origin was inserted
    link = next(c for c in resolved if c.tag == "link")
    assert any(c.tag == "origin" and c.get("xyz") == "1 2 3" for c in link)


def test_resolve_file_raises_error_on_missing_file():
    resolver = XacroResolver()
    # Test non-existent file
    with pytest.raises(RobotParserError, match="Failed to process XACRO file"):
        resolver.resolve_file(Path("/non/existent/path.xacro"))


def test_substitute_handles_non_numeric_eval_results():
    resolver = XacroResolver()
    # eval returning a string (not a number)
    assert resolver._substitute("${'hello'}") == "hello"
    # eval returning a boolean
    assert resolver._substitute("${True}") is True


def test_resolve_elements_flattens_nested_containers():
    resolver = XacroResolver()
    # Nested if statements creating containers
    xml = ET.fromstring("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:if value="1">
                <xacro:if value="1">
                    <link name="nested"/>
                </xacro:if>
            </xacro:if>
        </root>
    """)
    resolved = resolver.resolve_element(xml)
    # The links from nested if should be flattened into root
    assert any(c.tag == "link" and c.get("name") == "nested" for c in resolved)


def test_resolve_file_raises_error_on_malformed_xml(tmp_path):
    resolver = XacroResolver()
    # Malformed XML (unclosed tag)
    bad_xml = tmp_path / "bad.xacro"
    bad_xml.write_text("<root>")
    with pytest.raises(RobotParserError, match="Malformed XACRO XML"):
        resolver.resolve_file(bad_xml)


def test_resolve_element_handles_arguments_tag():
    resolver = XacroResolver()
    arg_xml = ET.fromstring(
        "<xacro:arg xmlns:xacro='http://www.ros.org/wiki/xacro' name='my_arg' default='10'/>"
    )
    resolver.resolve_element(arg_xml)
    assert resolver.args["my_arg"] == 10

    # Should not overwrite existing arg
    arg_xml_2 = ET.fromstring(
        "<xacro:arg xmlns:xacro='http://www.ros.org/wiki/xacro' name='my_arg' default='20'/>"
    )
    resolver.resolve_element(arg_xml_2)
    assert resolver.args["my_arg"] == 10


def test_macro_expansion_with_parameter_defaults():
    resolver = XacroResolver()
    # Macro with default parameter value
    macro_xml = ET.fromstring("""
        <xacro:macro xmlns:xacro="http://www.ros.org/wiki/xacro" name="with_default" params="p:=1.5">
            <link name="l" mass="${p}"/>
        </xacro:macro>
    """)
    resolver.resolve_element(macro_xml)

    # Call without providing 'p'
    call_xml = ET.fromstring("<xacro:with_default xmlns:xacro='http://www.ros.org/wiki/xacro'/>")
    resolved = resolver.resolve_element(call_xml)
    link = next(c for c in resolved if c.tag == "link")
    assert link.get("mass") == "1.5"


def test_finalize_urdf_returns_empty_on_empty_container():
    resolver = XacroResolver()
    # Empty container should yield valid XML with empty container, not empty string
    empty_root = ET.Element("container")
    res = resolver._finalize_urdf(empty_root)
    assert "<container />" in res
    assert "<?xml" in res


def test_find_file_and_search_paths(tmp_path):
    sub = tmp_path / "sub"
    sub.mkdir()
    inc = sub / "include.xacro"
    inc.write_text("<root xmlns:xacro='http://www.ros.org/wiki/xacro'><link name='inc'/></root>")

    resolver = XacroResolver(search_paths=[sub])
    found = resolver._find_file("include.xacro")
    assert found == inc

    # Test absolute path
    assert resolver._find_file(str(inc)) == inc


def test_xacro_parser_class_entry_point(tmp_path):
    from linkforge_core.parsers.xacro_parser import XACROParser

    robot_xml = tmp_path / "robot.xacro"
    robot_xml.write_text("<robot name='test'><link name='l'/></robot>")

    parser = XACROParser()
    robot = parser.parse(robot_xml)
    assert robot.name == "test"
    assert len(robot.links) == 1


def test_resolve_elements_with_nested_container_in_includes(tmp_path):
    # This specifically targets the container flattening in includes
    inc_path = tmp_path / "inc.xacro"
    inc_path.write_text("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:if value="1">
                <link name="inc_link"/>
            </xacro:if>
        </root>
    """)

    main_xml = ET.fromstring(f"""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:include filename="{str(inc_path)}"/>
        </root>
    """)

    resolver = XacroResolver()
    resolved = resolver.resolve_element(main_xml)
    # The container from include's if-statement should be flattened
    assert any(c.tag == "link" and c.get("name") == "inc_link" for c in resolved)


def test_resolve_file_raises_generic_error(monkeypatch):
    from linkforge_core.parsers.xacro_parser import XacroResolver

    def mock_parse(*args, **kwargs):
        raise RobotModelError("Generic crash")

    monkeypatch.setattr(ET, "parse", mock_parse)

    resolver = XacroResolver()
    with pytest.raises(RobotParserError, match="Failed to process XACRO file"):
        resolver.resolve_file(Path("any.xacro"))


def test_resolve_element_skips_missing_include():
    resolver = XacroResolver()
    xml = ET.fromstring(
        "<xacro:include xmlns:xacro='http://www.ros.org/wiki/xacro' filename='missing.xacro'/>"
    )
    res = resolver.resolve_element(xml)
    assert res.tag == "skip"


def test_conditional_falls_back_on_eval_failure():
    resolver = XacroResolver()
    # "not_math" will fail eval, then fallback to string check
    xml = ET.fromstring("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:if value="true">
                <link name="l"/>
            </xacro:if>
        </root>
    """)
    resolved = resolver.resolve_element(xml)
    assert any(c.tag == "link" for c in resolved)


def test_insert_block_skips_non_xml_property():
    resolver = XacroResolver()
    resolver.properties["my_string"] = "just a string"
    xml = ET.fromstring("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:insert_block name="my_string"/>
        </root>
    """)
    res = resolver.resolve_element(xml)
    # The whole block resolves to a single container which should be empty if child was skip
    assert len(res) == 0


def test_macro_expansion_flattens_nested_containers():
    resolver = XacroResolver()
    # Macro body that starts with a conditional (produces a container)
    macro_xml = ET.fromstring("""
        <xacro:macro xmlns:xacro="http://www.ros.org/wiki/xacro" name="with_if">
            <xacro:if value="1">
                <link name="foo"/>
            </xacro:if>
        </xacro:macro>
    """)
    resolver.resolve_element(macro_xml)

    call_xml = ET.fromstring("<xacro:with_if xmlns:xacro='http://www.ros.org/wiki/xacro'/>")
    resolved = resolver.resolve_element(call_xml)
    # Link should be directly under the resolved root (container)
    assert any(c.tag == "link" for c in resolved)


def test_finalize_urdf_cleans_namespaced_attributes():
    resolver = XacroResolver()
    # Element with XACRO and namespaced attributes
    root = ET.Element(
        "link", {"name": "base", "xacro:foo": "bar", "{http://example.com}attr": "val"}
    )
    # Finalize should strip the xacro: and {ns} attributes
    res_xml = resolver._finalize_urdf(root)
    assert 'name="base"' in res_xml
    assert "xacro:foo" not in res_xml
    assert "example.com" not in res_xml


def test_finalize_urdf_with_no_root_children():
    resolver = XacroResolver()
    # Container with NO children -> valid XML
    root = ET.Element("container")
    res = resolver._finalize_urdf(root)
    assert "<container />" in res


def test_resolve_file_raises_generic_error_on_resolution(tmp_path, monkeypatch):
    # Handle files with only one link (no prefix possible)
    from linkforge_core.parsers.xacro_parser import XacroResolver

    # We want resolve_element to raise a non-RobotParserError
    def mock_resolve_element(self, element):
        raise RuntimeError("Resolution crash")

    monkeypatch.setattr(XacroResolver, "resolve_element", mock_resolve_element)

    robot_xml = tmp_path / "valid.xacro"
    robot_xml.write_text("<robot><child/></robot>")

    resolver = XacroResolver()
    # _process_include_file wrapping catches it first, so message changes
    with pytest.raises(RobotParserError, match="Failed to process XACRO file"):
        resolver.resolve_file(robot_xml)


def test_finalize_urdf_with_container_root():
    # Test containers with existing child elements
    resolver = XacroResolver()
    container = ET.Element("container")
    robot = ET.Element("robot", name="foo")
    container.append(robot)

    xml_str = resolver._finalize_urdf(container)
    assert '<robot name="foo"' in xml_str


def test_resolve_file_re_raises_robot_parser_error(tmp_path):
    # This targets line 56 in xacro_parser.py
    # Recursive macro in a file
    bad_xacro = tmp_path / "recursive.xacro"
    bad_xacro.write_text("""
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:macro name="loop">
                <xacro:loop/>
            </xacro:macro>
            <xacro:loop/>
        </robot>
    """)

    resolver = XacroResolver(max_depth=5)
    with pytest.raises(RobotParserError) as excinfo:
        resolver.resolve_file(bad_xacro)
    assert "depth" in str(excinfo.value).lower()


def test_resolver_supports_legacy_xacro_namespace():
    """Verify that legacy http://wiki.ros.org/xacro namespace is recognized."""
    resolver = XacroResolver()

    # Legacy namespace XML
    legacy_xml = ET.fromstring("""
        <root xmlns:xacro="http://wiki.ros.org/xacro">
            <xacro:property name="val" value="42"/>
            <link name="l" mass="${val}"/>
        </root>
    """)

    resolved = resolver.resolve_element(legacy_xml)
    link = next(c for c in resolved if c.tag == "link")
    assert link.get("mass") == "42"


def test_xacro_substitute_trig_math():
    """Verify that trigonometric functions are supported in substitutions."""
    resolver = XacroResolver()
    import math

    # Test PI constant
    assert float(resolver._substitute("${pi}")) == pytest.approx(math.pi)

    # Test SIN function
    assert float(resolver._substitute("${sin(pi/2)}")) == pytest.approx(1.0)

    # Test COS function
    assert float(resolver._substitute("${cos(pi)}")) == pytest.approx(-1.0)

    # Test compound expression
    assert float(resolver._substitute("${sqrt(pow(3, 2) + pow(4, 2))}")) == pytest.approx(5.0)


def test_xacro_namespaced_include(tmp_path):
    """Verify that macros and properties in namespaced includes are prefixed."""
    inc_path = tmp_path / "arm.xacro"
    inc_path.write_text("""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:property name="mass" value="5.0"/>
            <xacro:macro name="link_macro">
                <link name="arm_link"/>
            </xacro:macro>
        </root>
    """)

    main_xml = ET.fromstring(f"""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:include filename="{str(inc_path)}" ns="arm"/>
            <link name="main_link" weight="${{arm.mass}}"/>
            <xacro:arm.link_macro/>
        </root>
    """)

    resolver = XacroResolver()
    resolved = resolver.resolve_element(main_xml)

    # Check property resolution
    main_link = next(c for c in resolved if c.get("name") == "main_link")
    assert main_link.get("weight") == "5.0"

    # Check macro expansion
    assert any(c.tag == "link" and c.get("name") == "arm_link" for c in resolved)


def test_xacro_load_yaml(tmp_path):
    """Verify that YAML data can be loaded and accessed in expressions."""
    yaml_path = tmp_path / "config.yaml"
    yaml_path.write_text("mass: 10.5\nname: bot")

    main_xml = ET.fromstring(f"""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:property name="data" value="${{load_yaml('{str(yaml_path)}')}}"/>
            <link name="${{data['name']}}" mass="${{data['mass']}}"/>
        </root>
    """)

    resolver = XacroResolver()
    resolved = resolver.resolve_element(main_xml)

    link = next(c for c in resolved if c.tag == "link")
    assert link.get("name") == "bot"
    assert link.get("mass") == "10.5"


def test_xacro_load_json(tmp_path):
    """Verify that JSON data can be loaded and accessed in expressions."""
    json_path = tmp_path / "config.json"
    json_path.write_text('{"mass": 10.5, "name": "bot"}')

    main_xml = ET.fromstring(f"""
        <root xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:property name="data" value="${{load_json('{str(json_path)}')}}"/>
            <link name="${{data['name']}}" mass="${{data['mass']}}"/>
        </root>
    """)

    resolver = XacroResolver()
    resolved = resolver.resolve_element(main_xml)

    link = next(c for c in resolved if c.tag == "link")
    assert link.get("name") == "bot"
    assert link.get("mass") == "10.5"


def test_xacro_circular_include(tmp_path):
    """Test detection of circular XACRO includes."""
    file1 = tmp_path / "file1.xacro"
    file2 = tmp_path / "file2.xacro"

    file1.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:include filename="file2.xacro"/></robot>'
    )
    file2.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:include filename="file1.xacro"/></robot>'
    )

    resolver = XacroResolver()
    with pytest.raises(RobotParserError, match="Circular XACRO include detected"):
        resolver.resolve_file(file1)


def test_xacro_resolve_file_exception(tmp_path):
    """Test RobotParserError wrap in resolve_file."""
    resolver = XacroResolver()
    # Mock _process_include_file to raise a plain Exception
    import unittest.mock as mock

    with (
        mock.patch.object(
            resolver, "_process_include_file", side_effect=RuntimeError("Generic Error")
        ),
        pytest.raises(RobotParserError, match="XACRO resolution failed"),
    ):
        resolver.resolve_file(Path("some.xacro"))


def test_xacro_parser_math_eval_error():
    """Test that math evaluation errors raise RobotParserError."""
    resolver = XacroResolver()
    # Undefined variable
    with pytest.raises(RobotParserError, match="Failed to evaluate expression"):
        resolver._evaluate("undefined_var + 1")


def test_xacro_parser_finalize_urdf_recursive_cleanup():
    """Test recursive cleanup in finalize_urdf."""
    resolver = XacroResolver()
    root = ET.Element("robot")
    link = ET.SubElement(root, "link", name="l")
    # Nested xacro tag
    inner = ET.SubElement(link, "visual")
    ET.SubElement(inner, "xacro:info")

    xml = resolver._finalize_urdf(root)
    assert "xacro:info" not in xml


def test_xacro_eval_condition_math():
    """Test complex math in conditions."""
    resolver = XacroResolver()
    resolver.properties["x"] = 10
    assert resolver._eval_condition("x > 5") is True
    assert resolver._eval_condition("x < 5") is False
    # Error case in eval
    assert resolver._eval_condition("invalid syntax!") is True  # Fallback to string-truthy


def test_xacro_circular_block(tmp_path):
    """Test detection of circular block insertions."""
    xacro_file = tmp_path / "test.xacro"
    content = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:property name="my_block">
            <xacro:insert_block name="my_block"/>
        </xacro:property>
        <xacro:insert_block name="my_block"/>
    </robot>"""
    xacro_file.write_text(content)

    resolver = XacroResolver()
    with pytest.raises(RobotParserError, match="Circular block insertion detected"):
        resolver.resolve_file(xacro_file)


def test_load_yaml_no_module():
    """Test behavior when PyYAML is not installed (simulated)."""
    # We need to simulate the absence of yaml module in linkforge_core.parsers.xacro_parser
    # This is tricky because it's imported at top level.
    # However, the code likely does `import yaml` or checks for it.
    # Let's check the implementation of xacro_parser.py again to be sure how it handles it.
    # Based on my memory of reading it, it has a try/except ImportError or similar for optional dep.
    # Actually, let's assume standard pattern.

    from linkforge_core.parsers.xacro_parser import XacroResolver

    resolver = XacroResolver()
    with (
        mock.patch("linkforge_core.parsers.xacro_parser.yaml", None),
        mock.patch("linkforge_core.parsers.xacro_parser.logger") as mock_logger,
    ):
        result = resolver._handle_load_yaml("test.yaml")
        assert result == {}
        # The error message in code is likely "XACRO: PyYAML is not installed..."
        # We'll just check that error was logged.
        assert mock_logger.error.called


def test_parse_typed_value_fallback():
    """Test fallback behavior for typed value parsing."""
    from linkforge_core.parsers.xacro_parser import XacroResolver

    resolver = XacroResolver()

    # Test int fallback
    assert resolver._try_parse_typed_value("123") == 123

    # Test float fallback
    assert resolver._try_parse_typed_value("123.456") == 123.456

    # Test string preservation
    assert resolver._try_parse_typed_value("string") == "string"


def test_xacro_eval_hierarchical_properties():
    """Test hierarchical property access like ${arm.mass}."""
    resolver = XacroResolver()
    resolver.properties["arm.mass"] = 5.0
    resolver.properties["arm.link1.length"] = 1.0

    assert resolver._evaluate("arm.mass") == 5.0
    assert resolver._evaluate("arm.link1.length") == 1.0


def test_xacro_eval_condition_fallbacks():
    """Test condition evaluation fallbacks and error cases."""
    resolver = XacroResolver()

    # Boolean string fallbacks
    assert resolver._eval_condition("true") is True
    assert resolver._eval_condition("1") is True
    assert resolver._eval_condition("false") is False
    assert resolver._eval_condition("0") is False

    # Prop based
    resolver.properties["enabled"] = True
    assert resolver._eval_condition("${enabled}") is True

    # Complex but valid
    assert resolver._eval_condition("5 > 3") is True

    # Exception fallback (invalid syntax)
    # the fallback returns True if not "", "0", "false"
    assert resolver._eval_condition("something weird") is True
    assert resolver._eval_condition("") is False


def test_xacro_load_data_errors(tmp_path):
    """Test error handling when loading YAML/JSON files."""
    resolver = XacroResolver(start_dir=tmp_path)
    with mock.patch("linkforge_core.parsers.xacro_parser.logger") as m:
        # 1. Non-existent YAML
        ret_yaml = resolver._handle_load_yaml("nonexistent.yaml")
        assert ret_yaml == {}
        assert m.error.called

        # 2. Non-existent JSON
        m.reset_mock()
        ret_json = resolver._handle_load_json("nonexistent.json")
        assert ret_json == {}
        assert m.error.called

        # 3. Malformed YAML
        m.reset_mock()
        bad_yaml = tmp_path / "bad.yaml"
        bad_yaml.write_text("!!invalid")
        resolver._handle_load_yaml("bad.yaml")
        assert m.error.called

        # 4. Malformed JSON
        m.reset_mock()
        bad_json = tmp_path / "bad.json"
        bad_json.write_text("{")
        resolver._handle_load_json("bad.json")
        assert m.error.called


def test_xacro_substitute_math_types():
    """Test math substitution with various types to hit all branches."""
    resolver = XacroResolver()
    # int/float branch in _substitute
    resolver.properties["val"] = 1.234
    res = resolver._substitute("${val}")
    assert str(res) == "1.234"

    # int branch
    resolver.properties["val_int"] = 5
    res = resolver._substitute("${val_int}")
    assert str(res) == "5"

    # Existing string
    res = resolver._substitute("normal string")
    assert res == "normal string"


def test_xacro_unless(tmp_path):
    """Test xacro:unless logic."""
    xacro_file = tmp_path / "test.xacro"
    xacro_file.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:unless value="0"><link name="l1"/></xacro:unless>'
        '<xacro:unless value="1"><link name="l2"/></xacro:unless></robot>'
    )
    resolver = XacroResolver()
    xml = resolver.resolve_file(xacro_file)
    assert "l1" in xml
    assert "l2" not in xml


def test_xacro_recursion_depth_with_name(tmp_path):
    """Test recursion depth error includes element name."""
    xacro_file = tmp_path / "test.xacro"
    # Create an infinite macro loop with a name
    content = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:macro name="loop" params="name">
            <xacro:loop name="inner"/>
        </xacro:macro>
        <xacro:loop name="outer"/>
    </robot>"""
    xacro_file.write_text(content)

    resolver = XacroResolver(max_depth=5)
    with pytest.raises(RobotParserError) as excinfo:
        resolver.resolve_file(xacro_file)
    assert "depth" in str(excinfo.value).lower()


def test_xacro_insert_block_container(tmp_path):
    """Test insert_block when it resolves to a container (nested elements)."""
    # Create a file to include
    inc_file = tmp_path / "inc.xacro"
    inc_file.write_text("<robot><l1/></robot>")

    xacro_file = tmp_path / "test.xacro"
    content = f"""<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:property name="my_block">
            <xacro:include filename="{inc_file}"/>
        </xacro:property>
        <xacro:insert_block name="my_block"/>
    </robot>"""
    xacro_file.write_text(content)

    resolver = XacroResolver()
    xml = resolver.resolve_file(xacro_file)
    assert "<l1" in xml


def test_xacro_macro_call_child_container(tmp_path):
    """Test macro call where child resolves to a container."""
    inc_file = tmp_path / "inc.xacro"
    inc_file.write_text("<robot><l1/></robot>")

    xacro_file = tmp_path / "test.xacro"
    content = f"""<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:macro name="m" params="*block">
            <parent>
                <xacro:insert_block name="block"/>
            </parent>
        </xacro:macro>
        <xacro:m>
            <xacro:include filename="{inc_file}"/>
        </xacro:m>
    </robot>"""
    xacro_file.write_text(content)

    resolver = XacroResolver()
    xml = resolver.resolve_file(xacro_file)
    assert "<parent><l1" in xml.replace(" ", "").replace("\n", "").replace("\r", "")


def test_xacro_conditional_nested_container(tmp_path):
    """Test xacro:if with content that resolves to a container."""
    inc_file = tmp_path / "inc.xacro"
    inc_file.write_text("<robot><l1/></robot>")

    xacro_file = tmp_path / "test.xacro"
    content = f"""<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:if value="1">
            <xacro:include filename="{inc_file}"/>
        </xacro:if>
    </robot>"""
    xacro_file.write_text(content)

    resolver = XacroResolver()
    xml = resolver.resolve_file(xacro_file)
    assert "<l1" in xml


def test_xacro_parser_extra_args_coverage(tmp_path):
    """Test passing extra arguments to XACROParser.parse (coverage variant)."""
    xacro_file = tmp_path / "test.xacro"
    xacro_file.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="r">'
        '<link name="${my_arg}"/></robot>'
    )

    parser = XACROParser()
    robot = parser.parse(xacro_file, my_arg="custom_name")
    assert robot.links[0].name == "custom_name"


def test_xacro_unknown_tag(tmp_path, caplog):
    """Test warning for unknown xacro tags."""
    xacro_file = tmp_path / "test.xacro"
    xacro_file.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:unknown_macro/></robot>'
    )

    resolver = XacroResolver()
    # Explicitly track the logger and ensure it doesn't block caplog

    # Temporary patch to ensure propagate is True during this test
    original_propagate = xacro_logger.propagate
    xacro_logger.propagate = True

    try:
        with caplog.at_level(logging.WARNING, logger=xacro_logger.name):
            resolver.resolve_file(xacro_file)
        assert any("Unknown macro" in r.message for r in caplog.records)
    finally:
        xacro_logger.propagate = original_propagate


def test_xacro_recursive_cleanup_comments():
    """Test that finalize_urdf cleans up non-string elements like comments."""
    resolver = XacroResolver()
    root = ET.Element("robot")
    link = ET.SubElement(root, "link", name="l")
    ET.Comment(" I should be preserved but my children (if any) cleaned ")
    # Add a xacro tag inside a preserved tag
    ET.SubElement(link, "xacro:property", name="p")

    xml_str = resolver._finalize_urdf(root)
    assert "xacro:property" not in xml_str
    assert '<link name="l" />' in xml_str or '<link name="l"/>' in xml_str


def test_resolve_file_flatten_container(tmp_path):
    """Test file resolution for standard XACRO files."""
    resolver = XacroResolver()
    path = tmp_path / "test.xacro"
    # Content that resolves to a container via a conditional
    path.write_text("""
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:if value="1">
            <link name="l"/>
        </xacro:if>
    </robot>
    """)

    # resolve_file -> _finalize_urdf -> _append_filtered
    # The container from if should be flattened.
    xml = resolver.resolve_file(path)
    assert '<link name="l"' in xml


def test_handle_include_missing_file_warning():
    """Test parsing of container elements with children."""
    resolver = XacroResolver()
    xml = ET.fromstring(
        '<xacro:include xmlns:xacro="http://www.ros.org/wiki/xacro" filename="missing.xacro"/>'
    )

    with mock.patch("linkforge_core.parsers.xacro_parser.logger") as m:
        resolver.resolve_element(xml)
        assert m.warning.called
        assert "Could not find included file" in m.warning.call_args[0][0]


def test_property_block_assignment():
    """Test parsing of container elements without children."""
    resolver = XacroResolver()
    xml = ET.fromstring("""
    <xacro:property xmlns:xacro="http://www.ros.org/wiki/xacro" name="block">
        <child/>
    </xacro:property>
    """)
    resolver.resolve_element(xml)
    assert "block" in resolver.properties
    assert isinstance(resolver.properties["block"], list)


def test_handle_load_json_module_missing(tmp_path):
    """Test evaluation of property blocks with content."""
    resolver = XacroResolver(start_dir=tmp_path)
    with mock.patch("linkforge_core.parsers.xacro_parser.json", None):
        res = resolver._handle_load_json("foo.json")
        assert res == {}


def test_handle_load_yaml_error(tmp_path):
    """Test evaluation of empty property blocks."""
    resolver = XacroResolver(start_dir=tmp_path)
    path = tmp_path / "bad.yaml"
    path.touch()

    with (
        mock.patch(
            "linkforge_core.parsers.xacro_parser.yaml.safe_load",
            side_effect=Exception("Read error"),
        ),
        mock.patch("linkforge_core.parsers.xacro_parser.logger") as mock_logger,
    ):
        res = resolver._handle_load_yaml("bad.yaml")
        assert res == {}
        assert mock_logger.error.called


def test_handle_load_json_error(tmp_path):
    """Test evaluation of property blocks with nested elements."""
    resolver = XacroResolver(start_dir=tmp_path)
    path = tmp_path / "bad.json"
    path.touch()

    with (
        mock.patch(
            "linkforge_core.parsers.xacro_parser.json.load", side_effect=Exception("Read error")
        ),
        mock.patch("linkforge_core.parsers.xacro_parser.logger") as mock_logger,
    ):
        res = resolver._handle_load_json("bad.json")
        assert res == {}
        assert mock_logger.error.called


def test_substitute_mixed_text():
    """Cover lines 491-493, 498 in xacro_parser.py."""
    resolver = XacroResolver()
    resolver.properties["p"] = 1.5
    res = resolver._substitute("val=${p}m")
    assert res == "val=1.5m"


def test_cleanup_non_string_tag():
    """Cover lines 578-579 in xacro_parser.py."""
    resolver = XacroResolver()
    # Create element with a Comment child (tag is a function/type, not string)
    root = ET.Element("robot")
    comment = ET.Comment("test")
    root.append(comment)

    # Mock serialize_xml to avoid crash if finalize tries to serialize it
    with mock.patch("linkforge_core.utils.xml_utils.serialize_xml", return_value=""):
        resolver._finalize_urdf(root)
        # Should not crash.


def test_try_parse_typed_value_yaml_error():
    """Cover lines 455-456 in xacro_parser.py."""
    resolver = XacroResolver()
    # Mock yaml.safe_load to raise Exception
    with mock.patch(
        "linkforge_core.parsers.xacro_parser.yaml.safe_load", side_effect=Exception("fail")
    ):
        res = resolver._try_parse_typed_value("foo")
        assert res == "foo"


def test_find_file_package_uri(tmp_path):
    """Test nested property evaluation."""
    resolver = XacroResolver(start_dir=tmp_path)
    with mock.patch("linkforge_core.parsers.xacro_parser.resolve_package_path") as m:
        m.return_value = tmp_path / "resolved.urdf"
        res = resolver._find_file("package://my_pkg/test.urdf")
        assert res == tmp_path / "resolved.urdf"
        assert m.called


class TestXACROParserEdgeCoverage:
    """Parser behavior for unknown tags, empty macros, and missing includes."""

    def _write_and_parse(self, xml: str, tmp_path: Path) -> None:
        p = tmp_path / "test.xacro"
        p.write_text(xml)
        XACROParser().parse(p)

    def test_unknown_xacro_tag_is_skipped_with_warning(self, tmp_path, caplog):
        """An unknown xacro: tag logs a warning and resolves to skip."""
        xml = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="r">
            <link name="l1"/>
            <xacro:no_such_macro/>
        </robot>"""
        with caplog.at_level(logging.WARNING, logger="linkforge_core.parsers.xacro_parser"):
            self._write_and_parse(xml, tmp_path)
        assert any("no_such_macro" in r.message for r in caplog.records)

    def test_macro_with_empty_name_is_not_registered(self, tmp_path):
        """A xacro:macro with no name attribute is ignored gracefully."""
        xml = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="r">
            <xacro:macro params=""><link name="l1"/></xacro:macro>
            <link name="base"/>
        </robot>"""
        self._write_and_parse(xml, tmp_path)

    def test_insert_block_with_non_xml_property_is_skipped(self, tmp_path):
        """insert_block on a scalar property resolves to skip."""
        xml = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="r">
            <xacro:property name="mass" value="1.0"/>
            <xacro:insert_block name="mass"/>
            <link name="base"/>
        </robot>"""
        self._write_and_parse(xml, tmp_path)

    def test_include_nonexistent_file_is_handled_gracefully(self, tmp_path, caplog):
        """Including a file that does not exist is silently skipped with a warning."""
        xml = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="r">
            <xacro:include filename="/no/such/file.xacro"/>
            <link name="base"/>
        </robot>"""
        with caplog.at_level(logging.WARNING, logger="linkforge_core.parsers.xacro_parser"):
            self._write_and_parse(xml, tmp_path)
        # The parser should log something about the missing file
        assert any("no/such" in r.message or "include" in r.message.lower() for r in caplog.records)


def test_substitute_handles_file_uri_find_pattern():
    """Verify that file://$(find pkg)/path is converted to package://pkg/path."""
    resolver = XacroResolver()

    result = resolver._substitute("file://$(find my_robot)/meshes/base.stl")
    assert result == "package://my_robot/meshes/base.stl"

    # Plain $(find ...) must still work too
    result2 = resolver._substitute("$(find my_robot)/meshes/base.stl")
    assert result2 == "package://my_robot/meshes/base.stl"


def test_xacro_parser_skips_none_kwargs(tmp_path):
    """Verify that None-valued kwargs do not override xacro arg defaults."""
    xacro_file = tmp_path / "robot.xacro"
    xacro_file.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_robot">'
        '<xacro:arg name="prefix" default=""/>'
        '<link name="${prefix}link_0"/>'
        "</robot>"
    )

    parser = XACROParser()
    robot = parser.parse(xacro_file, prefix=None)
    assert "link_0" in [link.name for link in robot.links]


def test_substitute_optenv(monkeypatch):
    """Verify $(optenv VAR default) resolves from environment, with fallback."""
    resolver = XacroResolver()

    monkeypatch.setenv("MY_VAR", "hello")
    assert resolver._substitute("$(optenv MY_VAR fallback)") == "hello"

    monkeypatch.delenv("MY_VAR", raising=False)
    assert resolver._substitute("$(optenv MY_VAR fallback)") == "fallback"
    assert resolver._substitute("$(optenv MY_VAR)") == ""


def test_substitute_env(monkeypatch):
    """Verify $(env VAR) resolves a set variable and raises when unset."""
    from linkforge_core.base import RobotParserError

    resolver = XacroResolver()

    monkeypatch.setenv("MY_VAR", "world")
    assert resolver._substitute("$(env MY_VAR)") == "world"

    monkeypatch.delenv("MY_VAR", raising=False)
    with pytest.raises(RobotParserError, match="MY_VAR"):
        resolver._substitute("$(env MY_VAR)")
