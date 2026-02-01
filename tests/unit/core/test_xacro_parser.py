import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotParserError
from linkforge_core.parsers.xacro_parser import XacroResolver


def test_xacro_substitute_basic_math():
    resolver = XacroResolver()
    # Simple math
    assert resolver._substitute("${1 + 1}") == "2"
    # Division
    assert resolver._substitute("${10 / 2}") == "5.0"
    # Parentheses and order of ops
    assert resolver._substitute("${(1 + 2) * 3}") == "9"


def test_substitute_resolves_properties_in_math():
    resolver = XacroResolver()
    resolver.properties["base_mass"] = "10.0"
    resolver.properties["x"] = "0.4"
    resolver.properties["y"] = "0.2"

    # Simple property substitution
    assert resolver._substitute("${base_mass}") == "10.0"
    # Math with properties
    assert resolver._substitute("${base_mass * 2}") == "20.0"
    # Math with properties and spaces
    assert float(resolver._substitute("${(1/12) * base_mass * (x*x + y*y)}")) == pytest.approx(
        0.16666666666666666
    )


def test_substitute_ignores_literal_vectors():
    resolver = XacroResolver()
    # Space-separated vector (should NOT be evaluated as math)
    assert resolver._substitute("${0 0 0.5}") == "0 0 0.5"

    # Vector from property
    resolver.properties["origin"] = "0 1 2"
    assert resolver._substitute("${origin}") == "0 1 2"


def test_substitute_returns_raw_expr_on_malformed_math():
    resolver = XacroResolver()
    # Malformed math should return the expression as-is (or substituted but not evaluated)
    assert resolver._substitute("${1 + / 2}") == "1 + / 2"


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
    with pytest.raises(RobotParserError, match="Failed to read XACRO file"):
        resolver.resolve_file(Path("/non/existent/path.xacro"))


def test_substitute_handles_non_numeric_eval_results():
    resolver = XacroResolver()
    # eval returning a string (not a number)
    assert resolver._substitute("${'hello'}") == "hello"
    # eval returning a boolean
    assert resolver._substitute("${True}") == "True"


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
    assert resolver.args["my_arg"] == "10"

    # Should not overwrite existing arg
    arg_xml_2 = ET.fromstring(
        "<xacro:arg xmlns:xacro='http://www.ros.org/wiki/xacro' name='my_arg' default='20'/>"
    )
    resolver.resolve_element(arg_xml_2)
    assert resolver.args["my_arg"] == "10"


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
    # Empty container should yield empty string
    empty_root = ET.Element("container")
    assert resolver._finalize_urdf(empty_root) == ""


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
        raise ValueError("Generic crash")

    monkeypatch.setattr(ET, "parse", mock_parse)

    resolver = XacroResolver()
    with pytest.raises(RobotParserError, match="Failed to read XACRO file"):
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
    # Container with NO children
    root = ET.Element("container")
    assert resolver._finalize_urdf(root) == ""


def test_resolve_file_raises_generic_error_on_resolution(tmp_path, monkeypatch):
    # This targets line 54-57 in xacro_parser.py
    from linkforge_core.parsers.xacro_parser import XacroResolver

    # We want resolve_element to raise a non-RobotParserError
    def mock_resolve_element(self, element):
        raise RuntimeError("Resolution crash")

    monkeypatch.setattr(XacroResolver, "resolve_element", mock_resolve_element)

    robot_xml = tmp_path / "valid.xacro"
    robot_xml.write_text("<robot/>")

    resolver = XacroResolver()
    with pytest.raises(RobotParserError, match="XACRO resolution failed"):
        resolver.resolve_file(robot_xml)


def test_finalize_urdf_with_container_root():
    # This targets line 271 where root.tag == "container" and len > 0
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

    resolver = XacroResolver()
    with pytest.raises(RobotParserError, match="Maximum XACRO recursion depth"):
        resolver.resolve_file(bad_xacro)
