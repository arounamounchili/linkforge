from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from linkforge.core import (
    RobotXacroError,
    RobotXacroExpressionError,
    RobotXacroRecursionError,
    XACROGenerator,
    XACROParser,
    XacroResolver,
    clear_xacro_cache,
)


@pytest.fixture
def resolver() -> XacroResolver:
    return XacroResolver()


@pytest.fixture
def generator() -> XACROGenerator:
    return XACROGenerator()


# Xacro Resolver and Macro Tests


class TestXacroResolver:
    def test_simple_macro_expansion(self, resolver) -> None:
        """Test basic macro definition and call."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="test" params="name">
            <link name="${name}"/>
          </xacro:macro>
          <xacro:test name="link1"/>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        links = root.findall("link")
        assert len(links) == 1
        assert links[0].get("name") == "link1"

    def test_nested_macros(self, resolver) -> None:
        """Test macro calling another macro."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="sub" params="n">
            <link name="${n}"/>
          </xacro:macro>
          <xacro:macro name="main" params="prefix">
            <xacro:sub n="${prefix}_link"/>
          </xacro:macro>
          <xacro:main prefix="base"/>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        link = root.find("link")
        assert link is not None
        assert link.get("name") == "base_link"

    def test_resolve_file_and_includes(self, resolver, tmp_path) -> None:
        """Test resolving a physical file with relative includes."""
        main_file = tmp_path / "main.xacro"
        inc_file = tmp_path / "inc.xacro"
        inc_file.write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="test"><link name="inc_link"/></xacro:macro></robot>'
        )
        main_file.write_text(
            f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{inc_file.name}"/><xacro:test/></robot>'
        )

        resolver.start_dir = tmp_path
        xml_str = resolver.resolve_file(main_file)
        root = ET.fromstring(xml_str)
        links = root.findall("link")
        assert len(links) == 1
        assert links[0].get("name") == "inc_link"

    def test_macro_inheritance(self, resolver) -> None:
        """Test macro parameter inheritance using ^ and ^| syntax."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="p1" value="outer_val"/>
          <xacro:macro name="m1" params="p1:=^ p2:=^|fallback_val">
            <link name="${p1}" type="${p2}"/>
          </xacro:macro>
          <xacro:m1/>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        link = root.find("link")
        assert link is not None
        assert link.get("name") == "outer_val"
        assert link.get("type") == "fallback_val"

    def test_insert_block(self, resolver) -> None:
        """Test xacro:insert_block feature and recursion protection."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m" params="*block">
            <group>
              <xacro:insert_block name="block"/>
            </group>
          </xacro:macro>
          <xacro:m>
            <link name="b1"/>
            <link name="b2"/>
          </xacro:m>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        links = root.findall("group/link")
        assert len(links) == 2
        assert links[0].get("name") == "b1"
        assert links[1].get("name") == "b2"

    def test_insert_block_recursion_error(self, resolver) -> None:
        """Test recursion error catching in insert_block."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="b">
            <xacro:insert_block name="b"/>
          </xacro:property>
          <xacro:insert_block name="b"/>
        </robot>
        """

        with pytest.raises(RobotXacroRecursionError):
            resolver.resolve_string(xml)


# Evaluation and Math Tests


class TestXacroEvaluation:
    def test_math_expressions(self, resolver) -> None:
        """Test complex math in property evaluation."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="pi" value="3.14159"/>
          <link name="l">
            <visual>
              <origin rpy="${pi/2} 0 0"/>
            </visual>
          </link>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        origin = root.find("link/visual/origin")
        assert origin is not None, "Failed to find 'link/visual/origin' in resolved XML"

        rpy_attr = origin.get("rpy")
        assert rpy_attr is not None, (
            f"rpy attribute missing from origin. Attributes: {origin.attrib}"
        )

        rpy = rpy_attr.split()
        assert len(rpy) == 3, f"Expected 3 values in rpy, got {len(rpy)}: {rpy}"
        assert float(rpy[0]) == pytest.approx(1.570795)

    def test_boolean_logic(self, resolver) -> None:
        """Test xacro:if and xacro:unless."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="use_it" value="true"/>
          <xacro:if value="${use_it}">
            <link name="yes"/>
          </xacro:if>
          <xacro:unless value="${use_it}">
            <link name="no"/>
          </xacro:unless>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        links = root.findall("link")
        assert len(links) == 1
        assert links[0].get("name") == "yes"

    def test_conditional_unless(self, resolver) -> None:
        """Test xacro:unless logic branching explicitly."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:unless value="false">
            <link name="unless_true"/>
          </xacro:unless>
          <xacro:unless value="true">
            <link name="unless_false"/>
          </xacro:unless>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        links = root.findall("link")
        assert len(links) == 1
        assert links[0].get("name") == "unless_true"

    def test_evaluation_failures_and_dunders(self, resolver) -> None:
        """Test math evaluation failures, dunder protection, and fallbacks."""
        # Dunder
        xml1 = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:if value="${__class__}"><link/></xacro:if></robot>'
        with pytest.raises(RobotXacroError) as exc:
            resolver.resolve_string(xml1)
        assert "Forbidden dunder" in str(exc.value)

        # Undefined variable
        xml2 = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="${undefined_var}"/></robot>'
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(xml2)


# Security and Infrastructure Tests


class TestXacroInfrastructure:
    def test_security_constraints(self, resolver, tmp_path) -> None:
        """Verify path traversal protection in includes."""
        xml = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="/etc/passwd"/></robot>'
        # find_file handles security
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(xml)

    def test_include_with_namespace(self, resolver, tmp_path) -> None:
        """Test xacro:include with ns attribute."""
        inc_file = tmp_path / "inc.xacro"
        inc_file.write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="m"><link name="ns_link"/></xacro:macro></robot>'
        )
        xml = f'''
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:include filename="{inc_file}" ns="my_ns"/>
          <xacro:my_ns.m/>
        </robot>
        '''
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        links = root.findall("link")
        assert len(links) == 1
        assert links[0].get("name") == "ns_link"

    def test_macro_def_smart_params(self, resolver) -> None:
        """Test macro definition with bracketed defaults."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m" params="a:=[1, 2] b:=(3, 4) c:={5, 6} d">
            <link name="${d}"/>
          </xacro:macro>
          <xacro:m d="l1"/>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        assert len(root.findall("link")) == 1

    def test_macro_call_missing_args(self, resolver) -> None:
        """Test macro call with missing arguments that have defaults."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m" params="a:=1 b:=2">
            <link name="l_${a}_${b}"/>
          </xacro:macro>
          <xacro:m a="3"/>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        assert root.findall("link")[0].get("name") == "l_3_2"

    def test_eval_condition_python_ast(self, resolver) -> None:
        """Test conditional evaluation using python AST parsing for complex boolean ops."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="a.b" value="1"/>
          <xacro:property name="c" value="${a.b + 1}"/>
          <link name="${c}"/>
          <xacro:if value="1 == 1 and True">
            <link name="l1"/>
          </xacro:if>
        </robot>
        """
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        assert root.findall("link")[0].get("name") == "2"
        assert len(root.findall("link")) == 2

    def test_missing_arg_and_env(self, resolver) -> None:
        """Test $(arg), $(env), $(optenv)."""
        import os

        os.environ["TEST_ENV"] = "test_val"

        xml_arg = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="$(arg missing)"/></robot>'
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(xml_arg)

        xml_env = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="$(env MISSING_ENV)"/></robot>'
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(xml_env)

        xml_success = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="$(env TEST_ENV)"/><link name="$(optenv MISSING default_val)"/></robot>'
        resolved = resolver.resolve_string(xml_success)
        root = ET.fromstring(resolved)
        assert root.findall("link")[0].get("name") == "test_val"
        assert root.findall("link")[1].get("name") == "default_val"

    def test_load_yaml_and_json_missing(self, resolver) -> None:
        """Test load_yaml and load_json missing files."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="y" value="${xacro.load_yaml('missing.yaml')}"/>
          <xacro:property name="j" value="${xacro.load_json('missing.json')}"/>
          <link name="${len(y)}_${len(j)}"/>
        </robot>
        """
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(xml)

    def test_xacro_parser_parse_file_kwargs(self, tmp_path) -> None:
        """Test XACROParser.resolve kwargs."""

        robot_file = tmp_path / "robot.xacro"
        robot_file.write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="t"><link name="$(arg myarg)"/></robot>'
        )
        parser = XACROParser()
        xml = parser.resolve(robot_file, myarg="passed_val")
        root = ET.fromstring(xml)
        link = root.find("link")
        assert link is not None
        assert link.get("name") == "passed_val"

    def test_xacro_arg_default_and_include_missing(self, resolver) -> None:
        """Test xacro:arg default values and include missing files."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:arg name="my_arg" default="def_val"/>
          <link name="$(arg my_arg)"/>
          <xacro:include filename="this_does_not_exist.xacro"/>
        </robot>
        """
        resolved = resolver.resolve_string(xml)
        root = ET.fromstring(resolved)
        assert root.findall("link")[0].get("name") == "def_val"

    def test_xacro_load_invalid_json_yaml(self, resolver, tmp_path) -> None:
        """Test loading invalid JSON and YAML files."""
        bad_json = tmp_path / "bad.json"
        bad_json.write_text("{bad json")

        bad_yaml = tmp_path / "bad.yaml"
        bad_yaml.write_text(":")

        xml = f"""
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="j" value="${{xacro.load_json('{bad_json}')}}"/>
          <xacro:property name="y" value="${{xacro.load_yaml('{bad_yaml}')}}"/>
          <link name="${{len(j)}}_${{len(y)}}"/>
        </robot>
        """
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(xml)

    def test_insert_block_single_and_missing(self, resolver) -> None:
        """Test insert_block with a single element and missing block."""
        import xml.etree.ElementTree as ET

        resolver.properties["fake_single"] = ET.Element("link", {"name": "fake"})

        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="single_block">
            <link name="b1"/>
          </xacro:property>
          <xacro:insert_block name="single_block"/>
          <xacro:insert_block name="missing_block"/>
          <xacro:insert_block name="fake_single"/>
          <xacro:if value="1 + ">
            <link name="syntax"/>
          </xacro:if>
        </robot>
        """
        resolved = resolver.resolve_string(xml)
        root = ET.fromstring(resolved)
        links = root.findall("link")
        assert len(links) == 3
        assert links[0].get("name") == "b1"
        assert links[1].get("name") == "fake"
        assert links[2].get("name") == "syntax"

    def test_xacro_xml_cleanup_edge_cases_and_missing_names(self, resolver) -> None:
        """Test XML cleanup for comments, non-xacro namespaces, and xacro attributes, plus arg/prop missing names."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro" xmlns:test_ns="http://test.org/ns">
          <!-- This is a comment -->
          <test_ns:group xacro:attr="test" regular="attr">
            <link name="l1"/>
          </test_ns:group>
          <xacro:arg name="no_def"/>
          <xacro:property value="no_name_prop"/>
        </robot>
        """
        resolved = resolver.resolve_string(xml)
        root = ET.fromstring(resolved)

        # Test manual comment insertion for cleanup coverage
        comment_root = ET.Element("robot")
        comment_root.append(ET.Comment("manual comment"))
        clean_xml = resolver._finalize_xml(comment_root)
        assert "<!--manualcomment-->" in clean_xml.replace(" ", "")

        # Non-xacro namespace should be stripped from tag
        groups = root.findall("group")
        assert len(groups) == 1
        assert "xacro:attr" not in groups[0].attrib
        assert groups[0].get("regular") == "attr"

    def test_unknown_macro_and_inheritance_errors(self, resolver) -> None:
        """Test unknown macro skipping and ^ inheritance errors."""
        xml_unknown = (
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:unknown_macro/></robot>'
        )
        resolved = resolver.resolve_string(xml_unknown)
        assert "unknown_macro" not in resolved

        xml_inherit_err = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m" params="prop:=^">
            <link name="${prop}"/>
          </xacro:macro>
          <xacro:m/>
        </robot>
        """
        with pytest.raises(RobotXacroError) as exc:
            resolver.resolve_string(xml_inherit_err)
        assert "Outer-scope property not found" in str(exc.value)

    def test_xacro_include_success_no_ns(self, resolver, tmp_path) -> None:
        """Test successful xacro include without a namespace."""

        parser = XACROParser()

        included_file = tmp_path / "included.xacro"
        included_file.write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="m"><link name="l1"/></xacro:macro></robot>'
        )

        main_file = tmp_path / "main.xacro"
        main_file.write_text(
            f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{included_file}"/><xacro:m/></robot>'
        )

        xml = parser.resolve(main_file)
        import xml.etree.ElementTree as ET

        root = ET.fromstring(xml)
        assert root.findall("link")[0].get("name") == "l1"

    def test_xacro_file_parsing_edge_cases(self, tmp_path) -> None:
        """Test file parsing coverage for missing includes, namespaced includes, and macro parsing in _get_structural_template."""

        parser = XACROParser()

        inc_file = tmp_path / "inc.xacro"
        inc_file.write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="inc_l"/></robot>'
        )

        main_file = tmp_path / "main.xacro"
        main_file.write_text(f'''
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:include filename="missing_file.xacro"/>
          <xacro:include filename="{inc_file}" ns="my_ns"/>
          <xacro:macro name="m1" params="p1 p2"/>
          <xacro:macro/>
        </robot>
        ''')
        xml = parser.resolve(main_file)
        assert "inc_l" in xml

    def test_xacro_invalid_xml_error(self, resolver, tmp_path) -> None:
        """Test XML parse errors are caught and re-raised as RobotXacroError."""
        bad_xml = "<robot><unclosed>"
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(bad_xml)

        bad_file = tmp_path / "bad.xacro"
        bad_file.write_text(bad_xml)

        parser = XACROParser()
        with pytest.raises(RobotXacroError):
            parser.resolve(bad_file)

    def test_xacro_recursion_limit(self, resolver) -> None:
        """Test exceeding the maximum recursion depth."""
        resolver.max_depth = 1
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m1">
            <xacro:macro name="m2">
              <link/>
            </xacro:macro>
          </xacro:macro>
        </robot>
        """
        with pytest.raises(RobotXacroRecursionError):
            resolver.resolve_string(xml)

    def test_xacro_circular_include_and_cache(self, tmp_path) -> None:
        """Test circular includes to trigger RobotXacroRecursionError, and caching."""

        parser = XACROParser()

        file_a = tmp_path / "a.xacro"
        file_b = tmp_path / "b.xacro"

        file_a.write_text(
            f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{file_b}"/></robot>'
        )
        file_b.write_text(
            f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{file_a}"/></robot>'
        )

        with pytest.raises(RobotXacroRecursionError):
            parser.resolve(file_a)

        # Test cache by resolving a successful file twice
        file_c = tmp_path / "c.xacro"
        file_c.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link/></robot>')
        parser.resolve(file_c)
        parser.resolve(file_c)  # Hits TEMPLATE_CACHE

    def test_xacro_namespaced_property(self, tmp_path) -> None:
        """Test defining a property inside a namespace."""

        parser = XACROParser()

        inc_file = tmp_path / "inc.xacro"
        inc_file.write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:property name="my_prop" value="123"/></robot>'
        )

        main_file = tmp_path / "main.xacro"
        main_file.write_text(
            f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{inc_file}" ns="ns1"/></robot>'
        )

        resolved = parser.resolve(main_file)
        # Property should be namespaced
        # However, it's stored in self.properties as ns1.my_prop
        # Let's test calling it
        main_file.write_text(f'''
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:include filename="{inc_file}" ns="ns1"/>
          <link name="${{ns1.my_prop}}"/>
        </robot>
        ''')
        clear_xacro_cache()
        assert "123" in parser.resolve(main_file)

    def test_xacro_cache_and_io_error_handling(self, tmp_path) -> None:
        clear_xacro_cache()

        parser = XACROParser()
        resolver = XacroResolver()
        f = tmp_path / "test_resolver.xacro"
        f.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link/></robot>')
        resolver.resolve_file(f)

        import unittest.mock as mock

        with mock.patch(
            "linkforge.core.parsers.xacro_parser.XacroResolver._get_structural_template",
            side_effect=RuntimeError("Unexpected"),
        ):
            with pytest.raises(RobotXacroError) as exc:
                parser.resolve(f)
            assert "Unexpected" in str(exc.value)

    def test_xacro_typed_value_and_eval_errors(self, resolver) -> None:
        assert resolver._try_parse_typed_value("123") == 123
        assert resolver._try_parse_typed_value("1.23") == 1.23
        assert resolver._try_parse_typed_value("True") is True
        assert resolver._try_parse_typed_value("[1, 2, 3]") == [1, 2, 3]
        assert resolver._try_parse_typed_value("{'a': 1}") == {"a": 1}
        assert resolver._try_parse_typed_value("invalid python") == "invalid python"

        with pytest.raises(RobotXacroExpressionError):
            resolver._eval_condition("__class__ == 'str'")

        def raise_parser_error():
            raise RobotXacroError("Bubble")

        resolver.eval_context["fail"] = raise_parser_error
        with pytest.raises(RobotXacroError):
            resolver._eval_condition("fail()")

    def test_xacro_package_and_find_resolution(self, resolver, tmp_path) -> None:
        resolver.start_dir = tmp_path

        # Test $(find) substitution
        assert resolver._substitute("$(find my_pkg)/test.xacro") == "package://my_pkg/test.xacro"
        assert (
            resolver._substitute("file://$(find my_pkg)/test.xacro")
            == "package://my_pkg/test.xacro"
        )

        # Test _find_file with package://
        import unittest.mock as mock

        with mock.patch(
            "linkforge.core.parsers.xacro_parser.resolve_package_path",
            return_value=tmp_path / "found.xacro",
        ):
            assert resolver._find_file("package://my_pkg/found.xacro") == tmp_path / "found.xacro"

    def test_xacro_yaml_fallback_and_typed_exceptions(self, resolver) -> None:
        """Test YAML fallback and exception handling in _try_parse_typed_value."""
        import unittest.mock as mock

        assert resolver._try_parse_typed_value(":") == ":"

        with mock.patch("linkforge.core.parsers.xacro_parser.yaml", None):
            assert resolver._try_parse_typed_value("123") == 123
            assert resolver._try_parse_typed_value("True") == "True"  # Fallback only does numbers

    def test_xacro_load_errors(self, resolver, tmp_path) -> None:
        with pytest.raises(RobotXacroError):
            resolver._handle_load_yaml("missing.yaml")
        with pytest.raises(RobotXacroError):
            resolver._handle_load_json("missing.json")

        f = tmp_path / "bad.json"
        f.write_text("{")
        with pytest.raises(RobotXacroError):
            resolver._handle_load_json(str(f))

        f_yaml = tmp_path / "bad.yaml"
        f_yaml.write_text(":")
        with pytest.raises(RobotXacroError):
            resolver._handle_load_yaml(str(f_yaml))

    def test_xacro_macro_param_and_block_edge_cases(self, resolver) -> None:
        xml = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="my_arg" default="arg_val"/><link name="$(arg my_arg)"/></robot>'
        assert "arg_val" in resolver.resolve_string(xml)

        resolver.args["existing_arg"] = "old"
        xml = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="existing_arg" default="new"/><link name="$(arg existing_arg)"/></robot>'
        assert "old" in resolver.resolve_string(xml)

        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m" params="a:=1 b:=^">
            <link name="${a}_${b}"/>
          </xacro:macro>
          <xacro:property name="b" value="global"/>
          <xacro:m/>
        </robot>
        """
        resolved = resolver.resolve_string(xml)
        assert "1_global" in resolved

        resolver.properties["my_blocks"] = [ET.Element("link"), ET.Element("joint")]
        xml = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:insert_block name="my_blocks"/></robot>'
        # Manually call _handle_insert_block to hit the branch
        elem = ET.Element("insert_block", name="my_blocks")
        res = resolver._handle_insert_block(elem)
        assert res.tag == "container"
        assert len(list(res)) == 2

    def test_xacro_macro_call_param_edge_cases(self, resolver) -> None:
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m" params="a b:=1">
            <link name="${a}_${b}"/>
          </xacro:macro>
          <xacro:m a="val1" b="val2"/>
        </robot>
        """
        resolved = resolver.resolve_string(xml)
        assert "val1_val2" in resolved

    def test_include_no_namespace_string(self, resolver, tmp_path) -> None:
        """Test xacro:include without a namespace via resolve_string."""
        inc_file = tmp_path / "inc.xacro"
        inc_file.write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="m"><link name="no_ns_link"/></xacro:macro></robot>'
        )
        xml = f'''
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:include filename="{inc_file}"/>
          <xacro:m/>
        </robot>
        '''
        resolved_xml = resolver.resolve_string(xml)
        root = ET.fromstring(resolved_xml)
        assert len(root.findall("link")) == 1
        assert root.findall("link")[0].get("name") == "no_ns_link"

    def test_macro_param_spacing_variations(self, resolver) -> None:
        """Test macro parameter parsing with multiple consecutive spaces and trailing spaces."""
        xml = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:macro name="m" params="a   b ">
            <link name="${a}_${b}"/>
          </xacro:macro>
          <xacro:m a="1" b="2"/>
        </robot>
        """
        resolved = resolver.resolve_string(xml)
        assert "1_2" in resolved

    def test_insert_block_not_xml(self, resolver) -> None:
        """Test insert_block with a property that is a string instead of XML element/list."""
        resolver.properties["non_xml"] = "hello"
        xml = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:insert_block name="non_xml"/></robot>'
        resolved = resolver.resolve_string(xml)
        assert "hello" not in resolved

    def test_macro_call_nested_namespaces_and_block_container(self, resolver) -> None:
        """Test macro lookup with active namespace stack, and block parameter resolving to container."""
        # Define m inside my_ns
        resolver.macros["my_ns.m"] = (["*block"], ET.Element("group"))

        # Test active namespace stack lookup for a macro called from inside the namespace
        resolver._ns_stack = ["my_ns"]
        elem = ET.Element("xacro:m")
        # Add a child that is a conditional (resolves to container)
        child = ET.Element("{http://www.ros.org/wiki/xacro}if", value="true")
        child.append(ET.Element("link", name="l1"))
        elem.append(child)
        # Also add a child that resolves to skip
        elem.append(ET.Element("{http://www.ros.org/wiki/xacro}property", name="p", value="v"))

        res = resolver._handle_macro_call("xacro:m", elem)
        assert res.tag == "container"
        # Reset namespace stack
        resolver._ns_stack = []

    def test_evaluate_dot_namespaces_and_local_lookup(self, resolver) -> None:
        """Test evaluating hierarchical namespace property assignment and local lookup inside namespace stack."""
        resolver.properties["ns.a"] = 10
        resolver.properties["ns.b"] = 20

        # This will evaluate and build the nested SimpleNamespace context in ctx
        assert resolver._evaluate("ns.a + ns.b") == 30

        # Test local lookup when inside a namespace
        resolver._ns_stack = ["my_ns"]
        resolver.properties["my_ns.local_prop"] = "hello"
        resolver.properties["my_ns.nested.prop"] = "world"  # has dot in short name, skipped
        assert resolver._evaluate("local_prop") == "hello"
        resolver._ns_stack = []

    def test_arg_function_missing(self, resolver) -> None:
        """Test arg() function in expression returning empty string for missing arguments."""
        xml = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="${arg(\'missing_arg\')}"/></robot>'
        resolved = resolver.resolve_string(xml)
        root = ET.fromstring(resolved)
        assert root.findall("link")[0].get("name") == ""

    def test_load_yaml_no_pyyaml(self, resolver) -> None:
        """Test load_yaml raises RobotXacroError if PyYAML is not installed."""
        import unittest.mock as mock

        with (
            mock.patch("linkforge.core.parsers.xacro_parser.yaml", None),
            pytest.raises(RobotXacroError, match="PyYAML is not installed"),
        ):
            resolver._handle_load_yaml("dummy.yaml")

    def test_load_yaml_and_json_success(self, resolver, tmp_path) -> None:
        """Test successful loading of YAML and JSON files inside XACRO context."""
        yaml_file = tmp_path / "data.yaml"
        yaml_file.write_text("val: 42")

        json_file = tmp_path / "data.json"
        json_file.write_text('{"val": 100}')

        resolver.start_dir = tmp_path
        xml = f"""
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:property name="y" value="${{xacro.load_yaml('{yaml_file}')}}"/>
          <xacro:property name="j" value="${{xacro.load_json('{json_file}')}}"/>
          <link name="${{y.val}}_${{j.val}}"/>
        </robot>
        """
        resolved = resolver.resolve_string(xml)
        root = ET.fromstring(resolved)
        assert root.findall("link")[0].get("name") == "42_100"

    def test_finalize_xml_direct(self, resolver) -> None:
        """Test direct _finalize_xml cleanup for skip and xacro elements."""
        root = ET.Element("robot")
        child1 = ET.Element("skip")
        child2 = ET.Element("{http://www.ros.org/wiki/xacro}macro")
        root.append(child1)
        root.append(child2)
        res = resolver._finalize_xml(root)
        assert "<skip" not in res
        assert "macro" not in res

    def test_xacro_parser_resolve_with_none_val(self, tmp_path) -> None:
        """Test resolving with a kwarg set to None, checking if it is ignored."""
        robot_file = tmp_path / "robot.xacro"
        robot_file.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link/></robot>')
        parser = XACROParser()
        xml = parser.resolve(robot_file, dummy=None)
        assert "<link" in xml

    def test_xacro_parser_import_yaml_missing(self) -> None:
        """Test that importing xacro_parser when yaml is not installed sets yaml to None."""
        import importlib
        import sys
        from unittest import mock

        # Mock the import of yaml to raise ImportError
        with mock.patch.dict(sys.modules, {"yaml": None}):
            # Reload the module to trigger the try/except block
            import linkforge.core.parsers.xacro_parser as xacro_parser

            importlib.reload(xacro_parser)
            assert xacro_parser.yaml is None

        # Clean up and reload again to restore normal yaml module
        import linkforge.core.parsers.xacro_parser as xacro_parser

        importlib.reload(xacro_parser)
        assert xacro_parser.yaml is not None
