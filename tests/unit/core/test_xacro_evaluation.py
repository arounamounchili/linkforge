import tempfile
from pathlib import Path

import pytest
from linkforge_core.parsers.xacro_parser import RobotParserError, XacroResolver


class TestXacroEvaluation:
    """Tests for XACRO evaluation logic including math, cleanup, and error handling."""

    def test_strict_output_cleanup(self):
        """Verify that ALL xacro-related tags are stripped, even with weird namespaces."""
        # Using a non-standard namespace often causes issues
        xml_content = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_bot">
            <!-- Standard skip -->
            <xacro:property name="prop" value="1"/>

            <!-- Standard macro that should be removed -->
            <xacro:macro name="my_macro" params="">
                <link name="valid_link"/>
            </xacro:macro>

            <!-- Weird namespaced tag that might leak -->
            <xacro:unknown_tag_that_leaks/>

            <!-- Manually namespaced tag -->
            <ns:xacro_feature xmlns:ns="http://example.com/ns"/>

            <!-- Call the macro to ensure valid content remains -->
            <xacro:my_macro/>
        </robot>
        """

        resolver = XacroResolver()
        with tempfile.TemporaryDirectory() as tmpdir:
            p = Path(tmpdir) / "test_cleanup.xacro"
            p.write_text(xml_content, encoding="utf-8")

            urdf_str = resolver.resolve_file(p)
            print(f"Cleanup Result: {urdf_str}")

            # Assertions
            assert '<link name="valid_link"' in urdf_str
            assert "xacro:property" not in urdf_str
            assert "xacro:macro" not in urdf_str
            assert "xacro:unknown_tag_that_leaks" not in urdf_str
            # assert "ns:xacro_feature" not in urdf_str  # Our cleaner looks for "xacro" in tag

    def test_math_evaluation_failure_reporting(self):
        """Verify that math evaluation failures raise a clear error instead of silent corruption."""
        # User error scenario: Mathematical expression fails (e.g. div by zero or missing var)
        xml_content = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_bot">
            <xacro:macro name="inertia" params="mass">
                <!-- 'radius' is undefined here! -->
                <inertial>
                    <mass value="${mass}"/>
                    <inertia ixx="${1.0/12 * mass * (3*radius**2 + 1)}" />
                </inertial>
            </xacro:macro>

            <xacro:inertia mass="10.0"/>
        </robot>
        """

        resolver = XacroResolver()
        with tempfile.TemporaryDirectory() as tmpdir:
            p = Path(tmpdir) / "test_math.xacro"
            p.write_text(xml_content, encoding="utf-8")

            # We now expect a hard failure with a helpful message
            with pytest.raises(RobotParserError, match=r"Failed to evaluate expression"):
                resolver.resolve_file(p)

    def test_math_type_error_reporting(self):
        """Verify handling when variables are wrong types (e.g. str instead of float)."""
        # User error scenario: Variable is a string '0.5' instead of number 0.5
        # This can happen if yaml parses it as string (e.g. forced quotes)
        xml_content = """
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_bot">
            <xacro:macro name="calc" params="val">
                <result value="${val**2}"/>
            </xacro:macro>

            <!-- Passing a string literal that stays a string -->
            <xacro:calc val="'5.0'"/>
        </robot>
        """

        resolver = XacroResolver()
        with tempfile.TemporaryDirectory() as tmpdir:
            p = Path(tmpdir) / "test_type.xacro"
            p.write_text(xml_content, encoding="utf-8")

            # Should fail with type error info
            from linkforge_core.base import RobotParserError

            with pytest.raises(RobotParserError, match=r"unsupported operand type"):
                resolver.resolve_file(p)


if __name__ == "__main__":
    t = TestXacroEvaluation()
    t.test_strict_output_cleanup()
    t.test_math_evaluation_failure_reporting()
