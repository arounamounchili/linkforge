"""Consolidated unit tests for Xacro Parser and Generator.

This module provides comprehensive testing for Xacro processing, including:
- XML Parsing and Macro expansion
- Property evaluation and Math
- File caching and security
- Prefixing and Namespacing
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from linkforge_core.exceptions import RobotXacroError
from linkforge_core.generators.xacro_generator import XACROGenerator
from linkforge_core.parsers.xacro_parser import XacroResolver


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


# Security and Infrastructure Tests


class TestXacroInfrastructure:
    def test_security_constraints(self, resolver, tmp_path) -> None:
        """Verify path traversal protection in includes."""
        xml = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="/etc/passwd"/></robot>'
        # find_file handles security
        with pytest.raises(RobotXacroError):
            resolver.resolve_string(xml)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
