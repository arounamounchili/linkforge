"""Unit tests for SRDF Models, Parser, and Generator."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from linkforge_core.generators.srdf_generator import SRDFGenerator
from linkforge_core.models import Link, Robot
from linkforge_core.models.srdf import (
    CollisionPair,
    PlanningGroup,
    SemanticRobotDescription,
)
from linkforge_core.parsers.srdf_parser import SRDFParser


@pytest.fixture
def parser() -> SRDFParser:
    return SRDFParser()


@pytest.fixture
def generator() -> SRDFGenerator:
    return SRDFGenerator()


# SRDF Model Tests


class TestSRDFModels:
    def test_srdf_model_creation(self) -> None:
        """Test creating a basic SRDF model."""
        srdf = SemanticRobotDescription(robot_name="test_robot")
        assert srdf.robot_name == "test_robot"
        assert len(srdf.groups) == 0

    def test_add_group(self) -> None:
        """Test creating a planning group."""
        group = PlanningGroup(name="arm", joints=["j1", "j2"])
        srdf = SemanticRobotDescription(robot_name="r", groups=[group])
        assert len(srdf.groups) == 1
        assert srdf.groups[0].name == "arm"


# SRDF Parser Tests


class TestSRDFParser:
    def test_parse_simple_srdf(self, parser) -> None:
        """Test parsing a simple SRDF string."""
        xml = """<?xml version="1.0"?>
        <robot name="test">
            <group name="arm">
                <joint name="j1"/>
            </group>
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base_link"/>
        </robot>
        """
        srdf = parser.parse_string(xml)
        assert srdf.robot_name == "test"
        assert len(srdf.groups) == 1
        assert srdf.groups[0].name == "arm"
        assert len(srdf.virtual_joints) == 1


# SRDF Generator Tests


class TestSRDFGenerator:
    def test_generate_srdf(self, generator) -> None:
        """Test generating SRDF XML from robot model."""
        robot = Robot(name="gen_test")
        robot.add_link(Link(name="l1"))
        robot.add_link(Link(name="l2"))

        group = PlanningGroup(name="hand", links=["l1", "l2"])
        semantic = SemanticRobotDescription(
            robot_name="gen_test",
            groups=[group],
            disabled_collisions=[CollisionPair(link1="l1", link2="l2", reason="adjacent")],
        )
        robot.semantic = semantic

        # SRDFGenerator.generate() requires a Robot object
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)
        assert root.tag == "robot"
        assert root.get("name") == "gen_test"

        # Safe element retrieval
        group_tag = root.find("group")
        assert group_tag is not None, f"Generated XML missing <group> tag: {xml_str}"
        assert group_tag.get("name") == "hand"

        # Check disable_collisions
        dc = root.find("disable_collisions")
        assert dc is not None, "Generated XML missing <disable_collisions> tag"
        assert dc.get("link1") == "l1"
        assert dc.get("link2") == "l2"
        assert dc.get("reason") == "adjacent"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
