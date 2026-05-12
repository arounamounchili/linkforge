"""Tests for Gazebo URDF extension models."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import (
    GazeboElement,
    GazeboPlugin,
)


class TestGazeboPlugin:
    """Tests for GazeboPlugin model."""

    def test_plugin_creation(self) -> None:
        """Test creating a basic plugin."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
        )
        assert plugin.name == "test_plugin"
        assert plugin.filename == "libtest.so"
        assert len(plugin.parameters) == 0

    def test_plugin_with_parameters(self) -> None:
        """Test creating a plugin with parameters."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
            parameters={"param1": "value1", "param2": "42"},
        )
        assert plugin.parameters["param1"] == "value1"
        assert plugin.parameters["param2"] == "42"

    def test_prefix(self) -> None:
        """Test creating a plugin with a prefix."""
        plugin = GazeboPlugin(name="p1", filename="f1")
        pre = plugin.with_prefix("g_")
        assert pre.name == "g_p1"

    def test_empty_name(self) -> None:
        """Test that empty name raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            GazeboPlugin(name="", filename="lib.so")

    def test_empty_filename(self) -> None:
        """Test that empty filename raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            GazeboPlugin(name="test", filename="")


class TestGazeboElement:
    """Tests for GazeboElement model."""

    def test_robot_level_element(self) -> None:
        """Test creating a robot-level Gazebo element (no reference)."""
        element = GazeboElement(
            reference=None,
            properties={"gravity": "true"},
            static=True,
        )
        assert element.reference is None
        assert element.properties["gravity"] == "true"
        assert element.static is True

    def test_link_element(self) -> None:
        """Test creating a link-level Gazebo element."""
        element = GazeboElement(
            reference="base_link",
            material="Gazebo/Red",
        )
        assert element.reference == "base_link"
        assert element.material == "Gazebo/Red"

    def test_joint_element(self) -> None:
        """Test creating a joint-level Gazebo element."""
        element = GazeboElement(
            reference="joint1",
            stop_cfm=0.0,
            stop_erp=0.2,
            provide_feedback=True,
            implicit_spring_damper=True,
        )
        assert element.reference == "joint1"
        assert element.stop_cfm == pytest.approx(0.0)
        assert element.stop_erp == pytest.approx(0.2)
        assert element.provide_feedback is True
        assert element.implicit_spring_damper is True

    def test_element_with_plugin(self) -> None:
        """Test Gazebo element with plugin."""
        plugin = GazeboPlugin(name="test", filename="lib.so")
        element = GazeboElement(
            reference=None,
            plugins=[plugin],
        )
        assert len(element.plugins) == 1
        assert element.plugins[0].name == "test"

    def test_element_with_properties(self) -> None:
        """Test Gazebo element with custom properties."""
        element = GazeboElement(
            reference="link1",
            properties={"custom_prop": "custom_value"},
        )
        assert element.properties["custom_prop"] == "custom_value"

    def test_prefix(self) -> None:
        """Test creating a gazebo element with a prefix."""
        plugin = GazeboPlugin(name="p1", filename="f1")
        ge = GazeboElement(reference="l1", plugins=[plugin])
        pre = ge.with_prefix("g_")
        assert pre.reference == "g_l1"
        assert pre.plugins[0].name == "g_p1"

    def test_empty_reference_string(self) -> None:
        """Test that empty string reference raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            GazeboElement(reference="")
