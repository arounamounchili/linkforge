"""Tests for Gazebo URDF extension models."""

from __future__ import annotations

import pytest

from linkforge.core.models import (
    GazeboElement,
    GazeboPlugin,
)


class TestGazeboPlugin:
    """Tests for GazeboPlugin model."""

    def test_plugin_creation(self):
        """Test creating a basic plugin."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
        )
        assert plugin.name == "test_plugin"
        assert plugin.filename == "libtest.so"
        assert len(plugin.parameters) == 0

    def test_plugin_with_parameters(self):
        """Test creating a plugin with parameters."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
            parameters={"param1": "value1", "param2": "42"},
        )
        assert plugin.parameters["param1"] == "value1"
        assert plugin.parameters["param2"] == "42"

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(ValueError, match="Plugin name cannot be empty"):
            GazeboPlugin(name="", filename="lib.so")

    def test_empty_filename(self):
        """Test that empty filename raises error."""
        with pytest.raises(ValueError, match="Plugin filename cannot be empty"):
            GazeboPlugin(name="test", filename="")


class TestGazeboElement:
    """Tests for GazeboElement model."""

    def test_robot_level_element(self):
        """Test creating a robot-level Gazebo element (no reference)."""
        element = GazeboElement(
            reference=None,
            properties={"gravity": "true"},
            static=True,
        )
        assert element.reference is None
        assert element.properties["gravity"] == "true"
        assert element.static is True

    def test_link_element(self):
        """Test creating a link-level Gazebo element."""
        element = GazeboElement(
            reference="base_link",
            material="Gazebo/Red",
            self_collide=True,
            mu1=0.8,
            mu2=0.8,
            kp=1000.0,
            kd=100.0,
        )
        assert element.reference == "base_link"
        assert element.material == "Gazebo/Red"
        assert element.self_collide is True
        assert element.mu1 == pytest.approx(0.8)
        assert element.mu2 == pytest.approx(0.8)
        assert element.kp == pytest.approx(1000.0)
        assert element.kd == pytest.approx(100.0)

    def test_joint_element(self):
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

    def test_element_with_plugin(self):
        """Test Gazebo element with plugin."""
        plugin = GazeboPlugin(name="test", filename="lib.so")
        element = GazeboElement(
            reference=None,
            plugins=[plugin],
        )
        assert len(element.plugins) == 1
        assert element.plugins[0].name == "test"

    def test_element_with_properties(self):
        """Test Gazebo element with custom properties."""
        element = GazeboElement(
            reference="link1",
            properties={"custom_prop": "custom_value"},
        )
        assert element.properties["custom_prop"] == "custom_value"

    def test_empty_reference_string(self):
        """Test that empty string reference raises error."""
        with pytest.raises(ValueError, match="Gazebo reference cannot be empty string"):
            GazeboElement(reference="")
