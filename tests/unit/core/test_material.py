"""Comprehensive tests for Material and Color models."""

from __future__ import annotations

import pytest

from linkforge.core.models import Color, Material


class TestColor:
    """Tests for Color class."""

    def test_creation(self):
        """Test creating a color."""
        color = Color(r=1.0, g=0.5, b=0.0, a=1.0)
        assert color.r == 1.0
        assert color.g == 0.5
        assert color.b == 0.0
        assert color.a == 1.0

    def test_to_tuple(self):
        """Test converting to tuple."""
        color = Color(r=0.1, g=0.2, b=0.3, a=0.4)
        assert color.to_tuple() == (0.1, 0.2, 0.3, 0.4)

    def test_string_representation(self):
        """Test string representation."""
        color = Color(r=0.1, g=0.2, b=0.3, a=0.4)
        result = str(color)
        assert "0.1" in result
        assert "0.2" in result
        assert "0.3" in result
        assert "0.4" in result

    def test_out_of_range_red(self):
        """Test that out of range red raises error."""
        with pytest.raises(ValueError, match="must be in range"):
            Color(r=1.5, g=0.5, b=0.5, a=1.0)

    def test_negative_red(self):
        """Test that negative red raises error."""
        with pytest.raises(ValueError, match="must be in range"):
            Color(r=-0.1, g=0.5, b=0.5, a=1.0)

    def test_out_of_range_green(self):
        """Test that out of range green raises error."""
        with pytest.raises(ValueError, match="must be in range"):
            Color(r=0.5, g=1.5, b=0.5, a=1.0)

    def test_out_of_range_blue(self):
        """Test that out of range blue raises error."""
        with pytest.raises(ValueError, match="must be in range"):
            Color(r=0.5, g=0.5, b=1.5, a=1.0)

    def test_out_of_range_alpha(self):
        """Test that out of range alpha raises error."""
        with pytest.raises(ValueError, match="must be in range"):
            Color(r=0.5, g=0.5, b=0.5, a=1.5)

    def test_boundary_values(self):
        """Test boundary values 0.0 and 1.0."""
        # All zeros
        color1 = Color(r=0.0, g=0.0, b=0.0, a=0.0)
        assert color1.r == 0.0
        assert color1.a == 0.0

        # All ones
        color2 = Color(r=1.0, g=1.0, b=1.0, a=1.0)
        assert color2.r == 1.0
        assert color2.a == 1.0


class TestMaterial:
    """Tests for Material class."""

    def test_creation_with_color(self):
        """Test creating material with color."""
        color = Color(1.0, 0.0, 0.0, 1.0)
        material = Material(name="red", color=color)
        assert material.name == "red"
        assert material.color == color
        assert material.texture is None

    def test_creation_with_texture(self):
        """Test creating material with texture."""
        material = Material(name="textured", texture="texture.png")
        assert material.name == "textured"
        assert material.texture == "texture.png"
        assert material.color is None

    def test_both_color_and_texture(self):
        """Test that having both color and texture is valid (URDF allows it)."""
        color = Color(1.0, 0.0, 0.0, 1.0)
        # URDF spec allows both - texture takes precedence
        material = Material(name="both", color=color, texture="texture.png")
        assert material.color == color
        assert material.texture == "texture.png"

    def test_neither_color_nor_texture(self):
        """Test that having neither color nor texture is invalid."""
        with pytest.raises(ValueError, match="must have either color or texture"):
            Material(name="invalid")

    def test_name_only(self):
        """Test material with name only is invalid."""
        with pytest.raises(ValueError, match="must have either color or texture"):
            Material(name="invalid_material")

    def test_string_representation_with_color(self):
        """Test string representation for colored material."""
        color = Color(1.0, 0.0, 0.0, 1.0)
        material = Material(name="red", color=color)
        result = str(material)
        assert "red" in result
        assert "color" in result.lower()

    def test_string_representation_with_texture(self):
        """Test string representation for textured material."""
        material = Material(name="textured", texture="texture.png")
        result = str(material)
        assert "textured" in result
        assert "texture.png" in result
