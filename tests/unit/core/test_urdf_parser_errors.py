"""Tests for URDF parser error handling and edge cases."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from linkforge.core.parsers.urdf_parser import (
    _validate_xml_depth,
    parse_float,
    parse_geometry,
    parse_int,
    parse_material,
    parse_vector3,
)


def test_xml_depth_validation():
    """Test validation of XML nesting depth."""
    # Create a deeply nested structure
    root = ET.Element("robot")
    curr = root
    for _ in range(101):
        child = ET.SubElement(curr, "link")
        curr = child

    with pytest.raises(ValueError, match="XML nesting too deep"):
        _validate_xml_depth(root, 0)


def test_parse_float_errors():
    """Test error cases for float parsing."""
    # Empty string
    assert parse_float("   ", "test", default=1.0) == 1.0

    # Missing required attribute
    with pytest.raises(ValueError, match="Missing required attribute 'test'"):
        parse_float(None, "test")

    # NaN and Inf
    with pytest.raises(ValueError, match="NaN"):
        parse_float("NaN", "test")
    with pytest.raises(ValueError, match="Infinite"):
        parse_float("inf", "test")

    # Out of range
    with pytest.raises(ValueError, match="outside reasonable range"):
        parse_float("1e11", "test")


def test_parse_int_errors():
    """Test error cases for int parsing."""
    # Empty string
    assert parse_int("   ", "test", default=1) == 1

    # Missing required attribute
    with pytest.raises(ValueError, match="Missing required attribute 'test'"):
        parse_int(None, "test")

    # Out of range
    with pytest.raises(ValueError, match="outside reasonable range"):
        parse_int("1000001", "test")


def test_parse_vector3_errors():
    """Test error cases for Vector3 parsing."""
    # Not 3 values
    with pytest.raises(ValueError, match="Expected 3 values"):
        parse_vector3("1.0 2.0")

    # Invalid format message check
    with pytest.raises(ValueError, match="Invalid Vector3 format"):
        parse_vector3("1.0 2.0 abc")


def test_parse_geometry_errors():
    """Test error cases for geometry parsing."""
    # Box missing size
    box_elem = ET.Element("geometry")
    ET.SubElement(box_elem, "box")
    assert parse_geometry(box_elem) is None

    # Box negative dimension
    box_elem = ET.Element("geometry")
    box = ET.SubElement(box_elem, "box")
    box.set("size", "-1 1 1")
    assert parse_geometry(box_elem) is None

    # Cylinder negative radius/length
    cyl_elem = ET.Element("geometry")
    cyl = ET.SubElement(cyl_elem, "cylinder")
    cyl.set("radius", "-1")
    assert parse_geometry(cyl_elem) is None

    # Sphere negative radius
    sph_elem = ET.Element("geometry")
    sph = ET.SubElement(sph_elem, "sphere")
    sph.set("radius", "-1")
    assert parse_geometry(cyl_elem) is None


def test_parse_material_rgba_errors():
    """Test error cases for material RGBA parsing."""
    mat_elem = ET.Element("material")
    color = ET.SubElement(mat_elem, "color")

    # Too few components
    color.set("rgba", "1.0 0.0")
    with pytest.raises(ValueError, match="expected at least 3 components"):
        parse_material(mat_elem, {})

    # Too many components
    color.set("rgba", "1.0 0.0 0.0 1.0 0.5")
    with pytest.raises(ValueError, match="expected at most 4 components"):
        parse_material(mat_elem, {})
