"""Logic-only unit tests for Blender adapters using pure mocks.

These tests demonstrate the 'decoupled' testing pattern by verifying adapter
logic using unittest.mock, ensuring that the core transformation logic remains
independent of the actual Blender environment or even fake-bpy-module.
"""

import typing
from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.adapters.blender_to_core import (
    detect_primitive_type,
    matrix_to_transform,
)

from tests.mock_bpy_env import MockMatrix, MockMesh, MockObject, MockVector


def test_matrix_to_transform_mocked() -> None:
    """Verify matrix_to_transform works with a pure mock matrix object.

    This confirms the logic is decoupled from the mathutils.Matrix implementation.
    """
    # Setup mock matrix
    mock_matrix = MockMatrix.Identity(4)
    mock_matrix.data[0][3] = 1.0
    mock_matrix.data[1][3] = 2.0
    mock_matrix.data[2][3] = 3.0
    mock_matrix._euler_hint = MockVector(0.1, 0.2, 0.3)

    # Call adapter
    import mathutils

    transform = matrix_to_transform(typing.cast(mathutils.Matrix, mock_matrix))

    # Verify
    assert transform.xyz.x == 1.0
    assert transform.xyz.y == 2.0
    assert transform.xyz.z == 3.0
    assert transform.rpy.x == 0.1
    assert transform.rpy.y == 0.2
    assert transform.rpy.z == 0.3


def test_detect_primitive_type_box_mocked() -> None:
    """Verify detect_primitive_type logic for a box using mocks.

    A box requires 8 vertices and 6 quad polygons.
    """
    mock_obj = MockObject(name="Box")
    mock_obj.type = "MESH"

    # Mock mesh data using high-fidelity MockMesh to satisfy isinstance checks
    mock_mesh = MockMesh(name="BoxMesh")
    mock_mesh.vertices.clear()
    mock_mesh.vertices.extend([MagicMock()] * 8)

    # 6 faces, each with 4 vertices (quads)
    mock_poly = MagicMock()
    mock_poly.vertices = [0, 1, 2, 3]
    mock_mesh.polygons.clear()
    mock_mesh.polygons.extend([mock_poly] * 6)

    mock_obj.data = mock_mesh

    # Call logic
    result = detect_primitive_type(typing.cast(bpy.types.Object, mock_obj))

    assert result == "BOX"


def test_detect_primitive_type_sphere_mocked() -> None:
    """Verify detect_primitive_type logic for a sphere using mocks.

    A sphere is detected by vertex/face count and bounding box uniformity.
    """
    mock_obj = MockObject(name="Sphere")
    mock_obj.type = "MESH"
    # Set base dimensions so the setter below can calculate scale correctly
    if hasattr(mock_obj, "_base_dimensions"):
        mock_obj._base_dimensions = MockVector(1.0, 1.0, 1.0)
    mock_obj.dimensions = MockVector(1.0, 1.0, 1.0)

    # UV Sphere default subdivision (e.g. 482 verts)
    mock_mesh = MockMesh(name="SphereMesh")
    mock_mesh.vertices.clear()
    mock_mesh.vertices.extend([MagicMock()] * 482)
    mock_mesh.polygons.clear()
    mock_mesh.polygons.extend([MagicMock()] * 480)
    mock_obj.data = mock_mesh

    # Call logic
    result = detect_primitive_type(typing.cast(bpy.types.Object, mock_obj))

    assert result == "SPHERE"


def test_detect_primitive_type_none_mocked() -> None:
    """Verify detect_primitive_type returns None for non-mesh types."""
    mock_obj = MockObject(name="Empty")
    mock_obj.type = "EMPTY"

    assert detect_primitive_type(typing.cast(bpy.types.Object, mock_obj)) is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
