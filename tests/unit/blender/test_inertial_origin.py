import sys
from unittest.mock import MagicMock, patch

import pytest


# Reuse the mocking logic from test_converters.py
def setup_blender_suite():
    class MockOperator:
        pass

    class MockExportHelper:
        pass

    class MockImportHelper:
        pass

    mock_bpy = MagicMock()
    mock_props = MagicMock()
    mock_types = MagicMock()
    mock_app = MagicMock()

    # Assign conflict-free classes
    mock_types.Operator = MockOperator

    mock_bpy.props = mock_props
    mock_bpy.types = mock_types
    mock_bpy.app = mock_app

    mock_mathutils = MagicMock()

    class MockMatrix:
        def __init__(self, data=None):
            self.data = data or [[0.0] * 4 for _ in range(4)]

        def inverted(self):
            return self

        def to_translation(self):
            return MagicMock(x=0.0, y=0.0, z=0.0)

        def to_euler(self, order="XYZ"):
            return MagicMock(x=0.0, y=0.0, z=0.0)

        def __matmul__(self, other):
            return self

        def to_scale(self):
            return MagicMock(x=1.0, y=1.0, z=1.0)

        @classmethod
        def Identity(cls, size):  # noqa: N802
            return cls()

    mock_mathutils.Matrix = MockMatrix
    mock_mathutils.Vector = MagicMock
    mock_mathutils.Euler = MagicMock

    mock_bpy_extras = MagicMock()
    mock_io_utils = MagicMock()
    mock_io_utils.ExportHelper = MockExportHelper
    mock_io_utils.ImportHelper = MockImportHelper
    mock_bpy_extras.io_utils = mock_io_utils

    modules = {
        "bpy": mock_bpy,
        "bpy.props": mock_props,
        "bpy.types": mock_types,
        "bpy.app": mock_app,
        "mathutils": mock_mathutils,
        "bpy_extras": mock_bpy_extras,
        "bpy_extras.io_utils": mock_io_utils,
        "gpu": MagicMock(),
        "bgl": MagicMock(),
        "blf": MagicMock(),
        "gpu_extras": MagicMock(),
        "gpu_extras.batch": MagicMock(),
    }
    return modules


MOCK_MODULES = setup_blender_suite()


@pytest.fixture(autouse=True)
def apply_mocks():
    with patch.dict(sys.modules, MOCK_MODULES):
        yield


def test_inertial_origin_extraction():
    """Verify that inertial origin properties are correctly converted to Core model."""
    from linkforge.blender.converters import blender_link_to_core_with_origin

    # Setup mock object
    mock_obj = MagicMock()
    mock_obj.name = "test_link"
    mock_obj.children = []

    # Mock linkforge property group
    props = MagicMock()
    props.is_robot_link = True
    props.link_name = "test_link"
    props.mass = 1.0
    props.use_auto_inertia = False  # Manual mode

    # Set the target properties we want to test
    props.inertia_origin_xyz = (1.2, 3.4, 5.6)
    props.inertia_origin_rpy = (0.1, 0.2, 0.3)

    # Set manual inertia tensor values
    props.inertia_ixx = 1.0
    props.inertia_ixy = 0.0
    props.inertia_ixz = 0.0
    props.inertia_iyy = 1.0
    props.inertia_iyz = 0.0
    props.inertia_izz = 1.0

    mock_obj.linkforge = props

    # Call conversion
    # We patch clean_float to just return the value for simplicity
    with patch("linkforge.blender.converters.clean_float", side_effect=lambda x: float(x)):
        # We need to mock Robot properties if used, but here we can pass None
        link = blender_link_to_core_with_origin(mock_obj)

    assert link is not None
    assert link.inertial is not None

    # Verify Position
    assert link.inertial.origin.xyz.x == 1.2
    assert link.inertial.origin.xyz.y == 3.4
    assert link.inertial.origin.xyz.z == 5.6

    # Verify Rotation
    assert link.inertial.origin.rpy.x == 0.1
    assert link.inertial.origin.rpy.y == 0.2
    assert link.inertial.origin.rpy.z == 0.3
