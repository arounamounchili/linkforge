import sys
from unittest.mock import MagicMock, patch

import pytest


# Reuse the mocking logic from "test_inertial_origin.py" which is proven to work
def setup_blender_mocks():
    class MockOperator:
        pass

    class MockExportHelper:
        pass

    class MockImportHelper:
        pass

    class MockAddonPreferences:
        pass

    class MockPanel:
        pass

    class MockPropertyGroup:
        pass

    class MockEvent:
        pass

    class MockContext:
        selected_objects = []
        visible_objects = []
        active_object = None
        scene = MagicMock()
        scene.objects = []

        class MockPreferences:
            addons = MagicMock()
            addons.get.return_value = None  # Return None to use defaults

        preferences = MockPreferences()

    mock_bpy = MagicMock()
    mock_props = MagicMock()
    mock_types = MagicMock()
    mock_app = MagicMock()

    # Assign conflict-free classes
    mock_types.Operator = MockOperator
    mock_types.Context = MockContext
    mock_types.AddonPreferences = MockAddonPreferences
    mock_types.Panel = MockPanel
    mock_types.PropertyGroup = MockPropertyGroup
    mock_types.Event = MockEvent

    mock_bpy.props = mock_props
    mock_bpy.types = mock_types
    mock_bpy.app = mock_app
    # Ensure context is available via bpy.context as well
    mock_bpy.context = MockContext

    mock_mathutils = MagicMock()

    class MockMatrix:
        def __init__(self, data=None):
            pass

        def inverted(self):
            return self

        def to_translation(self):
            return MagicMock(x=0.0, y=0.0, z=0.0)

        @property
        def translation(self):
            return MagicMock(x=0.0, y=0.0, z=0.0)

        def to_euler(self, order="XYZ"):
            return MagicMock(x=0.0, y=0.0, z=0.0)

        def __matmul__(self, other):
            if hasattr(other, "xyz"):  # Vector check
                return MagicMock(x=0.0, y=0.0, z=0.0)
            return self  # Matrix check

        def to_scale(self):
            return MagicMock(x=1.0, y=1.0, z=1.0)

        def to_3x3(self):
            return self

        @classmethod
        def Rotation(cls, angle, size, axis):  # noqa: N802
            return cls()

        @classmethod
        def Identity(cls, size):  # noqa: N802
            return cls()

    # Ensure Vector behaves like a sequence for unpacking
    class MockVector:
        def __init__(self, x=0, y=0, z=0):
            if isinstance(x, (tuple, list)):
                self.x, self.y, self.z = x[0], x[1], x[2]
            else:
                self.x, self.y, self.z = x, y, z

        def normalize(self):
            pass

        def __add__(self, other):
            return MockVector(0, 0, 0)

        def __mul__(self, other):
            return MockVector(0, 0, 0)

        def __getitem__(self, idx):
            return (self.x, self.y, self.z)[idx]

        def __iter__(self):
            return iter((self.x, self.y, self.z))

        @property
        def xyz(self):
            return (self.x, self.y, self.z)

    mock_mathutils.Matrix = MockMatrix
    mock_mathutils.Vector = MockVector
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
        "linkforge.preferences": MagicMock(),
    }
    return modules


MOCK_MODULES = setup_blender_mocks()


@pytest.fixture(autouse=True)
def apply_mocks():
    with patch.dict(sys.modules, MOCK_MODULES):
        yield


def test_draw_inertia_gizmos_iteration():
    """Test that draw function iterates over selected objects."""
    from linkforge.blender.utils.inertia_gizmos import draw_inertia_gizmos

    # Mock objects
    obj1 = MagicMock()
    obj1.linkforge.is_robot_link = True
    obj1.linkforge.use_auto_inertia = False  # Should draw
    obj1.linkforge.inertia_origin_xyz = (1, 0, 0)

    obj2 = MagicMock()
    obj2.linkforge.is_robot_link = True
    obj2.linkforge.use_auto_inertia = True  # Should NOT draw

    obj3 = MagicMock()
    del obj3.linkforge  # Not a link -> Should NOT draw (AttributeError handling check)

    # Setup context
    import bpy

    # Setup context

    bpy.context.visible_objects = [obj1, obj2, obj3]
    bpy.context.scene.objects = [obj1, obj2, obj3]  # Fallback coverage

    with (
        patch("linkforge.blender.utils.inertia_gizmos.generate_inertia_axes_geometry") as mock_gen,
        patch("linkforge.blender.utils.inertia_gizmos.batch_for_shader") as mock_batch,
        patch("linkforge.blender.utils.inertia_gizmos.get_shader"),
    ):
        mock_gen.return_value = {
            "lines": [(0, 0, 0), (1, 1, 1)],
            "line_colors": [(1, 1, 1, 1), (1, 1, 1, 1)],
        }
        draw_inertia_gizmos()

        # Should have called generator for obj1 only
        mock_gen.assert_called_once_with(obj1, axis_length=0.1)

        # Should have called batch_for_shader once
        mock_batch.assert_called_once()


def test_generate_inertia_axes_geometry_empty():
    """Test graceful handling of None object."""
    from linkforge.blender.utils.inertia_gizmos import generate_inertia_axes_geometry

    data = generate_inertia_axes_geometry(None)
    assert data["lines"] == []
    assert data["line_colors"] == []


def test_generate_inertia_axes_geometry_values():
    """Test correct geometry generation logic."""
    from linkforge.blender.utils.inertia_gizmos import generate_inertia_axes_geometry

    # Mock object with linkforge properties
    mock_obj = MagicMock()
    mock_obj.linkforge.inertia_origin_xyz = (1.0, 0.0, 0.0)
    mock_obj.linkforge.inertia_origin_rpy = (0.0, 0.0, 0.0)

    # Call function
    data = generate_inertia_axes_geometry(mock_obj)

    # Expect 104 points:
    # - 3 axes * 2 points = 6 points
    # - 1 connector * 2 points = 2 points
    # - 3 rings * 16 segments * 2 points = 96 points
    # Total = 104 points
    assert len(data["lines"]) == 104
    assert len(data["line_colors"]) == 104
