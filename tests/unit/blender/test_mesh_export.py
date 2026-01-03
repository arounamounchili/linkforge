import sys
from unittest.mock import MagicMock, patch

import pytest

# CRITICAL: Mock all Blender-related modules BEFORE any linkforge imports.
# This prevents ModuleNotFoundError when linkforge.blender submodules are imported.


def setup_blender_mocks():
    mock_bpy = MagicMock()
    mock_app = MagicMock()
    mock_handlers = MagicMock()
    mock_ops = MagicMock()
    mock_wm = MagicMock()
    mock_props = MagicMock()
    mock_types = MagicMock()
    mock_utils = MagicMock()
    mock_data = MagicMock()
    mock_context = MagicMock()
    mock_path_utils = MagicMock()

    # Setup hierarchy
    mock_bpy.app = mock_app
    mock_app.handlers = mock_handlers
    mock_bpy.ops = mock_ops
    mock_ops.wm = mock_wm
    mock_bpy.props = mock_props
    mock_bpy.types = mock_types
    mock_bpy.utils = mock_utils
    mock_bpy.data = mock_data
    mock_bpy.context = mock_context
    mock_bpy.path_utils = mock_path_utils

    # Mock mathutils
    mock_mathutils = MagicMock()
    mock_matrix = MagicMock()
    mock_vector = MagicMock()
    mock_mathutils.Matrix = mock_matrix
    mock_mathutils.Vector = mock_vector

    # Mock bpy_extras
    mock_bpy_extras = MagicMock()
    mock_io_utils = MagicMock()
    mock_bpy_extras.io_utils = mock_io_utils

    # Mock gpu and bgl
    mock_gpu = MagicMock()
    mock_bgl = MagicMock()
    mock_blf = MagicMock()

    # Mock base classes to avoid metaclass conflicts
    class MockOperator:
        bl_idname = "mock.operator"

    class MockExportHelper:
        filepath: str = ""

    class MockImportHelper:
        filepath: str = ""

    mock_types.Operator = MockOperator
    mock_io_utils.ExportHelper = MockExportHelper
    mock_io_utils.ImportHelper = MockImportHelper

    # Mock property types
    mock_props.StringProperty = MagicMock()
    mock_props.FloatProperty = MagicMock()
    mock_props.IntProperty = MagicMock()
    mock_props.BoolProperty = MagicMock()
    mock_props.PointerProperty = MagicMock()
    mock_props.CollectionProperty = MagicMock()
    mock_props.EnumProperty = MagicMock()
    mock_props.FloatVectorProperty = MagicMock()

    # Mock 'persistent' decorator
    def persistent_decorator(func):
        return func

    mock_handlers.persistent = persistent_decorator

    # Define the modules to be injected into sys.modules
    modules = {
        "bpy": mock_bpy,
        "bpy.app": mock_app,
        "bpy.app.handlers": mock_handlers,
        "bpy.ops": mock_ops,
        "bpy.ops.wm": mock_wm,
        "bpy.props": mock_props,
        "bpy.types": mock_types,
        "bpy.utils": mock_utils,
        "bpy.data": mock_data,
        "bpy.context": mock_context,
        "bpy.path_utils": mock_path_utils,
        "bpy_extras": mock_bpy_extras,
        "bpy_extras.io_utils": mock_io_utils,
        "mathutils": mock_mathutils,
        "gpu": mock_gpu,
        "gpu_extras": MagicMock(),
        "gpu_extras.batch": MagicMock(),
        "bgl": mock_bgl,
        "blf": mock_blf,
    }

    return modules


# Apply mocks immediately at module level
MOCK_MODULES = setup_blender_mocks()
PATCHER = patch.dict(sys.modules, MOCK_MODULES)
PATCHER.start()


@pytest.fixture(autouse=True)
def blender_env():
    """Ensure mocks are fresh for each test."""
    import bpy

    # Reset version to 4.5.0 by default
    bpy.app.version = (4, 5, 0)

    # Clear previous side effects/returns
    bpy.ops.wm.collada_export = MagicMock()

    yield bpy


def test_is_dae_supported_on_blender_4x_operator_present(blender_env):
    """Test is_dae_supported on Blender 4.x when operator is present."""
    bpy = blender_env
    from linkforge.blender.utils.mesh_export import is_dae_supported

    bpy.app.version = (4, 5, 0)
    bpy.ops.wm.collada_export = MagicMock()

    assert is_dae_supported() is True


def test_is_dae_supported_on_blender_4x_operator_missing(blender_env):
    """Test is_dae_supported on Blender 4.x when operator is missing."""
    bpy = blender_env
    from linkforge.blender.utils.mesh_export import is_dae_supported

    bpy.app.version = (4, 5, 0)
    if hasattr(bpy.ops.wm, "collada_export"):
        del bpy.ops.wm.collada_export

    assert is_dae_supported() is False


def test_is_dae_supported_on_blender_5x(blender_env):
    """Test is_dae_supported on Blender 5.x."""
    bpy = blender_env
    from linkforge.blender.utils.mesh_export import is_dae_supported

    bpy.app.version = (5, 0, 0)

    # Even if operator exists, it should return False on 5.x
    bpy.ops.wm.collada_export = MagicMock()

    assert is_dae_supported() is False


def test_export_mesh_dae_calls_operator_on_success(blender_env):
    """Test export_mesh_dae successfully calls collada_export on 4.x."""
    bpy = blender_env
    from pathlib import Path

    from linkforge.blender.utils.mesh_export import export_mesh_dae

    bpy.app.version = (4, 5, 0)
    bpy.ops.wm.collada_export = MagicMock()

    mock_obj = MagicMock()
    test_path = Path("test.dae")

    with patch("pathlib.Path.mkdir"):  # Avoid actual directory creation
        result = export_mesh_dae(mock_obj, test_path)

    assert result is True
    bpy.ops.wm.collada_export.assert_called_once()
