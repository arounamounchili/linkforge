from __future__ import annotations

import sys
from unittest.mock import MagicMock

import pytest

# Check if we are running inside real Blender or a stubbed environment
try:
    import bpy

    # If it's a real Blender, bpy will have 'app'
    is_real_blender = hasattr(bpy, "app")
except (ImportError, AttributeError):
    is_real_blender = False

HAS_BPY = is_real_blender


if not is_real_blender:

    class BlenderBase:
        """Shared base for all mocked Blender components."""

        def __init__(self, *args, **kwargs):
            pass

    # Unique dummy classes to avoid duplicate base class errors
    class MockOperator(BlenderBase):
        pass

    class MockPanel(BlenderBase):
        pass

    class MockPropertyGroup(BlenderBase):
        pass

    class MockAddonPreferences(BlenderBase):
        pass

    class MockExportHelper(BlenderBase):
        pass

    class MockImportHelper(BlenderBase):
        pass

    # Mock bpy.types before any imports
    mock_types = MagicMock()
    mock_types.Operator = MockOperator
    mock_types.Panel = MockPanel
    mock_types.PropertyGroup = MockPropertyGroup
    mock_types.AddonPreferences = MockAddonPreferences
    mock_types.Header = BlenderBase
    mock_types.Menu = BlenderBase
    mock_types.UIList = BlenderBase

    # Create the main bpy mock
    mock_bpy = MagicMock()
    mock_bpy.types = mock_types
    mock_bpy.app = MagicMock()
    mock_bpy.app.version = (4, 2, 0)

    # Mock bpy.props
    mock_props = MagicMock()
    mock_props.IntProperty = MagicMock(return_value=0)
    mock_props.StringProperty = MagicMock(return_value="")
    mock_props.BoolProperty = MagicMock(return_value=False)
    mock_props.FloatProperty = MagicMock(return_value=0.0)
    mock_props.EnumProperty = MagicMock(return_value=None)
    mock_props.PointerProperty = MagicMock(return_value=None)
    mock_props.CollectionProperty = MagicMock(return_value=[])
    mock_props.FloatVectorProperty = MagicMock(return_value=(0.0, 0.0, 0.0))
    mock_bpy.props = mock_props

    # Inject into sys.modules
    sys.modules["bpy"] = mock_bpy
    sys.modules["bpy.types"] = mock_types
    sys.modules["bpy.props"] = mock_props

    # Mock mathutils
    mock_mathutils = MagicMock()
    mock_mathutils.Vector = MagicMock
    mock_mathutils.Matrix = MagicMock
    mock_mathutils.Quaternion = MagicMock
    mock_mathutils.Euler = MagicMock
    sys.modules["mathutils"] = mock_mathutils

    # Mock bpy_extras
    mock_bpy_extras = MagicMock()
    mock_io_utils = MagicMock()
    mock_io_utils.ExportHelper = MockExportHelper
    mock_io_utils.ImportHelper = MockImportHelper
    mock_bpy_extras.io_utils = mock_io_utils
    sys.modules["bpy_extras"] = mock_bpy_extras
    sys.modules["bpy_extras.io_utils"] = mock_io_utils

    # Mock gpu
    sys.modules["gpu"] = MagicMock()
    sys.modules["gpu_extras"] = MagicMock()
    sys.modules["gpu_extras.batch"] = MagicMock()

    # Define a basic scene for the mock
    mock_scene = MagicMock(name="MockScene")
    mock_scene.name = "Scene"
    mock_scene.linkforge = MagicMock(name="LinkForgeProps")
    mock_bpy.context.scene = mock_scene
    mock_bpy.data.scenes = [mock_scene]

    HAS_BPY = False
else:
    HAS_BPY = True

# =============================================================================
# FIXTURES
# =============================================================================


@pytest.fixture
def blender_context():
    """Unified fixture providing either a real or mock BlenderContext."""
    import bpy
    from linkforge.blender.adapters.context import BlenderContext

    if is_real_blender:
        return BlenderContext(bpy.context)
    else:
        # Create a robust mock that mimics BlenderContext
        mock = MagicMock(name="MockBlenderContext")
        mock.scene = bpy.context.scene
        mock.data = bpy.data
        mock.ops = bpy.ops

        # Simulate IBlenderContext methods
        mock.get_objects.return_value = []
        mock.get_active_object.return_value = None

        return mock


@pytest.fixture
def mock_blender_context(blender_context):
    """Alias for blender_context for backward compatibility."""
    return blender_context


@pytest.fixture
def mock_context(blender_context):
    """Alias for blender_context for legacy tests."""
    return blender_context


@pytest.fixture
def scene(blender_context):
    return blender_context.scene


@pytest.fixture(autouse=True)
def clean_scene(blender_context):
    if is_real_blender:
        import bpy

        for obj in list(bpy.data.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
    yield
