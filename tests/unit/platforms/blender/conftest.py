from __future__ import annotations

import sys
from typing import Any
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

    class MockObject(BlenderBase):
        children: list = []
        users_collection: list = []
        matrix_world: Any = MagicMock()
        matrix_parent_inverse: Any = MagicMock()
        location: list = [0.0, 0.0, 0.0]
        rotation_euler: list = [0.0, 0.0, 0.0]
        type: str = "EMPTY"
        name: str = "Object"
        linkforge: Any = MagicMock()
        linkforge_joint: Any = MagicMock()
        linkforge_transmission: Any = MagicMock()
        linkforge_sensor: Any = MagicMock()
        parent: Any = None

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            # Re-initialize instance variables to avoid shared state if needed
            self.children = []
            self.users_collection = []
            self.matrix_world = MagicMock()
            self.matrix_world.translation = [0.0, 0.0, 0.0]
            self.matrix_parent_inverse = MagicMock()

        def select_get(self):
            return True

        def select_set(self, state):
            pass

        def __getattr__(self, name):
            # Fallback for attributes not explicitly defined
            return MagicMock(name=name)

    class MockCollection(BlenderBase):
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
    mock_types.Object = MockObject
    mock_types.Collection = MockCollection

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

    # Mock bpy.ops
    mock_ops = MagicMock()

    def mock_empty_add(*args, **kwargs):
        name = kwargs.get("name", "Empty")
        new_obj = MagicMock(name=name, spec=MockObject)
        new_obj.children = []
        new_obj.name = name
        new_obj.type = "EMPTY"

        # Explicitly set robot properties to False to avoid MagicMock truthiness
        new_obj.linkforge = MagicMock()
        new_obj.linkforge.id_data = new_obj
        new_obj.linkforge.is_robot_link = False
        new_obj.linkforge.mass = 0.0
        new_obj.linkforge.link_name = ""
        new_obj.linkforge_joint = MagicMock()
        new_obj.linkforge_joint.id_data = new_obj
        new_obj.linkforge_joint.is_robot_joint = False
        new_obj.linkforge_joint.joint_name = ""
        new_obj.linkforge_transmission = MagicMock()
        new_obj.linkforge_transmission.id_data = new_obj
        new_obj.linkforge_transmission.is_robot_transmission = False
        new_obj.linkforge_sensor = MagicMock()
        new_obj.linkforge_sensor.id_data = new_obj
        new_obj.linkforge_sensor.is_robot_sensor = False

        new_obj.select_get.return_value = True
        mock_bpy.data.objects[name] = new_obj
        mock_bpy.context.active_object = new_obj
        if hasattr(mock_bpy.context.scene, "objects"):
            mock_bpy.context.scene.objects.append(new_obj)
        return {"FINISHED"}

    mock_ops.object.empty_add = mock_empty_add

    def mock_cube_add(*args, **kwargs):
        return mock_empty_add(name="Cube")

    mock_ops.mesh.primitive_cube_add = mock_cube_add

    def mock_create_transmission(*args, **kwargs):
        # Very basic simulation of the operator
        active = mock_bpy.context.active_object
        if active and hasattr(active, "linkforge_joint") and active.linkforge_joint.is_robot_joint:
            name = f"{active.name}_trans"
            trans = mock_objects_new(name)
            trans.parent = active
            trans.linkforge_transmission.is_robot_transmission = True
            mock_bpy.context.active_object = trans
            if hasattr(mock_bpy.context.scene, "objects"):
                mock_bpy.context.scene.objects.append(trans)
            return {"FINISHED"}
        return {"CANCELLED"}

    mock_ops.linkforge.create_transmission = mock_create_transmission
    mock_bpy.ops = mock_ops

    def mock_objects_new(name, data=None):
        obj = MagicMock(name=name, spec=MockObject)
        obj.children = []
        obj.name = name
        obj.type = "EMPTY"
        obj.linkforge = MagicMock()
        obj.linkforge.id_data = obj
        obj.linkforge.is_robot_link = False
        obj.linkforge.mass = 0.0
        obj.linkforge.link_name = ""
        obj.linkforge_joint = MagicMock()
        obj.linkforge_joint.id_data = obj
        obj.linkforge_joint.is_robot_joint = False
        obj.linkforge_joint.joint_name = ""
        obj.linkforge_transmission = MagicMock()
        obj.linkforge_transmission.id_data = obj
        obj.linkforge_transmission.is_robot_transmission = False
        obj.linkforge_sensor = MagicMock()
        obj.linkforge_sensor.id_data = obj
        obj.linkforge_sensor.is_robot_sensor = False
        mock_bpy.data.objects[name] = obj
        return obj

    mock_bpy.data.objects.new = mock_objects_new

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

    # Mock bmesh
    sys.modules["bmesh"] = MagicMock()

    # Mock gpu
    sys.modules["gpu"] = MagicMock()
    sys.modules["gpu_extras"] = MagicMock()
    sys.modules["gpu_extras.batch"] = MagicMock()

    # Define a basic scene for the mock
    mock_scene = MagicMock(name="MockScene")
    mock_scene.name = "Scene"
    mock_scene.linkforge = MagicMock(name="LinkForgeProps")
    mock_scene.objects = []
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
    else:
        # For mock environment, we need to clear our custom objects list
        if hasattr(blender_context.scene, "objects"):
            blender_context.scene.objects = []

    # Clear architectural statistics cache for test isolation
    import os

    os.environ["LINKFORGE_DISABLE_CACHE"] = "1"

    from linkforge.blender.utils.scene_utils import clear_stats_cache

    clear_stats_cache()
    yield
