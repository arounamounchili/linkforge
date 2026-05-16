import bpy
from linkforge.blender.adapters.translator import ITranslator, LinkTranslator, TranslationRegistry
from linkforge.core import ValidationErrorCode, ValidationResult

from tests.blender_test_utils import cleanup_blender_scene, create_mesh_object


class MockTranslator:
    """A minimal mock translator for testing the registry."""

    def translate(self, *args, **kwargs):
        return "translated"


def test_translation_registry_lifecycle():
    """Verify that translators can be registered and retrieved correctly."""
    registry = TranslationRegistry()
    mock_trans = MockTranslator()

    # 1. Initial state
    assert registry.get("link") is None

    # 2. Registration
    registry.register("link", mock_trans)
    assert registry.get("link") == mock_trans

    # 3. Multiple registrations
    mock_joint = MockTranslator()
    registry.register("joint", mock_joint)
    assert registry.get("joint") == mock_joint
    assert registry.get("link") == mock_trans

    # 4. Overwriting
    new_mock = MockTranslator()
    registry.register("link", new_mock)
    assert registry.get("link") == new_mock


def test_translator_protocol_compliance():
    """Verify that our core translators comply with the ITranslator protocol."""
    from linkforge.blender.adapters.translator import (
        JointTranslator,
        LinkTranslator,
        Ros2ControlTranslator,
        SensorTranslator,
        TransmissionTranslator,
    )

    translators = [
        LinkTranslator(),
        JointTranslator(),
        SensorTranslator(),
        TransmissionTranslator(),
        Ros2ControlTranslator(),
    ]

    for t in translators:
        assert isinstance(t, ITranslator)
        # Verify it has the translate method
        assert hasattr(t, "translate")
        assert callable(t.translate)


def test_validate_mesh_handles_quads_without_warnings(scene, blender_context):
    """Regression test: Verify that meshes with quads (like the default Cube)
    do not trigger 'boundary edge' warnings.
    """
    cleanup_blender_scene(scene)

    # 1. Create a standard cube (which uses quads in Blender)
    obj = create_mesh_object("Part", scene=scene, with_cube=True)

    # 2. Setup validation result
    result = ValidationResult(robot_name="test_robot")
    translator = LinkTranslator()

    # 3. Run validation
    translator._validate_mesh(obj, "Part", "visual", result)

    # 4. Verify no boundary edge warnings (MESH_BOUNDARY_EDGE)

    boundary_warnings = [
        w for w in result.warnings if w.code == ValidationErrorCode.MESH_BOUNDARY_EDGE
    ]

    assert len(boundary_warnings) == 0
    assert len(result.errors) == 0


def test_validate_mesh_with_modifiers(scene, blender_context):
    """Regression test: Verify that validation respects modifiers via depsgraph."""
    cleanup_blender_scene(scene)

    # 1. Create a cube
    obj = create_mesh_object("Part", scene=scene, with_cube=True)

    # 2. Add a Bevel modifier
    mod = obj.modifiers.new(name="Bevel", type="BEVEL")
    mod.width = 0.1

    # 3. Get evaluated depsgraph
    depsgraph = bpy.context.evaluated_depsgraph_get()

    # 4. Run validation with depsgraph
    result = ValidationResult(robot_name="test_robot")
    translator = LinkTranslator()

    translator._validate_mesh(obj, "Part", "visual", result, depsgraph=depsgraph)

    # 5. Should have no boundary warnings

    boundary_warnings = [
        w for w in result.warnings if w.code == ValidationErrorCode.MESH_BOUNDARY_EDGE
    ]

    assert len(boundary_warnings) == 0
    assert len(result.errors) == 0
