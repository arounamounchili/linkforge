from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.adapters.context import BlenderContext
from linkforge.blender.adapters.translator import (
    ITranslator,
    JointTranslator,
    LinkTranslator,
    Ros2ControlTranslator,
    SensorTranslator,
    TranslationRegistry,
    TransmissionTranslator,
)
from linkforge.core import (
    RobotBuilder,
    RobotValidationError,
    SensorType,
    ValidationErrorCode,
    ValidationResult,
)

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_mesh_object,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_transmission,
)


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

    # 5. Verify no boundary edge warnings
    boundary_warnings = [
        w for w in result.warnings if w.code == ValidationErrorCode.MESH_BOUNDARY_EDGE
    ]

    assert len(boundary_warnings) == 0
    assert len(result.errors) == 0


def test_blender_context_adapter():
    """Verify BlenderContext adapter behavior and fallback paths."""
    # 1. Default initialization
    ctx_default = BlenderContext()
    assert ctx_default.scene == bpy.context.scene
    assert ctx_default.data == bpy.data
    assert ctx_default.ops == bpy.ops
    assert ctx_default.active_object == bpy.context.active_object
    assert ctx_default.preferences == bpy.context.preferences
    assert ctx_default.window_manager == bpy.context.window_manager
    assert list(ctx_default.get_objects()) == list(bpy.data.objects)
    assert ctx_default.get_active_object() == bpy.context.active_object

    # 2. Custom mock instance WITH context attribute
    mock_bpy = MagicMock()
    mock_bpy.context.scene = "mock_scene"
    mock_bpy.data = "mock_data"
    mock_bpy.ops = "mock_ops"

    ctx_custom = BlenderContext(mock_bpy)
    assert ctx_custom.scene == "mock_scene"
    assert ctx_custom.data == "mock_data"
    assert ctx_custom.ops == "mock_ops"

    # 3. Custom mock instance WITHOUT context/data/ops attributes to trigger fallback branches
    class LackingBpy:
        pass

    lacking_instance = LackingBpy()
    ctx_fallback = BlenderContext(lacking_instance)

    assert ctx_fallback._ctx == lacking_instance
    assert ctx_fallback.data == bpy.data
    assert ctx_fallback.ops == bpy.ops


def test_link_translator_uncovered_branches(scene, blender_context):
    """Verify LinkTranslator uncovered branches in visuals, collisions, mesh validation, and suffixes."""
    cleanup_blender_scene(scene)

    # 1. Translate when obj has no link properties (None)
    translator = LinkTranslator()
    builder = RobotBuilder("test_robot")
    assert translator.translate(None, builder, blender_context) is None

    # 2. Visual and Collision translation skip null geometry
    link_obj = create_test_object("test_link", None, scene=scene)
    lf_props = safe_get_linkforge(link_obj, scene)
    lf_props.is_robot_link = True
    lf_props.link_name = "test_link"

    # Create child with visual suffix but no actual geometry
    visual_child = create_test_object("visual_child_visual", None, scene=scene)
    visual_child.parent = link_obj

    # Create child with collision suffix but get_object_geometry returns None
    collision_child = create_test_object("collision_child_collision", None, scene=scene)
    collision_child.parent = link_obj

    # Mock get_object_geometry to return (None, None)
    with patch(
        "linkforge.blender.adapters.blender_to_core.get_object_geometry", return_value=(None, None)
    ):
        lb = translator.translate(link_obj, builder, blender_context)
        # Verify it still translated successfully
        assert lb is not None

    # 3. _get_geom_suffix with TAG_SOURCE_NAME ("source_name")
    def sanitize_func(x):
        return f"sanitized_{x}"

    child_with_source = create_test_object("child_src", None, scene=scene)
    child_with_source["source_name"] = "my_source_mesh"

    suffix = translator._get_geom_suffix(child_with_source, link_obj, "_visual", sanitize_func)
    assert suffix == "_sanitized_my_source_mesh"

    # 4. _validate_mesh when extract_mesh_triangles raises an exception
    mesh_obj = create_mesh_object("test_mesh", scene=scene, with_cube=True)
    val_result = ValidationResult(robot_name="test_robot")

    # Mock extract_mesh_triangles to raise ValueError
    with patch(
        "linkforge.blender.adapters.blender_to_core.extract_mesh_triangles",
        side_effect=ValueError("Mesh error"),
    ):
        translator._validate_mesh(mesh_obj, "test_link", "collision", val_result)
        # Should gracefully catch the exception, no crash
        assert len(val_result.errors) == 0

    # 5. _validate_mesh when extract_mesh_triangles returns None
    with patch(
        "linkforge.blender.adapters.blender_to_core.extract_mesh_triangles", return_value=None
    ):
        translator._validate_mesh(mesh_obj, "test_link", "collision", val_result)
        assert len(val_result.errors) == 0


def test_joint_translator_uncovered_branches(scene, blender_context):
    """Verify JointTranslator early exit, frame fallback, custom types, and axis fallback."""
    cleanup_blender_scene(scene)

    translator = JointTranslator()
    builder = RobotBuilder("test_robot")

    # 1. Early exit when lb is None
    builder.link("parent_link").commit()
    joint_obj = create_test_object("test_joint", None, scene=scene)
    j_props = safe_get_joint(joint_obj, scene)
    j_props.is_robot_joint = True
    parent_link = create_test_object("parent_link", None, scene=scene)
    child_link = create_test_object("child_link", None, scene=scene)

    j_props.parent_link = parent_link
    j_props.child_link = child_link

    # Verify early exit when lb=None
    assert translator.translate(joint_obj, builder, blender_context, lb=None) is None

    # 2. Missing link frames fallback
    lb = builder.link("child_link", parent="parent_link")
    link_frames = {"some_other_link": bpy.types.Matrix()}

    j_props.joint_type = "REVOLUTE"
    translator.translate(joint_obj, builder, blender_context, lb=lb, link_frames=link_frames)
    lb.commit()
    assert builder.robot.get_joint("test_joint") is not None

    # 3. Invalid axis fallback to DEFAULT_AXIS_XYZ
    joint_obj_axis = create_test_object("joint_invalid_axis", None, scene=scene)
    ja_props = safe_get_joint(joint_obj_axis, scene)
    ja_props.is_robot_joint = True
    ja_props.parent_link = parent_link
    ja_props.child_link = child_link
    ja_props.joint_type = "REVOLUTE"
    ja_props.axis = "INVALID_AXIS_VALUE"

    lb2 = builder.link("child_link_axis", parent="parent_link")
    translator.translate(joint_obj_axis, builder, blender_context, lb=lb2)
    lb2.commit()
    assert builder.robot.get_joint("joint_invalid_axis") is not None
    axis = builder.robot.get_joint("joint_invalid_axis").axis
    assert (axis.x, axis.y, axis.z) == (0.0, 0.0, 1.0)

    # 4. Special joint types: FLOATING and PLANAR
    for jt in ["FLOATING", "PLANAR"]:
        j_obj = create_test_object(f"joint_{jt.lower()}", None, scene=scene)
        jp = safe_get_joint(j_obj, scene)
        jp.is_robot_joint = True
        jp.parent_link = parent_link
        jp.child_link = child_link
        jp.joint_type = jt
        lb_jt = builder.link(f"child_{jt.lower()}", parent="parent_link")
        translator.translate(j_obj, builder, blender_context, lb=lb_jt)
        lb_jt.commit()
        assert builder.robot.get_joint(f"joint_{jt.lower()}") is not None


def test_sensor_translator_uncovered_branches(scene, blender_context):
    """Verify SensorTranslator validation exception handling, missing parent link, force-torque, and contact collision guess."""
    cleanup_blender_scene(scene)

    translator = SensorTranslator()
    builder = RobotBuilder("test_robot")

    # 1. Sensor not attached to any link raises RobotValidationError
    sensor_obj = create_test_object("test_sensor", None, scene=scene)
    s_props = safe_get_sensor(sensor_obj, scene)
    s_props.is_robot_sensor = True
    s_props.attached_link = None

    # Case A: without validation_result (bubbles up)
    with pytest.raises(RobotValidationError) as exc_info:
        translator.translate(sensor_obj, builder, blender_context, validation_result=None)
    assert exc_info.value.code == ValidationErrorCode.NOT_FOUND

    # Case B: with validation_result (caught and recorded as error)
    val_result = ValidationResult(robot_name="test_robot")
    translator.translate(sensor_obj, builder, blender_context, validation_result=val_result)
    assert len(val_result.errors) == 1
    assert "Sensor is not attached to any link" in val_result.errors[0].message

    # 2. FORCE_TORQUE sensor translation
    link_obj = create_test_object("link_for_sensor", None, scene=scene)
    safe_get_linkforge(link_obj, scene).is_robot_link = True
    safe_get_linkforge(link_obj, scene).link_name = "link_for_sensor"
    builder.link("link_for_sensor").commit()

    ft_sensor = create_test_object("ft_sensor", None, scene=scene)
    ftp = safe_get_sensor(ft_sensor, scene)
    ftp.is_robot_sensor = True
    ftp.attached_link = link_obj
    ftp.sensor_type = "FORCE_TORQUE"

    translator.translate(ft_sensor, builder, blender_context)
    assert any(s.name == "ft_sensor" for s in builder.robot.sensors)
    assert builder.robot.get_sensor("ft_sensor") is not None
    assert builder.robot.get_sensor("ft_sensor").type == SensorType.FORCE_TORQUE

    # 3. CONTACT sensor with blank collision name (fallback guesses from link name)
    contact_sensor = create_test_object("contact_sensor", None, scene=scene)
    cp = safe_get_sensor(contact_sensor, scene)
    cp.is_robot_sensor = True
    cp.attached_link = link_obj
    cp.sensor_type = "CONTACT"
    cp.contact_collision = ""

    translator.translate(contact_sensor, builder, blender_context)
    assert any(s.name == "contact_sensor" for s in builder.robot.sensors)
    assert (
        builder.robot.get_sensor("contact_sensor").contact_info.collision
        == "link_for_sensor_collision"
    )


def test_ros2_control_translator_uncovered_branches(scene, blender_context):
    """Verify Ros2ControlTranslator fallbacks, cmd interfaces stripping, empty joint skip, and multi-joint actuator truncation."""
    cleanup_blender_scene(scene)

    translator = Ros2ControlTranslator()
    builder = RobotBuilder("test_robot")

    # 1. Early return on None props or False use_ros2_control
    assert translator._blender_ros2_control_to_core(None) is None

    class FakeProps:
        use_ros2_control = False

    assert translator._blender_ros2_control_to_core(FakeProps()) is None

    # 2. Translate exception caught in validation_result
    class BrokenProps:
        use_ros2_control = True

        @property
        def ros2_control_type(self):
            raise RuntimeError("Broken ros2 control")

    val_result = ValidationResult(robot_name="test_robot")
    translator.translate(BrokenProps(), builder, blender_context, validation_result=val_result)
    assert len(val_result.errors) == 1
    assert "ROS2 Control translation failed" in val_result.errors[0].title

    # 3. Hardware type 'sensor' cannot have command interfaces (warning/strip) and empty state_ifs default
    class MockControlJoint:
        def __init__(self, name):
            self.name = name
            self.cmd_position = True
            self.state_position = False
            self.cmd_velocity = False
            self.cmd_effort = False
            self.state_velocity = False
            self.state_effort = False
            self.parameters = []
            self.joint_obj = None

    class MockControlProps:
        use_ros2_control = True
        ros2_control_name = "sensor_control"
        ros2_control_type = "sensor"
        hardware_plugin = "mock_plugin"
        ros2_control_joints = [MockControlJoint("joint_1")]

    control = translator._blender_ros2_control_to_core(MockControlProps())
    assert control is not None
    assert len(control.joints[0].command_interfaces) == 0
    assert "position" in control.joints[0].state_interfaces

    # 4. Joint with empty interfaces is skipped
    class MockControlJointEmpty:
        def __init__(self, name):
            self.name = name
            self.cmd_position = False
            self.cmd_velocity = False
            self.cmd_effort = False
            self.state_position = False
            self.state_velocity = False
            self.state_effort = False
            self.parameters = []
            self.joint_obj = None

    class MockControlPropsEmpty:
        use_ros2_control = True
        ros2_control_name = "empty_control"
        ros2_control_type = "system"
        hardware_plugin = "mock_plugin"
        ros2_control_joints = [MockControlJointEmpty("joint_empty")]

    control_empty = translator._blender_ros2_control_to_core(MockControlPropsEmpty())
    assert control_empty is None

    # 5. Actuator type with multiple joints (truncates list to exactly 1)
    class MockControlJointActuator:
        def __init__(self, name):
            self.name = name
            self.cmd_position = True
            self.state_position = True
            self.cmd_velocity = False
            self.cmd_effort = False
            self.state_velocity = False
            self.state_effort = False
            self.parameters = []
            self.joint_obj = None

    class MockControlPropsActuator:
        use_ros2_control = True
        ros2_control_name = "actuator_control"
        ros2_control_type = "actuator"
        hardware_plugin = "mock_plugin"
        ros2_control_joints = [
            MockControlJointActuator("joint_1"),
            MockControlJointActuator("joint_2"),
        ]

    control_actuator = translator._blender_ros2_control_to_core(MockControlPropsActuator())
    assert control_actuator is not None
    assert len(control_actuator.joints) == 1
    assert control_actuator.joints[0].name == "joint_1"


def test_transmission_translator_uncovered_branches(scene, blender_context):
    """Verify TransmissionTranslator properties check, differential missing joints skip, joint name fallbacks, and translation errors."""
    cleanup_blender_scene(scene)

    translator = TransmissionTranslator()
    builder = RobotBuilder("test_robot")

    # 1. Return None on None props or is_robot_transmission=False
    assert translator._blender_transmission_to_core(None) is None

    class FakeTransProps:
        is_robot_transmission = False

    assert translator._blender_transmission_to_core(FakeTransProps()) is None

    # 2. Differential type with missing joints returns None
    trans_obj = create_test_object("test_trans", None, scene=scene)
    tp = safe_get_transmission(trans_obj, scene)
    tp.is_robot_transmission = True
    tp.transmission_type = "DIFFERENTIAL"
    tp.joint1_name = None
    tp.joint2_name = None

    assert translator._blender_transmission_to_core(trans_obj) is None

    # 3. Simple transmission fallback when joint_props.joint_name is empty/None
    joint_obj_no_name = create_test_object("joint_without_custom_name", None, scene=scene)
    jp = safe_get_joint(joint_obj_no_name, scene)
    jp.is_robot_joint = True
    jp.joint_name = ""

    simple_trans_obj = create_test_object("simple_trans", None, scene=scene)
    stp = safe_get_transmission(simple_trans_obj, scene)
    stp.is_robot_transmission = True
    stp.transmission_type = "SIMPLE"
    stp.joint_name = joint_obj_no_name
    stp.use_custom_actuator_name = False

    trans_model = translator._blender_transmission_to_core(simple_trans_obj)
    assert trans_model is not None
    assert trans_model.joints[0].name == "joint_without_custom_name"
    assert trans_model.actuators[0].name == "joint_without_custom_name_motor"

    # 4. Translate exception caught in validation_result
    class BrokenTransProps:
        @property
        def linkforge_transmission(self):
            class BrokenProps:
                is_robot_transmission = True

                @property
                def transmission_type(self):
                    raise RuntimeError("Broken transmission type")

            return BrokenProps()

        @property
        def name(self):
            return "broken_trans"

    val_result = ValidationResult(robot_name="test_robot")
    translator.translate(BrokenTransProps(), builder, blender_context, validation_result=val_result)
    assert len(val_result.errors) == 1
    assert "Transmission translation failed: broken_trans" in val_result.errors[0].title
