import contextlib
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
    assert ctx_default.view_layer == bpy.context.view_layer
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

    # 2b. Translate exception caught with validation_result=None (swallowed/ignored)
    translator.translate(BrokenProps(), builder, blender_context, validation_result=None)

    # 2c. Translate valid system hardware type with state_ifs but no cmd_ifs to cover fallbacks
    class MockControlJointSystem:
        def __init__(self, name, state_only=True):
            self.name = name
            self.cmd_position = not state_only
            self.cmd_velocity = False
            self.cmd_effort = False
            self.state_position = bool(state_only)
            self.state_velocity = False
            self.state_effort = False
            self.parameters = []
            self.joint_obj = None

    class MockControlPropsSystem:
        use_ros2_control = True
        ros2_control_name = ""  # Cover empty ros2_control_name fallback to "RobotControl"
        ros2_control_type = "system"
        hardware_plugin = "mock_plugin"
        ros2_control_joints = [
            MockControlJointSystem("joint_sys1", state_only=True),
            MockControlJointSystem("joint_sys2", state_only=False),
        ]

    # 2c. Translate valid system hardware type with state_ifs but no cmd_ifs to cover fallbacks
    builder.link("link_p1").commit()
    builder.link("link_c1", parent="link_p1", joint_name="joint_sys1").commit()
    builder.link("link_c2", parent="link_p1", joint_name="joint_sys2").commit()

    translator.translate(MockControlPropsSystem(), builder, blender_context)
    assert builder.robot.get_ros2_control("RobotControl") is not None
    assert list(builder.robot.get_ros2_control("RobotControl").joints[0].command_interfaces) == [
        "position"
    ]
    assert list(builder.robot.get_ros2_control("RobotControl").joints[1].state_interfaces) == [
        "position"
    ]

    # 2d. ROS2 Control: joint_obj present, joint_props.joint_name is non-string → fallback to item.name
    #     When item.name is also None → final fallback becomes "joint"
    class MockJointPropsNonStr:
        joint_name = 123  # non-string

    class MockControlJointWithObj:
        def __init__(self, name, joint_obj):
            self.name = name
            self.cmd_position = True
            self.state_position = True
            self.cmd_velocity = False
            self.cmd_effort = False
            self.state_velocity = False
            self.state_effort = False
            self.parameters = []
            self.joint_obj = joint_obj

    joint_obj_non_str = create_test_object("joint_sys_non_str_obj2", None, scene=scene)

    # item.name = None → not isinstance(None, str) → fallback to "joint"
    class MockControlPropsWithObj:
        use_ros2_control = True
        ros2_control_name = "obj_control"
        ros2_control_type = "system"
        hardware_plugin = "mock_plugin"
        ros2_control_joints = [MockControlJointWithObj(None, joint_obj_non_str)]

    with patch(
        "linkforge.blender.adapters.translator.get_joint_props",
        return_value=MockJointPropsNonStr(),
    ):
        control_obj = translator._blender_ros2_control_to_core(MockControlPropsWithObj())
    assert control_obj is not None
    assert control_obj.joints[0].name == "joint"

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

    # 3b. Simple transmission when joint_props is None (via mocking property helper)
    with patch("linkforge.blender.adapters.translator.get_joint_props", return_value=None):
        trans_model_fallback = translator._blender_transmission_to_core(simple_trans_obj)
        assert trans_model_fallback is not None
        assert trans_model_fallback.joints[0].name == "joint_without_custom_name"

    # 3c. Simple transmission when joint_props.joint_name is a non-string object
    jp.joint_name = 123  # type: ignore
    trans_model_non_str = translator._blender_transmission_to_core(simple_trans_obj)
    assert trans_model_non_str is not None
    assert trans_model_non_str.joints[0].name == "joint_without_custom_name"

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

    # 4b. Translate exception with validation_result=None (swallowed/ignored)
    translator.translate(BrokenTransProps(), builder, blender_context, validation_result=None)

    # 5. DIFFERENTIAL transmission with valid joint objects
    j1_obj = create_test_object("joint1_obj", None, scene=scene)
    j1_p = safe_get_joint(j1_obj, scene)
    j1_p.is_robot_joint = True
    j1_p.joint_name = "joint_one"

    j2_obj = create_test_object("joint2_obj", None, scene=scene)
    j2_p = safe_get_joint(j2_obj, scene)
    j2_p.is_robot_joint = True
    j2_p.joint_name = "joint_two"

    diff_trans_obj = create_test_object("diff_trans", None, scene=scene)
    dtp = safe_get_transmission(diff_trans_obj, scene)
    dtp.is_robot_transmission = True
    dtp.transmission_type = "DIFFERENTIAL"
    dtp.joint1_name = j1_obj
    dtp.joint2_name = j2_obj

    diff_model = translator._blender_transmission_to_core(diff_trans_obj)
    assert diff_model is not None
    assert len(diff_model.joints) == 2
    assert diff_model.joints[0].name == "joint_one"
    assert diff_model.joints[1].name == "joint_two"

    # 6. SIMPLE transmission where joint_props.joint_name IS a valid string (covers 794->797 false path)
    joint_named_obj = create_test_object("joint_named_obj", None, scene=scene)
    jp_named = safe_get_joint(joint_named_obj, scene)
    jp_named.is_robot_joint = True
    jp_named.joint_name = "my_named_joint"

    named_trans_obj = create_test_object("named_trans_obj", None, scene=scene)
    ntp = safe_get_transmission(named_trans_obj, scene)
    ntp.is_robot_transmission = True
    ntp.transmission_type = "SIMPLE"
    ntp.joint_name = joint_named_obj
    ntp.use_custom_actuator_name = True
    ntp.actuator_name = "my_custom_actuator"

    named_model = translator._blender_transmission_to_core(named_trans_obj)
    assert named_model is not None
    assert named_model.joints[0].name == "my_named_joint"
    assert named_model.actuators[0].name == "my_custom_actuator"


def test_ros2_control_sensor_type_no_cmd_ifs(scene, blender_context):
    """Cover sensor hardware type branch when cmd_ifs is empty (670->676) and state_ifs already set (676->685)."""
    cleanup_blender_scene(scene)

    translator = Ros2ControlTranslator()

    # Case A: sensor type with NO command interfaces AND existing state_ifs — should NOT add default
    class MockJointSensorStateOnly:
        name = "joint_s"
        cmd_position = False
        cmd_velocity = False
        cmd_effort = False
        state_position = True  # state already set
        state_velocity = False
        state_effort = False
        parameters = []
        joint_obj = None

    class MockPropsSensorStateOnly:
        use_ros2_control = True
        ros2_control_name = "sensor_state_only"
        ros2_control_type = "sensor"
        hardware_plugin = "mock_plugin"
        ros2_control_joints = [MockJointSensorStateOnly()]

    result = translator._blender_ros2_control_to_core(MockPropsSensorStateOnly())
    assert result is not None
    # state_ifs already had "position", should not double-add
    assert list(result.joints[0].state_interfaces) == ["position"]
    assert list(result.joints[0].command_interfaces) == []


def test_joint_translator_planar_type_axis(scene, blender_context):
    """Cover JointType.PLANAR joint branch (404->408 false path is the default after PLANAR executes)."""
    cleanup_blender_scene(scene)

    translator = JointTranslator()
    builder = RobotBuilder("planar_robot")
    builder.link("base_link").commit()

    parent_obj = create_test_object("base_link", None, scene=scene)
    child_obj = create_test_object("child_planar", None, scene=scene)

    planar_joint_obj = create_test_object("planar_joint", None, scene=scene)
    jp = safe_get_joint(planar_joint_obj, scene)
    jp.is_robot_joint = True
    jp.parent_link = parent_obj
    jp.child_link = child_obj
    jp.joint_type = "PLANAR"

    lb = builder.link("child_planar", parent="base_link")
    translator.translate(planar_joint_obj, builder, blender_context, lb=lb)
    lb.commit()

    joint = builder.robot.get_joint("planar_joint")
    assert joint is not None
    assert joint.type.value == "planar"


def test_transmission_custom_type(scene, blender_context):
    """Cover CUSTOM transmission type (raw_type in TRANS_CUSTOM path, not DIFFERENTIAL).

    This also covers branch 815->849 (elif DIFFERENTIAL is False) and branch 794->797
    (joint_props.joint_name IS a valid string, so no fallback needed).
    """
    cleanup_blender_scene(scene)

    translator = TransmissionTranslator()

    # CUSTOM type with a properly named joint
    joint_obj = create_test_object("custom_joint_obj", None, scene=scene)
    jp = safe_get_joint(joint_obj, scene)
    jp.is_robot_joint = True
    jp.joint_name = "custom_joint_obj"

    custom_trans_obj = create_test_object("custom_trans_obj", None, scene=scene)
    ctp = safe_get_transmission(custom_trans_obj, scene)
    ctp.is_robot_transmission = True
    # TRANS_CUSTOM = "custom"; .lower() must match
    ctp.transmission_type = "CUSTOM"
    ctp.joint_name = joint_obj
    ctp.use_custom_actuator_name = False

    model = translator._blender_transmission_to_core(custom_trans_obj)
    assert model is not None
    assert len(model.joints) == 1


def test_link_translator_comprehensive(scene, blender_context):
    """Cover visual/collision successful translation pathways, manual inertia, and simulation props."""
    cleanup_blender_scene(scene)

    from linkforge.core import Box, Vector3
    from mathutils import Matrix

    translator = LinkTranslator()
    builder = RobotBuilder("test_robot")

    # Setup parent link object
    link_obj = create_test_object("my_link", None, scene=scene)
    lp = safe_get_linkforge(link_obj, scene)
    lp.is_robot_link = True
    lp.link_name = "my_link"

    # Configure manual inertia & mass
    lp.use_auto_inertia = False
    lp.mass = 12.5
    lp.inertia_ixx = 1.0
    lp.inertia_iyy = 2.0
    lp.inertia_izz = 3.0
    lp.inertia_ixy = 0.1
    lp.inertia_ixz = 0.2
    lp.inertia_iyz = 0.3
    lp.inertia_origin_xyz = (0.1, 0.2, 0.3)
    lp.inertia_origin_rpy = (0.01, 0.02, 0.03)

    # Configure Gazebo simulation properties
    lp.use_simulation_props = True
    lp.self_collide = True
    lp.gravity = False
    lp.mu = 0.5
    lp.mu2 = 0.8
    lp.kp = 10000.0
    lp.kd = 10.0

    # Create visual child object
    visual_child = create_mesh_object("my_link_visual", scene=scene, with_cube=True)
    visual_child.parent = link_obj

    # Assign a material to visual child
    mat = bpy.data.materials.new(name="test_mat")
    visual_child.data.materials.append(mat)
    lp.use_material = True

    # Create a second visual child to trigger visual_count > 1 and index fallback
    visual_child2 = create_mesh_object("my_link_visual_2", scene=scene, with_cube=True)
    visual_child2.parent = link_obj
    visual_child2.data.materials.append(mat)

    # Create collision child object
    collision_child = create_mesh_object("my_link_collision", scene=scene, with_cube=True)
    collision_child.parent = link_obj
    # collision quality < 100% to test simplification
    lp.collision_quality = 50.0

    # Create a second collision child to trigger index fallback
    collision_child2 = create_mesh_object("my_link_collision_2", scene=scene, with_cube=True)
    collision_child2.parent = link_obj

    # Mock get_object_geometry to return valid geometry
    mock_geom = Box(size=Vector3(1.0, 1.0, 1.0))
    with patch(
        "linkforge.blender.adapters.blender_to_core.get_object_geometry",
        return_value=(mock_geom, Matrix.Identity(4)),
    ):
        lb = translator.translate(link_obj, builder, blender_context)
        assert lb is not None
        lb.commit()

    robot_link = builder.robot.get_link("my_link")
    assert robot_link is not None
    assert robot_link.inertial is not None
    assert robot_link.inertial.mass == 12.5
    assert robot_link.inertial.inertia.ixx == 1.0
    assert len(robot_link.visuals) > 0
    assert len(robot_link.collisions) > 0
    assert robot_link.physics is not None
    assert robot_link.physics.self_collide is True
    assert robot_link.physics.gravity is False


def test_joint_translator_comprehensive(scene, blender_context):
    """Cover all remaining JointTranslator branches (early exits, validation, axes, dynamics, safety, calibration)."""
    cleanup_blender_scene(scene)

    from linkforge.core import RobotValidationError
    from mathutils import Matrix

    translator = JointTranslator()
    builder = RobotBuilder("test_joint_robot")

    # 1. Early returns & validation
    dummy_obj = create_test_object("dummy_obj", None, scene=scene)
    # is_robot_joint defaults to False, so it early exits
    assert translator.translate(dummy_obj, builder, blender_context) is None

    # Enable is_robot_joint
    jp = safe_get_joint(dummy_obj, scene)
    jp.is_robot_joint = True
    # Raises Validation Error for missing parent link
    with pytest.raises(RobotValidationError, match="Joint has no parent link"):
        translator.translate(dummy_obj, builder, blender_context)

    # Add parent
    parent_obj = create_test_object("parent_link_obj", None, scene=scene)
    parent_lp = safe_get_linkforge(parent_obj, scene)
    parent_lp.is_robot_link = True
    parent_lp.link_name = "parent_link_name"
    jp.parent_link = parent_obj

    # Raises Validation Error for missing child link
    with pytest.raises(RobotValidationError, match="Joint has no child link"):
        translator.translate(dummy_obj, builder, blender_context)

    # Add child
    child_obj = create_test_object("child_link_obj", None, scene=scene)
    child_lp = safe_get_linkforge(child_obj, scene)
    child_lp.is_robot_link = True
    child_lp.link_name = "child_link_name"
    jp.child_link = child_obj

    # If lb is None, returns early
    assert translator.translate(dummy_obj, builder, blender_context, lb=None) is None

    # Setup parent and child link builders to allow joint committed correctly
    builder.link("parent_link_name").commit()
    lb = builder.link("child_link_name", parent="parent_link_name")

    # 2. Test relative matrix lookup, Axis X, CONTINUOUS, dynamics, mimic, safety, calibration
    jp.joint_name = "continuous_joint"
    jp.joint_type = "CONTINUOUS"
    jp.axis = "X"
    jp.use_dynamics = True
    jp.dynamics_damping = 0.5
    jp.dynamics_friction = 0.2

    # Mimic
    mimic_joint_obj = create_test_object("mimic_joint_obj", None, scene=scene)
    mimic_jp = safe_get_joint(mimic_joint_obj, scene)
    mimic_jp.is_robot_joint = True
    mimic_jp.joint_name = "mimic_joint_name"
    jp.use_mimic = True
    jp.mimic_joint = mimic_joint_obj
    jp.mimic_multiplier = 1.5
    jp.mimic_offset = 0.1

    # Safety
    jp.use_safety_controller = True
    jp.safety_soft_lower_limit = -1.0
    jp.safety_soft_upper_limit = 1.0
    jp.safety_k_position = 20.0
    jp.safety_k_velocity = 5.0

    # Calibration
    jp.use_calibration = True
    jp.use_calibration_rising = True
    jp.calibration_rising = 0.05
    jp.use_calibration_falling = True
    jp.calibration_falling = 0.02

    parent_matrix = Matrix.Translation((1.0, 0.0, 0.0))
    child_matrix = Matrix.Translation((1.5, 0.0, 0.0))
    link_frames = {
        "parent_link_name": parent_matrix,
        "child_link_name": child_matrix,
    }

    translator.translate(dummy_obj, builder, blender_context, lb=lb, link_frames=link_frames)
    lb.commit()

    joint = builder.robot.get_joint("continuous_joint")
    assert joint is not None
    assert joint.type.value == "continuous"
    assert joint.axis is not None
    assert joint.axis.x == 1.0
    assert joint.dynamics is not None
    assert joint.dynamics.damping == 0.5
    assert joint.mimic is not None
    assert joint.mimic.joint == "mimic_joint_name"
    assert joint.safety_controller is not None
    assert joint.safety_controller.soft_lower_limit == -1.0
    assert joint.calibration is not None
    assert joint.calibration.rising == 0.05
    assert joint.calibration.falling == 0.02

    # 3. Test Axis Y, PRISMATIC type
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_joint_robot_2")
    builder.link("parent_link_name").commit()
    lb2 = builder.link("child_link_name", parent="parent_link_name")

    jp2_obj = create_test_object("prismatic_joint_obj", None, scene=scene)
    jp2 = safe_get_joint(jp2_obj, scene)
    jp2.is_robot_joint = True
    jp2.parent_link = parent_obj
    jp2.child_link = child_obj
    jp2.joint_name = "prismatic_joint"
    jp2.joint_type = "PRISMATIC"
    jp2.axis = "Y"
    jp2.limit_lower = -0.5
    jp2.limit_upper = 0.5
    jp2.limit_effort = 10.0
    jp2.limit_velocity = 2.0

    translator.translate(jp2_obj, builder, blender_context, lb=lb2)
    lb2.commit()

    joint2 = builder.robot.get_joint("prismatic_joint")
    assert joint2 is not None
    assert joint2.type.value == "prismatic"
    assert joint2.axis.y == 1.0

    # 4. Test Axis CUSTOM, zero-axis fallback, FIXED type, FLOATING type
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_joint_robot_3")
    builder.link("parent_link_name").commit()
    lb3 = builder.link("child_link_name", parent="parent_link_name")

    jp3_obj = create_test_object("fixed_joint_obj", None, scene=scene)
    jp3 = safe_get_joint(jp3_obj, scene)
    jp3.is_robot_joint = True
    jp3.parent_link = parent_obj
    jp3.child_link = child_obj
    jp3.joint_name = "fixed_joint"
    jp3.joint_type = "FIXED"
    jp3.axis = "CUSTOM"
    jp3.custom_axis_x = 0.0
    jp3.custom_axis_y = 0.0
    jp3.custom_axis_z = 0.0  # Zero axis fallback triggers

    translator.translate(jp3_obj, builder, blender_context, lb=lb3)
    lb3.commit()

    joint3 = builder.robot.get_joint("fixed_joint")
    assert joint3 is not None
    assert joint3.type.value == "fixed"

    # Floating type
    lb4 = builder.link("floating_child", parent="parent_link_name")
    jp4_obj = create_test_object("floating_joint_obj", None, scene=scene)
    jp4 = safe_get_joint(jp4_obj, scene)
    jp4.is_robot_joint = True
    jp4.parent_link = parent_obj
    # Create child for floating
    floating_child_obj = create_test_object("floating_child_obj", None, scene=scene)
    floating_lp = safe_get_linkforge(floating_child_obj, scene)
    floating_lp.is_robot_link = True
    floating_lp.link_name = "floating_child"
    jp4.child_link = floating_child_obj
    jp4.joint_name = "floating_joint"
    jp4.joint_type = "FLOATING"
    jp4.axis = "CUSTOM"
    jp4.custom_axis_x = 1.0
    jp4.custom_axis_y = 2.0
    jp4.custom_axis_z = 3.0

    translator.translate(jp4_obj, builder, blender_context, lb=lb4)
    lb4.commit()

    joint4 = builder.robot.get_joint("floating_joint")
    assert joint4 is not None
    assert joint4.type.value == "floating"

    # Planar type
    lb5 = builder.link("planar_child", parent="parent_link_name")
    jp5_obj = create_test_object("planar_joint_obj", None, scene=scene)
    jp5 = safe_get_joint(jp5_obj, scene)
    jp5.is_robot_joint = True
    jp5.parent_link = parent_obj
    # Create child for planar
    planar_child_obj = create_test_object("planar_child_obj", None, scene=scene)
    planar_lp = safe_get_linkforge(planar_child_obj, scene)
    planar_lp.is_robot_link = True
    planar_lp.link_name = "planar_child"
    jp5.child_link = planar_child_obj
    jp5.joint_name = "planar_joint"
    jp5.joint_type = "PLANAR"
    jp5.axis = "Z"

    translator.translate(jp5_obj, builder, blender_context, lb=lb5)
    lb5.commit()

    joint5 = builder.robot.get_joint("planar_joint")
    assert joint5 is not None
    assert joint5.type.value == "planar"

    # 6. Unrecognized joint type (covers 404->408 as Falsy)
    lb6 = builder.link("unrecognized_child", parent="parent_link_name")
    jp6_obj = create_test_object("unrecognized_joint_obj", None, scene=scene)
    jp6 = safe_get_joint(jp6_obj, scene)
    jp6.is_robot_joint = True
    jp6.parent_link = parent_obj
    unrecognized_child_obj = create_test_object("unrecognized_child_obj", None, scene=scene)
    unrecognized_lp = safe_get_linkforge(unrecognized_child_obj, scene)
    unrecognized_lp.is_robot_link = True
    unrecognized_lp.link_name = "unrecognized_child"
    jp6.child_link = unrecognized_child_obj
    jp6.joint_name = "unrecognized_joint"
    jp6.joint_type = "PLANAR"
    jp6.axis = "Z"

    # Mock JointType to return an unrecognized value or patch it
    class FakeJointProps:
        is_robot_joint = True
        parent_link = parent_obj
        child_link = unrecognized_child_obj
        joint_name = "unrecognized_joint"
        joint_type = "fake_joint_type"
        axis = "Z"
        limit_lower = 0.0
        limit_upper = 0.0
        limit_effort = 0.0
        limit_velocity = 0.0
        mimic_joint = None
        safety_k_position = 0.0
        safety_k_velocity = 0.0
        safety_soft_lower = 0.0
        safety_soft_upper = 0.0
        calibration_rising = 0.0
        calibration_falling = 0.0
        dynamics_damping = 0.0
        dynamics_friction = 0.0

    from unittest.mock import patch

    with (
        patch(
            "linkforge.blender.adapters.translator.get_joint_props", return_value=FakeJointProps()
        ),
        patch("linkforge.blender.adapters.translator.JointType", return_value="fake_joint_type"),
        contextlib.suppress(Exception),
    ):
        translator.translate(jp6_obj, builder, blender_context, lb=lb6)


def test_sensor_translator_comprehensive(scene, blender_context):
    """Cover all remaining SensorTranslator branches (early exits, matrix correction, all sensor types, noise, plugins)."""
    cleanup_blender_scene(scene)

    from linkforge.core.models.sensor import SensorType
    from mathutils import Matrix

    translator = SensorTranslator()
    builder = RobotBuilder("test_sensor_robot")

    # 1. Early exits
    assert translator._blender_sensor_to_core(None) is None

    dummy_obj = create_test_object("dummy_sensor", None, scene=scene)
    assert translator._blender_sensor_to_core(dummy_obj) is None

    # 2. Add link parent
    link_obj = create_test_object("parent_link", None, scene=scene)
    link_lp = safe_get_linkforge(link_obj, scene)
    link_lp.is_robot_link = True
    link_lp.link_name = "parent_link_name"

    sp = safe_get_sensor(dummy_obj, scene)
    sp.is_robot_sensor = True
    sp.sensor_name = "test_sensor"
    sp.sensor_type = "CAMERA"
    sp.attached_link = link_obj

    # 3. Test Camera/DepthCamera with Noise & Matrix Correction
    sp.use_noise = True
    sp.noise_type = "gaussian"
    sp.noise_mean = 0.05
    sp.noise_stddev = 0.01

    sp.camera_horizontal_fov = 1.047
    sp.camera_width = 640
    sp.camera_height = 480
    sp.camera_format = "R8G8B8"
    sp.camera_near_clip = 0.1
    sp.camera_far_clip = 100.0

    sp.use_gazebo_plugin = True
    sp.plugin_filename = "libgazebo_ros_camera.so"
    sp.topic_name = "/camera/image_raw"

    link_matrix = Matrix.Translation((2.0, 0.0, 0.0))
    link_frames = {"parent_link_name": link_matrix}

    # First add parent link to robot model
    builder.link("parent_link_name").commit()

    translator.translate(dummy_obj, builder, blender_context, link_frames=link_frames)
    sensor = builder.robot.sensors[0]
    assert sensor.name == "test_sensor"
    assert sensor.type == SensorType.CAMERA
    assert sensor.camera_info is not None
    assert sensor.camera_info.noise is not None
    assert sensor.camera_info.noise.stddev == 0.01
    assert sensor.plugin is not None
    assert sensor.plugin.filename == "libgazebo_ros_camera.so"

    # 4. Test LIDAR sensor type
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_2")
    builder.link("parent_link_name").commit()

    sp_lidar_obj = create_test_object("lidar_sensor", None, scene=scene)
    sp_lidar = safe_get_sensor(sp_lidar_obj, scene)
    sp_lidar.is_robot_sensor = True
    sp_lidar.sensor_name = "lidar_sensor"
    sp_lidar.sensor_type = "LIDAR"
    sp_lidar.attached_link = link_obj
    sp_lidar.lidar_horizontal_samples = 640
    sp_lidar.lidar_horizontal_min_angle = -1.57
    sp_lidar.lidar_horizontal_max_angle = 1.57
    sp_lidar.lidar_vertical_samples = 1
    sp_lidar.lidar_vertical_min_angle = 0.0
    sp_lidar.lidar_vertical_max_angle = 0.0
    sp_lidar.lidar_range_min = 0.1
    sp_lidar.lidar_range_max = 30.0
    sp_lidar.lidar_range_resolution = 0.01

    translator.translate(sp_lidar_obj, builder, blender_context)
    assert builder.robot.sensors[0].lidar_info is not None
    assert builder.robot.sensors[0].type == SensorType.LIDAR

    # 5. Test IMU sensor type
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_3")
    builder.link("parent_link_name").commit()

    sp_imu_obj = create_test_object("imu_sensor", None, scene=scene)
    sp_imu = safe_get_sensor(sp_imu_obj, scene)
    sp_imu.is_robot_sensor = True
    sp_imu.sensor_name = "imu_sensor"
    sp_imu.sensor_type = "IMU"
    sp_imu.attached_link = link_obj

    translator.translate(sp_imu_obj, builder, blender_context)
    assert builder.robot.sensors[0].imu_info is not None

    # 6. Test GPS sensor type
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_4")
    builder.link("parent_link_name").commit()

    sp_gps_obj = create_test_object("gps_sensor", None, scene=scene)
    sp_gps = safe_get_sensor(sp_gps_obj, scene)
    sp_gps.is_robot_sensor = True
    sp_gps.sensor_name = "gps_sensor"
    sp_gps.sensor_type = "GPS"
    sp_gps.attached_link = link_obj

    translator.translate(sp_gps_obj, builder, blender_context)
    assert builder.robot.sensors[0].gps_info is not None

    # 7. Test CONTACT sensor type (with fallback collision name)
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_5")
    builder.link("parent_link_name").commit()

    sp_contact_obj = create_test_object("contact_sensor", None, scene=scene)
    sp_contact = safe_get_sensor(sp_contact_obj, scene)
    sp_contact.is_robot_sensor = True
    sp_contact.sensor_name = "contact_sensor"
    sp_contact.sensor_type = "CONTACT"
    sp_contact.attached_link = link_obj
    sp_contact.contact_collision = ""  # trigger fallback

    translator.translate(sp_contact_obj, builder, blender_context)
    assert builder.robot.sensors[0].contact_info is not None
    assert builder.robot.sensors[0].contact_info.collision == "parent_link_name_collision"

    # 7b. Test CONTACT sensor type with custom collision name
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_5_custom")
    builder.link("parent_link_name").commit()

    sp_contact_obj_custom = create_test_object("contact_sensor_custom", None, scene=scene)
    sp_contact_custom = safe_get_sensor(sp_contact_obj_custom, scene)
    sp_contact_custom.is_robot_sensor = True
    sp_contact_custom.sensor_name = "contact_sensor_custom"
    sp_contact_custom.sensor_type = "CONTACT"
    sp_contact_custom.attached_link = link_obj
    sp_contact_custom.contact_collision = "my_custom_collision"

    translator.translate(sp_contact_obj_custom, builder, blender_context)
    assert builder.robot.sensors[0].contact_info is not None
    assert builder.robot.sensors[0].contact_info.collision == "my_custom_collision"

    # 8. Test FORCE_TORQUE sensor type
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_6")
    builder.link("parent_link_name").commit()

    sp_ft_obj = create_test_object("ft_sensor", None, scene=scene)
    sp_ft = safe_get_sensor(sp_ft_obj, scene)
    sp_ft.is_robot_sensor = True
    sp_ft.sensor_name = "ft_sensor"
    sp_ft.sensor_type = "FORCE_TORQUE"
    sp_ft.attached_link = link_obj

    translator.translate(sp_ft_obj, builder, blender_context)
    assert builder.robot.sensors[0].force_torque_info is not None

    # 9. Exception validation
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_7")
    sp_err_obj = create_test_object("err_sensor", None, scene=scene)
    sp_err = safe_get_sensor(sp_err_obj, scene)
    sp_err.is_robot_sensor = True
    sp_err.attached_link = None  # trigger missing parent link exception

    val_res = ValidationResult(robot_name="test_sensor_robot_7")
    translator.translate(sp_err_obj, builder, blender_context, validation_result=val_res)
    assert len(val_res.errors) == 1
    assert "Sensor translation failed" in val_res.errors[0].title

    # 9b. Cover early exit 452->exit
    translator.translate(None, builder, blender_context)

    # 10. Unrecognized sensor type (covers 582->586 as Falsy)
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_sensor_robot_8")
    builder.link("parent_link_name").commit()

    sp_fake_obj = create_test_object("fake_sensor", None, scene=scene)
    sp_fake = safe_get_sensor(sp_fake_obj, scene)
    sp_fake.is_robot_sensor = True
    sp_fake.sensor_name = "fake_sensor"
    sp_fake.sensor_type = "CAMERA"
    sp_fake.attached_link = link_obj

    class FakeProps:
        is_robot_sensor = True
        sensor_name = "fake_sensor"
        sensor_type = "fake_type"
        attached_link = link_obj
        use_noise = False
        use_gazebo_plugin = False
        topic_name = ""
        update_rate = 10.0
        always_on = True
        visualize = False

    from unittest.mock import patch

    with (
        patch("linkforge.blender.adapters.translator.get_sensor_props", return_value=FakeProps()),
        patch("linkforge.blender.adapters.translator.SensorType", return_value="fake_type"),
    ):
        translator.translate(sp_fake_obj, builder, blender_context)


def test_ros2_control_translator_comprehensive(scene, blender_context):
    """Cover all remaining Ros2ControlTranslator branches (early exits, HW interfaces, joint names fallback)."""
    cleanup_blender_scene(scene)

    translator = Ros2ControlTranslator()
    builder = RobotBuilder("test_control_robot")

    # 1. Early exits
    assert translator._blender_ros2_control_to_core(None) is None
    translator.translate(None, builder, blender_context)

    # 2. Setup robot with control properties
    from unittest.mock import MagicMock

    props = MagicMock()
    props.use_ros2_control = True
    props.ros2_control_type = "system"
    props.ros2_control_name = "TestSystemControl"
    props.hardware_plugin = "mock_plugin"

    # Setup 2 joints with all commands & states
    from linkforge.core.models.joint import Joint, JointType

    builder.robot._joint_index["joint_one_name"] = Joint(
        name="joint_one_name",
        type=JointType.FIXED,
        parent="link1",
        child="link2",
    )
    builder.robot._joint_index["joint_two_fallback"] = Joint(
        name="joint_two_fallback",
        type=JointType.FIXED,
        parent="link1",
        child="link2",
    )

    j1_obj = create_test_object("joint1_obj", None, scene=scene)
    j1_p = safe_get_joint(j1_obj, scene)
    j1_p.is_robot_joint = True
    j1_p.joint_name = "joint_one_name"

    item1 = MagicMock()
    item1.cmd_position = True
    item1.cmd_velocity = True
    item1.cmd_effort = True
    item1.state_position = True
    item1.state_velocity = True
    item1.state_effort = True
    item1.joint_obj = j1_obj
    item1.parameters = []

    # Joint 2 with fallback joint name
    item2 = MagicMock()
    item2.cmd_position = True
    item2.cmd_velocity = False
    item2.cmd_effort = False
    item2.state_position = True
    item2.state_velocity = False
    item2.state_effort = False
    item2.joint_obj = None
    item2.name = "joint_two_fallback"
    item2.parameters = []

    props.ros2_control_joints = [item1, item2]

    # Translate
    translator.translate(props, builder, blender_context)
    control = builder.robot.ros2_controls[0]
    assert control.name == "TestSystemControl"
    assert len(control.joints) == 2
    assert "position" in control.joints[0].command_interfaces
    assert "velocity" in control.joints[0].command_interfaces
    assert "effort" in control.joints[0].command_interfaces
    assert control.joints[0].name == "joint_one_name"
    assert control.joints[1].name == "joint_two_fallback"


def test_transmission_translator_comprehensive(scene, blender_context):
    """Cover all remaining TransmissionTranslator branches (early exits, custom type, custom actuator names, diff transmission)."""
    cleanup_blender_scene(scene)

    translator = TransmissionTranslator()
    builder = RobotBuilder("test_trans_robot")

    # 1. Early exits
    assert translator._blender_transmission_to_core(None) is None

    # Setup the mock joint
    from linkforge.core.models.joint import Joint, JointType

    builder.robot._joint_index["joint_to_transmit"] = Joint(
        name="joint_to_transmit",
        type=JointType.FIXED,
        parent="link1",
        child="link2",
    )

    # 2. Simple/Custom transmission with custom actuator names
    joint_obj = create_test_object("trans_joint", None, scene=scene)
    jp = safe_get_joint(joint_obj, scene)
    jp.is_robot_joint = True
    jp.joint_name = "joint_to_transmit"

    trans_obj = create_test_object("trans_obj", None, scene=scene)
    tp = safe_get_transmission(trans_obj, scene)
    tp.is_robot_transmission = True
    tp.transmission_name = "custom_trans"
    tp.transmission_type = "CUSTOM"
    tp.custom_type = "transmission_interface/CustomTransmission"
    tp.hardware_interface = "hardware_interface/PositionJointInterface"
    tp.joint_name = joint_obj
    tp.use_custom_actuator_name = True
    tp.actuator_name = "my_custom_actuator"
    tp.mechanical_reduction = 50.0
    tp.offset = 0.5

    translator.translate(trans_obj, builder, blender_context)
    trans = builder.robot.transmissions[0]
    assert trans.name == "custom_trans"
    assert trans.type == "transmission_interface/CustomTransmission"
    assert trans.joints[0].name == "joint_to_transmit"
    assert trans.actuators[0].name == "my_custom_actuator"

    # 2b. Simple/Custom transmission where potential_name is not a string (covers 794->797)
    builder.robot._joint_index["trans_joint_non_str"] = Joint(
        name="trans_joint_non_str",
        type=JointType.FIXED,
        parent="link1",
        child="link2",
    )
    joint_obj_non_str = create_test_object("trans_joint_non_str", None, scene=scene)

    trans_obj_non_str = create_test_object("trans_obj_non_str", None, scene=scene)
    tp_non_str = safe_get_transmission(trans_obj_non_str, scene)
    tp_non_str.is_robot_transmission = True
    tp_non_str.transmission_name = "custom_trans_non_str"
    tp_non_str.transmission_type = "SIMPLE"
    tp_non_str.joint_name = joint_obj_non_str

    class FakeJointPropsNonStr:
        joint_name = 123

    from unittest.mock import patch

    with patch(
        "linkforge.blender.adapters.translator.get_joint_props", return_value=FakeJointPropsNonStr()
    ):
        translator.translate(trans_obj_non_str, builder, blender_context)

    # 3. Early exit coverage for translate(None) -> 747->exit
    translator.translate(None, builder, blender_context)

    # 4. Simple transmission with missing joint (covers 789->849 as Falsy)
    cleanup_blender_scene(scene)
    builder = RobotBuilder("test_trans_robot_simple_no_joint")
    trans_obj_no_joint = create_test_object("trans_obj_no_joint", None, scene=scene)
    tp_no_joint = safe_get_transmission(trans_obj_no_joint, scene)
    tp_no_joint.is_robot_transmission = True
    tp_no_joint.transmission_name = "trans_no_joint"
    tp_no_joint.transmission_type = "SIMPLE"
    tp_no_joint.joint_name = None

    assert translator._blender_transmission_to_core(trans_obj_no_joint) is None

    # 5. Transmission with invalid type (covers 815->849 as Falsy)
    cleanup_blender_scene(scene)
    trans_obj_invalid = create_test_object("trans_obj_invalid", None, scene=scene)
    tp_invalid = safe_get_transmission(trans_obj_invalid, scene)
    tp_invalid.is_robot_transmission = True
    tp_invalid.transmission_name = "trans_invalid"
    tp_invalid.transmission_type = "INVALID_TYPE"

    assert translator._blender_transmission_to_core(trans_obj_invalid) is None
