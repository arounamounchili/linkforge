from unittest.mock import MagicMock

import bpy
from linkforge.blender.operators.control_ops import (
    LINKFORGE_OT_add_ros2_control_joint,
    LINKFORGE_OT_move_ros2_control_joint,
    LINKFORGE_OT_remove_ros2_control_joint,
    register,
    unregister,
)


def test_add_ros2_control_joint_execute(mocker, clean_scene) -> None:
    """Test adding a joint using real collection properties."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.ros2_control_joints.clear()

    # Mock actual joint in scene for the new joint_obj reference fetch logic
    joint_obj = bpy.data.objects.new("joint1", None)
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "joint1"
    scene.collection.objects.link(joint_obj)

    mock_self = MagicMock()
    mock_self.report = MagicMock()
    mock_self.joint_name = "joint1"

    # Execute
    result = LINKFORGE_OT_add_ros2_control_joint.execute(mock_self, bpy.context)

    assert result == {"FINISHED"}
    assert len(props.ros2_control_joints) == 1
    assert props.ros2_control_joints[0].name == "joint1"
    assert props.ros2_control_joints[0].joint_obj == joint_obj
    mock_self.report.assert_called_with({"INFO"}, mocker.ANY)


def test_remove_ros2_control_joint_execute(mocker, clean_scene) -> None:
    """Test removing a joint using real collection properties."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.ros2_control_joints.clear()

    # Add a joint manually
    joint = props.ros2_control_joints.add()
    joint.name = "joint1"
    props.ros2_control_active_joint_index = 0

    mock_self = MagicMock()
    mock_self.report = MagicMock()

    # Execute
    result = LINKFORGE_OT_remove_ros2_control_joint.execute(mock_self, bpy.context)

    assert result == {"FINISHED"}
    assert len(props.ros2_control_joints) == 0
    mock_self.report.assert_called_with({"INFO"}, mocker.ANY)


def test_move_ros2_control_joint_execute(mocker, clean_scene) -> None:
    """Test moving a joint using real collection properties."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.ros2_control_joints.clear()

    # Add two joints
    j1 = props.ros2_control_joints.add()
    j1.name = "j1"
    j2 = props.ros2_control_joints.add()
    j2.name = "j2"

    props.ros2_control_active_joint_index = 1  # Select j2

    mock_self = MagicMock()
    mock_self.direction = "UP"

    # Execute (Move j2 UP to index 0)
    result = LINKFORGE_OT_move_ros2_control_joint.execute(mock_self, bpy.context)

    assert result == {"FINISHED"}
    assert props.ros2_control_joints[0].name == "j2"
    assert props.ros2_control_active_joint_index == 0


def test_control_ops_add_failures(mocker, clean_scene) -> None:
    """Test add_ros2_control_joint error branches."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.ros2_control_joints.clear()

    # 1. Duplicate check by name
    joint = props.ros2_control_joints.add()
    joint.name = "duplicate_joint"

    mock_self = MagicMock()
    mock_self.joint_name = "duplicate_joint"

    res = LINKFORGE_OT_add_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"CANCELLED"}

    # 2. Duplicate check by object reference
    props.ros2_control_joints.clear()
    joint_obj = bpy.data.objects.new("test_joint_obj", None)
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "new_name"  # Physical object name is new
    scene.collection.objects.link(joint_obj)

    joint = props.ros2_control_joints.add()
    joint.name = "old_stale_name"
    joint.joint_obj = joint_obj

    mock_self.joint_name = "new_name"  # Trying to add it under its new name
    res = LINKFORGE_OT_add_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"CANCELLED"}


def test_control_ops_remove_failures(clean_scene) -> None:
    """Test remove_ros2_control_joint edge cases."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.ros2_control_joints.clear()

    # 1. Invalid index
    props.ros2_control_active_joint_index = -1
    mock_self = MagicMock()
    res = LINKFORGE_OT_remove_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"CANCELLED"}

    # 2. Out of bounds index
    props.ros2_control_active_joint_index = 10
    res = LINKFORGE_OT_remove_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"CANCELLED"}


def test_control_ops_move_variants(clean_scene) -> None:
    """Test move_ros2_control_joint direction and boundary branches."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.ros2_control_joints.clear()

    # Need 3 joints to test move logic properly
    j1 = props.ros2_control_joints.add()
    j1.name = "j1"
    j2 = props.ros2_control_joints.add()
    j2.name = "j2"
    j3 = props.ros2_control_joints.add()
    j3.name = "j3"

    mock_self = MagicMock()

    # 1. Move DOWN from index 1 to 2
    props.ros2_control_active_joint_index = 1
    mock_self.direction = "DOWN"
    res = LINKFORGE_OT_move_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"FINISHED"}
    assert props.ros2_control_joints[2].name == "j2"
    assert props.ros2_control_active_joint_index == 2

    # 2. Move DOWN from BOTTOM (should cancel)
    res = LINKFORGE_OT_move_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"CANCELLED"}

    # 3. Move UP from index 2 to 1
    mock_self.direction = "UP"
    res = LINKFORGE_OT_move_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"FINISHED"}
    assert props.ros2_control_joints[1].name == "j2"

    # 4. Move UP from index 0 (should cancel)
    props.ros2_control_active_joint_index = 0
    res = LINKFORGE_OT_move_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"CANCELLED"}

    # 5. Invalid direction
    mock_self.direction = "SIDEWAYS"
    res = LINKFORGE_OT_move_ros2_control_joint.execute(mock_self, bpy.context)
    assert res == {"CANCELLED"}


def test_control_ops_polls_extended(clean_scene) -> None:
    """Test poll success/fail cases."""
    scene = bpy.context.scene
    scene.linkforge.ros2_control_joints.clear()

    # Initial state: NO joints
    assert LINKFORGE_OT_add_ros2_control_joint.poll(bpy.context) is True
    assert LINKFORGE_OT_remove_ros2_control_joint.poll(bpy.context) is False
    assert LINKFORGE_OT_move_ros2_control_joint.poll(bpy.context) is False

    # Add one joint
    scene.linkforge.ros2_control_joints.add()
    assert LINKFORGE_OT_remove_ros2_control_joint.poll(bpy.context) is True
    assert LINKFORGE_OT_move_ros2_control_joint.poll(bpy.context) is False

    # Add second joint
    scene.linkforge.ros2_control_joints.add()
    assert LINKFORGE_OT_move_ros2_control_joint.poll(bpy.context) is True


def test_control_ops_registry() -> None:
    """Target registration branches."""
    # unregister first to hit RobotModelErrors if any, then register
    unregister()
    register()
    # Re-register to hit the "except RobotModelError" branch (class already registered)
    register()


def test_control_ops_no_scene_fail(mocker) -> None:
    """Extreme case: context.scene is missing."""
    mock_context = MagicMock()
    mock_context.scene = None

    mock_self = MagicMock()
    # execute branches
    assert LINKFORGE_OT_add_ros2_control_joint.execute(mock_self, mock_context) == {"CANCELLED"}
    assert LINKFORGE_OT_remove_ros2_control_joint.execute(mock_self, mock_context) == {"CANCELLED"}
    assert LINKFORGE_OT_move_ros2_control_joint.execute(mock_self, mock_context) == {"CANCELLED"}

    # poll branches
    assert LINKFORGE_OT_remove_ros2_control_joint.poll(mock_context) is False
    assert LINKFORGE_OT_move_ros2_control_joint.poll(mock_context) is False
