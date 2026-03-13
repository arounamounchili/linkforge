from pathlib import Path
from unittest.mock import MagicMock

import bpy
from linkforge.blender.logic.asynchronous_builder import AsynchronousRobotBuilder
from linkforge.linkforge_core.exceptions import RobotModelError
from linkforge_core.models import Joint, JointType, Link, Robot


def test_builder_prepare_tasks():
    """Test that tasks are correctly queued based on robot structure."""
    l1 = Link(name="base_link")
    l2 = Link(name="link1")
    j1 = Joint(name="joint1", type=JointType.FIXED, parent="base_link", child="link1")

    robot = Robot(name="test_robot", initial_links=[l1, l2], initial_joints=[j1])

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context)

    # Expected tasks:
    # 1. setup_scene
    # 2. create_collection
    # 3. create_link (base_link)
    # 4. create_link (link1)
    # 5. create_joint (joint1)
    # 6. resolve_mimics
    # 7. finalize

    task_types = [t[0] for t in builder.tasks]
    assert "setup_scene" in task_types
    assert "create_collection" in task_types
    assert task_types.count("create_link") == 2
    assert "create_joint" in task_types
    assert "resolve_mimics" in task_types
    assert "finalize" in task_types


def test_builder_execution_flow(mocker):
    """Test that process_next_chunk executes tasks and updates status."""
    l1 = Link(name="base_link")
    robot = Robot(name="test_robot", initial_links=[l1])

    # Mock scenebuilder functions
    mocker.patch("linkforge.blender.logic.asynchronous_builder.setup_scene_for_robot")
    mocker.patch(
        "linkforge.blender.logic.asynchronous_builder.create_link_object", return_value=MagicMock()
    )

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context, chunk_size=1)

    # Manually run chunks
    # Chunk 1: setup_scene
    builder.process_next_chunk()
    assert builder.completed_tasks == 1

    # Chunk 2: create_collection
    builder.process_next_chunk()
    assert builder.completed_tasks == 2

    # Chunk 3: create_link
    builder.process_next_chunk()
    assert builder.completed_tasks == 3
    # Check that status was updated
    assert bpy.context.scene.linkforge.import_status != ""


def test_builder_abort(mocker):
    """Test that import can be aborted via scene property."""
    robot = Robot(name="test_robot")
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context)

    bpy.context.scene.linkforge.abort_import = True
    result = builder.process_next_chunk()

    assert result is None  # Timer stopped
    assert builder.is_finished is True
    assert "cancelled" in builder.error.lower()


def test_builder_error_handling(mocker):
    """Test that exceptions in task execution are caught and reported."""
    robot = Robot(name="test_robot")
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context)

    # Force an error in _execute_task
    mocker.patch.object(builder, "_execute_task", side_effect=RobotModelError("Boom"))

    result = builder.process_next_chunk()
    assert result is None
    assert builder.error == "Boom"
    assert builder.is_finished is True


def test_builder_timer_start(mocker):
    """Test that start() registers the timer."""
    robot = Robot(name="test_robot")
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context)

    mock_register = mocker.patch("bpy.app.timers.register")
    builder.start()

    mock_register.assert_called_once()
    # Callback should be process_next_chunk
    args, _ = mock_register.call_args
    assert args[0] == builder.process_next_chunk


def test_builder_timer_callback_interval(mocker):
    """Test that the callback returns a float interval while running."""
    # Add many tasks so it doesn't finish immediately
    robot = Robot(name="test_robot", initial_links=[Link(name=f"link{i}") for i in range(10)])

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context, chunk_size=1)

    # Mock task execution to avoid real Blender calls
    mocker.patch.object(builder, "_execute_task")

    result = builder.process_next_chunk()
    # Should return real-time interval (float)
    assert isinstance(result, float)
    assert result > 0

    # Abort to finish
    bpy.context.scene.linkforge.abort_import = True
    result = builder.process_next_chunk()
    assert result is None  # Finished


def test_builder_full_completion(mocker):
    """Test that builder runs all tasks and finishes correctly."""
    robot = Robot(name="test_robot", initial_links=[Link(name="link1")])

    # Mock all task executors
    mocker.patch("linkforge.blender.logic.asynchronous_builder.setup_scene_for_robot")
    mocker.patch("linkforge.blender.logic.asynchronous_builder.create_link_object")
    mocker.patch("linkforge.blender.logic.asynchronous_builder.create_joint_object")

    # Chunk size logic: set to 1 to run one by one if desired, or large to finish at once
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context, chunk_size=100)

    # Run first chunk (should finish all since chunk_size=100 and only ~6 tasks)
    result = builder.process_next_chunk()
    assert result is None
    assert builder.is_finished is True
    assert builder.completed_tasks == builder.total_tasks
    assert builder.error is None


def test_builder_with_joints_and_sensors(mocker):
    """Test that builder correctly queues joints and sensors."""
    l1 = Link(name="l1")
    l2 = Link(name="l2")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
    # Mock sensor as it's just a data object for the builder
    s1 = MagicMock()
    s1.name = "s1"
    s1.link_name = "l1"
    # Use mock with proper interface or specify type if needed
    # Here we just need it in the sensors list

    robot = Robot(name="robot", initial_links=[l1, l2], initial_joints=[j1], initial_sensors=[s1])

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), bpy.context)

    task_types = [t[0] for t in builder.tasks]
    assert "create_joint" in task_types
    assert "create_sensor" in task_types
    assert "resolve_mimics" in task_types
    assert "finalize" in task_types
