from pathlib import Path
from unittest.mock import MagicMock, patch

from linkforge.blender.logic.asynchronous_builder import AsynchronousRobotBuilder
from linkforge.core import Joint, JointType, Link, Robot, RobotModelError


def test_builder_prepare_tasks(scene, blender_context) -> None:
    """Test that tasks are correctly queued based on robot structure."""
    l1 = Link(name="base_link")
    l2 = Link(name="link1")
    j1 = Joint(name="joint1", type=JointType.FIXED, parent="base_link", child="link1")

    robot = Robot(name="test_robot", links=[l1, l2], joints=[j1])

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), blender_context)

    # Expected tasks:
    # setup_scene
    # create_collection
    # create_link (base_link)
    # create_link (link1)
    # create_joint (joint1)
    # resolve_mimics
    # finalize

    task_types = [t[0] for t in builder.tasks]
    assert "setup_scene" in task_types
    assert "create_collection" in task_types
    assert task_types.count("create_link") == 2
    assert "create_joint" in task_types
    assert "resolve_mimics" in task_types
    assert "finalize" in task_types


def test_builder_execution_flow(scene, blender_context) -> None:
    """Test that process_next_chunk executes tasks and updates status."""
    l1 = Link(name="base_link")
    robot = Robot(name="test_robot", links=[l1])

    with (
        patch("linkforge.blender.logic.asynchronous_builder.setup_scene_for_robot"),
        patch(
            "linkforge.blender.logic.asynchronous_builder.create_link_object",
            return_value=MagicMock(),
        ),
    ):
        builder = AsynchronousRobotBuilder(
            robot, Path("/tmp/robot.urdf"), blender_context, chunk_size=1
        )

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
        assert scene.linkforge.import_status != ""


def test_builder_abort(scene, blender_context) -> None:
    """Test that import can be aborted via scene property."""
    # Add a link to ensure there are tasks to process
    robot = Robot(name="test_robot", links=[Link(name="base_link")])
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), blender_context)

    scene.linkforge.abort_import = True

    result = builder.process_next_chunk()

    assert result is None  # Timer stopped
    assert builder.is_finished is True
    assert "cancelled" in (builder.error or "").lower()


def test_builder_error_handling(scene, blender_context) -> None:
    """Test that exceptions in task execution are caught and reported."""
    robot = Robot(name="test_robot")
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), blender_context)

    # Force an error in _execute_task
    with patch.object(builder, "_execute_task", side_effect=RobotModelError("Boom")):
        result = builder.process_next_chunk()
        assert result is None
        assert builder.error == "Boom"
        assert builder.is_finished is True


def test_builder_timer_start(scene, blender_context) -> None:
    """Test that start() registers the timer."""
    robot = Robot(name="test_robot")
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), blender_context)

    with patch("bpy.app.timers.register") as mock_register:
        builder.start()

        mock_register.assert_called_once()
        # Callback should be process_next_chunk
        args, _ = mock_register.call_args
        assert args[0] == builder.process_next_chunk


def test_builder_timer_callback_interval(scene, blender_context) -> None:
    """Test that the callback returns a float interval while running."""
    # Add many tasks so it doesn't finish immediately
    robot = Robot(name="test_robot", links=[Link(name=f"link{i}") for i in range(10)])

    builder = AsynchronousRobotBuilder(
        robot, Path("/tmp/robot.urdf"), blender_context, chunk_size=1
    )

    # Mock task execution to avoid real Blender calls
    with patch.object(builder, "_execute_task"):
        result = builder.process_next_chunk()
        # Should return real-time interval (float)
        assert isinstance(result, float)
        assert result > 0

    # Abort to finish
    scene.linkforge.abort_import = True

    result = builder.process_next_chunk()
    assert result is None  # Finished


def test_builder_full_completion(scene, blender_context) -> None:
    """Test that builder runs all tasks and finishes correctly."""
    robot = Robot(name="test_robot", links=[Link(name="link1")])

    # Mock all task executors
    with (
        patch("linkforge.blender.logic.asynchronous_builder.setup_scene_for_robot"),
        patch("linkforge.blender.logic.asynchronous_builder.create_link_object"),
        patch("linkforge.blender.logic.asynchronous_builder.create_joint_object"),
    ):
        # Chunk size logic: set to 1 to run one by one if desired, or large to finish at once
        builder = AsynchronousRobotBuilder(
            robot, Path("/tmp/robot.urdf"), blender_context, chunk_size=100
        )

        # Run first chunk (should finish all since chunk_size=100 and only ~6 tasks)
        result = builder.process_next_chunk()
        assert result is None
        assert builder.is_finished is True
        assert builder.completed_tasks == builder.total_tasks
        assert builder.error is None


def test_builder_with_joints_and_sensors(scene, blender_context) -> None:
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

    robot = Robot(name="robot", links=[l1, l2], joints=[j1], sensors=[s1])

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), blender_context)

    task_types = [t[0] for t in builder.tasks]
    assert "create_joint" in task_types
    assert "create_sensor" in task_types
    assert "resolve_mimics" in task_types
    assert "finalize" in task_types


def test_builder_start_without_robot_properties() -> None:
    """Verify builder initialization and start when the scene is missing robot properties and window manager is null."""
    mock_scene = MagicMock(spec=[])  # no attributes/properties
    mock_context = MagicMock()
    mock_context.scene = mock_scene
    mock_context.window_manager = None
    robot = Robot(name="test_robot")
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), mock_context)
    with patch("bpy.app.timers.register"):
        builder.start()
    assert builder.active_scene == mock_scene
    builder.finish()
    assert builder.is_finished is True


def test_builder_empty_task_queue_processing() -> None:
    """Verify that process_next_chunk completes immediately if the task queue is empty and window_manager is None."""
    mock_context = MagicMock()
    mock_context.scene = None
    mock_context.window_manager = None
    l1 = Link(name="l1")
    robot = Robot(name="robot", links=[l1])
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), mock_context)
    builder.tasks = []
    result = builder.process_next_chunk()
    assert result is None
    assert builder.is_finished is True


def test_builder_process_joints_and_sensors_chunks(blender_context) -> None:
    """Verify sequential builder task execution runs successfully and updates import status for joints and sensors."""
    l1_node = Link(name="l1")
    l2_node = Link(name="l2")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
    s1 = MagicMock()
    s1.name = "s1"
    s1.link_name = "l1"
    with (
        patch("linkforge.blender.logic.asynchronous_builder.setup_scene_for_robot"),
        patch(
            "linkforge.blender.logic.asynchronous_builder.create_link_object",
            return_value=MagicMock(),
        ),
        patch(
            "linkforge.blender.logic.asynchronous_builder.create_joint_object",
            return_value=MagicMock(),
        ),
        patch("linkforge.blender.logic.asynchronous_builder.create_sensor_object"),
    ):
        robot = Robot(name="robot", links=[l1_node, l2_node], joints=[j1], sensors=[s1])
        builder = AsynchronousRobotBuilder(
            robot, Path("/tmp/robot.urdf"), blender_context, chunk_size=1
        )
        # Process first chunk: setup_scene
        builder.process_next_chunk()
        # Process second chunk: create_collection
        builder.process_next_chunk()
        # Process third chunk: create_link (l1)
        builder.process_next_chunk()
        # Process fourth chunk: create_link (l2)
        builder.process_next_chunk()
        # Process fifth chunk: create_joint (j1) -> updates status
        builder.process_next_chunk()
        # Process sixth chunk: resolve_mimics
        builder.process_next_chunk()
        # Process seventh chunk: create_sensor (s1)
        builder.process_next_chunk()
        # Process eighth chunk: finalize
        builder.process_next_chunk()
        assert builder.is_finished is True


def test_builder_task_exits_on_null_scene() -> None:
    """Verify setup, collection, link, and joint creation tasks handle a null scene or empty objects gracefully."""
    mock_context = MagicMock()
    mock_context.scene = None
    builder = AsynchronousRobotBuilder(Robot(name="robot"), Path("/tmp/robot.urdf"), mock_context)

    # setup_scene with None scene
    builder._execute_task("setup_scene", None)

    # create_collection with None scene
    builder.collection = MagicMock()
    builder._execute_task("create_collection", None)

    # create_link returning None
    with patch(
        "linkforge.blender.logic.asynchronous_builder.create_link_object", return_value=None
    ):
        builder._execute_task("create_link", Link(name="l1"))

    # create_joint returning None
    with patch(
        "linkforge.blender.logic.asynchronous_builder.create_joint_object", return_value=None
    ):
        builder._execute_task("create_joint", MagicMock())

    # finalize with None scene
    builder._execute_task("finalize", None)


def test_builder_finalize_without_view_layer_or_ros2_control() -> None:
    """Verify finalization processes successfully with missing view layers and when ROS 2 control is disabled."""
    mock_context = MagicMock()
    mock_context.scene = MagicMock(spec=["linkforge_robot"])
    mock_context.scene.linkforge_robot = MagicMock(use_ros2_control=False)
    mock_context.view_layer = None
    builder = AsynchronousRobotBuilder(Robot(name="robot"), Path("/tmp/robot.urdf"), mock_context)
    builder._execute_task("finalize", None)


def test_builder_finalize_ros2_control_joint_mapping() -> None:
    """Verify builder's finalization task correctly links active ROS 2 Control joint items to the created scene objects."""

    class DummyJointItem:
        def __init__(self, name):
            self.name = name
            self.joint_obj = None

    rc_joint = DummyJointItem("joint1")
    rc_joint2 = DummyJointItem("joint2")  # Unmatched name to cover target_obj is None loop branch

    mock_context = MagicMock()
    mock_scene = MagicMock(spec=["linkforge_robot"])
    mock_context.scene = mock_scene
    mock_context.view_layer = None

    mock_scene.linkforge_robot = MagicMock(
        use_ros2_control=True, ros2_control_joints=[rc_joint, rc_joint2], show_collisions=True
    )

    builder = AsynchronousRobotBuilder(Robot(name="robot"), Path("/tmp/robot.urdf"), mock_context)
    mock_joint_obj = MagicMock()
    builder.joint_objects["joint1"] = mock_joint_obj
    builder._execute_task("finalize", None)
    assert rc_joint.joint_obj == mock_joint_obj
    assert rc_joint2.joint_obj is None


def test_builder_task_exception_handling() -> None:
    """Verify that when a task raises an exception, the builder logs it and propagates the exception."""
    import pytest

    mock_context = MagicMock()
    builder = AsynchronousRobotBuilder(Robot(name="robot"), Path("/tmp/robot.urdf"), mock_context)
    with (
        patch(
            "linkforge.blender.logic.asynchronous_builder.setup_scene_for_robot",
            side_effect=ValueError("Test exception"),
        ),
        pytest.raises(ValueError, match="Test exception"),
    ):
        builder._execute_task("setup_scene", None)


def test_builder_process_chunks_without_window_manager() -> None:
    """Verify process_next_chunk execution when window_manager is None."""
    mock_context = MagicMock()
    mock_context.scene = None
    mock_context.window_manager = None
    robot = Robot(name="robot")
    builder = AsynchronousRobotBuilder(robot, Path("/tmp/robot.urdf"), mock_context)
    builder.tasks = [("dummy", None)]
    with patch.object(builder, "_execute_task"):
        builder.process_next_chunk()
    assert builder.is_finished is True


def test_builder_unknown_task_type() -> None:
    """Verify that _execute_task safely ignores unknown task types, covering the implicit else branch of the task_type if-elif chain."""
    mock_context = MagicMock()
    builder = AsynchronousRobotBuilder(Robot(name="robot"), Path("/tmp/robot.urdf"), mock_context)
    # This should run without raising any exceptions and execute the false path of all ifs/elifs
    builder._execute_task("unknown_task_type", None)
