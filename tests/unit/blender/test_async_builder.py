from pathlib import Path
from unittest.mock import MagicMock

import bpy
from linkforge.blender.asynchronous_builder import AsynchronousRobotBuilder
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
    mocker.patch("linkforge.blender.asynchronous_builder.setup_scene_for_robot")
    mocker.patch(
        "linkforge.blender.asynchronous_builder.create_link_object", return_value=MagicMock()
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
    mocker.patch.object(builder, "_execute_task", side_effect=ValueError("Boom"))

    result = builder.process_next_chunk()
    assert result is None
    assert builder.error == "Boom"
    assert builder.is_finished is True
