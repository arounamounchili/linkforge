"""Asynchronous Robot Builder for Blender.

This module provides an asynchronous task runner for importing robot models
into Blender without blocking the UI. It uses `bpy.app.timers` to process
the robot structure in chunks, allowing for a responsive UI and progress updates.
"""

from __future__ import annotations

import typing
from pathlib import Path

import bpy
from linkforge.core import Robot, get_logger

from ..adapters.context import IBlenderContext
from ..adapters.core_to_blender import (
    create_joint_object,
    create_link_object,
    create_sensor_object,
    setup_scene_for_robot,
)
from ..constants import (
    PROP_ROBOT,
)
from ..utils.joint_utils import resolve_mimic_joints

logger = get_logger(__name__)


class AsynchronousRobotBuilder:
    """Task runner for asynchronous robot import."""

    def __init__(
        self,
        robot: Robot,
        source_path: Path,
        context: IBlenderContext,
        chunk_size: int = 50,
    ):
        self.robot = robot
        self.source_path = source_path
        self.context = context
        self.chunk_size = chunk_size

        self.collection: bpy.types.Collection | None = None
        self.link_objects: dict[str, bpy.types.Object] = {}
        self.joint_objects: dict[str, bpy.types.Object] = {}

        # Task queue
        self.tasks: list[tuple[str, typing.Any]] = []
        self._prepare_tasks()

        self.total_tasks = len(self.tasks)
        self.completed_tasks = 0

        self.is_finished = False
        self.error: str | None = None
        self.active_scene: bpy.types.Scene | None = None

    def _prepare_tasks(self) -> None:
        """Build the list of tasks to be performed."""
        if not self.robot.links:
            logger.warning(
                f"Robot '{self.robot.name}' has no links. The resulting Blender collection will be empty."
            )

        # 1. Setup Scene (ROS 2 Control, Gazebo, etc.)
        self.tasks.append(("setup_scene", None))

        # 2. Create collection
        self.tasks.append(("create_collection", None))

        # 3. Create link tasks
        for link in self.robot.links:
            self.tasks.append(("create_link", link))

        # 4. Create sorted joint tasks
        sorted_joints = self.robot.graph.get_topological_joints()
        for joint in sorted_joints:
            self.tasks.append(("create_joint", joint))

        # 5. Mimic joints resolution
        self.tasks.append(("resolve_mimics", None))

        # 6. Sensors
        for sensor in self.robot.sensors:
            self.tasks.append(("create_sensor", sensor))

        # 7. Finalization
        self.tasks.append(("finalize", None))

    def start(self) -> None:
        """Register the timer and start processing."""
        logger.info(f"Starting asynchronous import of '{self.robot.name}'...")

        # Setup background state (store scene locally to avoid context sensitivity)
        self.active_scene = self.context.scene or (bpy.data.scenes[0] if bpy.data.scenes else None)
        if self.active_scene and hasattr(self.active_scene, PROP_ROBOT):
            lp = getattr(self.active_scene, PROP_ROBOT)  # pyright: ignore[reportAttributeAccessIssue]
            lp.is_importing = True
            lp.abort_import = False
            lp.import_status = "Starting..."

        # Setup progress bar
        if self.context.window_manager:
            self.context.window_manager.progress_begin(0, self.total_tasks)

        # Register timer
        bpy.app.timers.register(self.process_next_chunk)

    def process_next_chunk(self) -> float | None:
        """Process a chunk of tasks. Return interval or None to stop."""
        # Use stored active_scene if start() was called, otherwise fallback to context (for unit tests)
        scene = (
            self.active_scene
            or self.context.scene
            or (bpy.data.scenes[0] if bpy.data.scenes else None)
        )

        # Use the stored scene to check for cancellation, immune to context changes
        if scene and hasattr(scene, PROP_ROBOT) and getattr(scene, PROP_ROBOT).abort_import:  # pyright: ignore[reportAttributeAccessIssue]
            logger.warning("Import aborted by user.")
            self.error = "Import cancelled by user."
            self.finish()
            return None

        if not self.tasks:
            self.finish()
            return None

        try:
            processed_count = 0
            current_status = ""

            while self.tasks and processed_count < self.chunk_size:
                task_type, data = self.tasks.pop(0)

                # Update status text based on task
                if task_type == "create_link":
                    current_status = f"Importing Link: {data.name}..."
                elif task_type == "create_joint":
                    current_status = f"Importing Joint: {data.name}..."

                self._execute_task(task_type, data)
                processed_count += 1
                self.completed_tasks += 1

            # Update UI
            if current_status and scene and hasattr(scene, PROP_ROBOT):
                getattr(scene, PROP_ROBOT).import_status = current_status  # pyright: ignore[reportAttributeAccessIssue]

            if self.context.window_manager:
                self.context.window_manager.progress_update(self.completed_tasks)

            if not self.tasks:
                self.finish()
                return None

            return 0.001

        except Exception as e:
            self.error = str(e)
            logger.error(f"Asynchronous import failed: {e}")
            self.finish()
            return None

    def _execute_task(self, task_type: str, data: typing.Any) -> None:
        """Execute a single unit of work."""
        try:
            if task_type == "setup_scene":
                if self.context.scene:
                    setup_scene_for_robot(self.context, self.robot)

            elif task_type == "create_collection":
                self.collection = self.context.data.collections.new(self.robot.name)
                if self.context.scene:
                    self.context.scene.collection.children.link(self.collection)

            elif task_type == "create_link":
                obj = create_link_object(
                    self.context, data, self.robot, self.source_path.parent, self.collection
                )
                if obj:
                    self.link_objects[data.name] = obj

            elif task_type == "create_joint":
                obj = create_joint_object(self.context, data, self.link_objects, self.collection)
                if obj:
                    self.joint_objects[data.name] = obj

            elif task_type == "resolve_mimics":
                # Convert to list for type-safety with mimic resolver
                joints_list = list(self.robot.joints)
                resolve_mimic_joints(joints_list, self.joint_objects)

            elif task_type == "create_sensor":
                create_sensor_object(self.context, data, self.link_objects, self.collection)

            elif task_type == "finalize":
                if self.context.view_layer is not None:
                    self.context.view_layer.update()

                # Sync collision visibility
                scene = self.context.scene
                if scene and hasattr(scene, PROP_ROBOT):
                    # Force update collision visibility toggle
                    lp_tmp = getattr(scene, PROP_ROBOT)
                    lp_tmp.show_collisions = lp_tmp.show_collisions  # pyright: ignore[reportAttributeAccessIssue]

                    # Auto-link ROS 2 Control joint pointers to newly created objects
                    # Match by persistent robot model identity (source_name_stored)
                    lp = getattr(scene, PROP_ROBOT)
                    if lp.use_ros2_control:
                        for rc_joint in lp.ros2_control_joints:
                            # Find the joint object in the current import set
                            target_obj = self.joint_objects.get(rc_joint.name)
                            if target_obj:
                                rc_joint.joint_obj = target_obj
                                logger.debug(
                                    f"Auto-linked ROS2 Control joint '{rc_joint.name}' to {target_obj.name}"
                                )
        except Exception as e:
            logger.debug(f"Task {task_type} failed: {e}")
            raise

    def finish(self) -> None:
        """Clean up and finalize."""
        if self.context.window_manager:
            self.context.window_manager.progress_end()
        self.is_finished = True

        # Clear background state
        scene = self.context.scene or (bpy.data.scenes[0] if bpy.data.scenes else None)
        if scene and hasattr(scene, PROP_ROBOT):
            lp = getattr(scene, PROP_ROBOT)  # pyright: ignore[reportAttributeAccessIssue]
            lp.is_importing = False
            lp.import_status = ""
            lp.abort_import = False

        if self.error:
            # Report error if cancelled or failed
            logger.info(f"Asynchronous import ended: {self.error}")
        else:
            logger.info(f"Asynchronous import complete - '{self.robot.name}' is ready.")
