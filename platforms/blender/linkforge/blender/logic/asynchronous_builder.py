"""Asynchronous Robot Builder for Blender.

This module provides an asynchronous task runner for importing robot models
into Blender without blocking the UI. It uses `bpy.app.timers` to process
the robot structure in chunks, allowing for a responsive UI and progress updates.
"""

from __future__ import annotations

import typing
from pathlib import Path

import bpy

from ...linkforge_core.logging_config import get_logger
from ...linkforge_core.models import Robot
from ...linkforge_core.utils.kinematics import sort_joints_topological
from ..adapters.core_to_blender import (
    create_joint_object,
    create_link_object,
    create_sensor_object,
    setup_scene_for_robot,
)
from ..utils.joint_utils import resolve_mimic_joints

logger = get_logger(__name__)


class AsynchronousRobotBuilder:
    """Task runner for asynchronous robot import."""

    def __init__(
        self,
        robot: Robot,
        urdf_path: Path,
        context: bpy.types.Context,
        chunk_size: int = 50,
    ):
        self.robot = robot
        self.urdf_path = urdf_path
        self.context = context
        self.chunk_size = chunk_size

        self.collection = None
        self.link_objects = {}
        self.joint_objects = {}

        # Task queue
        self.tasks = []
        self._prepare_tasks()

        self.total_tasks = len(self.tasks)
        self.completed_tasks = 0

        self.is_finished = False
        self.error = None

    def _prepare_tasks(self):
        """Build the list of tasks to be performed."""
        # 1. Setup Scene (ROS 2 Control, Gazebo, etc.)
        self.tasks.append(("setup_scene", None))

        # 2. Create collection
        self.tasks.append(("create_collection", None))

        # 3. Create link tasks
        for link in self.robot.links:
            self.tasks.append(("create_link", link))

        # 4. Create sorted joint tasks
        sorted_joints = sort_joints_topological(self.robot.joints, self.robot.links)
        for joint in sorted_joints:
            self.tasks.append(("create_joint", joint))

        # 5. Mimic joints resolution
        self.tasks.append(("resolve_mimics", None))

        # 6. Sensors
        if hasattr(self.robot, "sensors"):
            for sensor in self.robot.sensors:
                # Assuming SceneBuilder can handle sensors
                self.tasks.append(("create_sensor", sensor))

        # 7. Finalization
        self.tasks.append(("finalize", None))

    def start(self):
        """Register the timer and start processing."""
        logger.info(f"Starting asynchronous import of '{self.robot.name}'...")

        # Setup background state
        scene = self.context.scene
        if hasattr(scene, "linkforge"):
            scene.linkforge.is_importing = True
            scene.linkforge.abort_import = False
            scene.linkforge.import_status = "Starting..."

        # Setup progress bar
        self.context.window_manager.progress_begin(0, self.total_tasks)

        # Register timer
        bpy.app.timers.register(self.process_next_chunk)

    def process_next_chunk(self) -> float | None:
        """Process a chunk of tasks. Return interval or None to stop."""
        scene = self.context.scene

        # Check for cancellation
        if hasattr(scene, "linkforge") and scene.linkforge.abort_import:
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
            if current_status and hasattr(scene, "linkforge"):
                scene.linkforge.import_status = current_status

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
        if task_type == "setup_scene":
            if self.context.scene:
                setup_scene_for_robot(self.context.scene, self.robot)

        elif task_type == "create_collection":
            self.collection = bpy.data.collections.new(self.robot.name)
            if self.context.scene:
                self.context.scene.collection.children.link(self.collection)

        elif task_type == "create_link":
            obj = create_link_object(data, self.urdf_path.parent, self.collection)
            if obj:
                self.link_objects[data.name] = obj

        elif task_type == "create_joint":
            obj = create_joint_object(data, self.link_objects, self.collection)
            if obj:
                self.joint_objects[data.name] = obj

        elif task_type == "resolve_mimics":
            resolve_mimic_joints(self.robot.joints, self.joint_objects)

        elif task_type == "create_sensor":
            create_sensor_object(data, self.link_objects, self.collection)

        elif task_type == "finalize":
            if self.context.view_layer:
                self.context.view_layer.update()

            # Sync collision visibility
            scene = self.context.scene
            if hasattr(scene, "linkforge"):
                # Force update collision visibility if the property exist
                scene.linkforge.show_collisions = scene.linkforge.show_collisions

    def finish(self) -> None:
        """Clean up and finalize."""
        if self.context.window_manager:
            self.context.window_manager.progress_end()
        self.is_finished = True

        # Clear background state
        scene = self.context.scene
        if scene and hasattr(scene, "linkforge"):
            scene.linkforge.is_importing = False
            scene.linkforge.import_status = ""
            scene.linkforge.abort_import = False

        if self.error:
            # Report error if cancelled or failed
            logger.info(f"Asynchronous import ended: {self.error}")
        else:
            logger.info(f"Asynchronous import complete - '{self.robot.name}' is ready.")
