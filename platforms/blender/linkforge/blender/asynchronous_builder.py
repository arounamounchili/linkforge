"""Asynchronous Robot Builder for Blender.

This module provides an asynchronous task runner for importing robot models
into Blender without blocking the UI. It uses `bpy.app.timers` to process
the robot structure in chunks, allowing for a responsive UI and progress updates.
"""

from __future__ import annotations

from pathlib import Path

import bpy

from ..linkforge_core.logging_config import get_logger
from ..linkforge_core.models import Robot
from .scene_builder import (
    create_joint_object,
    create_link_object,
    create_sensor_object,
)

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
        # 1. Create collection
        self.tasks.append(("create_collection", None))

        # 2. Create link tasks
        for link in self.robot.links:
            self.tasks.append(("create_link", link))

        # 3. Create sorted joint tasks
        # We need to sort joints topologically exactly like in scene_builder
        # We reuse the internal sort function logic
        def sort_joints_topological(joints, links):
            child_links = {j.child for j in joints}
            root_links = {link.name for link in links if link.name not in child_links}
            children_of = {}
            for joint in joints:
                if joint.parent not in children_of:
                    children_of[joint.parent] = []
                children_of[joint.parent].append(joint)

            sorted_joints = []
            visited = set()

            def visit(link_name):
                if link_name in visited:
                    return
                visited.add(link_name)
                if link_name in children_of:
                    for joint in children_of[link_name]:
                        sorted_joints.append(joint)
                        visit(joint.child)

            for root in root_links:
                visit(root)
            return sorted_joints

        sorted_joints = sort_joints_topological(self.robot.joints, self.robot.links)
        for joint in sorted_joints:
            self.tasks.append(("create_joint", joint))

        # 4. Mimic joints resolution
        self.tasks.append(("resolve_mimics", None))

        # 5. Sensors
        if hasattr(self.robot, "sensors"):
            for sensor in self.robot.sensors:
                # Assuming SceneBuilder can handle sensors
                self.tasks.append(("create_sensor", sensor))

        # 6. Finalization
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

            return 0.001

        except Exception as e:
            self.error = str(e)
            logger.error(f"Asynchronous import failed: {e}")
            self.finish()
            return None

    def _execute_task(self, task_type, data):
        """Execute a single unit of work."""
        if task_type == "create_collection":
            self.collection = bpy.data.collections.new(self.robot.name)
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
            for joint in self.robot.joints:
                if joint.mimic and joint.name in self.joint_objects:
                    joint_obj = self.joint_objects[joint.name]
                    mimic_joint_obj = self.joint_objects.get(joint.mimic.joint)
                    if mimic_joint_obj:
                        joint_obj.linkforge_joint.mimic_joint = mimic_joint_obj

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

    def finish(self):
        """Clean up and finalize."""
        self.context.window_manager.progress_end()
        self.is_finished = True

        # Clear background state
        scene = self.context.scene
        if hasattr(scene, "linkforge"):
            scene.linkforge.is_importing = False
            scene.linkforge.import_status = ""
            scene.linkforge.abort_import = False

        if self.error:
            # Report error if cancelled or failed
            logger.info(f"Asynchronous import ended: {self.error}")
        else:
            logger.info(f"Asynchronous import complete - '{self.robot.name}' is ready.")
