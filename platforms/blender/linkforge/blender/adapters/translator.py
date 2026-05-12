"""Protocols and base classes for scene translation.

This module defines the interfaces for mapping Blender scene data
to LinkForge core models using the Composer API.
"""

from __future__ import annotations

import typing
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

if typing.TYPE_CHECKING:
    from linkforge_core.composer.link_builder import LinkBuilder
    from linkforge_core.composer.robot_builder import RobotBuilder
    from linkforge_core.validation.result import ValidationResult

    from .context import IBlenderContext


@runtime_checkable
class ITranslator(Protocol):
    """Base protocol for translating Blender objects to Core models."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,
        meshes_dir: Path | None = None,
        dry_run: bool = False,
        depsgraph: Any | None = None,
        validation_result: ValidationResult | None = None,
    ) -> Any:
        """Translate a Blender object using the provided builder."""
        ...


class TranslationRegistry:
    """Registry for managing specialized translators for different component types."""

    def __init__(self) -> None:
        self._translators: dict[str, ITranslator] = {}

    def register(self, component_type: str, translator: ITranslator) -> None:
        """Register a translator for a specific component type."""
        self._translators[component_type] = translator

    def get(self, component_type: str) -> ITranslator | None:
        """Retrieve a translator for a component type."""
        return self._translators.get(component_type)


class LinkTranslator(ITranslator):
    """Translates Blender objects marked as robot links."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,
        meshes_dir: Path | None = None,
        dry_run: bool = False,
        depsgraph: Any | None = None,
        validation_result: ValidationResult | None = None,
        lb: LinkBuilder | None = None,
        **_kwargs: Any,
    ) -> LinkBuilder | None:
        """Translate a Blender link to a Core Link using RobotBuilder."""
        from linkforge_core.models.link import InertiaTensor
        from linkforge_core.utils.string_utils import sanitize_name

        from .blender_to_core import (
            get_object_geometry,
            get_object_material,
            matrix_to_transform,
        )

        props = getattr(obj, "linkforge", None)
        if not props:
            return None

        link_name = props.link_name if props.link_name else obj.name
        robot_props = getattr(context.scene, "linkforge", None)
        mesh_format = robot_props.mesh_format if robot_props else "STL"

        # Use provided LinkBuilder or create a new one
        active_lb = lb if lb else builder.link(link_name)

        # 1. Translate visuals
        for child in obj.children:
            if "_visual" in child.name:
                mat = get_object_material(child, props)
                suffix = self._get_geom_suffix(child, obj, "_visual", sanitize_name)

                geom, world_mat = get_object_geometry(
                    child,
                    "AUTO",
                    link_name,
                    "visual",
                    meshes_dir,
                    mesh_format,
                    dry_run=dry_run,
                    suffix=suffix,
                    depsgraph=depsgraph,
                )
                if mat and mat.name not in builder.robot.materials:
                    # Register material in the robot model to satisfy LinkBuilder validation
                    builder.robot.materials[mat.name] = mat

                if geom:
                    rel_mat = obj.matrix_world.inverted() @ world_mat
                    origin = matrix_to_transform(rel_mat)
                    active_lb.visual(
                        geom,
                        xyz=origin.xyz.to_tuple(),
                        rpy=origin.rpy.to_tuple(),
                        material=mat.name if mat else None,
                        name=child.get("source_name"),
                    )
                    # Mesh Topology Validation
                    self._validate_mesh(child, link_name, "visual", validation_result)

        # 2. Translate collisions
        for child in obj.children:
            if "_collision" in child.name:
                suffix = self._get_geom_suffix(child, obj, "_collision", sanitize_name)
                quality = props.collision_quality / 100.0
                is_imported = child.get("imported_from_source", False)

                geom, world_mat = get_object_geometry(
                    child,
                    "AUTO",
                    link_name,
                    "collision",
                    meshes_dir,
                    mesh_format,
                    simplify=(quality < 1.0) and not is_imported,
                    decimation_ratio=quality,
                    dry_run=dry_run,
                    suffix=suffix,
                    depsgraph=depsgraph,
                )
                if geom:
                    rel_mat = obj.matrix_world.inverted() @ world_mat
                    origin = matrix_to_transform(rel_mat)
                    active_lb.collision(
                        geom,
                        xyz=origin.xyz.to_tuple(),
                        rpy=origin.rpy.to_tuple(),
                        name=child.get("source_name"),
                    )
                    # Mesh Topology Validation
                    self._validate_mesh(child, link_name, "collision", validation_result)

        # 3. Translate Physics (Inertia & Mass)
        if props.use_auto_inertia:
            active_lb.mass(props.mass)
        else:
            inertia = InertiaTensor(
                ixx=props.inertia_ixx,
                ixy=props.inertia_ixy,
                ixz=props.inertia_ixz,
                iyy=props.inertia_iyy,
                iyz=props.inertia_iyz,
                izz=props.inertia_izz,
            )
            active_lb.mass(
                props.mass,
                origin_xyz=tuple(props.inertia_origin_xyz),
                origin_rpy=tuple(props.inertia_origin_rpy),
                inertia=inertia,
            )

        # 4. Translate Gazebo Physics
        if props.use_simulation_props:
            active_lb.physics(
                self_collide=props.self_collide,
                gravity=props.gravity,
                mu1=props.mu1,
                mu2=props.mu2,
                kp=props.kp,
                kd=props.kd,
            )

        return active_lb

    def _get_geom_suffix(
        self, child: Any, parent_obj: Any, type_tag: str, sanitize_func: Any
    ) -> str:
        visual_count = sum(1 for c in parent_obj.children if type_tag in c.name)
        source_name = child.get("source_name", None)
        if source_name:
            return f"_{sanitize_func(source_name)}"
        elif visual_count > 1:
            idx = [c for c in parent_obj.children if type_tag in c.name].index(child)
            return f"_{idx}"
        return ""

    def _validate_mesh(
        self, obj: Any, link_name: str, purpose: str, result: ValidationResult | None
    ) -> None:
        if not result or obj.type != "MESH":
            return

        from linkforge_core.physics.mesh_validation import validate_mesh_topology
        from linkforge_core.validation.result import Severity

        try:
            mesh = obj.data
            verts = [v.co.to_tuple() for v in mesh.vertices]
            tris = [tuple(p.vertices) for p in mesh.polygons]

            issues = validate_mesh_topology(
                vertices=verts, triangles=tris, name=f"{link_name} ({purpose})", level=2
            )

            for issue in issues:
                if issue.severity == Severity.ERROR:
                    result.add_error(
                        title=issue.title,
                        message=issue.message,
                        code=issue.code,
                        affected_objects=[link_name, obj.name],
                        suggestion=issue.suggestion,
                    )
                else:
                    result.add_warning(
                        title=issue.title,
                        message=issue.message,
                        code=issue.code,
                        affected_objects=[link_name, obj.name],
                        suggestion=issue.suggestion,
                    )
        except Exception:
            pass


class JointTranslator(ITranslator):
    """Translates Blender objects marked as robot joints."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,  # noqa: ARG002
        context: IBlenderContext,  # noqa: ARG002
        meshes_dir: Path | None = None,  # noqa: ARG002
        dry_run: bool = False,  # noqa: ARG002
        depsgraph: Any | None = None,  # noqa: ARG002
        validation_result: ValidationResult | None = None,  # noqa: ARG002
        lb: LinkBuilder | None = None,
        link_frames: dict[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        """Translate a Blender joint to a Core Joint using the LinkBuilder."""
        if not lb:
            return

        from linkforge_core.models.joint import JointType

        from .blender_to_core import matrix_to_transform

        props = getattr(obj, "linkforge_joint", None)
        if not props:
            return

        # Calculate joint origin
        if link_frames:
            parent_name = props.parent_link.linkforge.link_name if props.parent_link else ""
            child_name = props.child_link.linkforge.link_name if props.child_link else ""

            if parent_name in link_frames and child_name in link_frames:
                parent_frame = link_frames[parent_name]
                child_frame = link_frames[child_name]
                joint_relative = parent_frame.inverted() @ child_frame
                origin = matrix_to_transform(joint_relative)
            else:
                origin = matrix_to_transform(obj.matrix_world)
        else:
            origin = matrix_to_transform(obj.matrix_world)

        # Joint Axis
        axis: tuple[float, float, float]
        if props.axis == "X":
            axis = (1.0, 0.0, 0.0)
        elif props.axis == "Y":
            axis = (0.0, 1.0, 0.0)
        elif props.axis == "Z":
            axis = (0.0, 0.0, 1.0)
        elif props.axis == "CUSTOM":
            axis = (
                float(props.custom_axis_x),
                float(props.custom_axis_y),
                float(props.custom_axis_z),
            )
            # Fallback for zero axis
            if all(abs(v) < 1e-6 for v in axis):
                axis = (0.0, 0.0, 1.0)
        else:
            axis = (0.0, 0.0, 1.0)

        # Select joint type and configure
        joint_type = JointType(props.joint_type.lower())
        j_name = props.joint_name if props.joint_name else obj.name

        if joint_type == JointType.REVOLUTE:
            lb.revolute(
                name=j_name,
                axis=axis,
                limits=(props.limit_lower, props.limit_upper),
                effort=props.limit_effort,
                velocity=props.limit_velocity,
                xyz=origin.xyz.to_tuple(),
                rpy=origin.rpy.to_tuple(),
            )
        elif joint_type == JointType.CONTINUOUS:
            lb.continuous(
                name=j_name,
                axis=axis,
                effort=props.limit_effort,
                velocity=props.limit_velocity,
                xyz=origin.xyz.to_tuple(),
                rpy=origin.rpy.to_tuple(),
            )
        elif joint_type == JointType.PRISMATIC:
            lb.prismatic(
                name=j_name,
                axis=axis,
                limits=(props.limit_lower, props.limit_upper),
                effort=props.limit_effort,
                velocity=props.limit_velocity,
                xyz=origin.xyz.to_tuple(),
                rpy=origin.rpy.to_tuple(),
            )
        elif joint_type == JointType.FIXED:
            lb.fixed(name=j_name, xyz=origin.xyz.to_tuple(), rpy=origin.rpy.to_tuple())
        elif joint_type == JointType.FLOATING:
            lb.floating(name=j_name, xyz=origin.xyz.to_tuple(), rpy=origin.rpy.to_tuple())
        elif joint_type == JointType.PLANAR:
            lb.planar(name=j_name, axis=axis, xyz=origin.xyz.to_tuple(), rpy=origin.rpy.to_tuple())

        # Dynamics
        if props.use_dynamics:
            lb.dynamics(damping=props.dynamics_damping, friction=props.dynamics_friction)

        # Mimic
        if props.use_mimic and props.mimic_joint:
            mimic_props = getattr(props.mimic_joint, "linkforge_joint", None)
            mimic_name = mimic_props.joint_name if mimic_props else props.mimic_joint.name
            lb.mimic(mimic_name, multiplier=props.mimic_multiplier, offset=props.mimic_offset)

        # Safety & Calibration
        if props.use_safety_controller:
            lb.safety(
                soft_lower=props.safety_soft_lower_limit,
                soft_upper=props.safety_soft_upper_limit,
                k_position=props.safety_k_position,
                k_velocity=props.safety_k_velocity,
            )

        if props.use_calibration:
            lb.calibration(
                rising=props.calibration_rising if props.use_calibration_rising else None,
                falling=props.calibration_falling if props.use_calibration_falling else None,
            )


class SensorTranslator(ITranslator):
    """Translates Blender objects marked as robot sensors."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,  # noqa: ARG002
        meshes_dir: Path | None = None,  # noqa: ARG002
        dry_run: bool = False,  # noqa: ARG002
        depsgraph: Any | None = None,  # noqa: ARG002
        validation_result: ValidationResult | None = None,
        link_frames: dict[str, Any] | None = None,
    ) -> None:
        """Translate a Blender sensor to a Core Sensor and add it to the robot."""
        from dataclasses import replace

        from .blender_to_core import blender_sensor_to_core, matrix_to_transform

        try:
            sensor = blender_sensor_to_core(obj)
            if sensor:
                # Calculate origin relative to link
                link_name = sensor.link_name
                if link_frames and link_name in link_frames:
                    link_frame_inv = link_frames[link_name].inverted()
                    sensor_relative = link_frame_inv @ obj.matrix_world
                    corrected_origin = matrix_to_transform(sensor_relative)
                    sensor = replace(sensor, origin=corrected_origin)

                builder.robot.add_sensor(sensor)
        except Exception as e:
            if validation_result:
                from linkforge_core.exceptions import ValidationErrorCode

                validation_result.add_error(
                    title=f"Sensor translation failed: {obj.name}",
                    message=str(e),
                    code=ValidationErrorCode.INVALID_VALUE,
                    affected_objects=[obj.name],
                )
