"""SRDF XML generator for LinkForge.

This module implements a generator to export LinkForge's semantic robot
description back to MoveIt-standard SRDF XML format.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from .. import __version__
from ..base import RobotGeneratorError
from ..logging_config import get_logger
from ..models.robot import Robot
from ..models.srdf import (
    CollisionPair,
    EndEffector,
    GroupState,
    JointProperty,
    LinkSphereApproximation,
    PassiveJoint,
    PlanningGroup,
    VirtualJoint,
)
from ..utils.math_utils import format_float
from ..utils.xml_utils import create_xml_element, serialize_xml
from ..validation import RobotValidator
from .xml_base import RobotXMLGenerator

logger = get_logger(__name__)


class SRDFGenerator(RobotXMLGenerator):
    """Semantic Robot Description Format (SRDF) generator."""

    def __init__(self, pretty_print: bool = True, srdf_path: Path | None = None) -> None:
        """Initialize SRDF generator.

        Args:
            pretty_print: If True, format XML with indentation for readability (default: True)
            srdf_path: Path where SRDF will be saved.
        """
        super().__init__(pretty_print=pretty_print, output_path=srdf_path)

    def generate(self, robot: Robot, validate: bool = True, **kwargs: Any) -> str:
        """Generate SRDF XML string from robot.

        Args:
            robot: Robot model with semantic description.
            validate: Whether to validate robot structure before generation.
            **kwargs: Additional generation options (passed to serializer).

        Returns:
            SRDF XML as formatted string with proper indentation
        """
        if validate:
            validator = RobotValidator()
            result = validator.validate(robot)
            if not result.is_valid:
                error_msgs = [str(issue) for issue in result.errors]
                raise RobotGeneratorError("Robot validation failed:\n" + "\n".join(error_msgs))

        if robot.semantic is None:
            logger.warning(f"Robot '{robot.name}' has no semantic description to generate.")

        root = self.generate_robot_element(robot)
        return serialize_xml(root, pretty_print=self.pretty_print, version=__version__, **kwargs)

    def generate_robot_element(self, robot: Robot) -> ET.Element:
        """Generate SRDF XML Element tree from robot."""
        name = (
            robot.semantic.robot_name
            if robot.semantic and robot.semantic.robot_name
            else robot.name
        )
        root = ET.Element("robot", name=name)

        if not robot.semantic:
            return root

        semantic = robot.semantic
        self._add_virtual_joints(root, sorted(semantic.virtual_joints, key=lambda x: x.name))
        self._add_groups(root, sorted(semantic.groups, key=lambda x: x.name))
        self._add_group_states(root, sorted(semantic.group_states, key=lambda x: x.name))
        self._add_end_effectors(root, sorted(semantic.end_effectors, key=lambda x: x.name))
        self._add_passive_joints(root, sorted(semantic.passive_joints, key=lambda x: x.name))
        self._add_disabled_collisions(
            root, sorted(semantic.disabled_collisions, key=lambda x: (x.link1, x.link2))
        )
        self._add_enabled_collisions(
            root, sorted(semantic.enabled_collisions, key=lambda x: (x.link1, x.link2))
        )
        self._add_no_default_collision_links(root, sorted(semantic.no_default_collision_links))
        self._add_link_sphere_approximations(
            root, sorted(semantic.link_sphere_approximations, key=lambda x: x.link)
        )
        self._add_joint_properties(
            root, sorted(semantic.joint_properties, key=lambda x: (x.joint_name, x.property_name))
        )

        return root

    def _add_virtual_joints(self, root: ET.Element, virtual_joints: list[VirtualJoint]) -> None:
        """Add virtual joint elements to root."""
        for vj in virtual_joints:
            create_xml_element(
                root,
                "virtual_joint",
                formatter=self._format_value,
                name=vj.name,
                type=vj.type,
                parent_frame=vj.parent_frame,
                child_link=vj.child_link,
            )

    def _add_groups(self, root: ET.Element, groups: list[PlanningGroup]) -> None:
        """Add planning group elements to root."""
        for group in groups:
            group_elem = ET.SubElement(root, "group", name=group.name)
            for link_name in group.links:
                ET.SubElement(group_elem, "link", name=link_name)
            for joint_name in group.joints:
                ET.SubElement(group_elem, "joint", name=joint_name)
            for chain in group.chains:
                ET.SubElement(
                    group_elem, "chain", base_link=chain.base_link, tip_link=chain.tip_link
                )
            for subgroup in group.subgroups:
                ET.SubElement(group_elem, "group", name=subgroup)

    def _add_group_states(self, root: ET.Element, states: list[GroupState]) -> None:
        """Add group state elements to root."""
        for state in states:
            state_elem = ET.SubElement(root, "group_state", name=state.name, group=state.group)
            for j_name, j_vals in state.joint_values.items():
                val_str = " ".join(format_float(v) for v in j_vals)
                ET.SubElement(state_elem, "joint", name=j_name, value=val_str)

    def _add_end_effectors(self, root: ET.Element, end_effectors: list[EndEffector]) -> None:
        """Add end effector elements to root."""
        for ee in end_effectors:
            create_xml_element(
                root,
                "end_effector",
                formatter=self._format_value,
                name=ee.name,
                group=ee.group,
                parent_link=ee.parent_link,
                parent_group=ee.parent_group,
            )

    def _add_passive_joints(self, root: ET.Element, passive_joints: list[PassiveJoint]) -> None:
        """Add passive joint elements to root."""
        for pj in passive_joints:
            ET.SubElement(root, "passive_joint", name=pj.name)

    def _add_disabled_collisions(
        self, root: ET.Element, disabled_collisions: list[CollisionPair]
    ) -> None:
        """Add disabled collision elements to root."""
        for dc in disabled_collisions:
            create_xml_element(
                root,
                "disable_collisions",
                formatter=self._format_value,
                link1=dc.link1,
                link2=dc.link2,
                reason=dc.reason,
            )

    def _add_enabled_collisions(
        self, root: ET.Element, enabled_collisions: list[CollisionPair]
    ) -> None:
        """Add enabled collision elements to root."""
        for ec in enabled_collisions:
            create_xml_element(
                root,
                "enable_collisions",
                formatter=self._format_value,
                link1=ec.link1,
                link2=ec.link2,
                reason=ec.reason,
            )

    def _add_no_default_collision_links(self, root: ET.Element, links: list[str]) -> None:
        """Add disable default collisions elements to root."""
        for link in links:
            ET.SubElement(root, "disable_default_collisions", link=link)

    def _add_link_sphere_approximations(
        self, root: ET.Element, approximations: list[LinkSphereApproximation]
    ) -> None:
        """Add link sphere approximation elements to root."""
        for lsa in approximations:
            lsa_elem = ET.SubElement(root, "link_sphere_approximation", link=lsa.link)
            for sphere in lsa.spheres:
                center_str = f"{format_float(sphere.center_x)} {format_float(sphere.center_y)} {format_float(sphere.center_z)}"
                create_xml_element(
                    lsa_elem,
                    "sphere",
                    formatter=self._format_value,
                    center=center_str,
                    radius=sphere.radius,
                )

    def _add_joint_properties(self, root: ET.Element, properties: list[JointProperty]) -> None:
        """Add joint property elements to root."""
        for jp in properties:
            ET.SubElement(
                root,
                "joint_property",
                joint_name=jp.joint_name,
                property_name=jp.property_name,
                value=jp.value,
            )
