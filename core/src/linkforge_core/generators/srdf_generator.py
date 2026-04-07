"""SRDF XML generator for LinkForge.

This module implements a generator to export LinkForge's semantic robot
description back to MoveIt-standard SRDF XML format.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from .. import __version__
from ..logging_config import get_logger
from ..models import (
    DisabledCollision,
    EndEffector,
    GroupState,
    PassiveJoint,
    PlanningGroup,
    VirtualJoint,
)
from ..models.robot import Robot
from ..utils.math_utils import format_float
from ..utils.xml_utils import create_xml_element, serialize_xml
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
        if validate and robot.semantic is None:
            logger.warning(f"Robot '{robot.name}' has no semantic description to generate.")

        root = self.generate_robot_element(robot)
        return serialize_xml(root, pretty_print=self.pretty_print, version=__version__, **kwargs)

    def generate_robot_element(self, robot: Robot) -> ET.Element:
        """Generate SRDF XML Element tree from robot."""
        root = ET.Element("robot", name=robot.name)

        if not robot.semantic:
            return root

        semantic = robot.semantic
        self._add_virtual_joints(root, semantic.virtual_joints)
        self._add_groups(root, semantic.groups)
        self._add_group_states(root, semantic.group_states)
        self._add_end_effectors(root, semantic.end_effectors)
        self._add_passive_joints(root, semantic.passive_joints)
        self._add_disabled_collisions(root, semantic.disabled_collisions)

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
            for base, tip in group.chains:
                ET.SubElement(group_elem, "chain", base_link=base, tip_link=tip)
            for subgroup in group.subgroups:
                ET.SubElement(group_elem, "group", name=subgroup)

    def _add_group_states(self, root: ET.Element, states: list[GroupState]) -> None:
        """Add group state elements to root."""
        for state in states:
            state_elem = ET.SubElement(root, "group_state", name=state.name, group=state.group)
            for j_name, j_val in state.joint_values.items():
                ET.SubElement(state_elem, "joint", name=j_name, value=format_float(j_val))

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
        self, root: ET.Element, disabled_collisions: list[DisabledCollision]
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
