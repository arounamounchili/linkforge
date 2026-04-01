"""SRDF XML generator for LinkForge.

This module implements a generator to export LinkForge's semantic robot
description back to MoveIt-standard SRDF XML format.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from ..models.robot import Robot
from ..utils.math_utils import format_float
from ..utils.xml_utils import serialize_xml
from .xml_base import RobotXMLGenerator


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
            validate: Whether to validate robot structure before generation (not implemented for SRDF yet)
            **kwargs: Additional generation options

        Returns:
            SRDF XML as formatted string with proper indentation
        """
        from .. import __version__

        root = self.generate_robot_element(robot)
        return serialize_xml(root, pretty_print=self.pretty_print, version=__version__)

    def generate_robot_element(self, robot: Robot) -> ET.Element:
        """Generate SRDF XML Element tree from robot."""
        root = ET.Element("robot", name=robot.name)

        if not robot.semantic:
            return root

        semantic = robot.semantic

        # 1. Virtual Joints
        for vj in semantic.virtual_joints:
            ET.SubElement(
                root,
                "virtual_joint",
                name=vj.name,
                type=vj.type,
                parent_frame=vj.parent_frame,
                child_link=vj.child_link,
            )

        # 2. Planning Groups
        for group in semantic.groups:
            group_elem = ET.SubElement(root, "group", name=group.name)

            for link_name in group.links:
                ET.SubElement(group_elem, "link", name=link_name)

            for joint_name in group.joints:
                ET.SubElement(group_elem, "joint", name=joint_name)

            for base, tip in group.chains:
                ET.SubElement(group_elem, "chain", base_link=base, tip_link=tip)

            for subgroup in group.subgroups:
                ET.SubElement(group_elem, "group", name=subgroup)

        # 3. Group States
        for state in semantic.group_states:
            state_elem = ET.SubElement(root, "group_state", name=state.name, group=state.group)
            for j_name, j_val in state.joint_values.items():
                ET.SubElement(state_elem, "joint", name=j_name, value=format_float(j_val))

        # 4. End Effectors
        for ee in semantic.end_effectors:
            attrib = {
                "name": ee.name,
                "group": ee.group,
                "parent_link": ee.parent_link,
            }
            if ee.parent_group:
                attrib["parent_group"] = ee.parent_group
            ET.SubElement(root, "end_effector", **attrib)  # type: ignore[arg-type]

        # 5. Passive Joints
        for pj in semantic.passive_joints:
            ET.SubElement(root, "passive_joint", name=pj.name)

        # 6. Disabled Collisions
        for dc in semantic.disabled_collisions:
            attrib = {
                "link1": dc.link1,
                "link2": dc.link2,
            }
            if dc.reason:
                attrib["reason"] = dc.reason
            ET.SubElement(root, "disable_collisions", **attrib)  # type: ignore[arg-type]

        return root
