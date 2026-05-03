"""SRDF XML parser for LinkForge.

This module implements a robust SRDF (Semantic Robot Description Format) parser
that supports MoveIt-style tags and native XACRO resolution.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from ..base import IResourceResolver
from ..exceptions import (
    RobotParserError,
    RobotParserIOError,
    RobotParserUnexpectedError,
    RobotParserXMLRootError,
)
from ..logging_config import get_logger
from ..models.srdf import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    JointProperty,
    LinkSphereApproximation,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    SrdfSphere,
    VirtualJoint,
)
from ..utils.xml_utils import parse_float
from .xml_base import MAX_FILE_SIZE, RobotXMLParser

logger = get_logger(__name__)


def _strip_ns(tag: str) -> str:
    """Strip XML namespace from tag if present."""
    return tag.split("}", 1)[-1] if "}" in tag else tag


class SRDFParser(RobotXMLParser[SemanticRobotDescription]):
    """Semantic Robot Description Format (SRDF) Parser.

    This parser converts SRDF XML content into a structured
    ``SemanticRobotDescription`` model. It supports MoveIt-specific tags
    such as planning groups, end effectors, and collision disabling.
    """

    def __init__(
        self,
        max_file_size: int = MAX_FILE_SIZE,
        sandbox_root: Path | None = None,
        resource_resolver: IResourceResolver | None = None,
    ) -> None:
        """Initialize SRDF parser.

        Args:
            max_file_size: Maximum allowed file size in bytes.
            sandbox_root: Optional root directory for security sandbox.
            resource_resolver: Optional resolver for URIs.
        """
        super().__init__(
            max_file_size=max_file_size,
            sandbox_root=sandbox_root,
            resource_resolver=resource_resolver,
        )

    def _parse_planning_group(self, group_elem: ET.Element) -> PlanningGroup | None:
        """Parse a <group> element into a PlanningGroup model.

        Args:
            group_elem: The XML element for the group.
        Returns:
            A populated PlanningGroup instance or None if invalid.
        """
        name = group_elem.get("name")
        if not name:
            logger.warning("SRDF: Planning group missing name attribute, skipping")
            return None

        links: list[str] = []
        joints: list[str] = []
        chains: list[Chain] = []
        subgroups: list[str] = []

        for child in group_elem:
            tag = _strip_ns(child.tag)
            if tag == "link":
                link_name = child.get("name")
                if link_name:
                    links.append(link_name)
            elif tag == "joint":
                joint_name = child.get("name")
                if joint_name:
                    joints.append(joint_name)
            elif tag == "chain":
                base = child.get("base_link")
                tip = child.get("tip_link")
                if base and tip:
                    chains.append(Chain(base_link=base, tip_link=tip))
            elif tag == "group":
                subgroup_name = child.get("name")
                if subgroup_name:
                    subgroups.append(subgroup_name)

        try:
            return PlanningGroup(
                name=name,
                links=tuple(links),
                joints=tuple(joints),
                chains=tuple(chains),
                subgroups=tuple(subgroups),
            )
        except Exception as e:
            logger.warning(f"SRDF: Skipping planning group '{name}': {e}")
            return None

    def _parse_group_state(self, state_elem: ET.Element) -> GroupState | None:
        """Parse a <group_state> element into a GroupState model.

        Args:
            state_elem: The XML element for the group state.

        Returns:
            A populated GroupState instance, or None if invalid.
        """
        name = state_elem.get("name")
        group = state_elem.get("group")

        if not name or not group:
            logger.warning("SRDF: Group state missing name or group attribute, skipping")
            return None

        joint_values: dict[str, tuple[float, ...]] = {}

        for joint_elem in state_elem:
            if _strip_ns(joint_elem.tag) != "joint":
                continue
            j_name = joint_elem.get("name")
            j_val_str = joint_elem.get("value")

            if not j_name or j_val_str is None:
                logger.warning(
                    f"SRDF: Joint in group state '{name}' missing name or value, skipping"
                )
                continue

            try:
                # Parse space-separated floats
                vals = tuple(parse_float(v, f"joint {j_name} value") for v in j_val_str.split())
                if vals:
                    joint_values[j_name] = vals
            except Exception as e:
                logger.warning(
                    f"SRDF: Invalid joint value for '{j_name}' in group state '{name}': {e}"
                )

        return GroupState(name=name, group=group, joint_values=joint_values)

    def parse_string(
        self,
        content: str,
        **kwargs: Any,
    ) -> SemanticRobotDescription:
        """Parse SRDF content from a string.

        Args:
            content: The raw SRDF XML string.
            **kwargs: Additional options for future extensions.

        Returns:
            A SemanticRobotDescription model representing the SRDF.

        Raises:
            RobotParserUnexpectedError: If the XML is malformed.
            RobotParserXMLRootError: If the root tag is not <robot>.
        """

        try:
            root = ET.fromstring(content)
        except ET.ParseError as e:
            raise RobotParserUnexpectedError(source_area="SRDF parse", original_error=e) from e
        except Exception as e:
            raise RobotParserUnexpectedError(
                source_area="Unexpected SRDF parse", original_error=e
            ) from e

        if root.tag != "robot":
            raise RobotParserXMLRootError(root.tag)

        semantic = self._parse_elements(root)

        # Handle kwargs if any (e.g. for future extensions)
        if kwargs:
            logger.debug(f"SRDFParser received unused options: {list(kwargs.keys())}")

        return semantic

    def _parse_elements(self, root: ET.Element) -> SemanticRobotDescription:
        """Iterate through the XML root and parse all supported SRDF tags.

        Args:
            root: The <robot> XML root element.

        Returns:
            A SemanticRobotDescription containing all parsed elements.
        """
        robot_name = root.get("name", "")
        virtual_joints: list[VirtualJoint] = []
        groups: list[PlanningGroup] = []
        group_states: list[GroupState] = []
        end_effectors: list[EndEffector] = []
        passive_joints: list[PassiveJoint] = []
        disabled_collisions: list[CollisionPair] = []
        enabled_collisions: list[CollisionPair] = []
        no_default_collision_links: list[str] = []
        link_sphere_approximations: list[LinkSphereApproximation] = []
        joint_properties: list[JointProperty] = []

        for child in root:
            tag = _strip_ns(child.tag)
            if tag == "virtual_joint":
                vj = self._parse_virtual_joint_elem(child)
                if vj:
                    virtual_joints.append(vj)
            elif tag == "group":
                group = self._parse_planning_group(child)
                if group:
                    groups.append(group)
            elif tag == "group_state":
                gs = self._parse_group_state(child)
                if gs:
                    group_states.append(gs)
            elif tag == "end_effector":
                ee = self._parse_end_effector_elem(child)
                if ee:
                    end_effectors.append(ee)
            elif tag == "passive_joint":
                pj_name = child.get("name")
                if pj_name:
                    passive_joints.append(PassiveJoint(name=pj_name))
                else:
                    logger.warning("SRDF: Passive joint missing name, skipping")
            elif tag == "disable_collisions":
                dc = self._parse_collision_pair_elem(child)
                if dc:
                    disabled_collisions.append(dc)
            elif tag == "enable_collisions":
                ec = self._parse_collision_pair_elem(child)
                if ec:
                    enabled_collisions.append(ec)
            elif tag == "disable_default_collisions":
                link = child.get("link")
                if link:
                    no_default_collision_links.append(link)
                else:
                    logger.warning(
                        "SRDF: disable_default_collisions missing link attribute, skipping"
                    )
            elif tag == "link_sphere_approximation":
                lsa = self._parse_link_sphere_approximation_elem(child)
                if lsa:
                    link_sphere_approximations.append(lsa)
            elif tag == "joint_property":
                jp = self._parse_joint_property_elem(child)
                if jp:
                    joint_properties.append(jp)

        # Cross-reference validation
        group_names = {g.name for g in groups}
        for gs in group_states:
            if gs.group not in group_names:
                logger.warning(
                    f"SRDF: Group state '{gs.name}' refers to unknown group '{gs.group}'"
                )
        for ee in end_effectors:
            if ee.group not in group_names:
                logger.warning(
                    f"SRDF: End effector '{ee.name}' refers to unknown group '{ee.group}'"
                )

        return SemanticRobotDescription(
            robot_name=robot_name,
            virtual_joints=tuple(virtual_joints),
            groups=tuple(groups),
            group_states=tuple(group_states),
            end_effectors=tuple(end_effectors),
            passive_joints=tuple(passive_joints),
            disabled_collisions=tuple(disabled_collisions),
            enabled_collisions=tuple(enabled_collisions),
            no_default_collision_links=tuple(no_default_collision_links),
            link_sphere_approximations=tuple(link_sphere_approximations),
            joint_properties=tuple(joint_properties),
        )

    def _parse_virtual_joint_elem(self, elem: ET.Element) -> VirtualJoint | None:
        """Parse a <virtual_joint> element.

        Args:
            elem: The XML element.

        Returns:
            A VirtualJoint model, or None if invalid.
        """
        name = elem.get("name")
        vtype = elem.get("type")
        parent = elem.get("parent_frame")
        child = elem.get("child_link")

        if not name or not vtype or not parent or not child:
            logger.warning("SRDF: Virtual joint missing required attributes, skipping")
            return None

        return VirtualJoint(
            name=name,
            type=vtype,
            parent_frame=parent,
            child_link=child,
        )

    def _parse_end_effector_elem(self, elem: ET.Element) -> EndEffector | None:
        """Parse an <end_effector> element.

        Args:
            elem: The XML element.

        Returns:
            An EndEffector model, or None if invalid.
        """
        name = elem.get("name")
        group = elem.get("group")
        parent = elem.get("parent_link")

        if not name or not group or not parent:
            logger.warning("SRDF: End effector missing required attributes, skipping")
            return None

        return EndEffector(
            name=name,
            group=group,
            parent_link=parent,
            parent_group=elem.get("parent_group"),
        )

    def _parse_collision_pair_elem(self, elem: ET.Element) -> CollisionPair | None:
        """Parse a <disable_collisions> or <enable_collisions> element.

        Args:
            elem: The XML element.

        Returns:
            A CollisionPair model, or None if invalid.
        """
        link1 = elem.get("link1")
        link2 = elem.get("link2")

        if not link1 or not link2:
            logger.warning("SRDF: Collision pair missing link1 or link2, skipping")
            return None

        return CollisionPair(
            link1=link1,
            link2=link2,
            reason=elem.get("reason"),
        )

    def _parse_link_sphere_approximation_elem(
        self, elem: ET.Element
    ) -> LinkSphereApproximation | None:
        """Parse a <link_sphere_approximation> element."""
        link = elem.get("link")
        if not link:
            logger.warning("SRDF: Link sphere approximation missing link attribute, skipping")
            return None

        spheres: list[SrdfSphere] = []
        for child in elem:
            if _strip_ns(child.tag) == "sphere":
                center_str = child.get("center")
                radius_str = child.get("radius")
                if not center_str or not radius_str:
                    logger.warning(
                        f"SRDF: Sphere in link '{link}' missing center or radius, skipping"
                    )
                    continue
                try:
                    cx, cy, cz = (parse_float(v, "sphere center") for v in center_str.split())
                    r = parse_float(radius_str, "sphere radius")
                    spheres.append(SrdfSphere(center_x=cx, center_y=cy, center_z=cz, radius=r))
                except Exception as e:
                    logger.warning(f"SRDF: Invalid sphere in link '{link}': {e}")

        return LinkSphereApproximation(link=link, spheres=tuple(spheres))

    def _parse_joint_property_elem(self, elem: ET.Element) -> JointProperty | None:
        """Parse a <joint_property> element."""
        joint_name = elem.get("joint_name")
        property_name = elem.get("property_name")
        value = elem.get("value")

        if not joint_name or not property_name or not value:
            logger.warning("SRDF: Joint property missing required attributes, skipping")
            return None

        return JointProperty(
            joint_name=joint_name,
            property_name=property_name,
            value=value,
        )

    def parse(self, filepath: Path, **kwargs: Any) -> SemanticRobotDescription:
        """Load and parse an SRDF file from disk.

        Args:
            filepath: Path to the .srdf file.
            **kwargs: Passed to parse_string.

        Returns:
            A SemanticRobotDescription model.

        Raises:
            RobotParserIOError: If the file is missing or exceeds max_file_size.
        """
        if not filepath.exists():
            raise RobotParserIOError(filepath=filepath, reason="Missing file")

        # Security check: File size
        file_size = filepath.stat().st_size
        if file_size > self.max_file_size:
            raise RobotParserIOError(filepath=filepath, reason="File too large")

        try:
            content = filepath.read_text(encoding="utf-8")
            return self.parse_string(content, **kwargs)

        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotParserIOError(filepath=filepath, reason=str(e)) from e
