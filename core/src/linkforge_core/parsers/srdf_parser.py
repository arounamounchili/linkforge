"""SRDF XML parser for LinkForge.

This module implements a robust SRDF (Semantic Robot Description Format) parser
that supports MoveIt-style tags and native XACRO resolution.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from ..base import IResourceResolver
from ..exceptions import (
    RobotParserError,
    RobotParserIOError,
    RobotParserUnexpectedError,
    RobotParserXMLRootError,
)
from ..logging_config import get_logger
from ..models.srdf import (
    DisabledCollision,
    EndEffector,
    GroupState,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    VirtualJoint,
)
from ..utils.xml_utils import parse_float
from .xml_base import MAX_FILE_SIZE, RobotXMLParser

logger = get_logger(__name__)


@runtime_checkable
class ITemplateResolver(Protocol):
    """Protocol for resolving templated XML strings (e.g., XACRO, Jinja).

    Implementing this protocol allows the SRDFParser to handle files that
    require preprocessing before they can be parsed as standard XML.
    """

    def resolve_string(self, xml_string: str) -> str:
        """Resolve a templated string into plain XML.

        Args:
            xml_string: The raw string containing template directives.
        Returns:
            A resolved XML string ready for parsing.
        """
        ...


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
        search_paths: list[Path] | None = None,
        template_resolver: ITemplateResolver | None = None,
    ) -> None:
        """Initialize SRDF parser.

        Args:
            max_file_size: Maximum allowed file size in bytes.
            sandbox_root: Optional root directory for security sandbox.
            resource_resolver: Optional resolver for URIs.
            search_paths: Optional search paths for XACRO includes.
            template_resolver: Optional template resolver for preprocessing the SRDF content.
        """
        super().__init__(
            max_file_size=max_file_size,
            sandbox_root=sandbox_root,
            resource_resolver=resource_resolver,
        )
        self.search_paths = search_paths or []
        self.template_resolver = template_resolver

    def _parse_planning_group(self, group_elem: ET.Element) -> PlanningGroup:
        """Parse a <group> element into a PlanningGroup model.

        Args:
            group_elem: The XML element for the group.
        Returns:
            A populated PlanningGroup instance.
        """
        name = group_elem.get("name")
        if not name:
            logger.warning("SRDF: Planning group missing name attribute, skipping")
            return None  # type: ignore[return-value]

        links: list[str] = []
        joints: list[str] = []
        chains: list[tuple[str, str]] = []
        subgroups: list[str] = []

        for child in group_elem:
            if child.tag == "link":
                link_name = child.get("name")
                if link_name:
                    links.append(link_name)
            elif child.tag == "joint":
                joint_name = child.get("name")
                if joint_name:
                    joints.append(joint_name)
            elif child.tag == "chain":
                base = child.get("base_link")
                tip = child.get("tip_link")
                if base and tip:
                    chains.append((base, tip))
            elif child.tag == "group":
                subgroup_name = child.get("name")
                if subgroup_name:
                    subgroups.append(subgroup_name)

        return PlanningGroup(
            name=name,
            links=tuple(links),
            joints=tuple(joints),
            chains=tuple(chains),
            subgroups=tuple(subgroups),
        )

    def _parse_group_state(self, state_elem: ET.Element) -> GroupState:
        """Parse a <group_state> element into a GroupState model.

        Args:
            state_elem: The XML element for the group state.

        Returns:
            A populated GroupState instance.
        """
        name = state_elem.get("name", "unnamed_state")
        group = state_elem.get("group", "")
        joint_values: dict[str, float] = {}

        for joint_elem in state_elem.findall("joint"):
            j_name = joint_elem.get("name")
            j_val = parse_float(joint_elem.get("value"), f"joint {j_name} value", default=0.0)
            if j_name:
                joint_values[j_name] = j_val

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
        # Handle templating resolution
        if self.template_resolver is not None:
            content = self.template_resolver.resolve_string(content)

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
        virtual_joints: list[VirtualJoint] = []
        groups: list[PlanningGroup] = []
        group_states: list[GroupState] = []
        end_effectors: list[EndEffector] = []
        passive_joints: list[PassiveJoint] = []
        disabled_collisions: list[DisabledCollision] = []

        for child in root:
            if child.tag == "virtual_joint":
                virtual_joints.append(self._parse_virtual_joint_elem(child))
            elif child.tag == "group":
                group = self._parse_planning_group(child)
                if group:
                    groups.append(group)
            elif child.tag == "group_state":
                group_states.append(self._parse_group_state(child))
            elif child.tag == "end_effector":
                end_effectors.append(self._parse_end_effector_elem(child))
            elif child.tag == "passive_joint":
                pj_name = child.get("name")
                if pj_name:
                    passive_joints.append(PassiveJoint(name=pj_name))
                else:
                    logger.warning("SRDF: Passive joint missing name, skipping")
            elif child.tag == "disable_collisions":
                disabled_collisions.append(self._parse_disable_collisions_elem(child))

        return SemanticRobotDescription(
            virtual_joints=tuple(virtual_joints),
            groups=tuple(groups),
            group_states=tuple(group_states),
            end_effectors=tuple(end_effectors),
            passive_joints=tuple(passive_joints),
            disabled_collisions=tuple(disabled_collisions),
        )

    def _parse_virtual_joint_elem(self, elem: ET.Element) -> VirtualJoint:
        """Parse a <virtual_joint> element.

        Args:
            elem: The XML element.

        Returns:
            A VirtualJoint model.
        """
        return VirtualJoint(
            name=elem.get("name", "unnamed_vj"),
            type=elem.get("type", "fixed"),
            parent_frame=elem.get("parent_frame", "world"),
            child_link=elem.get("child_link", "base_link"),
        )

    def _parse_end_effector_elem(self, elem: ET.Element) -> EndEffector:
        """Parse an <end_effector> element.

        Args:
            elem: The XML element.

        Returns:
            An EndEffector model.
        """
        return EndEffector(
            name=elem.get("name", "unnamed_ee"),
            group=elem.get("group", ""),
            parent_link=elem.get("parent_link", ""),
            parent_group=elem.get("parent_group"),
        )

    def _parse_disable_collisions_elem(self, elem: ET.Element) -> DisabledCollision:
        """Parse a <disable_collisions> element.

        Args:
            elem: The XML element.

        Returns:
            A DisabledCollision model.
        """
        return DisabledCollision(
            link1=elem.get("link1", ""),
            link2=elem.get("link2", ""),
            reason=elem.get("reason"),
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
