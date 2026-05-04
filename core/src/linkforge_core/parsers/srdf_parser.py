"""SRDF XML parser for LinkForge.

This module implements a robust SRDF (Semantic Robot Description Format) parser
that supports MoveIt-style tags and native XACRO resolution.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Callable
from pathlib import Path
from typing import Any, TypeVar

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
from ..utils.xml_utils import parse_float, strip_xml_namespace
from .xml_base import MAX_FILE_SIZE, RobotXMLParser

# Define a TypeVar for generic collection parsing
T = TypeVar("T")

logger = get_logger(__name__)


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

    def _collect_elements(
        self, root: ET.Element, tag: str, parse_func: Callable[[ET.Element], T | None]
    ) -> list[T]:
        """Generic helper to find and parse multiple XML elements.

        Args:
            root: The XML root or parent element to search.
            tag: The local name of the tag to find (supports wildcard namespaces).
            parse_func: The internal method to parse a single element.

        Returns:
            A list of successfully parsed models.
        """
        results: list[T] = []
        for elem in root.findall(f"{{*}}{tag}"):
            item = parse_func(elem)
            if item is not None:
                results.append(item)
        return results

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

        for link_elem in group_elem.findall("{*}link"):
            link_name = link_elem.get("name")
            if link_name:
                links.append(link_name)

        for joint_elem in group_elem.findall("{*}joint"):
            joint_name = joint_elem.get("name")
            if joint_name:
                joints.append(joint_name)

        for chain_elem in group_elem.findall("{*}chain"):
            base = chain_elem.get("base_link")
            tip = chain_elem.get("tip_link")
            if base and tip:
                chains.append(Chain(base_link=base, tip_link=tip))

        for subgroup_elem in group_elem.findall("{*}group"):
            subgroup_name = subgroup_elem.get("name")
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

        for joint_elem in state_elem.findall("{*}joint"):
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

        try:
            return GroupState(name=name, group=group, joint_values=joint_values)
        except Exception as e:
            logger.warning(f"SRDF: Skipping group state '{name}': {e}")
            return None

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
        self._validate_content(content)

        try:
            root = ET.fromstring(content)
        except ET.ParseError as e:
            raise RobotParserUnexpectedError(source_area="SRDF parse", original_error=e) from e
        except Exception as e:
            raise RobotParserUnexpectedError(
                source_area="Unexpected SRDF parse", original_error=e
            ) from e

        if strip_xml_namespace(root.tag) != "robot":
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

        # 1. Collect all semantic elements using the generic helper
        virtual_joints = self._collect_elements(
            root, "virtual_joint", self._parse_virtual_joint_elem
        )
        groups = self._collect_elements(root, "group", self._parse_planning_group)
        group_states = self._collect_elements(root, "group_state", self._parse_group_state)
        end_effectors = self._collect_elements(root, "end_effector", self._parse_end_effector_elem)
        disabled_collisions = self._collect_elements(
            root, "disable_collisions", self._parse_collision_pair_elem
        )
        enabled_collisions = self._collect_elements(
            root, "enable_collisions", self._parse_collision_pair_elem
        )
        link_sphere_approximations = self._collect_elements(
            root, "link_sphere_approximation", self._parse_link_sphere_approximation_elem
        )
        joint_properties = self._collect_elements(
            root, "joint_property", self._parse_joint_property_elem
        )

        # 2. Parse Passive Joints (Direct mapping, no special helper needed)
        passive_joints: list[PassiveJoint] = []
        for pj_elem in root.findall("{*}passive_joint"):
            pj_name = pj_elem.get("name")
            if pj_name:
                passive_joints.append(PassiveJoint(name=pj_name))
            else:
                logger.warning("SRDF: Passive joint missing name, skipping")

        # 3. Parse Default Collision Link Rules
        no_default_collision_links: list[str] = []
        for ddc_elem in root.findall("{*}disable_default_collisions"):
            link = ddc_elem.get("link")
            if link:
                no_default_collision_links.append(link)
            else:
                logger.warning("SRDF: disable_default_collisions missing link attribute, skipping")

        # 4. Final cross-reference validation
        self._validate_cross_references(groups, group_states, end_effectors)

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

    def _validate_cross_references(
        self,
        groups: list[PlanningGroup],
        group_states: list[GroupState],
        end_effectors: list[EndEffector],
    ) -> None:
        """Validate that group states and end effectors refer to existing groups.

        Args:
            groups: List of parsed planning groups.
            group_states: List of parsed group states.
            end_effectors: List of parsed end effectors.
        """
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

    def _parse_virtual_joint_elem(self, elem: ET.Element) -> VirtualJoint | None:
        """Parse a <virtual_joint> element into a VirtualJoint model.

        Args:
            elem: The XML element for the virtual joint.

        Returns:
            A populated VirtualJoint instance, or None if invalid.
        """
        name = elem.get("name")
        vtype = elem.get("type")
        parent = elem.get("parent_frame")
        child = elem.get("child_link")

        if not name or not vtype or not parent or not child:
            logger.warning("SRDF: Virtual joint missing required attributes, skipping")
            return None

        try:
            return VirtualJoint(
                name=name,
                type=vtype,
                parent_frame=parent,
                child_link=child,
            )
        except Exception as e:
            logger.warning(f"SRDF: Skipping virtual joint '{name}': {e}")
            return None

    def _parse_end_effector_elem(self, elem: ET.Element) -> EndEffector | None:
        """Parse an <end_effector> element into an EndEffector model.

        Args:
            elem: The XML element for the end effector.

        Returns:
            A populated EndEffector instance, or None if invalid.
        """
        name = elem.get("name")
        group = elem.get("group")
        parent = elem.get("parent_link")

        if not name or not group or not parent:
            logger.warning("SRDF: End effector missing required attributes, skipping")
            return None

        try:
            return EndEffector(
                name=name,
                group=group,
                parent_link=parent,
                parent_group=elem.get("parent_group"),
            )
        except Exception as e:
            logger.warning(f"SRDF: Skipping end effector '{name}': {e}")
            return None

    def _parse_collision_pair_elem(self, elem: ET.Element) -> CollisionPair | None:
        """Parse a collision rule element into a CollisionPair model.

        Args:
            elem: The XML element for the collision pair (disable/enable).

        Returns:
            A populated CollisionPair instance, or None if invalid.
        """
        link1 = elem.get("link1")
        link2 = elem.get("link2")

        if not link1 or not link2:
            logger.warning("SRDF: Collision pair missing link1 or link2, skipping")
            return None

        try:
            return CollisionPair(
                link1=link1,
                link2=link2,
                reason=elem.get("reason"),
            )
        except Exception as e:
            logger.warning(f"SRDF: Skipping collision pair '{link1}/{link2}': {e}")
            return None

    def _parse_link_sphere_approximation_elem(
        self, elem: ET.Element
    ) -> LinkSphereApproximation | None:
        """Parse a <link_sphere_approximation> element into a model.

        Args:
            elem: The XML element for the sphere approximation.

        Returns:
            A populated LinkSphereApproximation instance, or None if invalid.
        """
        link = elem.get("link")
        if not link:
            logger.warning("SRDF: Link sphere approximation missing link attribute, skipping")
            return None

        spheres: list[SrdfSphere] = []
        for sphere_elem in elem.findall("{*}sphere"):
            center_str = sphere_elem.get("center")
            radius_str = sphere_elem.get("radius")
            if not center_str or not radius_str:
                logger.warning(f"SRDF: Sphere in link '{link}' missing center or radius, skipping")
                continue
            try:
                cx, cy, cz = (parse_float(v, "sphere center") for v in center_str.split())
                r = parse_float(radius_str, "sphere radius")
                spheres.append(SrdfSphere(center_x=cx, center_y=cy, center_z=cz, radius=r))
            except Exception as e:
                logger.warning(f"SRDF: Invalid sphere in link '{link}': {e}")

        try:
            return LinkSphereApproximation(link=link, spheres=tuple(spheres))
        except Exception as e:
            logger.warning(f"SRDF: Skipping sphere approximation for link '{link}': {e}")
            return None

    def _parse_joint_property_elem(self, elem: ET.Element) -> JointProperty | None:
        """Parse a <joint_property> element into a JointProperty model.

        Args:
            elem: The XML element for the joint property.

        Returns:
            A populated JointProperty instance, or None if invalid.
        """
        joint_name = elem.get("joint_name")
        property_name = elem.get("property_name")
        value = elem.get("value")

        if not joint_name or not property_name or not value:
            logger.warning("SRDF: Joint property missing required attributes, skipping")
            return None

        try:
            return JointProperty(
                joint_name=joint_name,
                property_name=property_name,
                value=value,
            )
        except Exception as e:
            logger.warning(f"SRDF: Skipping joint property for '{joint_name}': {e}")
            return None

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
        self._validate_file(filepath)

        try:
            content = filepath.read_text(encoding="utf-8")
            return self.parse_string(content, **kwargs)

        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotParserIOError(filepath=filepath, reason=str(e)) from e
