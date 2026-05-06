"""SRDF XML parser for LinkForge.

This module implements a robust SRDF (Semantic Robot Description Format) parser
that supports MoveIt-style tags and native XACRO resolution.
"""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET
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
from ..utils.xml_utils import (
    MAX_XML_DEPTH,
    XACRO_URIS,
    get_xml_namespace,
    parse_float,
    strip_xml_namespace,
)
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

    def _detect_xacro_content(self, root: ET.Element) -> None:
        """Detect if the XML contains unexpanded XACRO macros.

        Args:
            root: The XML root element.

        Raises:
            RobotParserUnexpectedError: If XACRO is detected.

        """
        is_xacro = False
        for child in root:
            if get_xml_namespace(child.tag) in XACRO_URIS:
                is_xacro = True
                break

        if not is_xacro:
            for elem in root.iter():
                if any("${" in v or "$(" in v for v in elem.attrib.values() if isinstance(v, str)):
                    is_xacro = True
                    break

        if is_xacro:
            raise RobotParserUnexpectedError(
                source_area="SRDF Parser",
                original_error="Unexpanded XACRO detected in SRDF. Please use parse_xacro().",
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
        **_kwargs: Any,
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
        f = io.StringIO(content)
        try:
            context = ET.iterparse(f, events=("start", "end"))
            _, root = next(context)
        except ET.ParseError as e:
            raise RobotParserUnexpectedError(source_area="SRDF parse", original_error=e) from e
        except StopIteration:
            raise RobotParserUnexpectedError(
                source_area="SRDF parse", original_error="Empty or truncated XML"
            ) from None
        except Exception as e:
            raise RobotParserUnexpectedError(
                source_area="Unexpected SRDF parse", original_error=e
            ) from e

        if strip_xml_namespace(root.tag) != "robot":
            raise RobotParserXMLRootError(root.tag)

        try:
            return self._parse_from_context(context, root)
        except ET.ParseError as e:
            raise RobotParserUnexpectedError(source_area="SRDF parse", original_error=e) from e
        except RobotParserError:
            raise
        except Exception as e:
            raise RobotParserUnexpectedError(
                source_area="Unexpected SRDF parse", original_error=e
            ) from e

    def _parse_from_context(self, context: Any, root: ET.Element) -> SemanticRobotDescription:
        """Process iterative XML parsing internally for O(1) memory complexity.

        Args:
            context: The iterparse context.
            root: The root <robot> element.

        Returns:
            A fully populated SemanticRobotDescription model.

        Raises:
            RobotParserUnexpectedError: If XML nesting exceeds MAX_XML_DEPTH.

        """
        self._detect_xacro_content(root)

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

        depth = 0
        for event, elem in context:
            if event == "start":
                depth += 1
                if depth > MAX_XML_DEPTH:
                    raise RobotParserUnexpectedError(
                        source_area="XML nesting", original_error=depth
                    )
            elif event == "end":
                if depth == 1:
                    tag = strip_xml_namespace(elem.tag)

                    if tag == "virtual_joint":
                        vj = self._parse_virtual_joint_elem(elem)
                        if vj:
                            virtual_joints.append(vj)
                    elif tag == "group":
                        g = self._parse_planning_group(elem)
                        if g:
                            groups.append(g)
                    elif tag == "group_state":
                        gs = self._parse_group_state(elem)
                        if gs:
                            group_states.append(gs)
                    elif tag == "end_effector":
                        ee = self._parse_end_effector_elem(elem)
                        if ee:
                            end_effectors.append(ee)
                    elif tag == "disable_collisions":
                        cp = self._parse_collision_pair_elem(elem)
                        if cp:
                            disabled_collisions.append(cp)
                    elif tag == "enable_collisions":
                        cp = self._parse_collision_pair_elem(elem)
                        if cp:
                            enabled_collisions.append(cp)
                    elif tag == "passive_joint":
                        pj_name = elem.get("name")
                        if pj_name:
                            passive_joints.append(PassiveJoint(name=pj_name))
                    elif tag == "disable_default_collisions":
                        link = elem.get("link")
                        if link:
                            no_default_collision_links.append(link)
                    elif tag == "link_sphere_approximation":
                        lsa = self._parse_link_sphere_approximation_elem(elem)
                        if lsa:
                            link_sphere_approximations.append(lsa)
                    elif tag == "joint_property":
                        jp = self._parse_joint_property_elem(elem)
                        if jp:
                            joint_properties.append(jp)

                    # Clear element to free memory (O(1) complexity)
                    root.clear()
                depth -= 1

        # Final cross-reference validation
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

    def parse(self, filepath: Path, **_kwargs: Any) -> SemanticRobotDescription:
        """Load and parse an SRDF file from disk.

        Args:
            filepath: Path to the .srdf file.
            **kwargs: Additional options (unused).

        Returns:
            A SemanticRobotDescription model.

        Raises:
            RobotParserIOError: If the file is missing or exceeds max_file_size.
            RobotParserXMLRootError: If the root tag is not <robot>.

        """
        self._validate_file(filepath)

        try:
            context = ET.iterparse(str(filepath), events=("start", "end"))
            _, root = next(context)

            if strip_xml_namespace(root.tag) != "robot":
                raise RobotParserXMLRootError(root.tag)

            return self._parse_from_context(context, root)

        except ET.ParseError as e:
            raise RobotParserUnexpectedError(source_area="SRDF file parse", original_error=e) from e
        except StopIteration:
            raise RobotParserUnexpectedError(
                source_area="SRDF file parse", original_error="Empty or truncated XML"
            ) from None
        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotParserIOError(filepath=filepath, reason=str(e)) from e
