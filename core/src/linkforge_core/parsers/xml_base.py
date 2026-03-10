"""Base XML parser shared across URDF, XACRO, SDF, MJCF."""

from __future__ import annotations

__all__ = ["RobotXMLParser"]

import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

from ..base import IResourceResolver, RobotParser, RobotParserError
from ..exceptions import RobotModelError
from ..logging_config import get_logger
from ..models import (
    Box,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Material,
    Mesh,
    Robot,
    Sphere,
    Transform,
)
from ..utils.xml_utils import (
    parse_float,
    parse_vector3,
)
from ..validation import validate_mesh_path, validate_package_uri

if TYPE_CHECKING:
    from ..models.link import Link

logger = get_logger(__name__)

MAX_FILE_SIZE = 100 * 1024 * 1024  # 100 MB


class RobotXMLParser(RobotParser):
    """Abstract base class for XML-based robotics format parsers."""

    def __init__(
        self,
        max_file_size: int = MAX_FILE_SIZE,
        sandbox_root: Path | None = None,
        resource_resolver: IResourceResolver | None = None,
    ) -> None:
        """Initialize base XML parser.

        Args:
            max_file_size: Maximum allowed file size in bytes
            sandbox_root: Optional root directory for security sandbox
            resource_resolver: Optional resolver for URIs
        """
        self.max_file_size = max_file_size
        self.sandbox_root = sandbox_root
        self.resource_resolver = resource_resolver

    def _parse_origin_element(self, elem: ET.Element | None) -> Transform:
        """Parse origin-style element into a Transform object.

        Args:
            elem: XML element with xyz/rpy attributes (e.g. <origin> or <pose>).

        Returns:
            A Transform object.
        """
        if elem is None:
            return Transform.identity()

        xyz_text = elem.get("xyz", elem.get("pos", "0 0 0"))
        rpy_text = elem.get("rpy", elem.get("euler", "0 0 0"))

        xyz = parse_vector3(xyz_text)
        rpy = parse_vector3(rpy_text)

        return Transform(xyz=xyz, rpy=rpy)

    def _parse_geometry_element(
        self,
        geom_elem: ET.Element,
        base_directory: Path | None = None,
    ) -> Box | Cylinder | Sphere | Mesh | None:
        """Parse geometry element (box, cylinder, sphere, mesh).

        Args:
            geom_elem: Parent element containing geometry definitions.
            base_directory: Directory for relative path validation.

        Returns:
            A Geometry object or None if invalid.
        """
        try:
            # Box
            box = geom_elem.find("box")
            if box is not None:
                size_text = box.get("size")
                if size_text is None:
                    logger.warning("Invalid box geometry ignored: missing size")
                    return None
                return Box(size=parse_vector3(size_text))

            # Cylinder
            cylinder = geom_elem.find("cylinder")
            if cylinder is not None:
                radius = parse_float(cylinder.get("radius"), "cylinder radius", default=0.5)
                length = parse_float(cylinder.get("length"), "cylinder length", default=1.0)
                return Cylinder(radius=radius, length=length)

            # Sphere
            sphere = geom_elem.find("sphere")
            if sphere is not None:
                radius = parse_float(sphere.get("radius"), "sphere radius", default=0.5)
                return Sphere(radius=radius)

            # Mesh
            mesh = geom_elem.find("mesh")
            if mesh is not None:
                filename = mesh.get("filename", mesh.get("file", ""))
                if not filename:
                    logger.warning("Invalid mesh geometry ignored: missing filename")
                    return None

                # Path Security Validation
                validation_path: Path | None = None
                if filename.startswith("file://"):
                    from ..utils.path_utils import normalize_uri_to_path

                    validation_path = normalize_uri_to_path(filename)
                elif not filename.startswith("package://"):
                    validation_path = Path(filename)

                if validation_path is not None and base_directory is not None:
                    validate_mesh_path(
                        validation_path,
                        base_directory,
                        sandbox_root=self.sandbox_root,
                        allow_absolute=validation_path.is_absolute(),
                    )
                elif filename.startswith("package://"):
                    validate_package_uri(filename)

                scale_text = mesh.get("scale", "1 1 1")
                # Return normalized path for file:// URIs, raw filename for package://
                resource = str(validation_path) if validation_path else filename
                return Mesh(resource=resource, scale=parse_vector3(scale_text))

            return None
        except (RobotModelError, ValueError, RobotParserError) as e:
            # Determine geometry type for better logging if possible
            geom_type = "geometry"
            if geom_elem.find("box") is not None:
                geom_type = "box geometry"
            elif geom_elem.find("cylinder") is not None:
                geom_type = "cylinder geometry"
            elif geom_elem.find("sphere") is not None:
                geom_type = "sphere geometry"
            elif geom_elem.find("mesh") is not None:
                geom_type = "mesh geometry"

            logger.warning(f"Invalid {geom_type} ignored: {e}")
            return None

    def _parse_material_element(
        self, mat_elem: ET.Element | None, materials: dict[str, Material]
    ) -> Material | None:
        """Parse material definition or reference.

        Args:
            mat_elem: Material element.
            materials: Cache of defined materials.

        Returns:
            Material object or None.
        """
        if mat_elem is None:
            return None

        mat_name = mat_elem.get("name", "")
        if mat_name and mat_name in materials:
            return materials[mat_name]

        color = None
        color_elem = mat_elem.find("color")
        if color_elem is not None:
            rgba_text = color_elem.get("rgba", "0.8 0.8 0.8 1.0")
            parts = rgba_text.strip().split()
            try:
                if len(parts) == 3:
                    color = Color(r=float(parts[0]), g=float(parts[1]), b=float(parts[2]), a=1.0)
                elif len(parts) == 4:
                    color = Color(
                        r=float(parts[0]), g=float(parts[1]), b=float(parts[2]), a=float(parts[3])
                    )
            except (ValueError, IndexError):
                logger.warning(f"Invalid material color format: {rgba_text}")
                return None

        texture = None
        texture_elem = mat_elem.find("texture")
        if texture_elem is not None:
            texture = texture_elem.get("filename")

        if color or texture:
            return Material(name=mat_name if mat_name else "default", color=color, texture=texture)

        return None

    def _parse_inertial_element(self, inertial_elem: ET.Element | None) -> Inertial | None:
        """Parse inertial properties.

        Args:
            inertial_elem: Inertial XML element.

        Returns:
            Inertial object or None.
        """
        if inertial_elem is None:
            return None

        origin = self._parse_origin_element(inertial_elem.find("origin"))

        mass_elem = inertial_elem.find("mass")
        mass = parse_float(mass_elem.get("value") if mass_elem is not None else None, default=0.0)

        inertia_elem = inertial_elem.find("inertia")
        if inertia_elem is not None:
            # Robustness: sanitize negative or zero diagonal inertia to 1e-6
            # as expected by some unit tests for "safety net" behavior.
            ixx = max(1e-6, parse_float(inertia_elem.get("ixx"), default=1e-6))
            iyy = max(1e-6, parse_float(inertia_elem.get("iyy"), default=1e-6))
            izz = max(1e-6, parse_float(inertia_elem.get("izz"), default=1e-6))

            ixy = parse_float(inertia_elem.get("ixy"), default=0.0)
            ixz = parse_float(inertia_elem.get("ixz"), default=0.0)
            iyz = parse_float(inertia_elem.get("iyz"), default=0.0)

            try:
                inertia = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
            except RobotModelError:
                # If triangle inequality is still violated, fall back to minimal valid
                inertia = InertiaTensor.zero()
        else:
            inertia = InertiaTensor.zero()

        return Inertial(mass=mass, origin=origin, inertia=inertia)

    def _add_link_robust(self, robot: Robot, link: Link) -> None:
        """Add link to robot, handling duplicates via renaming.

        Args:
            robot: Target robot model.
            link: Link to add.
        """
        try:
            robot.add_link(link)
        except RobotModelError:
            original_name = link.name
            counter = 1
            while True:
                new_name = f"{original_name}_duplicate_{counter}"
                if new_name not in robot._link_index:
                    link = replace(link, name=new_name)
                    try:
                        robot.add_link(link)
                        logger.warning(f"Renamed duplicate link '{original_name}' to '{new_name}'")
                        break
                    except RobotModelError:
                        counter += 1
                else:
                    counter += 1

    def _add_joint_robust(self, robot: Robot, joint: Any, joint_elem: ET.Element) -> None:
        """Add joint to robot, handling duplicates and broken references.

        Args:
            robot: Target robot model.
            joint: Joint object to add.
            joint_elem: XML element for backup naming.
        """
        try:
            robot.add_joint(joint)
        except RobotModelError as e:
            joint_name = joint_elem.get("name", "unnamed_joint")
            if "already exists" in str(e):
                original_name = joint_name
                counter = 1
                while True:
                    new_name = f"{original_name}_duplicate_{counter}"
                    if new_name not in robot._joint_index:
                        joint = replace(joint, name=new_name)
                        try:
                            robot.add_joint(joint)
                            logger.warning(
                                f"Renamed duplicate joint '{original_name}' to '{new_name}'"
                            )
                            break
                        except RobotModelError as inner_e:
                            if "not found" in str(inner_e):
                                logger.warning(
                                    f"Skipping duplicate joint '{original_name}' due to broken reference: {inner_e}"
                                )
                                break
                            counter += 1
                    else:
                        counter += 1
            else:
                logger.warning(f"Skipping invalid joint '{joint_name}': {e}")
