"""XACRO generation from robot models.

Transforms internal models into modular, parameterized XACRO descriptions.
Supports global property extraction, Pattern-based macro generation,
and file splitting for maintainability.
"""

from __future__ import annotations

import contextlib
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any

from ..base import RobotGeneratorError
from ..models.geometry import Box, Cylinder, Geometry, Mesh, Sphere
from ..models.joint import Joint
from ..models.link import Link, Visual
from ..models.material import Material
from ..models.robot import Robot
from ..utils.math_utils import format_float, format_vector
from ..utils.string_utils import sanitize_name
from ..utils.xml_utils import serialize_xml
from .urdf_generator import URDFGenerator

# XACRO Namespace URI
XACRO_URI = "http://www.ros.org/wiki/xacro"
XACRO_NS = f"{{{XACRO_URI}}}"


class XACROGenerator(URDFGenerator):
    """Generate XACRO from Robot model.

    Supports material and dimension extraction, auto-macro generation,
    and modular file splitting.
    """

    def __init__(
        self,
        pretty_print: bool = True,
        advanced_mode: bool = True,
        extract_materials: bool = True,
        extract_dimensions: bool = True,
        generate_macros: bool = False,
        split_files: bool = False,
        urdf_path: Path | None = None,
        use_ros2_control: bool = True,
    ) -> None:
        """Initialize XACRO generator.

        Args:
            pretty_print: If True, format XML with indentation
            advanced_mode: Enable advanced XACRO features
            extract_materials: Extract material colors as properties
            extract_dimensions: Extract common dimensions as properties
            generate_macros: Auto-generate macros for repeated patterns
            split_files: Split into multiple files (materials, macros, robot)
            urdf_path: Path where XACRO will be saved (for relative mesh paths)
            use_ros2_control: Whether to generate ROS2 Control from transmissions

        """
        super().__init__(
            pretty_print=pretty_print,
            urdf_path=urdf_path,
            use_ros2_control=use_ros2_control,
        )
        self.advanced_mode = advanced_mode
        self.extract_materials = extract_materials if advanced_mode else False
        self.extract_dimensions = extract_dimensions if advanced_mode else False
        self.generate_macros = generate_macros if advanced_mode else False
        self.split_files = split_files if advanced_mode else False

        # Track extracted properties
        self.material_properties: dict[str, str] = {}
        self.dimension_properties: dict[str, str] = {}
        self.generated_macros: list[dict[str, Any]] = []

    def generate(self, robot: Robot, validate: bool = True, **kwargs: Any) -> str:
        """Generate XACRO XML string from robot."""
        from .. import __version__

        root = self.generate_robot_element(robot, validate=validate)
        return serialize_xml(root, pretty_print=self.pretty_print, version=__version__)

    def generate_robot_element(self, robot: Robot, validate: bool = True) -> ET.Element:
        """Generate XACRO XML Element tree from robot.

        This is used internally by generate() and for multi-file exports
        to preserve structural comments and metadata.

        Args:
            robot: Robot model to convert
            validate: Whether to validate robot structure before generation

        Returns:
            XACRO XML Element tree
        """
        # Validate robot structure
        if validate:
            from ..validation import RobotValidator

            validator = RobotValidator(robot)
            result = validator.validate()
            if not result.is_valid:
                error_msgs = [str(issue) for issue in result.errors]
                raise RobotGeneratorError("Robot validation failed:\n" + "\n".join(error_msgs))

        # Reset property tracking
        self.material_properties = {}
        self.dimension_properties = {}
        self.generated_macros = []

        # Create root element
        root = ET.Element("robot", name=robot.name)

        # 1. Collect properties (Materials & Dimensions)
        properties: list[tuple[str, str]] = []

        if self.advanced_mode:
            if self.extract_materials:
                self._extract_materials(robot, properties)

            if self.extract_dimensions:
                self._extract_dimensions(robot, properties)

        # 2. Identify Macros
        self.macro_groups = {}
        self.links_in_macros = set()

        if self.generate_macros:
            self.macro_groups = self._identify_macro_groups(robot)
            for group in self.macro_groups.values():
                for link, _ in group:
                    self.links_in_macros.add(link.name)

        # 3. Add Properties to Root
        if properties:
            root.append(ET.Comment(" Properties "))
        for prop_name, prop_value in properties:
            ET.SubElement(root, f"{XACRO_NS}property", name=prop_name, value=str(prop_value))

        # Collect global materials (always needed for proper XACRO structure)
        self.global_materials = self._collect_materials(robot)

        # Add global material definitions (after properties, before macros)
        # This follows standard URDF/XACRO practice: define materials once, reference by name
        if self.global_materials:
            root.append(ET.Comment(" Materials "))
        if self.extract_materials:
            # Add global material definitions with property references
            for material in self.global_materials.values():
                mat_elem = ET.SubElement(root, "material", name=material.name)
                if material.name in self.material_properties:
                    prop_name = self.material_properties[material.name]
                    ET.SubElement(mat_elem, "color", rgba=f"${{{prop_name}}}")
                elif material.color:
                    rgba = f"{format_float(material.color.r)} {format_float(material.color.g)} {format_float(material.color.b)} {format_float(material.color.a)}"
                    ET.SubElement(mat_elem, "color", rgba=rgba)
                if material.texture:
                    ET.SubElement(mat_elem, "texture", filename=material.texture)
        else:
            # Standard URDF behavior: add global materials without properties
            for material in self.global_materials.values():
                self._add_material_element(root, material)

        # 4. Generate Macro Definitions
        if self.generate_macros and self.macro_groups:
            root.append(ET.Comment(" Macros "))
        if self.generate_macros:
            for signature, group in self.macro_groups.items():
                self._generate_macro_definition(root, signature, group)

        # 5. Generate Links and Joints (using unified Template Method)
        self.add_links_section(root, robot)
        self.add_joints_section(root, robot)

        # Add transmissions
        self.add_transmissions(root, robot)

        # Add ROS2 Control
        if self.use_ros2_control:
            self.add_ros2_control(root, robot)

        # Add Gazebo extensions (robot level)
        self.add_gazebo(root, robot)

        # Add sensors (Gazebo format)
        self.add_sensors(root, robot)

        return root

    def _add_link_to_xml(self, parent: ET.Element, link: Link, robot: Robot) -> None:
        """Override: Check for macro usage before adding link."""
        # Check if this link is part of a macro group
        if link.name in self.links_in_macros:
            # Find the joint for this link
            joint = None
            for j in robot.joints:
                if j.child == link.name:
                    joint = j
                    break

            if joint:
                # Find which group it belongs to
                link_sig = self._get_macro_signature(link, joint)
                if link_sig and link_sig in self.macro_groups:
                    # Generate Macro Call
                    self._generate_macro_call(parent, link_sig, link, joint)
        else:
            # Standard Link Generation
            self._add_link_element(parent, link)

    def _add_joint_to_xml(self, parent: ET.Element, joint: Joint, robot: Robot) -> None:
        """Override: Skip joint if it's included in a macro call."""
        # If joint is part of a macro (child link is in macro), skip it
        # because the macro call includes the joint
        if joint.child in self.links_in_macros:
            return

        # Standard Joint Generation
        self._add_joint_element(parent, joint)

    def _extract_materials(self, robot: Robot, properties: list[tuple[str, str]]) -> None:
        """Extract unique materials as XACRO properties.

        Args:
            robot: Robot model to scan for materials
            properties: List to append (name, value) tuples to
        """
        materials = self._collect_materials(robot)
        for mat_name, mat in materials.items():
            # Skip materials without color (should not happen, but type-safe)
            if mat.color is None:
                continue
            # Sanitize property name for XACRO (Python identifier: no hyphens, no leading digits)
            prop_name = sanitize_name(mat_name.lower(), allow_hyphen=False)
            prop_value = (
                f"{format_float(mat.color.r)} {format_float(mat.color.g)} "
                f"{format_float(mat.color.b)} {format_float(mat.color.a)}"
            )
            properties.append((prop_name, prop_value))
            self.material_properties[mat_name] = prop_name

    def _extract_dimensions(self, robot: Robot, properties: list[tuple[str, str]]) -> None:
        """Extract common dimensions as XACRO properties.

        Scans all geometries in the robot and identifies dimensions that appear
        multiple times with the same value. These are extracted as properties
        for better maintainability.

        Example:
            4 wheels with radius=0.05 → <xacro:property name="wheel_radius" value="0.05"/>

        Args:
            robot: Robot model to scan
            properties: List to append (name, value) tuples to
        """
        # Collect all dimensions from visual geometries
        # Format: {dimension_key: [(link_name, value), ...]}
        dimensions: dict[str, list[tuple[str, float]]] = defaultdict(list)

        for link in robot.links:
            for visual in link.visuals:
                geom = visual.geometry

                # Extract dimensions based on geometry type
                if isinstance(geom, Cylinder):
                    dimensions["cylinder_radius"].append((link.name, geom.radius))
                    dimensions["cylinder_length"].append((link.name, geom.length))
                elif isinstance(geom, Box):
                    dimensions["box_x"].append((link.name, geom.size.x))
                    dimensions["box_y"].append((link.name, geom.size.y))
                    dimensions["box_z"].append((link.name, geom.size.z))
                elif isinstance(geom, Sphere):
                    dimensions["sphere_radius"].append((link.name, geom.radius))

        # Find repeated dimensions (2+ occurrences with same value)
        for dim_key, dim_values in dimensions.items():
            # Group by value (with tolerance for floating-point comparison)
            value_groups = self._group_dimensions_by_value(dim_values)

            for value, link_names in value_groups.items():
                if len(link_names) >= 2:  # Only extract if repeated
                    # Generate property name
                    prop_name = self._generate_dimension_property_name(dim_key, link_names)

                    # Add to properties
                    properties.append((prop_name, format_float(value)))

                    # Store mapping for substitution during geometry generation
                    # Key format: "dimension_key:value" → property_name
                    self.dimension_properties[f"{dim_key}:{value}"] = prop_name

    def _group_dimensions_by_value(
        self, dim_values: list[tuple[str, float]]
    ) -> dict[float, list[str]]:
        """Group dimensions by value with floating-point tolerance.

        Args:
            dim_values: List of (link_name, dimension_value) tuples

        Returns:
            Dictionary mapping rounded values to lists of link names
        """
        groups: dict[float, list[str]] = defaultdict(list)

        for link_name, value in dim_values:
            # Round to 3 decimals for grouping (tolerance = 0.001)
            rounded = round(value, 3)
            groups[rounded].append(link_name)

        return groups

    def _generate_dimension_property_name(self, dim_key: str, link_names: list[str]) -> str:
        """Generate descriptive property name for a dimension.

        Attempts to create readable names by finding common prefixes in link names
        and mapping dimension keys to readable suffixes.

        Examples:
            cylinder_radius + [fl_wheel, fr_wheel, rl_wheel, rr_wheel] → "wheel_radius"
            box_z + [left_leg, right_leg] → "leg_height"
            sphere_radius + [ball1, ball2] → "ball_radius"

        Args:
            dim_key: Dimension key (e.g., "cylinder_radius", "box_x")
            link_names: List of link names that share this dimension

        Returns:
            Property name (e.g., "wheel_radius", "leg_height")
        """
        # Find common prefix in link names
        common_prefix = self._find_common_prefix(link_names)

        # Map dimension keys to readable names
        dim_map = {
            "cylinder_radius": "radius",
            "cylinder_length": "length",
            "box_x": "width",
            "box_y": "depth",
            "box_z": "height",
            "sphere_radius": "radius",
        }

        readable_dim = dim_map.get(dim_key, dim_key)

        # Sanitize for XACRO property (must be valid Python identifier)
        if common_prefix:
            prop_name = f"{common_prefix}_{readable_dim}"
        else:
            # No common prefix, use dimension type
            geom_type = dim_key.split("_")[0]  # "cylinder", "box", "sphere"
            prop_name = f"{geom_type}_{readable_dim}"

        return sanitize_name(prop_name, allow_hyphen=False)

    def _find_common_prefix(self, names: list[str]) -> str:
        """Find common prefix in a list of names.

        Examples:
            [fl_wheel, fr_wheel, rl_wheel, rr_wheel] → "wheel"
            [left_leg, right_leg] → "leg"
            [link1, link2, link3] → "link"

        Args:
            names: List of names to analyze

        Returns:
            Common prefix (without trailing underscore), or empty string if none
        """
        if not names:
            return ""

        # Split names by underscore and find common suffix
        # (e.g., "fl_wheel" → ["fl", "wheel"], common suffix is "wheel")
        name_parts = [name.split("_") for name in names]

        # Try to find common suffix (most common case: "wheel", "leg", etc.)
        if all(len(parts) >= 2 for parts in name_parts):
            # Check if last part is common
            last_parts = [parts[-1] for parts in name_parts]
            if len(set(last_parts)) == 1:
                return last_parts[0]

        # Fallback: find common prefix
        prefix = names[0]
        for name in names[1:]:
            # Find common prefix between current prefix and this name
            i = 0
            while i < len(prefix) and i < len(name) and prefix[i] == name[i]:
                i += 1
            prefix = prefix[:i]

        # Remove trailing underscore
        return prefix.rstrip("_")

    def _identify_macro_groups(self, robot: Robot) -> dict[str, list[tuple[Link, Joint | None]]]:
        """Group links by geometry and joint functional signature."""
        link_groups: dict[str, list[tuple[Link, Joint | None]]] = defaultdict(list)

        for link in robot.links:
            # Find the joint for this link
            joint = None
            for j in robot.joints:
                if j.child == link.name:
                    joint = j
                    break

            # Create signature based on both link geometry and joint properties
            # If no joint, we don't group it into a macro (standard practice)
            if joint:
                signature = self._get_macro_signature(link, joint)
                if signature:
                    link_groups[signature].append((link, joint))

        # Filter for groups with 2+ members
        return {k: v for k, v in link_groups.items() if len(v) >= 2}

    def _get_macro_signature(self, link: Link, joint: Joint) -> str | None:
        """Create a signature string for a macro based on link geometry and joint properties.

        Includes all visuals, collisions, and their relative origins to ensure
        that only truly identical links are grouped into macros.
        """
        if not link.visuals:
            return None

        parts = []

        # Process all visuals
        for visual in link.visuals:
            geom = visual.geometry
            parts.append(f"v_{geom.type.value}")

            if isinstance(geom, Box):
                parts.extend([f"{geom.size.x:.3f}", f"{geom.size.y:.3f}", f"{geom.size.z:.3f}"])
            elif isinstance(geom, Cylinder):
                parts.extend([f"{geom.radius:.3f}", f"{geom.length:.3f}"])
            elif isinstance(geom, Sphere):
                parts.extend([f"{geom.radius:.3f}"])
            elif isinstance(geom, Mesh):
                parts.append(str(geom.filepath))

            # Include visual origin (Critical for transform fidelity)
            if visual.origin:
                xyz = visual.origin.xyz
                rpy = visual.origin.rpy
                parts.extend(
                    [
                        f"p_{xyz.x:.3f}_{xyz.y:.3f}_{xyz.z:.3f}",
                        f"r_{rpy.x:.3f}_{rpy.y:.3f}_{rpy.z:.3f}",
                    ]
                )

            # Include material
            if visual.material:
                parts.append(visual.material.name)

        # Process all collisions
        for collision in link.collisions:
            geom = collision.geometry
            parts.append(f"c_{geom.type.value}")

            if isinstance(geom, Box):
                parts.extend([f"{geom.size.x:.3f}", f"{geom.size.y:.3f}", f"{geom.size.z:.3f}"])
            elif isinstance(geom, Cylinder):
                parts.extend([f"{geom.radius:.3f}", f"{geom.length:.3f}"])
            elif isinstance(geom, Sphere):
                parts.extend([f"{geom.radius:.3f}"])
            elif isinstance(geom, Mesh):
                parts.append(str(geom.filepath))

            # Include collision origin
            if collision.origin:
                xyz = collision.origin.xyz
                rpy = collision.origin.rpy
                parts.extend(
                    [
                        f"p_{xyz.x:.3f}_{xyz.y:.3f}_{xyz.z:.3f}",
                        f"r_{rpy.x:.3f}_{rpy.y:.3f}_{rpy.z:.3f}",
                    ]
                )

        # --- Include Joint Properties in Signature ---
        parts.append(f"j_{joint.type.value}")

        if joint.axis:
            parts.append(f"a_{joint.axis.x:.3f}_{joint.axis.y:.3f}_{joint.axis.z:.3f}")

        if joint.limits:
            parts.append(f"l_{joint.limits.effort:.3f}_{joint.limits.velocity:.3f}")
            if joint.limits.lower is not None:
                parts.append(f"{joint.limits.lower:.3f}")
            if joint.limits.upper is not None:
                parts.append(f"{joint.limits.upper:.3f}")

        if joint.dynamics:
            parts.append(f"d_{joint.dynamics.damping:.3f}_{joint.dynamics.friction:.3f}")

        if joint.mimic:
            parts.append(
                f"m_{joint.mimic.joint}_{joint.mimic.multiplier:.3f}_{joint.mimic.offset:.3f}"
            )

        if joint.safety_controller:
            s = joint.safety_controller
            parts.append(
                f"s_{s.soft_lower_limit:.3f}_{s.soft_upper_limit:.3f}_{s.k_position:.3f}_{s.k_velocity:.3f}"
            )

        if joint.calibration:
            c = joint.calibration
            rising = f"{c.rising:.3f}" if c.rising is not None else "N"
            falling = f"{c.falling:.3f}" if c.falling is not None else "N"
            parts.append(f"cal_{rising}_{falling}")

        return "_".join(parts)

    def _get_macro_name(self, signature: str) -> str:
        """Generate a consistent macro name from a signature.

        Format: {geom_type}_{short_hash}_macro
        Example: cylinder_a1b2_macro

        Args:
            signature: Geometry signature string

        Returns:
            Descriptive macro name (valid XML tag name)
        """
        import hashlib

        # Extract geometry type from signature (e.g., "v_cylinder_..." -> "cylinder")
        parts = signature.split("_")
        geom_type = parts[1] if len(parts) >= 2 and parts[0] in ("v", "c") else parts[0]

        short_hash = hashlib.md5(signature.encode()).hexdigest()[:4]
        return f"{geom_type}_{short_hash}_macro"

    def _generate_macro_definition(
        self, root: ET.Element, signature: str, group: list[tuple[Link, Joint | None]]
    ) -> None:
        """Generate <xacro:macro> definition.

        Args:
            root: Root XML element to append definition to
            signature: Geometry signature for this macro
            group: List of (link, joint) tuples that share this macro
        """
        template_link, template_joint = group[0]
        if not template_joint:
            return

        macro_name = self._get_macro_name(signature)

        macro_elem = ET.SubElement(root, f"{XACRO_NS}macro")
        macro_elem.set("name", macro_name)
        macro_elem.set("params", "name parent xyz rpy")

        # Add comment
        comment = ET.Comment(f" Macro for {len(group)} similar {macro_name}s ")
        macro_elem.append(comment)

        # --- Generate Link Body (Parameterized) ---
        link_elem = ET.SubElement(macro_elem, "link", name="${name}")
        self._add_link_contents(link_elem, template_link)

        # --- Generate Joint Body (Parameterized) ---
        joint_elem = ET.SubElement(
            macro_elem, "joint", name="${name}_joint", type=template_joint.type.value
        )

        ET.SubElement(joint_elem, "parent", link="${parent}")
        ET.SubElement(joint_elem, "child", link="${name}")
        ET.SubElement(joint_elem, "origin", xyz="${xyz}", rpy="${rpy}")

        # Use shared logic for all functional properties (Axis, Limits, Mimic, Safety Controller, Calibration)
        self._add_joint_properties(joint_elem, template_joint)

    def _generate_macro_call(
        self, root: ET.Element, signature: str, link: Link, joint: Joint | None
    ) -> None:
        """Generate <xacro:call ... /> for a link/joint pair.

        Args:
            root: Parent XML element to append call to
            signature: Geometry signature for the macro to call
            link: Link instance to parameterize the call
            joint: Joint instance providing origin data
        """
        if not joint:
            return

        macro_name = self._get_macro_name(signature)

        origin = joint.origin
        xyz = format_vector(origin.xyz.x, origin.xyz.y, origin.xyz.z)
        rpy = format_vector(origin.rpy.x, origin.rpy.y, origin.rpy.z)

        call_elem = ET.SubElement(root, f"{XACRO_NS}{macro_name}")
        call_elem.set("name", link.name)
        call_elem.set("parent", joint.parent)
        call_elem.set("xyz", xyz)
        call_elem.set("rpy", rpy)

    def _add_visual_element(self, parent: ET.Element, visual: Visual) -> None:
        """Add visual element to parent with XACRO property support.

        Args:
            parent: Parent XML element
            visual: Visual component to add
        """
        visual_elem = ET.SubElement(parent, "visual")

        if visual.name:
            visual_elem.set("name", visual.name)

        # Origin
        self._add_origin_element(visual_elem, visual.origin)

        # Geometry
        self._add_geometry_element(visual_elem, visual.geometry)

        # Material
        if visual.material:
            # Check if material is in global definitions
            is_global = (
                visual.material.name
                and visual.material.name in self.global_materials
                and self.global_materials[visual.material.name] == visual.material
            )

            if is_global:
                # Just reference the global material by name
                ET.SubElement(visual_elem, "material", name=visual.material.name)
            else:
                # Inline material (not in global definitions)
                self._add_material_element(visual_elem, visual.material)

    # Override _add_material_element to use properties
    def _add_material_element(self, parent: ET.Element, material: Material) -> None:
        """Add material element, using property reference if available.

        Args:
            parent: Parent XML element
            material: Material instance to add
        """
        mat_elem = ET.SubElement(parent, "material", name=material.name)

        if self.extract_materials and material.name in self.material_properties:
            prop_name = self.material_properties[material.name]
            ET.SubElement(mat_elem, "color", rgba=f"${{{prop_name}}}")
        else:
            # Standard URDF behavior
            if material.color:
                rgba = f"{format_float(material.color.r)} {format_float(material.color.g)} {format_float(material.color.b)} {format_float(material.color.a)}"
                ET.SubElement(mat_elem, "color", rgba=rgba)
            if material.texture:
                ET.SubElement(mat_elem, "texture", filename=material.texture)

    def _add_geometry_element(self, parent: ET.Element, geometry: Geometry) -> None:
        """Add geometry element with dimension substitution.

        Overrides parent method to substitute dimensions with XACRO properties
        when extract_dimensions is enabled.
        """
        geom_elem = ET.SubElement(parent, "geometry")

        if isinstance(geometry, Box):
            # Check if dimensions should be substituted
            x_prop = self._get_dimension_property("box_x", geometry.size.x)
            y_prop = self._get_dimension_property("box_y", geometry.size.y)
            z_prop = self._get_dimension_property("box_z", geometry.size.z)

            size_str = (
                f"${{{x_prop}}} ${{{y_prop}}} ${{{z_prop}}}"
                if x_prop and y_prop and z_prop
                else format_vector(geometry.size.x, geometry.size.y, geometry.size.z)
            )
            ET.SubElement(geom_elem, "box", size=size_str)

        elif isinstance(geometry, Cylinder):
            # Check if dimensions should be substituted
            radius_prop = self._get_dimension_property("cylinder_radius", geometry.radius)
            length_prop = self._get_dimension_property("cylinder_length", geometry.length)

            ET.SubElement(
                geom_elem,
                "cylinder",
                radius=f"${{{radius_prop}}}" if radius_prop else format_float(geometry.radius),
                length=f"${{{length_prop}}}" if length_prop else format_float(geometry.length),
            )

        elif isinstance(geometry, Sphere):
            # Check if dimension should be substituted
            radius_prop = self._get_dimension_property("sphere_radius", geometry.radius)

            ET.SubElement(
                geom_elem,
                "sphere",
                radius=f"${{{radius_prop}}}" if radius_prop else format_float(geometry.radius),
            )

        elif isinstance(geometry, Mesh):
            # Mesh handling (same as parent class)
            mesh_path = geometry.filepath
            if self.urdf_path and mesh_path.is_absolute():
                with contextlib.suppress(ValueError):
                    mesh_path = mesh_path.relative_to(self.urdf_path.parent)

            attrib: dict[str, str] = {"filename": str(mesh_path)}
            if geometry.scale.x != 1.0 or geometry.scale.y != 1.0 or geometry.scale.z != 1.0:
                scale_str = format_vector(geometry.scale.x, geometry.scale.y, geometry.scale.z)
                attrib["scale"] = scale_str
            ET.SubElement(geom_elem, "mesh", **attrib)  # type: ignore[arg-type]

    def _get_dimension_property(self, dim_key: str, value: float) -> str | None:
        """Get property name for a dimension if it was extracted.

        Args:
            dim_key: Dimension key (e.g., "cylinder_radius", "box_x")
            value: Dimension value

        Returns:
            Property name if dimension was extracted, None otherwise
        """
        # Round value to match extraction tolerance
        rounded = round(value, 3)
        lookup_key = f"{dim_key}:{rounded}"
        return self.dimension_properties.get(lookup_key)

    def write(self, robot: Robot, filepath: Path, validate: bool = True, **kwargs: Any) -> None:
        """Write XACRO to file."""
        if self.split_files:
            # Shared logic for directory creation
            filepath.parent.mkdir(parents=True, exist_ok=True)
            self._write_split_files(robot, filepath, validate=validate)
        else:
            # Use base class template method (handles generate + mkdir + _save_to_file)
            super().write(robot, filepath, validate=validate, **kwargs)

    def _write_split_files(self, robot: Robot, main_filepath: Path, validate: bool = True) -> None:
        """Write robot to multiple XACRO files (main, properties, macros).

        Args:
            robot: Robot model to export
            main_filepath: Path to the main .xacro file
            validate: Whether to validate robot structure before generation
        """
        # Build the full element tree directly (preserves comments)
        root = self.generate_robot_element(robot, validate=validate)

        # Create separate files
        base_dir = main_filepath.parent
        # Use robot name for consistency (not filename)
        robot_name = robot.name

        # Extract top-level properties (materials, dimensions, etc.)
        properties_root = ET.Element("robot")
        if root.findall(f"{XACRO_NS}property"):
            properties_root.append(ET.Comment(" Properties "))

        # Extract properties
        for prop in list(root.findall(f"{XACRO_NS}property")):
            properties_root.append(prop)
            root.remove(prop)

        # Extract macros (top-level)
        macros_root = ET.Element("robot")
        if root.findall(f"{XACRO_NS}macro"):
            macros_root.append(ET.Comment(" Macros "))

        for macro_elem in list(root.findall(f"{XACRO_NS}macro")):
            macros_root.append(macro_elem)
            root.remove(macro_elem)

        # Create main file with includes
        main_root = ET.Element("robot")
        main_root.set("name", root.get("name", robot.name))

        # Add includes at the top with comments
        if len(properties_root) > 1:  # Comment + at least one property
            main_root.append(ET.Comment(" Properties "))
            include_props = ET.Element(f"{XACRO_NS}include")
            include_props.set("filename", f"{robot_name}_properties.xacro")
            main_root.append(include_props)

        if len(macros_root) > 1:  # Comment + at least one macro
            main_root.append(ET.Comment(" Macros "))
            include_mac = ET.Element(f"{XACRO_NS}include")
            include_mac.set("filename", f"{robot_name}_macros.xacro")
            main_root.append(include_mac)

        # Clean up remaining comments in root that are no longer relevant
        # (e.g. if we moved all properties, remove the "Properties" comment)
        for child in list(root):
            if (
                child.tag is ET.Comment  # type: ignore[comparison-overlap]
                and child.text
                and child.text.strip() in ("Properties", "Macros")
            ):
                root.remove(child)

        # Copy remaining content (Links, Joints, etc.)
        for child in list(root):
            main_root.append(child)

        from .. import __version__

        ns = {"xacro": XACRO_URI}

        # Write files
        if len(properties_root) > 0:
            prop_path = base_dir / f"{robot_name}_properties.xacro"
            prop_path.write_text(
                serialize_xml(properties_root, self.pretty_print, __version__, ns),
                encoding="utf-8",
            )

        if len(macros_root) > 0:
            mac_path = base_dir / f"{robot_name}_macros.xacro"
            mac_path.write_text(
                serialize_xml(macros_root, self.pretty_print, __version__, ns),
                encoding="utf-8",
            )

        main_filepath.write_text(
            serialize_xml(main_root, self.pretty_print, __version__, ns),
            encoding="utf-8",
        )
