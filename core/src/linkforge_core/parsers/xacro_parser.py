"""Native XACRO resolver for LinkForge.

This module provides a pure-Python implementation for resolving XACRO macros,
properties, and includes, removing the need for external dependencies like xacrodoc.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from ..base import RobotParser, RobotParserError
from ..logging_config import get_logger
from ..models.robot import Robot

logger = get_logger(__name__)

XACRO_NS = "http://www.ros.org/wiki/xacro"


class XacroResolver:
    """Lightweight XACRO resolver with macro and math support."""

    def __init__(self, search_paths: list[Path] | None = None, max_depth: int = 50) -> None:
        self.search_paths = search_paths or []
        self.properties: dict[str, str] = {}
        self.macros: dict[str, tuple[list[str], ET.Element]] = {}
        self.args: dict[str, str] = {}
        self.max_depth = max_depth
        self._current_depth = 0

    def resolve_file(self, filepath: Path) -> str:
        """Resolve a XACRO file and return URDF string."""
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()
        except ET.ParseError as e:
            raise RobotParserError(f"Malformed XACRO XML in {filepath}: {e}") from e
        except Exception as e:
            raise RobotParserError(f"Failed to read XACRO file {filepath}: {e}") from e

        # Add parent directory to search paths for relative includes
        if filepath.parent not in self.search_paths:
            self.search_paths.insert(0, filepath.parent)

        try:
            resolved_root = self.resolve_element(root)

            # Clean up: strip xacro attributes and namespace-prefixed tags/attributes
            # This is critical for URDF parsers to accept the output
            return self._finalize_urdf(resolved_root)
        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotParserError(f"XACRO resolution failed for {filepath}: {e}") from e

    def resolve_element(self, element: ET.Element) -> ET.Element:
        """Process a single element recursively."""
        self._current_depth += 1
        if self._current_depth > self.max_depth:
            raise RobotParserError(f"Maximum XACRO recursion depth ({self.max_depth}) exceeded")

        try:
            return self._resolve_element_impl(element)
        finally:
            self._current_depth -= 1

    def _resolve_element_impl(self, element: ET.Element) -> ET.Element:
        """Core recursive resolution logic."""
        tag = element.tag.replace(f"{{{XACRO_NS}}}", "xacro:")

        # 1. Handle properties: <xacro:property name="..." value="..."/>
        if tag == "xacro:property":
            name = element.get("name")
            value = element.get("value") or element.text
            if name:
                self.properties[name] = self._substitute(value or "")
            return ET.Element("skip")

        # 2. Handle arguments: <xacro:arg name="..." default="..."/>
        if tag == "xacro:arg":
            name = element.get("name")
            default = element.get("default")
            if name and name not in self.args:
                self.args[name] = self._substitute(default or "")
            return ET.Element("skip")

        # 3. Handle includes: <xacro:include filename="..."/>
        if tag == "xacro:include":
            filename = self._substitute(element.get("filename") or "")
            included_path = self._find_file(filename)
            if included_path:
                # Store state to handle nested includes correctly
                old_paths = self.search_paths[:]
                if included_path.parent not in self.search_paths:
                    self.search_paths.insert(0, included_path.parent)

                included_tree = ET.parse(included_path)
                included_root = included_tree.getroot()

                # Resolve the root's children and collect them in a container
                container = ET.Element("container")
                for child in included_root:
                    resolved_child = self.resolve_element(child)
                    if resolved_child.tag == "container":
                        for sc in resolved_child:
                            container.append(sc)
                    elif resolved_child.tag != "skip":
                        container.append(resolved_child)

                self.search_paths = old_paths
                return container
            return ET.Element("skip")

        # 4. Handle macro definitions: <xacro:macro name="..." params="...">
        if tag == "xacro:macro":
            name = element.get("name")
            params = [p.strip() for p in (element.get("params") or "").split() if p.strip()]
            if name:
                self.macros[name] = (params, element)
            return ET.Element("skip")

        # 5. Handle conditionals: <xacro:if value="..."/> or <xacro:unless value="..."/>
        if tag in ("xacro:if", "xacro:unless"):
            condition = element.get("value") or "0"
            is_true = self._eval_condition(condition)
            if tag == "xacro:unless":
                is_true = not is_true

            if is_true:
                container = ET.Element("container")
                for child in element:
                    resolved_child = self.resolve_element(child)
                    if resolved_child.tag == "container":
                        for sc in resolved_child:
                            container.append(sc)
                    elif resolved_child.tag != "skip":
                        container.append(resolved_child)
                return container
            return ET.Element("skip")

        # 6. Handle block insertion: <xacro:insert_block name="..."/>
        if tag == "xacro:insert_block":
            name = element.get("name")
            if name and name in self.properties:
                block = self.properties[name]
                if isinstance(block, (ET.Element, list)):
                    # It's an XML block
                    container = ET.Element("container")
                    if isinstance(block, ET.Element):
                        # Single element
                        container.append(self.resolve_element(block))
                    else:
                        # List of elements (usual case for block params)
                        for b_elem in block:
                            container.append(self.resolve_element(b_elem))
                    return container
            return ET.Element("skip")

        # 7. Handle macro calls
        if tag.startswith("xacro:"):
            if tag[6:] in self.macros:
                params, macro_elem = self.macros[tag[6:]]
                local_props = {}

                # Parse parameters
                block_params = [p[1:] for p in params if p.startswith("*")]
                regular_params = [p for p in params if not p.startswith("*")]

                # Map children to block parameters (simplified: all children to all block params)
                for bp in block_params:
                    local_props[bp] = list(element)

                # Map attributes to regular parameters
                for p in regular_params:
                    # Handle default values in params (e.g. "mass:=1.0")
                    bits = p.split(":=")
                    p_name = bits[0]
                    # Substitute the default value from the parameter definition
                    default = self._substitute(bits[1] if len(bits) > 1 else "")
                    # Substitute the attribute value provided in the macro call
                    local_props[p_name] = self._substitute(element.get(p_name) or default)

                # Expand macro body
                parent_props = self.properties.copy()
                self.properties.update(local_props)

                container = ET.Element("container")
                for child in macro_elem:
                    resolved_child = self.resolve_element(child)
                    if resolved_child.tag == "container":
                        for sc in resolved_child:
                            container.append(sc)
                    elif resolved_child.tag != "skip":
                        container.append(resolved_child)

                self.properties = parent_props
                return container

            # Unknown xacro tag - skip it
            return ET.Element("skip")

        # 8. Handle substitutions in text and attributes for regular elements
        new_attrib = {}
        for key, val in element.attrib.items():
            new_attrib[key] = self._substitute(val)

        new_element = ET.Element(element.tag, attrib=new_attrib)
        new_element.text = self._substitute(element.text or "")
        new_element.tail = self._substitute(element.tail or "")

        # 9. Recursively process children
        for child in element:
            resolved_child = self.resolve_element(child)
            if resolved_child.tag == "container":
                for subchild in resolved_child:
                    new_element.append(subchild)
            elif resolved_child.tag != "skip":
                new_element.append(resolved_child)

        return new_element

    def _eval_condition(self, condition: str) -> bool:
        """Evaluate a XACRO condition string."""
        condition = self._substitute(condition).lower().strip()

        # Safe eval for boolean logic
        if condition in ("true", "1"):
            return True
        elif condition in ("false", "0"):
            return False
        else:
            try:
                # Basic string/bool comparison
                return bool(eval(condition, {"__builtins__": {}}, {}))
            except Exception:
                return condition not in ("", "0", "false")

    def _substitute(self, text: str) -> str:
        """Handle ${prop}, $(arg name), and $(find pkg) substitutions with math."""
        if not text:
            return ""

        # 1. Handle arguments: $(arg name)
        text = re.sub(r"\$\(arg (.*?)\)", lambda m: self.args.get(m.group(1), ""), text)

        # 2. Handle ROS package find: $(find package)
        # Note: We convert this to the package:// URI scheme commonly used in URDF.
        text = re.sub(r"\$\(find (.*?)\)", lambda m: f"package://{m.group(1)}", text)

        # 3. Handle properties and math: ${expression}
        def replace_expr(match: re.Match[str]) -> str:
            expr = match.group(1)

            # Replace property names with values
            for name, val in self.properties.items():
                # Ensure we only replace whole words to avoid partial matches
                if name in expr:
                    expr = re.sub(rf"\b{name}\b", str(val), expr)

            # Attempt to evaluate as math. If it fails, or if it looks like a vector
            # (non-math characters or multiple numbers), we return the substituted string as-is.
            try:
                # Safe eval for basic math (+, -, *, /, parentheses)
                # We restrict globals to empty and locals to basic math symbols
                res = eval(expr, {"__builtins__": {}}, {})
                # If the result is a number, return it as string
                if isinstance(res, (int, float)):
                    return str(res)
                return str(res)
            except Exception:
                return expr

        text = re.sub(r"\${(.*?)}", replace_expr, text)

        return text

    def _finalize_urdf(self, root: ET.Element) -> str:
        """Strip XACRO artifacts and format the final XML."""
        # Convert container root to real root if needed
        if root.tag == "container":
            if len(root) > 0:
                root = root[0]
            else:
                return ""

        # Recursive cleanup of namespaces and xacro tags
        def cleanup(elem: ET.Element) -> None:
            # Clean attributes
            for attr in list(elem.attrib.keys()):
                if "xacro" in attr or attr.startswith("{"):
                    del elem.attrib[attr]
            # Recurse
            for child in elem:
                cleanup(child)

        cleanup(root)
        return ET.tostring(root, encoding="unicode")

    def _find_file(self, filename: str) -> Path | None:
        """Find file in search paths."""
        path = Path(filename)
        if path.is_absolute() and path.exists():
            return path

        for search_path in self.search_paths:
            candidate = search_path / filename
            if candidate.exists():
                return candidate
        return None


class XACROParser(RobotParser):
    """Refined XACRO Parser using a class-based interface."""

    def parse(self, filepath: Path, **kwargs: Any) -> Robot:
        """Parse XACRO file into a Robot model.

        Args:
            filepath: Path to the input file
            **kwargs: Additional parsing options

        Returns:
            The generic Robot model (Intermediate Representation)
        """
        from .urdf_parser import URDFParser

        resolver = XacroResolver(search_paths=kwargs.get("search_paths"))

        # Pass additional kwargs as initial xacro arguments
        for k, v in kwargs.items():
            if k not in ["search_paths"]:
                resolver.args[k] = str(v)

        urdf_string = resolver.resolve_file(filepath)

        return URDFParser().parse_string(urdf_string, urdf_directory=filepath.parent, **kwargs)
