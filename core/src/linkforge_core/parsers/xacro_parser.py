"""Native XACRO resolver for LinkForge.

This module provides a pure-Python implementation for resolving XACRO macros,
properties, and includes.
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import yaml  # type: ignore

from ..base import RobotParser, RobotParserError
from ..logging_config import get_logger
from ..models.robot import Robot

logger = get_logger(__name__)

XACRO_URIS = [
    "http://www.ros.org/wiki/xacro",
    "http://wiki.ros.org/xacro",
]


# Safe math context for evaluations
MATH_CONTEXT: dict[str, Any] = {
    name: getattr(math, name) for name in dir(math) if not name.startswith("__")
}
# Security: Allow safe primitives but no dangerous builtins
MATH_CONTEXT["__builtins__"] = {
    "abs": abs,
    "float": float,
    "int": int,
    "len": len,
    "max": max,
    "min": min,
    "round": round,
    "str": str,
    "list": list,
    "dict": dict,
    "bool": bool,
}


class XacroResolver:
    """Lightweight XACRO resolver with macro and math support."""

    def __init__(self, search_paths: list[Path] | None = None, max_depth: int = 50) -> None:
        self.search_paths = search_paths or []
        self.properties: dict[str, Any] = {}
        self.macros: dict[str, tuple[list[str], ET.Element]] = {}
        self.args: dict[str, str] = {}
        self.max_depth = max_depth
        self._current_depth = 0
        self._ns_stack: list[str] = []

        # Per-instance evaluation context to allow file-aware data loading
        self.eval_context = MATH_CONTEXT.copy()
        self.eval_context["load_yaml"] = self._handle_load_yaml
        self.eval_context["load_json"] = self._handle_load_json

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
        # Convert any recognized XACRO namespace URI to 'xacro:' prefix
        tag = element.tag
        for uri in XACRO_URIS:
            prefix = f"{{{uri}}}"
            if tag.startswith(prefix):
                tag = tag.replace(prefix, "xacro:")
                break

        # 1. Handle properties: <xacro:property name="..." value="..."/>
        if tag == "xacro:property":
            name = element.get("name")
            value = element.get("value") or element.text
            if name:
                # Apply namespace prefix if active
                if self._ns_stack:
                    name = f"{'.'.join(self._ns_stack)}.{name}"
                self.properties[name] = self._try_parse_typed_value(self._substitute(value or ""))
            return ET.Element("skip")

        # 2. Handle arguments: <xacro:arg name="..." default="..."/>
        if tag == "xacro:arg":
            name = element.get("name")
            default = element.get("default")
            if name and name not in self.args:
                self.args[name] = self._try_parse_typed_value(self._substitute(default or ""))
            return ET.Element("skip")

        # 3. Handle includes: <xacro:include filename="..." ns="..."/>
        if tag == "xacro:include":
            filename = self._substitute(element.get("filename") or "")
            ns = element.get("ns")
            included_path = self._find_file(filename)
            if included_path:
                # Store state to handle nested includes correctly
                old_paths = self.search_paths[:]
                if included_path.parent not in self.search_paths:
                    self.search_paths.insert(0, included_path.parent)

                if ns:
                    self._ns_stack.append(ns)

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
                if ns:
                    self._ns_stack.pop()
                return container
            return ET.Element("skip")

        # 4. Handle macro definitions: <xacro:macro name="..." params="...">
        if tag == "xacro:macro":
            name = element.get("name")
            params = [p.strip() for p in (element.get("params") or "").split() if p.strip()]
            if name:
                # Apply namespace prefix if active
                if self._ns_stack:
                    name = f"{'.'.join(self._ns_stack)}.{name}"
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
                local_props: dict[str, Any] = {}

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
                    default = self._try_parse_typed_value(
                        self._substitute(bits[1] if len(bits) > 1 else "")
                    )

                    # Substitute the attribute value provided in the macro call
                    raw_val = element.get(p_name)
                    val = self._substitute(raw_val) if raw_val is not None else default

                    local_props[p_name] = self._try_parse_typed_value(val)

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
            # Attributes in XML must be strings
            new_attrib[key] = str(self._substitute(val))

        new_element = ET.Element(element.tag, attrib=new_attrib)
        new_element.text = str(self._substitute(element.text or ""))
        new_element.tail = str(self._substitute(element.tail or ""))

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
                # Basic string/bool comparison with math/data support
                ctx = {**self.eval_context, **self.properties, **self.args}
                return bool(eval(condition, ctx, {}))
            except Exception:
                return condition not in ("", "0", "false")

    def _try_parse_typed_value(self, value: Any) -> Any:
        """Attempt to parse a value into a more specific type (int, float, bool)."""
        if not isinstance(value, str):
            return value

        # Try YAML (most robust and standard compliant)
        try:
            # safe_load handles ints, floats, bools (true/false), nulls
            parsed = yaml.safe_load(value)
            # If it's a primitive type, use it. If it's a collection, we might keep it.
            if isinstance(parsed, (int, float, bool)) or parsed is None:
                return parsed
        except Exception:
            pass

        return value

    def _substitute(self, text: str) -> Any:
        """Handle ${prop}, $(arg name), and $(find pkg) substitutions with math."""
        if not text:
            return ""

        # 1. Handle arguments: $(arg name)
        text = re.sub(r"\$\(arg (.*?)\)", lambda m: str(self.args.get(m.group(1), "")), text)

        # 2. Handle ROS package find: $(find package)
        # Note: We convert this to the package:// URI scheme commonly used in URDF.
        text = re.sub(r"\$\(find (.*?)\)", lambda m: f"package://{m.group(1)}", text)

        # 3. Handle properties and math: ${expression}
        # If the entire string is a single ${...} block, return the object directly.
        # This keeps dicts/lists as real objects for subsequent evaluations.
        stripped = text.strip()
        if stripped.startswith("${") and stripped.count("${") == 1 and stripped.endswith("}"):
            expr = stripped[2:-1]
            return self._evaluate(expr)

        def replace_expr(match: re.Match[str]) -> str:
            expr = match.group(1)
            res = self._evaluate(expr)
            return str(res)

        # If we have mixed text and expressions, we must stringify everything.
        # Use a lambda to ensure we always return strings for re.sub.
        if "${" in text:
            text = re.sub(r"\${(.*?)}", replace_expr, text)

        return text

    def _evaluate(self, expr: str) -> Any:
        """Evaluate a single XACRO expression with hierarchical namespace support."""
        try:
            # Build nested context for hierarchical namespaces (e.g. arm.mass)
            ctx = self.eval_context.copy()
            ctx.update(self.args)

            for name, val in self.properties.items():
                if "." in name:
                    parts = name.split(".")
                    curr = ctx
                    for part in parts[:-1]:
                        if part not in curr or not isinstance(curr[part], SimpleNamespace):
                            curr[part] = SimpleNamespace()
                        curr = curr[part].__dict__
                    curr[parts[-1]] = val
                else:
                    ctx[name] = val

            return eval(expr, ctx, {})
        except Exception as e:
            logger.debug(f"XACRO evaluation failed for '{expr}': {e}")
            return expr

    def _handle_load_yaml(self, filename: str) -> Any:
        """Helper to load YAML file in XACRO context."""
        if yaml is None:
            logger.error("XACRO: PyYAML is not installed. load_yaml() failed.")
            return {}

        path = self._find_file(filename)
        if not path:
            logger.error(f"XACRO: Could not find YAML file {filename}")
            return {}
        try:
            with open(path) as f:
                return yaml.safe_load(f)
        except Exception as e:
            logger.error(f"XACRO: Failed to load YAML {filename}: {e}")
            return {}

    def _handle_load_json(self, filename: str) -> Any:
        """Helper to load JSON file in XACRO context."""
        if json is None:
            # This should theoretically not happen as json is stdlib
            return {}

        path = self._find_file(filename)
        if not path:
            logger.error(f"XACRO: Could not find JSON file {filename}")
            return {}
        try:
            with open(path) as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"XACRO: Failed to load JSON {filename}: {e}")
            return {}

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
                resolver.args[k] = resolver._try_parse_typed_value(str(v))

        urdf_string = resolver.resolve_file(filepath)

        return URDFParser().parse_string(urdf_string, urdf_directory=filepath.parent, **kwargs)
