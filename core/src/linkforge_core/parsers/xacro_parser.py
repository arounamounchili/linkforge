"""Native XACRO resolver for LinkForge.

This module provides a pure-Python implementation for resolving XACRO macros,
properties, and includes.
"""

from __future__ import annotations

import copy
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import yaml  # type: ignore

from ..base import RobotParser, RobotParserError
from ..logging_config import get_logger
from ..models.robot import Robot
from ..utils.path_utils import resolve_package_path

logger = get_logger(__name__)
DEFAULT_MAX_DEPTH = 2000  # Increased for extremely complex industrial robots
RECURSION_LIMIT_BOOST = 20000  # Python stack limit for deep recursions

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

    def __init__(
        self,
        search_paths: list[Path] | None = None,
        max_depth: int = 50,
        start_dir: Path | None = None,
    ) -> None:
        self.search_paths = search_paths or []
        self.start_dir = start_dir
        self.properties: dict[str, Any] = {}
        self.macros: dict[str, tuple[list[str], ET.Element]] = {}
        self.args: dict[str, str] = {}
        self.max_depth = max_depth or DEFAULT_MAX_DEPTH
        self._current_depth = 0
        self._ns_stack: list[str] = []
        self._file_stack: list[Path] = []
        self._block_stack: set[str] = (
            set()
        )  # Track active block insertions to prevent infinite recursion

        # Per-instance evaluation context to allow file-aware data loading
        self.eval_context = MATH_CONTEXT.copy()
        # Add 'load_yaml' and 'load_json' for top-level use
        self.eval_context["load_yaml"] = self._handle_load_yaml
        self.eval_context["load_json"] = self._handle_load_json

        # Add 'xacro' namespace for ROS-standard compliance (e.g. xacro.load_yaml)
        xacro_ns = SimpleNamespace()
        xacro_ns.load_yaml = self._handle_load_yaml
        xacro_ns.load_json = self._handle_load_json
        self.eval_context["xacro"] = xacro_ns

    def resolve_file(self, filepath: Path) -> str:
        """Resolve a XACRO file and return URDF string."""
        # Deeply nested XACRO files can exceed Python's default recursion limit (usually 1000)
        # We temporarily boost it to ensure we can handle complex industrial robots
        # (e.g. mobile_fr3_duo has very deep macro expansions)
        old_limit = sys.getrecursionlimit()
        if old_limit < RECURSION_LIMIT_BOOST:
            sys.setrecursionlimit(RECURSION_LIMIT_BOOST)

        filepath = filepath.resolve()

        # Set start_dir from file if not already set
        if self.start_dir is None:
            self.start_dir = filepath.parent

        try:
            container, root_tag, root_attrib = self._process_include_file(filepath)

            # Create a new robot root from the processed elements
            # Standard URDF requires a <robot> root tag

            # Create URDF root preserving original name and attributes
            out_root = ET.Element(root_tag, root_attrib)

            # Helper to flatten container and filter items
            def _append_filtered(parent: ET.Element, items: list[ET.Element] | ET.Element) -> None:
                if isinstance(items, ET.Element):
                    items = [items]  # pragma: no cover

                for item in items:
                    if item.tag == "container":
                        # Flatten container
                        _append_filtered(parent, list(item))  # pragma: no cover
                    elif item.tag != "skip" and not item.tag.startswith("xacro:"):
                        # Add valid element
                        parent.append(item)

            _append_filtered(out_root, list(container))

            return self._finalize_urdf(out_root)
        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotParserError(f"XACRO resolution failed for {filepath}: {e}") from e
        finally:
            if sys.getrecursionlimit() != old_limit:
                sys.setrecursionlimit(old_limit)

    def _process_include_file(
        self, filepath: Path, ns: str | None = None
    ) -> tuple[ET.Element, str, dict[str, str]]:
        """Process a file path, handling includes and circular detection."""
        filepath = filepath.resolve()

        if filepath in self._file_stack:
            chain = " -> ".join([f.name for f in self._file_stack] + [filepath.name])
            raise RobotParserError(f"Circular XACRO include detected: {chain}")

        self._file_stack.append(filepath)

        try:
            tree = ET.parse(filepath)
            root = tree.getroot()

            if ns:
                self._ns_stack.append(ns)

            # Store search path for relative includes within this file
            old_paths = self.search_paths[:]
            if filepath.parent not in self.search_paths:
                self.search_paths.insert(0, filepath.parent)

            container = ET.Element("container")
            for child in root:
                resolved_child = self.resolve_element(child)
                if resolved_child.tag == "container":
                    for sc in resolved_child:
                        container.append(sc)
                elif resolved_child.tag != "skip":
                    container.append(resolved_child)

            self.search_paths = old_paths
            if ns:
                self._ns_stack.pop()

            # Clean root attributes (remove xacro namespace)
            clean_attrib = {
                k: v
                for k, v in root.attrib.items()
                if not k.startswith("{http://www.ros.org/wiki/xacro}")
            }

            return container, root.tag, clean_attrib

        except ET.ParseError as e:
            raise RobotParserError(f"Malformed XACRO XML in {filepath}: {e}") from e
        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotParserError(f"Failed to process XACRO file {filepath}: {e}") from e
        finally:
            self._file_stack.pop()

    def resolve_element(self, element: ET.Element) -> ET.Element:
        """Process a single element recursively."""
        self._current_depth += 1
        if self._current_depth > self.max_depth:
            stack_info = ""
            if self._file_stack:
                stack_info = f" (File stack: {' -> '.join([f.name for f in self._file_stack])})"

            elem_info = f"Element: <{element.tag}"
            if "name" in element.attrib:
                elem_info += f" name='{element.get('name')}'"
            elem_info += ">"

            raise RobotParserError(
                f"Maximum XACRO recursion depth ({self.max_depth}) exceeded at {elem_info}{stack_info}. This usually indicates a circular include or infinite macro loop."
            )

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
            value = element.get("value")
            if name:
                # Apply namespace prefix if active
                if self._ns_stack:
                    name = f"{'.'.join(self._ns_stack)}.{name}"

                # If no 'value' attribute, check for children (block property)
                if value is None and len(element) > 0:
                    self.properties[name] = list(element)
                else:
                    self.properties[name] = self._try_parse_typed_value(
                        self._substitute(value or element.text or "")
                    )
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
            filename = str(self._substitute(element.get("filename") or ""))
            ns = element.get("ns")
            included_path = self._find_file(filename)
            if not included_path:
                logger.warning(
                    f"XACRO: Could not find included file: '{filename}'. Check your paths and $(find ...) usage."
                )
                return ET.Element("skip")

            container, _, _ = self._process_include_file(included_path, ns=ns)
            return container

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
            name = str(self._substitute(element.get("name") or ""))
            if name and name in self.properties:
                # Cycle detection for blocks
                if name in self._block_stack:
                    raise RobotParserError(
                        f"Circular block insertion detected: {name}. This causes infinite recursion."
                    )

                self._block_stack.add(name)
                try:
                    block = self.properties[name]
                    if isinstance(block, (ET.Element, list)):
                        # It's an XML block
                        # CRITICAL: We must deepcopy the block before insertion!
                        # Otherwise, if a block refers to itself (directly or via macro),
                        # we create a cycle in the Python object graph that causes infinite recursion.
                        container = ET.Element("container")

                        def _process_block_item(item: ET.Element) -> None:
                            res = self.resolve_element(copy.deepcopy(item))
                            if res.tag == "container":
                                for child in res:
                                    container.append(child)
                            elif res.tag != "skip":
                                container.append(res)

                        if isinstance(block, ET.Element):
                            # Single element
                            _process_block_item(block)
                        else:
                            # List of elements (usual case for block params)
                            for b_elem in block:
                                _process_block_item(b_elem)

                        return container
                finally:
                    self._block_stack.remove(name)
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
                # CRITICAL: We must resolve the block content in the CALLER'S scope before passing it
                # to the macro. Otherwise, if we pass a "lazy" insert_block instruction,
                # and the macro uses the same parameter name, we create an infinite loop.
                resolved_block_content: list[ET.Element] = []
                for child in element:
                    # Resolve child in current scope
                    # We use deepcopy to ensure we don't modify the original definition
                    res = self.resolve_element(copy.deepcopy(child))
                    if res.tag == "container":
                        resolved_block_content.extend(res)
                    elif res.tag != "skip":
                        resolved_block_content.append(res)

                for bp in block_params:
                    local_props[bp] = resolved_block_content

                # Map attributes to regular parameters
                for p in regular_params:
                    # Handle default values in params (e.g. "mass:=1.0")
                    bits = p.split(":=")
                    p_name = bits[0]
                    # Substitute the default value from the parameter definition
                    default = self._try_parse_typed_value(
                        self._substitute(bits[1] if len(bits) > 1 else "")
                    )

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

            # Unknown xacro tag - report warning and skip it
            macro_name = tag[6:]
            logger.warning(
                f"XACRO: Unknown macro or tag: '{macro_name}'. Did you forget to include the corresponding file?"
            )
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
        res = self._substitute(condition)
        if isinstance(res, bool):
            return res

        condition_str = str(res).lower().strip()

        # Safe eval for boolean logic
        if condition_str in ("true", "1"):
            return True
        elif condition_str in ("false", "0"):
            return False
        else:
            try:
                # Basic string/bool comparison with math/data support
                ctx = {**self.eval_context, **self.properties, **self.args}
                return bool(eval(condition_str, ctx, {}))
            except Exception:
                return condition_str not in ("", "0", "false")

    def _try_parse_typed_value(self, value: Any) -> Any:
        """Attempt to parse a value into a more specific type (int, float, bool, list, dict)."""
        if not isinstance(value, str):
            return value

        # Try YAML (most robust and standard compliant)
        try:
            # safe_load handles ints, floats, bools (true/false), nulls, lists, dicts
            parsed = yaml.safe_load(value)
            # If it's a primitive type or collection, use it.
            if isinstance(parsed, (int, float, bool, list, dict)) or parsed is None:
                return parsed
        except Exception:
            pass

        # Fallback manual parsing (in case yaml fails or returns string for number)
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
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
            # CRITICAL: Do not silent-fail! If math fails (e.g. missing variable),
            # we must tell the user immediately rather than producing a corrupt URDF.
            raise RobotParserError(f"Failed to evaluate expression '${{{expr}}}': {e}") from e

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
        except Exception as e:  # pragma: no cover
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
        except Exception as e:  # pragma: no cover
            logger.error(f"XACRO: Failed to load JSON {filename}: {e}")
            return {}

    def _finalize_urdf(self, root: ET.Element) -> str:
        """Strip XACRO artifacts and format the final XML."""

        # Recursive cleanup of namespaces and xacro tags
        def cleanup(elem: ET.Element) -> None:
            # Clean attributes
            for attr in list(elem.attrib.keys()):
                # Standard URDF parsers often fail if they see xacro prefixes or URI namespaces
                if "xacro" in attr or attr.startswith("{"):
                    del elem.attrib[attr]
            # Recurse and filter children
            # We must iterate over a copy of the list to allow modification during iteration
            for child in list(elem):
                tag = child.tag

                # Handle non-string tags (e.g. Comments) safely
                if not isinstance(tag, str):
                    cleanup(child)
                    continue

                # Aggressive filtering: Remove skip and any tag containing "xacro"
                # This matches URDFParser's strict detection logic
                is_xacro = (
                    tag == "skip" or "xacro:" in tag or (tag.startswith("{") and "xacro" in tag)
                )

                if is_xacro:
                    elem.remove(child)
                else:
                    cleanup(child)

        cleanup(root)
        from ..utils.xml_utils import serialize_xml

        return serialize_xml(root)

    def _find_file(self, filename: str) -> Path | None:
        """Find file in search paths, supporting both relative paths and package:// URIs."""
        # 1. Handle package:// URIs
        if filename.startswith("package:"):
            return resolve_package_path(filename, self.start_dir or Path.cwd())

        # 2. Handle absolute paths
        path = Path(filename)
        if path.is_absolute() and path.exists():
            return path

        # 3. Handle relative paths in search_paths
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
                search_paths: List of additional paths to search for includes
                start_dir: Base directory for package:// resolution (defaults to filepath.parent)

        Returns:
            The generic Robot model (Intermediate Representation)
        """
        from .urdf_parser import URDFParser

        resolver = XacroResolver(
            search_paths=kwargs.get("search_paths"),
            start_dir=kwargs.get("start_dir", filepath.parent),
        )

        # Pass additional kwargs as initial xacro arguments
        for k, v in kwargs.items():
            if k not in ["search_paths"]:
                resolver.args[k] = resolver._try_parse_typed_value(str(v))

        urdf_string = resolver.resolve_file(filepath)

        return URDFParser().parse_string(
            urdf_string, urdf_directory=filepath.parent, default_name=filepath.stem, **kwargs
        )
