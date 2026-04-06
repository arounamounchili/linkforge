"""Native XACRO resolver for LinkForge.

This module provides a pure-Python implementation for resolving XACRO macros,
properties, and includes.
"""

from __future__ import annotations

import copy
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

try:
    import yaml  # type: ignore[import-untyped]
except ImportError:
    yaml = None

from ..base import RobotParser
from ..exceptions import (
    RobotParserError,
    RobotXacroError,
    RobotXacroExpressionError,
    RobotXacroRecursionError,
)
from ..logging_config import get_logger
from ..models.robot import Robot
from ..utils.dict_utils import AttrDict
from ..utils.path_utils import resolve_package_path

logger = get_logger(__name__)
DEFAULT_MAX_DEPTH = 2000  # Increased for extremely complex industrial robots
RECURSION_LIMIT_BOOST = 20000  # Python stack limit for deep recursions

XACRO_URIS = [
    "http://www.ros.org/wiki/xacro",
    "http://wiki.ros.org/xacro",
]

_DUNDER_PATTERN: re.Pattern[str] = re.compile(r"__\w+__")

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
# Standard XACRO booleans
MATH_CONTEXT["true"] = True
MATH_CONTEXT["false"] = False


@dataclass
class XacroTemplate:
    """A pre-parsed structural template of a XACRO file.

    This represents the 'Structural Phase' of XACRO resolution where all
    includes are expanded and files are parsed into memory, but no
    substitutions or macro calls have been evaluated yet.
    """

    filepath: Path
    root_tag: str
    root_attrib: dict[str, str]
    container: ET.Element  # Container of pre-resolved elements
    macros: dict[str, tuple[list[str], ET.Element]]


# Global cache for structural templates to speed up repeated assembly of identical robots.
TEMPLATE_CACHE: dict[Path, XacroTemplate] = {}


class XacroResolver:
    """Lightweight XACRO resolver with macro and math support."""

    def __init__(
        self,
        search_paths: list[Path] | None = None,
        max_depth: int = 50,
        start_dir: Path | None = None,
    ) -> None:
        """Initialize the XACRO resolver.

        Args:
            search_paths: List of additional directories to search for includes.
            max_depth: Maximum recursion depth for macro expansions and includes.
            start_dir: Base directory for resolving package:// URIs and relative paths.
        """
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

        # Shared evaluation context (load functions and standard arg)
        self.eval_context = MATH_CONTEXT.copy()
        self.eval_context["load_yaml"] = self._handle_load_yaml
        self.eval_context["load_json"] = self._handle_load_json
        self.eval_context["arg"] = self._handle_arg_eval

        # ROS-standard 'xacro' namespace
        xacro_ns = SimpleNamespace()
        xacro_ns.load_yaml = self._handle_load_yaml
        xacro_ns.load_json = self._handle_load_json
        xacro_ns.arg = self._handle_arg_eval
        xacro_ns.warning = logger.warning
        xacro_ns.error = logger.error
        xacro_ns.fatal = logger.critical
        xacro_ns.message = logger.info
        self.eval_context["xacro"] = xacro_ns

    def resolve_file(self, filepath: Path) -> str:
        """Resolve a XACRO file and return the final URDF string.

        Args:
            filepath: Path to the XACRO file to resolve.

        Returns:
            The fully resolved URDF XML as a string.

        Raises:
            RobotXacroError: If resolution fails or internal error occurs
            RobotXacroRecursionError: If circular dependencies are found
        """
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
            template = self._get_structural_template(filepath)

            # Standard URDF requires a <robot> root tag
            # Create URDF root preserving original name and attributes from the template
            out_root = ET.Element(template.root_tag, template.root_attrib)

            # Copy template macros into our active resolver
            self.macros.update(copy.deepcopy(template.macros))

            # Evaluation Phase: Process the structural container
            # We deepcopy the container to ensure this evaluation doesn't mutate the cache.
            resolved_container = self.resolve_element(copy.deepcopy(template.container))

            # Helper to flatten container and filter items
            def _append_filtered(parent: ET.Element, items: list[ET.Element] | ET.Element) -> None:
                if isinstance(items, ET.Element):
                    items = [items]

                for item in items:
                    if item.tag == "container":
                        # Flatten container
                        _append_filtered(parent, list(item))
                    elif item.tag != "skip" and not item.tag.startswith("xacro:"):
                        # Add valid element
                        parent.append(item)

            _append_filtered(out_root, list(resolved_container))

            return self._finalize_urdf(out_root)
        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotXacroError(str(e), context=str(filepath)) from e
        finally:
            if sys.getrecursionlimit() != old_limit:
                sys.setrecursionlimit(old_limit)

    def resolve_string(self, xml_string: str) -> str:
        """Resolve a XACRO string and return the final XML string.

        Args:
            xml_string: The XACRO XML content as a string.

        Returns:
            The fully resolved XML as a string.

        Raises:
            RobotXacroError: If XML is malformed or resolution fails
        """
        old_limit = sys.getrecursionlimit()
        if old_limit < RECURSION_LIMIT_BOOST:
            sys.setrecursionlimit(RECURSION_LIMIT_BOOST)

        try:
            root = ET.fromstring(xml_string)
            resolved_root = self.resolve_element(root)

            # finalize_urdf expects an ET.Element and handles cleanup of xacro artifacts
            return self._finalize_urdf(resolved_root)
        except Exception as e:
            if isinstance(e, RobotParserError):
                raise
            raise RobotXacroError(str(e), context="string resolution") from e
        finally:
            if sys.getrecursionlimit() != old_limit:
                sys.setrecursionlimit(old_limit)

    def _get_structural_template(self, filepath: Path) -> XacroTemplate:
        """Retrieve or build a structural template for a file.

        Args:
            filepath: Path to the XACRO file.

        Returns:
            The cached or newly built structural template.

        Raises:
            RobotXacroRecursionError: If circular includes are detected
            RobotXacroError: If XML parsing fails or files cannot be located
        """
        filepath = filepath.resolve()
        if filepath in TEMPLATE_CACHE:
            return TEMPLATE_CACHE[filepath]

        logger.debug(f"XACRO: Building structural template for {filepath.name}")

        # Cycle detection for includes during template build
        if filepath in self._file_stack:
            raise RobotXacroRecursionError(str(filepath.name))

        self._file_stack.append(filepath)
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()

            # Store search path for relative includes within this file
            old_paths = self.search_paths[:]
            if filepath.parent not in self.search_paths:
                self.search_paths.insert(0, filepath.parent)

            structural_macros: dict[str, tuple[list[str], ET.Element]] = {}
            container = ET.Element("container")

            for child in root:
                # Basic tagging of XACRO elements
                tag = child.tag
                for uri in XACRO_URIS:
                    prefix = f"{{{uri}}}"
                    if tag.startswith(prefix):
                        tag = tag.replace(prefix, "xacro:")
                        break

                if tag == "xacro:include":
                    # Handle structural inclusion
                    # filename may depend on $(arg) or ${}, so we substitute using current context
                    filename = str(self._substitute(child.get("filename") or ""))
                    ns = child.get("ns")
                    inc_path = self._find_file(filename)

                    if inc_path:
                        inc_template = self._get_structural_template(inc_path)
                        structural_macros.update(inc_template.macros)

                        # If namespaced, we wrap the included elements but keep them structural
                        if ns:
                            ns_container = ET.Element("container", ns=ns)
                            for sc in inc_template.container:
                                ns_container.append(sc)
                            container.append(ns_container)
                        else:
                            for sc in inc_template.container:
                                container.append(sc)
                    else:
                        logger.warning(f"XACRO: Could not find included file: '{filename}'")
                elif tag == "xacro:macro":
                    # Collect macro definition but do not expand
                    name = child.get("name")
                    params_str = child.get("params") or ""
                    # Quick split (the resolver will do the full smart split during evaluation)
                    params = [p.strip() for p in params_str.split(",") if p.strip()]
                    if name:
                        structural_macros[name] = (params, child)
                    container.append(child)
                else:
                    # Keep all other elements (properties, conditionals, URDF tags) as is
                    container.append(child)

            self.search_paths = old_paths

            # Clean root attributes (remove xacro namespace)
            clean_attrib = {
                k: v
                for k, v in root.attrib.items()
                if not k.startswith("{http://www.ros.org/wiki/xacro}")
            }

            template = XacroTemplate(
                filepath=filepath,
                root_tag=root.tag,
                root_attrib=clean_attrib,
                container=container,
                macros=structural_macros,
            )

            TEMPLATE_CACHE[filepath] = template
            return template

        except ET.ParseError as e:
            raise RobotXacroError(str(e), context=str(filepath)) from e
        finally:
            self._file_stack.pop()

    def resolve_element(self, element: ET.Element) -> ET.Element:
        """Process a single element recursively, tracking depth.

        Args:
            element: The XML element to resolve.

        Returns:
            The resolved XML element or a container.

        Raises:
            RobotXacroRecursionError: If maximum recursion depth is exceeded
        """
        self._current_depth += 1
        if self._current_depth > self.max_depth:
            raise RobotXacroRecursionError(self.max_depth)

        try:
            return self._resolve_element_impl(element)
        finally:
            self._current_depth -= 1

    def _resolve_element_impl(self, element: ET.Element) -> ET.Element:
        """Core recursive resolution logic for a single XML element.

        Args:
            element: The element to dispatch for processing.

        Returns:
            The resolved XML element.
        """
        # Convert any recognized XACRO namespace URI to 'xacro:' prefix
        tag = element.tag
        for uri in XACRO_URIS:
            prefix = f"{{{uri}}}"
            if tag.startswith(prefix):
                tag = tag.replace(prefix, "xacro:")
                break

        # Dispatch dictionary for known xacro tags
        dispatch = {
            "xacro:property": self._handle_property,
            "xacro:arg": self._handle_arg,
            "xacro:include": self._handle_include,
            "xacro:macro": self._handle_macro_def,
            "xacro:if": self._handle_conditional,
            "xacro:unless": self._handle_conditional,
            "xacro:insert_block": self._handle_insert_block,
        }

        if tag in dispatch:
            return dispatch[tag](element)

        if tag == "container":
            ns = element.get("ns")
            if ns:
                self._ns_stack.append(ns)

            new_container = ET.Element("container")
            for child in element:
                resolved_child = self.resolve_element(child)
                if resolved_child.tag == "container":
                    for sc in resolved_child:
                        new_container.append(sc)
                elif resolved_child.tag != "skip":
                    new_container.append(resolved_child)

            if ns:
                self._ns_stack.pop()
            return new_container

        if tag.startswith("xacro:"):
            return self._handle_macro_call(tag, element)

        return self._handle_regular_element(element)

    def _handle_property(self, element: ET.Element) -> ET.Element:
        """Handle property definitions: <xacro:property name="..." value="..."/>.

        Args:
            element: The XACRO property XML element.

        Returns:
            A 'skip' element as properties are consumed during resolution.
        """
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

    def _handle_arg(self, element: ET.Element) -> ET.Element:
        """Handle argument definitions: <xacro:arg name="..." default="..."/>.

        Args:
            element: The XACRO arg XML element.

        Returns:
            A 'skip' element as args are consumed during resolution.
        """
        name = element.get("name")
        default = element.get("default")
        if name and name not in self.args:
            self.args[name] = self._try_parse_typed_value(self._substitute(default or ""))
        return ET.Element("skip")

    def _handle_include(self, element: ET.Element) -> ET.Element:
        """Handle file includes: <xacro:include filename="..." ns="..."/>.

        Args:
            element: The XACRO include XML element.

        Returns:
            A container element containing the resolved included content.
        """
        filename = str(self._substitute(element.get("filename") or ""))
        ns = element.get("ns")
        included_path = self._find_file(filename)

        if not included_path:
            logger.warning(
                f"XACRO: Could not find included file: '{filename}'. Check your paths and $(find ...) usage."
            )
            return ET.Element("skip")

        template = self._get_structural_template(included_path)

        # Merge template macros into current context
        # Skip namespaced macros here; they will be handled by the container evaluation phase
        if not ns:
            self.macros.update(copy.deepcopy(template.macros))

        if ns:
            self._ns_stack.append(ns)

        # Evaluate the template container in current context
        # We use resolve_element on a deepcopy to prevent per-instance pollution of the cache.
        container = self.resolve_element(copy.deepcopy(template.container))

        if ns:
            self._ns_stack.pop()

        return container

    def _handle_macro_def(self, element: ET.Element) -> ET.Element:
        """Handle macro definitions: <xacro:macro name="..." params="...">.

        Args:
            element: The XACRO macro XML element.

        Returns:
            A 'skip' element as macro definitions are cached for later calls.
        """
        name = element.get("name")
        params_str = element.get("params") or ""
        params = []
        if params_str:
            # Smart split that respects nesting in substitutions
            current: list[str] = []
            nesting = 0
            for char in params_str:
                if char in "({[":
                    nesting += 1
                elif char in ")}]":
                    nesting -= 1

                if char.isspace() and nesting == 0:
                    if current:
                        params.append("".join(current))
                        current = []
                else:
                    current.append(char)
            if current:
                params.append("".join(current))
        if name:
            # Apply namespace prefix if active
            if self._ns_stack:
                name = f"{'.'.join(self._ns_stack)}.{name}"
            self.macros[name] = (params, element)
        return ET.Element("skip")

    def _handle_conditional(self, element: ET.Element) -> ET.Element:
        """Handle conditionals: <xacro:if value="..."/> or <xacro:unless value="..."/>.

        Args:
            element: The XACRO conditional XML element.

        Returns:
            A container with children if the condition matches, otherwise a 'skip' element.
        """
        tag = element.tag
        # Check against normal and URI namespaces for the conditional tag
        is_unless = "xacro:unless" in tag or "unless" in tag

        condition = element.get("value") or "0"
        is_true = self._eval_condition(condition)
        if is_unless:
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

    def _handle_insert_block(self, element: ET.Element) -> ET.Element:
        """Handle block insertion: <xacro:insert_block name="..."/>.

        Args:
            element: The XACRO insert_block XML element.

        Returns:
            The resolved XML block if found, otherwise a 'skip' element.
        """
        name = str(self._substitute(element.get("name") or ""))
        if name and name in self.properties:
            # Cycle detection for blocks
            if name in self._block_stack:
                raise RobotXacroRecursionError(name)

            self._block_stack.add(name)
            try:
                block = self.properties[name]
                if isinstance(block, (ET.Element, list)):
                    # It's an XML block
                    # CRITICAL: We must deepcopy the block before insertion!
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
                        # List of elements
                        for b_elem in block:
                            _process_block_item(b_elem)

                    return container
            finally:
                self._block_stack.remove(name)
        return ET.Element("skip")

    def _handle_macro_call(self, tag: str, element: ET.Element) -> ET.Element:
        """Handle macro calls: <xacro:my_macro_name ...>.

        Args:
            tag: The original XML tag name.
            element: The XACRO macro call XML element.

        Returns:
            A container containing the fully expanded macro body.

        Raises:
            RobotParserError: If parent scope inheritance fails.
        """
        macro_name = tag[6:]
        if macro_name in self.macros:
            params, macro_elem = self.macros[macro_name]
            local_props: dict[str, Any] = {}

            # Parse parameters
            block_params = [p[1:] for p in params if p.startswith("*")]
            regular_params = [p for p in params if not p.startswith("*")]

            # Map children to block parameters
            resolved_block_content: list[ET.Element] = []
            for child in element:
                res = self.resolve_element(copy.deepcopy(child))
                if res.tag == "container":
                    resolved_block_content.extend(res)
                elif res.tag != "skip":
                    resolved_block_content.append(res)

            for bp in block_params:
                local_props[bp] = resolved_block_content

            # Map attributes to regular parameters
            for p in regular_params:
                bits = p.split(":=")
                p_name = bits[0]
                default_str = bits[1] if len(bits) > 1 else None

                # Handle ^ parent-scope inheritance
                if default_str is not None and default_str.startswith("^"):
                    fallback_str = default_str[2:] if default_str.startswith("^|") else None
                    if p_name in self.properties:
                        default = self.properties[p_name]
                    elif fallback_str is not None:
                        # Fallback value should be substituted too
                        default = self._try_parse_typed_value(self._substitute(fallback_str))
                    else:
                        raise RobotXacroExpressionError(
                            p_name, "Outer-scope property not found for '^' inheritance"
                        )
                elif default_str is not None:
                    default = self._try_parse_typed_value(self._substitute(default_str))
                else:
                    default = ""

                raw_val = element.get(p_name)
                val = self._substitute(raw_val) if raw_val is not None else default

                local_props[p_name] = self._try_parse_typed_value(val)

            # Expand macro body
            parent_props = copy.deepcopy(self.properties)
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
        logger.warning(
            f"XACRO: Unknown macro or tag: '{macro_name}'. Did you forget to include the corresponding file?"
        )
        return ET.Element("skip")

    def _handle_regular_element(self, element: ET.Element) -> ET.Element:
        """Handle substitutions in text and attributes for regular elements.

        Args:
            element: The XML element to process.

        Returns:
            A new XML element with all xacro expressions resolved.
        """
        new_attrib = {}
        for key, val in element.attrib.items():
            # Attributes in XML must be strings
            new_attrib[key] = str(self._substitute(val))

        new_element = ET.Element(element.tag, attrib=new_attrib)
        new_element.text = str(self._substitute(element.text or ""))
        new_element.tail = str(self._substitute(element.tail or ""))

        # Recursively process children
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
                # Standard expression evaluation
                if _DUNDER_PATTERN.search(condition_str):
                    raise RobotXacroExpressionError(condition_str, "Forbidden dunder attributes")
                ctx = {**self.eval_context, **self.properties, **self.args}
                return bool(eval(condition_str, ctx, {}))
            except Exception as e:
                if isinstance(e, RobotParserError):
                    raise
                return condition_str not in ("", "0", "false")

    def _try_parse_typed_value(self, value: Any) -> Any:
        """Attempt to parse a value into a more specific type (int, float, bool, list, dict)."""
        if not isinstance(value, str):
            return value

        # yaml.safe_load("") returns None, so empty strings must be returned as-is
        # before YAML parsing to avoid corrupting args with empty defaults.
        if value == "":
            return value

        # Try YAML (most robust and standard compliant)
        if yaml is not None:
            try:
                # safe_load handles ints, floats, bools (true/false), nulls, lists, dicts
                parsed = yaml.safe_load(value)
                # If it's a primitive type or collection, use it.
                if isinstance(parsed, (int, float, bool, list, dict)) or parsed is None:
                    return AttrDict._wrap(parsed)
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
        """Handle ${prop}, $(arg name), and $(find pkg) substitutions with math.

        Args:
            text: The input string containing xacro expressions.

        Returns:
            The resolved value, which may be a primitive type if the input was
            a single ${} block.

        Raises:
            RobotParserError: If an undefined argument or environment variable is required.
        """
        if not text:
            return ""

        # Handle $$ escaping (e.g. $${expr} -> literal ${expr})
        sentinel = "\x00LFDOLLAR\x00"
        text = text.replace("$$", sentinel)

        # 1. Handle arguments: $(arg name)
        def _resolve_arg(m: re.Match[str]) -> str:
            name = m.group(1).strip()
            if name not in self.args:
                raise RobotXacroExpressionError(name, "Undefined substitution argument")
            return str(self.args[name])

        text = re.sub(r"\$\(arg (.*?)\)", _resolve_arg, text)

        # 2. Handle evaluation: $(eval expression) - Standard ROS XACRO feature
        # We treat it as an alias for ${...} since our evaluation engine is Python-based.
        text = re.sub(r"\$\(eval (.*?)\)", r"${\1}", text)

        # 3. Handle environment variable substitution, matching roslaunch behaviour.
        # $(env VAR) raises if unset; $(optenv VAR) returns ""; $(optenv VAR default) returns default.
        def _resolve_env(m: re.Match[str]) -> str:
            parts = m.group(1).split(None, 1)
            var = parts[0]
            value = os.environ.get(var)
            if value is None:
                raise RobotXacroExpressionError(var, "Required environment variable not set")
            return value

        def _resolve_optenv(m: re.Match[str]) -> str:
            parts = m.group(1).split(None, 1)
            var = parts[0]
            default = parts[1] if len(parts) > 1 else ""
            return os.environ.get(var, default)

        text = re.sub(r"\$\(env (.*?)\)", _resolve_env, text)
        text = re.sub(r"\$\(optenv (.*?)\)", _resolve_optenv, text)

        # Strip file:// prefix from ROS package finds to prevent double-prefix.
        text = re.sub(r"file://\$\(find (.*?)\)", lambda m: f"package://{m.group(1)}", text)
        text = re.sub(r"\$\(find (.*?)\)", lambda m: f"package://{m.group(1)}", text)

        # 4. Handle properties and math: ${expression}
        # If the entire string is a single ${...} block, return the object directly.
        # This keeps dicts/lists as real objects for subsequent evaluations.
        stripped = text.strip()
        is_pure_expression = (
            stripped.startswith("${")
            and stripped.endswith("}")
            and stripped.count("${") == 1
            and sentinel not in stripped
        )

        if is_pure_expression:
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

        # Restore escaped dollar signs
        text = text.replace(sentinel, "$")

        return text

    def _evaluate(self, expr: str) -> Any:
        """Evaluate a single XACRO expression with hierarchical namespace support.

        Args:
            expr: The python-like expression to evaluate.

        Returns:
            The result of the evaluation.

        Raises:
            RobotParserError: If the expression contains malicious dunder attributes.
        """
        if _DUNDER_PATTERN.search(expr):
            raise RobotXacroExpressionError(expr, "Forbidden dunder attributes")

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
            raise RobotXacroExpressionError(expr, str(e)) from e

    def _handle_arg_eval(self, name: str) -> Any:
        """Access XACRO arguments in evaluation context.

        Args:
            name: Name of the argument to retrieve.

        Returns:
            The argument value if found, otherwise an empty string.
        """
        return self.args.get(name, "")

    def _handle_load_yaml(self, filename: str) -> Any:
        """Helper to load YAML file in XACRO context.

        Args:
            filename: Path to the YAML file.

        Returns:
            The parsed YAML data (dict or list).
        """
        if yaml is None:
            logger.error("XACRO: PyYAML is not installed. load_yaml() failed.")
            return {}

        path = self._find_file(filename)
        if not path:
            logger.error(f"XACRO: Could not find YAML file {filename}")
            return {}
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
                return AttrDict._wrap(data)
        except Exception as e:  # pragma: no cover
            logger.error(f"XACRO: Failed to load YAML {filename}: {e}")
            return AttrDict()

    def _handle_load_json(self, filename: str) -> Any:
        """Helper to load JSON file in XACRO context.

        Args:
            filename: Path to the JSON file.

        Returns:
            The parsed JSON data (dict or list).
        """
        if json is None:  # pragma: no cover
            return {}

        path = self._find_file(filename)
        if not path:
            logger.error(f"XACRO: Could not find JSON file {filename}")
            return {}
        try:
            with open(path) as f:
                data = json.load(f)
                return AttrDict._wrap(data)
        except Exception as e:  # pragma: no cover
            logger.error(f"XACRO: Failed to load JSON {filename}: {e}")
            return AttrDict()

    def _finalize_urdf(self, root: ET.Element) -> str:
        """Strip XACRO artifacts and format the final XML.

        Args:
            root: The root element of the resolved URDF.

        Returns:
            The serialized URDF string.
        """

        # Recursively strip XML namespaces and filter out XACRO-specific elements
        def cleanup(elem: ET.Element) -> None:
            # Strip XACRO attributes and any with URI namespaces (e.g., {uri}name)
            for attr in list(elem.attrib.keys()):
                if "xacro" in attr or attr.startswith("{"):
                    del elem.attrib[attr]

            # Flatten tag by removing the {namespace} prefix for URDF parser compatibility
            if isinstance(elem.tag, str) and elem.tag.startswith("{"):
                elem.tag = elem.tag.split("}", 1)[1]

            # Process children: filter out macros/properties and recurse into standard elements
            for child in list(elem):
                tag = child.tag

                # Handle non-string tags (e.g. Comments) safely
                if not isinstance(tag, str):
                    cleanup(child)
                    continue

                # Aggressive filtering: Remove skip and any tag containing "xacro"
                # We strip the namespace first to avoid false positives from the URI itself
                clean_tag = tag.split("}", 1)[1] if tag.startswith("{") else tag
                is_xacro = clean_tag == "skip" or "xacro" in clean_tag

                if is_xacro:
                    elem.remove(child)
                else:
                    cleanup(child)

        cleanup(root)
        from ..utils.xml_utils import serialize_xml

        return serialize_xml(root)

    def _find_file(self, filename: str) -> Path | None:
        """Find file in search paths, supporting both relative paths and package:// URIs.

        Args:
            filename: The filename or URI to search for.

        Returns:
            The absolute path to the file if found, otherwise None.
        """
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

        # Forward caller-supplied args to the resolver, skipping internal keys
        # and None values (str(None) would override xacro defaults like default="").
        for k, v in kwargs.items():
            if k not in ["search_paths", "start_dir"] and v is not None:
                resolver.args[k] = resolver._try_parse_typed_value(str(v))

        urdf_string = resolver.resolve_file(filepath)

        return URDFParser().parse_string(
            urdf_string, urdf_directory=filepath.parent, default_name=filepath.stem, **kwargs
        )
