"""Unit tests for IComposer protocol and public core utility exports."""

from __future__ import annotations

import linkforge.core as lf
from linkforge.core import (
    IComposer,
    RobotBuilder,
    clean_float,
    format_float,
    format_scientific,
    is_valid_name,
    parse_scientific,
    sanitize_name,
)


def test_robot_builder_implements_icomposer_protocol() -> None:
    """Verify that RobotBuilder satisfies the IComposer protocol at runtime."""
    builder = RobotBuilder("test_robot")
    assert isinstance(builder, IComposer)

    # Check key protocol attributes and methods
    assert hasattr(builder, "robot")
    assert callable(builder.link)
    assert callable(builder.material)
    assert callable(builder.build)
    assert callable(builder.register_link_builder)
    assert callable(builder.push_parent)
    assert callable(builder.pop_parent)


def test_public_core_utility_exports() -> None:
    """Verify that domain utilities are accessible directly from linkforge.core."""
    assert lf.sanitize_name("my robot link") == "my_robot_link"
    assert sanitize_name("123link") == "_123link"
    assert is_valid_name("valid_link_name") is True
    assert is_valid_name("123_invalid") is False

    assert clean_float(1e-12) == 0.0
    assert clean_float(1.5) == 1.5
    assert format_float(1.200000) == "1.2"

    assert format_scientific(0.00123) == "1.23e-03"
    assert parse_scientific("1.23e-03", fallback=0.0) == 0.00123
    assert parse_scientific("invalid_number", fallback=42.0) == 42.0
