"""Tests for string utility functions."""

from __future__ import annotations

import pytest
from linkforge.core import RobotModelError
from linkforge.core._utils.string_utils import is_valid_name, sanitize_name


def test_sanitize_name_basic() -> None:
    """Test basic name sanitization."""
    assert sanitize_name("test link") == "test_link"
    assert sanitize_name("test-link") == "test-link"
    assert sanitize_name("test-link", allow_hyphen=False) == "test_link"


def test_sanitize_name_empty() -> None:
    """Test sanitization of empty names."""
    assert sanitize_name("") == ""
    assert sanitize_name(None) == ""  # type: ignore[arg-type]


def test_sanitize_name_starting_with_number() -> None:
    """Test names starting with numbers."""
    assert sanitize_name("1link") == "_1link"


def test_sanitize_name_special_characters() -> None:
    """Test names with special characters."""
    assert sanitize_name("link@#!$") == "link____"


def test_sanitize_name_too_long() -> None:
    """Test ReDoS protection with long names."""
    long_name = "a" * 1001
    with pytest.raises(RobotModelError):
        sanitize_name(long_name)


def test_sanitize_name_all_special() -> None:
    """Test names made only of special characters."""
    assert sanitize_name("!@#$") == "____"


# Tests for is_valid_name


def test_is_valid_name_valid() -> None:
    """Test valid names without hyphens."""
    assert is_valid_name("base_link") is True
    assert is_valid_name("link1") is True
    assert is_valid_name("my_robot_link") is True
    assert is_valid_name("LinkName") is True


def test_is_valid_name_with_hyphen() -> None:
    """Test names with hyphens (valid in URDF/SDF)."""
    assert is_valid_name("base-link") is True
    assert is_valid_name("base-link", allow_hyphen=True) is True
    assert is_valid_name("base-link", allow_hyphen=False) is False
    assert is_valid_name("my-robot-link") is True


def test_is_valid_name_empty() -> None:
    """Test empty name (invalid)."""
    assert is_valid_name("") is False


def test_is_valid_name_starts_with_digit() -> None:
    """Test names starting with digits (invalid)."""
    assert is_valid_name("1link") is False
    assert is_valid_name("2nd_link") is False
    assert is_valid_name("0base") is False


def test_is_valid_name_special_characters() -> None:
    """Test names with invalid special characters."""
    assert is_valid_name("base link") is False  # Space
    assert is_valid_name("base@link") is False  # @
    assert is_valid_name("base#link") is False  # #
    assert is_valid_name("base!link") is False  # !
    assert is_valid_name("base$link") is False  # $
    assert is_valid_name("base.link") is False  # .


def test_is_valid_name_underscore_only() -> None:
    """Test names with only underscores (valid)."""
    assert is_valid_name("_") is True
    assert is_valid_name("__") is True
    assert is_valid_name("_link") is True


def test_is_valid_name_mixed_case() -> None:
    """Test mixed case names."""
    assert is_valid_name("BaseLink") is True
    assert is_valid_name("base_Link") is True
    assert is_valid_name("BASE_LINK") is True
