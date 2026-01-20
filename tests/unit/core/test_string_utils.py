"""Tests for string utility functions."""

from __future__ import annotations

import pytest

from linkforge.core.utils.string_utils import sanitize_name


def test_sanitize_name_basic():
    """Test basic name sanitization."""
    assert sanitize_name("test link") == "test_link"
    assert sanitize_name("test-link") == "test-link"
    assert sanitize_name("test-link", allow_hyphen=False) == "test_link"


def test_sanitize_name_empty():
    """Test sanitization of empty names."""
    assert sanitize_name("") == ""
    assert sanitize_name(None) == ""


def test_sanitize_name_starting_with_number():
    """Test names starting with numbers."""
    assert sanitize_name("1link") == "_1link"


def test_sanitize_name_special_characters():
    """Test names with special characters."""
    assert sanitize_name("link@#!$") == "link____"


def test_sanitize_name_too_long():
    """Test ReDoS protection with long names."""
    long_name = "a" * 1001
    with pytest.raises(ValueError, match="Name too long"):
        sanitize_name(long_name)


def test_sanitize_name_all_special():
    """Test names made only of special characters."""
    assert sanitize_name("!@#$") == "____"
