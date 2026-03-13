"""Tests for filter_utils module."""

from __future__ import annotations

from types import SimpleNamespace

from linkforge.blender.utils.filter_utils import filter_items_by_name


def _get_simple_named_object(name: str):
    """Helper function to create a simple object with a 'name' attribute."""
    return SimpleNamespace(name=name)


def test_filter_dict_empty_search_returns_all() -> None:
    """Test empty search term returns all dict input items."""
    items = {"Link1": "obj1", "Link2": "obj2", "base_link": "obj3"}
    result = filter_items_by_name(items, "")

    assert result == items
    assert len(result) == 3


def test_filter_list_empty_search_returns_all() -> None:
    """Test empty search term returns all list input items."""
    items = [
        _get_simple_named_object("Link1"),
        _get_simple_named_object("Link2"),
        _get_simple_named_object("base_link"),
    ]
    result = filter_items_by_name(items, "")

    assert result == items
    assert len(result) == 3


def test_filter_dict_case_sensitivity_uppercase() -> None:
    """Test non case-sensitive filtering with uppercase search term."""
    items = {"Link1": "obj1", "Link2": "obj2", "base_link": "obj3"}
    result = filter_items_by_name(items, "LINK")

    assert len(result) == 3
    assert "Link1" in result
    assert "Link2" in result
    assert "base_link" in result


def test_filter_dict_case_sensitivity_lowercase() -> None:
    """Test non case-sensitive filtering with lowercase search term."""
    items = {"LINK1": "obj1", "LINK2": "obj2", "BASE_LINK": "obj3"}
    result = filter_items_by_name(items, "link")

    assert len(result) == 3
    assert "LINK1" in result
    assert "LINK2" in result
    assert "BASE_LINK" in result


def test_filter_dict_case_sensitivity_mixed() -> None:
    """Test non case-sensitive filtering with mixed case (upper and lower)."""
    items = {"Link1": "obj1", "LINK2": "obj2", "base_link": "obj3"}
    result = filter_items_by_name(items, "LiNk")

    assert len(result) == 3


def test_filter_list_case_sensitivity_uppercase() -> None:
    """Test non case-sensitive list filtering with uppercase search term."""
    items = [
        _get_simple_named_object("Link1"),
        _get_simple_named_object("link2"),
        _get_simple_named_object("BASE_LINK"),
    ]
    result = filter_items_by_name(items, "LINK")

    assert len(result) == 3
    assert result[0].name == "Link1"
    assert result[1].name == "link2"
    assert result[2].name == "BASE_LINK"


def test_filter_list_case_sensitivity_lowercase() -> None:
    """Test non case-sensitive list filtering with lowercase search term."""
    items = [
        _get_simple_named_object("LINK1"),
        _get_simple_named_object("Link2"),
        _get_simple_named_object("base_link"),
    ]
    result = filter_items_by_name(items, "link")

    assert len(result) == 3


def test_filter_dict_substring_match() -> None:
    """Test substring matching for dictionary input types."""
    items = {"Link1": "obj1", "Link2": "obj2", "base_link": "obj3", "sensor_1": "obj4"}
    result = filter_items_by_name(items, "Link")

    assert len(result) == 3
    assert "Link1" in result
    assert "Link2" in result
    assert "base_link" in result
    assert "sensor_1" not in result


def test_filter_list_substring_match() -> None:
    """Test substring matching for list input types."""
    items = [
        _get_simple_named_object("Link1"),
        _get_simple_named_object("Link2"),
        _get_simple_named_object("base_link"),
        _get_simple_named_object("sensor_1"),
    ]
    result = filter_items_by_name(items, "Link")

    assert len(result) == 3
    assert result[0].name == "Link1"
    assert result[1].name == "Link2"
    assert result[2].name == "base_link"


def test_filter_dict_exact_match() -> None:
    """Test exact match returns single item (dictionary input type)."""
    items = {"Link1": "obj1", "Link2": "obj2", "base_link": "obj3"}
    result = filter_items_by_name(items, "Link1")

    assert len(result) == 1
    assert "Link1" in result


def test_filter_list_exact_match() -> None:
    """Test exact match returns single item (list input type)."""
    items = [
        _get_simple_named_object("Link1"),
        _get_simple_named_object("Link2"),
        _get_simple_named_object("base_link"),
    ]
    result = filter_items_by_name(items, "Link2")

    assert len(result) == 1
    assert result[0].name == "Link2"


def test_filter_dict_no_matches() -> None:
    """Test that no matches returns empty dictionary."""
    items = {"Link1": "obj1", "Link2": "obj2"}
    result = filter_items_by_name(items, "sensor")

    assert result == {}
    assert len(result) == 0


def test_filter_list_no_matches() -> None:
    """Test that no matches returns empty list."""
    items = [_get_simple_named_object("Link1"), _get_simple_named_object("Link2")]
    result = filter_items_by_name(items, "sensor")

    assert result == []
    assert len(result) == 0


def test_filter_empty_dict() -> None:
    """Test filtering empty dictionary."""
    result = filter_items_by_name({}, "test")

    assert result == {}


def test_filter_empty_list() -> None:
    """Test filtering empty list."""
    result = filter_items_by_name([], "test")

    assert result == []


def test_filter_none_search_term_dict() -> None:
    """Test None search term returns all input items (dictionary)."""
    items = {"Link1": "obj1", "Link2": "obj2"}
    result = filter_items_by_name(items, None)  # type: ignore

    assert result == items


def test_filter_none_search_term_list() -> None:
    """Test None search term returns all input items (list)."""
    items = [_get_simple_named_object("Link1"), _get_simple_named_object("Link2")]
    result = filter_items_by_name(items, None)  # type: ignore

    assert result == items


def test_filter_whitespace_only_search() -> None:
    """Test search term with only whitespace returns all input items."""
    items = {"Link1": "obj1", "Link2": "obj2"}
    result = filter_items_by_name(items, "   ")

    assert result == items


def test_filter_leading_trailing_whitespace() -> None:
    """Test search term with leading/trailing whitespace."""
    items = {"Link1": "obj1", "Link2": "obj2"}
    result = filter_items_by_name(items, " Link1 ")

    # since the current implementation does not trim whitespace,
    # this shld not match "Link1" because we look for " Link1 "
    # (with spaces)
    assert len(result) == 0


def test_filter_special_chars() -> None:
    """Test that special chars are treated correctly as 'normal' strings."""
    items = {
        "Link.1": "obj1",
        "Link*2": "obj2",
        "Link?3": "obj3",
        "base_link": "obj4",
    }

    result = filter_items_by_name(items, ".")
    assert len(result) == 1
    assert "Link.1" in result

    result = filter_items_by_name(items, "*")
    assert len(result) == 1
    assert "Link*2" in result

    result = filter_items_by_name(items, "?")
    assert len(result) == 1
    assert "Link?3" in result


def test_filter_trailing_spaces_in_search_query() -> None:
    """Test search term with multiple trailing spaces."""
    items = {"Link  1": "obj1", "Link 2": "obj2", "Link1": "obj3"}

    result = filter_items_by_name(items, "Link  ")
    assert len(result) == 1
    assert "Link  1" in result


def test_filter_very_long_search_query() -> None:
    """Test with very long search query."""
    items = {"short": "obj1", "a" * 1000: "obj2"}

    result = filter_items_by_name(items, "a" * 1000)
    assert len(result) == 1
    assert "a" * 1000 in result


def test_filter_list_objects_missing_name_attr() -> None:
    """Test list filtering with objects missing name attribute."""
    valid_obj = _get_simple_named_object("Link1")

    class NoNameObject:
        """Helper Object w/o any attributes"""

        pass

    invalid_obj = NoNameObject()

    items = [valid_obj, invalid_obj]
    result = filter_items_by_name(items, "Link")

    assert len(result) == 1
    assert result[0] == valid_obj


def test_dict_input_returns_dict_type() -> None:
    """Test that dictionary input returns dictionary type."""
    items = {"Link1": "obj1"}
    result = filter_items_by_name(items, "Link")

    assert isinstance(result, dict)


def test_list_input_returns_list_type() -> None:
    """Test that list input returns list type."""
    items = [_get_simple_named_object("Link1")]
    result = filter_items_by_name(items, "Link")

    assert isinstance(result, list)
