"""Unit tests for Core dictionary utilities."""

from __future__ import annotations

from dataclasses import dataclass

import pytest
from linkforge_core.utils.dict_utils import AttrDict, filter_items_by_name


class TestAttrDict:
    """Tests for the attribute-access dictionary."""

    def test_basic_access(self) -> None:
        """Test getting and setting attributes."""
        d = AttrDict({"a": 1})
        assert d.a == 1
        d.b = 2
        assert d["b"] == 2
        assert d.b == 2

    def test_nested_access(self) -> None:
        """Test recursive wrapping of nested dictionaries."""
        d = AttrDict({"outer": {"inner": 10}})
        assert isinstance(d.outer, AttrDict)
        assert d.outer.inner == 10

    def test_list_wrapping(self) -> None:
        """Test that dictionaries inside lists are also wrapped."""
        d = AttrDict({"elements": [{"val": 1}, {"val": 2}]})
        assert isinstance(d.elements[0], AttrDict)
        assert d.elements[1].val == 2

    def test_attribute_error(self) -> None:
        """Test that missing attributes raise AttributeError, not KeyError."""
        d = AttrDict({"a": 1})
        with pytest.raises(AttributeError):
            _ = d.non_existent

    def test_deletion(self) -> None:
        """Test deleting attributes."""
        d = AttrDict({"a": 1})
        del d.a
        assert "a" not in d
        with pytest.raises(AttributeError):
            del d.non_existent


@dataclass
class NamedObject:
    """Mock object for list filtering tests."""

    name: str


class TestFilterItemsByName:
    """Tests for the name-based filtering utility."""

    def test_filter_dict(self) -> None:
        """Test filtering a dictionary by keys."""
        items = {"base_link": 1, "link_1": 2, "joint_A": 3, "other": 4}

        # Match substring
        res = filter_items_by_name(items, "link")
        assert len(res) == 2
        assert "base_link" in res
        assert "link_1" in res

        # Case insensitive
        res = filter_items_by_name(items, "LINK")
        assert len(res) == 2

    def test_filter_list(self) -> None:
        """Test filtering a list of objects by their .name attribute."""
        items = [NamedObject("base_link"), NamedObject("link_1"), NamedObject("joint_A")]

        res = filter_items_by_name(items, "link")
        assert len(res) == 2
        assert any(obj.name == "base_link" for obj in res)
        assert any(obj.name == "link_1" for obj in res)

    def test_empty_search(self) -> None:
        """Test that empty search returns everything."""
        items = {"a": 1, "b": 2}
        assert filter_items_by_name(items, None) == items
        assert filter_items_by_name(items, "") == items
        assert filter_items_by_name(items, "   ") == items

    def test_no_matches(self) -> None:
        """Test behavior when no items match."""
        items = {"a": 1}
        assert filter_items_by_name(items, "xyz") == {}

        list_items = [NamedObject("a")]
        assert filter_items_by_name(list_items, "xyz") == []
