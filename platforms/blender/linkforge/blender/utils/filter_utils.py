"""Filtering utilities for robot components search."""

from __future__ import annotations

from typing import Any, overload


@overload
def filter_items_by_name(
    items: dict[str, Any],
    search_term: str | None,
) -> dict[str, Any]: ...


@overload
def filter_items_by_name(
    items: list[Any],
    search_term: str | None,
) -> list[Any]: ...


def filter_items_by_name(
    items: dict[str, Any] | list[Any],
    search_term: str | None,
) -> dict[str, Any] | list[Any]:
    """Filter items by non case-sensitive substring matching on their names.

    For dictionaries: filters by keys (dictionary keys treated as names).
    For lists: filters by the '.name' attribute of objects.

    Args:
        items: Dictionary (name->object) or list of objects to be filtered
        search_term: Search string to match against item names
          (non case-sensitive)

    Returns:
        Filtered items in the same format as input. If search_term is empty
          or None, returns all items.
    """
    if search_term is None:
        return items

    if isinstance(search_term, str) and search_term.strip() == "":
        return items

    search_lower = search_term.lower()

    if isinstance(items, dict):
        return {name: obj for name, obj in items.items() if search_lower in name.lower()}

    return [obj for obj in items if hasattr(obj, "name") and search_lower in obj.name.lower()]
