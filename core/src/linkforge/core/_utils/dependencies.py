"""Utility for handling optional dependencies in LinkForge."""

from __future__ import annotations

from typing import Any


def get_yaml() -> Any:
    """Safely retrieve the yaml module if installed.

    Returns:
        The yaml module or None if not installed.
    """
    try:
        import yaml

        return yaml
    except ImportError:
        return None
