"""Blender Handlers for LinkForge.

This module contains background handlers that respond to Blender events:
- name_sync_handler: Synchronizes LinkForge names with Blender object names
"""

from __future__ import annotations

from . import name_sync_handler

modules = [
    name_sync_handler,
]


def register() -> None:
    """Register all handlers."""
    for module in modules:
        module.register()


def unregister() -> None:
    """Unregister all handlers."""
    for module in reversed(modules):
        module.unregister()
