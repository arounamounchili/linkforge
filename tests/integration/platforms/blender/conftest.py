"""Integration test configuration for Blender."""

from __future__ import annotations

import typing

import pytest

from tests.mock_bpy_env import setup_mock_bpy

bpy = setup_mock_bpy()
HAS_BPY = True

if HAS_BPY:
    # Always force registration of linkforge properties to ensure test stability
    import linkforge.blender

    @pytest.fixture(scope="session", autouse=True)
    def register_addon() -> None:
        """Ensure the addon is registered at the start of the session."""
        linkforge.blender.register()

    @pytest.fixture(scope="module", autouse=True)
    def ensure_registered():
        """Ensure LinkForge properties are registered and fully active."""
        from tests.blender_test_utils import ensure_linkforge_registered

        ensure_linkforge_registered()
        yield

    @pytest.fixture(autouse=True)
    def blender_clean_scene() -> typing.Generator[None, None, None]:
        """Clear all objects and data from the scene before each test."""
        from tests.blender_test_utils import cleanup_blender_scene

        cleanup_blender_scene(bpy.context.scene)
        yield
