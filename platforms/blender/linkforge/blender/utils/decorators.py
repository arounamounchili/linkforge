"""Decorators for LinkForge Blender operators."""

import functools
import traceback
from collections.abc import Callable
from typing import Any

from ...linkforge_core.logging_config import get_logger

logger = get_logger(__name__)


def safe_execute(func: Callable) -> Callable:
    """Decorator to wrap operator execute methods with robust error handling.

    This ensures that unhandled exceptions are caught, logged with full tracebacks,
    and reported to the user as clean error messages instead of crashing Blender.

    Usage:
        @safe_execute
        def execute(self, context):
            ...
    """

    @functools.wraps(func)
    def wrapper(self, context: Any) -> set[str]:
        try:
            return func(self, context)
        except Exception as e:
            # Log full traceback for debugging
            logger.error(f"Generate Error in {self.bl_idname}: {e}")
            logger.error(traceback.format_exc())

            # Report clean error to user
            # We strip the traceback from the UI message to keep it pro
            self.report({"ERROR"}, f"Operation failed: {str(e)}")

            # Return CANCELLED to signal failure to Blender
            return {"CANCELLED"}

    return wrapper
