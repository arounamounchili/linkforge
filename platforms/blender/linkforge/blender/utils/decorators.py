"""Decorators for LinkForge Blender operators."""

import functools
import traceback
import typing
from collections.abc import Callable

from ...linkforge_core.logging_config import get_logger

logger = get_logger(__name__)


# Common return types for Blender operators
OperatorReturn = set[
    typing.Literal["RUNNING_MODAL", "CANCELLED", "FINISHED", "PASS_THROUGH", "INTERFACE"]
]


def safe_execute(func: Callable[..., OperatorReturn]) -> Callable[..., OperatorReturn]:
    """Decorator to wrap operator execute methods with robust error handling.

    This ensures that unhandled exceptions are caught, logged with full tracebacks,
    and reported to the user as clean error messages instead of crashing Blender.

    Usage:
        @safe_execute
        def execute(self, context):
            ...
    """

    @functools.wraps(func)
    def wrapper(
        self: typing.Any, context: typing.Any, *args: typing.Any, **kwargs: typing.Any
    ) -> OperatorReturn:
        try:
            return func(self, context, *args, **kwargs)
        except Exception as e:
            # Determine self for reporting
            self_obj = self

            # Log full traceback for debugging
            logger.error(f"Generate Error: {e}")
            logger.error(traceback.format_exc())

            if self_obj and hasattr(self_obj, "report"):
                # Report clean error to user
                self_obj.report({"ERROR"}, f"Operation failed: {str(e)}")

            # Return CANCELLED to signal failure to Blender
            return {"CANCELLED"}

    return wrapper
