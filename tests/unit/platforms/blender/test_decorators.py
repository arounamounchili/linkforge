"""Unit tests for Blender operator decorators."""

from __future__ import annotations

import typing
from unittest.mock import MagicMock

from linkforge.blender.utils.decorators import OperatorReturn, safe_execute
from linkforge.core import RobotModelError


class TestDecorators:
    def test_safe_execute_success(self, scene, blender_context) -> None:
        """Test successful execution of a decorated function."""
        mock_self = MagicMock()
        mock_self.reports = []
        mock_self.report = lambda t, m: mock_self.reports.append((t, m))

        @safe_execute
        def my_op(s: typing.Any, c: typing.Any) -> OperatorReturn:
            return {"FINISHED"}

        assert my_op(mock_self, None) == {"FINISHED"}
        assert len(mock_self.reports) == 0

    def test_safe_execute_failure(self, scene, blender_context) -> None:
        """Test error handling in a decorated function."""
        mock_self = MagicMock()
        mock_self.reports = []
        mock_self.report = lambda t, m: mock_self.reports.append((t, m))

        @safe_execute
        def failing_op(s: typing.Any, c: typing.Any) -> OperatorReturn:
            raise RobotModelError("Fail")

        assert failing_op(mock_self, None) == {"CANCELLED"}
        assert "Fail" in mock_self.reports[0][1]

    def test_safe_execute_failure_no_report(self, scene, blender_context) -> None:
        """Test error handling in a decorated function when self has no report method."""

        @safe_execute
        def failing_op(s: typing.Any, c: typing.Any) -> OperatorReturn:
            raise RobotModelError("Fail")

        assert failing_op(None, None) == {"CANCELLED"}

        class NoReport:
            pass

        assert failing_op(NoReport(), None) == {"CANCELLED"}
