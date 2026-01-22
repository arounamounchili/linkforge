"""Unit tests for decorators."""

import logging
import sys
from unittest.mock import MagicMock


# Define compatible base classes to avoid metaclass conflicts
class MockOperator:
    """Mock Blender Operator class."""

    bl_idname = "test.operator"
    bl_label = "Test Operator"

    def report(self, type, message):
        pass


class MockExportHelper:
    """Mock ExportHelper class."""


class MockImportHelper:
    """Mock ImportHelper class."""


# Mock bpy and bpy_extras
mock_bpy = MagicMock()
mock_bpy.types.Operator = MockOperator
mock_bpy.props = MagicMock()

mock_bpy_extras = MagicMock()
mock_bpy_extras.io_utils.ExportHelper = MockExportHelper
mock_bpy_extras.io_utils.ImportHelper = MockImportHelper

sys.modules["mathutils"] = MagicMock()
sys.modules["gpu"] = MagicMock()
sys.modules["gpu_extras"] = MagicMock()
sys.modules["gpu_extras.batch"] = MagicMock()
sys.modules["bmesh"] = MagicMock()
sys.modules["blf"] = MagicMock()

# Inject mocks into sys.modules
sys.modules["bpy"] = mock_bpy
sys.modules["bpy.types"] = mock_bpy.types
sys.modules["bpy.props"] = mock_bpy.props
sys.modules["bpy_extras"] = mock_bpy_extras
sys.modules["bpy_extras.io_utils"] = mock_bpy_extras.io_utils

# Late imports must happen after mocking
from linkforge.blender.utils.decorators import safe_execute  # noqa: E402


class TestSafeExecute:
    """Test standard error handling decorator."""

    def test_successful_execution(self):
        """Test that successful execution returns expected value."""
        mock_operator = MagicMock()
        mock_context = MagicMock()

        @safe_execute
        def successful_op(self, context):
            return {"FINISHED"}

        result = successful_op(mock_operator, mock_context)
        assert result == {"FINISHED"}

    def test_exception_handling(self):
        """Test that exceptions are caught and reported."""
        mock_operator = MagicMock()
        mock_context = MagicMock()

        @safe_execute
        def failing_op(self, context):
            raise ValueError("Test Error")

        result = failing_op(mock_operator, mock_context)

        # Should return CANCELLED
        assert result == {"CANCELLED"}

        # Should verify report was called
        mock_operator.report.assert_called_once()
        args = mock_operator.report.call_args
        assert args[0][0] == {"ERROR"}
        assert "Operation failed: Test Error" in args[0][1]

    def test_logging(self, caplog):
        """Test that full traceback is logged."""
        mock_operator = MagicMock()
        mock_context = MagicMock()

        @safe_execute
        def failing_op(self, context):
            raise RuntimeError("Critical Failure")

        with caplog.at_level(logging.ERROR):
            failing_op(mock_operator, mock_context)

        # Verify log contains error and traceback hint
        assert "Generate Error in" in caplog.text
        assert "Critical Failure" in caplog.text
