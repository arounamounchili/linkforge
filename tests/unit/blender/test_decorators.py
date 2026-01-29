"""Unit tests for Blender decorators using real environment."""

import logging

from linkforge.blender.utils.decorators import safe_execute


class SpyOperator:
    """A minimal spy of a Blender operator that records reports."""

    bl_idname = "test.operator"

    def __init__(self):
        self.reports = []

    def report(self, type_set, message):
        self.reports.append((type_set, message))


class TestSafeExecute:
    """Test standard error handling decorator."""

    def test_successful_execution(self):
        """Test that successful execution returns expected value."""
        op = SpyOperator()
        context = {}

        @safe_execute
        def successful_op(self, context):
            return {"FINISHED"}

        result = successful_op(op, context)
        assert result == {"FINISHED"}
        assert len(op.reports) == 0

    def test_exception_handling(self):
        """Test that exceptions are caught and reported."""
        op = SpyOperator()
        context = {}

        @safe_execute
        def failing_op(self, context):
            raise ValueError("Test Error")

        result = failing_op(op, context)

        # Should return CANCELLED
        assert result == {"CANCELLED"}

        # Should verify report was called
        assert len(op.reports) == 1
        type_set, message = op.reports[0]
        assert type_set == {"ERROR"}
        assert "Operation failed: Test Error" in message

    def test_logging(self, caplog):
        """Test that full traceback is logged."""
        op = SpyOperator()
        context = {}

        @safe_execute
        def failing_op(self, context):
            raise RuntimeError("Critical Failure")

        with caplog.at_level(logging.ERROR):
            failing_op(op, context)

        # Verify log contains error and traceback hint
        assert "Generate Error in" in caplog.text
        assert "Critical Failure" in caplog.text
