"""Tests for validation result classes."""

from linkforge_core.validation.result import Severity, ValidationIssue, ValidationResult


class TestValidationIssue:
    """Tests for ValidationIssue class."""

    def test_str_with_affected_objects(self) -> None:
        """Test __str__ includes affected objects."""
        issue = ValidationIssue(
            severity=Severity.ERROR,
            title="Test Error",
            message="This is a test error",
            affected_objects=["link1", "link2"],
        )

        str_repr = str(issue)
        assert "ERROR" in str_repr
        assert "Test Error" in str_repr
        assert "link1" in str_repr
        assert "link2" in str_repr

    def test_str_without_affected_objects(self) -> None:
        """Test __str__ without affected objects."""
        issue = ValidationIssue(
            severity=Severity.WARNING,
            title="Test Warning",
            message="This is a test warning",
        )

        str_repr = str(issue)
        assert "WARNING" in str_repr
        assert "Test Warning" in str_repr
        assert "[" not in str_repr  # No affected objects

    def test_is_error(self) -> None:
        """Test is_error property."""
        error = ValidationIssue(severity=Severity.ERROR, title="Error", message="msg")
        warning = ValidationIssue(severity=Severity.WARNING, title="Warning", message="msg")

        assert error.is_error
        assert not warning.is_error

    def test_is_warning(self) -> None:
        """Test is_warning property."""
        error = ValidationIssue(severity=Severity.ERROR, title="Error", message="msg")
        warning = ValidationIssue(severity=Severity.WARNING, title="Warning", message="msg")

        assert not error.is_warning
        assert warning.is_warning


class TestValidationResult:
    """Tests for ValidationResult class."""

    def test_empty_result_is_valid(self) -> None:
        """Test that empty result is valid."""
        result = ValidationResult()
        assert result.is_valid
        assert len(result.issues) == 0

    def test_add_error(self) -> None:
        """Test adding an error."""
        result = ValidationResult()
        result.add_error(title="Error", message="Test error")

        assert not result.is_valid
        assert len(result.issues) == 1
        assert result.issues[0].is_error

    def test_add_warning(self) -> None:
        """Test adding a warning."""
        result = ValidationResult()
        result.add_warning(title="Warning", message="Test warning")

        assert result.is_valid  # Warnings don't invalidate
        assert len(result.issues) == 1
        assert result.issues[0].is_warning
