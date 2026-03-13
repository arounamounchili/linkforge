import bpy


def test_validation_issue_line_splitting() -> None:
    """Test correctly splitting long messages and suggestions into lines."""
    # We can't easily instantiate PropertyGroups directly without registration/ID data integration,
    # but we can test the property methods on a registered instance.

    wm = bpy.context.window_manager
    res = wm.linkforge_validation
    res.clear()

    # Add an error
    err = res.errors.add()
    err.title = "Long Error"
    err.message = "This is a very long message that should be split into multiple lines because it exceeds the sixty character limit specified in the code."
    err.suggestion = (
        "Try to simplify the robot structure by removing redundant links or checking joint types."
    )

    # Check message lines (max 60 chars)
    lines = err.message_lines
    assert len(lines) > 1
    for line in lines:
        assert len(line) <= 60

    # Check suggestion lines (max 58 chars)
    s_lines = err.suggestion_lines
    assert len(s_lines) > 1
    for line in s_lines:
        assert len(line) <= 58

    assert err.has_suggestion is True
    assert err.has_objects is False


def test_validation_result_clearing() -> None:
    """Test clearing validation results."""
    wm = bpy.context.window_manager
    res = wm.linkforge_validation

    # Fill with some data
    res.has_results = True
    res.is_valid = False
    res.error_count = 1
    res.errors.add()

    # Clear
    res.clear()

    assert res.has_results is False
    assert res.error_count == 0
    assert len(res.errors) == 0


def test_validation_issue_objects_str() -> None:
    """Test affected objects string formatting."""
    wm = bpy.context.window_manager
    res = wm.linkforge_validation
    res.clear()

    issue = res.warnings.add()
    issue.affected_objects = "base_link, wheel_link"

    assert issue.has_objects is True
    assert issue.objects_str == "base_link, wheel_link"
