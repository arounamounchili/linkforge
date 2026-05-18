"""Tests for custom exceptions in LinkForge."""

from linkforge.core.exceptions import (
    LinkForgeError,
    RobotGeneratorError,
    RobotMathError,
    RobotModelError,
    RobotParserError,
    RobotParserIOError,
    RobotParserUnexpectedError,
    RobotParserXMLRootError,
    RobotPhysicsError,
    RobotSecurityError,
    RobotValidationError,
    RobotXacroError,
    RobotXacroExpressionError,
    RobotXacroRecursionError,
    ValidationErrorCode,
    XacroDetectedError,
)


def test_exception_hierarchy():
    """Verify the core exception inheritance structure."""
    assert issubclass(LinkForgeError, Exception)
    assert issubclass(RobotModelError, LinkForgeError)
    assert issubclass(RobotParserError, LinkForgeError)
    assert issubclass(RobotGeneratorError, LinkForgeError)
    assert issubclass(RobotParserIOError, RobotParserError)
    assert issubclass(RobotParserXMLRootError, RobotParserError)
    assert issubclass(RobotParserUnexpectedError, RobotParserError)
    assert issubclass(RobotPhysicsError, RobotModelError)
    assert issubclass(RobotValidationError, RobotModelError)
    assert issubclass(RobotSecurityError, RobotModelError)
    assert issubclass(RobotMathError, RobotModelError)
    assert issubclass(RobotXacroError, RobotParserError)
    assert issubclass(RobotXacroRecursionError, RobotXacroError)
    assert issubclass(RobotXacroExpressionError, RobotXacroError)
    assert issubclass(XacroDetectedError, RobotParserError)


def test_robot_parser_io_error():
    """Verify RobotParserIOError defaults and formatting."""
    err = RobotParserIOError()
    assert "unknown" in str(err)
    assert "error" in str(err)

    err2 = RobotParserIOError("robot.urdf", "file not found")
    assert "robot.urdf" in str(err2)
    assert "file not found" in str(err2)


def test_robot_parser_xml_root_error():
    """Verify RobotParserXMLRootError formatting."""
    err = RobotParserXMLRootError("mesh", "robot")
    assert "mesh" in str(err)
    assert "robot" in str(err)


def test_robot_parser_unexpected_error():
    """Verify RobotParserUnexpectedError message formatting branches."""
    # Test without original error (Line 99->101 coverage)
    err = RobotParserUnexpectedError("URDFParser")
    assert "Unexpected error in URDFParser" in str(err)
    assert ":" not in str(err)

    # Test with original error
    err_with = RobotParserUnexpectedError("URDFParser", ValueError("empty XML"))
    assert "Unexpected error in URDFParser: empty XML" in str(err_with)


def test_robot_physics_error():
    """Verify RobotPhysicsError formatting combinations."""
    # Test neither
    err1 = RobotPhysicsError(ValidationErrorCode.PHYSICS_VIOLATION, "negative mass")
    assert "[PHYSICS_PHYSICS_VIOLATION] negative mass" in str(err1)

    # Test only target
    err2 = RobotPhysicsError(ValidationErrorCode.PHYSICS_VIOLATION, "negative mass", target="link1")
    assert "target: link1" in str(err2)

    # Test only value
    err3 = RobotPhysicsError(ValidationErrorCode.PHYSICS_VIOLATION, "negative mass", value=-10.0)
    assert "value: -10.0" in str(err3)

    # Test both
    err4 = RobotPhysicsError(
        ValidationErrorCode.PHYSICS_VIOLATION, "negative mass", target="link1", value=-10.0
    )
    assert "target: link1" in str(err4)
    assert "value: -10.0" in str(err4)


def test_robot_validation_error():
    """Verify RobotValidationError formatting combinations."""
    # Test neither
    err_none = RobotValidationError(ValidationErrorCode.DUPLICATE_NAME, "duplicate link name")
    assert str(err_none) == "[DUPLICATE_NAME] duplicate link name"

    # Test only target
    err_target = RobotValidationError(
        ValidationErrorCode.DUPLICATE_NAME, "duplicate link name", target="link_a"
    )
    assert str(err_target) == "[DUPLICATE_NAME] duplicate link name (target: link_a)"

    # Test only value
    err_value = RobotValidationError(
        ValidationErrorCode.DUPLICATE_NAME, "duplicate link name", value="link_val"
    )
    assert str(err_value) == "[DUPLICATE_NAME] duplicate link name (value: link_val)"

    # Test both
    err_both = RobotValidationError(
        ValidationErrorCode.DUPLICATE_NAME, "duplicate link name", target="link_a", value="link_val"
    )
    assert (
        str(err_both) == "[DUPLICATE_NAME] duplicate link name (target: link_a) (value: link_val)"
    )


def test_robot_security_error():
    """Verify RobotSecurityError formatting."""
    err = RobotSecurityError("/etc/passwd", "sandbox escape")
    assert "/etc/passwd" in str(err)
    assert "sandbox escape" in str(err)


def test_robot_math_error():
    """Verify RobotMathError message formatting branches (Lines 178->180, 180->183)."""
    # Test neither target nor value
    err_none = RobotMathError(ValidationErrorCode.MATH_ERROR, "divide by zero")
    assert str(err_none) == "[MATH_MATH_ERROR] divide by zero"

    # Test only target
    err_target = RobotMathError(ValidationErrorCode.MATH_ERROR, "divide by zero", target="joint_a")
    assert "target: joint_a" in str(err_target)
    assert "value" not in str(err_target)

    # Test only value
    err_value = RobotMathError(ValidationErrorCode.MATH_ERROR, "divide by zero", value=float("nan"))
    assert "value: nan" in str(err_value)
    assert "target" not in str(err_value)

    # Test both target and value
    err_both = RobotMathError(
        ValidationErrorCode.MATH_ERROR,
        "divide by zero",
        target="joint_a",
        value=float("nan"),
    )
    assert "target: joint_a" in str(err_both)
    assert "value: nan" in str(err_both)


def test_robot_xacro_error():
    """Verify RobotXacroError formatting."""
    err1 = RobotXacroError("macro not found")
    assert "macro not found" in str(err1)

    err2 = RobotXacroError("macro not found", "line 15")
    assert "macro not found" in str(err2)
    assert "line 15" in str(err2)


def test_robot_xacro_recursion_error():
    """Verify RobotXacroRecursionError formatting and branches (Line 200)."""
    # Test without reason
    err_no_reason = RobotXacroRecursionError(100)
    assert "Recursion depth exceeded: 100" in str(err_no_reason)
    assert "(" not in str(err_no_reason)

    # Test with reason
    err_reason = RobotXacroRecursionError(100, "circular imports")
    assert "Recursion depth exceeded: 100 (circular imports)" in str(err_reason)


def test_robot_xacro_expression_error():
    """Verify RobotXacroExpressionError formatting."""
    err = RobotXacroExpressionError("1/0", "ZeroDivisionError")
    assert "${1/0}" in str(err)
    assert "ZeroDivisionError" in str(err)


def test_xacro_detected_error():
    """Verify XacroDetectedError formatting."""
    err = XacroDetectedError("myfile.xacro")
    assert "myfile.xacro" in str(err)
