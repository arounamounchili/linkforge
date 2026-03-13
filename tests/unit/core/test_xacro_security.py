import pytest
from linkforge_core.base import RobotParserError
from linkforge_core.parsers.xacro_parser import XacroResolver


class TestXacroEvalSandbox:
    """Tests to ensure the XACRO eval() sandbox properly blocks malicious code."""

    @pytest.fixture
    def resolver(self):
        """Provide a fresh XacroResolver instance for each test."""
        return XacroResolver()

    def test_allows_safe_math(self, resolver) -> None:
        """Verify that standard mathematical operations are allowed."""

        # Simple arithmetic
        assert resolver._evaluate("1 + 2 * 3") == 7
        assert resolver._evaluate("10 / 2.0") == 5.0

        # Math module functions via MATH_CONTEXT
        assert resolver._evaluate("sin(0)") == 0.0
        assert resolver._evaluate("cos(pi)") == pytest.approx(-1.0)
        assert resolver._evaluate("pow(2, 3)") == 8

    def test_blocks_builtins(self, resolver) -> None:
        """Verify that standard Python built-ins like open() are blocked."""
        with pytest.raises(RobotParserError, match="name 'open' is not defined"):
            resolver._evaluate("open('/etc/passwd', 'r').read()")

        with pytest.raises(RobotParserError, match="name 'eval' is not defined"):
            resolver._evaluate("eval('1 + 1')")

        with pytest.raises(RobotParserError, match="name 'exec' is not defined"):
            resolver._evaluate("exec('x = 1')")

    def test_blocks_dunder_attribute_access(self, resolver) -> None:
        """Verify that dunder attributes (__class__, etc.) are strictly blocked.

        This is the primary mitigation against the well-known eval() sandbox
        escape vector: `"".__class__.__mro__[1].__subclasses__()`
        """
        malicious_payloads = [
            # Direct class access
            "''.__class__",
            "1.__class__",
            # The full typical exploit chain
            "''.__class__.__mro__[1].__subclasses__()",
            # Subclasses method
            "().__class__.__base__.__subclasses__()",
            # Globals access
            "__globals__",
            # Builtins access
            "__builtins__",
            # Import access
            "__import__('os')",
        ]

        for payload in malicious_payloads:
            with pytest.raises(RobotParserError, match="dunder attributes.*are not allowed"):
                resolver._evaluate(payload)
