import xml.etree.ElementTree as ET
from pathlib import Path

from linkforge_core.exceptions import RobotModelError
from linkforge_core.utils.xml_utils import (
    MAX_XML_DEPTH,
    parse_float,
    parse_int,
    parse_optional_bool,
    parse_optional_float,
    parse_vector3,
    serialize_xml,
    validate_xml_depth,
)


def test_parse_float_valid() -> None:
    assert parse_float("1.23") == 1.23
    assert parse_float(None, default=5.0) == 5.0
    assert parse_float("  0.1  ") == 0.1


def test_parse_int_valid() -> None:
    assert parse_int("10") == 10
    assert parse_int(None, default=5) == 5


def test_parse_vector3_valid() -> None:
    vec = parse_vector3("1 2 3")
    assert vec.x == 1.0
    assert vec.y == 2.0
    assert vec.z == 3.0


def test_parse_optional_bool() -> None:
    root = ET.fromstring("<root><val>true</val><other>false</other></root>")
    assert parse_optional_bool(root, "val") is True
    assert parse_optional_bool(root, "other") is False
    assert parse_optional_bool(root, "missing") is None


def test_parse_optional_float() -> None:
    root = ET.fromstring("<root><val>1.5</val></root>")
    assert parse_optional_float(root, "val") == 1.5
    assert parse_optional_float(root, "missing") is None


def test_serialize_xml() -> None:
    root = ET.Element("robot", name="test")
    xml_str = serialize_xml(root, version="1.2.0")
    assert "Robot: test" in xml_str
    assert "v1.2.0" in xml_str
    assert '<robot name="test"' in xml_str


def test_validate_xml_depth_ok() -> None:
    root = ET.fromstring("<root><a/></root>")
    validate_xml_depth(root)  # Should not raise


def test_serialize_xml_with_namespaces() -> None:
    """Test XML serialization with custom namespaces."""
    import xml.etree.ElementTree as ET

    from linkforge_core.utils.xml_utils import serialize_xml

    root = ET.Element("robot")
    child = ET.SubElement(root, "link")
    child.set("name", "test")

    # Serialize with custom namespace
    xml_str = serialize_xml(root, namespaces={"custom": "http://example.com/custom"})

    assert "<robot" in xml_str
    assert "test" in xml_str


def test_parsing_fallbacks() -> None:
    """Test fallbacks for invalid numeric and boolean parsing."""
    import pytest

    # Invalid floats
    with pytest.raises(RobotModelError, match="NaN"):
        parse_float("NaN")
    with pytest.raises(RobotModelError, match="Infinite"):
        parse_float("inf")
    with pytest.raises(RobotModelError, match="outside reasonable range"):
        parse_float("1e11")
    with pytest.raises(RobotModelError, match="Invalid value"):
        parse_float("not-a-float")

    # Invalid ints
    with pytest.raises(RobotModelError, match="outside reasonable range"):
        parse_int("2000000")
    with pytest.raises(RobotModelError, match="Invalid value"):
        parse_int("not-an-int")

    # Optional bool
    root = ET.fromstring("<root><val>TRUE</val></root>")
    assert parse_optional_bool(root, "val") is True
    root2 = ET.fromstring("<root><val>yep</val></root>")
    assert parse_optional_bool(root2, "val") is False


def test_parse_vector3_errors() -> None:
    """Test parse_vector3 with various errors."""
    import pytest

    with pytest.raises(RobotModelError, match="Expected 3 values"):
        parse_vector3("1 2")
    with pytest.raises(RobotModelError, match="Expected 3 values"):
        parse_vector3("1 2 3 4")
    with pytest.raises(RobotModelError, match="Invalid Vector3 format"):
        parse_vector3("1 2 a")


def test_validate_xml_depth_exceeded() -> None:
    """Test XML depth validation with exceeding depth."""
    import pytest

    # Create very deep XML
    root = ET.Element("root")
    curr = root
    for _ in range(MAX_XML_DEPTH + 1):
        curr = ET.SubElement(curr, "a")

    with pytest.raises(RobotModelError, match="XML nesting too deep"):
        validate_xml_depth(root)


def test_parsing_missing_attribute() -> None:
    """Test RobotModelError when attribute is missing and no default is provided."""
    import pytest

    with pytest.raises(RobotModelError, match="Missing required attribute"):
        parse_float(None, attribute_name="test_float")
    with pytest.raises(RobotModelError, match="Missing required attribute"):
        parse_int(None, attribute_name="test_int")
    # Whitespace only should be treated as None
    with pytest.raises(RobotModelError, match="Missing required attribute"):
        parse_float("   ", attribute_name="test_float")
    with pytest.raises(RobotModelError, match="Missing required attribute"):
        parse_int("   ", attribute_name="test_int")


def test_validate_package_uri_complex() -> None:
    """Test complex valid package URI."""
    from linkforge_core.validation.security import validate_package_uri

    uri = "package://my_robot/meshes/arm.stl"
    assert validate_package_uri(uri) == uri


def test_is_suspicious_location_match() -> None:
    """Test suspicious location detection."""
    from linkforge_core.validation.security import is_suspicious_location

    # On most systems /etc exists and resolves to /private/etc or itself.
    # We use a path that is definitely suspicious.
    assert is_suspicious_location(Path("/etc/passwd")) is True


def test_xml_add_text() -> None:
    """Test xml_add_text utility function."""
    from linkforge_core.utils.xml_utils import xml_add_text

    parent = ET.Element("root")

    # Test adding a string
    elem1 = xml_add_text(parent, "child1", "value1")
    assert elem1.tag == "child1"
    assert elem1.text == "value1"
    assert parent.find("child1") is not None

    # Test adding a number
    elem2 = xml_add_text(parent, "child2", 42)
    assert elem2.tag == "child2"
    assert elem2.text == "42"

    # Test adding None (should not set text)
    elem3 = xml_add_text(parent, "child3", None)
    assert elem3.tag == "child3"
    assert elem3.text is None


def test_xml_add_vector() -> None:
    """Test xml_add_vector utility function."""
    from linkforge_core.models import Vector3
    from linkforge_core.utils.xml_utils import xml_add_vector

    parent = ET.Element("root")
    vec = Vector3(1.1234, 2.0, -3.5)

    # Custom formatter for testing
    def mock_formatter(val: float) -> str:
        return f"{val:.2f}"

    elem = xml_add_vector(parent, "origin", vec, formatter=mock_formatter)

    assert elem.tag == "origin"
    assert elem.text == "1.12 2.00 -3.50"
    assert parent.find("origin") is not None
