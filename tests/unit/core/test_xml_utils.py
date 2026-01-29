import xml.etree.ElementTree as ET

from linkforge_core.utils.xml_utils import (
    parse_float,
    parse_int,
    parse_optional_bool,
    parse_optional_float,
    parse_vector3,
    serialize_xml,
    validate_xml_depth,
)


def test_parse_float_valid():
    assert parse_float("1.23") == 1.23
    assert parse_float(None, default=5.0) == 5.0
    assert parse_float("  0.1  ") == 0.1


def test_parse_int_valid():
    assert parse_int("10") == 10
    assert parse_int(None, default=5) == 5


def test_parse_vector3_valid():
    vec = parse_vector3("1 2 3")
    assert vec.x == 1.0
    assert vec.y == 2.0
    assert vec.z == 3.0


def test_parse_optional_bool():
    root = ET.fromstring("<root><val>true</val><other>false</other></root>")
    assert parse_optional_bool(root, "val") is True
    assert parse_optional_bool(root, "other") is False
    assert parse_optional_bool(root, "missing") is None


def test_parse_optional_float():
    root = ET.fromstring("<root><val>1.5</val></root>")
    assert parse_optional_float(root, "val") == 1.5
    assert parse_optional_float(root, "missing") is None


def test_serialize_xml():
    root = ET.Element("robot", name="test")
    xml_str = serialize_xml(root, version="1.2.0")
    assert "Robot: test" in xml_str
    assert "v1.2.0" in xml_str
    assert '<robot name="test"' in xml_str


def test_validate_xml_depth_ok():
    root = ET.fromstring("<root><a/></root>")
    validate_xml_depth(root)  # Should not raise


def test_serialize_xml_with_namespaces():
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
