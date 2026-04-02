import xml.etree.ElementTree as ET
from unittest import mock

import pytest
from linkforge_core.parsers.xacro_parser import (
    TEMPLATE_CACHE,
    XacroResolver,
    XacroTemplate,
)


@pytest.fixture(autouse=True)
def clear_cache():
    """Clear the global template cache before each test."""
    TEMPLATE_CACHE.clear()


def test_xacro_template_cache_population(tmp_path):
    """Verify that TEMPLATE_CACHE is populated after first resolution."""
    xacro_file = tmp_path / "robot.xacro"
    xacro_file.write_text(
        '<robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="l"/></robot>'
    )

    resolver = XacroResolver()
    resolver.resolve_file(xacro_file)

    assert xacro_file.resolve() in TEMPLATE_CACHE
    template = TEMPLATE_CACHE[xacro_file.resolve()]
    assert isinstance(template, XacroTemplate)
    assert template.root_tag == "robot"
    assert template.root_attrib == {"name": "test"}


def test_xacro_template_cache_reuse(tmp_path):
    """Verify that ET.parse is only called once for the same file."""
    xacro_file = tmp_path / "robot.xacro"
    xacro_file.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="l"/></robot>'
    )

    resolver = XacroResolver()

    with mock.patch("xml.etree.ElementTree.parse", wraps=ET.parse) as mock_parse:
        # First call: Should parse from disk
        resolver.resolve_file(xacro_file)
        assert mock_parse.call_count == 1

        # Second call: Should hit cache
        resolver.resolve_file(xacro_file)
        assert mock_parse.call_count == 1


def test_xacro_structural_inclusion_caching(tmp_path):
    """Verify that includes are structural and cached recursively."""
    inc_file = tmp_path / "inc.xacro"
    inc_file.write_text(
        '<root xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="inc_link"/></root>'
    )

    main_file = tmp_path / "main.xacro"
    main_file.write_text(
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{inc_file.name}"/></robot>'
    )

    resolver = XacroResolver(search_paths=[tmp_path])

    with mock.patch("xml.etree.ElementTree.parse", wraps=ET.parse) as mock_parse:
        resolver.resolve_file(main_file)
        # Should parse both files
        assert mock_parse.call_count == 2

        # Clear stack and resolve again
        resolver.resolve_file(main_file)
        # Should still be 2 (hit cache for both)
        assert mock_parse.call_count == 2


def test_xacro_template_isolation_with_deepcopy(tmp_path):
    """Verify that multiple resolutions of the same template with different args are isolated."""
    macro_file = tmp_path / "macro.xacro"
    macro_file.write_text("""
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="test_macro" params="prefix">
        <link name="${prefix}link"/>
    </xacro:macro>
</robot>
""")

    main_file = tmp_path / "main.xacro"
    main_file.write_text(f'''
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="{macro_file.name}"/>
    <xacro:test_macro prefix="${{my_prefix}}"/>
</robot>
''')

    resolver = XacroResolver(search_paths=[tmp_path])

    # Resolve with prefix 1
    resolver.args["my_prefix"] = "r1_"
    res1 = resolver.resolve_file(main_file)
    assert 'name="r1_link"' in res1

    # Resolve with prefix 2
    # Ensure properties and args are cleared between runs as they would be in different instances
    resolver2 = XacroResolver(search_paths=[tmp_path])
    resolver2.args["my_prefix"] = "r2_"
    res2 = resolver2.resolve_file(main_file)
    assert 'name="r2_link"' in res2

    # Cross-verify isolation
    assert 'name="r1_link"' not in res2


def test_xacro_template_namespaced_include_caching(tmp_path):
    """Verify that namespaced includes correctly wrap elements in structural containers."""
    inc_file = tmp_path / "inc.xacro"
    inc_file.write_text(
        '<root xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:property name="p" value="10"/></root>'
    )

    main_file = tmp_path / "main.xacro"
    main_file.write_text(
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="{inc_file.name}" ns="foo"/><link m="${{foo.p}}"/></robot>'
    )

    resolver = XacroResolver(search_paths=[tmp_path])
    res = resolver.resolve_file(main_file)
    assert 'm="10"' in res


def test_xacro_template_macro_collection(tmp_path):
    """Verify that macros in templates are correctly indexed and merged."""
    inc_file = tmp_path / "inc.xacro"
    inc_file.write_text(
        '<root xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="m"><link name="l"/></xacro:macro></root>'
    )

    resolver = XacroResolver()
    template = resolver._get_structural_template(inc_file)

    assert "m" in template.macros
    params, elem = template.macros["m"]
    assert params == []
    assert elem.tag.endswith("macro")


def test_xacro_structural_template_error_handling(tmp_path):
    """Verify that ET.ParseError is caught during structural template building."""
    bad_file = tmp_path / "bad.xacro"
    bad_file.write_text("<root>")  # Unclosed tag

    resolver = XacroResolver()
    from linkforge_core.exceptions import RobotXacroError

    with pytest.raises(RobotXacroError):
        resolver._get_structural_template(bad_file)


def test_xacro_template_conditional_include_static(tmp_path):
    """Verify that conditional includes are handled during the evaluation phase, not structural."""
    # Current implementation expands all includes structurally.
    # We need to verify if this breaks conditional includes.
    # Standard ROS xacro resolves includes statically/immediately.
    # If the include is inside a conditional, it should only be resolved if the condition is met.

    inc_file = tmp_path / "inc.xacro"
    inc_file.write_text(
        '<root xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="inc_link"/></root>'
    )

    main_file = tmp_path / "main.xacro"
    main_file.write_text(f'''
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:arg name="load" default="0" />
    <xacro:if value="$(arg load)">
        <xacro:include filename="{inc_file.name}"/>
    </xacro:if>
</robot>
''')

    resolver = XacroResolver(search_paths=[tmp_path])

    # 1. Condition false
    res1 = resolver.resolve_file(main_file)
    assert "inc_link" not in res1

    # 2. Condition true
    resolver2 = XacroResolver(search_paths=[tmp_path])
    resolver2.args["load"] = "1"
    res2 = resolver2.resolve_file(main_file)
    assert "inc_link" in res2
