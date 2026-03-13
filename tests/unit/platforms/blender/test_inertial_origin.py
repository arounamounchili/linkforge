import bpy
import pytest
from linkforge.blender.adapters.blender_to_core import blender_link_to_core_with_origin


def test_inertial_origin_extraction() -> None:
    """Verify that inertial origin properties are correctly converted to Core model using real Blender objects."""
    # 1. Setup real Blender object
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object
    obj.name = "test_link"

    # 2. Configure LinkForge properties (real PropertyGroup)
    props = obj.linkforge
    props.is_robot_link = True
    props.link_name = "test_link"
    props.mass = 1.0
    props.use_auto_inertia = False  # Manual mode

    # Set the target properties we want to test
    props.inertia_origin_xyz = (1.2, 3.4, 5.6)
    props.inertia_origin_rpy = (0.1, 0.2, 0.3)

    # Set manual inertia tensor values
    props.inertia_ixx = 1.0
    props.inertia_ixy = 0.0
    props.inertia_ixz = 0.0
    props.inertia_iyy = 1.0
    props.inertia_iyz = 0.0
    props.inertia_izz = 1.0

    # 3. Call conversion
    link = blender_link_to_core_with_origin(obj)

    # 4. Assertions
    assert link is not None
    assert link.inertial is not None

    # Verify Position (LinkForge Core uses simple float comparison or clean_float internally)
    # We allow small precision differences if any, but with real objects it should match.
    assert pytest.approx(link.inertial.origin.xyz.x) == 1.2
    assert pytest.approx(link.inertial.origin.xyz.y) == 3.4
    assert pytest.approx(link.inertial.origin.xyz.z) == 5.6

    # Verify Rotation
    assert pytest.approx(link.inertial.origin.rpy.x) == 0.1
    assert pytest.approx(link.inertial.origin.rpy.y) == 0.2
    assert pytest.approx(link.inertial.origin.rpy.z) == 0.3
