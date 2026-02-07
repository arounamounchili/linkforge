from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.operators.link_ops import (
    calculate_inertia_for_link,
    create_collision_for_link,
    execute_collision_preview_update,
    schedule_collision_preview_update,
)


def test_add_empty_link():
    """Test creating an empty link frame at the 3D cursor."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.scene.cursor.location = (1.0, 2.0, 3.0)

    bpy.ops.linkforge.add_empty_link()

    obj = bpy.context.active_object
    assert obj is not None
    assert obj.type == "EMPTY"
    assert obj.linkforge.is_robot_link is True
    assert tuple(obj.location) == pytest.approx((1.0, 2.0, 3.0))
    assert obj.empty_display_type == "PLAIN_AXES"


def test_create_link_from_mesh():
    """Test converting a mesh to a robot link."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    cube = bpy.context.active_object
    cube.name = "test_cube"

    bpy.ops.linkforge.create_link_from_mesh()

    # Active object should now be the Empty parent
    link_empty = bpy.context.active_object
    assert link_empty.type == "EMPTY"
    assert link_empty.name == "test_cube"
    assert link_empty.linkforge.is_robot_link is True

    # Cube should be parented and renamed
    assert cube.parent == link_empty
    assert cube.name == "test_cube_visual"
    assert tuple(cube.location) == pytest.approx((0, 0, 0))


def test_generate_collision_box():
    """Test generating a box collision for a link."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # Select it and run operator
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj

    bpy.ops.linkforge.generate_collision()

    collision_obj = next((c for c in link_obj.children if "_collision" in c.name), None)
    assert collision_obj is not None
    assert collision_obj.type == "MESH"
    assert collision_obj["collision_geometry_type"] == "BOX"
    # Cube (size 2) bounding box is 2x2x2
    assert tuple(collision_obj.dimensions) == pytest.approx((2.0, 2.0, 2.0))


def test_generate_collision_sphere():
    """Test generating a sphere collision for a link."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    create_collision_for_link(link_obj, "SPHERE", bpy.context)

    collision_obj = next((c for c in link_obj.children if "_collision" in c.name), None)
    assert collision_obj is not None
    assert collision_obj["collision_geometry_type"] == "SPHERE"
    assert tuple(collision_obj.dimensions) == pytest.approx((2.0, 2.0, 2.0))


def test_generate_collision_cylinder():
    """Test generating a cylinder collision for a link."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=2.0)
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    create_collision_for_link(link_obj, "CYLINDER", bpy.context)

    collision_obj = next((c for c in link_obj.children if "_collision" in c.name), None)
    assert collision_obj is not None
    assert collision_obj["collision_geometry_type"] == "CYLINDER"
    assert tuple(collision_obj.dimensions) == pytest.approx((2.0, 2.0, 2.0))


def test_generate_collision_hull():
    """Test generating a convex hull collision for a link."""
    bpy.ops.object.select_all(action="DESELECT")
    # Use a non-primitive shape (monkey)
    bpy.ops.mesh.primitive_monkey_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    create_collision_for_link(link_obj, "CONVEX_HULL", bpy.context)

    collision_obj = next((c for c in link_obj.children if "_collision" in c.name), None)
    assert collision_obj is not None
    assert collision_obj["collision_geometry_type"] == "CONVEX_HULL"
    # Monkey is roughly 2.7x1.7x1.9
    assert collision_obj.dimensions.x > 2.0


def test_calculate_inertia_box():
    """Test calculating inertia for a box link."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # Set mass
    link_obj.linkforge.mass = 2.0

    # Generate collision first so inertia has a target
    create_collision_for_link(link_obj, "BOX", bpy.context)

    # Select it and run operator
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj

    bpy.ops.linkforge.calculate_inertia()

    # Solid box inertia: Ixx = 1/12 * m * (y^2 + z^2)
    # m=2, y=2, z=2 => 1/12 * 2 * (4 + 4) = 16/12 = 1.333
    assert link_obj.linkforge.inertia_ixx == pytest.approx(1.333333, rel=1e-3)
    assert link_obj.linkforge.inertia_iyy == pytest.approx(1.333333, rel=1e-3)
    assert link_obj.linkforge.inertia_izz == pytest.approx(1.333333, rel=1e-3)


def test_calculate_inertia_sphere():
    """Test calculating inertia for a sphere link."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    link_obj.linkforge.mass = 2.0
    create_collision_for_link(link_obj, "SPHERE", bpy.context)

    success = calculate_inertia_for_link(link_obj)
    assert success is True
    # Solid sphere inertia: I = 2/5 * m * r^2
    # m=2, r=1 => 2/5 * 2 * 1 = 0.8
    assert link_obj.linkforge.inertia_ixx == pytest.approx(0.8)


def test_calculate_inertia_cylinder():
    """Test calculating inertia for a cylinder link."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=2.0)
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    link_obj.linkforge.mass = 2.0
    create_collision_for_link(link_obj, "CYLINDER", bpy.context)

    success = calculate_inertia_for_link(link_obj)
    assert success is True
    # Cylinder (Z-axis) inertia: Izz = 1/2 * m * r^2, Ixx = Iyy = 1/12 * m * (3r^2 + h^2)
    # m=2, r=1, h=2 => Izz = 1, Ixx = 1/12 * 2 * (3 + 4) = 14/12 = 1.1666
    assert link_obj.linkforge.inertia_izz == pytest.approx(1.0)
    assert link_obj.linkforge.inertia_ixx == pytest.approx(1.166666, rel=1e-3)


def test_remove_link():
    """Test removing a link and its children."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    obj_name = link_obj.name

    bpy.ops.linkforge.remove_link()

    # Object should be gone
    assert obj_name not in bpy.data.objects


def test_toggle_collision_visibility():
    """Test toggling visibility of collision meshes."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    create_collision_for_link(link_obj, "BOX", bpy.context)

    collision_obj = next(c for c in link_obj.children if "_collision" in c.name)

    # Ensure link_obj is active and selected for the operator
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj

    # Initially hidden
    assert collision_obj.hide_viewport is True

    # Toggle (Show)
    bpy.ops.linkforge.toggle_collision_visibility()
    assert collision_obj.hide_viewport is False

    # Toggle (Hide)
    bpy.ops.linkforge.toggle_collision_visibility()
    assert collision_obj.hide_viewport is True


def test_generate_collision_all():
    """Test generating collisions for all links in the scene."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create two links
    for _i in range(2):
        bpy.ops.mesh.primitive_cube_add()
        bpy.ops.linkforge.create_link_from_mesh()

    bpy.ops.linkforge.generate_collision_all()

    # Check that both have collisions
    collisions = [obj for obj in bpy.data.objects if "_collision" in obj.name]
    assert len(collisions) == 2


def test_calculate_inertia_all():
    """Test calculating inertia for all links in the scene."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create two links with mass and collisions
    for _i in range(2):
        bpy.ops.mesh.primitive_cube_add()
        bpy.ops.linkforge.create_link_from_mesh()
        link_obj = bpy.context.active_object
        link_obj.linkforge.mass = 1.0
        create_collision_for_link(link_obj, "BOX", bpy.context)

    bpy.ops.linkforge.calculate_inertia_all()

    # Check that both have inertia calculated
    links = [obj for obj in bpy.data.objects if obj.linkforge.is_robot_link]
    for link in links:
        assert link.linkforge.inertia_ixx > 0


def test_add_material_slot():
    """Test adding a material slot to a link's visual mesh."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    visual_obj = next(c for c in link_obj.children if "_visual" in c.name)

    assert len(visual_obj.data.materials) == 0

    bpy.ops.linkforge.add_material_slot()

    assert len(visual_obj.data.materials) == 1
    assert visual_obj.data.materials[0].name == f"{link_obj.name}_material"


def test_schedule_collision_preview(mocker):
    """Test that collision preview update is scheduled via timer."""
    mock_register = mocker.patch("bpy.app.timers.register")

    obj = MagicMock()
    schedule_collision_preview_update(obj)

    mock_register.assert_called_once()
    # Check if execute_collision_preview_update was the callback
    args, kwargs = mock_register.call_args
    assert args[0].__name__ == "execute_collision_preview_update"
    assert kwargs["first_interval"] == 0.3


def test_execute_collision_preview_no_obj():
    """Test that preview update handles missing pending object."""
    from linkforge.blender.operators import link_ops

    link_ops._preview_pending_object = None

    result = execute_collision_preview_update()
    assert result is None
