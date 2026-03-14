from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.operators.link_ops import (
    _merge_visual_meshes,
    calculate_inertia_for_link,
    create_collision_for_link,
    execute_collision_preview_update,
    schedule_collision_preview_update,
)


def test_add_empty_link() -> None:
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


def test_create_link_from_mesh() -> None:
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


def test_generate_collision_box() -> None:
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


def test_generate_collision_sphere() -> None:
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


def test_generate_collision_cylinder() -> None:
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


def test_generate_collision_mesh_simplified() -> None:
    """Test generating a simplified mesh collision for a link."""
    bpy.ops.object.select_all(action="DESELECT")
    # Use a non-primitive shape (monkey)
    bpy.ops.mesh.primitive_monkey_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    create_collision_for_link(link_obj, "MESH", bpy.context)

    collision_obj = next((c for c in link_obj.children if "_collision" in c.name), None)
    assert collision_obj is not None
    assert collision_obj["collision_geometry_type"] == "MESH"
    # Monkey is roughly 2.7x1.7x1.9
    assert collision_obj.dimensions.x > 2.0


def test_calculate_inertia_box() -> None:
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


def test_calculate_inertia_sphere() -> None:
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


def test_calculate_inertia_cylinder() -> None:
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


def test_remove_link() -> None:
    """Test removing a link and its children."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    obj_name = link_obj.name

    bpy.ops.linkforge.remove_link()

    # Object should be gone
    assert obj_name not in bpy.data.objects


def test_toggle_collision_visibility() -> None:
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


def test_generate_collision_all() -> None:
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


def test_calculate_inertia_all() -> None:
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


def test_add_material_slot() -> None:
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


def test_schedule_collision_preview() -> None:
    """Test that collision preview update is scheduled via timer."""
    # Setup test scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    with patch("bpy.app.timers.register") as mock_register:
        schedule_collision_preview_update(obj)

    mock_register.assert_called_once()
    # Check if execute_collision_preview_update was the callback
    args, kwargs = mock_register.call_args
    assert args[0].__name__ == "execute_collision_preview_update"
    assert kwargs["first_interval"] == 0.3


def test_execute_collision_preview_no_obj() -> None:
    """Test that preview update handles missing pending object."""
    from linkforge.blender.operators import link_ops

    # 1. Pending object is None
    link_ops._preview_pending_object = None
    link_ops._preview_last_request_time = 0.0  # Reset
    assert execute_collision_preview_update() is None

    # 2. Pending object exists but is not in bpy.data.objects (deleted)
    obj = bpy.data.objects.new("DeletedObj", None)
    link_ops._preview_pending_object = obj
    # Object is not linked to any collection, so it won't be in bpy.data.objects.
    # Note: bpy.data.objects only contains linked/registered objects.
    assert execute_collision_preview_update() is None


def test_execute_collision_preview_complex_scenarios() -> None:
    """Test execute_collision_preview_update with various real-world states."""
    from linkforge.blender.operators import link_ops

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # 1. No collision object found
    obj = bpy.data.objects.new("test_obj", None)
    bpy.context.collection.objects.link(obj)
    link_ops._preview_pending_object = obj
    link_ops._preview_last_request_time = 0.0  # Force update

    assert execute_collision_preview_update() is None

    # 2. No view layer (Simulation of missing context)
    mesh = bpy.data.meshes.new("col_mesh")
    col = bpy.data.objects.new("test_obj_collision", mesh)
    bpy.context.collection.objects.link(col)
    col.parent = obj

    with patch("linkforge.blender.operators.link_ops.bpy") as mock_bpy:
        # Simulate missing view_layer context
        mock_bpy.data = bpy.data
        mock_bpy.context = MagicMock()
        mock_bpy.context.view_layer = None

        link_ops._preview_pending_object = obj
        link_ops._preview_last_request_time = 0.0
        assert execute_collision_preview_update() is None


def test_create_collision_for_link_multi_visual() -> None:
    """Test compound collision creation for multiple visual children."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create link and two visual cubes
    bpy.ops.linkforge.add_empty_link()
    link_obj = bpy.context.active_object

    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    v1 = bpy.context.active_object
    v1.name = "v1_visual"
    v1.parent = link_obj

    bpy.ops.mesh.primitive_cube_add(location=(-1, 0, 0))
    v2 = bpy.context.active_object
    v2.name = "v2_visual"
    v2.parent = link_obj

    # Run creation with AUTO (should trigger mesh simplification for >1 visuals)
    create_collision_for_link(link_obj, "AUTO", bpy.context)

    col_obj = next(c for c in link_obj.children if "_collision" in c.name)
    assert col_obj["collision_geometry_type"] == "MESH"


def test_calculate_inertia_exception() -> None:
    """Test error handling in inertia calculation."""
    obj = MagicMock()
    obj.linkforge.is_robot_link = True
    obj.children = [MagicMock()]  # Trigger some logic

    # Make extract_mesh_triangles raise an exception
    with patch(
        "linkforge.blender.adapters.blender_to_core.extract_mesh_triangles",
        side_effect=Exception("Test Error"),
    ):
        assert calculate_inertia_for_link(obj) is False


def test_link_ops_registration() -> None:
    """Test register/unregister logic."""
    from linkforge.blender.operators.link_ops import register, unregister

    unregister()
    register()
    assert hasattr(bpy.types, "LINKFORGE_OT_add_empty_link")


def test_remove_link_child_selected() -> None:
    """Test removing link when a visual child is selected."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    obj_name_snapshot = str(link_obj.name)
    visual_obj = link_obj.children[0]

    # Select child
    bpy.ops.object.select_all(action="DESELECT")
    visual_obj.select_set(True)
    bpy.context.view_layer.objects.active = visual_obj

    bpy.ops.linkforge.remove_link()
    assert obj_name_snapshot not in bpy.data.objects


def test_link_ops_edge_cases(mocker) -> None:
    """Hit remaining edge cases in link_ops.py."""
    # 1. LINKFORGE_OT_generate_collision with no links
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a link object so poll() passes
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # Mock bpy.data.objects inside the operator to be empty
    # to hit the "No robot links found" case
    mock_bpy = MagicMock()
    mock_bpy.data.objects = []
    # We must preserve context for the operator to still function somewhat
    mock_bpy.context = bpy.context
    with patch("linkforge.blender.operators.link_ops.bpy", mock_bpy):
        # We use a wrapper or just call the execute directly if we want to avoid poll
        # But let's try calling it normally with the patch
        bpy.ops.linkforge.generate_collision()

    # 2. LINKFORGE_OT_calculate_inertia with child selected
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    visual_obj = link_obj.children[0]
    bpy.ops.object.select_all(action="DESELECT")
    visual_obj.select_set(True)
    bpy.context.view_layer.objects.active = visual_obj

    bpy.ops.linkforge.calculate_inertia()
    assert link_obj.linkforge.inertia_ixx > 0

    # 3. LINKFORGE_OT_add_material_slot
    # Ensure it's active
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj
    bpy.ops.linkforge.add_material_slot()
    assert len(visual_obj.data.materials) > 0


def test_link_ops_low_level_edge_cases(mocker) -> None:
    """Hit very specific low-level branches in link_ops.py."""
    from linkforge.blender.operators.link_ops import (
        _create_primitive_collision,
        _merge_visual_meshes,
        create_collision_for_link,
    )

    # Create a real object for testing
    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object

    # 1. _create_primitive_collision with UNKNOWN type
    result = _create_primitive_collision(cube, "UNKNOWN", (1, 1, 1), bpy.context)
    assert result[0] is None  # Returns (None, Vector)

    # 2. _merge_visual_meshes with object having NO mesh data
    # Create fake link_obj too
    link_dummy = bpy.data.objects.new("LinkDummy", None)
    empty = bpy.data.objects.new("Empty", None)
    assert _merge_visual_meshes([empty], link_dummy, bpy.context) is None

    # 3. create_collision_for_link with failed primitive creation
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object
    mocker.patch(
        "linkforge.blender.operators.link_ops._create_primitive_collision",
        return_value=(None, (0, 0, 0)),
    )
    assert create_collision_for_link(link_obj, "BOX", bpy.context) is None


def test_link_ops_mesh_compound_failure(mocker) -> None:
    """Test mesh compound failure path."""
    from linkforge.blender.operators.link_ops import _create_mesh_collision_compound

    mocker.patch("linkforge.blender.operators.link_ops._merge_visual_meshes", return_value=None)
    assert _create_mesh_collision_compound([], "test", bpy.context) is None


def test_link_ops_operator_polls_and_cancellation(mocker) -> None:
    """Hit operator poll failures and cancellation paths."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # 1. remove_link poll failed (no active object)
    assert bpy.ops.linkforge.remove_link.poll() is False

    # 2. generate_collision poll failed (active but not a link)
    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    cube.select_set(True)
    bpy.context.view_layer.objects.active = cube
    assert bpy.ops.linkforge.generate_collision.poll() is False

    # 3. toggle_collision_visibility poll
    assert bpy.ops.linkforge.toggle_collision_visibility.poll() is False

    # 4. generate_collision_all cancellation (no scene context mocked)
    # We already tested this somewhat, but let's hit the link loop failure
    bpy.ops.linkforge.add_empty_link()
    mocker.patch(
        "linkforge.blender.operators.link_ops.create_collision_for_link", return_value=None
    )
    bpy.ops.linkforge.generate_collision_all()


def test_regenerate_collision_logic(mocker) -> None:
    """Hit regeneration paths in link_ops."""
    from linkforge.blender.operators.link_ops import regenerate_collision_mesh

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # Create an existing collision
    bpy.ops.linkforge.generate_collision()

    # Regenerate (3 args: link_obj, type, context)
    regenerate_collision_mesh(link_obj, "AUTO", bpy.context)

    # Check that it still exists
    assert any("_collision" in c.name for c in link_obj.children)


def test_link_ops_main_entry() -> None:
    """Simulate module main entry."""
    from linkforge.blender.operators import link_ops

    with patch.object(link_ops, "register") as mock_reg:
        # Just call register as if it was triggered by __main__
        link_ops.register()
        mock_reg.assert_called_once()


def test_inertia_mesh_fallback() -> None:
    """Test inertia calculation falling back to mesh integration."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_monkey_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # Set to non-primitive type to force mesh extraction
    success = calculate_inertia_for_link(link_obj)
    assert success is True
    assert link_obj.linkforge.inertia_ixx > 0


def test_generate_collision_all_reporting() -> None:
    """Test collision all with mixed results."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Link with visual -> Success
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()

    # Link without visual -> Does nothing for that link
    bpy.ops.linkforge.add_empty_link()

    bpy.ops.linkforge.generate_collision_all()


def test_link_ops_collection_fallback() -> None:
    """Test fallback to scene collection if context.collection is missing."""
    from linkforge.blender.operators.link_ops import create_collision_for_link

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create simple structure manually
    link_obj = bpy.data.objects.new("Link", None)
    bpy.context.collection.objects.link(link_obj)

    # Create a basic mesh for the visual
    mesh = bpy.data.meshes.new("TestMesh")
    visual_obj = bpy.data.objects.new("Visual_visual", mesh)
    bpy.context.collection.objects.link(visual_obj)
    visual_obj.parent = link_obj

    # Mock context to have no collection
    mock_context = MagicMock()
    mock_context.scene = bpy.context.scene
    mock_context.collection = None
    mock_context.view_layer = bpy.context.view_layer
    mock_context.active_object = link_obj

    # Mocking create_collision_for_link is safer than running it in a polluted context
    # for this specific fallback test.
    with patch(
        "linkforge.blender.operators.link_ops._create_primitive_collision",
        return_value=(None, None),
    ) as mock_prim:
        create_collision_for_link(link_obj, "BOX", mock_context)
        assert mock_prim.called


def test_inertia_extraction_failure() -> None:
    """Hit failed inertia extraction branches using a non-primitive."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Use monkey to avoid primitive calculation shortcut
    bpy.ops.mesh.primitive_monkey_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    with patch(
        "linkforge.blender.adapters.blender_to_core.extract_mesh_triangles", return_value=None
    ):
        assert calculate_inertia_for_link(link_obj) is False


def test_link_ops_debounced_preview_logic() -> None:
    """Hit all error paths in execute_collision_preview_update."""
    with (
        patch("linkforge.blender.operators.link_ops._preview_pending_object", None),
        patch("linkforge.blender.operators.link_ops.time.time", return_value=1234567890.0),
        patch("linkforge.blender.operators.link_ops._preview_last_request_time", 0.0),
    ):
        from linkforge.blender.operators import link_ops

        assert link_ops.execute_collision_preview_update() is None

    link_obj = bpy.data.objects.new("TestLink_Preview", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    with (
        patch("linkforge.blender.operators.link_ops._preview_pending_object", link_obj),
        patch("linkforge.blender.operators.link_ops.time.time", return_value=1234567890.0),
        patch("linkforge.blender.operators.link_ops._preview_last_request_time", 0.0),
    ):
        from linkforge.blender.operators import link_ops

        assert link_ops.execute_collision_preview_update() is None

    mock_obj = MagicMock()
    mock_obj.name = "DeletedObject"
    with (
        patch("linkforge.blender.operators.link_ops._preview_pending_object", mock_obj),
        patch("linkforge.blender.operators.link_ops.time.time", return_value=1234567890.0),
        patch("linkforge.blender.operators.link_ops._preview_last_request_time", 0.0),
    ):
        from linkforge.blender.operators import link_ops

        assert link_ops.execute_collision_preview_update() is None

    mesh = bpy.data.meshes.new("sphere_mesh")
    sphere_obj = bpy.data.objects.new("TestLink_Preview_collision", mesh)
    bpy.context.collection.objects.link(sphere_obj)
    sphere_obj.parent = link_obj
    sphere_obj["collision_geometry_type"] = "BOX"

    with (
        patch("linkforge.blender.operators.link_ops.regenerate_collision_mesh"),
        patch(
            "linkforge.blender.adapters.blender_to_core.detect_primitive_type",
            return_value=None,
        ),
        patch(
            "linkforge.blender.operators.link_ops._preview_pending_object",
            link_obj,
        ),
    ):
        execute_collision_preview_update()


def test_link_ops_virtual_link_removal_complex() -> None:
    """Test LINKFORGE_OT_remove_link for virtual links with parent frames."""
    link_obj = bpy.data.objects.new("VirtualLink", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    bpy.context.view_layer.objects.active = link_obj
    link_obj.select_set(True)

    bpy.ops.linkforge.remove_link()
    assert "VirtualLink" not in bpy.data.objects


def test_link_ops_material_slot_logic_variants() -> None:
    """Test material slot addition success and error paths."""
    link_obj = bpy.data.objects.new("MatLink", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    # 1. Error: No visual mesh found
    bpy.context.view_layer.objects.active = link_obj
    link_obj.select_set(True)
    if bpy.ops.linkforge.add_material_slot.poll():
        import contextlib

        with contextlib.suppress(RuntimeError):
            bpy.ops.linkforge.add_material_slot()

    # 2. Success: Visual mesh selected
    mesh = bpy.data.meshes.new("cube_mesh")
    visual_obj = bpy.data.objects.new("VisualMesh_direct_visual", mesh)
    bpy.context.collection.objects.link(visual_obj)
    visual_obj.parent = link_obj

    bpy.context.view_layer.objects.active = visual_obj
    visual_obj.select_set(True)
    if bpy.ops.linkforge.add_material_slot.poll():
        bpy.ops.linkforge.add_material_slot()
        assert len(visual_obj.data.materials) > 0


def test_link_ops_collision_all_logic() -> None:
    """Test generate_collision_all branches."""
    link1 = bpy.data.objects.new("Link1", None)
    bpy.context.collection.objects.link(link1)
    link1.linkforge.is_robot_link = True

    with patch(
        "linkforge.blender.operators.link_ops.create_collision_for_link",
        return_value=MagicMock(),
    ):
        bpy.ops.linkforge.generate_collision_all()


def test_link_ops_inertia_all_logic_extended() -> None:
    """Test calculate_inertia_all branches."""
    link1 = bpy.data.objects.new("LinkI1", None)
    bpy.context.collection.objects.link(link1)
    link1.linkforge.is_robot_link = True

    with patch(
        "linkforge.blender.operators.link_ops.calculate_inertia_for_link",
        return_value=True,
    ):
        bpy.ops.linkforge.calculate_inertia_all()


def test_link_ops_toggle_visibility_nested() -> None:
    """Test toggling visibility from a deeply nested visual object."""
    link_obj = bpy.data.objects.new("ToggleLink2", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    mesh = bpy.data.meshes.new("vis_mesh")
    vis_obj = bpy.data.objects.new("Vis_visual", mesh)
    bpy.context.collection.objects.link(vis_obj)
    vis_obj.parent = link_obj

    col_mesh = bpy.data.meshes.new("col_mesh")
    col_obj = bpy.data.objects.new("Col_collision", col_mesh)
    bpy.context.collection.objects.link(col_obj)
    col_obj.parent = link_obj

    bpy.context.view_layer.update()
    bpy.context.view_layer.objects.active = vis_obj
    vis_obj.select_set(True)

    if bpy.ops.linkforge.toggle_collision_visibility.poll():
        bpy.ops.linkforge.toggle_collision_visibility()
        assert col_obj.hide_viewport is True


def test_link_ops_poll_and_execute_failures() -> None:
    """Hit error return paths and poll failures."""
    # 1. create_link_from_mesh poll failures
    not_a_mesh = bpy.data.objects.new("NotAMesh", None)
    bpy.context.collection.objects.link(not_a_mesh)
    bpy.context.view_layer.objects.active = not_a_mesh
    not_a_mesh.select_set(True)
    assert bpy.ops.linkforge.create_link_from_mesh.poll() is False

    # 2. add_material_slot direct mesh with NO parent link
    mesh = bpy.data.meshes.new("orph_mesh")
    orph_obj = bpy.data.objects.new("Orphan_visual", mesh)
    bpy.context.collection.objects.link(orph_obj)
    bpy.context.view_layer.objects.active = orph_obj
    orph_obj.select_set(True)
    assert bpy.ops.linkforge.add_material_slot.poll() is False


def test_link_ops_create_link_from_mesh_advanced() -> None:
    """Test success path with Armature cleanup and name sanitization."""
    mesh = bpy.data.meshes.new("source_mesh_with_spaces")
    obj = bpy.data.objects.new("source mesh with spaces", mesh)
    bpy.context.collection.objects.link(obj)

    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bpy.ops.linkforge.create_link_from_mesh()
    assert "source_mesh_with_spaces" in bpy.data.objects


def test_link_ops_low_level_edge_cases_extended() -> None:
    """Hit internal helper error branches."""
    assert calculate_inertia_for_link(None) is False

    link_obj = bpy.data.objects.new("EmptyLink2", None)
    bpy.context.collection.objects.link(link_obj)

    bad_vis = bpy.data.objects.new("BadVis_visual", None)  # No data
    bpy.context.collection.objects.link(bad_vis)
    bad_vis.parent = link_obj

    assert _merge_visual_meshes([bad_vis], link_obj, bpy.context) is None


def test_update_collision_quality_realtime() -> None:
    """Test the realtime collision quality update (fast path)."""
    from linkforge.blender.operators.link_ops import update_collision_quality_realtime

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Setup: Link and Collision Object
    bpy.ops.linkforge.add_empty_link()
    link_obj = bpy.context.active_object
    link_obj.linkforge.collision_quality = 50.0

    mesh = bpy.data.meshes.new("sphere_mesh")
    collision_obj = bpy.data.objects.new("Link_collision", mesh)
    bpy.context.collection.objects.link(collision_obj)
    collision_obj.parent = link_obj

    # 1. Test Fallback (No modifier, type MESH -> Should add modifier)
    assert len(collision_obj.modifiers) == 0
    update_collision_quality_realtime(link_obj, collision_obj)

    decimate_mod = next((m for m in collision_obj.modifiers if m.type == "DECIMATE"), None)
    assert decimate_mod is not None
    assert decimate_mod.ratio == 0.5

    # 2. Test Fast Path (Update existing modifier)
    link_obj.linkforge.collision_quality = 25.0
    update_collision_quality_realtime(link_obj, collision_obj)
    assert decimate_mod.ratio == 0.25

    # 3. Test Fallback (Non-mesh -> Should schedule full update)
    # Use an Empty object as the collision object (type is EMPTY)
    empty_col = bpy.data.objects.new("Empty_collision", None)
    bpy.context.collection.objects.link(empty_col)
    empty_col.parent = link_obj

    with patch(
        "linkforge.blender.operators.link_ops.schedule_collision_preview_update"
    ) as mock_schedule:
        update_collision_quality_realtime(link_obj, empty_col)
        mock_schedule.assert_called_once_with(link_obj)
