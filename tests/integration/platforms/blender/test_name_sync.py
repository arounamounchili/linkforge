import bpy


def test_link_name_sync():
    """Test that renaming a Blender object updates its LinkForge URDF identity."""

    # 1. Setup: Create a link
    obj_data = None
    obj = bpy.data.objects.new("base_link", obj_data)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.linkforge.is_robot_link = True
    obj.linkforge.link_name = "base_link"

    # Initial state
    assert obj.name == "base_link"
    assert obj.linkforge.link_name == "base_link"

    # 2. Action: Rename in "Outliner" (simulated via API)
    obj.name = "chassis"

    # 3. Trigger depsgraph update
    bpy.context.view_layer.update()

    # 4. Verify: LinkForge name should now match the new Blender name
    assert obj.linkforge.link_name == "chassis"
    assert obj.linkforge.urdf_name_stored == "chassis"


def test_link_child_renaming():
    """Test that standard children are renamed but custom ones are preserved."""

    # 1. Setup: Create a link with standard and custom children
    parent = bpy.data.objects.new("base_link", None)
    bpy.context.collection.objects.link(parent)
    parent.linkforge.is_robot_link = True
    parent.linkforge.link_name = "base_link"

    # Standard visual
    visual_mesh = bpy.data.meshes.new("base_link_visual")
    visual = bpy.data.objects.new("base_link_visual", visual_mesh)
    bpy.context.collection.objects.link(visual)
    visual.parent = parent

    # Standard collision with suffix
    collision_mesh = bpy.data.meshes.new("base_link_collision.001")
    collision = bpy.data.objects.new("base_link_collision.001", collision_mesh)
    bpy.context.collection.objects.link(collision)
    collision.parent = parent

    # Custom child (should NOT be renamed)
    custom_mesh = bpy.data.meshes.new("camera_bracket")
    custom = bpy.data.objects.new("camera_bracket", custom_mesh)
    bpy.context.collection.objects.link(custom)
    custom.parent = parent

    # 2. Action: Rename parent
    parent.name = "torso"
    bpy.context.view_layer.update()

    # 3. Verify
    assert parent.linkforge.link_name == "torso"
    assert visual.name == "torso_visual"
    assert collision.name == "torso_collision.001"
    assert custom.name == "camera_bracket"  # Preserved!


def test_joint_name_sync():
    """Test that renaming a Joint object updates its LinkForge identity."""

    # 1. Setup: Create a joint
    obj = bpy.data.objects.new("arm_joint", None)
    bpy.context.collection.objects.link(obj)
    obj.linkforge_joint.is_robot_joint = True
    obj.linkforge_joint.joint_name = "arm_joint"

    # 2. Action: Rename
    obj.name = "elbow_joint"
    bpy.context.view_layer.update()

    # 3. Verify
    assert obj.linkforge_joint.joint_name == "elbow_joint"


def test_name_sanitization_on_sync():
    """Test that syncing also sanitizes the name for URDF."""

    # 1. Setup: Create a link
    obj = bpy.data.objects.new("link", None)
    bpy.context.collection.objects.link(obj)
    obj.linkforge.is_robot_link = True

    # 2. Action: Rename to something invalid for URDF (with spaces)
    # Blender allows spaces in object names
    obj.name = "front wheel"
    bpy.context.view_layer.update()

    # 3. Verify: LinkForge name is sanitized, but obj.name remains (Blender's rule)
    assert obj.linkforge.link_name == "front_wheel"
    # Note: LinkForge's setter currently also updates obj.name back to the sanitized version
    # to maintain consistency, which is good.
    assert obj.name == "front_wheel"
