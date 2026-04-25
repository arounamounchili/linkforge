import bpy


def test_link_name_sync():
    """Verify that renaming a basic object updates the LinkForge link name."""
    from typing import Any

    # 1. Create a link
    obj = bpy.data.objects.new("base_link", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(obj)

    # 2. Mark it as a robot link
    obj_lf: Any = getattr(obj, "linkforge")
    obj_lf.is_robot_link = True
    obj_lf.link_name = "base_link"

    # 3. Rename it in the outliner
    obj.name = "chassis"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # 4. Success: LinkForge should now match
    assert obj_lf.link_name == "chassis"


def test_link_child_renaming():
    """Verify standard children are renamed while custom meshes are kept safe."""
    from typing import Any

    # 1. Setup a link with some meshes
    parent = bpy.data.objects.new("base_link", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(parent)
    parent_lf: Any = getattr(parent, "linkforge")
    parent_lf.is_robot_link = True
    parent_lf.link_name = "base_link"

    # Standard naming (should rename)
    v_mesh = bpy.data.meshes.new("base_link_visual")
    visual = bpy.data.objects.new("base_link_visual", v_mesh)
    collection.objects.link(visual)
    visual.parent = parent

    # Custom naming (should stay the same)
    c_mesh = bpy.data.meshes.new("camera_lens")
    custom = bpy.data.objects.new("camera_lens", c_mesh)
    collection.objects.link(custom)
    custom.parent = parent

    # 2. Rename the main link
    parent.name = "housing"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # 3. Check results
    assert visual.name == "housing_visual"
    assert custom.name == "camera_lens"  # Custom name was protected


def test_joint_name_sync():
    """Verify that joint outliner renames are synchronized."""
    from typing import Any

    # 1. Create a joint object
    obj = bpy.data.objects.new("arm_joint", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(obj)
    obj_lf: Any = getattr(obj, "linkforge_joint")
    obj_lf.is_robot_joint = True
    obj_lf.joint_name = "arm_joint"

    # 2. Rename it
    obj.name = "elbow_joint"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # 3. Success
    assert obj_lf.joint_name == "elbow_joint"


def test_sensor_name_sync():
    """Verify that sensor outliner renames are synchronized."""
    from typing import Any

    # 1. Create a lidar
    obj = bpy.data.objects.new("lidar", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(obj)
    obj_lf: Any = getattr(obj, "linkforge_sensor")
    obj_lf.is_robot_sensor = True
    obj_lf.sensor_name = "lidar"

    # 2. Rename it
    obj.name = "scanner"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # 3. Success
    assert obj_lf.sensor_name == "scanner"


def test_transmission_name_sync():
    """Verify that transmission outliner renames are synchronized."""
    from typing import Any

    # 1. Create a transmission
    obj = bpy.data.objects.new("drive_train", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(obj)
    obj_lf: Any = getattr(obj, "linkforge_transmission")
    obj_lf.is_robot_transmission = True
    obj_lf.transmission_name = "drive_train"

    # 2. Rename it
    obj.name = "wheel_drive"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # 3. Success
    assert obj_lf.transmission_name == "wheel_drive"


def test_name_sanitization():
    """Verify that outliner renames are always sanitized for URDF."""
    from typing import Any

    # 1. Create a link
    obj = bpy.data.objects.new("link", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(obj)
    obj_lf: Any = getattr(obj, "linkforge")
    obj_lf.is_robot_link = True

    # 2. Rename to something with spaces (illegal in URDF)
    obj.name = "front left wheel"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # 3. Success: Both current name and stored identity are sanitized
    assert obj_lf.link_name == "front_left_wheel"
    assert obj.name == "front_left_wheel"


def test_naming_guards():
    """Verify that empty names or non-robot objects are safely ignored."""
    from typing import Any

    # 1. Empty name should be ignored
    obj = bpy.data.objects.new("link", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(obj)
    obj_lf: Any = getattr(obj, "linkforge")
    obj_lf.is_robot_link = True
    obj_lf.link_name = ""  # Should return early
    assert obj_lf.link_name == "link"

    # 2. Non-robot objects should be ignored by the sync handler
    obj2 = bpy.data.objects.new("random_prop", None)
    collection.objects.link(obj2)
    obj2.name = "static_mesh"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # It shouldn't be marked as a robot joint
    from linkforge.blender.utils.scene_utils import is_robot_joint

    assert not is_robot_joint(obj2)


def test_sensor_and_transmission_guards():
    """Verify that sensors and transmissions handle naming edge cases safely."""
    from typing import Any

    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    # 1. Sensor empty name
    obj = bpy.data.objects.new("sensor", None)
    collection.objects.link(obj)
    obj_lf_s: Any = getattr(obj, "linkforge_sensor")
    obj_lf_s.is_robot_sensor = True
    obj_lf_s.sensor_name = ""
    assert obj_lf_s.sensor_name == "sensor"

    # 2. Transmission empty name
    obj2 = bpy.data.objects.new("transmission", None)
    collection.objects.link(obj2)
    obj_lf_t: Any = getattr(obj2, "linkforge_transmission")
    obj_lf_t.is_robot_transmission = True
    obj_lf_t.transmission_name = ""
    assert obj_lf_t.transmission_name == "transmission"

    # 3. Joint empty name
    obj3 = bpy.data.objects.new("joint", None)
    collection.objects.link(obj3)
    obj_lf_j: Any = getattr(obj3, "linkforge_joint")
    obj_lf_j.is_robot_joint = True
    obj_lf_j.joint_name = ""
    assert obj_lf_j.joint_name == "joint"
