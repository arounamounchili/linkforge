import bpy

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_transmission,
)


def test_link_name_sync(clean_scene):
    """Verify that renaming a basic object updates the LinkForge link name."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    obj = create_test_object("base_link", None, scene=scene)

    # Mark it as a robot link
    obj_lf = safe_get_linkforge(obj, scene=scene)
    obj_lf.is_robot_link = True
    obj_lf.link_name = "base_link"

    # Rename it in the outliner
    obj.name = "chassis"
    scene.view_layers[0].update()

    # Success: LinkForge should now match
    assert obj_lf.link_name == "chassis"


def test_link_child_renaming(clean_scene):
    """Verify standard children are renamed while custom meshes are kept safe."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    parent = create_test_object("base_link", None, scene=scene)

    parent_lf = safe_get_linkforge(parent, scene=scene)
    parent_lf.is_robot_link = True
    parent_lf.link_name = "base_link"

    # Standard naming (should rename)
    v_mesh = bpy.data.meshes.new("base_link_visual")
    visual = create_test_object("base_link_visual", v_mesh, scene=scene)
    visual.parent = parent

    # Custom naming (should stay the same)
    c_mesh = bpy.data.meshes.new("camera_lens")
    custom = create_test_object("camera_lens", c_mesh, scene=scene)
    custom.parent = parent

    # Rename the main link
    parent.name = "housing"
    scene.view_layers[0].update()

    # Check results
    assert visual.name == "housing_visual"
    assert custom.name == "camera_lens"  # Custom name was protected


def test_joint_name_sync(clean_scene):
    """Verify that joint outliner renames are synchronized."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    obj = create_test_object("arm_joint", None, scene=scene)

    obj_lf = safe_get_joint(obj, scene=scene)
    obj_lf.is_robot_joint = True
    obj_lf.joint_name = "arm_joint"

    # Rename it
    obj.name = "elbow_joint"
    scene.view_layers[0].update()

    # Success
    assert obj_lf.joint_name == "elbow_joint"


def test_sensor_name_sync(clean_scene):
    """Verify that sensor outliner renames are synchronized."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    obj = create_test_object("lidar", None, scene=scene)

    obj_lf = safe_get_sensor(obj, scene=scene)
    obj_lf.is_robot_sensor = True
    obj_lf.sensor_name = "lidar"

    # Rename it
    obj.name = "scanner"
    scene.view_layers[0].update()

    # Success
    assert obj_lf.sensor_name == "scanner"


def test_transmission_name_sync(clean_scene):
    """Verify that transmission outliner renames are synchronized."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    obj = create_test_object("drive_train", None, scene=scene)

    obj_lf = safe_get_transmission(obj, scene=scene)
    obj_lf.is_robot_transmission = True
    obj_lf.transmission_name = "drive_train"

    # Rename it
    obj.name = "wheel_drive"
    scene.view_layers[0].update()

    # Success
    assert obj_lf.transmission_name == "wheel_drive"


def test_name_sanitization(clean_scene):
    """Verify that outliner renames are always sanitized for URDF."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    obj = create_test_object("link", None, scene=scene)

    obj_lf = safe_get_linkforge(obj, scene=scene)
    obj_lf.is_robot_link = True

    # Rename to something with spaces (illegal in URDF)
    obj.name = "front left wheel"
    scene.view_layers[0].update()

    # Success: Both current name and stored identity are sanitized
    assert obj_lf.link_name == "front_left_wheel"
    assert obj.name == "front_left_wheel"


def test_naming_guards():
    """Verify that empty names or non-robot objects are safely ignored."""
    from typing import Any

    # Empty name should be ignored
    obj = create_test_object("link", None)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(obj)
    obj_lf: Any = getattr(obj, "linkforge")
    obj_lf.is_robot_link = True
    obj_lf.link_name = ""  # Should return early
    assert obj_lf.link_name == "link"

    # Non-robot objects should be ignored by the sync handler
    obj2 = create_test_object("random_prop", None)
    collection.objects.link(obj2)
    obj2.name = "static_mesh"
    view_layer = bpy.context.view_layer
    assert view_layer is not None
    view_layer.update()

    # It shouldn't be marked as a robot joint
    from linkforge.blender.utils.scene_utils import is_robot_joint

    assert not is_robot_joint(obj2)


def test_sensor_and_transmission_guards(clean_scene):
    """Verify that sensors and transmissions handle naming edge cases safely."""
    scene = bpy.context.scene or bpy.data.scenes[0]

    # Sensor empty name
    obj = create_test_object("sensor", None, scene=scene)
    obj_lf_s = safe_get_sensor(obj, scene=scene)
    obj_lf_s.is_robot_sensor = True
    obj_lf_s.sensor_name = ""
    assert obj_lf_s.sensor_name == "sensor"

    # Transmission empty name
    obj2 = create_test_object("transmission", None, scene=scene)
    obj_lf_t = safe_get_transmission(obj2, scene=scene)
    obj_lf_t.is_robot_transmission = True
    obj_lf_t.transmission_name = ""
    assert obj_lf_t.transmission_name == "transmission"

    # Joint empty name
    obj3 = create_test_object("joint", None, scene=scene)
    obj_lf_j = safe_get_joint(obj3, scene=scene)
    obj_lf_j.is_robot_joint = True
    obj_lf_j.joint_name = ""
    assert obj_lf_j.joint_name == "joint"
