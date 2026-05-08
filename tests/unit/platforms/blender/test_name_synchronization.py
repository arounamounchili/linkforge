import bpy

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
)


def test_link_source_name_persistence(scene) -> None:
    """Test that link_name remains persistent even if Blender renames the object."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj1 = bpy.context.active_object
    assert obj1 is not None

    # Set name
    safe_get_linkforge(obj1).is_robot_link = True
    safe_get_linkforge(obj1).link_name = "base_link"
    assert obj1.name == "base_link"
    assert safe_get_linkforge(obj1).link_name == "base_link"
    assert safe_get_linkforge(obj1).source_name_stored == "base_link"

    # Simulate Blender renaming (e.g. by manual rename or suffixing)
    obj1.name = "chassis"
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.update()

    # Getter should now return the SYNCED name
    assert safe_get_linkforge(obj1).link_name == "chassis"

    # Conflict resolution simulation
    bpy.ops.object.empty_add()
    obj2 = bpy.context.active_object
    assert obj2 is not None
    # This should be renamed by Blender to base_link.001 if base_link existed,
    # but here we manually test the setter's behavior with a conflict
    safe_get_linkforge(obj2).link_name = "base_link"
    assert safe_get_linkforge(obj2).link_name == "base_link"


def test_joint_source_name_persistence(scene) -> None:
    """Test that joint_name remains persistent even if Blender renames the object."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    assert obj is not None
    safe_get_joint(obj).is_robot_joint = True

    # Set name
    safe_get_joint(obj).joint_name = "elbow_joint"
    assert obj.name == "elbow_joint"
    assert safe_get_joint(obj).joint_name == "elbow_joint"
    assert safe_get_joint(obj).source_name_stored == "elbow_joint"

    # Simulate Blender suffixing
    obj.name = "elbow_joint.001"
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.update()

    # Sync should have sanitized and updated both
    assert safe_get_joint(obj).joint_name == "elbow_joint_001"
    assert obj.name == "elbow_joint_001"


def test_reimport_name_matching(scene) -> None:
    """Test that the importer correctly sets persistent names using real data."""
    from linkforge.blender.adapters.core_to_blender import create_joint_object
    from linkforge_core.models import Joint, JointLimits, JointType, Vector3

    joint_model = Joint(
        name="shoulder_joint",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="shoulder_link",
        axis=Vector3(0, 0, 1),
        limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0),
    )

    links = {
        "base_link": create_test_object("base_link", None, scene),
        "shoulder_link": create_test_object("shoulder_link", None, scene),
    }

    for obj_link in links.values():
        safe_get_linkforge(obj_link).is_robot_link = True

    obj = create_joint_object(joint_model, links)

    assert obj is not None
    # Verify persistent identity survives creation
    assert safe_get_joint(obj).source_name_stored == "shoulder_joint"
    assert safe_get_joint(obj).joint_name == "shoulder_joint"

    # Clean up
    bpy.data.objects.remove(obj)


def test_auto_linking_integration(scene) -> None:
    """Test that the builder auto-links real ROS 2 Control pointers by robot model identity."""
    from pathlib import Path

    from linkforge.blender.logic.asynchronous_builder import AsynchronousRobotBuilder
    from linkforge_core.models import Joint, JointType, Link, Robot

    # Setup robot and links
    l1 = Link(name="l1")
    l2 = Link(name="l2")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
    robot = Robot(name="test", initial_links=[l1, l2], initial_joints=[j1])

    # Setup scene-level ROS 2 control config
    lf_scene = safe_get_linkforge_scene(scene)
    lf_scene.use_ros2_control = True
    rc_joint = lf_scene.ros2_control_joints.add()
    rc_joint.name = "j1"
    # Initially dangling
    rc_joint.joint_obj = None

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/fake.urdf"), bpy.context)

    # Populate joint_objects with persistent identity
    joint_obj = create_test_object("j1.001", None, scene)
    assert joint_obj is not None
    safe_get_joint(joint_obj).is_robot_joint = True
    safe_get_joint(joint_obj).source_name_stored = "j1"

    builder.joint_objects["j1"] = joint_obj

    # Trigger finalize logic for auto-linking
    builder._execute_task("finalize", None)

    # Verify re-linking by robot model Identity
    lp_final = safe_get_linkforge_scene(scene)
    rc_joint = lp_final.ros2_control_joints[0]

    # Trigger one more update to ensure synchronization has finished
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.update()

    assert rc_joint.joint_obj == joint_obj
    assert rc_joint.joint_obj.name == "j1_001"  # Name was sanitized and synced

    # Cleanup
    bpy.data.objects.remove(joint_obj)
    safe_get_linkforge_scene(scene).ros2_control_joints.clear()
