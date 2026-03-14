import bpy


def test_link_urdf_name_persistence() -> None:
    """Test that link_name remains persistent even if Blender renames the object."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj1 = bpy.context.active_object

    # 1. Set name
    obj1.linkforge.link_name = "base_link"
    assert obj1.name == "base_link"
    assert obj1.linkforge.link_name == "base_link"
    assert obj1.linkforge.urdf_name_stored == "base_link"

    # 2. Simulate Blender renaming (e.g. by manual rename or suffixing)
    obj1.name = "base_link_manual"
    # Getter should still return the stored URDF name
    assert obj1.linkforge.link_name == "base_link"

    # 3. Conflict resolution simulation
    bpy.ops.object.empty_add()
    obj2 = bpy.context.active_object
    # This should be renamed by Blender to base_link.001 if base_link existed,
    # but here we manually test the setter's behavior with a conflict
    obj2.linkforge.link_name = "base_link"
    assert obj2.linkforge.link_name == "base_link"
    # The physical object name might be different due to Blender's internal state
    # but the logical name must match our intent.


def test_joint_urdf_name_persistence() -> None:
    """Test that joint_name remains persistent even if Blender renames the object."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge_joint.is_robot_joint = True

    # 1. Set name
    obj.linkforge_joint.joint_name = "elbow_joint"
    assert obj.name == "elbow_joint"
    assert obj.linkforge_joint.joint_name == "elbow_joint"
    assert obj.linkforge_joint.urdf_name_stored == "elbow_joint"

    # 2. Simulate Blender suffixing
    obj.name = "elbow_joint.001"
    # Getter MUST return the persistent "elbow_joint" for ROS 2 Control mapping
    assert obj.linkforge_joint.joint_name == "elbow_joint"


def test_reimport_name_matching() -> None:
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
        "base_link": bpy.data.objects.new("base_link", None),
        "shoulder_link": bpy.data.objects.new("shoulder_link", None),
    }

    for obj in links.values():
        obj.linkforge.is_robot_link = True

    obj = create_joint_object(joint_model, links)

    assert obj is not None
    # Verify persistent identity survives creation
    assert obj.linkforge_joint.urdf_name_stored == "shoulder_joint"
    assert obj.linkforge_joint.joint_name == "shoulder_joint"

    # Clean up
    bpy.data.objects.remove(obj)


def test_auto_linking_integration() -> None:
    """Test that the builder auto-links real ROS 2 Control pointers by URDF identity."""
    from pathlib import Path

    from linkforge.blender.logic.asynchronous_builder import AsynchronousRobotBuilder
    from linkforge_core.models import Joint, JointType, Link, Robot

    # Setup robot and links
    l1 = Link(name="l1")
    l2 = Link(name="l2")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
    robot = Robot(name="test", initial_links=[l1, l2], initial_joints=[j1])

    # Setup scene-level ROS 2 control config
    scene = bpy.context.scene
    scene.linkforge.use_ros2_control = True
    rc_joint = scene.linkforge.ros2_control_joints.add()
    rc_joint.name = "j1"
    # Initially dangling
    rc_joint.joint_obj = None

    builder = AsynchronousRobotBuilder(robot, Path("/tmp/fake.urdf"), bpy.context)

    # Populate joint_objects with persistent identity
    joint_obj = bpy.data.objects.new("j1.001", None)
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.urdf_name_stored = "j1"

    builder.joint_objects["j1"] = joint_obj

    # Trigger finalize logic for auto-linking
    builder._execute_task("finalize", None)

    # Verify re-linking by URDF Identity (casting to Any to avoid dynamic property errors)
    lp_final = scene.linkforge  # type: ignore[attr-defined]
    rc_joint = lp_final.ros2_control_joints[0]
    assert rc_joint.joint_obj == joint_obj
    assert rc_joint.joint_obj.name == "j1.001"  # Linked despite suffix

    # Cleanup
    bpy.data.objects.remove(joint_obj)
    scene.linkforge.ros2_control_joints.clear()
