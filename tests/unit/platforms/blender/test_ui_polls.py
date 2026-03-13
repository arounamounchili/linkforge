from linkforge.blender.panels.control_panel import LINKFORGE_PT_control
from linkforge.blender.panels.joint_panel import LINKFORGE_PT_joints
from linkforge.blender.panels.link_panel import LINKFORGE_PT_links
from linkforge.blender.panels.robot_panel import LINKFORGE_OT_select_root_link
from linkforge.blender.panels.sensor_panel import LINKFORGE_PT_perceive


def test_panels_exist() -> None:
    # Verify that panel classes are correctly named and imported
    assert LINKFORGE_PT_links.bl_idname == "LINKFORGE_PT_links"
    assert LINKFORGE_PT_joints.bl_idname == "LINKFORGE_PT_joints"
    assert LINKFORGE_PT_perceive.bl_idname == "LINKFORGE_PT_perceive"
    assert LINKFORGE_PT_control.bl_idname == "LINKFORGE_PT_control"


def test_operator_exists() -> None:
    assert LINKFORGE_OT_select_root_link.bl_idname == "linkforge.select_root_link"
