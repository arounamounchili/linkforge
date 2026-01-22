"""UI Panel for robot-level properties and validation."""

from __future__ import annotations

import bpy
from bpy.types import Context, Operator, Panel

from ..utils.decorators import safe_execute


def build_tree_structure(scene):
    """Build robot tree structure from scene objects.

    Returns:
        tuple: (tree dict, root_link str, joints dict, links dict)
            - tree: mapping parent links to list of (child_link, joint_name, joint_type)
            - root_link: name of root link
            - joints: dict mapping (parent, child) to joint object
            - links: dict mapping link_name to link object

    """
    # Collect all links
    links = {obj.linkforge.link_name: obj for obj in scene.objects if obj.linkforge.is_robot_link}

    # Build parent->children mapping from joints
    tree = {link_name: [] for link_name in links}
    joints = {}
    root_link = None

    for obj in scene.objects:
        if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
            props = obj.linkforge_joint
            parent = props.parent_link
            child = props.child_link

            if parent and child and parent in tree:
                tree[parent].append((child, props.joint_name, props.joint_type))
                joints[(parent, child)] = obj

    # Find root (link with no parent)
    all_children = set()
    for children_list in tree.values():
        for child, _, _ in children_list:
            all_children.add(child)

    # Root is a link that appears as parent but not as child
    for link_name in links:
        if link_name not in all_children:
            root_link = link_name
            break

    return tree, root_link, joints, links


class LINKFORGE_OT_select_tree_object(Operator):
    """Select object from kinematic tree."""

    bl_idname = "linkforge.select_tree_object"
    bl_label = "Select Object"
    bl_description = "Select this object in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    object_name: bpy.props.StringProperty(
        name="Object Name", description="Name of the object to select"
    )  # type: ignore
    object_type: bpy.props.StringProperty(
        name="Object Type", description="Type of object (link, joint, sensor, transmission)"
    )  # type: ignore

    @safe_execute
    def execute(self, context):
        """Execute the operator."""
        # Find the object
        obj = context.scene.objects.get(self.object_name)
        if not obj:
            self.report({"WARNING"}, f"Object '{self.object_name}' not found")
            return {"CANCELLED"}

        # Deselect all
        bpy.ops.object.select_all(action="DESELECT")

        # Select and activate the object
        obj.select_set(True)
        context.view_layer.objects.active = obj

        return {"FINISHED"}


class LINKFORGE_PT_validate_export(Panel):
    """Validate & Export panel - robot configuration, validation, and export settings."""

    bl_label = "Validate & Export"
    bl_description = "Step 4: Validate robot structure and export to URDF/XACRO"
    bl_idname = "LINKFORGE_PT_validate_export"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 3
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context: Context):
        """Draw the panel."""
        layout = self.layout
        scene = context.scene
        props = scene.linkforge

        # Count components
        num_links = sum(1 for obj in scene.objects if obj.linkforge.is_robot_link)
        num_joints = sum(
            1 for obj in scene.objects if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint
        )
        num_sensors = sum(
            1
            for obj in scene.objects
            if obj.type == "EMPTY"
            and hasattr(obj, "linkforge_sensor")
            and obj.linkforge_sensor.is_robot_sensor
        )
        num_transmissions = sum(
            1
            for obj in scene.objects
            if obj.type == "EMPTY"
            and hasattr(obj, "linkforge_transmission")
            and obj.linkforge_transmission.is_robot_transmission
        )

        # Only show robot properties if there are links in the scene
        if num_links == 0:
            box = layout.box()
            box.label(text="No robot in scene", icon="INFO")
            box.label(text="Create links in Forge panel to start", icon="FORWARD")
            return

        # Build tree structure to find root
        tree, root_link, joints_dict, links_dict = build_tree_structure(scene)

        # Calculate total mass and DOF
        total_mass = 0.0
        total_dof = 0
        for obj in scene.objects:
            if obj.linkforge.is_robot_link and obj.linkforge.mass > 0:
                total_mass += obj.linkforge.mass

            # Count DOF from actuated joints
            if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
                joint_type = obj.linkforge_joint.joint_type
                # Map joint types to DOF contribution
                dof_map = {
                    "FIXED": 0,
                    "REVOLUTE": 1,
                    "CONTINUOUS": 1,
                    "PRISMATIC": 1,
                    "PLANAR": 2,
                    "FLOATING": 6,
                }
                total_dof += dof_map.get(joint_type, 0)

        # === ROBOT PROPERTIES (with essential stats) ===
        layout.separator()
        box = layout.box()
        box.label(text="Properties", icon="ARMATURE_DATA")
        box.prop(props, "robot_name")

        # Show stats in a compact grid layout
        if root_link or total_mass > 0 or total_dof > 0:
            box.separator()

            # Use grid flow for compact 2-column layout
            flow = box.grid_flow(row_major=True, columns=2, even_columns=False, align=True)

            # Root Link
            if root_link:
                flow.label(text="Root Link:")
                flow.label(text=root_link)

            # Total Mass
            if total_mass > 0:
                flow.label(text="Total Mass:")
                flow.label(text=f"{total_mass:.1f} kg")

            # DOF
            if total_dof > 0:
                flow.label(text="DOF:")
                flow.label(text=str(total_dof))

        # === VALIDATION (combined button + results) ===
        layout.separator()
        val_section = layout.box()
        val_section.label(text="Validation", icon="CHECKMARK")

        # Validate button
        row = val_section.row()
        row.scale_y = 1.3
        row.operator("linkforge.validate_robot", text="Validate Robot", icon="CHECKMARK")

        # Show validation results inline (inside same box)
        wm = context.window_manager
        if hasattr(wm, "linkforge_validation"):
            validation = wm.linkforge_validation
            if validation.has_results:
                val_section.separator()

                if validation.is_valid and validation.warning_count == 0:
                    val_section.label(text="Robot is valid!", icon="CHECKMARK")
                elif validation.is_valid:
                    val_section.label(
                        text=f"Valid with {validation.warning_count} warning(s)", icon="ERROR"
                    )
                    # Show warnings inline
                    if validation.warning_count > 0:
                        val_section.prop(
                            validation,
                            "show_warnings",
                            toggle=True,
                            text=f"Show {validation.warning_count} Warning(s)",
                            icon="TRIA_DOWN" if validation.show_warnings else "TRIA_RIGHT",
                        )
                        if validation.show_warnings:
                            for i in range(validation.warning_count):
                                warning = validation.get_warning(i)
                                warn_row = val_section.row()
                                warn_row.label(text=f"  • {warning.title}", icon="ERROR")
                                # Show all message lines (not just first)
                                if warning.message_lines:
                                    for msg_line in warning.message_lines:
                                        if msg_line.strip():  # Skip empty lines
                                            msg_row = val_section.row()
                                            msg_row.label(text=f"    {msg_line}", icon="BLANK1")
                else:
                    val_section.label(
                        text=f"{validation.error_count} error(s), {validation.warning_count} warning(s)",
                        icon="CANCEL",
                    )
                    # Show errors inline
                    if validation.error_count > 0:
                        val_section.prop(
                            validation,
                            "show_errors",
                            toggle=True,
                            text=f"Show {validation.error_count} Error(s)",
                            icon="TRIA_DOWN" if validation.show_errors else "TRIA_RIGHT",
                        )
                        if validation.show_errors:
                            for i in range(validation.error_count):
                                error = validation.get_error(i)
                                err_row = val_section.row()
                                err_row.label(text=f"  • {error.title}", icon="CANCEL")
                                # Show all message lines (not just first)
                                if error.message_lines:
                                    for msg_line in error.message_lines:
                                        if msg_line.strip():  # Skip empty lines
                                            msg_row = val_section.row()
                                            msg_row.label(text=f"    {msg_line}", icon="BLANK1")
                                if error.has_objects:
                                    obj_row = val_section.row()
                                    obj_row.label(
                                        text=f"    Affected: {error.objects_str}",
                                        icon="OBJECT_DATA",
                                    )
                                # Show all suggestion lines (not just first)
                                if error.has_suggestion:
                                    for sug_line in error.suggestion_lines:
                                        if sug_line.strip():  # Skip empty lines
                                            sug_row = val_section.row()
                                            sug_row.label(
                                                text=f"    → Fix: {sug_line}", icon="INFO"
                                            )

        # === EXPORT CONFIGURATION ===
        layout.separator()
        export_box = layout.box()
        export_box.label(text="Export Configuration", icon="EXPORT")
        export_box.prop(props, "export_format", expand=True)

        # XACRO specific settings
        if props.export_format == "XACRO":
            export_box.separator()
            row = export_box.row()
            row.prop(
                props,
                "xacro_advanced_mode",
                icon="TRIA_DOWN" if props.xacro_advanced_mode else "TRIA_RIGHT",
                icon_only=False,
                emboss=False,
            )

            if props.xacro_advanced_mode:
                adv_box = export_box.box()
                adv_box.prop(props, "xacro_extract_materials")
                adv_box.prop(props, "xacro_extract_dimensions")
                adv_box.prop(props, "xacro_generate_macros")
                adv_box.prop(props, "xacro_split_files")

        # ROS2 Control settings
        export_box.separator()
        export_box.prop(props, "use_ros2_control")

        # Mesh export options
        export_box.separator()
        export_box.prop(props, "export_meshes")
        if props.export_meshes:
            export_box.prop(props, "mesh_format")
            export_box.prop(props, "mesh_directory_name")

        # Validation option
        export_box.separator()
        export_box.prop(props, "validate_before_export")

        # === EXPORT BUTTON ===
        layout.separator()
        export_row = layout.row()
        export_row.scale_y = 1.5
        export_row.operator("linkforge.export_urdf", text="Export URDF/XACRO", icon="EXPORT")

        # === COMPONENT BROWSER (Quick select all components) ===
        layout.separator()

        # Collision Visibility Toggle
        row = layout.row()
        row.prop(props, "show_collisions", toggle=True, icon="SHADING_WIRE")

        layout.separator()
        layout.prop(
            props,
            "show_kinematic_tree",
            toggle=True,
            text="Component Browser",
            icon="VIEWZOOM",
            emboss=True,
        )

        if props.show_kinematic_tree:
            select_box = layout.box()

            # Links list
            link_header = select_box.row()
            link_header.label(text=f"Links ({num_links}):", icon="MESH_CUBE")

            for link_name in sorted(links_dict.keys()):
                link_obj = links_dict[link_name]
                row = select_box.row(align=True)
                op = row.operator(
                    "linkforge.select_tree_object", text=f"  {link_name}", emboss=False
                )
                op.object_name = link_obj.name
                op.object_type = "link"

            # Joints list
            if num_joints > 0:
                select_box.separator()
                joint_header = select_box.row()
                joint_header.label(text=f"Joints ({num_joints}):", icon="EMPTY_AXIS")

                # Collect all joints
                all_joints = []
                for obj in scene.objects:
                    if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
                        all_joints.append(
                            (obj.linkforge_joint.joint_name, obj.linkforge_joint.joint_type, obj)
                        )

                # Sort and display
                for joint_name, joint_type, joint_obj in sorted(all_joints):
                    row = select_box.row(align=True)
                    op = row.operator(
                        "linkforge.select_tree_object",
                        text=f"  {joint_name} ({joint_type.lower()})",
                        emboss=False,
                    )
                    op.object_name = joint_obj.name
                    op.object_type = "joint"

            # Sensors list (if any)
            if num_sensors > 0:
                select_box.separator()
                sensor_header = select_box.row()
                sensor_header.label(text=f"Sensors ({num_sensors}):", icon="OUTLINER_OB_CAMERA")

                # Collect all sensors
                all_sensors = []
                for obj in scene.objects:
                    if (
                        obj.type == "EMPTY"
                        and hasattr(obj, "linkforge_sensor")
                        and obj.linkforge_sensor.is_robot_sensor
                    ):
                        all_sensors.append(
                            (
                                obj.linkforge_sensor.sensor_name,
                                obj.linkforge_sensor.sensor_type,
                                obj,
                            )
                        )

                # Sort and display
                for sensor_name, sensor_type, sensor_obj in sorted(all_sensors):
                    row = select_box.row(align=True)
                    op = row.operator(
                        "linkforge.select_tree_object",
                        text=f"  {sensor_name} ({sensor_type.lower()})",
                        emboss=False,
                    )
                    op.object_name = sensor_obj.name
                    op.object_type = "sensor"

            # Transmissions list (if any)
            if num_transmissions > 0:
                select_box.separator()
                trans_header = select_box.row()
                trans_header.label(text=f"Transmissions ({num_transmissions}):", icon="DRIVER")

                # Collect all transmissions
                all_transmissions = []
                for obj in scene.objects:
                    if (
                        obj.type == "EMPTY"
                        and hasattr(obj, "linkforge_transmission")
                        and obj.linkforge_transmission.is_robot_transmission
                    ):
                        all_transmissions.append(
                            (
                                obj.linkforge_transmission.transmission_name,
                                obj.linkforge_transmission.transmission_type,
                                obj,
                            )
                        )

                # Sort and display
                for trans_name, trans_type, trans_obj in sorted(all_transmissions):
                    row = select_box.row(align=True)
                    op = row.operator(
                        "linkforge.select_tree_object",
                        text=f"  {trans_name} ({trans_type})",
                        emboss=False,
                    )
                    op.object_name = trans_obj.name
                    op.object_type = "transmission"


# Registration
classes = [
    LINKFORGE_OT_select_tree_object,
    LINKFORGE_PT_validate_export,
]


def register():
    """Register panel."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister():
    """Unregister panel."""
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass


if __name__ == "__main__":
    register()
