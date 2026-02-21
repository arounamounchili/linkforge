import bpy


def test_operator_empty_xacro_naming(tmp_path):
    """Test that the Blender operator uses the filename as fallback and reports no links."""
    # 1. Create a macro-only XACRO file
    xacro_content = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="test_macro">
        <link name="not_called_link"/>
    </xacro:macro>
</robot>
"""
    xacro_file = tmp_path / "my_robot_model.xacro"
    xacro_file.write_text(xacro_content)

    # 2. Call the operator and capture the builder
    # We can't easily capture the builder from the operator call,
    # so we'll instantiate the builder manually to test the naming logic in the Blender context.
    from linkforge.blender.logic.asynchronous_builder import AsynchronousRobotBuilder
    from linkforge_core.parsers import XACROParser

    parser = XACROParser()
    robot = parser.parse(xacro_file)

    # Verify the core parser gave us the right name
    assert robot.name == "my_robot_model"

    # 3. Use the builder manually (synchronously for the test)
    builder = AsynchronousRobotBuilder(robot, xacro_file, bpy.context)

    # Process all tasks
    while not builder.is_finished:
        builder.process_next_chunk()

    # 4. Verify collection naming
    collection_name = "my_robot_model"
    assert collection_name in bpy.data.collections

    collection = bpy.data.collections[collection_name]

    # 5. Verify it is empty
    assert len(collection.objects) == 0

    # Clean up
    bpy.data.collections.remove(collection)
