"""Tests for Blender property groups.

Tests property registration, default values, and basic functionality
for all LinkForge property groups.
"""

import bpy

from tests.blender_test_utils import (
    create_test_object,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
    safe_get_transmission,
)


class TestControlProperties:
    """Tests for ros2_control property groups."""

    def test_ros2_control_parameter_property(self, scene) -> None:
        """Test Ros2ControlParameterProperty creation and defaults."""
        from linkforge.blender.properties import control_props

        # Create a test scene property collection

        if not hasattr(scene, "test_params"):
            bpy.types.Scene.test_params = bpy.props.CollectionProperty(
                type=control_props.Ros2ControlParameterProperty
            )

        # Add a parameter
        param = scene.test_params.add()
        assert param is not None
        assert param.name == "param"
        assert param.value == "0.0"

        # Modify values
        param.name = "my_param"
        param.value = "1.5"
        assert param.name == "my_param"
        assert param.value == "1.5"

        # Cleanup
        scene.test_params.clear()
        del bpy.types.Scene.test_params

    def test_ros2_control_interface_property(self, scene) -> None:
        """Test Ros2ControlInterfaceProperty creation and defaults."""
        from linkforge.blender.properties import control_props

        if not hasattr(scene, "test_interfaces"):
            bpy.types.Scene.test_interfaces = bpy.props.CollectionProperty(
                type=control_props.Ros2ControlInterfaceProperty
            )

        # Add an interface
        interface = scene.test_interfaces.add()
        assert interface is not None
        assert interface.name == "position"

        # Test enum values
        interface.name = "velocity"
        assert interface.name == "velocity"

        interface.name = "effort"
        assert interface.name == "effort"

        # Test parameters collection
        param = interface.parameters.add()
        assert param is not None
        param.name = "test"
        assert len(interface.parameters) == 1

        # Cleanup
        scene.test_interfaces.clear()
        del bpy.types.Scene.test_interfaces

    def test_ros2_control_joint_property(self, scene) -> None:
        """Test Ros2ControlJointProperty creation and defaults."""
        from linkforge.blender.properties import control_props

        if not hasattr(scene, "test_joints"):
            bpy.types.Scene.test_joints = bpy.props.CollectionProperty(
                type=control_props.Ros2ControlJointProperty
            )

        # Add a joint
        joint = scene.test_joints.add()
        assert joint is not None
        assert joint.name == ""

        # Test command interface defaults
        assert joint.cmd_position is True
        assert joint.cmd_velocity is False
        assert joint.cmd_effort is False

        # Test state interface defaults
        assert joint.state_position is True
        assert joint.state_velocity is True
        assert joint.state_effort is False
        assert joint.show_parameters is False

        # Modify values
        joint.name = "joint1"
        joint.cmd_velocity = True
        joint.state_effort = True

        assert joint.name == "joint1"
        assert joint.cmd_velocity is True
        assert joint.state_effort is True

        # Test parameters
        param = joint.parameters.add()
        assert param is not None
        param.name = "param1"
        assert len(joint.parameters) == 1

        # Cleanup
        scene.test_joints.clear()
        del bpy.types.Scene.test_joints


class TestSensorProperties:
    """Tests for sensor property groups."""

    def test_sensor_property_defaults(self, clean_scene, scene) -> None:
        """Test SensorPropertyGroup default values."""
        # Create sensor object
        sensor_obj = create_test_object("test_sensor", None, scene)
        assert sensor_obj is not None

        sensor = safe_get_sensor(sensor_obj)

        # Test defaults
        assert sensor.is_robot_sensor is False
        assert sensor.sensor_type == "CAMERA"
        assert sensor.update_rate == 30.0
        assert sensor.always_on is False
        assert sensor.visualize is False

        # Test camera defaults
        assert sensor.camera_width == 640
        assert sensor.camera_height == 480
        assert sensor.camera_format == "R8G8B8"

        # Test LIDAR defaults
        assert sensor.lidar_horizontal_samples == 640
        assert sensor.lidar_vertical_samples == 1

        # Test noise defaults
        assert sensor.use_noise is False
        assert sensor.noise_type == "gaussian"
        assert sensor.noise_mean == 0.0
        assert sensor.noise_stddev == 0.0

        # Cleanup
        bpy.data.objects.remove(sensor_obj)

    def test_sensor_name_property(self, clean_scene, scene) -> None:
        """Test sensor name getter/setter."""
        sensor_obj = create_test_object("my_sensor", None, scene)
        assert sensor_obj is not None

        sensor = safe_get_sensor(sensor_obj)

        # Test getter
        assert sensor.sensor_name == "my_sensor"

        # Test setter
        sensor.sensor_name = "new_sensor_name"
        assert sensor_obj.name == "new_sensor_name"
        assert sensor.sensor_name == "new_sensor_name"

        # Cleanup
        bpy.data.objects.remove(sensor_obj)

    def test_sensor_types(self, clean_scene, scene) -> None:
        """Test all sensor types."""
        sensor_obj = create_test_object("test_sensor", None, scene)
        assert sensor_obj is not None

        sensor = safe_get_sensor(sensor_obj)

        # Test all sensor types
        sensor_types = ["CAMERA", "DEPTH_CAMERA", "LIDAR", "IMU", "GPS", "CONTACT", "FORCE_TORQUE"]
        for sensor_type in sensor_types:
            sensor.sensor_type = sensor_type
            assert sensor.sensor_type == sensor_type

        # Cleanup
        bpy.data.objects.remove(sensor_obj)


class TestRobotProperties:
    """Tests for robot property groups."""

    def test_robot_property_exists(self, clean_scene, scene) -> None:
        """Test robot property exists on scene."""

        # Robot properties should be registered by addon
        # Check for common property names
        assert (
            hasattr(scene, "linkforge")
            or hasattr(scene, "linkforge_robot")
            or hasattr(scene, "robot")
        )

    def test_robot_property_access(self, clean_scene, scene) -> None:
        """Test accessing robot properties."""

        # Verify we can access scene properties without error
        assert scene is not None
        assert safe_get_linkforge_scene(scene).show_ros2_control_parameters is True


class TestTransmissionProperties:
    """Tests for transmission property groups."""

    def test_transmission_property_defaults(self, clean_scene, scene) -> None:
        """Test transmission property defaults."""
        # Create transmission object
        trans_obj = create_test_object("test_transmission", None, scene)
        assert trans_obj is not None

        trans = safe_get_transmission(trans_obj)

        # Test defaults
        assert trans.is_robot_transmission is False
        if hasattr(trans, "transmission_type"):
            # Check actual enum values (SIMPLE, DIFFERENTIAL, etc.)
            assert trans.transmission_type in [
                "SIMPLE",
                "DIFFERENTIAL",
                "FOUR_BAR_LINKAGE",
                "CUSTOM",
            ]

        # Cleanup
        bpy.data.objects.remove(trans_obj)

    def test_transmission_types(self, clean_scene, scene) -> None:
        """Test transmission types."""
        # Create transmission object
        trans_obj = create_test_object("test_transmission", None, scene)
        assert trans_obj is not None

        trans = safe_get_transmission(trans_obj)

        # Test type switching if available (use actual enum values)
        if hasattr(trans, "transmission_type"):
            trans.transmission_type = "SIMPLE"
            assert trans.transmission_type == "SIMPLE"

            trans.transmission_type = "DIFFERENTIAL"
            assert trans.transmission_type == "DIFFERENTIAL"

        # Cleanup
        bpy.data.objects.remove(trans_obj)


class TestPropertyHelpers:
    """Tests for Blender property helper utilities."""

    def test_find_property_owner_strategies(self, scene) -> None:
        """Test find_property_owner with various strategies."""
        from linkforge.blender.utils.property_helpers import find_property_owner

        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.object.empty_add()
        obj = bpy.context.active_object
        assert obj is not None
        obj.name = "owner_obj"
        props = safe_get_linkforge(obj)
        props.is_robot_link = True

        # Strategy 1: id_data (should work immediately)
        found = find_property_owner(bpy.context, props, "linkforge")
        assert found == obj

        # Strategy 2: Active object
        # Strategy 1 is built-in to Blender PropertyGroups
        assert props.id_data == obj

        # Check with non-existent property
        assert find_property_owner(bpy.context, props, "invalid_attr") is None
