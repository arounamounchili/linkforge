"""Tests for sensor models."""

from __future__ import annotations

import pytest
from linkforge.core import (
    CameraInfo,
    ContactInfo,
    ForceTorqueInfo,
    GazeboPlugin,
    GPSInfo,
    IMUInfo,
    LidarInfo,
    RobotModelError,
    Sensor,
    SensorNoise,
    SensorType,
    Transform,
    URDFParser,
    Vector3,
)


class TestContactInfo:
    """Tests for ContactInfo model."""

    def test_prefix(self) -> None:
        """Test creating a contact info with a prefix."""
        contact = ContactInfo(collision="c1")
        pre = contact.with_prefix("b_")
        assert pre.collision == "b_c1"


class TestSensorNoise:
    """Tests for SensorNoise model."""

    def test_default_noise(self) -> None:
        """Test default noise parameters."""
        noise = SensorNoise()
        assert noise.type == "gaussian"
        assert noise.mean == 0.0
        assert noise.stddev == 0.0

    def test_custom_noise(self) -> None:
        """Test custom noise parameters."""
        noise = SensorNoise(
            type="gaussian_quantized",
            mean=0.1,
            stddev=0.05,
            bias_mean=0.01,
            bias_stddev=0.001,
        )
        assert noise.type == "gaussian_quantized"
        assert noise.mean == 0.1
        assert noise.stddev == 0.05


class TestCameraInfo:
    """Tests for CameraInfo model."""

    def test_default_camera(self) -> None:
        """Test default camera parameters."""
        camera = CameraInfo()
        assert camera.horizontal_fov == pytest.approx(1.047, rel=0.01)
        assert camera.width == 640
        assert camera.height == 480
        assert camera.near_clip == 0.1
        assert camera.far_clip == 100.0

    def test_custom_camera(self) -> None:
        """Test custom camera parameters."""
        camera = CameraInfo(
            horizontal_fov=1.57,
            width=1920,
            height=1080,
            near_clip=0.05,
            far_clip=50.0,
        )
        assert camera.horizontal_fov == pytest.approx(1.57)
        assert camera.width == 1920
        assert camera.height == 1080

    def test_invalid_fov(self) -> None:
        """Test invalid field of view."""
        with pytest.raises(RobotModelError):
            CameraInfo(horizontal_fov=-0.5)

        with pytest.raises(RobotModelError):
            CameraInfo(horizontal_fov=3.2)

    def test_invalid_dimensions(self) -> None:
        """Test invalid image dimensions."""
        with pytest.raises(RobotModelError):
            CameraInfo(width=-10)

        with pytest.raises(RobotModelError):
            CameraInfo(height=0)

    def test_invalid_clip(self) -> None:
        """Test invalid clip planes."""
        with pytest.raises(RobotModelError):
            CameraInfo(near_clip=-0.1)

        with pytest.raises(RobotModelError):
            CameraInfo(near_clip=10.0, far_clip=5.0)


class TestLidarInfo:
    """Tests for LidarInfo model."""

    def test_default_lidar(self) -> None:
        """Test default 2D LIDAR parameters."""
        lidar = LidarInfo()
        assert lidar.horizontal_samples == 640
        assert lidar.horizontal_min_angle == pytest.approx(-1.570796, rel=0.01)
        assert lidar.horizontal_max_angle == pytest.approx(1.570796, rel=0.01)
        assert lidar.range_min == 0.1
        assert lidar.range_max == 10.0
        assert lidar.vertical_samples == 1

    def test_3d_lidar(self) -> None:
        """Test 3D LIDAR parameters."""
        lidar = LidarInfo(
            horizontal_samples=1024,
            vertical_samples=64,
            vertical_min_angle=-0.2617,  # -15 degrees
            vertical_max_angle=0.2617,  # +15 degrees
            range_max=100.0,
        )
        assert lidar.horizontal_samples == 1024
        assert lidar.vertical_samples == 64
        assert lidar.range_max == 100.0

    def test_invalid_samples(self) -> None:
        """Test invalid sample count."""
        with pytest.raises(RobotModelError):
            LidarInfo(horizontal_samples=0)

    def test_invalid_range(self) -> None:
        """Test invalid range parameters."""
        with pytest.raises(RobotModelError):
            LidarInfo(range_min=-0.1)

        with pytest.raises(RobotModelError):
            LidarInfo(range_min=10.0, range_max=5.0)

    def test_invalid_angles(self) -> None:
        """Test invalid angle range."""
        with pytest.raises(RobotModelError):
            LidarInfo(horizontal_min_angle=1.0, horizontal_max_angle=-1.0)

    def test_invalid_range_resolution(self) -> None:
        """Test invalid range resolution."""
        with pytest.raises(RobotModelError):
            LidarInfo(range_resolution=0.0)

    def test_invalid_vertical_angle_range(self) -> None:
        """Test invalid vertical angle range for 3D scans."""
        with pytest.raises(RobotModelError):
            LidarInfo(vertical_samples=2, vertical_min_angle=1.0, vertical_max_angle=-1.0)


class TestIMUInfo:
    """Tests for IMUInfo model."""

    def test_default_imu(self) -> None:
        """Test default IMU parameters."""
        imu = IMUInfo()
        assert imu.angular_velocity_noise is None
        assert imu.linear_acceleration_noise is None

    def test_imu_with_noise(self) -> None:
        """Test IMU with noise models."""
        noise = SensorNoise(stddev=0.01)
        imu = IMUInfo(
            angular_velocity_noise=noise,
            linear_acceleration_noise=noise,
        )
        assert imu.angular_velocity_noise is not None
        assert imu.linear_acceleration_noise is not None


class TestGPSInfo:
    """Tests for GPSInfo model."""

    def test_default_gps(self) -> None:
        """Test default GPS parameters."""
        gps = GPSInfo()
        assert gps.position_sensing_horizontal_noise is None

    def test_gps_with_noise(self) -> None:
        """Test GPS with noise models."""
        pos_noise = SensorNoise(stddev=0.5)
        vel_noise = SensorNoise(stddev=0.1)
        gps = GPSInfo(
            position_sensing_horizontal_noise=pos_noise,
            velocity_sensing_horizontal_noise=vel_noise,
        )
        assert gps.position_sensing_horizontal_noise is not None
        assert gps.velocity_sensing_horizontal_noise is not None


class TestGazeboPlugin:
    """Tests for GazeboPlugin model."""

    def test_plugin_creation(self) -> None:
        """Test creating a plugin."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
            parameters={"param1": "value1", "param2": "value2"},
        )
        assert plugin.name == "test_plugin"
        assert plugin.filename == "libtest.so"
        assert plugin.parameters["param1"] == "value1"

    def test_empty_name(self) -> None:
        """Test that empty name raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            GazeboPlugin(name="", filename="libtest.so")

    def test_empty_filename(self) -> None:
        """Test that empty filename raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            GazeboPlugin(name="test", filename="")


class TestSensor:
    """Tests for Sensor model."""

    def test_camera_sensor(self) -> None:
        """Test creating a camera sensor."""
        camera_info = CameraInfo(width=1920, height=1080)
        sensor = Sensor(
            name="front_camera",
            type=SensorType.CAMERA,
            link_name="camera_link",
            camera_info=camera_info,
        )
        assert sensor.name == "front_camera"
        assert sensor.type == SensorType.CAMERA
        assert sensor.link_name == "camera_link"
        assert sensor.camera_info is not None
        assert sensor.camera_info.width == 1920

    def test_lidar_sensor(self) -> None:
        """Test creating a LIDAR sensor."""
        lidar_info = LidarInfo(horizontal_samples=1024)
        sensor = Sensor(
            name="lidar",
            type=SensorType.LIDAR,
            link_name="lidar_link",
            lidar_info=lidar_info,
        )
        assert sensor.name == "lidar"
        assert sensor.type == SensorType.LIDAR
        assert sensor.lidar_info is not None
        assert sensor.lidar_info.horizontal_samples == 1024

    def test_imu_sensor(self) -> None:
        """Test creating an IMU sensor."""
        imu_info = IMUInfo()
        sensor = Sensor(
            name="imu",
            type=SensorType.IMU,
            link_name="imu_link",
            imu_info=imu_info,
            update_rate=100.0,
        )
        assert sensor.name == "imu"
        assert sensor.type == SensorType.IMU
        assert sensor.update_rate == 100.0

    def test_gps_sensor(self) -> None:
        """Test creating a GPS sensor."""
        gps_info = GPSInfo()
        sensor = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="gps_link",
            gps_info=gps_info,
        )
        assert sensor.name == "gps"
        assert sensor.type == SensorType.GPS
        assert sensor.gps_info is not None

    def test_contact_sensor(self) -> None:
        """Test creating a contact sensor."""
        contact_info = ContactInfo(collision="my_link_collision")
        sensor = Sensor(
            name="contact",
            type=SensorType.CONTACT,
            link_name="link1",
            contact_info=contact_info,
        )
        assert sensor.name == "contact"
        assert sensor.contact_info is not None
        assert sensor.contact_info.collision == "my_link_collision"

    def test_force_torque_sensor(self) -> None:
        """Test creating a force/torque sensor."""
        ft_info = ForceTorqueInfo(frame="parent", measure_direction="parent_to_child")
        sensor = Sensor(
            name="ft_sensor",
            type=SensorType.FORCE_TORQUE,
            link_name="joint1",
            force_torque_info=ft_info,
        )
        assert sensor.name == "ft_sensor"
        assert sensor.force_torque_info is not None
        assert sensor.force_torque_info.frame == "parent"

    def test_force_torque_invalid_params(self) -> None:
        """Test invalid F/T parameters."""
        with pytest.raises(RobotModelError, match="Invalid F/T frame"):
            ForceTorqueInfo(frame="invalid_frame")

        with pytest.raises(RobotModelError, match="Invalid F/T direction"):
            ForceTorqueInfo(measure_direction="invalid_dir")
        """Test sensor with plugin."""
        camera_info = CameraInfo()
        plugin = GazeboPlugin(name="camera_plugin", filename="libgazebo_ros_camera.so")
        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",
            camera_info=camera_info,
            plugin=plugin,
            topic="camera/image_raw",
        )
        assert sensor.plugin is not None
        assert sensor.plugin.name == "camera_plugin"
        assert sensor.topic == "camera/image_raw"

    def test_sensor_with_transform(self) -> None:
        """Test sensor with custom transform."""
        camera_info = CameraInfo()
        transform = Transform(xyz=Vector3(0.1, 0.0, 0.2))
        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",
            camera_info=camera_info,
            origin=transform,
        )
        assert sensor.origin is not None
        assert sensor.origin.xyz.x == pytest.approx(0.1)
        assert sensor.origin.xyz.z == pytest.approx(0.2)

    def test_camera_without_info(self) -> None:
        """Test that camera sensor requires camera_info."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="camera_link",
            )

    def test_lidar_without_info(self) -> None:
        """Test that LIDAR sensor requires lidar_info."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="lidar",
                type=SensorType.LIDAR,
                link_name="lidar_link",
            )

    def test_imu_without_info(self) -> None:
        """Test that IMU sensor requires imu_info."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="imu",
                type=SensorType.IMU,
                link_name="imu_link",
            )

    def test_gps_without_info(self) -> None:
        """Test that GPS sensor requires gps_info."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="gps",
                type=SensorType.GPS,
                link_name="gps_link",
            )

    def test_contact_without_info(self) -> None:
        """Test that contact sensor requires contact_info."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="contact",
                type=SensorType.CONTACT,
                link_name="link1",
            )

    def test_force_torque_without_info(self) -> None:
        """Test that force_torque sensor requires force_torque_info."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="ft",
                type=SensorType.FORCE_TORQUE,
                link_name="joint1",
            )

    def test_empty_name(self) -> None:
        """Test that empty name raises error."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="",
                type=SensorType.CAMERA,
                link_name="camera_link",
                camera_info=CameraInfo(),
            )

    def test_empty_link_name(self) -> None:
        """Test that empty link name raises error."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="",
                camera_info=CameraInfo(),
            )

    def test_invalid_update_rate(self) -> None:
        """Test that invalid update rate raises error."""
        with pytest.raises(RobotModelError):
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="camera_link",
                camera_info=CameraInfo(),
                update_rate=-10.0,
            )

    def test_prefix(self) -> None:
        """Test creating a sensor with a prefix."""
        contact = ContactInfo(collision="c1")
        plugin = GazeboPlugin(name="p1", filename="f1")
        sensor = Sensor(
            name="s1",
            type=SensorType.CONTACT,
            link_name="l1",
            topic="/t1",
            contact_info=contact,
            plugin=plugin,
        )

        pre = sensor.with_prefix("b_")
        assert pre.name == "b_s1"
        assert pre.link_name == "b_l1"
        assert pre.topic == "b_/t1"

        contact_info = pre.contact_info
        assert contact_info is not None
        assert contact_info.collision == "b_c1"

        plugin = pre.plugin
        assert plugin is not None
        assert plugin.name == "b_p1"


def test_sensor_parsing_pose_robustness() -> None:
    """Verify that malformed or incomplete sensor pose elements are handled gracefully."""
    import xml.etree.ElementTree as ET

    parser = URDFParser()
    xml = """
    <gazebo reference="link1">
        <sensor name="cam" type="camera">
            <pose>0 0</pose> <!-- Invalid format: not 6 floats -->
            <camera><image><width>640</width></image></camera>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml)
    sensor = parser._parse_sensor_from_gazebo(elem)
    # Pose should revert to identity or skip if invalid
    assert sensor is not None
    assert sensor.origin is not None
    assert sensor.origin.xyz.x == 0
