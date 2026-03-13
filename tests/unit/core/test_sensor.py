"""Tests for sensor models."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import (
    CameraInfo,
    GazeboPlugin,
    GPSInfo,
    IMUInfo,
    LidarInfo,
    Sensor,
    SensorNoise,
    SensorType,
    Transform,
    Vector3,
)


class TestSensorNoise:
    """Tests for SensorNoise model."""

    def test_default_noise(self):
        """Test default noise parameters."""
        noise = SensorNoise()
        assert noise.type == "gaussian"
        assert noise.mean == 0.0
        assert noise.stddev == 0.0

    def test_custom_noise(self):
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

    def test_default_camera(self):
        """Test default camera parameters."""
        camera = CameraInfo()
        assert camera.horizontal_fov == pytest.approx(1.047, rel=0.01)
        assert camera.width == 640
        assert camera.height == 480
        assert camera.near_clip == 0.1
        assert camera.far_clip == 100.0

    def test_custom_camera(self):
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

    def test_invalid_fov(self):
        """Test invalid field of view."""
        with pytest.raises(RobotModelError, match="Horizontal FOV must be between"):
            CameraInfo(horizontal_fov=-0.5)

        with pytest.raises(RobotModelError, match="Horizontal FOV must be between"):
            CameraInfo(horizontal_fov=3.2)

    def test_invalid_dimensions(self):
        """Test invalid image dimensions."""
        with pytest.raises(RobotModelError, match="Image dimensions must be positive"):
            CameraInfo(width=-10)

        with pytest.raises(RobotModelError, match="Image dimensions must be positive"):
            CameraInfo(height=0)

    def test_invalid_clip(self):
        """Test invalid clip planes."""
        with pytest.raises(RobotModelError, match="Near clip must be positive"):
            CameraInfo(near_clip=-0.1)

        with pytest.raises(RobotModelError, match="Far clip must be greater than near clip"):
            CameraInfo(near_clip=10.0, far_clip=5.0)


class TestLidarInfo:
    """Tests for LidarInfo model."""

    def test_default_lidar(self):
        """Test default 2D LIDAR parameters."""
        lidar = LidarInfo()
        assert lidar.horizontal_samples == 640
        assert lidar.horizontal_min_angle == pytest.approx(-1.570796, rel=0.01)
        assert lidar.horizontal_max_angle == pytest.approx(1.570796, rel=0.01)
        assert lidar.range_min == 0.1
        assert lidar.range_max == 10.0
        assert lidar.vertical_samples == 1

    def test_3d_lidar(self):
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

    def test_invalid_samples(self):
        """Test invalid sample count."""
        with pytest.raises(RobotModelError, match="Horizontal samples must be positive"):
            LidarInfo(horizontal_samples=0)

    def test_invalid_range(self):
        """Test invalid range parameters."""
        with pytest.raises(RobotModelError, match="Range min must be positive"):
            LidarInfo(range_min=-0.1)

        with pytest.raises(RobotModelError, match="Range max must be greater than range min"):
            LidarInfo(range_min=10.0, range_max=5.0)

    def test_invalid_angles(self):
        """Test invalid angle range."""
        with pytest.raises(
            RobotModelError, match="Horizontal min angle must be less than max angle"
        ):
            LidarInfo(horizontal_min_angle=1.0, horizontal_max_angle=-1.0)


class TestIMUInfo:
    """Tests for IMUInfo model."""

    def test_default_imu(self):
        """Test default IMU parameters."""
        imu = IMUInfo()
        assert imu.angular_velocity_noise is None
        assert imu.linear_acceleration_noise is None

    def test_imu_with_noise(self):
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

    def test_default_gps(self):
        """Test default GPS parameters."""
        gps = GPSInfo()
        assert gps.position_sensing_horizontal_noise is None

    def test_gps_with_noise(self):
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

    def test_plugin_creation(self):
        """Test creating a plugin."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
            parameters={"param1": "value1", "param2": "value2"},
        )
        assert plugin.name == "test_plugin"
        assert plugin.filename == "libtest.so"
        assert plugin.parameters["param1"] == "value1"

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(RobotModelError, match="Plugin name cannot be empty"):
            GazeboPlugin(name="", filename="libtest.so")

    def test_empty_filename(self):
        """Test that empty filename raises error."""
        with pytest.raises(RobotModelError, match="Plugin filename cannot be empty"):
            GazeboPlugin(name="test", filename="")


class TestSensor:
    """Tests for Sensor model."""

    def test_camera_sensor(self):
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
        assert sensor.camera_info.width == 1920

    def test_lidar_sensor(self):
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
        assert sensor.lidar_info.horizontal_samples == 1024

    def test_imu_sensor(self):
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

    def test_gps_sensor(self):
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

    def test_sensor_with_plugin(self):
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

    def test_sensor_with_transform(self):
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
        assert sensor.origin.xyz.x == pytest.approx(0.1)
        assert sensor.origin.xyz.z == pytest.approx(0.2)

    def test_camera_without_info(self):
        """Test that camera sensor requires camera_info."""
        with pytest.raises(RobotModelError, match="Camera sensor .* requires camera_info"):
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="camera_link",
            )

    def test_lidar_without_info(self):
        """Test that LIDAR sensor requires lidar_info."""
        with pytest.raises(RobotModelError, match="LIDAR sensor .* requires lidar_info"):
            Sensor(
                name="lidar",
                type=SensorType.LIDAR,
                link_name="lidar_link",
            )

    def test_imu_without_info(self):
        """Test that IMU sensor requires imu_info."""
        with pytest.raises(RobotModelError, match="IMU sensor .* requires imu_info"):
            Sensor(
                name="imu",
                type=SensorType.IMU,
                link_name="imu_link",
            )

    def test_gps_without_info(self):
        """Test that GPS sensor requires gps_info."""
        with pytest.raises(RobotModelError, match="GPS sensor .* requires gps_info"):
            Sensor(
                name="gps",
                type=SensorType.GPS,
                link_name="gps_link",
            )

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(RobotModelError, match="Sensor name cannot be empty"):
            Sensor(
                name="",
                type=SensorType.CAMERA,
                link_name="camera_link",
                camera_info=CameraInfo(),
            )

    def test_empty_link_name(self):
        """Test that empty link name raises error."""
        with pytest.raises(RobotModelError, match="Sensor must be attached to a link"):
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="",
                camera_info=CameraInfo(),
            )

    def test_invalid_update_rate(self):
        """Test that invalid update rate raises error."""
        with pytest.raises(RobotModelError, match="Update rate must be positive"):
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="camera_link",
                camera_info=CameraInfo(),
                update_rate=-10.0,
            )


def test_sensor_parsing_pose_robustness():
    """Verify that malformed or incomplete sensor pose elements are handled gracefully."""
    import xml.etree.ElementTree as ET

    from linkforge_core.parsers.urdf_parser import URDFParser

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
    assert sensor.origin is not None
    assert sensor.origin.xyz.x == 0
