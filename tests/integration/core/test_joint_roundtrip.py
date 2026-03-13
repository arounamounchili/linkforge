from linkforge_core.generators.urdf_generator import URDFGenerator
from linkforge_core.parsers.urdf_parser import URDFParser


def test_round_trip_safety_calibration() -> None:
    # 1. Start with URDF XML
    original_xml = """<?xml version='1.0' encoding='utf-8'?>
<robot name="round_trip_bot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="10.0" lower="-1.57" upper="1.57" velocity="5.0"/>
    <safety_controller k_position="15.0" k_velocity="10.0" soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
    <calibration falling="1.0" rising="0.5"/>
  </joint>
</robot>"""

    # 2. Parse into Robot model
    parser = URDFParser()
    robot = parser.parse_string(original_xml)

    # Verify model content
    joint = robot.get_joint("joint1")
    assert joint.safety_controller.k_position == 15.0
    assert joint.calibration.rising == 0.5

    # 3. Generate back to XML
    generator = URDFGenerator(pretty_print=True)
    generated_xml = generator.generate(robot)

    # 4. Re-parse and verify
    robot2 = parser.parse_string(generated_xml)
    joint2 = robot2.get_joint("joint1")

    assert joint2.safety_controller.k_position == 15.0
    assert joint2.safety_controller.soft_lower_limit == -1.5
    assert joint2.calibration.rising == 0.5
    assert joint2.calibration.falling == 1.0

    print("Round-trip verification successful!")


if __name__ == "__main__":
    test_round_trip_safety_calibration()
