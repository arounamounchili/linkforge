"""Unit tests for the Translation Registry and specialized translators."""

from linkforge.blender.adapters.translator import ITranslator, TranslationRegistry


class MockTranslator:
    """A minimal mock translator for testing the registry."""

    def translate(self, *args, **kwargs):
        return "translated"


def test_translation_registry_lifecycle():
    """Verify that translators can be registered and retrieved correctly."""
    registry = TranslationRegistry()
    mock_trans = MockTranslator()

    # 1. Initial state
    assert registry.get("link") is None

    # 2. Registration
    registry.register("link", mock_trans)
    assert registry.get("link") == mock_trans

    # 3. Multiple registrations
    mock_joint = MockTranslator()
    registry.register("joint", mock_joint)
    assert registry.get("joint") == mock_joint
    assert registry.get("link") == mock_trans

    # 4. Overwriting
    new_mock = MockTranslator()
    registry.register("link", new_mock)
    assert registry.get("link") == new_mock


def test_translator_protocol_compliance():
    """Verify that our core translators comply with the ITranslator protocol."""
    from linkforge.blender.adapters.translator import (
        JointTranslator,
        LinkTranslator,
        Ros2ControlTranslator,
        SensorTranslator,
        TransmissionTranslator,
    )

    translators = [
        LinkTranslator(),
        JointTranslator(),
        SensorTranslator(),
        TransmissionTranslator(),
        Ros2ControlTranslator(),
    ]

    for t in translators:
        assert isinstance(t, ITranslator)
        # Verify it has the translate method
        assert hasattr(t, "translate")
        assert callable(t.translate)
