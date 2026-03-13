def test_operator_registration_logic() -> None:
    """Test that register/unregister functions don't crash."""
    from linkforge.blender.operators.import_ops import register, unregister

    # These should be safe to call multiple times or in isolation
    register()
    unregister()
