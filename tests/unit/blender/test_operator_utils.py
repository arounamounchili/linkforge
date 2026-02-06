import os

from linkforge.blender.operators.import_ops import working_directory


def test_working_directory_context_manager(tmp_path):
    """Test that working_directory correctly changes and restores CWD."""
    original_cwd = os.getcwd()
    new_dir = tmp_path / "test_dir"
    new_dir.mkdir()

    with working_directory(new_dir):
        assert os.getcwd() == str(new_dir)

    assert os.getcwd() == original_cwd


def test_operator_registration_logic():
    """Test that register/unregister functions don't crash."""
    from linkforge.blender.operators.import_ops import register, unregister

    # These should be safe to call multiple times or in isolation
    register()
    unregister()
