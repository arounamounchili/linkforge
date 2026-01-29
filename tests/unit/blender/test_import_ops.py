import bpy
import pytest
from linkforge.blender.operators.import_ops import working_directory


def test_working_directory_context_manager(tmp_path):
    """Test working_directory context manager changes and restores CWD."""
    import os

    original_cwd = os.getcwd()

    test_dir = tmp_path / "test_subdir"
    test_dir.mkdir()

    # Use context manager
    with working_directory(test_dir):
        assert os.getcwd() == str(test_dir)

    # Should restore original directory
    assert os.getcwd() == original_cwd


def test_working_directory_exception_handling(tmp_path):
    """Test working_directory restores CWD even on exception."""
    import os

    original_cwd = os.getcwd()

    test_dir = tmp_path / "test_subdir"
    test_dir.mkdir()

    # Raise exception inside context
    with pytest.raises(ValueError), working_directory(test_dir):
        assert os.getcwd() == str(test_dir)
        raise ValueError("Test exception")

    # Should still restore original directory
    assert os.getcwd() == original_cwd


def test_import_operator_registered():
    """Test that import operator is registered and accessible."""
    # The operator should be accessible via bpy.ops
    assert hasattr(bpy.ops, "linkforge")
    assert hasattr(bpy.ops.linkforge, "import_urdf")
