"""Blender test runner — executed inside Blender's embedded Python.

Invoked automatically by blender_launcher.py. Do not run directly.
"""

import importlib.util
import os
import site
import subprocess
import sys


def setup_environment():
    """Setup paths and dependencies for the test run."""
    # Get the project root directory
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    # Add platforms/blender to sys.path
    blender_path = os.path.join(project_root, "platforms", "blender")
    if blender_path not in sys.path:
        sys.path.insert(0, blender_path)

    # Add core/src to sys.path (required for linkforge_core dependency)
    core_path = os.path.join(project_root, "core", "src")
    if core_path not in sys.path:
        sys.path.insert(0, core_path)

    # Ensure pytest is available in Blender's Python
    if importlib.util.find_spec("pytest") is None:
        print("pytest not found in Blender's Python. Attempting to install...")
        subprocess.check_call(
            [
                sys.executable,
                "-m",
                "pip",
                "install",
                "pytest",
                "pytest-cov",
                "pytest-mock",
                "PyYAML",
            ]
        )

        # Refresh site-packages
        from importlib import reload

        reload(site)

        # Manually add user site packages if not present
        user_site = site.getusersitepackages()
        if user_site not in sys.path:
            sys.path.append(user_site)


def run_tests():
    """Execute pytest within the Blender environment."""
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    test_dirs = [
        os.path.join(project_root, "tests", "unit", "platforms", "blender"),
        os.path.join(project_root, "tests", "integration", "platforms", "blender"),
    ]

    print(f"\nRunning Blender tests in: {', '.join(test_dirs)}")

    # We use a custom configuration to avoid conflicts with global pytest settings
    # and to ensure we only run the Blender-specific tests.
    # Parse pass-through arguments (after --)
    extra_args = []
    if "--" in sys.argv:
        idx = sys.argv.index("--")
        extra_args = sys.argv[idx + 1 :]

    # If the user provided a target (file/dir), use it. Otherwise use default test_dir.
    # Targets are arguments that don't start with '-'
    has_target = any(not a.startswith("-") for a in extra_args)

    args = []
    args.extend(["-v", "--tb=short"])

    # options first
    args.extend(extra_args)

    if not has_target:
        args.extend(test_dirs)

    import pytest

    exit_code = pytest.main(args)

    # Signal success/failure to the parent process
    sys.exit(exit_code)


if __name__ == "__main__":
    setup_environment()
    run_tests()
