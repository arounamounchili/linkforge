#!/usr/bin/env python3
"""Entry point for running Blender tests from the command line.

Finds the Blender executable and spawns it in headless mode, delegating
actual test execution to tests/blender_test_runner.py which runs inside
Blender's embedded Python interpreter.

Usage:
    python blender_launcher.py
    python blender_launcher.py -- --cov=linkforge
    BLENDER_PATH=/custom/blender python blender_launcher.py
"""

import os
import shutil
import subprocess
import sys


def find_blender() -> str | None:
    """Attempt to find the Blender executable path."""
    env_path = os.environ.get("BLENDER_PATH")
    if env_path and os.path.exists(env_path):
        return env_path

    if sys.platform == "darwin":
        standard_path = "/Applications/Blender.app/Contents/MacOS/Blender"
        if os.path.exists(standard_path):
            return standard_path
    elif sys.platform.startswith("linux"):
        path = shutil.which("blender")
        if path:
            return path
    elif sys.platform == "win32":
        standard_path = r"C:\Program Files\Blender Foundation\Blender\blender.exe"
        if os.path.exists(standard_path):
            return standard_path

    return None


def main() -> None:
    """Find Blender and delegate test execution to tests/blender_test_runner.py."""
    blender_path = find_blender()

    if not blender_path:
        print("Error: Blender executable not found.")
        print(
            "Set the BLENDER_PATH environment variable or install Blender at its default location."
        )
        sys.exit(1)

    print(f"Using Blender: {blender_path}")

    project_root = os.path.abspath(os.path.dirname(__file__))
    runner_script = os.path.join(project_root, "tests", "blender_test_runner.py")

    if not os.path.exists(runner_script):
        print(f"Error: Internal runner script not found at {runner_script}")
        sys.exit(1)

    # Invoke Blender in background (-b) and pass the runner as its Python script.
    # Everything after '--' is forwarded to the runner as extra pytest arguments.
    command = [blender_path, "-b", "--python", runner_script]
    if len(sys.argv) > 1:
        command.append("--")
        command.extend(sys.argv[1:])

    try:
        process = subprocess.run(command, check=False)
        sys.exit(process.returncode)
    except Exception as e:
        print(f"Failed to execute Blender: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
