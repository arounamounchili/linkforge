#!/usr/bin/env python3
"""CLI script to run LinkForge Blender tests.

This script detects the Blender executable and runs the internal test runner.
"""

import os
import shutil
import subprocess
import sys


def find_blender():
    """Attempt to find the Blender executable path."""
    # Check if BLENDER_PATH environment variable is set
    env_path = os.environ.get("BLENDER_PATH")
    if env_path and os.path.exists(env_path):
        return env_path

    # Platform-specific defaults
    if sys.platform == "darwin":  # macOS
        standard_path = "/Applications/Blender.app/Contents/MacOS/Blender"
        if os.path.exists(standard_path):
            return standard_path
    elif sys.platform.startswith("linux"):
        # Try 'blender' in PATH
        path = shutil.which("blender")
        if path:
            return path
    elif sys.platform == "win32":
        standard_path = r"C:\Program Files\Blender Foundation\Blender\blender.exe"
        if os.path.exists(standard_path):
            return standard_path

    return None


def main():
    """Main entry point."""
    blender_path = find_blender()

    if not blender_path:
        print("Error: Blender executable not found.")
        print(
            "Please set the BLENDER_PATH environment variable or ensure Blender is in your /Applications folder."
        )
        sys.exit(1)

    print(f"Using Blender: {blender_path}")

    # Determine project root
    project_root = os.path.abspath(os.path.dirname(__file__))
    runner_script = os.path.join(project_root, "tests", "blender_test_runner.py")

    if not os.path.exists(runner_script):
        print(f"Error: Internal runner script not found at {runner_script}")
        sys.exit(1)

    # Construct the command
    command = [
        blender_path,
        "-b",  # Background mode
        "--python",
        runner_script,
    ]

    # Pass remaining arguments to the internal runner
    if len(sys.argv) > 1:
        command.append("--")
        command.extend(sys.argv[1:])

    # Run the process and propagate output
    try:
        process = subprocess.run(command, check=False)
        sys.exit(process.returncode)
    except Exception as e:
        print(f"Failed to execute Blender: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
