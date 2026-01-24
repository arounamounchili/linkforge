#!/usr/bin/env python3
"""Build script for packaging LinkForge as a Blender Extension.

This script manages dependencies, updates the manifest, and creates a .zip package.
1. sync: Automatically downloads wheels for target platforms/versions.
2. build: Packages the extension into a .zip file.

Usage:
    python3 build_extension.py sync   # Update dependencies and manifest
    python3 build_extension.py build  # Create the extension package
    python3 build_extension.py clean  # Remove build artifacts
"""

from __future__ import annotations

import fnmatch
import re
import shutil
import subprocess
import sys
from pathlib import Path

# Use tomllib (Py3.11+) or fall back to string parsing for manifest metadata
try:
    import tomllib
except ImportError:
    tomllib = None

# --- Dependency Configuration ---
# Packages to bundle as wheels for cross-platform/cross-version compatibility
DEP_CONFIG = {
    "PyYAML": {
        "version": "6.0.3",
        "platforms": [
            "win_amd64",
            "manylinux2014_x86_64",
            "macosx_11_0_arm64",
            "macosx_11_0_x86_64",
        ],
        "py_versions": ["311"],  # Standardize on 3.11+ for modern Blender versions
    },
    "xacrodoc": {"version": "1.3.1", "universal": True},
    "rospkg": {"version": "1.6.1", "universal": True},
    "docutils": {"version": "0.22.4", "universal": True},
}


def read_manifest_value(key: str) -> str:
    """Read a value from blender_manifest.toml."""
    manifest_path = Path(__file__).parent / "blender_manifest.toml"
    content = manifest_path.read_text()
    match = re.search(f'^{key}\\s*=\\s*"([^"]+)"', content, re.MULTILINE)
    if match:
        return match.group(1)
    return "0.0.0"


def sync_dependencies():
    """Download required wheels and update the manifest."""
    root_dir = Path(__file__).parent
    wheels_dir = root_dir / "wheels"

    print("🔄 Syncing dependencies...")

    # Clear existing wheels to ensure a clean slate and avoid duplicates
    if wheels_dir.exists():
        print(f"  Cleaning existing wheels in {wheels_dir.name}/...")
        shutil.rmtree(wheels_dir)
    wheels_dir.mkdir(exist_ok=True)

    for pkg, config in DEP_CONFIG.items():
        print(f"  Fetching {pkg}...")
        version = config.get("version")
        req = f"{pkg}=={version}" if version else pkg

        if config.get("universal"):
            # Download pure-python universal wheel
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pip",
                    "download",
                    req,
                    "--no-deps",
                    "--dest",
                    str(wheels_dir),
                ],
                check=True,
            )
        else:
            # Download platform/version specific wheels
            for platform in config["platforms"]:
                for py_ver in config["py_versions"]:
                    subprocess.run(
                        [
                            sys.executable,
                            "-m",
                            "pip",
                            "download",
                            req,
                            "--no-deps",
                            "--only-binary=:all:",
                            "--platform",
                            platform,
                            "--python-version",
                            py_ver,
                            "--dest",
                            str(wheels_dir),
                        ],
                        check=True,
                    )

    update_manifest_wheels()


def update_manifest_wheels():
    """Scan wheels/ directory and update blender_manifest.toml."""
    root_dir = Path(__file__).parent
    manifest_path = root_dir / "blender_manifest.toml"
    wheels_dir = root_dir / "wheels"

    if not manifest_path.exists():
        print("❌ Error: blender_manifest.toml not found")
        return

    # Collect all wheel paths relative to project root
    wheel_paths = []
    if wheels_dir.exists():
        for whl in sorted(wheels_dir.glob("*.whl")):
            wheel_paths.append(f'    "./wheels/{whl.name}",')

    # Update manifest file using markers
    content = manifest_path.read_text()
    marker_start = "# BEGIN AUTOMATED WHEELS"
    marker_end = "# END AUTOMATED WHEELS"

    if marker_start not in content or marker_end not in content:
        print("⚠️  Warning: Automation markers not found in manifest. Skipping auto-update.")
        return

    new_section = f"{marker_start}\nwheels = [\n" + "\n".join(wheel_paths) + f"\n]\n{marker_end}"
    pattern = re.escape(marker_start) + r".*?" + re.escape(marker_end)
    new_content = re.sub(pattern, new_section, content, flags=re.DOTALL)

    manifest_path.write_text(new_content)
    print("✅ Updated blender_manifest.toml with bundled wheels")


def is_excluded(path: Path, root: Path, patterns: list[str]) -> bool:
    """Check if a path matches any exclusion patterns."""
    rel_path = str(path.relative_to(root))
    for pattern in patterns:
        if fnmatch.fnmatch(rel_path, pattern) or fnmatch.fnmatch(path.name, pattern):
            return True
        # Check if any parent directory matches
        for parent in path.parents:
            if parent == root:
                break
            if fnmatch.fnmatch(str(parent.relative_to(root)), pattern.rstrip("/")):
                return True
    return False


def build_extension() -> Path:
    """Build the Blender Extension package using official Blender CLI."""
    root_dir = Path(__file__).parent
    version = read_manifest_value("version")
    dist_dir = root_dir / "dist"
    dist_dir.mkdir(exist_ok=True)

    # Create a staging directory for the flattened structure
    # Blender Extension expects __init__.py at the root for "add-on" type
    staging_dir = dist_dir / "staging"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True)

    print(f"📦 Staging LinkForge Extension v{version} for build...")

    # 1. Copy manifest
    shutil.copy(root_dir / "blender_manifest.toml", staging_dir)

    # 2. Copy source code (Flattened)
    # We copy the contents of linkforge/ so __init__.py is at the root of staging_dir
    src_dir = root_dir / "linkforge"
    for item in src_dir.iterdir():
        if item.name.startswith((".", "__pycache__")):
            continue
        dest = staging_dir / item.name
        if item.is_dir():
            shutil.copytree(item, dest)
        else:
            shutil.copy2(item, dest)

    # 3. Copy wheels
    shutil.copytree(root_dir / "wheels", staging_dir / "wheels")

    # 4. Copy license/readme
    for f in ["LICENSE", "README.md", "THIRD_PARTY_LICENSES.md"]:
        if (root_dir / f).exists():
            shutil.copy2(root_dir / f, staging_dir)

    print("🚀 Building split-platform packages...")

    # Find Blender CLI
    # 1. Check environment variable
    # 2. Check standard 'blender' command in PATH
    # 3. Fallback to common Mac path for local dev
    import os

    blender_path = os.environ.get("BLENDER_PATH", "blender")

    # Verify if blender is accessible
    if not shutil.which(blender_path):
        # Specific fallback for Mac local development
        mac_fallback = "/Applications/Blender.app/Contents/MacOS/Blender"
        if Path(mac_fallback).exists():
            blender_path = mac_fallback
        else:
            print(f"❌ Error: Blender command '{blender_path}' not found in PATH.")
            print("Please install Blender or set the BLENDER_PATH environment variable.")
            sys.exit(1)

    try:
        subprocess.run(
            [
                blender_path,
                "--command",
                "extension",
                "build",
                "--split-platforms",
                "--output-dir",
                str(dist_dir),
            ],
            check=True,
            cwd=str(staging_dir),
        )
    except subprocess.CalledProcessError as e:
        print(f"❌ Error building extension: {e}")
        # Clean up staging on failure
        # shutil.rmtree(staging_dir)
        sys.exit(1)

    # Clean up staging on success
    shutil.rmtree(staging_dir)

    print(f"\n✅ Created split-platform packages in {dist_dir}/")
    return dist_dir


def clean():
    """Clean build artifacts."""
    root_dir = Path(__file__).parent
    for d in ["dist", "wheels"]:
        path = root_dir / d
        if path.exists():
            shutil.rmtree(path)
            print(f"🗑️  Removed {d}/")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "build"

    if cmd == "sync":
        sync_dependencies()
    elif cmd == "build":
        # Auto-update manifest before build if markers exist
        update_manifest_wheels()
        build_extension()
    elif cmd == "clean":
        clean()
    else:
        print(f"Unknown command: {cmd}")
        print("Usage: python3 build_extension.py [sync|build|clean]")
