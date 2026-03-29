#!/usr/bin/env python3
"""Build script for packaging LinkForge as a Blender Extension.

This script manages dependencies, updates the manifest, and creates a .zip package.
1. sync: Automatically downloads wheels for target platforms/versions.
2. build: Packages the extension into a .zip file.

Usage:
    python3 platforms/blender/scripts/build.py build  # Create the extension package
    python3 platforms/blender/scripts/build.py clean  # Remove build artifacts
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import typing
from pathlib import Path

# Use tomllib (Py3.11+) or fall back to string parsing for manifest metadata
try:
    import tomllib
except ImportError:
    tomllib = None  # type: ignore[assignment]

# --- Configuration ---
REPO_ROOT = Path(__file__).resolve().parents[3]  # platforms/blender/scripts/build.py -> root
PLATFORM_DIR = REPO_ROOT / "platforms" / "blender"
SOURCE_DIR = PLATFORM_DIR / "linkforge"
CORE_DIR = REPO_ROOT / "core" / "src" / "linkforge_core"
MANIFEST_PATH = PLATFORM_DIR / "blender_manifest.toml"
WHEELS_DIR = PLATFORM_DIR / "wheels"
DIST_DIR = REPO_ROOT / "dist"  # Keep dist in root for easy access

# Packages to bundle as wheels for cross-platform/cross-version compatibility
DEP_CONFIG: dict[str, dict[str, typing.Any]] = {
    "PyYAML": {
        "version": "6.0.3",
        "universal": False,
        "platforms": [
            "win_amd64",
            "macosx_11_0_arm64",
            "macosx_10_13_x86_64",
            "manylinux2014_x86_64",
        ],
        "py_versions": ["311", "312", "313"],  # Blender 4.2, 5.0, 5.1+
    }
}


def read_manifest_value(key: str) -> str:
    """Read a value from blender_manifest.toml."""
    if not MANIFEST_PATH.exists():
        return "0.0.0"

    content = MANIFEST_PATH.read_text()
    match = re.search(f'^{key}\\s*=\\s*"([^"]+)"', content, re.MULTILINE)
    if match:
        return match.group(1)
    return "0.0.0"


def sync_dependencies() -> None:
    """Download required wheels and update the manifest."""
    print("🔄 Syncing dependencies...")

    # Clear existing wheels to ensure a clean slate and avoid duplicates
    if WHEELS_DIR.exists():
        print("  Cleaning existing wheels in platforms/blender/wheels/...")
        shutil.rmtree(WHEELS_DIR)

    if not DEP_CONFIG:
        print("  Zero dependencies configured. Nothing to sync.")
        return

    WHEELS_DIR.mkdir(exist_ok=True, parents=True)

    for pkg, config in DEP_CONFIG.items():
        print(f"  Fetching {pkg}...")
        version = config.get("version")
        req = f"{pkg}=={version}" if version else pkg

        if config.get("universal"):
            # Download pure-python universal wheel
            subprocess.run(
                [
                    "uv",
                    "run",
                    "--with",
                    "pip",
                    "pip",
                    "download",
                    req,
                    "--no-deps",
                    "--dest",
                    str(WHEELS_DIR),
                ],
                check=True,
            )
        else:
            # Download platform/version specific wheels
            for platform in config["platforms"]:
                for py_ver in config["py_versions"]:
                    subprocess.run(
                        [
                            "uv",
                            "run",
                            "--with",
                            "pip",
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
                            str(WHEELS_DIR),
                        ],
                        check=True,
                    )

    update_manifest_wheels()


def update_manifest_wheels() -> None:
    """Scan wheels/ directory and update blender_manifest.toml."""
    if not MANIFEST_PATH.exists():
        print(f"❌ Error: {MANIFEST_PATH} not found")
        return

    # Collect all wheel paths relative to the platform root (where manifest lives)
    wheel_paths = []
    if WHEELS_DIR.exists():
        for whl in sorted(WHEELS_DIR.glob("*.whl")):
            # In the final zip, wheels will be in ./wheels/
            wheel_paths.append(f'    "./wheels/{whl.name}",')

    # Update manifest file using markers
    content = MANIFEST_PATH.read_text()
    marker_start = "# BEGIN AUTOMATED WHEELS"
    marker_end = "# END AUTOMATED WHEELS"

    if marker_start not in content or marker_end not in content:
        print("⚠️  Warning: Automation markers not found in manifest. Skipping auto-update.")
        return

    new_section = f"{marker_start}\nwheels = [\n" + "\n".join(wheel_paths) + f"\n]\n{marker_end}"
    pattern = re.escape(marker_start) + r".*?" + re.escape(marker_end)
    new_content = re.sub(pattern, new_section, content, flags=re.DOTALL)

    MANIFEST_PATH.write_text(new_content)
    print("✅ Updated blender_manifest.toml with bundled wheels")


def build_extension() -> Path:
    """Build the Blender Extension package using official Blender CLI."""
    version = read_manifest_value("version")
    DIST_DIR.mkdir(exist_ok=True)

    # Create a staging directory for the flattened structure
    staging_dir = DIST_DIR / "staging"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True)

    print(f"📦 Staging LinkForge Extension v{version} for build...")

    # 1. Copy manifest
    shutil.copy(MANIFEST_PATH, staging_dir)

    # 2. Copy source code (Extension)
    # Copy contents of platforms/blender/linkforge/ so __init__.py is at root
    for item in SOURCE_DIR.iterdir():
        if item.name.startswith((".", "__pycache__")) or item.name == "linkforge_core":
            continue
        dest = staging_dir / item.name
        if item.is_dir():
            shutil.copytree(item, dest)
        else:
            shutil.copy2(item, dest)

    # 3. Copy Core Library (linkforge_core)
    # Bundle it inside the linkforge package for policy compliance and reliable imports
    if not CORE_DIR.exists():
        print(f"❌ Error: Core directory {CORE_DIR} not found.")
        sys.exit(1)

    shutil.copytree(CORE_DIR, staging_dir / "linkforge_core")
    print(f"  Bundled linkforge_core -> {staging_dir / 'linkforge_core'}")

    # 3. Copy dependencies (if any)
    if WHEELS_DIR.exists() and any(WHEELS_DIR.iterdir()):
        shutil.copytree(WHEELS_DIR, staging_dir / "wheels")
        print(f"  Bundled dependencies -> {staging_dir / 'wheels'}")

    # 4. Copy license/readme
    for f in ["LICENSE", "README.md"]:
        if (REPO_ROOT / f).exists():
            shutil.copy2(REPO_ROOT / f, staging_dir)

    print("🚀 Building split-platform packages...")

    # Find Blender CLI
    import os

    blender_path = os.environ.get("BLENDER_PATH", "blender")

    if not shutil.which(blender_path):
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
                str(DIST_DIR),
            ],
            check=True,
            cwd=str(staging_dir),
        )
    except subprocess.CalledProcessError as e:
        print(f"❌ Error building extension: {e}")
        sys.exit(1)

    # Clean up staging on success
    shutil.rmtree(staging_dir)

    # 5. Rename packages for platform clarity (LinkForge Multi-Platform Vision)
    # This distinguishes 'linkforge-blender' from future 'linkforge-freecad', etc.
    print("✨ Renaming packages for platform clarity...")
    extension_id = read_manifest_value("id")
    for zip_file in DIST_DIR.glob(f"{extension_id}*.zip"):
        if "-blender" not in zip_file.name:
            new_name = zip_file.name.replace(extension_id, f"{extension_id}-blender")
            zip_file.rename(DIST_DIR / new_name)
            print(f"  {zip_file.name} -> {new_name}")

    print(f"\n✅ Created split-platform packages in {DIST_DIR}/")
    return DIST_DIR


def clean() -> None:
    """Clean build artifacts."""
    if DIST_DIR.exists():
        shutil.rmtree(DIST_DIR)
        print("🗑️  Removed dist/")

    # We don't remove wheels/ from platforms/blender because they might be cached/committed
    # But strictly speaking 'clean' usually means removing generated files.
    # We will skip cleaning wheels to be safe for now, as syncing manages them.


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "build"

    if cmd == "sync":
        sync_dependencies()
    elif cmd == "build":
        if DEP_CONFIG:
            update_manifest_wheels()
        build_extension()
    elif cmd == "clean":
        clean()
    else:
        print(f"Unknown command: {cmd}")
        print("Usage: python3 scripts/build_blender.py [sync|build|clean]")
