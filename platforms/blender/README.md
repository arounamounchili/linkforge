# LinkForge for Blender

Blender Extension for LinkForge.
This package integrates the LinkForge core logic directly into Blender's UI and Operator system, transforming Blender into a robust robotics editor.

## Structure

- `linkforge/`: The Blender extension source code containing panels, operators, and property groups.
  - `blender/`: The main Blender integration module.
    - `adapters/`: Bridges between Blender objects and `linkforge_core` models.
    - `handlers/`: Scene post-load and object update handlers.
    - `logic/`: High-level operations bridging core models and the viewport.
    - `operators/`: Blender Operators for importing, exporting, and managing robot data.
    - `panels/`: Blender UI layouts and side panels.
    - `properties/`: Blender Property Groups binding LinkForge data to Blender objects.
    - `visualization/`: Interactive 3D view elements for limits, joints, and inertia.
    - `utils/`: Blender-specific helper functions.
    - `preferences.py`: Extension configuration and settings.
  - `linkforge_core/`: Internal reference to the core library (symlinked for standalone use).
- `scripts/`: Development scripts and utilities.
- `wheels/`: Pre-built wheels for bundled Python dependencies.
- `blender_manifest.toml`: The extension metadata for Blender 4.2+.

## Development

To work on the Blender extension, make sure `uv` is installed, then sync the dependencies:

```bash
uv sync
```

### Running Tests

To run Blender-specific integration and unit tests, from the project root use:

```bash
python blender_launcher.py
```
