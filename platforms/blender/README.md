# LinkForge for Blender

Blender Extension for LinkForge.
This package integrates the LinkForge core logic directly into Blender's UI and Operator system, transforming Blender into a robust robotics editor.

## Structure

- `linkforge/`: The primary extension package.
  - `blender/`: Main integration logic (adapters, handlers, logic, operators, panels, etc.).
  - `blender_manifest.toml`: Extension metadata for Blender 4.2+.
- `scripts/`: Build and development utilities (e.g., `build.py`).
- `wheels/`: Platform-specific Python dependencies (bundled in the final `.zip`).
- `pyproject.toml`: Local development configuration.

> [!NOTE]
> The **LinkForge Core** library is located at the project root (`/core`) and is automatically bundled into this extension during the build process.

## Development

To work on the Blender extension, make sure `uv` is installed, then sync the dependencies:

```bash
just install
```

### Running Tests

To run Blender-specific integration and unit tests, from the project root use:

```bash
just test-blender
```
