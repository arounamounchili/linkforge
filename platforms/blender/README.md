# LinkForge for Blender
**The High-Fidelity Robotics Digital Twin Platform.**

This package integrates the LinkForge core logic directly into Blender's UI and Operator system, transforming Blender into a robust, physics-compliant robotics editor.

## 📂 Structure

- `src/linkforge/blender/`: The main integration logic.
  - `adapters/`: Unified translators between Core IR and Blender data models.
  - `logic/`: Asynchronous builders and scene orchestration.
  - `operators/`: Blender operators for Forge, Perceive, and Control actions.
  - `panels/`: Centralized UI dashboards for robot design.
  - `properties/`: Metadata-rich Blender property groups for round-trip fidelity.
- `blender_manifest.toml`: Extension metadata for Blender 4.2+.
- `scripts/`: Build and development utilities (e.g., `build.py`).
- `pyproject.toml`: Local development and workspace configuration.

> [!NOTE]
> The **LinkForge Core** library is located at the project root (`../../core`) and is automatically bundled into this extension during the build process to ensure zero-loss translation.

## 🛠️ Development

To work on the Blender extension, ensure `uv` and `just` are installed:

```bash
just install   # Setup dev environment
just develop   # Link workspace to Blender for live-editing
```

### 🧪 Running Tests

LinkForge uses a tiered testing strategy for Blender:

1.  **Fast Logic Tests** (Mocked `bpy`):
    ```bash
    just test-unit-blender
    ```
2.  **Full Integration Tests** (Real Blender):
    ```bash
    just test-integration-blender
    ```

For the complete testing strategy, see the [Tests README](../../tests/README.md).
