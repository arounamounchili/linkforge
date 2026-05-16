# ADR-002: Centralized API and Source Layout Migration

- **Status**: Accepted
- **Date**: 2026-05-16
- **Authors**: Arouna Patouossa Mounchili (@arounamounchili)

---

## Context

Prior to v1.4.0, the LinkForge codebase used a standard "flat" layout where packages were located in the root of their respective workspace directories (e.g., `core/linkforge/core/`). Additionally, the public API was highly fragmented, requiring users to know deep module paths (e.g., `from linkforge.core.models.robot import Robot`) to use basic library features.

This led to several architectural problems:
1.  **Test Shadowing**: In Python, having the source code in the root can cause tests to run against the local source directory instead of the installed package. This hides installation bugs and dependency issues.
2.  **Poor Discoverability**: New users struggled to find the "front door" of the library.
3.  **Namespace Fragility**: Without a strict `src/` layout, it was easier to accidentally pollute the namespace with utility scripts and non-package files.

---

## Decision

**We will migrate the entire LinkForge ecosystem to a "src layout" and centralize the public API at the `linkforge.core` level.**

### 1. The `src` Layout
All source code is moved into a `src/` subdirectory within each workspace member.
- **Core**: `core/src/linkforge/core/`
- **Blender**: `platforms/blender/src/linkforge/blender/`

### 2. API Centralization (The "Curated" API)
The `linkforge.core.__init__.py` file now acts as a centralized "Front Door." It explicitly exports the most important classes, functions, and constants using `__all__`.

Users can now simply do:
```python
import linkforge.core as lf

robot = lf.Robot(name="my_robot")
lf.read_urdf("robot.urdf")
```

---

## Consequences

### Positive
- **Stable Public API**: We can change the internal module structure (refactor) without breaking the user's code, as long as the centralized export remains stable.
- **Improved Testing**: Tests are now guaranteed to run against the installed package, making CI much more reliable and catching deployment issues early.
- **Namespace Protection**: The `src` layout prevents the `linkforge` namespace from being accidentally polluted by top-level project files.
- **PEP 420 Consistency**: This structure makes it easier to support multiple platforms (FreeCAD, Onshape, etc.) as separate installable units under the same `linkforge.*` namespace.

### Negative
- **Path Length**: Developers now have to navigate deeper paths (e.g., `core/src/linkforge/core/...`) which can be slightly more tedious in the terminal.
- **Import Overhead**: Centralizing the API in `__init__.py` means that importing `linkforge.core` loads many sub-modules into memory. However, for a robotics library of this size, this impact is negligible compared to the benefits of discoverability.

---

## Alternatives Considered

### Alternative: Keep Flat Layout
Rejected. While simpler for small scripts, it is considered an anti-pattern for professional Python libraries that aim for high test reliability and proper distribution.

### Alternative: Deep-only Imports
Rejected. Requiring users to navigate `linkforge.core.models.robot.Robot` creates high friction and makes refactoring internal logic nearly impossible without breaking user scripts.

---

## When to Revisit This Decision

This ADR should be reconsidered when:
- The startup time of `import linkforge.core` exceeds 1 second (indicating we need lazy loading or a less centralized `__init__.py`).
- A specific platform requires a non-standard Python environment where the `src` layout interferes with platform-specific packaging (e.g., very restricted embedded systems).

---

## References
- [PyPA: src layout vs flat layout](https://packaging.python.org/en/latest/discussions/src-layout-vs-flat-layout/)
- [Hynek Schlawack: Testing & Packaging](https://hynek.me/articles/testing-packaging/)
- [PEP 420: Implicit Namespace Packages](https://peps.python.org/pep-0420/)
