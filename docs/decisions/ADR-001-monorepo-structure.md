# ADR-001: Monorepo Structure for the LinkForge Ecosystem

- **Status**: Accepted
- **Date**: 2026-03-04
- **Authors**: Arouna Patouossa Mounchili (@arounamounchili)

---

## Context

As the LinkForge ecosystem grows to support multiple platforms (Blender, ROS 2, and future adapters for FreeCAD, Unity, Isaac Sim, etc.), a recurring architectural question arises: should each platform adapter live in its own dedicated GitHub repository, or should all components remain in a single monorepo?

This ADR documents the decision made and the reasoning behind it, so that future contributors and maintainers can understand *why* the structure is the way it is.

---

## Decision

**All components of the LinkForge ecosystem — `linkforge_core` and all platform adapters — will be maintained in a single monorepo, with one exception: platform adapters that have a fundamentally separate installer ecosystem may be extracted to a standalone repository.**

The extraction rule is:

> Extract a platform to its own repo **only if** its users install it via a completely separate package manager (e.g., `apt` for ROS, Blender Extension Manager).
> All pip-installable adapters remain in the monorepo.

### Current platform decisions:

| Package | Installer | Location |
|---|---|---|
| `linkforge_core` | `pip` | `core/` — always in this repo |
| `linkforge-blender` | Blender Extension Manager | `platforms/blender/` — stays in this repo |
| `linkforge-ros` | `pip` + `apt` (ROS Index) | `platforms/ros/` — may be extracted after v1.4.0 stabilizes |
| Future adapters (FreeCAD, Web, etc.) | `pip` | `platforms/<name>/` — stays in this repo |

---

## Consequences

### Positive
- **Atomic changes**: A core model change is validated against all adapters in one Pull Request. There is no risk of adapters silently going out of sync with the core API.
- **Single contributor setup**: `git clone` + `uv sync` is sufficient to work on any part of the ecosystem. No cross-repo dependency wiring required.
- **Unified CI/CD**: One pipeline enforces consistent code quality and test coverage across core and all adapters.
- **No version matrix hell**: End users cannot install a combination of `linkforge_core` and adapter versions that are incompatible with each other.

### Negative
- **Repo size**: As the number of adapters grows, the repository will grow. This is mitigated by the `platforms/` directory structure which keeps adapters well-isolated.
- **Broader CI scope**: Every push triggers tests for all platforms. Engineering effort is required to keep CI fast as the project grows.

---

## Alternatives Considered

### Alternative: Full Multi-Repo (One Repo per Package)
Rejected. The primary reason is **version synchronization complexity**. When `linkforge_core` releases a breaking change, every adapter repo must release a compatible update simultaneously. This is operationally very expensive for a small team and a common cause of open-source project abandonment. The monorepo eliminates this risk entirely.

### Alternative: Separate Core Repo + Platform Monorepo
Rejected. `linkforge_core` is the dependency anchor for every adapter. Keeping it in a separate repo creates the same version synchronization problem as above. It would require every adapter to pin to a specific core release, making development iteration significantly slower.

---

## When to Revisit This Decision

This ADR should be reconsidered when:
1. CI pipeline duration exceeds 15 minutes consistently due to unrelated platform tests.
2. A platform community has grown large enough to warrant its own governance (issue trackers, maintainers, release cadence).
3. A new platform adapter requires a completely different runtime environment (e.g., a specific NVIDIA driver version for Isaac Sim) that cannot coexist with the Python-only requirements of the core.

---

## References
- [Monorepos in Open Source — Babel, React, SQLAlchemy](https://monorepo.tools/)
- [python-poetry/poetry — Monorepo with multiple packages](https://github.com/python-poetry/poetry)
- [SQLAlchemy — single-repo multi-package example](https://github.com/sqlalchemy/sqlalchemy)
