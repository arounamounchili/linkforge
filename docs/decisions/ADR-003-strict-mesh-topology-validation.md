# ADR-003: Strict Mesh Topology Validation & Inertia Pipeline

## Status
Accepted

## Context
LinkForge initially used a loose approach to mesh validation, performing only basic manifold checks. However, "dirty" CAD data (unwelded vertices, inconsistent normals, boundary holes) led to physically impossible inertia tensors (negative mass distribution) in simulators like Gazebo and MuJoCo.

To achieve production-grade numerical robustness, we needed a more disciplined approach to how mesh data enters the physics pipeline.

## Decision
We have implemented a "Hardened Architecture" for mesh processing:

1.  **Strict Validation Layer**: The `validate_mesh_topology` utility now enforces five distinct check phases, including vertex proximity (welding detection) and consistent winding.
2.  **Mirtich Algorithm**: Inertia calculation now strictly follows the 1996 Mirtich algorithm (Divergence Theorem) with numerical conditioning (local origin shifting).
3.  **Physicality Guardrails**: Every inertia calculation is subjected to Sylvester's Criterion to ensure the resulting tensor is positive semi-definite (physically possible).
4.  **Fail-in-Editor Philosophy**: Any structural topology violation (holes or non-manifold edges) now raises a `RobotPhysicsError` in strict mode, forcing the user to fix the mesh before proceeding to simulation.
5.  **Positional API**: The `calculate_mesh_inertia_from_triangles` function now requires both `vertices` and `triangles` as positional arguments to ensure validation can always be performed.

## Consequences
- **Positive**: Numerical stability in downstream simulators is guaranteed. Silent "NaN" failures in inertia calculations are eliminated.
- **Negative**: Breaking change for internal APIs that only passed triangle data.
- **Maintenance**: The physics code is now more complex but much more documented with academic references.
- **Performance**: Vertex proximity check adds an $O(V)$ pass using spatial hashing, but remains efficient for large meshes.
