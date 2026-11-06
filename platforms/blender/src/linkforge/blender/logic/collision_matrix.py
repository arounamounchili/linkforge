"""Sampled Allowed Collision Matrix (ACM) generation logic for MoveIt 2 SRDF planning."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any

import bpy

try:
    from mathutils import Matrix, Vector
except ImportError:
    Matrix = None  # type: ignore[assignment,misc]
    Vector = None  # type: ignore[assignment,misc]

from ..constants import SUFFIX_COLLISION
from ..core.constants import (
    JOINT_CONTINUOUS,
    JOINT_FIXED,
    JOINT_PRISMATIC,
    JOINT_REVOLUTE,
    SRDF_REASON_ADJACENT,
    SRDF_REASON_ALWAYS,
    SRDF_REASON_DEFAULT,
    SRDF_REASON_NEVER,
)
from ..utils.joint_utils import get_joint_axis_vector
from ..utils.property_helpers import get_joint_props, get_link_props
from ..utils.transform_utils import live_world_matrix


@dataclass(slots=True)
class LinkGeometryBounds:
    """Bounding sphere representation of a robot link for fast collision queries."""

    name: str
    center_local: tuple[float, float, float]
    radius: float
    obj: Any


@dataclass(slots=True)
class KinematicJointNode:
    """Internal node for Forward Kinematics tree traversal during Monte Carlo sampling."""

    name: str
    parent_link: str
    child_link: str
    joint_type: str
    axis_vec: tuple[float, float, float]
    lower: float
    upper: float
    # Relative transform from parent link frame to this joint's origin
    origin_transform: Any
    obj: Any


def _get_bounding_sphere(link_obj: Any) -> tuple[tuple[float, float, float], float]:
    """Compute local bounding center and sphere radius for a robot link."""
    # Look for collision children or evaluate link bounding box
    mesh_objs = [
        c
        for c in getattr(link_obj, "children", [])
        if getattr(c, "type", "") == "MESH" and SUFFIX_COLLISION in getattr(c, "name", "")
    ]
    if not mesh_objs:
        mesh_objs = [
            c for c in getattr(link_obj, "children", []) if getattr(c, "type", "") == "MESH"
        ]

    if not mesh_objs:
        # Check if link itself has bound_box
        bb = getattr(link_obj, "bound_box", None)
        if bb and len(bb) >= 8:
            pts = [(float(p[0]), float(p[1]), float(p[2])) for p in bb]
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            cz = sum(p[2] for p in pts) / len(pts)
            rad = max(
                math.sqrt((p[0] - cx) ** 2 + (p[1] - cy) ** 2 + (p[2] - cz) ** 2) for p in pts
            )
            return (cx, cy, cz), max(rad, 0.02)
        return (0.0, 0.0, 0.0), 0.05

    # Aggregate bounding boxes from mesh objects
    all_pts: list[tuple[float, float, float]] = []
    for m in mesh_objs:
        bb = getattr(m, "bound_box", None)
        if bb and len(bb) >= 8:
            for p in bb:
                # Local to link transform
                all_pts.append((float(p[0]), float(p[1]), float(p[2])))

    if not all_pts:
        return (0.0, 0.0, 0.0), 0.05

    cx = sum(p[0] for p in all_pts) / len(all_pts)
    cy = sum(p[1] for p in all_pts) / len(all_pts)
    cz = sum(p[2] for p in all_pts) / len(all_pts)
    rad = max(math.sqrt((p[0] - cx) ** 2 + (p[1] - cy) ** 2 + (p[2] - cz) ** 2) for p in all_pts)
    return (cx, cy, cz), max(rad, 0.02)


def compute_allowed_collision_matrix(
    scene: bpy.types.Scene,
    sample_count: int = 500,
    include_adjacent: bool = True,
    include_never: bool = True,
    include_default: bool = True,
    include_always: bool = True,
) -> list[tuple[str, str, str]]:
    """Compute MoveIt-compatible Allowed Collision Matrix (ACM) via kinematic sampling.

    Classifies link pairs into:
    - 'Adjacent': Physically connected directly or through fixed-joint clusters.
    - 'Default': Overlapping in the default/rest pose (q = 0).
    - 'Never': Kinematic workspaces never collide across Monte Carlo samples.
    - 'Always': Colliding in 100% of reachable sampled states.

    Args:
        scene: Active Blender scene containing robot model.
        sample_count: Number of random kinematic configurations to sample.
        include_adjacent: Include directly adjacent and fixed-connected pairs.
        include_never: Include pairs that never collide across samples.
        include_default: Include pairs that collide in default pose.
        include_always: Include pairs that always collide.

    Returns:
        list[tuple[str, str, str]]: List of (link1, link2, reason).
    """
    if Matrix is None or not scene:
        return []

    sample_count = int(sample_count)

    # 1. Gather all robot links
    link_objs: dict[str, Any] = {}
    for obj in getattr(scene, "objects", []):
        lp = get_link_props(obj)
        if lp and lp.is_robot_link:
            name = str(getattr(lp, "link_name", "") or getattr(obj, "name", "")).strip()
            if name:
                link_objs[name] = obj

    link_names = sorted(link_objs.keys())
    if len(link_names) < 2:
        return []

    # 2. Gather all robot joints and build connectivity
    joint_nodes: list[KinematicJointNode] = []
    parent_map: dict[str, str] = {}
    child_map: dict[str, list[KinematicJointNode]] = {ln: [] for ln in link_names}
    fixed_clusters: dict[str, set[str]] = {ln: {ln} for ln in link_names}
    direct_adjacent: set[frozenset[str]] = set()

    for obj in getattr(scene, "objects", []):
        jp = get_joint_props(obj)
        if not (jp and jp.is_robot_joint):
            continue

        p_obj = getattr(jp, "parent_link", None)
        c_obj = getattr(jp, "child_link", None)
        if not (p_obj and c_obj):
            continue

        p_lp = get_link_props(p_obj)
        c_lp = get_link_props(c_obj)
        p_name = str(getattr(p_lp, "link_name", "") or getattr(p_obj, "name", "")).strip()
        c_name = str(getattr(c_lp, "link_name", "") or getattr(c_obj, "name", "")).strip()

        if not (p_name in link_objs and c_name in link_objs and p_name != c_name):
            continue

        direct_adjacent.add(frozenset([p_name, c_name]))
        parent_map[c_name] = p_name

        j_type = str(getattr(jp, "joint_type", JOINT_REVOLUTE)).lower()
        axis_vec = get_joint_axis_vector(jp)
        low = float(getattr(jp, "limit_lower", -math.pi))
        high = float(getattr(jp, "limit_upper", math.pi))
        if low >= high:
            low, high = -math.pi, math.pi

        # Compute relative origin from parent link to child link at rest
        p_mw = live_world_matrix(p_obj)
        c_mw = live_world_matrix(c_obj)
        origin_m = p_mw.inverted() @ c_mw

        node = KinematicJointNode(
            name=str(getattr(jp, "joint_name", "") or getattr(obj, "name", "")),
            parent_link=p_name,
            child_link=c_name,
            joint_type=j_type,
            axis_vec=axis_vec,
            lower=low,
            upper=high,
            origin_transform=origin_m,
            obj=obj,
        )
        joint_nodes.append(node)
        child_map[p_name].append(node)

        # Fixed joints form rigid clusters (never move relative to each other)
        if j_type == JOINT_FIXED:
            merged = fixed_clusters[p_name] | fixed_clusters[c_name]
            for member in merged:
                fixed_clusters[member] = merged

    # 3. Compute geometric bounding spheres
    bounds_map: dict[str, LinkGeometryBounds] = {}
    for name, obj in link_objs.items():
        center, radius = _get_bounding_sphere(obj)
        bounds_map[name] = LinkGeometryBounds(
            name=name,
            center_local=center,
            radius=radius,
            obj=obj,
        )

    # 4. Find root link (link with no parent)
    root_candidates = [ln for ln in link_names if ln not in parent_map]
    root_name = root_candidates[0] if root_candidates else link_names[0]

    # Pre-identify moving joints
    moving_joints = [
        j
        for j in joint_nodes
        if j.joint_type in (JOINT_REVOLUTE, JOINT_CONTINUOUS, JOINT_PRISMATIC)
    ]

    # 5. Forward Kinematics Evaluator
    def compute_fk(joint_values: dict[str, float]) -> dict[str, Any]:
        """Compute world matrices for all links given joint values."""
        if Matrix is None:
            return {}
        fk_frames: dict[str, Any] = {root_name: Matrix.Identity(4)}

        def traverse(p_name: str) -> None:
            if Matrix is None:
                return
            p_mat = fk_frames[p_name]
            for jn in child_map.get(p_name, []):
                q = joint_values.get(jn.name, 0.0)
                if jn.joint_type in (JOINT_REVOLUTE, JOINT_CONTINUOUS):
                    rot = Matrix.Rotation(q, 4, jn.axis_vec)
                    c_mat = p_mat @ jn.origin_transform @ rot
                elif jn.joint_type == JOINT_PRISMATIC:
                    disp = (
                        jn.axis_vec[0] * q,
                        jn.axis_vec[1] * q,
                        jn.axis_vec[2] * q,
                    )
                    trans = Matrix.Translation(Vector(disp) if Vector is not None else disp)
                    c_mat = p_mat @ jn.origin_transform @ trans
                else:
                    c_mat = p_mat @ jn.origin_transform
                fk_frames[jn.child_link] = c_mat
                traverse(jn.child_link)

        traverse(root_name)
        # Any disconnected link stays at rest
        for ln, l_obj in link_objs.items():
            if ln not in fk_frames:
                fk_frames[ln] = live_world_matrix(l_obj)
        return fk_frames

    # 6. Evaluate Default Pose Collisions
    rest_fk = compute_fk({})
    default_collision_pairs: set[frozenset[str]] = set()

    def check_pair_collision(l1: str, l2: str, frames: dict[str, Any]) -> bool:
        """Check if bounding spheres of link1 and link2 intersect."""
        b1 = bounds_map[l1]
        b2 = bounds_map[l2]
        m1 = frames[l1]
        m2 = frames[l2]

        v1 = Vector(b1.center_local) if Vector is not None else b1.center_local
        v2 = Vector(b2.center_local) if Vector is not None else b2.center_local

        # Transform local centers to world
        w1 = m1 @ v1 if hasattr(m1, "__matmul__") else m1
        w2 = m2 @ v2 if hasattr(m2, "__matmul__") else m2

        dx = (getattr(w1, "x", w1[0]) if hasattr(w1, "x") else w1[0]) - (
            getattr(w2, "x", w2[0]) if hasattr(w2, "x") else w2[0]
        )
        dy = (getattr(w1, "y", w1[1]) if hasattr(w1, "y") else w1[1]) - (
            getattr(w2, "y", w2[1]) if hasattr(w2, "y") else w2[1]
        )
        dz = (getattr(w1, "z", w1[2]) if hasattr(w1, "z") else w1[2]) - (
            getattr(w2, "z", w2[2]) if hasattr(w2, "z") else w2[2]
        )
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist < (b1.radius + b2.radius)

    for i in range(len(link_names)):
        for j in range(i + 1, len(link_names)):
            pair = frozenset([link_names[i], link_names[j]])
            if pair in direct_adjacent:
                continue
            if link_names[j] in fixed_clusters[link_names[i]]:
                continue
            if check_pair_collision(link_names[i], link_names[j], rest_fk):
                default_collision_pairs.add(pair)

    # 7. Monte Carlo Sampling
    collision_counts: dict[frozenset[str], int] = {}
    candidate_pairs: list[tuple[str, str, frozenset[str]]] = []

    for i in range(len(link_names)):
        for j in range(i + 1, len(link_names)):
            l1, l2 = link_names[i], link_names[j]
            pair = frozenset([l1, l2])
            if pair in direct_adjacent or l2 in fixed_clusters[l1]:
                continue
            candidate_pairs.append((l1, l2, pair))
            collision_counts[pair] = 0

    rng = random.Random(42)  # Deterministic seed for reproducible testing
    for _ in range(sample_count):
        # Sample random joint values
        sample_q: dict[str, float] = {}
        for jn in moving_joints:
            sample_q[jn.name] = rng.uniform(jn.lower, jn.upper)

        fk = compute_fk(sample_q)
        for l1, l2, pair in candidate_pairs:
            if check_pair_collision(l1, l2, fk):
                collision_counts[pair] += 1

    # 8. Categorize and build results
    results: list[tuple[str, str, str]] = []
    processed_pairs: set[frozenset[str]] = set()

    # A. Adjacent (Direct and Fixed Chains)
    if include_adjacent:
        for pair in direct_adjacent:
            items = sorted(pair)
            results.append((items[0], items[1], SRDF_REASON_ADJACENT))
            processed_pairs.add(pair)

        # Fixed clusters
        for l1 in link_names:
            for l2 in fixed_clusters[l1]:
                if l1 < l2:
                    pair = frozenset([l1, l2])
                    if pair not in processed_pairs:
                        results.append((l1, l2, SRDF_REASON_ADJACENT))
                        processed_pairs.add(pair)

    # B. Default / Rest Collisions
    if include_default:
        for pair in default_collision_pairs:
            if pair not in processed_pairs:
                items = sorted(pair)
                results.append((items[0], items[1], SRDF_REASON_DEFAULT))
                processed_pairs.add(pair)

    # C. Never Colliding & Always Colliding
    for l1, l2, pair in candidate_pairs:
        if pair in processed_pairs:
            continue
        hits = collision_counts.get(pair, 0)
        if include_never and hits == 0:
            results.append((l1, l2, SRDF_REASON_NEVER))
            processed_pairs.add(pair)
        elif include_always and sample_count > 0 and hits == sample_count:
            results.append((l1, l2, SRDF_REASON_ALWAYS))
            processed_pairs.add(pair)

    # Sort results for stability
    results.sort(key=lambda x: (x[0], x[1], x[2]))
    return results
