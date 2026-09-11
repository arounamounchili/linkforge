"""Unit tests for geometry and material extraction in geometry_extractor."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.adapters.geometry_extractor import (
    detect_primitive_type,
    extract_mesh_triangles,
    get_object_geometry,
    get_object_material,
)
from linkforge.blender.properties.geom_props import (
    GEOM_CYLINDER,
    GEOM_SPHERE,
    PROP_GEOM,
)
from linkforge.core import Box, Cylinder, Mesh, Sphere
from mathutils import Matrix

from tests.blender_test_utils import (
    create_mesh_object,
    create_test_object,
    safe_get_linkforge,
)


class TestDetectPrimitiveType:
    def test_detect_primitive_type_none_and_non_mesh(self, scene) -> None:
        """Verify detect_primitive_type handles None and empty objects."""
        assert detect_primitive_type(None) is None
        empty = create_test_object("empty_obj", None, scene)
        assert detect_primitive_type(empty) is None

    def test_detect_primitive_type_box(self, scene) -> None:
        """Verify that a basic cube mesh is detected as box."""
        bpy.ops.mesh.primitive_cube_add()
        obj = bpy.context.active_object
        assert obj is not None
        assert detect_primitive_type(obj) == "box"

    def test_detect_primitive_type_sphere(self, scene) -> None:
        """Verify that a UV sphere is detected as sphere."""
        bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=1.0)
        obj = bpy.context.active_object
        assert obj is not None
        assert detect_primitive_type(obj) == "sphere"

    def test_detect_primitive_type_cylinder(self, scene) -> None:
        """Verify that a standard cylinder is detected as cylinder."""
        bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=1.0, depth=3.0)
        obj = bpy.context.active_object
        assert obj is not None
        assert detect_primitive_type(obj) == "cylinder"

    def test_detect_primitive_type_complex_mesh(self, scene) -> None:
        """Verify that a complex mesh (Suzanne) returns None."""
        bpy.ops.mesh.primitive_monkey_add()
        obj = bpy.context.active_object
        assert obj is not None
        assert detect_primitive_type(obj) is None

    def test_detect_primitive_type_forced_tag(self, scene) -> None:
        """Verify that custom tags or properties override detection."""
        obj = create_mesh_object("tagged_obj", scene, with_cube=True)
        geom_props = getattr(obj, PROP_GEOM)
        geom_props.geometry_type = GEOM_CYLINDER
        assert detect_primitive_type(obj) == "cylinder"


class TestGetObjectGeometry:
    def test_get_object_geometry_none_and_empty(self, scene) -> None:
        """Verify None object returns (None, Identity) and non-mesh returns (None, matrix_world)."""
        assert get_object_geometry(None) == (None, Matrix.Identity(4))
        empty = create_test_object("empty_geom", None, scene)
        assert get_object_geometry(empty) == (None, empty.matrix_world)

    def test_get_object_geometry_primitives(self, scene) -> None:
        """Verify auto-detection of Box, Sphere, and Cylinder."""
        # Box
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        box_obj = bpy.context.active_object
        assert box_obj is not None
        geom_b, wm_b = get_object_geometry(box_obj)
        assert isinstance(geom_b, Box)
        assert wm_b == box_obj.matrix_world

        # Sphere
        bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=0.5)
        sph_obj = bpy.context.active_object
        assert sph_obj is not None
        geom_s, wm_s = get_object_geometry(sph_obj)
        assert isinstance(geom_s, Sphere)
        assert geom_s.radius > 0.0

        # Cylinder
        bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=0.3, depth=1.0)
        cyl_obj = bpy.context.active_object
        assert cyl_obj is not None
        geom_c, wm_c = get_object_geometry(cyl_obj)
        assert isinstance(geom_c, Cylinder)
        assert geom_c.radius > 0.0
        assert geom_c.length > 0.0

    def test_get_object_geometry_forced_override(self, scene) -> None:
        """Verify forced primitive types via geom properties."""
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        obj = bpy.context.active_object
        assert obj is not None
        geom_props = getattr(obj, PROP_GEOM)

        geom_props.geometry_type = GEOM_SPHERE
        geom_s, _ = get_object_geometry(obj)
        assert isinstance(geom_s, Sphere)
        assert pytest.approx(geom_s.radius) == 1.0

        geom_props.geometry_type = GEOM_CYLINDER
        geom_c, _ = get_object_geometry(obj)
        assert isinstance(geom_c, Cylinder)
        assert pytest.approx(geom_c.radius) == 1.0
        assert pytest.approx(geom_c.length) == 2.0

    def test_get_object_geometry_mesh_export(self, tmp_path, scene) -> None:
        """Verify export to mesh file when meshes_dir is provided."""
        bpy.ops.mesh.primitive_monkey_add()
        obj = bpy.context.active_object
        assert obj is not None
        geom, _ = get_object_geometry(obj, meshes_dir=tmp_path, link_name="monkey_link")
        assert isinstance(geom, (Box, Mesh))


class TestGetObjectMaterial:
    def test_get_object_material_no_props_or_disabled(self, scene) -> None:
        """Verify material extraction when disabled or missing."""
        obj = create_mesh_object("mat_obj", scene, with_cube=True)
        props = safe_get_linkforge(obj)
        props.use_material = False
        assert get_object_material(obj, props) is None

    def test_get_object_material_principled_bsdf(self, scene) -> None:
        """Verify material extraction with Principled BSDF node."""
        obj = create_mesh_object("bsdf_mat_obj", scene, with_cube=True)
        props = safe_get_linkforge(obj)
        props.use_material = True

        mat = bpy.data.materials.new(name="RobotMat")
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        assert bsdf is not None
        socket = bsdf.inputs.get("Base Color")
        if socket and hasattr(socket, "default_value"):
            socket.default_value = 0.2, 0.4, 0.8, 1.0

        obj.data.materials.append(mat)
        core_mat = get_object_material(obj, props)
        assert core_mat is not None
        assert "RobotMat" in core_mat.name
        assert core_mat.color is not None
        assert pytest.approx(core_mat.color.r) == 0.2
        assert pytest.approx(core_mat.color.g) == 0.4
        assert pytest.approx(core_mat.color.b) == 0.8


class TestExtractMeshTriangles:
    def test_extract_mesh_triangles_none_or_empty(self, scene) -> None:
        """Verify extract_mesh_triangles with None or empty mesh."""
        assert extract_mesh_triangles(None) is None
        empty = create_test_object("empty_extract", None, scene)
        assert extract_mesh_triangles(empty) is None

    def test_extract_mesh_triangles_valid_cube(self, scene) -> None:
        """Verify extracting vertices and triangles from a cube."""
        obj = create_mesh_object("extract_cube", scene, with_cube=True)
        res = extract_mesh_triangles(obj, as_numpy=False)
        assert res is not None
        verts, triangles = res
        assert len(verts) > 0
        assert len(triangles) > 0
        assert all(len(tri) == 3 for tri in triangles)

    def test_extract_mesh_triangles_numpy(self, scene) -> None:
        """Verify numpy array extraction mode and fallback."""
        obj = create_mesh_object("extract_cube_np", scene, with_cube=True)

        class MockArray:
            def __init__(self, data):
                self.data = data

            def __getitem__(self, idx):
                return self

            def __setitem__(self, idx, val):
                pass

            def __imul__(self, other):
                return self

            def __mul__(self, other):
                return self

            def __rmul__(self, other):
                return self

            def reshape(self, shape):
                return self

            def tolist(self):
                return self.data

        mock_np = MagicMock()
        mock_np.zeros = lambda size, dtype=None: MockArray([1.0] * size)

        with patch("linkforge.blender.adapters.geometry_extractor.np", mock_np):
            res_np = extract_mesh_triangles(obj, as_numpy=True)
            assert res_np is not None
            verts, tris = res_np
            assert hasattr(verts, "tolist")
            assert hasattr(tris, "tolist")

        with patch("linkforge.blender.adapters.geometry_extractor.np", None):
            res_py = extract_mesh_triangles(obj)
            assert res_py is not None
