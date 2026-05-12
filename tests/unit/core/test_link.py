"""Comprehensive tests for Link model and related classes."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models.geometry import Box, Transform, Vector3
from linkforge_core.models.link import Collision, Inertial, InertiaTensor, Link, LinkPhysics, Visual
from linkforge_core.models.material import Color, Material


class TestInertiaTensor:
    """Tests for InertiaTensor validation."""

    def test_valid_tensor(self) -> None:
        """Test creating valid inertia tensor."""
        tensor = InertiaTensor(
            ixx=1.0,
            ixy=0.0,
            ixz=0.0,
            iyy=1.0,
            iyz=0.0,
            izz=1.0,
        )
        assert tensor.ixx == 1.0
        assert tensor.iyy == 1.0
        assert tensor.izz == 1.0

    def test_negative_diagonal(self) -> None:
        """Test that negative diagonal elements raise error."""
        with pytest.raises(RobotModelError):
            InertiaTensor(
                ixx=-1.0,  # Invalid
                ixy=0.0,
                ixz=0.0,
                iyy=1.0,
                iyz=0.0,
                izz=1.0,
            )

    def test_negative_iyy(self) -> None:
        """Test that negative iyy raises error."""
        with pytest.raises(RobotModelError):
            InertiaTensor(
                ixx=1.0,
                ixy=0.0,
                ixz=0.0,
                iyy=-1.0,  # Invalid
                iyz=0.0,
                izz=1.0,
            )

    def test_negative_izz(self) -> None:
        """Test that negative izz raises error."""
        with pytest.raises(RobotModelError):
            InertiaTensor(
                ixx=1.0,
                ixy=0.0,
                ixz=0.0,
                iyy=1.0,
                iyz=0.0,
                izz=-1.0,  # Invalid
            )

    def test_triangle_inequality(self) -> None:
        """Test triangle inequality validation."""
        # Valid case: ixx + iyy >= izz
        tensor = InertiaTensor(
            ixx=1.0,
            ixy=0.0,
            ixz=0.0,
            iyy=1.0,
            iyz=0.0,
            izz=2.0,
        )
        assert tensor.izz == 2.0

    def test_triangle_inequality_violation(self) -> None:
        """Test that triangle inequality violations raise error."""
        with pytest.raises(RobotModelError):
            InertiaTensor(
                ixx=1.0,
                ixy=0.0,
                ixz=0.0,
                iyy=1.0,
                iyz=0.0,
                izz=10.0,  # Too large: ixx + iyy < izz
            )

    def test_zero_tensor(self) -> None:
        """Test zero inertia tensor."""
        tensor = InertiaTensor.zero()
        # Zero tensor has minimal valid values
        assert tensor.ixx > 0
        assert tensor.iyy > 0
        assert tensor.izz > 0
        assert tensor.ixx == tensor.iyy == tensor.izz

    def test_symmetric_off_diagonals(self) -> None:
        """Test that off-diagonal elements can be non-zero."""
        tensor = InertiaTensor(
            ixx=2.0,
            ixy=0.5,
            ixz=0.3,
            iyy=2.0,
            iyz=0.4,
            izz=2.0,
        )
        assert tensor.ixy == 0.5
        assert tensor.ixz == 0.3
        assert tensor.iyz == 0.4


class TestInertial:
    """Tests for Inertial class."""

    def test_creation(self) -> None:
        """Test creating inertial properties."""
        tensor = InertiaTensor.zero()
        inertial = Inertial(
            mass=10.0,
            inertia=tensor,
        )
        assert inertial.mass == 10.0
        assert inertial.inertia == tensor

    def test_negative_mass(self) -> None:
        """Test that negative mass raises error."""
        tensor = InertiaTensor.zero()
        with pytest.raises(RobotModelError):
            Inertial(
                mass=-1.0,  # Invalid
                inertia=tensor,
            )

    def test_zero_mass(self) -> None:
        """Test that zero mass is valid (massless link)."""
        tensor = InertiaTensor.zero()
        inertial = Inertial(mass=0.0, inertia=tensor)
        assert inertial.mass == 0.0

    def test_with_origin(self) -> None:
        """Test inertial with custom origin."""
        tensor = InertiaTensor.zero()
        origin = Transform(xyz=Vector3(0.1, 0.2, 0.3))
        inertial = Inertial(
            mass=5.0,
            origin=origin,
            inertia=tensor,
        )
        assert inertial.origin == origin


class TestVisual:
    """Tests for Visual class."""

    def test_creation(self) -> None:
        """Test creating visual element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom)
        assert visual.geometry == geom

    def test_with_material(self) -> None:
        """Test visual with material."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
        visual = Visual(geometry=geom, material=material)
        assert visual.material == material

    def test_with_origin(self) -> None:
        """Test visual with origin."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        origin = Transform(xyz=Vector3(0.1, 0.2, 0.3))
        visual = Visual(geometry=geom, origin=origin)
        assert visual.origin == origin

    def test_with_name(self) -> None:
        """Test visual with name."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom, name="my_visual")
        assert visual.name == "my_visual"

    def test_prefix(self) -> None:
        """Test creating a visual with a prefix."""
        geom = Box(size=Vector3(1, 1, 1))
        mat = Material(name="red", color=Color(1, 0, 0))
        vis = Visual(name="vis", geometry=geom, material=mat)
        vis_pre = vis.with_prefix("p_")
        assert vis_pre.name == "p_vis"

        material = vis_pre.material
        assert material is not None
        assert material.name == "p_red"


class TestCollision:
    """Tests for Collision class."""

    def test_creation(self) -> None:
        """Test creating collision element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom)
        assert collision.geometry == geom

    def test_with_origin(self) -> None:
        """Test collision with origin."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        origin = Transform(xyz=Vector3(0.1, 0.2, 0.3))
        collision = Collision(geometry=geom, origin=origin)
        assert collision.origin == origin

    def test_with_name(self) -> None:
        """Test collision with name."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom, name="my_collision")
        assert collision.name == "my_collision"

    def test_prefix(self) -> None:
        """Test creating a collision with a prefix."""
        geom = Box(size=Vector3(1, 1, 1))
        col = Collision(name="col", geometry=geom)
        col_pre = col.with_prefix("p_")
        assert col_pre.name == "p_col"


class TestLinkPhysics:
    """Tests for LinkPhysics class."""

    def test_default_values(self) -> None:
        """Test that default values are correctly assigned."""
        physics = LinkPhysics()
        from linkforge_core.constants import (
            DEFAULT_CONTACT_KD,
            DEFAULT_CONTACT_KP,
            DEFAULT_FRICTION_MU,
            DEFAULT_GRAVITY,
            DEFAULT_SELF_COLLIDE,
        )

        assert physics.self_collide == DEFAULT_SELF_COLLIDE
        assert physics.gravity == DEFAULT_GRAVITY
        assert physics.mu == DEFAULT_FRICTION_MU
        assert physics.kp == DEFAULT_CONTACT_KP
        assert physics.kd == DEFAULT_CONTACT_KD

    def test_custom_values(self) -> None:
        """Test creating physics with custom values."""
        physics = LinkPhysics(
            self_collide=True,
            gravity=False,
            mu=0.5,
            kp=1e6,
            kd=10.0,
        )
        assert physics.self_collide is True
        assert physics.gravity is False
        assert physics.mu == 0.5
        assert physics.kp == 1e6
        assert physics.kd == 10.0


class TestLink:
    """Tests for Link model."""

    def test_simple_link(self) -> None:
        """Test creating a simple link."""
        link = Link(name="link1")
        assert link.name == "link1"
        assert not link.visuals
        assert not link.collisions
        assert link.inertial is None
        assert isinstance(link.physics, LinkPhysics)

    def test_empty_name(self) -> None:
        """Test that empty name raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            Link(name="")

    def test_invalid_name_characters(self) -> None:
        """Test that invalid characters raise error."""
        with pytest.raises(RobotModelError):
            Link(name="link with spaces!")

    def test_valid_name_with_underscore(self) -> None:
        """Test that underscores are valid."""
        link = Link(name="link_1")
        assert link.name == "link_1"

    def test_valid_name_with_hyphen(self) -> None:
        """Test that hyphens are valid."""
        link = Link(name="link-1")
        assert link.name == "link-1"

    def test_link_with_visual(self) -> None:
        """Test link with visual element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom)
        link = Link(name="link1", visuals=[visual])
        assert link.visuals[0] == visual

    def test_link_with_collision(self) -> None:
        """Test link with collision element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom)
        link = Link(name="link1", collisions=[collision])
        assert link.collisions[0] == collision

    def test_link_with_inertial(self) -> None:
        """Test link with inertial properties."""
        tensor = InertiaTensor.zero()
        inertial = Inertial(mass=5.0, inertia=tensor)
        link = Link(name="link1", inertial=inertial)
        assert link.inertial == inertial

    def test_link_mass_property(self) -> None:
        """Test link mass property."""
        # Link without inertial
        link1 = Link(name="link1")
        assert link1.mass == 0.0

        # Link with inertial
        tensor = InertiaTensor.zero()
        inertial = Inertial(mass=10.0, inertia=tensor)
        link2 = Link(name="link2", inertial=inertial)
        assert link2.mass == 10.0

    def test_complete_link(self) -> None:
        """Test link with all elements."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        material = Material(name="blue", color=Color(0.0, 0.0, 1.0, 1.0))
        visual = Visual(geometry=geom, material=material)
        collision = Collision(geometry=geom)
        tensor = InertiaTensor.zero()
        inertial = Inertial(mass=5.0, inertia=tensor)

        link = Link(
            name="complete_link",
            visuals=[visual],
            collisions=[collision],
            inertial=inertial,
        )

        assert link.name == "complete_link"
        assert link.visuals[0] == visual
        assert link.collisions[0] == collision
        assert link.inertial == inertial
        assert link.mass == 5.0

    def test_prefix(self) -> None:
        """Test creating a link with a prefix."""
        link = Link(name="base")
        link.add_visual(Visual(name="v", geometry=Box(Vector3(1, 1, 1))))
        link.add_collision(Collision(name="c", geometry=Box(Vector3(1, 1, 1))))

        prefixed = link.with_prefix("r_")
        assert prefixed.name == "r_base"
        assert prefixed.visuals[0].name == "r_v"
        assert prefixed.collisions[0].name == "r_c"
