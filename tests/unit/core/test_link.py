"""Comprehensive tests for Link model and related classes."""

from __future__ import annotations

import pytest

from linkforge.core.models import (
    Box,
    Collision,
    Color,
    Inertial,
    InertiaTensor,
    Link,
    Material,
    Transform,
    Vector3,
    Visual,
)


class TestInertiaTensor:
    """Tests for InertiaTensor validation."""

    def test_valid_tensor(self):
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

    def test_negative_diagonal(self):
        """Test that negative diagonal elements raise error."""
        with pytest.raises(ValueError, match="must be positive"):
            InertiaTensor(
                ixx=-1.0,  # Invalid
                ixy=0.0,
                ixz=0.0,
                iyy=1.0,
                iyz=0.0,
                izz=1.0,
            )

    def test_negative_iyy(self):
        """Test that negative iyy raises error."""
        with pytest.raises(ValueError, match="must be positive"):
            InertiaTensor(
                ixx=1.0,
                ixy=0.0,
                ixz=0.0,
                iyy=-1.0,  # Invalid
                iyz=0.0,
                izz=1.0,
            )

    def test_negative_izz(self):
        """Test that negative izz raises error."""
        with pytest.raises(ValueError, match="must be positive"):
            InertiaTensor(
                ixx=1.0,
                ixy=0.0,
                ixz=0.0,
                iyy=1.0,
                iyz=0.0,
                izz=-1.0,  # Invalid
            )

    def test_triangle_inequality(self):
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

    def test_triangle_inequality_violation(self):
        """Test that triangle inequality violations raise error."""
        with pytest.raises(ValueError, match="triangle inequality"):
            InertiaTensor(
                ixx=1.0,
                ixy=0.0,
                ixz=0.0,
                iyy=1.0,
                iyz=0.0,
                izz=10.0,  # Too large: ixx + iyy < izz
            )

    def test_zero_tensor(self):
        """Test zero inertia tensor."""
        tensor = InertiaTensor.zero()
        # Zero tensor has minimal valid values
        assert tensor.ixx > 0
        assert tensor.iyy > 0
        assert tensor.izz > 0
        assert tensor.ixx == tensor.iyy == tensor.izz

    def test_symmetric_off_diagonals(self):
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

    def test_creation(self):
        """Test creating inertial properties."""
        tensor = InertiaTensor.zero()
        inertial = Inertial(
            mass=10.0,
            inertia=tensor,
        )
        assert inertial.mass == 10.0
        assert inertial.inertia == tensor

    def test_negative_mass(self):
        """Test that negative mass raises error."""
        tensor = InertiaTensor.zero()
        with pytest.raises(ValueError, match="Mass must be non-negative"):
            Inertial(
                mass=-1.0,  # Invalid
                inertia=tensor,
            )

    def test_zero_mass(self):
        """Test that zero mass is valid (massless link)."""
        tensor = InertiaTensor.zero()
        inertial = Inertial(mass=0.0, inertia=tensor)
        assert inertial.mass == 0.0

    def test_with_origin(self):
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

    def test_creation(self):
        """Test creating visual element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom)
        assert visual.geometry == geom

    def test_with_material(self):
        """Test visual with material."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
        visual = Visual(geometry=geom, material=material)
        assert visual.material == material

    def test_with_origin(self):
        """Test visual with origin."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        origin = Transform(xyz=Vector3(0.1, 0.2, 0.3))
        visual = Visual(geometry=geom, origin=origin)
        assert visual.origin == origin

    def test_with_name(self):
        """Test visual with name."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom, name="my_visual")
        assert visual.name == "my_visual"


class TestCollision:
    """Tests for Collision class."""

    def test_creation(self):
        """Test creating collision element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom)
        assert collision.geometry == geom

    def test_with_origin(self):
        """Test collision with origin."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        origin = Transform(xyz=Vector3(0.1, 0.2, 0.3))
        collision = Collision(geometry=geom, origin=origin)
        assert collision.origin == origin

    def test_with_name(self):
        """Test collision with name."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom, name="my_collision")
        assert collision.name == "my_collision"


class TestLink:
    """Tests for Link model."""

    def test_simple_link(self):
        """Test creating a simple link."""
        link = Link(name="link1")
        assert link.name == "link1"
        assert not link.visuals
        assert not link.collisions
        assert link.inertial is None

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(ValueError, match="name cannot be empty"):
            Link(name="")

    def test_invalid_name_characters(self):
        """Test that invalid characters raise error."""
        with pytest.raises(ValueError, match="invalid characters"):
            Link(name="link with spaces!")

    def test_valid_name_with_underscore(self):
        """Test that underscores are valid."""
        link = Link(name="link_1")
        assert link.name == "link_1"

    def test_valid_name_with_hyphen(self):
        """Test that hyphens are valid."""
        link = Link(name="link-1")
        assert link.name == "link-1"

    def test_link_with_visual(self):
        """Test link with visual element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom)
        link = Link(name="link1", visuals=[visual])
        assert link.visuals[0] == visual

    def test_link_with_collision(self):
        """Test link with collision element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom)
        link = Link(name="link1", collisions=[collision])
        assert link.collisions[0] == collision

    def test_link_with_inertial(self):
        """Test link with inertial properties."""
        tensor = InertiaTensor.zero()
        inertial = Inertial(mass=5.0, inertia=tensor)
        link = Link(name="link1", inertial=inertial)
        assert link.inertial == inertial

    def test_link_mass_property(self):
        """Test link mass property."""
        # Link without inertial
        link1 = Link(name="link1")
        assert link1.mass == 0.0

        # Link with inertial
        tensor = InertiaTensor.zero()
        inertial = Inertial(mass=10.0, inertia=tensor)
        link2 = Link(name="link2", inertial=inertial)
        assert link2.mass == 10.0

    def test_complete_link(self):
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
