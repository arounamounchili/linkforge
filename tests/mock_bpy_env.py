from __future__ import annotations

import contextlib
import math
import re
import sys
import types
import typing
from collections.abc import Callable
from pathlib import Path
from typing import Any, Generic, TypeVar, cast, overload
from unittest.mock import MagicMock, PropertyMock


class MockState:
    def __init__(self):
        self.data = None
        self.context = None
        self.types = None
        self.props = None
        self.ops = None
        self.app = None


state = MockState()


class DynamicModule(types.ModuleType):
    def __getattr__(self, name):
        if name not in self.__dict__:
            # For category-like access in bpy.ops, return another DynamicModule
            # to allow bpy.ops.any_category.any_operator()
            if self.__name__.startswith("bpy.ops"):
                self.__dict__[name] = DynamicModule(f"{self.__name__}.{name}")
            else:
                self.__dict__[name] = MagicMock(name=name)
        return self.__dict__[name]

    def __setattr__(self, name, value):
        self.__dict__[name] = value


class MockVector:
    """Mock for mathutils.Vector."""

    _data: list[float]

    def __init__(self, x: float | typing.Iterable[float] = 0.0, y: float = 0.0, z: float = 0.0):
        if isinstance(x, (list, tuple, MockVector, MagicMock)):
            self._data = [float(v) for v in x]
        elif hasattr(x, "x") and hasattr(x, "y") and hasattr(x, "z"):
            self._data = [float(getattr(x, "x")), float(getattr(x, "y")), float(getattr(x, "z"))]
        else:
            self._data = [float(x), float(y), float(z)]

    @property
    def x(self):
        return self._data[0]

    @x.setter
    def x(self, v):
        self._data[0] = float(v)

    @property
    def y(self):
        return self._data[1]

    @y.setter
    def y(self, v):
        self._data[1] = float(v)

    @property
    def z(self):
        return self._data[2]

    @z.setter
    def z(self, v):
        self._data[2] = float(v)

    def __getitem__(self, i):
        return self._data[i]

    def __setitem__(self, i, v):
        self._data[i] = float(v)

    def __len__(self):
        return 3

    def __iter__(self):
        return iter(self._data)

    def __add__(self, other):
        return MockVector(self.x + other[0], self.y + other[1], self.z + other[2])

    def __sub__(self, other):
        return MockVector(self.x - other[0], self.y - other[1], self.z - other[2])

    @property
    def length(self):
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5

    def copy(self):
        return MockVector(self.x, self.y, self.z)

    def normalized(self):
        len_val = self.length
        if len_val < 1e-6:
            return MockVector(0, 0, 0)
        return MockVector(self.x / len_val, self.y / len_val, self.z / len_val)

    def normalize(self):
        len_val = self.length
        if len_val > 1e-6:
            self.x /= len_val
            self.y /= len_val
            self.z /= len_val

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return MockVector(self.x * other, self.y * other, self.z * other)
        return self

    def __truediv__(self, other):
        if isinstance(other, (int, float)) and other != 0:
            return MockVector(self.x / other, self.y / other, self.z / other)
        return self

    def dot(self, other):
        return self.x * other[0] + self.y * other[1] + self.z * other[2]

    def cross(self, other):
        return MockVector(
            self.y * other[2] - self.z * other[1],
            self.z * other[0] - self.x * other[2],
            self.x * other[1] - self.y * other[0],
        )

    def __repr__(self):
        return f"Vector(({self.x}, {self.y}, {self.z}))"

    def rotation_difference(self, other):
        return MockQuaternion()


class MockQuaternion(MockVector):
    """Mock for mathutils.Quaternion."""

    _euler_hint: MockEuler | None
    _quaternion_hint: MockQuaternion | None

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        super().__init__(x, y, z)
        self.w = float(w)
        self._euler_hint = None
        self._quaternion_hint = None

    def copy(self):
        q = MockQuaternion()
        q.w, q.x, q.y, q.z = self.w, self.x, self.y, self.z
        return q

    def to_euler(self, order="XYZ"):
        return MockEuler(self.x, self.y, self.z, order)

    def to_matrix(self):
        res = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            res[i][i] = 1.0
        m = MockMatrix(res)
        m._quaternion_hint = self.copy()
        if self._euler_hint is not None:
            m._euler_hint = self._euler_hint.copy()
        return m


class MockEuler(MockVector):
    """Mock for mathutils.Euler."""

    _euler_hint: MockEuler | None
    order: str

    def __init__(
        self,
        x: float | typing.Iterable[float] = 0.0,
        y: float | str = 0.0,
        z: float = 0.0,
        order: str = "XYZ",
    ):
        if isinstance(x, (list, tuple, MockVector)):
            super().__init__(x)
            self.order = y if isinstance(y, str) else order
        else:
            # y must be a float if x is not an iterable
            super().__init__(x, cast(float, y), z)
            self.order = order
        self._euler_hint = None

    def __repr__(self):
        return f"Euler(({self.x}, {self.y}, {self.z}), '{self.order}')"

    def to_matrix(self):
        m = MockMatrix.Identity(3)
        m._euler_hint = self.copy()
        return m

    def to_quaternion(self):
        q = MockQuaternion()
        q._euler_hint = self.copy()
        return q

    def to_4x4(self):
        m = MockMatrix.Identity(4)
        m._euler_hint = self
        return m


class MockMatrix:
    """Mock for mathutils.Matrix."""

    data: list[list[float]]
    _euler_hint: MockEuler | None
    _quaternion_hint: MockQuaternion | None

    def __init__(self, data=None):
        if data is None:
            self.data = [[0.0] * 4 for _ in range(4)]
            for i in range(4):
                self.data[i][i] = 1.0
        elif (
            isinstance(data, (list, tuple)) and len(data) > 0 and isinstance(data[0], (list, tuple))
        ):
            rows = len(data)
            cols = len(data[0])
            self.data = [[float(data[i][j]) for j in range(cols)] for i in range(rows)]
        else:
            self.data = data
        self._euler_hint = None
        self._quaternion_hint = None

    def __getitem__(self, i):
        return self.data[i]

    def __setitem__(self, i, v):
        self.data[i] = v

    def __len__(self):
        return len(self.data)

    @staticmethod
    def Identity(n):  # noqa: N802
        m = MockMatrix([[0.0] * n for _ in range(n)])
        for i in range(n):
            m.data[i][i] = 1.0
        return m

    @staticmethod
    def Rotation(angle, size, axis):  # noqa: N802
        m = MockMatrix.Identity(size)
        if axis == "X":
            m._euler_hint = MockEuler(angle, 0, 0)
        elif axis == "Y":
            m._euler_hint = MockEuler(0, angle, 0)
        elif axis == "Z":
            m._euler_hint = MockEuler(0, 0, angle)
        return m

    @staticmethod
    def Diagonal(vec):  # noqa: N802
        m = MockMatrix()
        for i in range(min(len(vec), 4)):
            m.data[i][i] = vec[i]
        return m

    @staticmethod
    def Translation(vec):  # noqa: N802
        m = MockMatrix()
        m.data[0][3] = vec[0]
        m.data[1][3] = vec[1]
        m.data[2][3] = vec[2]
        return m

    @property
    def translation(self):
        return MockVector(self.data[0][3], self.data[1][3], self.data[2][3])

    def to_translation(self):
        return self.translation

    def to_quaternion(self):
        hint = getattr(self, "_quaternion_hint", None)
        if hint is not None:
            return hint
        return MockQuaternion()

    @property
    def is_identity(self):
        n = len(self.data)
        for i in range(n):
            for j in range(len(self.data[0])):
                if i == j:
                    if abs(self.data[i][j] - 1.0) > 1e-6:
                        return False
                else:
                    if abs(self.data[i][j]) > 1e-6:
                        return False
        return True

    def to_euler(self, order="XYZ"):
        hint = getattr(self, "_euler_hint", None)
        if hint is not None:
            return hint
        return MockEuler(0, 0, 0, order=order)

    def to_4x4(self):
        if len(self.data) == 4:
            return self
        res = MockMatrix.Identity(4)
        for i in range(min(len(self.data), 4)):
            for j in range(min(len(self.data[0]), 4)):
                res.data[i][j] = self.data[i][j]
        res._euler_hint = getattr(self, "_euler_hint", None)
        res._quaternion_hint = getattr(self, "_quaternion_hint", None)
        return res

    def to_3x3(self):
        res_data = [[0.0] * 3 for _ in range(3)]
        for i in range(min(len(self.data), 3)):
            for j in range(min(len(self.data[0]), 3)):
                res_data[i][j] = self.data[i][j]
        res = MockMatrix(res_data)
        res._euler_hint = getattr(self, "_euler_hint", None)
        res._quaternion_hint = getattr(self, "_quaternion_hint", None)
        return res

    def to_scale(self):
        return MockVector(self.data[0][0], self.data[1][1], self.data[2][2])

    def identity(self):
        self.data = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            self.data[i][i] = 1.0

    def inverted(self):
        # High-fidelity inversion for 4x4 transform matrices
        inv = MockMatrix()
        # Transpose rotation part (assuming it's orthonormal)
        for i in range(3):
            for j in range(3):
                inv.data[i][j] = self.data[j][i]
        # inv_t = -R^T * t
        t = [self.data[0][3], self.data[1][3], self.data[2][3]]
        for i in range(3):
            dot = 0.0
            for j in range(3):
                dot += inv.data[i][j] * t[j]
            inv.data[i][3] = -dot
        return inv

    def __repr__(self):
        rows = ", ".join([str(tuple(r)) for r in self.data])
        return f"Matrix(({rows}))"

    def __eq__(self, other):
        if not isinstance(other, MockMatrix):
            return False
        return self.data == other.data

    def copy(self):
        m = MockMatrix([list(r) for r in self.data])
        m._euler_hint = self._euler_hint.copy() if self._euler_hint is not None else None
        m._quaternion_hint = (
            self._quaternion_hint.copy() if self._quaternion_hint is not None else None
        )
        return m

    def __matmul__(self, other):
        if isinstance(other, MockVector):
            res = [0.0] * 4
            v = [other.x, other.y, other.z, 1.0]
            for i in range(len(self.data)):
                for j in range(len(self.data[0])):
                    res[i] += self.data[i][j] * v[j]
            return MockVector(res[0], res[1], res[2])
        elif isinstance(other, MockMatrix):
            res_data = [[0.0] * 4 for _ in range(4)]
            for i in range(4):
                for j in range(4):
                    for k in range(4):
                        res_data[i][j] += self.data[i][k] * other.data[k][j]
            res = MockMatrix(res_data)
            res._euler_hint = getattr(other, "_euler_hint", getattr(self, "_euler_hint", None))
            return res
        return self

    def __mul__(self, other):
        return self.__matmul__(other)


class MockPropertyDescriptor:
    """Mock for Blender property descriptors (IntProperty, PointerProperty, etc.)."""

    def __init__(self, **kwargs):
        self.name = kwargs.get("name", "")
        self.prop_type = kwargs.get("prop_type")
        self.default = kwargs.get("default")
        self.min = kwargs.get("min")
        self.max = kwargs.get("max")
        self.update = kwargs.get("update")
        self.setter = kwargs.get("setter") or kwargs.get("set")
        self.getter = kwargs.get("getter") or kwargs.get("get")

    def __set_name__(self, owner, name):
        self.name = name

    def _discover_name(self, obj, cls):
        if not self.name or self.name == "unnamed_prop":
            for c in cls.__mro__:
                for k, v in c.__dict__.items():
                    if v is self:
                        self.name = k
                        return k
        return self.name

    def __get__(self, obj, cls):
        if obj is None:
            return self
        if not hasattr(obj, "_values"):
            obj._values = {}

        name = self._discover_name(obj, cls)

        if self.getter:
            try:
                return self.getter(obj)
            except Exception:
                pass

        if name not in obj._values:
            # Special handling for PointerProperty: they default to None in Blender,
            # unless they are nested PropertyGroups or Collections which are auto-instantiated.
            is_pointer = self.prop_type in (MockObject, MockMesh, MockMaterial)

            if self.prop_type and not is_pointer:
                try:
                    if self.prop_type == MockCollection:
                        val = MockCollection(prop_type=MockPropertyGroup)
                    else:
                        val = self.prop_type()

                    with contextlib.suppress(AttributeError, TypeError):
                        val.id_data = obj

                    obj._values[name] = val
                    return val
                except Exception:
                    pass

            obj._values[name] = self.default
            return self.default

        val = obj._values.get(name)
        return val

    def __set__(self, obj, value):
        if obj is None:
            return
        if not hasattr(obj, "_values"):
            obj._values = {}

        name = self._discover_name(obj, type(obj))

        if self.prop_type == MockVector and not isinstance(value, MockVector):
            value = MockVector(value)
        elif self.prop_type == MockEuler and not isinstance(value, MockEuler):
            value = MockEuler(value)

        obj._values[name] = value
        if self.setter:
            with contextlib.suppress(Exception):
                self.setter(obj, value)
        if self.update:
            bpy = sys.modules.get("bpy")
            if bpy is not None:
                with contextlib.suppress(Exception):
                    self.update(obj, bpy.context)
            else:
                with contextlib.suppress(Exception):
                    self.update(obj, None)


class PropertyMetaclass(type):
    def __init__(cls, name, bases, attrs):
        super().__init__(name, bases, attrs)
        annotations = getattr(cls, "__annotations__", {})
        for key, val in annotations.items():
            if isinstance(val, str) and ("Property" in val):
                try:
                    module = sys.modules.get(cls.__module__)
                    if module:
                        namespace = {**vars(module), "bpy": sys.modules.get("bpy")}
                        val = eval(val, namespace)
                except Exception:
                    pass
            if isinstance(val, MockPropertyDescriptor):
                val.name = key
                setattr(cls, key, val)
            elif isinstance(val, type) and issubclass(val, (int, float, bool, str)):
                prop = MockPropertyDescriptor(name=key, default=val())
                setattr(cls, key, prop)
        for key, val in attrs.items():
            if isinstance(val, MockPropertyDescriptor):
                val.name = key


RESERVED_RNA_PROPS = {
    "linkforge",
    "linkforge_joint",
    "linkforge_sensor",
    "linkforge_transmission",
    "linkforge_validation",
    "linkforge_scene",
}


DEFAULT_PROPERTY_VALUES = {
    "is_robot_link": False,
    "is_robot_joint": False,
    "is_robot_visual": False,
    "is_robot_collision": False,
    "is_robot_sensor": False,
    "is_robot_transmission": False,
    "is_robot_part": False,
    "mass": 0.0,
    "inertia_ixx": 0.0,
    "inertia_iyy": 0.0,
    "inertia_izz": 0.0,
    "inertia_ixy": 0.0,
    "inertia_ixz": 0.0,
    "inertia_iyz": 0.0,
    "collision_quality": 100.0,
    "collision_geometry_type": "MESH",
    "joint_type": "FIXED",
    "axis": (0.0, 0.0, 1.0),
    "use_limit": False,
    "limit_lower": 0.0,
    "limit_upper": 0.0,
    "ros2_control_active_joint_index": 0,
    "child_link": None,
    "parent_link": None,
    "ros2_control_type": "system",
    "ros2_control_name": "DefaultControl",
    "hardware_plugin": "fake_components/GenericSystem",
    "cmd_position": False,
    "cmd_velocity": False,
    "cmd_effort": False,
    "state_position": False,
    "state_velocity": False,
    "state_effort": False,
    "gazebo_plugin_name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
    "controllers_yaml_path": "$(find robot_description)/config/controllers.yaml",
    "robot_name": "robot",
    "strict_mode": False,
    "use_ros2_control": True,
    "export_format": "URDF",
    "export_meshes": True,
    "mesh_format": "OBJ",
    "mesh_directory_name": "meshes",
    "validate_before_export": True,
    "xacro_advanced_mode": True,
    "xacro_extract_materials": True,
    "xacro_extract_dimensions": True,
    "xacro_generate_macros": False,
    "xacro_split_files": False,
    "show_collisions": True,
    "show_kinematic_tree": False,
    "joint_name": "",
    "link_name": "",
    "sensor_name": "",
    "transmission_name": "",
    "use_material": True,
}


class MockPropertyGroup(metaclass=PropertyMetaclass):
    """Base class for mocked Blender PropertyGroups."""

    def __init__(self, **kwargs):
        self.__dict__["_values"] = {}
        self.__dict__["id_data"] = None
        for k, v in kwargs.items():
            setattr(self, k, v)
        if "name" not in self._values:
            self._values["name"] = kwargs.get("name", "Unnamed")

    def __setattr__(self, name, value):
        if name.startswith("_") or name in ("_values", "id_data"):
            super().__setattr__(name, value)
            return

        for cls in type(self).__mro__:
            if name in cls.__dict__:
                prop = cls.__dict__[name]
                if isinstance(prop, (property, MockPropertyDescriptor)):
                    super().__setattr__(name, value)
                    return

        self._values[name] = value

    def __getattr__(self, key):
        if key.startswith("__"):
            raise AttributeError(key)

        for cls in type(self).__mro__:
            if key in cls.__dict__:
                prop = cls.__dict__[key]
                if isinstance(prop, (property, MockPropertyDescriptor)):
                    return prop.__get__(self, type(self))

        if key in self._values:
            return self._values[key]

        if (
            key.startswith("is_robot_")
            or key.startswith("cmd_")
            or key.startswith("state_")
            or key.startswith("use_")
        ):
            # Try default values first
            if key in DEFAULT_PROPERTY_VALUES:
                return DEFAULT_PROPERTY_VALUES[key]
            return False

        if key in DEFAULT_PROPERTY_VALUES:
            val = DEFAULT_PROPERTY_VALUES[key]
            if isinstance(val, tuple) and len(val) == 3:
                vec = MockVector(val)
                self._values[key] = vec
                return vec
            return val

        if key == "id_data":
            return None
        if key == "bl_rna":
            return MagicMock(name="bl_rna")
        if key == "get":
            return self._mock_get

        if key in (
            "active_object",
            "object",
            "mesh",
            "material",
            "parent",
            "active_bone",
            "active_joint",
            "joint_obj",
            "joint_name",
            "joint1_name",
            "joint2_name",
            "parent_link",
            "child_link",
            "attached_link",
            "contact_collision",
        ):
            return None

        if key in RESERVED_RNA_PROPS:
            raise AttributeError(
                f"RNA property '{key}' not found on '{getattr(self, 'name', 'Unnamed')}'"
            )

        val = MagicMock(name=key)
        self._values[key] = val
        return val

    def clear(self):
        self._values.clear()

    @property
    def bl_rna(self):
        return MagicMock()

    def _mock_get(self, key, default=None):
        if key in self._values:
            return self._values[key]
        return default

    def __getitem__(self, key):
        if key in self._values:
            return self._values[key]
        raise KeyError(key)

    def __setitem__(self, key, value):
        self._values[key] = value

    def __contains__(self, key):
        return key in self._values

    def keys(self):
        return self._values.keys()


T = TypeVar("T")


class MockCollection(Generic[T]):
    """Mock for Blender's CollectionProperty items."""

    def __init__(
        self,
        prop_type: Callable[..., T] | None = None,
        name: str = "Collection",
        is_real_collection: bool = False,
    ):
        self._items: list[T] = []
        self.prop_type = prop_type
        self.name = name
        self.is_real_collection = is_real_collection
        self.new_from_object = None
        self.id_data = None
        self._id = id(self)
        self._objects = None
        self._children = None
        self._parent_collection = None

    def __hash__(self):
        return hash(self._id)

    def __eq__(self, other):
        return isinstance(other, MockCollection) and self._id == other._id

    @property
    def objects(self):
        if self._objects is None:
            self._objects = self
        return self._objects

    @objects.setter
    def objects(self, val):
        self._objects = val

    @property
    def children(self):
        if self._children is None:
            self._children = MockCollection(name=f"{self.name}_children")
        return self._children

    @children.setter
    def children(self, val):
        self._children = val

    def append(self, item: T):
        if item in self:
            return

        # Handle Blender's unique naming behavior
        if hasattr(item, "name") and getattr(item, "name"):
            base_name = getattr(item, "name")
            name = base_name
            counter = 1
            # Avoid infinite recursion by checking against private items list if needed,
            # but super() check is enough for basic uniqueness.
            existing_names = {
                getattr(obj, "name") for obj in self if obj != item and hasattr(obj, "name")
            }
            while name in existing_names:
                name = f"{base_name}.{counter:03d}"
                counter += 1
            # We use Any to avoid Pyright errors on unconstrained generic T
            typing.cast(Any, item).name = name

        self._items.append(item)

    def extend(self, items):
        for item in items:
            self.append(item)

    def __setitem__(self, key, value):
        if isinstance(key, (int, slice)):
            self._items[key] = value
            return
        # If it's a string, we might want to support replacement by name,
        # but Blender collections usually don't support direct name assignment like this.
        # However, for mocks it might be useful.
        for i, item in enumerate(self._items):
            if hasattr(item, "name") and item.name == key:
                self._items[i] = value
                return
        self.append(value)

    def link(self, item):
        self.append(item)
        # If this is an 'objects' collection of a real Collection, update item.users_collection
        parent_coll = getattr(self, "_parent_collection", None)
        if (
            parent_coll
            and hasattr(item, "users_collection")
            and parent_coll not in item.users_collection
        ):
            item.users_collection.append(parent_coll)

    def foreach_get(self, attr, data):
        for i, item in enumerate(self):
            val = getattr(item, attr)
            if hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
                for j, v in enumerate(val):
                    data[i * len(val) + j] = v
            else:
                data[i] = val

    def add(self, **kwargs) -> T:
        item = self.prop_type(**kwargs) if self.prop_type else MockPropertyGroup(**kwargs)
        # Use cast to satisfy Pyright since MockPropertyGroup might not be T if prop_type is missing
        casted_item = typing.cast(T, item)
        self.append(casted_item)
        return casted_item

    def new(self, name=None, data=None, type=None):  # noqa: A002
        if self.prop_type is MockObject:
            item = MockObject(name=name, data=data)
        elif self.prop_type:
            item = self.prop_type(name=name)
        else:
            item = MockPropertyGroup(name=name)

        if type and hasattr(item, "type"):
            typing.cast(Any, item).type = type

        casted_item = typing.cast(T, item)
        self.append(casted_item)
        return casted_item

    def __len__(self):
        return len(self._items)

    def __iter__(self) -> typing.Iterator[T]:
        return iter(self._items)

    @overload
    def __getitem__(self, key: int | str) -> T: ...

    @overload
    def __getitem__(self, key: slice) -> list[T]: ...

    def __getitem__(self, key: int | slice | str) -> T | list[T]:
        if isinstance(key, (int, slice)):
            return self._items[key]
        for item in self._items:
            if hasattr(item, "name") and getattr(item, "name") == key:
                return item
        raise KeyError(f"Item '{key}' not found in collection '{self.name}'")

    def __contains__(self, key):
        if isinstance(key, str):
            return any(hasattr(item, "name") and item.name == key for item in self._items)
        return key in self._items

    def get(self, key, default=None):
        try:
            return self[key]
        except (KeyError, TypeError):
            return default

    def clear(self):
        self._items.clear()

    def remove(self, item, do_unlink=True):
        if isinstance(item, int):
            if 0 <= item < len(self._items):
                self.pop(item)
        elif item in self._items:
            self._items.remove(item)

    def pop(self, index=-1):
        return self._items.pop(index)

    def keys(self):
        return [item.name for item in self if hasattr(item, "name")]

    @property
    def bl_rna(self):
        return MagicMock()

    def __getattr__(self, key):
        if key.startswith("__"):
            raise AttributeError(key)
        val = MagicMock(name=key)
        setattr(self, key, val)
        return val

    def unlink(self, obj):
        if obj in self:
            self.remove(obj)


class MockHandlers:
    def __init__(self):
        self.load_post = []
        self.save_pre = []
        self.save_post = []
        self.depsgraph_update_post = []
        self.depsgraph_update_pre = []
        self.render_pre = []
        self.render_post = []


class MockTimers:
    """Mock for bpy.app.timers."""

    def __init__(self):
        self._timers = []

    def register(self, func, first_interval=0.0):
        """Register a timer function."""
        self._timers.append(func)

    def run_all(self):
        """Execute all pending timers. Handles re-scheduling if a timer returns an interval."""
        current_timers = list(self._timers)
        self._timers.clear()

        while current_timers:
            func = current_timers.pop(0)
            try:
                result = func()
                # If the timer returns a float or int, it wants to be re-scheduled
                if isinstance(result, (int, float)):
                    self._timers.append(func)
            except Exception:
                pass


class MockMaterialSlot(MockPropertyGroup):
    def __init__(self, material=None, **kwargs):
        super().__init__(**kwargs)
        self.material = material


class MockVertex(MockPropertyGroup):
    def __init__(self, co=None, **kwargs):
        super().__init__(**kwargs)
        self.co = MockVector(co) if co is not None else MockVector()


class MockMesh(MockPropertyGroup):
    def __init__(self, name="Mesh"):
        super().__init__(name=name)
        self.name = name
        self.vertices = MockCollection(prop_type=MockVertex, name="vertices")
        self.polygons = MockCollection(
            prop_type=lambda **kwargs: MockPropertyGroup(vertices=MockCollection(), **kwargs),
            name="polygons",
        )
        self.materials = MockCollection(prop_type=MockMaterial, name="materials")

    def transform(self, matrix):
        for v in self.vertices:
            v.co = matrix @ v.co

    def copy(self):
        new_mesh = MockMesh(f"{self.name}_copy")
        for v in self.vertices:
            nv = new_mesh.vertices.add()
            nv.co = MockVector(v.co)
        for p in self.polygons:
            np = new_mesh.polygons.add()
            # In Blender, polygon vertices are indices. In mock, we support both but default to a collection.
            np.vertices = MockCollection()
            for v_idx in p.vertices:
                np.vertices.append(v_idx)
        return new_mesh

    def calc_loop_triangles(self):
        self.loop_triangles = MockCollection(
            prop_type=lambda **kwargs: MockPropertyGroup(vertices=MockCollection(), **kwargs)
        )
        for poly in self.polygons:
            if len(poly.vertices) == 4:
                t1 = self.loop_triangles.add()
                t1.vertices = MockCollection()
                t1.vertices.extend([poly.vertices[0], poly.vertices[1], poly.vertices[2]])
                t2 = self.loop_triangles.add()
                t2.vertices = MockCollection()
                t2.vertices.extend([poly.vertices[0], poly.vertices[2], poly.vertices[3]])
            else:
                t = self.loop_triangles.add()
                t.vertices = MockCollection()
                t.vertices.extend(list(poly.vertices))

    def to_mesh_clear(self):
        pass


class MockNodeInput(MockPropertyGroup):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.default_value = (0.0, 0.0, 0.0, 1.0)
        self.name = "Input"


class MockNodeOutput(MockPropertyGroup):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.name = "Output"


class MockNode(MockPropertyGroup):
    def __init__(self, name="Node", node_type="BSDF_PRINCIPLED", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.type = node_type
        self.inputs = MockCollection(prop_type=MockNodeInput)
        self.outputs = MockCollection(prop_type=MockNodeOutput)
        self.inputs.add()
        self.outputs.add()


class MockNodeTree(MockPropertyGroup):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.nodes = MockCollection(prop_type=MockNode)
        self.links = MockCollection(prop_type=MockPropertyGroup)


class MockMaterial(MockPropertyGroup):
    def __init__(self, name="Material", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self._use_nodes = False
        self.node_tree = MockNodeTree()
        self.diffuse_color = (0.8, 0.8, 0.8, 1.0)

    @property
    def use_nodes(self):
        return self._use_nodes

    @use_nodes.setter
    def use_nodes(self, value):
        self._use_nodes = value
        if value and len(self.node_tree.nodes) == 0:
            bsdf = self.node_tree.nodes.add()
            bsdf.name = "Principled BSDF"
            bsdf.type = "BSDF_PRINCIPLED"
            base_color = bsdf.inputs[0]
            base_color.name = "Base Color"
            base_color.default_value = (0.8, 0.8, 0.8, 1.0)

            output = self.node_tree.nodes.add()
            output.name = "Material Output"
            output.type = "OUTPUT_MATERIAL"


class MockCamera(MockPropertyGroup):
    def __init__(self, name="Camera", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.type = "PERSP"


class MockLight(MockPropertyGroup):
    def __init__(self, name="Light", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.type = "POINT"


class MockObject(MockPropertyGroup):
    data: Any
    linkforge: MockPropertyGroup
    linkforge_joint: MockPropertyGroup
    linkforge_sensor: MockPropertyGroup
    linkforge_transmission: MockPropertyGroup
    linkforge_validation: MockPropertyGroup
    linkforge_scene: MockPropertyGroup

    def __init__(self, name="Object", data=None, **kwargs):
        super().__init__(**kwargs)
        self._name = name
        self.data = data
        self.type = "MESH" if (data and not isinstance(data, MagicMock)) else "EMPTY"
        if data is not None and not isinstance(data, MagicMock):
            if isinstance(data, MockMesh):
                self.type = "MESH"
            elif isinstance(data, MockCamera):
                self.type = "CAMERA"
            elif isinstance(data, MockLight):
                self.type = "LIGHT"

        self._selected = False
        self._parent = None
        self._matrix_world = MockMatrix.Identity(4)
        self._matrix_local = MockMatrix.Identity(4)
        self.matrix_parent_inverse = MockMatrix.Identity(4)
        self.matrix_basis = MockMatrix.Identity(4)
        self._location = MockVector(0, 0, 0)
        self._rotation_euler = MockEuler(0, 0, 0)
        self.rotation_mode = "XYZ"
        self._scale = MockVector(1, 1, 1)
        self._base_dimensions = MockVector(0, 0, 0)
        self.constraints = MockCollection(name="constraints")
        self.modifiers = MockCollection(name="modifiers")
        self.children = MockCollection(name="children")
        self.users_collection = MockCollection(name="users_collection")
        self.bound_box = [(0.0, 0.0, 0.0)] * 8
        self.empty_display_type = "PLAIN_AXES"
        self.empty_display_size = 0.5
        self.hide_viewport = False
        self.hide_render = False

        has_linkforge = any("linkforge" in c.__dict__ for c in type(self).__mro__)
        if not has_linkforge:
            self.linkforge = MockPropertyGroup(name="linkforge")
            self.linkforge.ros2_control_joints = MockCollection(prop_type=MockPropertyGroup)
            self.linkforge.ros2_control_parameters = MockCollection(prop_type=MockPropertyGroup)

        if not any("linkforge_scene" in c.__dict__ for c in type(self).__mro__):
            self.linkforge_scene = MockPropertyGroup(name="linkforge_scene")
            self.linkforge_scene.ros2_control_joints = MockCollection(prop_type=MockPropertyGroup)

        if not any("linkforge_joint" in c.__dict__ for c in type(self).__mro__):
            self.linkforge_joint = MockPropertyGroup(name="linkforge_joint")
            self.linkforge_joint.is_robot_joint = False

        if not any("linkforge_sensor" in c.__dict__ for c in type(self).__mro__):
            self.linkforge_sensor = MockPropertyGroup(name="linkforge_sensor")
            self.linkforge_sensor.is_robot_sensor = False

        if not any("linkforge_transmission" in c.__dict__ for c in type(self).__mro__):
            self.linkforge_transmission = MockPropertyGroup(name="linkforge_transmission")
            self.linkforge_transmission.is_robot_transmission = False

        if not any("linkforge_validation" in c.__dict__ for c in type(self).__mro__):
            self.linkforge_validation = MockPropertyGroup(name="linkforge_validation")

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)
        if hasattr(self, "linkforge") and hasattr(self.linkforge, "link_name"):
            self.linkforge.link_name = str(value)
        if hasattr(self, "linkforge_joint") and hasattr(self.linkforge_joint, "joint_name"):
            self.linkforge_joint.joint_name = str(value)

    @property
    def matrix_world(self):
        if getattr(self, "_calculating_matrix_world", False):
            return self._matrix_local

        self._calculating_matrix_world = True
        try:
            if self.parent:
                return self.parent.matrix_world @ self.matrix_parent_inverse @ self.matrix_local
            return self.matrix_local
        finally:
            self._calculating_matrix_world = False

    @matrix_world.setter
    def matrix_world(self, value):
        m = MockMatrix(value)
        self._matrix_world = m
        if self.parent:
            self._matrix_local = (
                self.parent.matrix_world @ self.matrix_parent_inverse
            ).inverted() @ m
        else:
            self._matrix_local = m.copy()
        self._update_transforms_from_matrix_local()

    def _update_transforms_from_matrix_local(self):
        """Sync location, rotation, and scale from the local matrix."""
        self._location.x = self._matrix_local.data[0][3]
        self._location.y = self._matrix_local.data[1][3]
        self._location.z = self._matrix_local.data[2][3]

        # Extract scale (magnitude of basis vectors)
        for i in range(3):
            col = [self._matrix_local.data[j][i] for j in range(3)]
            self._scale[i] = math.sqrt(sum(c * c for c in col))

        # Euler extraction (simplistic for mock)
        euler = self._matrix_local.to_euler()
        self._rotation_euler.x = euler.x
        self._rotation_euler.y = euler.y
        self._rotation_euler.z = euler.z

    @property
    def matrix_local(self):
        return self._matrix_local

    @matrix_local.setter
    def matrix_local(self, value):
        self._matrix_local = MockMatrix(value)
        self._update_transforms_from_matrix_local()

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        if self._parent and hasattr(self._parent, "children") and self in self._parent.children:
            self._parent.children.remove(self)

        self._parent = value

        if value and hasattr(value, "children") and self not in value.children:
            value.children.append(self)

        if value is not None:
            self.matrix_parent_inverse = value.matrix_world.inverted()
        else:
            self.matrix_parent_inverse = MockMatrix.Identity(4)

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, value):
        self._location = MockVector(value)
        self._update_matrix_local()

    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, value):
        self._scale = MockVector(value)
        self._update_matrix_local()

    @property
    def rotation_euler(self):
        return self._rotation_euler

    @rotation_euler.setter
    def rotation_euler(self, value):
        self._rotation_euler = MockEuler(value)
        self._update_matrix_local()

    def _update_matrix_local(self):
        self._matrix_local.data[0][3] = self._location.x
        self._matrix_local.data[1][3] = self._location.y
        self._matrix_local.data[2][3] = self._location.z
        self._matrix_local.data[0][0] = self._scale.x
        self._matrix_local.data[1][1] = self._scale.y
        self._matrix_local.data[2][2] = self._scale.z

    def _get_base_dimensions(self):
        if (
            self._base_dimensions.x != 0
            or self._base_dimensions.y != 0
            or self._base_dimensions.z != 0
        ):
            return self._base_dimensions

        if self.data and hasattr(self.data, "vertices") and len(self.data.vertices) > 0:
            min_v = [float("inf")] * 3
            max_v = [float("-inf")] * 3
            for v in self.data.vertices:
                co = (0, 0, 0)
                if hasattr(v, "co") and not isinstance(v.co, MagicMock):
                    co = v.co

                try:
                    for i in range(3):
                        val = float(co[i])
                        min_v[i] = min(min_v[i], val)
                        max_v[i] = max(max_v[i], val)
                except (TypeError, ValueError, IndexError):
                    continue

            if min_v[0] != float("inf"):
                self._base_dimensions = MockVector(
                    max_v[0] - min_v[0], max_v[1] - min_v[1], max_v[2] - min_v[2]
                )

        return self._base_dimensions

    @property
    def dimensions(self):
        world_scale = self.scale.copy()
        p = self.parent
        visited = {self}
        while p:
            if p in visited:
                break
            visited.add(p)
            world_scale.x *= p.scale.x
            world_scale.y *= p.scale.y
            world_scale.z *= p.scale.z
            p = p.parent

        base_dim = self._get_base_dimensions()
        dims = MockVector(
            base_dim.x * world_scale.x, base_dim.y * world_scale.y, base_dim.z * world_scale.z
        )
        return dims

    @dimensions.setter
    def dimensions(self, value):
        target_dim = MockVector(value)
        base_dim = self._get_base_dimensions()

        parent_world_scale = MockVector(1.0, 1.0, 1.0)
        p = self.parent
        visited = {self}
        while p:
            if p in visited:
                break
            visited.add(p)
            parent_world_scale.x *= p.scale.x
            parent_world_scale.y *= p.scale.y
            parent_world_scale.z *= p.scale.z
            p = p.parent

        if abs(base_dim.x * parent_world_scale.x) > 1e-6:
            self.scale.x = target_dim.x / (base_dim.x * parent_world_scale.x)
        if abs(base_dim.y * parent_world_scale.y) > 1e-6:
            self.scale.y = target_dim.y / (base_dim.y * parent_world_scale.y)
        if abs(base_dim.z * parent_world_scale.z) > 1e-6:
            self.scale.z = target_dim.z / (base_dim.z * parent_world_scale.z)

        self._update_matrix_local()

    @property
    def material_slots(self):
        slots = MockCollection(prop_type=MockMaterialSlot)
        if self.data and hasattr(self.data, "materials"):
            for mat in self.data.materials:
                slot = slots.add()
                slot.material = mat
        return slots

    def select_get(self):
        return True

    def select_set(self, state):
        self._selected = state

    def transform(self, matrix):
        pass

    def copy(self):
        new_obj = MockObject(name=f"{self.name}_copy", data=self.data)
        new_obj.matrix_world = self.matrix_world.copy()
        # Register with global state so operators can find it
        if state.data:
            state.data.objects.append(new_obj)
        return new_obj

    def evaluated_get(self, depsgraph):
        return self

    def to_mesh(self, **kwargs):
        return self.data

    def to_mesh_clear(self):
        pass


class MockScene(MockPropertyGroup):
    """Mock for bpy.types.Scene."""

    linkforge: MockPropertyGroup
    linkforge_scene: MockPropertyGroup
    linkforge_validation: MockPropertyGroup

    def __init__(self, name="Scene", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.objects = MockCollection(prop_type=MockObject)
        self.collection = MockCollection(name="Master Collection", is_real_collection=True)
        self.collection.objects = self.objects
        self.collection.children = MockCollection()
        self.view_layers = MockCollection()
        self.view_layers.append(MockPropertyGroup(name="ViewLayer"))
        self.cursor = MagicMock(name="Cursor")
        self.cursor.location = MockVector(0, 0, 0)
        self.cursor.rotation_euler = MockEuler(0, 0, 0)

        # Pre-initialize LinkForge properties
        self.linkforge = MockPropertyGroup(name="linkforge")
        self.linkforge.robot_name = "Robot"
        self.linkforge.is_importing = False
        self.linkforge.abort_import = False
        self.linkforge.import_status = ""
        self.linkforge.show_collisions = False
        self.linkforge.use_ros2_control = True
        self.linkforge.ros2_control_joints = MockCollection(prop_type=MockPropertyGroup)
        self.linkforge.ros2_control_parameters = MockCollection(prop_type=MockPropertyGroup)

        self.linkforge_scene = self.linkforge
        self.linkforge_validation = MockPropertyGroup(name="linkforge_validation")


class MockOperator:
    """Mock for bpy.types.Operator."""

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        return {"FINISHED"}

    def invoke(self, context, event):
        return {"FINISHED"}

    def report(self, report_type, message):
        pass


class MockContext:
    """High-fidelity mock for bpy.types.Context."""

    def __init__(self):
        self.scene = None
        self.view_layer = None
        self.ops = None
        self.selected_objects = []
        self.data = None
        self.app = None
        self.window_manager = MagicMock(name="WindowManager")
        self.preferences = MagicMock(name="Preferences")

    @property
    def active_object(self):
        if self.view_layer and hasattr(self.view_layer.objects, "active"):
            return self.view_layer.objects.active
        return None

    @active_object.setter
    def active_object(self, value):
        if self.view_layer:
            self.view_layer.objects.active = value

    def __getattr__(self, name):
        # Fallback to MagicMock for any other attributes
        return MagicMock(name=name)


class MockIOHelper:
    def invoke(self, context, event):
        return {"FINISHED"}


mock_mathutils = DynamicModule("mathutils")
mock_mathutils.Vector = MockVector
mock_mathutils.Matrix = MockMatrix
mock_mathutils.Euler = MockEuler
mock_mathutils.Quaternion = MockQuaternion

mock_data = MockPropertyGroup(name="Data")
mock_context = MagicMock(name="bpy.context")

mock_ops = DynamicModule("bpy.ops")
mock_ops.mesh = DynamicModule("bpy.ops.mesh")
mock_ops.object = DynamicModule("bpy.ops.object")
mock_ops.wm = DynamicModule("bpy.ops.wm")
mock_ops.export_scene = DynamicModule("bpy.ops.export_scene")
mock_ops.linkforge = DynamicModule("bpy.ops.linkforge")

mock_bpy = DynamicModule("bpy")
mock_bpy.ops = mock_ops
mock_bpy.types = DynamicModule("bpy.types")
mock_bpy.types.Object = MockObject
mock_bpy.types.Mesh = MockMesh
mock_bpy.types.Scene = MockScene
mock_bpy.types.Collection = MockCollection
mock_bpy.types.PropertyGroup = MockPropertyGroup
mock_bpy.types.Operator = MockOperator
mock_bpy.props = DynamicModule("bpy.props")
mock_bpy.props.StringProperty = MockPropertyDescriptor
mock_bpy.props.FloatProperty = MockPropertyDescriptor
mock_bpy.props.IntProperty = MockPropertyDescriptor
mock_bpy.props.BoolProperty = MockPropertyDescriptor
mock_bpy.props.PointerProperty = MockPropertyDescriptor
mock_bpy.props.CollectionProperty = MockPropertyDescriptor
mock_bpy.props.EnumProperty = MockPropertyDescriptor
mock_bpy.props.FloatVectorProperty = MockPropertyDescriptor

mock_app = DynamicModule("bpy.app")

# Force promotion of mocks into sys.modules to ensure standalone execution
# matches the high-fidelity mock environment even if real Blender is present.
sys.modules["mathutils"] = typing.cast(types.ModuleType, mock_mathutils)
sys.modules["bpy"] = typing.cast(types.ModuleType, mock_bpy)
sys.modules["bpy.data"] = typing.cast(types.ModuleType, mock_data)
sys.modules["bpy.context"] = typing.cast(types.ModuleType, mock_context)
sys.modules["bpy.ops"] = typing.cast(types.ModuleType, mock_ops)
sys.modules["bpy.types"] = typing.cast(types.ModuleType, mock_bpy.types)
sys.modules["bpy.props"] = typing.cast(types.ModuleType, mock_bpy.props)
sys.modules["bpy.app"] = typing.cast(types.ModuleType, mock_app)


def setup_mock_bpy():
    """Configure high-fidelity mocks for Blender test environment."""
    # Reset persistent data for each test to ensure isolation
    mock_data.clear()
    mock_data.objects = MockCollection(prop_type=MockObject)
    # Collections in Blender can contain other objects/collections
    mock_data.collections = MockCollection(
        prop_type=lambda name: MockCollection(name=name, is_real_collection=True)
    )
    mock_data.meshes = MockCollection(prop_type=MockMesh)
    mock_data.actions = MockCollection()
    mock_data.node_groups = MockCollection()
    mock_data.cameras = MockCollection(prop_type=MockCamera)
    mock_data.lights = MockCollection(prop_type=MockLight)
    mock_data.materials = MockCollection(prop_type=MockMaterial)
    mock_data.scenes = MockCollection(prop_type=MockScene)

    active_scene = MockScene(name="Scene")
    mock_data.scenes.clear()
    mock_data.scenes.append(active_scene)

    # Setup Context
    global mock_context
    mock_context = MagicMock(name="Context")
    mock_bpy.data = mock_data
    state.data = mock_data
    mock_bpy.context = mock_context
    mock_bpy.app = mock_app

    mock_context.scene = active_scene
    mock_view_layer = typing.cast(MockPropertyGroup, active_scene.view_layers[0])
    mock_view_layer.objects = mock_data.objects
    mock_context.view_layer = mock_view_layer

    # Ensure active_object is always synced with view_layer
    # We use a PropertyMock on the instance's class to handle it properly
    type(mock_context).active_object = PropertyMock(
        side_effect=lambda *args: getattr(mock_view_layer.objects, "active", None)
    )

    mock_context.ops = mock_ops
    mock_context.selected_objects = []

    def mock_view_layer_update():
        """Trigger depsgraph handlers to simulate Blender's update cycle."""
        # Create a mock depsgraph with updates for each object
        mock_depsgraph = MagicMock(name="Depsgraph")
        mock_depsgraph.updates = []
        for obj in mock_data.objects:
            update = MagicMock()
            update.id = obj
            mock_depsgraph.updates.append(update)

        for handler in mock_app.handlers.depsgraph_update_post:
            with contextlib.suppress(Exception):
                handler(active_scene, mock_depsgraph)

        # Also run any timers scheduled during the handlers (like deferred renames)
        mock_app.timers.run_all()

    mock_view_layer.update = mock_view_layer_update
    mock_context.view_layer = mock_view_layer

    class ObjectsCollection(MockCollection):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self._active = None

        @property
        def active(self):
            return self._active

        @active.setter
        def active(self, val):
            self._active = val

    mock_view_layer.objects = ObjectsCollection(prop_type=MockObject)
    for obj in mock_data.objects:
        mock_view_layer.objects.append(obj)
    mock_data.objects = mock_view_layer.objects

    # Sync scene objects with data objects
    active_scene.objects = mock_data.objects
    active_scene.collection.objects = mock_data.objects
    active_scene.collection.objects._parent_collection = active_scene.collection

    mock_context.evaluated_depsgraph_get = lambda: MagicMock(name="Depsgraph")
    mock_context.window_manager = MockPropertyGroup()

    # Setup handlers and timers
    mock_app.timers = MockTimers()
    mock_app.handlers = MockHandlers()
    mock_app.version = (4, 2, 0)
    mock_app.driver_namespace = {}

    # High-fidelity operators logic
    # (Global mock_ops already initialized above)

    def mock_file_op(filepath=None, **kwargs):
        if filepath:
            p = Path(filepath)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.touch()
        return {"FINISHED"}

    mock_ops.wm.stl_export = mock_file_op
    mock_ops.wm.obj_export = mock_file_op
    mock_ops.wm.stl_import = mock_file_op
    mock_ops.wm.obj_import = mock_file_op
    mock_ops.export_scene.gltf = mock_file_op

    def _setup_new_object(obj, location=(0, 0, 0)):
        """Helper to place object in world and sync context."""
        # Use world matrix for placement – explicit index assignment avoids
        # the tuple-unpacking ambiguity with MockMatrix row access.
        mw = MockMatrix.Identity(4)
        mw.data[0][3] = float(location[0])
        mw.data[1][3] = float(location[1])
        mw.data[2][3] = float(location[2])
        obj.matrix_world = mw
        if obj not in mock_data.objects:
            mock_data.objects.append(obj)

        # Link to master collection and update users_collection
        if mock_context.scene and mock_context.scene.collection:
            if obj not in mock_context.scene.collection.objects:
                mock_context.scene.collection.objects.append(obj)
            if mock_context.scene.collection not in obj.users_collection:
                obj.users_collection.append(mock_context.scene.collection)

        mock_context.active_object = obj
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = obj
        return obj

    def mock_empty_add(type="PLAIN_AXES", location=(0, 0, 0), **kwargs):  # noqa: A002
        # Use name if passed (though real empty_add doesn't take one, some callers might mock it)
        name = kwargs.get("name", "Empty")
        obj = MockObject(name=name)
        obj.type = "EMPTY"
        obj.empty_display_type = type
        _setup_new_object(obj, location)
        return {"FINISHED"}

    def mock_cube_add(size=2.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="CubeMesh")
        # Vertices for a cube with side length 'size'
        half = size / 2.0
        coords = [
            (-half, -half, -half),
            (half, -half, -half),
            (half, half, -half),
            (-half, half, -half),
            (-half, -half, half),
            (half, -half, half),
            (half, half, half),
            (-half, half, half),
        ]
        for c in coords:
            v = mesh.vertices.add()
            v.co = MockVector(c)

        for p_idx in [
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 4, 5, 1),
            (1, 5, 6, 2),
            (2, 6, 7, 3),
            (3, 7, 4, 0),
        ]:
            p = mesh.polygons.add()
            p.vertices = MockCollection()
            p.vertices.extend(list(p_idx))

        obj = MockObject(name="Cube", data=mesh)
        _setup_new_object(obj, location)
        # base_dimensions should be calculated from vertices
        obj._base_dimensions = MockVector(0, 0, 0)
        obj.dimensions = MockVector(size, size, size)  # This will set scale to 1.0
        mock_data.meshes.append(mesh)
        return {"FINISHED"}

    def mock_sphere_add(radius=1.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="SphereMesh")
        # Add vertices to satisfy topology detection (default: 32 segs, 16 rings = 482 verts)
        for _ in range(482):
            mesh.vertices.add()
        # Add faces to satisfy topology detection (default: 480 faces)
        for _ in range(480):
            mesh.polygons.add()
        # Mock a sphere with just its bounding box vertices to ensure dimensions work
        mesh.vertices[0].co = MockVector(-1, -1, -1)
        mesh.vertices[1].co = MockVector(1, 1, 1)

        obj = MockObject(name="Sphere", data=mesh)
        _setup_new_object(obj, location)
        # Unit sphere in Blender has diameter 2.0 (radius 1.0)
        obj._base_dimensions = MockVector(2.0, 2.0, 2.0)
        obj.dimensions = MockVector(radius * 2, radius * 2, radius * 2)
        mock_data.meshes.append(mesh)
        return {"FINISHED"}

    def mock_cylinder_add(radius=1.0, depth=2.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="CylinderMesh")
        # Add vertices to satisfy topology detection (default: 32 vertices = 66 verts total)
        for _ in range(66):
            mesh.vertices.add()
        # Add faces to satisfy topology detection (32 segments = 32 side faces + 2 caps = 34 faces)
        for _ in range(34):
            mesh.polygons.add()
        # Bounding box for cylinder
        mesh.vertices[0].co = MockVector(-1, -1, -1)
        mesh.vertices[1].co = MockVector(1, 1, 1)

        obj = MockObject(name="Cylinder", data=mesh)
        _setup_new_object(obj, location)
        # Unit cylinder: radius 1.0, depth 2.0 -> dimensions (2, 2, 2)
        obj._base_dimensions = MockVector(2.0, 2.0, 2.0)
        obj.dimensions = MockVector(radius * 2, radius * 2, depth)
        mock_data.meshes.append(mesh)
        return {"FINISHED"}

    def mock_monkey_add(**kwargs):
        mesh = MockMesh(name="MonkeyMesh")
        # Suzanne: 1200 verts / 1100 faces – well outside all primitive thresholds
        # (sphere range is 240-1000 verts, so 1200 is clearly complex mesh)
        for _ in range(1200):
            mesh.vertices.add()
        for _ in range(1100):
            p = mesh.polygons.add()
            p.vertices = MockCollection()
            p.vertices.extend([0, 1, 2])  # Triangles (not quads)
        obj = MockObject(name="Suzanne", data=mesh)
        obj.dimensions = MockVector(2.0, 2.0, 2.0)
        _setup_new_object(obj)
        mock_data.meshes.append(mesh)
        return {"FINISHED"}

    mock_ops.object.empty_add = mock_empty_add
    mock_ops.mesh.primitive_cube_add = mock_cube_add
    mock_ops.mesh.primitive_uv_sphere_add = mock_sphere_add
    mock_ops.mesh.primitive_cylinder_add = mock_cylinder_add
    mock_ops.mesh.primitive_monkey_add = mock_monkey_add
    mock_ops.object.select_all = lambda action="TOGGLE": {"FINISHED"}

    def mock_select_all(action="TOGGLE"):
        for obj in mock_data.objects:
            if action == "SELECT":
                obj._selected = True
            elif action == "DESELECT":
                obj._selected = False
            elif action == "TOGGLE":
                obj._selected = not getattr(obj, "_selected", False)
        return {"FINISHED"}

    mock_ops.object.select_all = mock_select_all

    def mock_add_empty_link(**kwargs):
        name = kwargs.get("name", "base_link")
        obj = MockObject(name=name)
        obj.type = "EMPTY"
        _setup_new_object(obj)
        return {"FINISHED"}

    def mock_duplicate():
        selected = [obj for obj in mock_data.objects if getattr(obj, "_selected", False)]
        for obj in selected:
            new_obj = obj.copy()
            new_obj.select_set(True)
            # Blender makes the new one active if it was active
            if obj == mock_context.active_object:
                mock_context.active_object = new_obj
                if mock_context.view_layer:
                    mock_context.view_layer.objects.active = new_obj
        return {"FINISHED"}

    def mock_join():
        active = mock_context.active_object
        selected = [obj for obj in mock_data.objects if getattr(obj, "_selected", False)]
        if not active or not selected or active.type != "MESH":
            return {"CANCELLED"}

        for obj in selected:
            if obj == active or obj.type != "MESH":
                continue
            # Merge vertices
            offset = len(active.data.vertices)
            for v in obj.data.vertices:
                nv = active.data.vertices.add()
                nv.co = MockVector(v.co)
            # Merge polygons
            for poly in obj.data.polygons:
                np = active.data.polygons.add()
                np.vertices = MockCollection()
                np.vertices.extend([idx + offset for idx in poly.vertices])
            # Remove joined object
            mock_data.objects.remove(obj)

        # Clear cached dimensions
        active._base_dimensions = MockVector(0, 0, 0)
        return {"FINISHED"}

    def mock_transform_apply(location=True, rotation=True, scale=True):
        selected = [obj for obj in mock_data.objects if obj._selected]
        for obj in selected:
            if obj.type != "MESH":
                continue
            # Baking transform into vertices
            mat = obj.matrix_local
            for v in obj.data.vertices:
                v.co = mat @ v.co

            if location:
                obj.location = (0, 0, 0)
            if rotation:
                obj.rotation_euler = (0, 0, 0)
            if scale:
                obj.scale = (1, 1, 1)

            obj._update_matrix_local()
            obj._base_dimensions = MockVector(0, 0, 0)  # Force recalculation
        return {"FINISHED"}

    if not hasattr(mock_ops, "object"):
        mock_ops.object = DynamicModule("bpy.ops.object")
    mock_ops.object.join = mock_join
    mock_ops.object.duplicate = mock_duplicate
    mock_ops.object.transform_apply = mock_transform_apply
    mock_ops.object.parent_set = lambda **kwargs: {"FINISHED"}
    mock_ops.object.parent_clear = lambda **kwargs: {"FINISHED"}
    mock_ops.object.delete = lambda **kwargs: {"FINISHED"}

    if not hasattr(mock_ops, "linkforge"):
        mock_ops.linkforge = DynamicModule("bpy.ops.linkforge")
    mock_ops.linkforge.add_empty_link = mock_add_empty_link
    mock_ops.linkforge.calculate_inertia = lambda **kwargs: {"FINISHED"}
    mock_ops.linkforge.generate_collision = lambda **kwargs: {"FINISHED"}
    mock_ops.linkforge.create_sensor = lambda **kwargs: {"FINISHED"}
    mock_ops.linkforge.export_robot_model = lambda **kwargs: {"FINISHED"}

    mock_bpy.ops = mock_ops

    # Importer mocks to simulate object creation
    def mock_import(filepath="", **kwargs):
        name = Path(filepath).stem if filepath else "ImportedObject"
        mesh = MockMesh(name=f"{name}_mesh")
        v = mesh.vertices.add()
        v.co = MockVector(0, 0, 0)
        obj = MockObject(name=name, data=mesh)
        _setup_new_object(obj)
        return {"FINISHED"}

    mock_ops.wm.stl_import = mock_import
    mock_ops.wm.obj_import = mock_import
    mock_ops.wm.gltf_import = mock_import
    mock_ops.import_scene.gltf = mock_import
    mock_ops.wm.collada_import = mock_import
    mock_ops.wm.stl_export = mock_file_op

    # Type and Property Registration
    def mock_prop_func(**kwargs):
        return MockPropertyDescriptor(
            getter=kwargs.get("get"),
            setter=kwargs.get("set"),
            update=kwargs.get("update"),
            default=kwargs.get("default"),
            prop_type=kwargs.get("type"),
        )

    mock_bpy.props.StringProperty = mock_prop_func
    mock_bpy.props.BoolProperty = mock_prop_func
    mock_bpy.props.FloatProperty = mock_prop_func
    mock_bpy.props.IntProperty = mock_prop_func
    mock_bpy.props.EnumProperty = mock_prop_func
    mock_bpy.props.PointerProperty = mock_prop_func
    mock_bpy.props.FloatVectorProperty = mock_prop_func

    def mock_collection_prop(**kwargs):
        coll_type = kwargs.pop("type", None)
        return mock_prop_func(type=lambda: MockCollection(prop_type=coll_type), **kwargs)

    mock_bpy.props.CollectionProperty = mock_collection_prop

    mock_bpy.types.Material = MockMaterial
    mock_bpy.types.Scene = MockScene
    mock_bpy.types.Collection = MockCollection
    mock_bpy.types.Operator = MockOperator
    mock_bpy.types.MaterialSlot = MockMaterialSlot
    mock_bpy.types.WindowManager = MockPropertyGroup
    # Boilerplate for UI classes
    mock_bpy.types.Panel = object
    mock_bpy.types.Menu = object
    mock_bpy.types.Header = object
    mock_bpy.types.UIList = object
    mock_bpy.types.AddonPreferences = object
    mock_bpy.types.Operator = MockOperator
    mock_bpy.types.PropertyGroup = MockPropertyGroup

    mock_mathutils.Quaternion = MockQuaternion

    mock_bpy.utils = DynamicModule("bpy.utils")

    def mock_register_class(cls):
        idname = getattr(cls, "bl_idname", None)
        if not idname or "." not in idname:
            return

        category, name = idname.split(".")

        # Ensure category exists in mock_ops and is a DynamicModule
        cat_mod = getattr(mock_ops, category)
        if not isinstance(cat_mod, DynamicModule):
            cat_mod = DynamicModule(f"bpy.ops.{category}")
            setattr(mock_ops, category, cat_mod)

        # Discover properties in __dict__ or __annotations__
        props: dict[str, dict[str, typing.Any] | MockPropertyDescriptor] = {}
        # 1. Check annotations (for newer Python/Blender style)
        for k, v in getattr(cls, "__annotations__", {}).items():
            if isinstance(v, str) and "bpy.props." in v:
                # If it's a string (due to from __future__ import annotations), we might need to "eval" or mock it
                default_match = re.search(r"default\s*=\s*['\"]([^'\"]+)['\"]", v)
                default_val = default_match.group(1) if default_match else None

                # Numeric defaults
                if not default_val:
                    num_match = re.search(r"default\s*=\s*([\d\.]+)", v)
                    if num_match:
                        default_val = (
                            float(num_match.group(1))
                            if "." in num_match.group(1)
                            else int(num_match.group(1))
                        )

                props[k] = {"default": default_val}
            elif isinstance(v, MockPropertyDescriptor):
                props[k] = v

        # 2. Check __dict__ (standard assignment style)
        for k, v in cls.__dict__.items():
            if isinstance(v, MockPropertyDescriptor):
                v._discover_name(None, cls)
                props[k] = v

        def operator_wrapper(**kwargs):
            # Create instance
            op_instance = cls()
            # Ensure properties from annotations exist on instance if not there
            for k, prop_info in props.items():
                if not hasattr(op_instance, k):
                    if k in kwargs:
                        continue

                    default_val = None
                    if isinstance(prop_info, dict):
                        default_val = prop_info.get("default")
                    elif isinstance(prop_info, MockPropertyDescriptor):
                        default_val = prop_info.default

                    if default_val is not None:
                        setattr(op_instance, k, default_val)
                    else:
                        # Fallback to MagicMock
                        setattr(op_instance, k, MagicMock(name=k))

            # Set properties from kwargs
            for k, v in kwargs.items():
                setattr(op_instance, k, v)

            # Check poll
            if not cls.poll(mock_context):
                return {"CANCELLED"}

            # Run execute
            res = op_instance.execute(mock_context)

            return res

        setattr(cat_mod, name, operator_wrapper)

    def mock_unregister_class(cls):
        if not hasattr(cls, "bl_idname"):
            return
        idname = cls.bl_idname
        if "." not in idname:
            return
        category, name = idname.split(".")
        if hasattr(mock_ops, category):
            cat_mod = getattr(mock_ops, category)
            if hasattr(cat_mod, name):
                delattr(cat_mod, name)

    mock_bpy.utils.register_class = mock_register_class
    mock_bpy.utils.unregister_class = mock_unregister_class

    mock_bmesh = DynamicModule("bmesh")

    class MockBMesh:
        def __init__(self):
            self.verts = MockCollection(
                prop_type=lambda **kwargs: MockPropertyGroup(co=MockVector(), **kwargs)
            )
            self.polygons = MockCollection(
                prop_type=lambda **kwargs: MockPropertyGroup(vertices=MockCollection(), **kwargs)
            )
            self.faces = self.polygons  # Alias for bmesh

        def from_mesh(self, mesh):
            self.verts.clear()
            v_list = []
            for v in mesh.vertices:
                bm_v = self.verts.add()
                bm_v.co = MockVector(v.co) if hasattr(v, "co") else MockVector()
                v_list.append(bm_v)

            self.faces.clear()
            for poly in mesh.polygons:
                bm_f = self.faces.add()
                bm_f.vertices = MockCollection()
                bm_f.vertices.extend([v_list[i] for i in poly.vertices if i < len(v_list)])

        def to_mesh(self, mesh):
            mesh.vertices.clear()
            v_map = {}
            for i, v in enumerate(self.verts):
                m_v = mesh.vertices.add()
                m_v.co = MockVector(v.co)
                v_map[v] = i

            mesh.polygons.clear()
            for f in self.faces:
                m_p = mesh.polygons.add()
                m_p.vertices = MockCollection()
                m_p.vertices.extend([v_map[v] for v in f.vertices if v in v_map])

            # Clear cached dimensions since mesh changed
            if hasattr(mesh, "id_data") and mesh.id_data:
                mesh.id_data._base_dimensions = MockVector(0, 0, 0)

        def free(self):
            pass

    mock_bmesh.new = MockBMesh
    mock_bmesh.ops = DynamicModule("bmesh.ops")

    def mock_create_cube(bm, size=2.0, matrix=None):
        half = size / 2.0
        coords = [
            (-half, -half, -half),
            (half, -half, -half),
            (half, half, -half),
            (-half, half, -half),
            (-half, -half, half),
            (half, -half, half),
            (half, half, half),
            (-half, half, half),
        ]
        bm_verts = []
        for c in coords:
            v = bm.verts.add()
            v.co = MockVector(c)
            bm_verts.append(v)
        # 6 faces
        face_indices = [
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 4, 5, 1),
            (1, 5, 6, 2),
            (2, 6, 7, 3),
            (3, 7, 4, 0),
        ]
        for indices in face_indices:
            f = bm.faces.add()
            f.vertices = MockCollection()
            f.vertices.extend([bm_verts[i] for i in indices])

    mock_bmesh.ops.create_cube = mock_create_cube
    mock_bmesh.ops.convex_hull = lambda bm, **kwargs: None  # Mock hull as identity

    # System Module Promotion (Persistence)
    mock_extras = DynamicModule("bpy_extras")
    mock_io_utils = DynamicModule("bpy_extras.io_utils")
    mock_io_utils.ExportHelper = MockIOHelper
    mock_io_utils.ImportHelper = MockIOHelper
    mock_extras.io_utils = mock_io_utils

    mock_gpu_extras = DynamicModule("gpu_extras")
    mock_batch = DynamicModule("gpu_extras.batch")
    mock_gpu_extras.batch = mock_batch

    sys.modules["bpy"] = typing.cast(types.ModuleType, mock_bpy)
    sys.modules["bpy.data"] = typing.cast(types.ModuleType, mock_data)
    sys.modules["bpy.context"] = typing.cast(types.ModuleType, mock_context)
    mock_context.data = mock_data
    sys.modules["bpy.ops"] = typing.cast(types.ModuleType, mock_ops)
    sys.modules["bpy.props"] = typing.cast(types.ModuleType, mock_bpy.props)
    sys.modules["bpy.types"] = typing.cast(types.ModuleType, mock_bpy.types)
    sys.modules["bpy.app"] = typing.cast(types.ModuleType, mock_app)
    sys.modules["bpy.app.handlers"] = typing.cast(types.ModuleType, mock_app.handlers)
    sys.modules["mathutils"] = typing.cast(types.ModuleType, mock_mathutils)
    sys.modules["bmesh"] = typing.cast(types.ModuleType, mock_bmesh)
    sys.modules["bpy_extras"] = typing.cast(types.ModuleType, mock_extras)
    sys.modules["bpy_extras.io_utils"] = typing.cast(types.ModuleType, mock_io_utils)
    sys.modules["gpu_extras"] = typing.cast(types.ModuleType, mock_gpu_extras)
    sys.modules["gpu_extras.batch"] = typing.cast(types.ModuleType, mock_batch)
    sys.modules["gpu"] = DynamicModule("gpu")

    # Finalize scene-context links
    def _new_collection(name):
        coll = MockCollection()
        coll.name = name
        mock_data.collections.append(coll)
        return coll

    mock_data.collections.new = _new_collection
    mock_data.objects.new = lambda name, data=None: _setup_new_object(
        MockObject(name=name, data=data)
    )
    mock_data.meshes.new = (
        lambda name: mock_data.meshes.append(MockMesh(name=name)) or mock_data.meshes[-1]
    )
    mock_data.meshes.new_from_object = (
        lambda obj, **kwargs: mock_data.meshes.append(MockMesh(name=f"{obj.name}_mesh"))
        or mock_data.meshes[-1]
    )
    mock_data.materials.new = (
        lambda name: mock_data.materials.append(MockMaterial(name=name)) or mock_data.materials[-1]
    )

    return mock_bpy
