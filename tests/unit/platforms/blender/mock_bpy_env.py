import contextlib
import sys
import types
import typing
from pathlib import Path
from unittest.mock import MagicMock


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
            self.__dict__[name] = MagicMock(name=name)
        return self.__dict__[name]

    def __setattr__(self, name, value):
        self.__dict__[name] = value


class MockVector:
    """Mock for mathutils.Vector (mutable)."""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        # Case 1: Object with .x, .y, .z attributes
        if hasattr(x, "x") and hasattr(x, "y") and hasattr(x, "z"):
            self._data = [float(x.x), float(x.y), float(x.z)]
            return

        if isinstance(x, (list, tuple, MockVector)) or (
            hasattr(x, "__getitem__") and not isinstance(x, (str, bytes, int, float))
        ):
            try:
                self._data = [float(x[0]), float(x[1]), float(x[2])]
                return
            except (IndexError, TypeError, AttributeError):
                self._data = [0.0, 0.0, 0.0]
                return

        try:
            self._data = [float(x), float(y), float(z)]
        except (TypeError, ValueError, AttributeError):
            self._data = [0.0, 0.0, 0.0]

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


class MockQuaternion:
    """Mock for mathutils.Quaternion."""

    def __init__(self, *args):
        self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0
        self._euler_hint = None

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

    def __init__(self, x=0, y=0, z=0, order="XYZ"):
        if isinstance(x, (list, tuple, MockVector)):
            super().__init__(x)
            self.order = y if isinstance(y, str) else order
        else:
            super().__init__(x, y, z)
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

        return obj._values.get(name)

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
    "gazebo_plugin_name": "",
    "controllers_yaml_path": "",
    "robot_name": "robot",
    "strict_mode": False,
    "use_ros2_control": False,
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
        if key.startswith("_"):
            raise AttributeError(key)

        for cls in type(self).__mro__:
            if key in cls.__dict__:
                prop = cls.__dict__[key]
                if isinstance(prop, (property, MockPropertyDescriptor)):
                    return prop.__get__(self, type(self))

        if key in self._values:
            return self._values[key]

        if key.startswith("is_robot_") or key.startswith("cmd_") or key.startswith("state_"):
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


class MockCollection(list):
    """Mock for Blender's CollectionProperty items."""

    def __init__(self, prop_type=None, name="Collection"):
        super().__init__()
        self.prop_type = prop_type
        self.name = name
        self.new_from_object = None
        self.id_data = None
        self._objects = None
        self._children = None

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

    def append(self, item):
        if item not in self:
            super().append(item)

    def link(self, item):
        self.append(item)

    def foreach_get(self, attr, data):
        for i, item in enumerate(self):
            val = getattr(item, attr)
            if hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
                for j, v in enumerate(val):
                    data[i * len(val) + j] = v
            else:
                data[i] = val

    def add(self):
        item = self.prop_type() if self.prop_type else MockPropertyGroup()
        self.append(item)
        return item

    def new(self, name=None, data=None, type=None):  # noqa: A002
        if self.prop_type is MockObject:
            item = MockObject(name=name, data=data)
        elif self.prop_type:
            item = self.prop_type(name=name)
        else:
            item = MockPropertyGroup(name=name)

        if type and hasattr(item, "type"):
            item.type = type
        self.append(item)
        return item

    def __getitem__(self, key):
        if isinstance(key, (int, slice)):
            return super().__getitem__(key)
        for item in self:
            if getattr(item, "name", None) == key:
                return item
        raise KeyError(key)

    def get(self, name, default=None):
        for item in self:
            if getattr(item, "name", None) == name:
                return item
        return default

    def remove(self, item, do_unlink=True):
        if isinstance(item, int):
            if 0 <= item < len(self):
                self.pop(item)
        elif item in self:
            super().remove(item)

    def clear(self):
        super().clear()

    def __contains__(self, key):
        if isinstance(key, str):
            return any(getattr(item, "name", None) == key for item in self)
        return super().__contains__(key)

    @property
    def bl_rna(self):
        return MagicMock()

    def __getattr__(self, key):
        if key.startswith("_"):
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


class MockMaterialSlot(MockPropertyGroup):
    def __init__(self, material=None, **kwargs):
        super().__init__(**kwargs)
        self.material = material


class MockMesh(MockPropertyGroup):
    def __init__(self, name="Mesh"):
        super().__init__(name=name)
        self.name = name
        self.vertices = MockCollection(prop_type=lambda: MockPropertyGroup(co=MockVector()))
        self.polygons = MockCollection(prop_type=lambda: MockPropertyGroup(vertices=[]))
        self.materials = MockCollection(prop_type=MockMaterial)

    def transform(self, matrix):
        pass

    def calc_loop_triangles(self):
        self.loop_triangles = MockCollection(prop_type=lambda: MockPropertyGroup(vertices=[]))
        for poly in self.polygons:
            if len(poly.vertices) == 4:
                t1 = self.loop_triangles.add()
                t1.vertices = [poly.vertices[0], poly.vertices[1], poly.vertices[2]]
                t2 = self.loop_triangles.add()
                t2.vertices = [poly.vertices[0], poly.vertices[2], poly.vertices[3]]
            else:
                t = self.loop_triangles.add()
                t.vertices = list(poly.vertices)

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
        self.constraints = MockCollection()
        self.modifiers = MockCollection()
        self.children = MockCollection()
        self.users_collection = MockCollection()
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

    @property
    def matrix_local(self):
        return self._matrix_local

    @matrix_local.setter
    def matrix_local(self, value):
        self._matrix_local = MockMatrix(value)
        self._location.x = self._matrix_local.data[0][3]
        self._location.y = self._matrix_local.data[1][3]
        self._location.z = self._matrix_local.data[2][3]
        self._scale.x = self._matrix_local.data[0][0]
        self._scale.y = self._matrix_local.data[1][1]
        self._scale.z = self._matrix_local.data[2][2]

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
        return MockVector(
            base_dim.x * world_scale.x, base_dim.y * world_scale.y, base_dim.z * world_scale.z
        )

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
        return new_obj

    def evaluated_get(self, depsgraph):
        return self

    def to_mesh(self, **kwargs):
        return self.data

    def to_mesh_clear(self):
        pass


class MockScene(MockPropertyGroup):
    """Mock for bpy.types.Scene."""

    def __init__(self, name="Scene", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.objects = MockCollection(prop_type=MockObject)
        self.collection = MockCollection()
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


class MockIOHelper:
    def invoke(self, context, event):
        return {"FINISHED"}


mock_mathutils = DynamicModule("mathutils")
mock_mathutils.Vector = MockVector
mock_mathutils.Matrix = MockMatrix
mock_mathutils.Euler = MockEuler
mock_mathutils.Quaternion = MockQuaternion

mock_bpy = DynamicModule("bpy")
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

mock_data = MockPropertyGroup(name="Data")
mock_context = MagicMock(name="bpy.context")
mock_ops = DynamicModule("bpy.ops")
mock_ops.mesh = DynamicModule("bpy.ops.mesh")
mock_ops.object = DynamicModule("bpy.ops.object")
mock_ops.wm = DynamicModule("bpy.ops.wm")
mock_ops.export_scene = DynamicModule("bpy.ops.export_scene")
mock_app = DynamicModule("bpy.app")

_is_real_blender = False
try:
    import bpy as _real_bpy

    if (
        hasattr(_real_bpy, "app")
        and hasattr(_real_bpy.app, "binary_path")
        and _real_bpy.app.binary_path
    ):
        _is_real_blender = True
except (ImportError, AttributeError):
    pass

if not _is_real_blender:
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
    mock_data.collections = MockCollection()
    mock_data.meshes = MockCollection(prop_type=MockMesh)
    mock_data.actions = MockCollection()
    mock_data.node_groups = MockCollection()
    mock_data.cameras = MockCollection(prop_type=MockCamera)
    mock_data.lights = MockCollection(prop_type=MockLight)
    mock_data.materials = MockCollection(prop_type=MockMaterial)
    mock_data.scenes = MockCollection(prop_type=MockScene)

    mock_data.objects.clear()
    mock_data.meshes.clear()
    mock_data.materials.clear()
    mock_data.collections.clear()

    # Reset Global State for this test run
    mock_data.objects.clear()
    mock_data.meshes.clear()
    mock_data.materials.clear()
    mock_data.collections.clear()

    active_scene = MockScene(name="Scene")
    mock_data.scenes.clear()
    mock_data.scenes.append(active_scene)

    # Setup Context
    mock_bpy.data = mock_data
    mock_bpy.context = mock_context
    mock_bpy.app = mock_app
    mock_bpy.types = mock_bpy.types
    mock_bpy.props = mock_bpy.props

    mock_context.scene = active_scene
    mock_context.active_object = None
    mock_context.ops = mock_ops
    mock_context.selected_objects = []

    mock_view_layer = typing.cast(MockPropertyGroup, active_scene.view_layers[0])
    mock_view_layer.objects = mock_data.objects

    mock_context.view_layer = mock_view_layer

    class ObjectsCollection(MockCollection):
        @property
        def active(self):
            return mock_context.active_object

        @active.setter
        def active(self, val):
            mock_context.active_object = val

    mock_view_layer.objects = ObjectsCollection(prop_type=MockObject)
    for obj in mock_data.objects:
        mock_view_layer.objects.append(obj)
    mock_data.objects = mock_view_layer.objects

    mock_context.evaluated_depsgraph_get = lambda: MagicMock(name="Depsgraph")
    mock_context.window_manager = MockPropertyGroup()

    # Setup handlers and timers
    mock_app.timers = MagicMock(name="Timers")
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
        mock_data.objects.append(obj)
        mock_context.active_object = obj
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = obj
        return obj

    def mock_empty_add(type="PLAIN_AXES", location=(0, 0, 0), **kwargs):  # noqa: A002
        obj = MockObject(name="Empty")
        obj.type = "EMPTY"
        obj.empty_display_type = type
        _setup_new_object(obj, location)
        return {"FINISHED"}

    def mock_cube_add(size=2.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="CubeMesh")
        [mesh.vertices.add() for _ in range(8)]
        for _ in range(6):
            p = mesh.polygons.add()
            p.vertices = [0, 1, 2, 3]  # Mock quad
        obj = MockObject(name="Cube", data=mesh)
        obj._base_dimensions = MockVector(1.0, 1.0, 1.0)
        obj.dimensions = MockVector(size, size, size)
        _setup_new_object(obj, location)
        mock_data.meshes.append(mesh)
        return {"FINISHED"}

    def mock_sphere_add(radius=1.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="SphereMesh")
        [mesh.vertices.add() for _ in range(482)]
        for _ in range(480):
            p = mesh.polygons.add()
            p.vertices = [0, 1, 2, 3]  # Mock quad
        obj = MockObject(name="Sphere", data=mesh)
        obj._base_dimensions = MockVector(1.0, 1.0, 1.0)
        obj.dimensions = MockVector(radius * 2, radius * 2, radius * 2)
        _setup_new_object(obj, location)
        mock_data.meshes.append(mesh)
        return {"FINISHED"}

    def mock_cylinder_add(radius=1.0, depth=2.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="CylinderMesh")
        [mesh.vertices.add() for _ in range(66)]
        for _ in range(64):
            p = mesh.polygons.add()
            p.vertices = [0, 1, 2, 3]  # Mock quad
        obj = MockObject(name="Cylinder", data=mesh)
        obj._base_dimensions = MockVector(1.0, 1.0, 1.0)
        obj.dimensions = MockVector(radius * 2, radius * 2, depth)
        _setup_new_object(obj, location)
        mock_data.meshes.append(mesh)
        return {"FINISHED"}

    def mock_monkey_add(**kwargs):
        mesh = MockMesh(name="MonkeyMesh")
        # Suzanne: 1200 verts / 1100 faces – well outside all primitive thresholds
        # (sphere range is 240-1000 verts, so 1200 is clearly complex mesh)
        [mesh.vertices.add() for _ in range(1200)]
        for _ in range(1100):
            p = mesh.polygons.add()
            p.vertices = [0, 1, 2]  # Triangles (not quads)
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

    def mock_transform_apply(location=False, rotation=False, scale=False, **kwargs):
        obj = mock_context.active_object
        if not obj:
            return {"FINISHED"}

        if scale:
            # Bake scale into base_dimensions
            if hasattr(obj, "_base_dimensions"):
                obj._base_dimensions.x *= obj.scale.x
                obj._base_dimensions.y *= obj.scale.y
                obj._base_dimensions.z *= obj.scale.z
            obj.scale = MockVector(1, 1, 1)

        return {"FINISHED"}

    mock_ops.object.transform_apply = mock_transform_apply

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

    def mock_join():
        active = mock_context.active_object
        if not active:
            selected = [obj for obj in mock_data.objects if getattr(obj, "_selected", False)]
            if not selected:
                return {"CANCELLED"}
            active = selected[-1]

        to_remove = [
            obj for obj in mock_data.objects if getattr(obj, "_selected", False) and obj != active
        ]
        for obj in to_remove:
            if obj in mock_data.objects:
                mock_data.objects.remove(obj)
        return {"FINISHED"}

    mock_ops.object.join = mock_join
    mock_ops.object.parent_set = lambda **kwargs: {"FINISHED"}
    mock_ops.object.parent_clear = lambda **kwargs: {"FINISHED"}
    mock_ops.object.delete = lambda **kwargs: {"FINISHED"}

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

    # Global Math and Extra Modules
    mock_mathutils = DynamicModule("mathutils")
    mock_mathutils.Vector = MockVector
    mock_mathutils.Matrix = MockMatrix
    mock_mathutils.Euler = MockEuler
    mock_mathutils.Quaternion = MockQuaternion

    mock_bmesh = DynamicModule("bmesh")

    class MockBMesh:
        def __init__(self):
            self.verts = MockCollection(prop_type=lambda: MockPropertyGroup(co=MockVector()))
            self.faces = MockCollection(prop_type=lambda: MockPropertyGroup(verts=[]))

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
                bm_f.verts = [v_list[i] for i in poly.vertices if i < len(v_list)]

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
                # Link vertices by index
                m_p.vertices = [v_map.get(v, 0) for v in getattr(f, "verts", [])]

        def free(self):
            pass

    mock_bmesh.new = lambda: MockBMesh()
    mock_bmesh.ops = DynamicModule("bmesh.ops")

    def mock_create_cube(bm, size=2.0, **kwargs):
        # Create 8 vertices for a cube
        s = size / 2.0
        coords = [
            (-s, -s, -s),  # 0
            (s, -s, -s),  # 1
            (s, s, -s),  # 2
            (-s, s, -s),  # 3
            (-s, -s, s),  # 4
            (s, -s, s),  # 5
            (s, s, s),  # 6
            (-s, s, s),  # 7
        ]
        verts = []
        for c in coords:
            v = bm.verts.add()
            v.co = MockVector(c)
            verts.append(v)

        face_indices = [
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 1, 5, 4),
            (1, 2, 6, 5),
            (2, 3, 7, 6),
            (3, 0, 4, 7),
        ]
        faces = []
        for indices in face_indices:
            f = bm.faces.add()
            f.verts = [verts[i] for i in indices]
            faces.append(f)

        return (verts, faces)

    mock_bmesh.ops.create_cube = mock_create_cube
    mock_bmesh.ops.create_uvsphere = lambda bm, **kwargs: (
        [bm.verts.add() for _ in range(482)],
        [bm.faces.add() for _ in range(480)],
    )
    mock_bmesh.ops.convex_hull = lambda bm, **kwargs: None

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
