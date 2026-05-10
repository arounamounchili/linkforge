import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

# -----------------------------------------------------------------------------
# GLOBAL STATE CONTAINER
# -----------------------------------------------------------------------------


class MockState:
    """Container for the global state of the mocked Blender environment."""

    def __init__(self):
        self.data = None
        self.context = None
        self.types = None
        self.props = None
        self.ops = None
        self.app = None


state = MockState()


# -----------------------------------------------------------------------------
# HIGH FIDELITY BLENDER MOCKS
# -----------------------------------------------------------------------------


class DynamicModule(types.ModuleType):
    """Module that returns a MagicMock for any missing attribute."""

    def __getattr__(self, name):
        if name not in self.__dict__:
            self.__dict__[name] = MagicMock(name=name)
        return self.__dict__[name]

    def __setattr__(self, name, value):
        self.__dict__[name] = value


class MockVector:
    """Mock for mathutils.Vector (mutable)."""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        if isinstance(x, (MockVector, list, tuple)) and len(x) >= 3:
            if hasattr(x, "x"):
                self._data = [float(x.x), float(x.y), float(x.z)]
            else:
                self._data = [float(x[0]), float(x[1]), float(x[2])]
        elif hasattr(x, "__len__") and not isinstance(x, (str, bytes)) and len(x) >= 3:
            self._data = [float(x[0]), float(x[1]), float(x[2])]
        else:
            # Fallback for single values
            try:
                self._data = [float(x), float(y), float(z)]
            except (TypeError, ValueError):
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

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return MockVector(self.x * other, self.y * other, self.z * other)
        return self

    def __truediv__(self, other):
        if isinstance(other, (int, float)) and other != 0:
            return MockVector(self.x / other, self.y / other, self.z / other)
        return self

    def __repr__(self):
        return f"Vector(({self.x}, {self.y}, {self.z}))"

    def rotation_difference(self, other):
        return MockQuaternion()


class MockQuaternion:
    """Mock for mathutils.Quaternion."""

    def __init__(self, *args):
        self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0

    def copy(self):
        q = MockQuaternion()
        q.w, q.x, q.y, q.z = self.w, self.x, self.y, self.z
        return q

    def to_euler(self, order="XYZ"):
        return MockEuler(self.x, self.y, self.z, order)

    def to_matrix(self):
        """Mock for mathutils.Quaternion.to_matrix."""
        res = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            res[i][i] = 1.0
        return MockMatrix(res)


class MockEuler(MockVector):
    """Mock for mathutils.Euler."""

    def __init__(self, x=0, y=0, z=0, order="XYZ"):
        if isinstance(x, (list, tuple, MockVector)):
            super().__init__(x)
            # If order was passed as second positional arg, it might be in 'y'
            self.order = y if isinstance(y, str) else order
        else:
            super().__init__(x, y, z)
            self.order = order

    def __repr__(self):
        return f"Euler(({self.x}, {self.y}, {self.z}), '{self.order}')"

    def to_matrix(self):
        m = MockMatrix.Identity(3)
        m._euler_hint = self
        return m

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
            # Support arbitrary sizes (2x2, 3x3, 4x4)
            rows = len(data)
            cols = len(data[0])
            self.data = [[float(data[i][j]) for j in range(cols)] for i in range(rows)]
        else:
            self.data = data

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
        """Mock for mathutils.Matrix.Rotation using the to_euler hack."""
        m = MockMatrix.Identity(size)
        if size >= 2:
            m.data[0][0] = 0.9  # Trigger the to_euler hack
            if axis == "X":
                m.data[0][1] = float(angle)
                m.data[0][2] = 0.0
                m.data[1][0] = 0.0
            elif axis == "Y":
                m.data[0][1] = 0.0
                m.data[0][2] = float(angle)
                m.data[1][0] = 0.0
            elif axis == "Z":
                m.data[0][1] = 0.0
                m.data[0][2] = 0.0
                m.data[1][0] = float(angle)
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
        """Property alias for Blender mathutils.Matrix parity."""
        return MockVector(self.data[0][3], self.data[1][3], self.data[2][3])

    def to_translation(self):
        return self.translation

    def to_quaternion(self):
        """Mock for mathutils.Matrix.to_quaternion."""
        return getattr(self, "_quaternion_hint", MockQuaternion())

    def to_euler(self, order="XYZ"):
        return getattr(self, "_euler_hint", MockEuler(0, 0, 0, order=order))

    def to_4x4(self):
        if len(self.data) == 4:
            return self
        res = MockMatrix.Identity(4)
        for i in range(min(len(self.data), 4)):
            for j in range(min(len(self.data[0]), 4)):
                res.data[i][j] = self.data[i][j]
        return res

    def to_3x3(self):
        """Mock for mathutils.Matrix.to_3x3."""
        res = [[0.0] * 3 for _ in range(3)]
        for i in range(min(len(self.data), 3)):
            for j in range(min(len(self.data[0]), 3)):
                res[i][j] = self.data[i][j]
        return MockMatrix(res)

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

    def __matmul__(self, other):
        if isinstance(other, MockVector):
            # 4x4 matrix * vector multiplication
            res = [0.0, 0.0, 0.0, 0.0]
            vec = [other.x, other.y, other.z, 1.0]
            for i in range(4):
                for j in range(4):
                    res[i] += self.data[i][j] * vec[j]
            return MockVector(res[0], res[1], res[2])

        if not isinstance(other, MockMatrix):
            return self
        res = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    res[i][j] += self.data[i][k] * (
                        other.data[k][j] if hasattr(other, "data") else 0
                    )
        return MockMatrix(res)

    def __eq__(self, other):
        if not isinstance(other, MockMatrix):
            return False
        return self.data == other.data

    def copy(self):
        return MockMatrix([list(r) for r in self.data])


class MockPropertyDescriptor:
    """Mock for Blender property descriptors (IntProperty, PointerProperty, etc.)."""

    def __init__(self, **kwargs):
        self.name = kwargs.get("name", "")
        self.prop_type = kwargs.get("prop_type")
        self.default = kwargs.get("default")
        self.min = kwargs.get("min")
        self.max = kwargs.get("max")
        self.update = kwargs.get("update")
        self.setter = kwargs.get("setter")

    def __set_name__(self, owner, name):
        self.name = name

    def __get__(self, obj, cls):
        if obj is None:
            return self
        if not hasattr(obj, "_values"):
            obj._values = {}

        # If name is not set or default, try to find it from the class (dynamic attachment)
        if not self.name or self.name == "unnamed_prop":
            for k, v in cls.__dict__.items():
                if v is self:
                    self.name = k
                    break

        if self.name not in obj._values:
            if self.prop_type:
                try:
                    # Specialized behavior for collections
                    if self.prop_type == MockCollection:
                        val = MockCollection(prop_type=MockPropertyGroup)
                    else:
                        val = self.prop_type()

                    if hasattr(val, "id_data"):
                        val.id_data = obj
                    obj._values[self.name] = val
                except Exception:
                    obj._values[self.name] = self.default
            else:
                obj._values[self.name] = self.default

        if self.getter:
            return self.getter(obj)
        return obj._values.get(self.name)

    def __set__(self, obj, value):
        if obj is None:
            return
        if not hasattr(obj, "_values"):
            obj._values = {}

        # Convert value to expected type if it's a known Blender math type
        if self.prop_type == MockVector and not isinstance(value, MockVector):
            value = MockVector(value)
        elif self.prop_type == MockEuler and not isinstance(value, MockEuler):
            value = MockEuler(value)

        obj._values[self.name] = value
        if self.setter:
            self.setter(obj, value)
        if self.update:
            # Blender update callbacks receive (self, context)
            # In mocks, context might be None
            try:
                self.update(obj, sys.modules.get("bpy").context)
            except Exception:
                self.update(obj, None)


class PropertyMetaclass(type):
    """Metaclass that automatically handles Blender-style property annotations."""

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
                # Handle simple type hints as properties
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


# Map of property names to their default values to avoid truthy MagicMocks
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
    "link_name": "",
    "joint_name": "",
    "child_link": None,
    "parent_link": None,
}


class MockPropertyGroup(metaclass=PropertyMetaclass):
    """Base class for mocked Blender PropertyGroups."""

    def __init__(self, **kwargs):
        self.__dict__["_values"] = {}
        self.__dict__["id_data"] = None
        self.__dict__["name"] = kwargs.get("name", "Unnamed")
        for k, v in kwargs.items():
            setattr(self, k, v)

    def __setattr__(self, name, value):
        # 1. If it's a property or descriptor defined in the class (or parents), use it.
        # This is critical for MockObject properties (location, dimensions, etc.)
        # and MockPropertyDescriptor instances (Blender props).
        for cls in type(self).__mro__:
            if name in cls.__dict__:
                prop = cls.__dict__[name]
                if isinstance(prop, (property, MockPropertyDescriptor)):
                    super().__setattr__(name, value)
                    return

        # 2. Handle internal mock state and known Blender attributes
        if name.startswith("_") or name in ("id_data", "name"):
            super().__setattr__(name, value)
        else:
            # 3. Dynamic attributes go to _values to simulate RNA
            self._values[name] = value

    def clear(self):
        self._values.clear()

    @property
    def bl_rna(self):
        return MagicMock()

    def __getattr__(self, key):
        if key.startswith("_"):
            raise AttributeError(key)

        # 1. Check if we have a stored value (RNA-like behavior)
        if key in self._values:
            return self._values[key]

        # 2. Force robot-detection properties to False if not set
        if key.startswith("is_robot_"):
            return False

        # 2. Check defaults
        if key in DEFAULT_PROPERTY_VALUES:
            val = DEFAULT_PROPERTY_VALUES[key]
            # Ensure math types are returned as fresh mocks/objects and stored
            if isinstance(val, tuple) and len(val) == 3:
                vec = MockVector(val)
                self._values[key] = vec
                return vec
            return val

        # 3. Handle special Blender behavior (id_data, bl_rna, get)
        if key == "id_data":
            return None
        if key == "bl_rna":
            return MagicMock(name="bl_rna")
        if key == "get":
            return self._mock_get

        # 4. Critical Blender properties that should return None instead of MagicMock
        if key in (
            "active_object",
            "object",
            "mesh",
            "material",
            "parent",
            "active_bone",
            "active_joint",
        ):
            return None

        if key in RESERVED_RNA_PROPS:
            raise AttributeError(f"RNA property '{key}' not found on '{self.name}'")

        # Fallback to MagicMock but store it to ensure identity persistence
        val = MagicMock(name=key)
        self._values[key] = val
        return val

    def _mock_get(self, key, default=None):
        """Simulate Blender's obj.get("prop") for custom properties."""
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

    def __init__(self, items=None, prop_type=None):
        super().__init__(items or [])
        self.prop_type = prop_type
        self.new = None
        self.new_from_object = None

    def add(self):
        item = self.prop_type() if self.prop_type else MockPropertyGroup()
        self.append(item)
        return item

    def __getitem__(self, key):
        if isinstance(key, (int, slice)):
            return super().__getitem__(key)
        # Search by name for string keys
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
        """Support 'in' operator for named items or objects."""
        if isinstance(key, str):
            return any(getattr(item, "name", None) == key for item in self)
        return super().__contains__(key)

    @property
    def bl_rna(self):
        return MagicMock()

    def __getattr__(self, key):
        if key.startswith("_"):
            raise AttributeError(key)
        # Cache the MagicMock on the instance to allow subsequent assignment or access
        val = MagicMock(name=key)
        setattr(self, key, val)
        return val

    def link(self, obj):
        if obj not in self:
            self.append(obj)

    def unlink(self, obj):
        if obj in self:
            self.remove(obj)


class MockHandlers:
    """Mock for bpy.app.handlers."""

    def __init__(self):
        self.load_post = []
        self.save_pre = []
        self.save_post = []
        self.depsgraph_update_post = []
        self.depsgraph_update_pre = []
        self.render_pre = []
        self.render_post = []


class MockMaterialSlot(MockPropertyGroup):
    """Mock for bpy.types.MaterialSlot."""

    def __init__(self, material=None, **kwargs):
        super().__init__(**kwargs)
        self.material = material


class MockMesh(MockPropertyGroup):
    """Mock for bpy.types.Mesh."""

    def __init__(self, name="Mesh", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.vertices = MockCollection()
        self.polygons = MockCollection()
        self.materials = MockCollection()

    def transform(self, matrix):
        """Mock for applying a transformation matrix to mesh data."""
        pass


class MockNodeInput(MockPropertyGroup):
    """Mock for bpy.types.NodeSocket."""

    def __init__(self, name="Input", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.default_value = (0.8, 0.8, 0.8, 1.0)


class MockNode(MockPropertyGroup):
    """Mock for bpy.types.Node."""

    def __init__(self, name="Node", node_type="BSDF_PRINCIPLED", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.type = node_type
        self.inputs = MockCollection(prop_type=MockNodeInput)


class MockNodeTree(MockPropertyGroup):
    """Mock for bpy.types.NodeTree."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.nodes = MockCollection(prop_type=MockNode)


class MockMaterial(MockPropertyGroup):
    """Mock for bpy.types.Material."""

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
            # Create default Principled BSDF node
            bsdf = self.node_tree.nodes.add()
            bsdf.name = "Principled BSDF"
            bsdf.type = "BSDF_PRINCIPLED"
            # Add Base Color input with default_value
            base_color = bsdf.inputs.add()
            base_color.name = "Base Color"
            base_color.default_value = (0.8, 0.8, 0.8, 1.0)


class MockObject(MockPropertyGroup):
    """Mock for bpy.types.Object."""

    def __init__(self, name="Object", data=None, **kwargs):
        super().__init__(**kwargs)
        self._name = name
        self.data = data
        self.type = "MESH" if (data and not isinstance(data, MagicMock)) else "EMPTY"
        self._parent = None
        self._matrix_world = MockMatrix.Identity(4)
        self._matrix_local = MockMatrix.Identity(4)
        self.matrix_basis = MockMatrix.Identity(4)
        self.matrix_parent_inverse = MockMatrix.Identity(4)
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

        # linkforge properties should only be pre-initialized if no descriptor exists on the class
        if not any("linkforge" in c.__dict__ for c in type(self).__mro__):
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
        # Sync with linkforge properties if they exist
        if hasattr(self, "linkforge") and hasattr(self.linkforge, "link_name"):
            self.linkforge.link_name = str(value)
        if hasattr(self, "linkforge_joint") and hasattr(self.linkforge_joint, "joint_name"):
            self.linkforge_joint.joint_name = str(value)

    @property
    def matrix_world(self):
        # In real Blender, world matrix depends on parent
        if self.parent:
            return self.parent.matrix_world @ self.matrix_local
        return self._matrix_world

    @matrix_world.setter
    def matrix_world(self, value):
        self._matrix_world = MockMatrix(value)

    @property
    def matrix_local(self):
        return self._matrix_local

    @matrix_local.setter
    def matrix_local(self, value):
        self._matrix_local = MockMatrix(value)

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
        """Update matrix_local based on location, rotation, and scale."""
        # Simplified translation part
        self.matrix_local.data[0][3] = self._location.x
        self.matrix_local.data[1][3] = self._location.y
        self.matrix_local.data[2][3] = self._location.z
        self.matrix_world = self.matrix_local.copy()

    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, value):
        self._scale = MockVector(value)

    @property
    def dimensions(self):
        # Calculate world scale (inherited from parents)
        world_scale = self.scale.copy()
        p = self.parent
        while p:
            world_scale.x *= p.scale.x
            world_scale.y *= p.scale.y
            world_scale.z *= p.scale.z
            p = p.parent

        if hasattr(self, "_base_dimensions") and self._base_dimensions.length > 1e-6:
            return MockVector(
                self._base_dimensions.x * world_scale.x,
                self._base_dimensions.y * world_scale.y,
                self._base_dimensions.z * world_scale.z,
            )
        # Fallback to direct value if base is not set
        return self._values.get("dimensions", MockVector(0, 0, 0))

    @dimensions.setter
    def dimensions(self, value):
        v = MockVector(value)
        self._values["dimensions"] = v
        if hasattr(self, "_base_dimensions") and self._base_dimensions.length > 1e-6:
            # Sync scale: scale = dimensions / base_dimensions
            self._scale = MockVector(
                v.x / self._base_dimensions.x if self._base_dimensions.x > 1e-6 else 1.0,
                v.y / self._base_dimensions.y if self._base_dimensions.y > 1e-6 else 1.0,
                v.z / self._base_dimensions.z if self._base_dimensions.z > 1e-6 else 1.0,
            )
        else:
            # Treatment for uninitialized base dimensions: value is base at scale 1.0
            self._base_dimensions = v.copy()
            self._scale = MockVector(1, 1, 1)

        # Ensure matrix_world/local are updated to reflect scale changes
        # (Simplified: assume identity rotation/translation for now)
        self.matrix_local.data[0][0] = self._scale.x
        self.matrix_local.data[1][1] = self._scale.y
        self.matrix_local.data[2][2] = self._scale.z
        self.matrix_world = self.matrix_local.copy()

    @property
    def material_slots(self):
        slots = MockCollection(prop_type=MockMaterialSlot)
        if self.data and hasattr(self.data, "materials"):
            for mat in self.data.materials:
                slot = slots.add()
                slot.material = mat
        return slots

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        # Update parent-child relationship for high-fidelity scene tree
        if self._parent and hasattr(self._parent, "children") and self in self._parent.children:
            self._parent.children.remove(self)

        self._parent = value

        if value and hasattr(value, "children") and self not in value.children:
            value.children.append(self)

    def select_get(self):
        return True

    def select_set(self, state):
        pass

    def transform(self, matrix):
        """Mock for applying a transformation matrix."""
        pass

    def copy(self):
        new_obj = MockObject(name=f"{self.name}_copy", data=self.data)
        new_obj.matrix_world = self.matrix_world.copy()
        return new_obj

    def evaluated_get(self, depsgraph):
        return self


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
        self.view_layers.append(MagicMock(name="ViewLayer"))
        self.cursor = MagicMock(name="Cursor")
        self.cursor.location = MockVector(0, 0, 0)
        self.cursor.rotation_euler = MockEuler(0, 0, 0)

        # Pre-initialize LinkForge properties
        self.linkforge = MockPropertyGroup(name="linkforge")
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
    """Mock for bpy_extras.io_utils.ExportHelper / ImportHelper."""

    def invoke(self, context, event):
        return {"FINISHED"}


# Module-level persistent mocks for cross-test access
mock_data = MockPropertyGroup(name="Data")
mock_context = MockPropertyGroup(name="Context")
mock_app = DynamicModule("bpy.app")


def setup_mock_bpy():
    """Initializes the entire Blender mock environment."""
    # Ensure modules exist in sys.modules
    if "bpy" not in sys.modules:
        sys.modules["bpy"] = DynamicModule("bpy")
    if "bpy.props" not in sys.modules:
        sys.modules["bpy.props"] = DynamicModule("bpy.props")
    if "bpy.types" not in sys.modules:
        sys.modules["bpy.types"] = DynamicModule("bpy.types")
    if "mathutils" not in sys.modules:
        sys.modules["mathutils"] = DynamicModule("mathutils")

    mock_bpy = sys.modules["bpy"]
    mock_props = sys.modules["bpy.props"]
    mock_types = sys.modules["bpy.types"]
    mock_mathutils = sys.modules["mathutils"]

    # Refresh modules and objects
    # Note: We preserve mock_types because Blender registrations are persistent
    for mod in [mock_bpy, mock_props, mock_mathutils]:
        for key in list(mod.__dict__.keys()):
            if not key.startswith("__"):
                del mod.__dict__[key]

    mock_data.clear()
    mock_context.clear()
    mock_app = DynamicModule("bpy.app")
    mock_app.timers = MagicMock(name="Timers")
    mock_app.handlers = MockHandlers()
    mock_app.version = (4, 2, 0)
    mock_app.driver_namespace = {}

    # Update global state for cross-module access
    state.data = mock_data
    state.context = mock_context
    state.types = mock_types
    state.props = mock_props
    state.app = mock_app

    # Assign to modules
    mock_bpy.data = mock_data
    mock_bpy.context = mock_context
    mock_bpy.app = mock_app
    mock_bpy.ops = DynamicModule("bpy.ops")
    mock_bpy.utils = MagicMock(name="Utils")

    # Map sub-modules in sys.modules
    sys.modules["bpy.app"] = mock_app
    sys.modules["bpy.app.handlers"] = mock_app.handlers
    sys.modules["bpy.ops"] = mock_bpy.ops
    sys.modules["bpy.data"] = mock_data
    sys.modules["bpy.context"] = mock_context
    sys.modules["bpy.props"] = mock_props
    sys.modules["bpy.types"] = mock_types

    mock_types.PropertyGroup = MockPropertyGroup
    mock_types.Object = MockObject
    mock_types.Mesh = MockMesh
    mock_types.Material = MockMaterial
    mock_types.Scene = MockScene
    mock_types.Collection = MockCollection
    mock_types.Operator = MockOperator
    mock_types.MaterialSlot = MockMaterialSlot
    mock_types.Panel = object
    mock_types.Menu = object
    mock_types.AddonPreferences = object
    mock_types.Header = object
    mock_types.UIList = object
    mock_types.WindowManager = MockPropertyGroup

    def mock_prop_func(**kwargs):
        return MockPropertyDescriptor(
            getter=kwargs.get("get"),
            setter=kwargs.get("set"),
            update=kwargs.get("update"),
            default=kwargs.get("default"),
            prop_type=kwargs.get("type"),
        )

    mock_props.StringProperty = mock_prop_func
    mock_props.BoolProperty = mock_prop_func
    mock_props.FloatProperty = mock_prop_func
    mock_props.IntProperty = mock_prop_func
    mock_props.EnumProperty = mock_prop_func
    mock_props.PointerProperty = mock_prop_func
    mock_props.FloatVectorProperty = mock_prop_func
    mock_props.CollectionProperty = lambda **kwargs: mock_prop_func(
        type=lambda: MockCollection(prop_type=kwargs.get("type")), **kwargs
    )

    mock_data.objects = MockCollection(prop_type=MockObject)

    def _new_obj_item(name, data=None):
        obj = MockObject(name=name, data=data)
        mock_data.objects.append(obj)
        return obj

    mock_data.objects.new = _new_obj_item

    mock_data.meshes = MockCollection(prop_type=MockMesh)

    def _new_mesh_item(name):
        m = MockMesh(name=name)
        mock_data.meshes.append(m)
        return m

    mock_data.meshes.new = _new_mesh_item
    mock_data.meshes.new_from_object = lambda obj, **kwargs: _new_mesh_item(f"{obj.name}_mesh")

    mock_data.collections = MockCollection(prop_type=MockCollection)
    mock_data.collections.new = lambda name: MockCollection()

    mock_data.scenes = MockCollection(prop_type=MockScene)
    active_scene = MockScene(name="Scene")
    mock_data.scenes.append(active_scene)

    mock_data.materials = MockCollection(prop_type=MockMaterial)

    def _new_material_item(name):
        mat = MockMaterial(name=name)
        mock_data.materials.append(mat)
        return mat

    mock_data.materials.new = _new_material_item

    # Initialize context with the active scene
    mock_context.scene = active_scene
    mock_context.active_object = None
    mock_context.selected_objects = []
    mock_context.view_layer = active_scene.view_layers[0]
    mock_context.view_layer.objects = mock_data.objects
    mock_context.evaluated_depsgraph_get = lambda: MagicMock(name="Depsgraph")

    # Initialize view_layer with a fallback to avoid NoneType errors
    mock_view_layer = MagicMock(name="ViewLayer")
    mock_view_layer.objects = mock_data.objects

    def _update_view_layer():
        # Propagation pass: root to leaves
        processed = set()

        def update_obj(obj):
            if obj in processed:
                return
            if obj.parent:
                update_obj(obj.parent)
                obj.matrix_world = obj.parent.matrix_world @ obj.matrix_local
            else:
                obj.matrix_world = obj.matrix_local.copy()
            processed.add(obj)

        for obj in mock_data.objects:
            update_obj(obj)

    # Initialize view_layer with a fallback to avoid NoneType errors
    mock_view_layer = MagicMock(name="ViewLayer")
    mock_view_layer.objects = mock_data.objects
    mock_view_layer.update = _update_view_layer

    mock_scene = mock_data.scenes[0]
    mock_scene.view_layers.clear()
    mock_scene.view_layers.append(mock_view_layer)

    mock_context.view_layer = mock_view_layer
    mock_context.scene = mock_scene
    mock_context.active_object = None
    mock_context.evaluated_depsgraph_get = lambda: MagicMock(name="Depsgraph")
    mock_context.window_manager = MockPropertyGroup()

    mock_app.driver_namespace = {}
    mock_app.version = (4, 2, 0)

    mock_bpy.props = mock_props
    mock_bpy.types = mock_types
    mock_bpy.context = mock_context
    mock_bpy.data = mock_data
    mock_ops = DynamicModule("bpy.ops")
    mock_ops.object = DynamicModule("bpy.ops.object")
    mock_ops.mesh = DynamicModule("bpy.ops.mesh")
    mock_ops.wm = DynamicModule("bpy.ops.wm")
    mock_ops.export_scene = DynamicModule("bpy.ops.export_scene")

    # Mock high-fidelity exporters to satisfy existence checks
    def mock_file_op(filepath=None, **kwargs):
        if filepath:
            Path(filepath).parent.mkdir(parents=True, exist_ok=True)
            Path(filepath).touch()
        return {"FINISHED"}

    mock_ops.wm.stl_export = mock_file_op
    mock_ops.wm.obj_export = mock_file_op
    mock_ops.wm.stl_import = mock_file_op
    mock_ops.wm.obj_import = mock_file_op
    mock_ops.export_scene.gltf = mock_file_op

    def mock_empty_add(type_="PLAIN_AXES", location=(0, 0, 0), **kwargs):
        new_empty = MockObject(name="Empty")
        new_empty.type = "EMPTY"
        new_empty.empty_display_type = type_
        new_empty.location = MockVector(location)
        mock_data.objects.append(new_empty)
        mock_context.active_object = new_empty
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = new_empty
        return {"FINISHED"}

    def mock_cube_add(size=2.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="CubeMesh")
        # Add 8 vertices and 6 polygons for a box
        for _ in range(8):
            mesh.vertices.add()
        for _ in range(6):
            p = mesh.polygons.add()
            p.vertices = [0, 1, 2, 3]  # Quad
        new_cube = MockObject(name="Cube", data=mesh)
        new_cube.type = "MESH"
        new_cube.location = MockVector(location)
        # Primitives are created with base size 1.0 (or 2.0 in some Blender versions)
        # and then scaled or dimensions set.
        new_cube._base_dimensions = MockVector(1.0, 1.0, 1.0)
        new_cube.dimensions = MockVector(size, size, size)
        mock_data.objects.append(new_cube)
        mock_data.meshes.append(mesh)
        mock_context.active_object = new_cube
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = new_cube
        return {"FINISHED"}

    def mock_sphere_add(radius=1.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="SphereMesh")
        # UV Sphere default (482 verts, 480 polys)
        for _ in range(482):
            mesh.vertices.add()
        for _ in range(480):
            mesh.polygons.add()
        new_sphere = MockObject(name="Sphere", data=mesh)
        new_sphere.type = "MESH"
        new_sphere.location = MockVector(location)
        new_sphere._base_dimensions = MockVector(1.0, 1.0, 1.0)
        new_sphere.dimensions = MockVector(radius * 2, radius * 2, radius * 2)
        mock_data.objects.append(new_sphere)
        mock_data.meshes.append(mesh)
        mock_context.active_object = new_sphere
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = new_sphere
        return {"FINISHED"}

    def mock_cylinder_add(radius=1.0, depth=2.0, location=(0, 0, 0), **kwargs):
        mesh = MockMesh(name="CylinderMesh")
        # Cylinder default (66 verts, 64 polys)
        for _ in range(66):
            mesh.vertices.add()
        for _ in range(64):
            mesh.polygons.add()
        new_cyl = MockObject(name="Cylinder", data=mesh)
        new_cyl.type = "MESH"
        new_cyl.location = MockVector(location)
        new_cyl._base_dimensions = MockVector(1.0, 1.0, 1.0)
        new_cyl.dimensions = MockVector(radius * 2, radius * 2, depth)
        mock_data.objects.append(new_cyl)
        mock_data.meshes.append(mesh)
        mock_context.active_object = new_cyl
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = new_cyl
        return {"FINISHED"}

    def mock_monkey_add(**kwargs):
        mesh = MockMesh(name="MonkeyMesh")
        # Suzanne has ~500 verts/faces, avoids primitive detection
        for _ in range(507):
            mesh.vertices.add()
        for _ in range(500):
            mesh.polygons.add()
        new_monkey = MockObject(name="Suzanne", data=mesh)
        new_monkey.type = "MESH"
        mock_data.objects.append(new_monkey)
        mock_context.active_object = new_monkey
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = new_monkey
        return {"FINISHED"}

    mock_ops.object.empty_add = mock_empty_add
    mock_ops.mesh.primitive_cube_add = mock_cube_add
    mock_ops.mesh.primitive_uv_sphere_add = mock_sphere_add
    mock_ops.mesh.primitive_cylinder_add = mock_cylinder_add
    mock_ops.mesh.primitive_monkey_add = mock_monkey_add
    mock_ops.object.select_all = lambda action="TOGGLE": {"FINISHED"}
    mock_ops.object.transform_apply = lambda **kwargs: {"FINISHED"}
    mock_ops.object.join = lambda: {"FINISHED"}
    mock_ops.object.parent_set = lambda **kwargs: {"FINISHED"}
    mock_ops.object.parent_clear = lambda **kwargs: {"FINISHED"}
    mock_ops.object.delete = lambda **kwargs: {"FINISHED"}

    mock_bpy.ops = mock_ops
    mock_bpy.app = mock_app
    mock_bpy.utils = MagicMock()

    mock_mathutils = DynamicModule("mathutils")
    mock_mathutils.Vector = MockVector
    mock_mathutils.Matrix = MockMatrix
    mock_mathutils.Euler = MockEuler
    mock_mathutils.Quaternion = MockQuaternion

    mock_extras = DynamicModule("bpy_extras")
    mock_io_utils = DynamicModule("bpy_extras.io_utils")
    mock_io_utils.ExportHelper = MockIOHelper
    mock_io_utils.ImportHelper = MockIOHelper
    mock_extras.io_utils = mock_io_utils

    mock_gpu_extras = DynamicModule("gpu_extras")
    mock_batch = DynamicModule("gpu_extras.batch")
    mock_gpu_extras.batch = mock_batch

    sys.modules["bpy"] = mock_bpy
    sys.modules["bpy.props"] = mock_props
    sys.modules["bpy.types"] = mock_types
    sys.modules["bpy_extras"] = mock_extras
    sys.modules["bpy_extras.io_utils"] = mock_io_utils
    sys.modules["gpu_extras"] = mock_gpu_extras
    sys.modules["gpu_extras.batch"] = mock_batch
    sys.modules["mathutils"] = mock_mathutils

    # High-fidelity bmesh mock
    class MockBMesh:
        def __init__(self):
            self.verts = MockCollection()
            self.faces = MockCollection()

        def from_mesh(self, mesh):
            self.verts.clear()
            for _v in mesh.vertices:
                self.verts.add()

        def to_mesh(self, mesh):
            mesh.vertices.clear()
            for _ in self.verts:
                mesh.vertices.add()

        def free(self):
            pass

    mock_bmesh = DynamicModule("bmesh")
    mock_bmesh.new = lambda: MockBMesh()
    mock_bmesh.ops = DynamicModule("bmesh.ops")

    def _bm_create_cube(bm, **kwargs):
        [bm.verts.add() for _ in range(8)]
        [bm.faces.add() for _ in range(6)]

    def _bm_create_sphere(bm, **kwargs):
        [bm.verts.add() for _ in range(482)]
        [bm.faces.add() for _ in range(480)]

    mock_bmesh.ops.create_cube = _bm_create_cube
    mock_bmesh.ops.create_uvsphere = _bm_create_sphere
    mock_bmesh.ops.convex_hull = lambda bm, **kwargs: None

    sys.modules["bmesh"] = mock_bmesh
    sys.modules["gpu"] = DynamicModule("gpu")

    return mock_bpy
