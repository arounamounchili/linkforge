import math
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

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


class MockVector(tuple):
    """Mock for mathutils.Vector."""

    def __new__(cls, x=0.0, y=0.0, z=0.0):
        if hasattr(x, "x") and hasattr(x, "y") and hasattr(x, "z"):
            return super().__new__(cls, (float(x.x), float(x.y), float(x.z)))
        if isinstance(x, (list, tuple)) and len(x) >= 3:
            return super().__new__(cls, (float(x[0]), float(x[1]), float(x[2])))
        return super().__new__(cls, (float(x), float(y), float(z)))

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def z(self):
        return self[2]

    def __add__(self, other):
        return MockVector(self.x + other[0], self.y + other[1], self.z + other[2])

    def __sub__(self, other):
        return MockVector(self.x - other[0], self.y - other[1], self.z - other[2])

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return MockVector(self.x / other, self.y / other, self.z / other)
        return self

    def __repr__(self):
        return f"Vector(({self.x}, {self.y}, {self.z}))"

    def copy(self):
        return MockVector(self.x, self.y, self.z)

    @property
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

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


class MockEuler:
    """Mock for mathutils.Euler."""

    def __init__(self, x=0.0, y=0.0, z=0.0, order="XYZ"):
        if isinstance(x, (list, tuple, MockEuler)):
            self.x, self.y, self.z = float(x[0]), float(x[1]), float(x[2])
            # If the first arg was a sequence, the second arg might be the order
            if isinstance(y, str):
                order = y
        else:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
        self.order = order

    def __getitem__(self, i):
        return [self.x, self.y, self.z][i]

    def __setitem__(self, i, v):
        if i == 0:
            self.x = v
        elif i == 1:
            self.y = v
        elif i == 2:
            self.z = v

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __len__(self):
        return 3

    def to_matrix(self):
        res = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            res[i][i] = 1.0
        if abs(self.x) > 1e-6 or abs(self.y) > 1e-6 or abs(self.z) > 1e-6:
            res[0][0] = 0.9
            res[0][1], res[0][2], res[1][0] = self.x, self.y, self.z
        return MockMatrix(res)

    def to_4x4(self):
        return self.to_matrix()

    def __repr__(self):
        return f"Euler(({self.x}, {self.y}, {self.z}), '{self.order}')"

    def copy(self):
        return MockEuler(self.x, self.y, self.z, self.order)


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
        return MockQuaternion()

    def to_euler(self, order="XYZ"):
        if self.data[0][0] == 0.9:
            return MockEuler(self.data[0][1], self.data[0][2], self.data[1][0], order)
        return MockEuler(0.0, 0.0, 0.0, order)

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
        # High-fidelity inversion for translation
        inv = MockMatrix()
        # Copy rotation part (simplified: assume identity or hack)
        for i in range(3):
            for j in range(3):
                inv.data[i][j] = self.data[i][j]
        # Invert translation
        for i in range(3):
            inv.data[i][3] = -self.data[i][3]
        return inv

    def __matmul__(self, other):
        if isinstance(other, MockVector):
            return MockVector(
                other.x + self.data[0][3], other.y + self.data[1][3], other.z + self.data[2][3]
            )
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
    """Mock for Blender's Property descriptors."""

    def __init__(self, getter=None, setter=None, update=None, default=None, prop_type=None):
        self.getter = getter
        self.setter = setter
        self.update = update
        self.default = default
        self.prop_type = prop_type
        self.name = "unnamed_prop"

    def __get__(self, obj, cls):
        if obj is None:
            return self
        if not hasattr(obj, "_values"):
            obj._values = {}
        if self.name not in obj._values:
            if self.prop_type:
                try:
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
        return obj._values[self.name]

    def __set__(self, obj, value):
        if obj is None:
            return
        if not hasattr(obj, "_values"):
            obj._values = {}
        obj._values[self.name] = value
        if self.setter:
            self.setter(obj, value)
        if self.update:
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


class MockPropertyGroup(metaclass=PropertyMetaclass):
    """Base class for mocked Blender PropertyGroups."""

    def __init__(self, **kwargs):
        self.__dict__["_values"] = {}
        self.__dict__["id_data"] = None
        self.__dict__["name"] = kwargs.get("name", "Unnamed")
        for k, v in kwargs.items():
            setattr(self, k, v)

    def __setattr__(self, name, value):
        if name.startswith("_") or name in ("id_data", "name"):
            super().__setattr__(name, value)
        else:
            self._values[name] = value

    def __getitem__(self, key):
        return self._values.get(key)

    def __setitem__(self, key, value):
        self._values[key] = value

    def __contains__(self, key):
        return key in self._values

    def get(self, key, default=None):
        return self._values.get(key, default)

    def clear(self):
        self._values.clear()

    @property
    def bl_rna(self):
        return MagicMock()

    def __getattr__(self, key):
        if key.startswith("_") or key in ("id_data", "bl_rna"):
            raise AttributeError(key)
        if key in self._values:
            return self._values[key]

        if key in RESERVED_RNA_PROPS:
            raise AttributeError(f"RNA property '{key}' not found on '{self.name}'")

        # Fallback to MagicMock for UI, operators, and internal Blender properties
        # to avoid breaking every single test that touches a minor API.
        return MagicMock(name=key)


class MockCollection:
    """Mock for Blender's CollectionProperty items."""

    def __init__(self, prop_type=None):
        self._items = []
        self.prop_type = prop_type
        self.new = None
        self.new_from_object = None

    def add(self):
        item = self.prop_type() if self.prop_type else MockPropertyGroup()
        self._items.append(item)
        return item

    def __getitem__(self, key):
        if isinstance(key, int):
            return self._items[key]
        return self.get(key)

    def __len__(self):
        return len(self._items)

    def __iter__(self):
        return iter(self._items)

    def get(self, name, default=None):
        for item in self._items:
            if getattr(item, "name", None) == name:
                return item
        return default

    def remove(self, item, do_unlink=True):
        if isinstance(item, int):
            if 0 <= item < len(self._items):
                self._items.pop(item)
        elif item in self._items:
            self._items.remove(item)

    def pop(self, index=-1):
        return self._items.pop(index)

    def clear(self):
        self._items.clear()

    def append(self, item):
        self._items.append(item)

    def __contains__(self, key):
        """Support 'in' operator for named items or objects."""
        if isinstance(key, str):
            return any(getattr(item, "name", None) == key for item in self._items)
        return key in self._items

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
        if obj not in self._items:
            self._items.append(obj)

    def unlink(self, obj):
        if obj in self._items:
            self._items.remove(obj)


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
        self.name = name
        self.data = data
        self.type = "MESH" if (data and not isinstance(data, MagicMock)) else "EMPTY"
        self._parent = None
        self.matrix_world = MockMatrix.Identity(4)
        self.matrix_local = MockMatrix.Identity(4)
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

        # Pre-initialize LinkForge property groups to satisfy safe_get_* helpers
        self.linkforge = MockPropertyGroup(name="linkforge")
        self.linkforge_joint = MockPropertyGroup(name="linkforge_joint")
        self.linkforge_sensor = MockPropertyGroup(name="linkforge_sensor")
        self.linkforge_transmission = MockPropertyGroup(name="linkforge_transmission")

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, value):
        self._location = MockVector(value)
        # Update matrix_local/world (simplified for identity parent)
        self.matrix_local.data[0][3] = self._location.x
        self.matrix_local.data[1][3] = self._location.y
        self.matrix_local.data[2][3] = self._location.z
        self.matrix_world = self.matrix_local.copy()

    @property
    def rotation_euler(self):
        return self._rotation_euler

    @rotation_euler.setter
    def rotation_euler(self, value):
        self._rotation_euler = MockEuler(value)
        # Update matrix_local/world (simplified)
        rot_mat = self._rotation_euler.to_4x4()
        rot_mat.data[0][3] = self._location.x
        rot_mat.data[1][3] = self._location.y
        rot_mat.data[2][3] = self._location.z
        self.matrix_local = rot_mat
        self.matrix_world = self.matrix_local.copy()

    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, value):
        self._scale = MockVector(value)

    @property
    def dimensions(self):
        if hasattr(self, "_base_dimensions") and self._base_dimensions.length > 1e-6:
            return MockVector(
                self._base_dimensions.x * self.scale.x,
                self._base_dimensions.y * self.scale.y,
                self._base_dimensions.z * self.scale.z,
            )
        return self._values.get("dimensions", MockVector(0, 0, 0))

    @dimensions.setter
    def dimensions(self, value):
        v = MockVector(value)
        self._values["dimensions"] = v
        if hasattr(self, "_base_dimensions") and self._base_dimensions.length > 1e-6:
            # Sync scale
            self.scale = MockVector(
                v.x / self._base_dimensions.x if self._base_dimensions.x > 0 else 1.0,
                v.y / self._base_dimensions.y if self._base_dimensions.y > 0 else 1.0,
                v.z / self._base_dimensions.z if self._base_dimensions.z > 0 else 1.0,
            )
        else:
            # Treatment for uninitialized base dimensions: value is base at scale 1.0
            self._base_dimensions = v
            self.scale = MockVector(1, 1, 1)

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


def setup_mock_bpy():
    """Initializes the entire Blender mock environment."""
    mock_bpy = DynamicModule("bpy")
    mock_props = DynamicModule("bpy.props")
    mock_types = DynamicModule("bpy.types")
    mock_context = MagicMock()
    mock_data = MagicMock()
    mock_app = MagicMock()

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
    mock_data.objects.new = lambda name, data=None: mock_data.objects.new_item(name, data=data)

    def _new_obj_item(name, data=None):
        obj = MockObject(name=name, data=data)
        mock_data.objects.append(obj)
        return obj

    mock_data.objects.new = _new_obj_item

    mock_data.meshes = MockCollection(prop_type=MockMesh)
    mock_data.meshes.new = lambda name: mock_data.meshes.new_item(name, cls=MockMesh)

    def _new_mesh_item(name):
        m = MockMesh(name=name)
        mock_data.meshes.append(m)
        return m

    mock_data.meshes.new = _new_mesh_item

    mock_data.meshes.new_from_object = lambda obj, **kwargs: _new_mesh_item(f"{obj.name}_mesh")
    mock_data.collections = MockCollection(prop_type=MockCollection)
    mock_data.collections.new = lambda name: MockCollection()
    mock_data.scenes = MockCollection(prop_type=MockScene)
    mock_data.scenes.append(MockScene(name="Scene"))
    mock_data.materials = MockCollection(prop_type=MockMaterial)
    mock_data.materials.new = lambda name: MockMaterial(name=name)

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
        new_cube.location = MockVector(location)
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
        new_sphere.location = MockVector(location)
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
        new_cyl.location = MockVector(location)
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
