import math
import sys
import types
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


class MockVector:
    """Mock for mathutils.Vector."""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        if isinstance(x, (list, tuple, MockVector)):
            self.x, self.y, self.z = float(x[0]), float(x[1]), float(x[2])
        else:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

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

    def copy(self):
        return MockVector(self.x, self.y, self.z)

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

    @property
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def rotation_difference(self, other):
        return MockQuaternion()


class MockQuaternion:
    """Mock for mathutils.Quaternion."""

    def __init__(self, *args):
        self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0

    def to_euler(self, order="XYZ"):
        return MockEuler(self.x, self.y, self.z, order)


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
        res[0][0] = 0.9
        res[3][3] = 1.0
        res[0][1], res[0][2], res[1][0] = self.x, self.y, self.z
        return MockMatrix(res)

    def to_4x4(self):
        return self.to_matrix()

    def __repr__(self):
        return f"Euler(({self.x}, {self.y}, {self.z}), '{self.order}')"


class MockMatrix:
    """Mock for mathutils.Matrix."""

    def __init__(self, data=None):
        if data is None:
            self.data = [[0.0] * 4 for _ in range(4)]
            for i in range(4):
                self.data[i][i] = 1.0
        else:
            self.data = data

    @staticmethod
    def Identity(n):  # noqa: N802
        return MockMatrix()

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

    def to_euler(self, order="XYZ"):
        if self.data[0][0] == 0.9:
            return MockEuler(self.data[0][1], self.data[0][2], self.data[1][0], order)
        return MockEuler(0.0, 0.0, 0.0, order)

    def to_4x4(self):
        return self

    def identity(self):
        self.data = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            self.data[i][i] = 1.0

    def inverted(self):
        inv = MockMatrix()
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


class MockPropertyGroup(metaclass=PropertyMetaclass):
    """Base class for mocked Blender PropertyGroups."""

    def __init__(self, **kwargs):
        self._values = {}
        for k, v in kwargs.items():
            setattr(self, k, v)

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
        if key.startswith("_"):
            raise AttributeError(key)
        return MagicMock(name=key)


class MockCollection:
    """Mock for Blender's CollectionProperty items."""

    def __init__(self, prop_type=None):
        self._items = []
        self.prop_type = prop_type
        self.new = None
        self.new_from_object = None

    def add(self):
        item = self.prop_type() if self.prop_type else MagicMock()
        self._items.append(item)
        return item

    def remove(self, item, do_unlink=True):
        if isinstance(item, int):
            self._items.pop(item)
        elif item in self._items:
            self._items.remove(item)

    def clear(self):
        self._items.clear()

    def append(self, item):
        self._items.append(item)

    def __len__(self):
        return len(self._items)

    def __getitem__(self, i):
        return self._items[i]

    def __iter__(self):
        return iter(self._items)

    def __contains__(self, item):
        return item in self._items

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


class MockObject(MockPropertyGroup):
    """Mock for bpy.types.Object."""

    def __init__(self, name="Object", data=None, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.data = data
        self.type = "MESH" if data else "EMPTY"
        self.matrix_world = MockMatrix()
        self.location = MockVector()
        self.rotation_euler = MockEuler()
        self.scale = MockVector(1, 1, 1)
        self.dimensions = MockVector(1, 1, 1)
        self.matrix_parent_inverse = MockMatrix()
        self.parent = None
        self.children = MockCollection()
        self.users_collection = MockCollection()
        self.vertices = MockCollection()
        self.polygons = MockCollection()
        self.modifiers = MockCollection()
        self.bound_box = [(0.0, 0.0, 0.0)] * 8
        self.empty_display_type = "PLAIN_AXES"
        self.hide_viewport = False

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
        self.collection = MockCollection()
        self.objects = MockCollection()
        self.view_layers = MockCollection()
        self.view_layers.append(MagicMock(name="ViewLayer"))


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
    mock_types.Scene = MockScene
    mock_types.Collection = MockCollection
    mock_types.Operator = MockOperator
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
    mock_data.objects.new = lambda name, data=None: MockObject(name=name, data=data)
    mock_data.meshes = MockCollection()
    mock_data.meshes.new = lambda name: MockObject(name=name)
    mock_data.meshes.new_from_object = lambda obj, **kwargs: MockObject(name=f"{obj.name}_mesh")
    mock_data.collections = MockCollection()
    mock_data.collections.new = lambda name: MockCollection()
    mock_data.scenes = MockCollection()
    mock_data.scenes.append(MockScene(name="Scene"))
    mock_data.materials = MockCollection()
    mock_data.materials.new = lambda name: MockObject(name=name)

    # Initialize view_layer with a fallback to avoid NoneType errors
    mock_view_layer = MagicMock(name="ViewLayer")
    mock_view_layer.objects = mock_data.objects
    mock_view_layer.update = lambda: None
    mock_context.view_layer = mock_view_layer

    if len(mock_data.scenes) > 0:
        mock_context.scene = mock_data.scenes[0]
        mock_context.scene.objects = mock_data.objects
        if hasattr(mock_context.scene, "view_layers") and len(mock_context.scene.view_layers) > 0:
            layer = mock_context.scene.view_layers[0]
            if layer is not None:
                mock_context.view_layer = layer
                # Ensure the layer has objects and update
                if not hasattr(mock_context.view_layer, "objects"):
                    mock_context.view_layer.objects = mock_data.objects
                if not hasattr(mock_context.view_layer, "update"):
                    mock_context.view_layer.update = lambda: None

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

    def mock_cube_add(location=(0, 0, 0), **kwargs):
        new_cube = MockObject(name="Cube")
        new_cube.type = "MESH"
        new_cube.location = MockVector(location)
        mock_data.objects.append(new_cube)
        mock_context.active_object = new_cube
        if mock_context.view_layer:
            mock_context.view_layer.objects.active = new_cube
        return {"FINISHED"}

    mock_ops.object.empty_add = mock_empty_add
    mock_ops.mesh.primitive_cube_add = mock_cube_add
    mock_ops.object.select_all = lambda action="TOGGLE": {"FINISHED"}

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
    sys.modules["bmesh"] = DynamicModule("bmesh")
    sys.modules["gpu"] = DynamicModule("gpu")

    return mock_bpy
