# Kinematic Graph

The `KinematicGraph` provides formal graph-theory logic for validating and
traversing the link-joint structure of a robot. It is used internally by
`RobotValidator` and `RobotAssembly`, but is also available for advanced users
who need custom traversal or analysis logic.

## KinematicGraph

```{eval-rst}
.. autoclass:: linkforge_core.models.graph.KinematicGraph
   :members:
   :undoc-members:
   :show-inheritance:
```

---

## Usage Examples

### Detect cycles in a robot

```python
from linkforge_core.models.graph import KinematicGraph
from linkforge_core.parsers import URDFParser
from pathlib import Path

robot = URDFParser().parse(Path("my_robot.urdf"))
graph = KinematicGraph(robot.links, robot.joints)

try:
    root = graph.root_link()
    print(f"Root link: {root.name}")
    print("No cycles detected.")
except Exception as e:
    print(f"Topology error: {e}")
```

### Topological traversal

```python
from linkforge_core.models.graph import KinematicGraph

graph = KinematicGraph(robot.links, robot.joints)

# Traverse links in depth-first order from the root
for link in graph.topological_sort():
    print(link.name)
```

:::{note}
`KinematicGraph` is stateless and can be re-created at any time from a `Robot`
model. It does not hold a reference to the original robot, making it safe to
use in parallel or in tests without side effects.
:::
