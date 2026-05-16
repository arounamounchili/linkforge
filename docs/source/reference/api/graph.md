# Kinematic Graph

The `KinematicGraph` provides formal graph-theory logic for validating and
traversing the link-joint structure of a robot. It is used internally by
`RobotValidator` and `RobotBuilder`, but is also available for advanced users
who need custom traversal or analysis logic.

## KinematicGraph

```{eval-rst}
.. autoclass:: linkforge.core.models.graph.KinematicGraph
   :members:
   :undoc-members:
   :show-inheritance:
```

---

## Usage Examples

### Detect cycles in a robot

```python
from linkforge.core.models.graph import KinematicGraph
from linkforge.core.parsers import URDFParser
from pathlib import Path

robot = URDFParser().parse(Path("my_robot.urdf"))
graph = KinematicGraph(robot.links, robot.joints)

try:
    roots = graph.get_root_links()
    print(f"Root links found: {len(roots)}")
    print("No cycles detected.")
except Exception as e:
    print(f"Topology error: {e}")
```

### Topological traversal

```python
from linkforge.core.models.graph import KinematicGraph

graph = KinematicGraph(robot.links, robot.joints)

# 1. Get ordered link names (Strings)
for link_name in graph.get_topological_link_names():
    print(f"Processing link: {link_name}")

# 2. Get ordered joint objects (Models)
for joint in graph.get_topological_joints():
    print(f"Configuring joint: {joint.name} ({joint.type.name})")
```

:::{note}
`KinematicGraph` is stateless and can be re-created at any time from a `Robot`
model. It does not hold a reference to the original robot, making it safe to
use in parallel or in tests without side effects.
:::
