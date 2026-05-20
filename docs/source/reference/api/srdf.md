# SRDF — Semantic Robot Description

The SRDF (Semantic Robot Description Format) layer provides data structures,
a parser, and a generator for MoveIt-compatible semantic information such as
planning groups, named poses, and collision filters.

## Data Models

```{eval-rst}
.. autoclass:: linkforge.core.SemanticRobotDescription
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.PlanningGroup
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Chain
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.GroupState
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.EndEffector
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.PassiveJoint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.VirtualJoint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.CollisionPair
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.LinkSphereApproximation
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.SrdfSphere
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.JointProperty
   :members:
   :undoc-members:
   :show-inheritance:
```

---

## SRDF Parser

```{eval-rst}
.. autoclass:: linkforge.core.SRDFParser
   :members:
   :undoc-members:
   :show-inheritance:
```

---

## SRDF Generator

```{eval-rst}
.. autoclass:: linkforge.core.SRDFGenerator
   :members:
   :undoc-members:
   :show-inheritance:
```

---

## Usage Examples

### Parse an existing SRDF file

```python
from linkforge.core import SRDFParser
from pathlib import Path

srdf = SRDFParser().parse(Path("my_robot.srdf"))

print(f"Planning groups: {len(srdf.groups)}")
for group in srdf.groups:
    print(f"  {group.name}: {len(group.links)} links, {len(group.joints)} joints")
```

### Build SRDF programmatically

```python
from linkforge.core import (
    SemanticRobotDescription,
    PlanningGroup,
    GroupState,
    CollisionPair,
)

srdf = SemanticRobotDescription(
    robot_name="my_arm",
    groups=[
        PlanningGroup(
            name="arm",
            links=["base_link", "link1", "link2"],
            joints=["joint1", "joint2"],
        )
    ],
    group_states=[
        GroupState(name="home", group="arm", joint_values={"joint1": 0.0, "joint2": 0.0})
    ],
    disabled_collisions=[
        CollisionPair(link1="base_link", link2="link1", reason="Adjacent"),
    ],
)
```

### Generate SRDF XML

```python
from linkforge.core import SRDFGenerator

generator = SRDFGenerator()
srdf_string = generator.generate(srdf)

with open("my_robot.srdf", "w") as f:
    f.write(srdf_string)
```

### Round-trip (parse → modify → re-generate)

```python
from linkforge.core import SRDFParser, SRDFGenerator, CollisionPair
from pathlib import Path
import dataclasses

original = SRDFParser().parse(Path("robot.srdf"))

# Add a new collision exclusion
updated = dataclasses.replace(
    original,
    disabled_collisions=[
        *original.disabled_collisions,
        CollisionPair(link1="hand_link", link2="wrist_link", reason="Adjacent"),
    ],
)

output = SRDFGenerator().generate(updated)
Path("robot_updated.srdf").write_text(output)
```

:::{note}
SRDF data is also produced automatically by ``RobotBuilder.export_srdf()`` when
you use the ``group()`` and ``disable_collisions()`` helper methods. Direct
use of the parser and generator is mainly needed when working with existing
SRDF files. See the [Composer reference](composer) for the higher-level API.
:::
