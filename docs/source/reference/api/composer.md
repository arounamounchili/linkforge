# LinkForge Composer API

The Composer provides a high-level, fluent API for programmatically constructing and assembling robot models. It is designed to be ergonomic, human-readable, and robust.

## RobotBuilder

The main entry point for constructing a robot model.

```{eval-rst}
.. autoclass:: linkforge.core.RobotBuilder
   :members:
   :undoc-members:
   :show-inheritance:
   :inherited-members:
```

## LinkBuilder

A staged builder for configuring individual links and their parent joints.

```{eval-rst}
.. autoclass:: linkforge.core.LinkBuilder
   :members:
   :undoc-members:
   :show-inheritance:
```

## SemanticBuilder

A namespace for SRDF and MoveIt planning groups, named states, and self-collision settings.

```{eval-rst}
.. autoclass:: linkforge.core.composer.semantic_builder.SemanticBuilder
   :members:
   :undoc-members:
   :show-inheritance:
```
