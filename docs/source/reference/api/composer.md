# LinkForge Composer API

The Composer provides a high-level, fluent API for programmatically constructing and assembling robot models. It is designed to be ergonomic, human-readable, and robust.

## RobotBuilder

The main entry point for constructing a robot model.

:::{tip} **Context Managers & Cloning**
`RobotBuilder` supports deep cloning via `.clone()`, allowing you to use a base builder as a template and fork it into multiple variations.
Additionally, `LinkBuilder` supports context managers (`with builder.link(...)`) to automatically set the parent context for nested links, removing the need to manually specify the `parent=` argument.
:::

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

## Interfaces

```{eval-rst}
.. autoclass:: linkforge.core.composer.interfaces.IComposer
   :members:
   :undoc-members:
   :show-inheritance:
```
