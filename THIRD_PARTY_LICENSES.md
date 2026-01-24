# Third-Party Licenses

LinkForge bundles several third-party Python libraries to provide a self-contained experience within Blender. This document provides clear attribution and licensing information for these redistributed components.

## Summary

| Library | Version | License | Link |
| :--- | :--- | :--- | :--- |
| **PyYAML** | 6.0.3 | [MIT](https://spdx.org/licenses/MIT.html) | [pyyaml.org](https://pyyaml.org/) |
| **xacrodoc** | 1.3.1 | [MIT](https://spdx.org/licenses/MIT.html) | [adamheins/xacrodoc](https://github.com/adamheins/xacrodoc) |
| **rospkg** | 1.6.1 | [BSD-3-Clause](https://spdx.org/licenses/BSD-3-Clause.html) | [ROS Wiki](http://wiki.ros.org/rospkg) |
| **docutils** | 0.22.4 | [Public Domain / BSD-2-Clause](https://docutils.sourceforge.io/COPYING.html) | [SourceForge](https://docutils.sourceforge.io/) |

## Bundled Components

All libraries listed below are bundled as `.whl` files in the `wheels/` directory of the extension package.

### PyYAML
- **License**: MIT
- **Copyright**: &copy; 2006-2016 Kirill Simonov, &copy; 2017-2024 Ingy döt Net and the YAML community
- **Summary**: YAML parser and issuer for Python.

### xacrodoc
- **License**: MIT
- **Copyright**: &copy; 2023-2025 Adam Heins
- **Summary**: Programmatic interface for Xacro.

### rospkg
- **License**: BSD-3-Clause
- **Copyright**: &copy; 2011, Willow Garage, Inc.
- **Summary**: Utilities for interacting with the ROS filesystem.

### docutils
- **License**: Public Domain / BSD-2-Clause
- **Copyright**: &copy; David Goodger and others (dedicated to the public domain)
- **Summary**: Documentation utilities (dependency for xacrodoc).

---

## Licensing Notes

LinkForge itself is licensed under **GPL-3.0-or-later**. The third-party libraries listed above are redistributed under their own permissive licenses. This redistribution is consistent with both the GPLv3 and the respective permissive licenses.
