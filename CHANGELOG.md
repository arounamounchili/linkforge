# Changelog

All notable changes to LinkForge will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0](https://github.com/arounamounchili/linkforge/compare/linkforge-v1.0.0...linkforge-v1.1.0) (2025-12-30)

> [!NOTE]
> v1.1.0: Documentation Overhaul & Enhanced Sensor Suite


### 🚀 Features

* Add transmission configuration steps and new screenshots to diff drive tutorial, updating README and index. ([78bddac](https://github.com/arounamounchili/linkforge/commit/78bddac388abea443b53aaf630c78f59fd276840))
* enhance documentation with new best practices, expanded troubleshooting, community links, and improved branding including a custom logo and social media metadata. ([b335947](https://github.com/arounamounchili/linkforge/commit/b335947256178b0320a2b37a3ebe3428d6180f7a))


### 🐞 Bug Fixes

* add CI collector job for stable branch protection status ([b473d6d](https://github.com/arounamounchili/linkforge/commit/b473d6d1053af7dce083ca7ad107cc987aa05088))
* resolve CI artifact name collision and pin stable actions ([#9](https://github.com/arounamounchili/linkforge/issues/9)) ([a923105](https://github.com/arounamounchili/linkforge/commit/a923105094fcae739b02a17394b1b99b419996f2))
* unify sensor visual shape to sphere ([#6](https://github.com/arounamounchili/linkforge/issues/6)) ([#7](https://github.com/arounamounchili/linkforge/issues/7)) ([eddc88e](https://github.com/arounamounchili/linkforge/commit/eddc88e4e6955685e12c49d29b748f19cb2e6ec0))


### 📚 Documentation

* Add a development guide, remove README from documentation, and restructure documentation indexes with grid-item-cards for improved navigation. ([96a3641](https://github.com/arounamounchili/linkforge/commit/96a36413fd4955df519cbfe43b3d84968db70c67))
* Add FAQ, troubleshooting, and glossary pages, and enhance existing guides with workflow diagrams and important notes. ([2730660](https://github.com/arounamounchili/linkforge/commit/2730660e0a1ee729d362ef6d5115fc198be8f6f9))
* Add favicon and restructure index navigation with new "About LinkForge" section ([a2ce277](https://github.com/arounamounchili/linkforge/commit/a2ce2779066f55819a3d0bbed7681b528765c473))
* Add issue template configuration ([392ef4c](https://github.com/arounamounchili/linkforge/commit/392ef4c800b356f368fd35f9439cd90bf1188c8d))
* Add lidar sensor, remove caster wheel, and refine robot dimensions and joint parameters. ([478296a](https://github.com/arounamounchili/linkforge/commit/478296ac0b07fd4409677d6889b41a6b4910b1f9))
* Add new tutorial screenshots and update UI tab references for sensors and transmissions. ([383648e](https://github.com/arounamounchili/linkforge/commit/383648e1e4d8fde284098987fb4b863ad1f7f68c))
* add workflow diagram and code owners ([dda0fd0](https://github.com/arounamounchili/linkforge/commit/dda0fd0a18d0ed3dbcf8d4911e5ab7b72134932b))
* clarify architecture layer count and include new sensor, transmission, and gizmo modules. ([54fc0c9](https://github.com/arounamounchili/linkforge/commit/54fc0c93306dd97fa224d91fada0233746b8e8da))
* detail new Empty and RViz-style joint visualization options and how to hide them. ([8517b45](https://github.com/arounamounchili/linkforge/commit/8517b451d4c3970d542df2aede586f4e31d35237))
* Enhance documentation styling, layout, and content with custom CSS, updated Sphinx configuration, and restructured section titles. ([fa6b78e](https://github.com/arounamounchili/linkforge/commit/fa6b78e5868b8f3ebeae25a3b6179b25636ee8c2))
* Enhance robot structure and collision geometry documentation, updating the diff drive tutorial with collision generation. ([a897076](https://github.com/arounamounchili/linkforge/commit/a8970760feec323f3032993fe5b05ba8019a07f2))
* Fix build errors and add missing How-to guides ([5c41537](https://github.com/arounamounchili/linkforge/commit/5c4153716abc13c0438636a03608c75dc699ada9))
* Refactor UI terminology from tabs to panels/sidebar and rewrite the troubleshooting guide. ([f011b9c](https://github.com/arounamounchili/linkforge/commit/f011b9c944f6720586dfb8449d0b528b11c7445d))
* remove examples directory reference ([c1b8e22](https://github.com/arounamounchili/linkforge/commit/c1b8e228d0fae0bc7ace91d204039e89c2cca6b7))
* remove sensor panel screenshot and its reference from add sensors guide ([990c18a](https://github.com/arounamounchili/linkforge/commit/990c18aace006f84c881a949dc8bf3ea3ab6c1ad))
* Remove unused `sphinx.ext.todo` and `sphinx.ext.coverage` extensions from documentation configuration. ([de87b6d](https://github.com/arounamounchili/linkforge/commit/de87b6d591fbc7f3549cabcb0e1b1a3f927b5879))
* Reorganize documentation according to Diátaxis framework ([9803d91](https://github.com/arounamounchili/linkforge/commit/9803d9162576458838d7d6441b2c615d6584c4b1))
* Shorten tagline and remove 'Robotics' tag from manifest ([bc9dce6](https://github.com/arounamounchili/linkforge/commit/bc9dce6070dd098eb56178e37917db6f1283b2a2))
* update collision visibility toggle instructions and clarify manual collision setup steps ([21f1c4b](https://github.com/arounamounchili/linkforge/commit/21f1c4b54996e1ce7f3a5cde005d8d15a4e714e0))
* update documentation links in contributing guide to official documentation and architecture guide. ([66c74d6](https://github.com/arounamounchili/linkforge/commit/66c74d6a1c8f740fa120ffea3f0c0a0ca088656c))
* Update inertia calculation terminology from 'Calculate Inertia button' to 'Auto-Calculate Inertia checkbox' in tutorials and troubleshooting guides. ([fccacfb](https://github.com/arounamounchili/linkforge/commit/fccacfb08b550bcf934be3bdc5e8fb47814cd210))
* Update linkforge logo and social preview images ([f6be879](https://github.com/arounamounchili/linkforge/commit/f6be879f87e18b5e74984d43d8e8bb7930059f3d))
* Update third-party license attribution with verified metadata ([e99a25c](https://github.com/arounamounchili/linkforge/commit/e99a25c084d8a07da31838475de09487a3172424))
* Update UI tab names from "Robot" and "Export" to "Validate & Export" and clarify ROS2 control generation. ([9210556](https://github.com/arounamounchili/linkforge/commit/9210556bb9338af5372c9bb37476e0e287d9717d))
* Update xacrodoc link, correct PyYAML copyright year, and reorder docutils license information. ([66036e2](https://github.com/arounamounchili/linkforge/commit/66036e21f0a2ffdec41c9885efa7e4d451e6e167))
* upgrade README with premium technical specs and roadmap ([fb8e0f2](https://github.com/arounamounchili/linkforge/commit/fb8e0f2f01a96687ad2e4b6a4df7f1983c74c816))
* use absolute github urls for root doc links to fix build ([4ae149e](https://github.com/arounamounchili/linkforge/commit/4ae149e8704de5769af0039f8d902358e331f4e2))
* use readthedocs urls for root docs to bypass sphinx xref issues ([f9bba0b](https://github.com/arounamounchili/linkforge/commit/f9bba0b96708e0eed4946e8f5a33714373322fd4))

## [1.0.0] - 2025-12-23

> [!NOTE]
> v1.0.0: Initial Production-Ready Release

LinkForge 1.0.0 is the first production-ready release of the professional URDF & XACRO exporter for Blender.

### Features
- **Complete URDF/XACRO Support**: Full bidirectional import and export logic.
- **Sensor Suite**: Comprehensive support for Camera, Depth Camera, LiDAR, IMU, GPS, Contact, and Force/Torque sensors.
- **ROS2 Control Integration**: Native `ros2_control` configuration generation for Gazebo and real hardware interfaces.
- **Automatic Physics**: Automatic calculation of mass and inertia tensors for both primitive shapes and complex meshes.
- **Smart Validation**: Built-in validator to catch errors like disconnected links, missing inertia, or invalid joint limits before export.
- **Round-Trip Fidelity**: Importing a URDF and re-exporting it preserves all properties, including sensor origins and visual/collision geometries.
- **Resilient Parsing**: The parser gracefully handles minor errors (like invalid geometry dimensions) by logging warnings instead of crashing.
- **XACRO Powers**: Automatic macro generation, material extraction, and support for split-file export organization.
- **Viewport Visualization**: Semantic feedback in the viewport for joint axes, limits, and transmission actuation vectors.
- **Modern Standards**: Support for modern Gazebo Sim sensor types (`gpu_lidar`, `navsat`) and ROS 2 conventions.

### Security
- **Path Validation**: Strict validation of mesh paths to prevent path traversal vulnerabilities.
- **XML Hardening**: Protection against XML bomb attacks through depth limits.
- **Input Sanitization**: Numeric constraints to prevent NaN/Inf injection.

---

## Release Notes - v1.0.0

**Installation:**
```bash
# From Blender Extensions
Edit > Preferences > Get Extensions > Search "LinkForge"

# Or manual installation
Download linkforge-1.0.0.zip from releases
Edit > Preferences > Get Extensions > Install from Disk
```

**Documentation:**
- [Architecture Guide](https://linkforge.readthedocs.io/en/latest/explanation/ARCHITECTURE.html)
- [Contributing Guide](CONTRIBUTING.md)

**Contributors:**
- Arouna Patouossa Mounchili ([@arounamounchili](https://github.com/arounamounchili))

[1.0.0]: https://github.com/arounamounchili/linkforge/releases/tag/v1.0.0
