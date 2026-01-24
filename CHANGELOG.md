# Changelog

All notable changes to LinkForge will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0](https://github.com/arounamounchili/linkforge/compare/v1.0.0...v1.1.0) (2026-01-07)


### 🚀 Features

* align documentation and versioning with product strategy ([#18](https://github.com/arounamounchili/linkforge/issues/18)) ([5e0ebee](https://github.com/arounamounchili/linkforge/commit/5e0ebeeeae09173b3c1cbe3207e45ca76c925cad))


### 🐞 Bug Fixes

* add CI collector job for stable branch protection status ([b473d6d](https://github.com/arounamounchili/linkforge/commit/b473d6d1053af7dce083ca7ad107cc987aa05088))
* add missing bpy import to property_helpers ([4d7b5ac](https://github.com/arounamounchili/linkforge/commit/4d7b5ac98087bf2710bbaac4bb61b50d84e80723))
* **blender:** modernize extension logic and fix GPU overlay ([#20](https://github.com/arounamounchili/linkforge/issues/20)) ([29a9dfb](https://github.com/arounamounchili/linkforge/commit/29a9dfb1d1b7db9ecf95ee33b74e7737686dfedb))
* **blender:** resolve UnboundLocalError and finalize v1.1.0 community standards ([#21](https://github.com/arounamounchili/linkforge/issues/21)) ([f4215fd](https://github.com/arounamounchili/linkforge/commit/f4215fd05ea0f34746fde7da558550f6f70f928f))
* **blender:** robust DAE export and Blender 5.0 compatibility ([#23](https://github.com/arounamounchili/linkforge/issues/23)) ([d3ac437](https://github.com/arounamounchili/linkforge/commit/d3ac43708ab829c1514ddc38ac28a4a5e9799cd5))
* **blender:** sanitize robot component names to resolve .001 conflicts ([5266ec1](https://github.com/arounamounchili/linkforge/commit/5266ec167602dedd67e57dac6df3caf3a1560899))
* **docs:** use correct relative paths for logo and favicon ([7af18f3](https://github.com/arounamounchili/linkforge/commit/7af18f36be9da89cdc91d3e0ecb2d1cfd1e5dfc0))
* reset manifest version ([#31](https://github.com/arounamounchili/linkforge/issues/31)) ([d6d2798](https://github.com/arounamounchili/linkforge/commit/d6d27985d426bf619bd6fd8900522bfc69c90dca))
* resolve CI artifact name collision and pin stable actions ([#9](https://github.com/arounamounchili/linkforge/issues/9)) ([a923105](https://github.com/arounamounchili/linkforge/commit/a923105094fcae739b02a17394b1b99b419996f2))
* unify sensor visual shape to sphere ([#6](https://github.com/arounamounchili/linkforge/issues/6)) ([#7](https://github.com/arounamounchili/linkforge/issues/7)) ([eddc88e](https://github.com/arounamounchili/linkforge/commit/eddc88e4e6955685e12c49d29b748f19cb2e6ec0))


### 📚 Documentation

* add workflow diagram and code owners ([dda0fd0](https://github.com/arounamounchili/linkforge/commit/dda0fd0a18d0ed3dbcf8d4911e5ab7b72134932b))
* clarify platform-specific zips in README ([3ce93c7](https://github.com/arounamounchili/linkforge/commit/3ce93c77589e79eb4f611e07c9fab11011c229ba))
* evergreen installation instructions in README ([a30b390](https://github.com/arounamounchili/linkforge/commit/a30b390aa2f3958e0527598994278eae8d37b90a))
* finalize architecture documentation for unified mirroring ([b227076](https://github.com/arounamounchili/linkforge/commit/b227076655aea799205f0980554ede0190b23448))
* remove stale handler comments in blender/__init__.py ([507d503](https://github.com/arounamounchili/linkforge/commit/507d50388a701d0c5ddff44d4f2da8a34638a1a7))
* unify assets and set single source of truth in docs/assets ([4da663c](https://github.com/arounamounchili/linkforge/commit/4da663c325d5f729b818b79ec2ecc6003706356c))
* update ARCHITECTURE.md to reflect handler removal ([365c2dc](https://github.com/arounamounchili/linkforge/commit/365c2dc428a34bb3c67ebdf3fe0a48579e77b372))
* upgrade Code of Conduct to v3.0 ([#33](https://github.com/arounamounchili/linkforge/issues/33)) ([777f2a4](https://github.com/arounamounchili/linkforge/commit/777f2a4d20af04ab58230499cee345e32a2bbd1e))
* upgrade README with premium technical specs and roadmap ([fb8e0f2](https://github.com/arounamounchili/linkforge/commit/fb8e0f2f01a96687ad2e4b6a4df7f1983c74c816))


### 🛠️ Refactors

* completely remove the handlers module to show standard-compliance ([24cf1e7](https://github.com/arounamounchili/linkforge/commit/24cf1e732da4a6b2b0f1c71efa2aac2232416a16))
* improve type safety in core generators ([#36](https://github.com/arounamounchili/linkforge/issues/36)) ([c5abbde](https://github.com/arounamounchili/linkforge/commit/c5abbdeb626cc6e05e4f0bdeee044c10158cc336))
* remove depsgraph handlers and implement property mirroring for names ([703b53f](https://github.com/arounamounchili/linkforge/commit/703b53f321dd74ffb6f883a62bb53ff33929a562))
* standardize URDF/Xacro headers and eliminate duplication ([ef475d0](https://github.com/arounamounchili/linkforge/commit/ef475d094621e152566a78bc6e6eaeb9e2839e42))
* unify imported object display sizes with addon preferences ([a781a71](https://github.com/arounamounchili/linkforge/commit/a781a71208a5c887521b6dce5b7b27dc3bc35c5e))

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



[1.0.0]: https://github.com/arounamounchili/linkforge/releases/tag/v1.0.0
