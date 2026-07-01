# Changelog

All notable changes to LinkForge will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.4.3](https://github.com/arounamounchili/linkforge/compare/v1.4.2...v1.4.3) (2026-07-01)


### 🐞 Bug Fixes

* **blender:** resolve top-level module policy violation via relative imports ([13a1fd8](https://github.com/arounamounchili/linkforge/commit/13a1fd88dad4d17aed6dc67b478cb2392af2c0d0))

## [1.4.2](https://github.com/arounamounchili/linkforge/compare/v1.4.1...v1.4.2) (2026-06-30)


### 🐞 Bug Fixes

* replace eval with secure AST parser and drop unsupported py3.12 wheels ([c5abb8e](https://github.com/arounamounchili/linkforge/commit/c5abb8e5a17a8f9060a754ecf09fca410adc70fb))

## [1.4.1](https://github.com/arounamounchili/linkforge/compare/v1.4.0...v1.4.1) (2026-06-04)


### 🐞 Bug Fixes

* **ci:** exclude github commits urls from link checker ([d652f36](https://github.com/arounamounchili/linkforge/commit/d652f362b0f7e3fe8b5fb39d723c0dd976678338))


### 📚 Documentation

* fix Sphinx warnings, document missing APIs, and refine architectural guides ([d3c9bec](https://github.com/arounamounchili/linkforge/commit/d3c9becbf1837161efcebe6690d4202ac773f77d))


### 🛠️ Refactors

* clean up test suite comments/fixtures and update docs ([77399d4](https://github.com/arounamounchili/linkforge/commit/77399d4bdd10ca282ed5cc179c80f94c26d3f273))
* **core:** address API design findings for ergonomics and stability ([eba7d18](https://github.com/arounamounchili/linkforge/commit/eba7d18153d7c0a9706459e8d22f7c1a64050e9c))
* **core:** remove dead code and deduplicate constants ([17cf8cf](https://github.com/arounamounchili/linkforge/commit/17cf8cf28b40586b44048d7cf1d0b04b0e5397ee))

## [1.4.0](https://github.com/arounamounchili/linkforge/compare/v1.3.0...v1.4.0) (2026-05-29)


### 🚀 Features

* **blender:** implement high-performance mesh inertia integrator using NumPy vectorization ([0c815ee](https://github.com/arounamounchili/linkforge/commit/0c815eebf7f960f24c38413083d79d6c85f8b724))
* **composer:** add context manager support and deep cloning to RobotBuilder ([a2876fe](https://github.com/arounamounchili/linkforge/commit/a2876feff1534678ef85a085b9a6b5dc4b5b2441))
* **core:** harden xacro eval sandbox against dunder escapes ([7def4d8](https://github.com/arounamounchili/linkforge/commit/7def4d817904796dbe71c8390c04cee97668ded0))
* **core:** implement abstract resource resolver ([#166](https://github.com/arounamounchili/linkforge/issues/166)) ([b746e40](https://github.com/arounamounchili/linkforge/commit/b746e4097ffef91cbab23e4b6eb0915b2210eff3))
* **core:** implement semantic validation linter and deep immutability ([dfdf3b8](https://github.com/arounamounchili/linkforge/commit/dfdf3b842d07d1c5130354c232c2ae75e0ea13c8))
* **core:** implement structural XACRO caching for optimized robot assembly ([a1dc8af](https://github.com/arounamounchili/linkforge/commit/a1dc8afea9b9c28d6e07f35ec45ee370ea5f5876))
* **core:** modular validation registry and targeted checks ([#167](https://github.com/arounamounchili/linkforge/issues/167)) ([efe67ba](https://github.com/arounamounchili/linkforge/commit/efe67ba18f1e8e884bb72b3e5c7e8dec10d4cb62))
* hybrid RobotAssembly API (The Composer) ([d7947a3](https://github.com/arounamounchili/linkforge/commit/d7947a3df3954fb44b0d294c99c598ca957062be))
* Implement srdf generator ([2019465](https://github.com/arounamounchili/linkforge/commit/2019465e53fc3bbed313ecd737b23f87171845c7))
* implement SRDF intermediate representation ([960ac48](https://github.com/arounamounchili/linkforge/commit/960ac48f1e1b3b1f6c28d1a63c52db963078a0d0))
* implement SRDF XML parser with XACRO support ([8235793](https://github.com/arounamounchili/linkforge/commit/8235793e14603b47bb7e2beb97c3636feec429e2))
* introduce sliver triangle detection and structured validation architecture ([4e7264d](https://github.com/arounamounchili/linkforge/commit/4e7264d3cf67558de9649548611db75abd3d200b))
* **parsers:** add xacro console logging and custom ROS package paths ([#179](https://github.com/arounamounchili/linkforge/issues/179)) ([cb09062](https://github.com/arounamounchili/linkforge/commit/cb0906208cc1506d3e4c3e3851a95788a5a7ea43))
* ROS XACRO Parity and Blender 5.x (Python 3.13) Support ([#198](https://github.com/arounamounchili/linkforge/issues/198)) ([7bfbd57](https://github.com/arounamounchili/linkforge/commit/7bfbd574b5af638954dd5a31d8c1572548061466))


### 🐞 Bug Fixes

* blender test stability ([a4cfcd8](https://github.com/arounamounchili/linkforge/commit/a4cfcd8255f467cf9be5a0175046e85e3e77e44d))
* **blender:** sensor visibility and live physics gizmo updates  ([777fb1d](https://github.com/arounamounchili/linkforge/commit/777fb1dcb7c984e8fcc4570ee587e7011fb85c26))
* **build:** shorten blender manifest tagline to respect 64 character limit ([7cc682e](https://github.com/arounamounchili/linkforge/commit/7cc682eafc566bd1cfaac2aed99558cfca9a9f5b))
* **ci:** exclude flaky monorepo.tools from link checker and canonicalize redirects ([8f623ad](https://github.com/arounamounchili/linkforge/commit/8f623addca19f611141156406ccf0aef86435cfa))
* support disconnected links validation in blender ([870d196](https://github.com/arounamounchili/linkforge/commit/870d196ceeb27911f6bcb08d74eb3501f0ea2e8f))
* synchronize LinkForge names with Blender object names ([c844a3a](https://github.com/arounamounchili/linkforge/commit/c844a3a0fd7a7675f195b505333a63dfc690a13e))
* **urdf:** handle out-of-order elements in iterative parser ([869182a](https://github.com/arounamounchili/linkforge/commit/869182aad9b2c161913f6798d03714330af797a1))
* **xacro:** enhance spec compliance and hardening ([010eeec](https://github.com/arounamounchili/linkforge/commit/010eeec5c54a698fab7470d797d037bde77363b6))


### 📚 Documentation

* add ADR-001 for monorepo structure decision ([#163](https://github.com/arounamounchili/linkforge/issues/163)) ([2300f25](https://github.com/arounamounchili/linkforge/commit/2300f2581864a46613b7cdb0b5185a73f9798d5d))
* add julianmueller as a contributor for test, and review ([aac782f](https://github.com/arounamounchili/linkforge/commit/aac782f8c5c548b468be116e789426fba876a2cc))
* add lionelfung7 as a contributor for bug ([#199](https://github.com/arounamounchili/linkforge/issues/199)) ([5e48eaf](https://github.com/arounamounchili/linkforge/commit/5e48eaf9cf38f8dfebaac603faf607c002372839))
* add peci1 as a contributor for bug ([3f2b0ad](https://github.com/arounamounchili/linkforge/commit/3f2b0ad491068a503ab3cdbbc206f408f70bd640))
* add peci1 as a contributor for ideas ([be038b1](https://github.com/arounamounchili/linkforge/commit/be038b14bb857e318dc16573ef220a98d781c585))
* add Python API reference and SRDF documentation ([e2f1b51](https://github.com/arounamounchili/linkforge/commit/e2f1b517b1b9001267506010b604601cb786841d))
* add Vision 2030 "Universal Connector" section to VISION.md ([af96a80](https://github.com/arounamounchili/linkforge/commit/af96a80a5346146ebe940abc5c8c9d7d2da97600))
* clarify support for both comma and os path separators ([630e6ac](https://github.com/arounamounchili/linkforge/commit/630e6acbd6fab6d1882cceadb210fb542b9b6a29))
* **core:** comprehensive README rewrite and examples API update ([0be0b5c](https://github.com/arounamounchili/linkforge/commit/0be0b5c97a7d0e446eda59f878f1ee3c0f8d8a2f))
* establish "LLVM for Robotics" vision and strategic roadmap ([f912115](https://github.com/arounamounchili/linkforge/commit/f912115d9527669fc136f4fb2a4f4966bbedae88))
* exclude flaky academic URL from link checker ([894d66f](https://github.com/arounamounchili/linkforge/commit/894d66fc4d88e4dfc060ad46af09c81290a61d66))
* formalize .lf standard and consolidate strategic technical roadmap  ([082bb02](https://github.com/arounamounchili/linkforge/commit/082bb022fdda075c60c58e0a5f546ca964592e56))
* overhaul vision, streamline README, and align documentation for v1.4.0 ([e70b47c](https://github.com/arounamounchili/linkforge/commit/e70b47c88f6e44c257c297e122b0e484708a375e))
* refine README, contributing guidelines and fix citation metadata ([46216d5](https://github.com/arounamounchili/linkforge/commit/46216d5ffdd6104ed5e760f65e21417cb30376c5))
* standardize core docstrings, clean up comment silencers, and align vision roadmap ([aecc258](https://github.com/arounamounchili/linkforge/commit/aecc2583466238440a5c4a3c4eed7f913bdab1f3))
* synchronize and standardize core and blender docstrings ([b06805e](https://github.com/arounamounchili/linkforge/commit/b06805e24cf39fb8fd00c38f3994a796d1285907))
* update docs and refine core API ([f544c76](https://github.com/arounamounchili/linkforge/commit/f544c76e06fafc99484489a0d3f1a559707d1418))
* update issue assignment policies in CONTRIBUTING.md ([bd8e315](https://github.com/arounamounchili/linkforge/commit/bd8e315281fc5235548b158e164d5954a8bb3885))
* update readme file ([381040b](https://github.com/arounamounchili/linkforge/commit/381040bcfa0100365f613e4d7a16b3d897fe539d))
* update v1.3.0 roadmap status and CITATION.cff release date ([#168](https://github.com/arounamounchili/linkforge/issues/168)) ([ab1ba5b](https://github.com/arounamounchili/linkforge/commit/ab1ba5b35b63a497737e85fa887e88fc0fec64f7))


### ⚡ Performance Updates

* optimize collision quality slider with BMesh and true debounce ([db6b0c3](https://github.com/arounamounchili/linkforge/commit/db6b0c3e13346ab74ea53521d4822ae591aeafc1))


### 🛠️ Refactors

* **blender:** clean up dead code, redundant facades, and improve imports ([61a1ee6](https://github.com/arounamounchili/linkforge/commit/61a1ee6b2263473b48dfa018b7195bd26a76b062))
* **blender:** consolidate test structure and boost platform coverage ([88a47e9](https://github.com/arounamounchili/linkforge/commit/88a47e9c52f516e769ba74bc09ef0ca26f456809))
* **blender:** stabilize naming synchronization and decouple translator architecture ([ebbcac9](https://github.com/arounamounchili/linkforge/commit/ebbcac90271f33826de5bd463683dd84a1f6304a))
* clean up generator comments and overhaul core documentation ([e5c54ed](https://github.com/arounamounchili/linkforge/commit/e5c54ed12214cb0e0ac99b7cb092dc849f57112d))
* **core:** implement shared RobotXMLGenerator engine ([#186](https://github.com/arounamounchili/linkforge/issues/186)) ([0e7646e](https://github.com/arounamounchili/linkforge/commit/0e7646ee0b4f0258dda4ba0a02882de062be5ddb))
* **core:** modularize xacro parser dispatch logic ([d18193d](https://github.com/arounamounchili/linkforge/commit/d18193d00b417820816d2ea337b4f7faa4b3b1ae))
* **core:** refactor URDFParser engine ([b417148](https://github.com/arounamounchili/linkforge/commit/b417148dbb234c6d77143ddd917dd6f0545334d0))
* extract heavy scene calculations from export_panel.py draw loop ([#182](https://github.com/arounamounchili/linkforge/issues/182)) ([6a5ecb8](https://github.com/arounamounchili/linkforge/commit/6a5ecb8a1437179eda835afbbd2102a42a873136))
* finalize blender platform stability and test  ([d3abbf3](https://github.com/arounamounchili/linkforge/commit/d3abbf3496204808208bfa8da60da187eb2dc505))
* harden core model encapsulation and stabilize naming utilities ([b4fea84](https://github.com/arounamounchili/linkforge/commit/b4fea84ad947dfaaba5d95bb46339e2329bf2f7a))
* harden integration infrastructure and professionalize static analysis for v1.4.0 ([de5fd2a](https://github.com/arounamounchili/linkforge/commit/de5fd2a3dbba57ffeab44b5da85fab70ee8c54a5))
* harden LinkForge architecture and centralize Core API ([1096425](https://github.com/arounamounchili/linkforge/commit/10964257c751089b15f335623870f457253533c0))
* harden mesh inertia calculations with strict validation and topology checks ([e009352](https://github.com/arounamounchili/linkforge/commit/e0093522834a6542923637748b556d39431ce1d0))
* harden physics inertia pipeline and blender platform API architecture ([d1faba2](https://github.com/arounamounchili/linkforge/commit/d1faba27369428560d86dd1d9b29f2c3448bd0f1))
* hardened core architecture and comprehensive test stabilisation ([9da8efa](https://github.com/arounamounchili/linkforge/commit/9da8efa322313c37f9d022c648c46c7ad0e69810))
* migrate to structured error handling with ValidationErrorCode and absolute Import Normalization  ([2b4c909](https://github.com/arounamounchili/linkforge/commit/2b4c9093a686367113f23ef0f0fb2cab8448b551))
* modernize core architecture and normalize XML generation ([06eda1d](https://github.com/arounamounchili/linkforge/commit/06eda1d49c54b59f5fb8d76963eb1d8097d57e45))
* **physics:** enhance mesh inertia robustness and numerical stability ([d7d6de9](https://github.com/arounamounchili/linkforge/commit/d7d6de97ddadcb949c77e9d303a51ebe6b009c22))
* **physics:** harden mesh topology validation and inertia pipeline ([b511d6a](https://github.com/arounamounchili/linkforge/commit/b511d6acd583eeed02a56fa3a73456d84eb540c3))
* **physics:** harden mesh topology validation and inertia pipeline ([0927906](https://github.com/arounamounchili/linkforge/commit/09279066324131145af992c5af3e003b5bb93c41))
* **physics:** mesh topology validation ([5787c52](https://github.com/arounamounchili/linkforge/commit/5787c52d729c5d3d137c231f69e8fa6f294b69a4))
* rename RobotAssembly to RobotBuilder and enhance programmatic API  ([b8d1a05](https://github.com/arounamounchili/linkforge/commit/b8d1a05d59e4ad7f36d455bd1f17a36e5a8dcd80))
* unify Blender architecture and optimize scene traversal ([#183](https://github.com/arounamounchili/linkforge/issues/183)) ([a777a8e](https://github.com/arounamounchili/linkforge/commit/a777a8e3030210ae47ea99a3947f5d3218fe27bc))

## [1.3.0](https://github.com/arounamounchili/linkforge/compare/v1.2.3...v1.3.0) (2026-03-01)


### 🚀 Features

* add component search filter for component browser ([#128](https://github.com/arounamounchili/linkforge/issues/128)) ([e7ac61a](https://github.com/arounamounchili/linkforge/commit/e7ac61a62cdf37a900054ec8b8f7adf17976d411))
* enhance joint model with safety/calibration and fix xacro grouping ([#140](https://github.com/arounamounchili/linkforge/issues/140)) ([a0c0c3e](https://github.com/arounamounchili/linkforge/commit/a0c0c3efa74808e86f90b6d78982f57905bb5e35))
* enhance ros2_control intelligence and unify simplified mesh terminology ([7de79fe](https://github.com/arounamounchili/linkforge/commit/7de79feb6264a8bf0756958ac6c1906aef3d7c90))
* **ros2-control:** joint sync, xacro modularity, and 100% core test coverage ([#156](https://github.com/arounamounchili/linkforge/issues/156)) ([7b78606](https://github.com/arounamounchili/linkforge/commit/7b7860631d302048d45a01797bf455cc05259e82))


### 🐞 Bug Fixes

* **core:** resolve security gaps and documentation discrepancies ([d1dc15a](https://github.com/arounamounchili/linkforge/commit/d1dc15a6942cd5f21f457865ea47564af31cbadb))
* enhance XACRO generator to remove additional types of empty placeholder ([#160](https://github.com/arounamounchili/linkforge/issues/160)) ([bbb99e6](https://github.com/arounamounchili/linkforge/commit/bbb99e6880bcc063cc1d06b1ec58af1eac50eec1))
* resolve xacro import failures for ros2-style robot descriptions ([#134](https://github.com/arounamounchili/linkforge/issues/134)) ([c162d5c](https://github.com/arounamounchili/linkforge/commit/c162d5c8b00964c43f55ff332a51f31c97dfb109))
* robust mesh import cleanup and xacro improvements ([#136](https://github.com/arounamounchili/linkforge/issues/136)) ([2dede4f](https://github.com/arounamounchili/linkforge/commit/2dede4f30cd3c752494b692fc843ad9ff29afe1b))


### 📚 Documentation

* add andreas-loeffler as a contributor for code, and test ([#130](https://github.com/arounamounchili/linkforge/issues/130)) ([fd02561](https://github.com/arounamounchili/linkforge/commit/fd02561bf867ac49da2343648824fbe22b82c1e5))
* add dco signoff requirement ([3484345](https://github.com/arounamounchili/linkforge/commit/34843455a535bb22168abe382c0d58d112ec7671))
* add julianmueller as a contributor for ideas ([f565cd5](https://github.com/arounamounchili/linkforge/commit/f565cd554fc49a56b9538841953cbae727e35749))
* align component READMEs and refine documentation ([af63a14](https://github.com/arounamounchili/linkforge/commit/af63a14bfaa00fe816e5ffe806c63f6f2e086809))
* align multi-package roadmaps and correct architectural references ([781a9b2](https://github.com/arounamounchili/linkforge/commit/781a9b2af9919139b9b85172e238ee931c62a71b))
* clarify link/joint frame hierarchy and mesh origin behaviour ([01094ea](https://github.com/arounamounchili/linkforge/commit/01094eaa6f79fc695040055ff396d31b791d92f5))
* transition core branding to "The Linter & Bridge for Robotics ([#127](https://github.com/arounamounchili/linkforge/issues/127)) ([73e44e2](https://github.com/arounamounchili/linkforge/commit/73e44e2e299632634654421d3f095fd80f75fdc9))
* **vision:** upgrade vision image to 'the truth layer' ([#137](https://github.com/arounamounchili/linkforge/issues/137)) ([3b1eea9](https://github.com/arounamounchili/linkforge/commit/3b1eea9756e98a9803112c583cdb36ef24255502))


### ⚡ Performance Updates

* Depsgraph Caching, NumPy acceleration, and Formal Kinematic Graph ([ab4485b](https://github.com/arounamounchili/linkforge/commit/ab4485bb4250ca21bdbc9fca9650e63637d7eb13))


### 🛠️ Refactors

* modernize exception hierarchy and domain-specific validation ([3c75d15](https://github.com/arounamounchili/linkforge/commit/3c75d15f69895cd78b7c8032abea16ea077dc5ca))

## [1.2.3](https://github.com/arounamounchili/linkforge/compare/v1.2.2...v1.2.3) (2026-02-14)


### 🐞 Bug Fixes

* **blender:** achieve 100% type safety, fix regressions, and modernize imports ([570c84b](https://github.com/arounamounchili/linkforge/commit/570c84b250e38a589b785ea2a2b467d4267116b2))
* **core:** parser hardening, hybrid package resolver, and DAE restoration ([0260082](https://github.com/arounamounchili/linkforge/commit/0260082b5aed46dba3bcdebdffb4f7cd3c89d5ba))


### 📚 Documentation

* add GeorgeKugler as a contributor for bug ([#112](https://github.com/arounamounchili/linkforge/issues/112)) ([80283a2](https://github.com/arounamounchili/linkforge/commit/80283a27f143cdcfc6c44cc8f87bea9206337a18))
* synchronize documentation with v1.2.2 and refactor CI workflows ([#113](https://github.com/arounamounchili/linkforge/issues/113)) ([9c275c4](https://github.com/arounamounchili/linkforge/commit/9c275c41fb97d896c9ec1c945034d72be793a5bb))

## [1.2.2](https://github.com/arounamounchili/linkforge/compare/v1.2.1...v1.2.2) (2026-02-07)


### 🐞 Bug Fixes

* clean mesh export naming and reach add more test coverage ([#107](https://github.com/arounamounchili/linkforge/issues/107)) ([384c708](https://github.com/arounamounchili/linkforge/commit/384c708c750787b30200d7680e7abcb91f267f71))
* preserve root link world transform and robust rotation mode handling ([#106](https://github.com/arounamounchili/linkforge/issues/106)) ([d7c1185](https://github.com/arounamounchili/linkforge/commit/d7c1185bd8df55ccc6e5b50628f4bb51505d0806))
* stabilize test registration infrastructure and resolve CI collection errors ([#108](https://github.com/arounamounchili/linkforge/issues/108)) ([4d7c2fa](https://github.com/arounamounchili/linkforge/commit/4d7c2fa483d91243aa1b0e6014e7e626119631d3))


### 📚 Documentation

* add MagnusHanses as a contributor for bug ([#102](https://github.com/arounamounchili/linkforge/issues/102)) ([c44d589](https://github.com/arounamounchili/linkforge/commit/c44d5898b59d23072092c5ca2be22c15bb1abdf2))


### 🛠️ Refactors

* orthogonal adapter architecture with comprehensive test coverage ([#109](https://github.com/arounamounchili/linkforge/issues/109)) ([64c5b82](https://github.com/arounamounchili/linkforge/commit/64c5b82892b815cf135591bd09caaeb62e774d05))

## [1.2.1](https://github.com/arounamounchili/linkforge/compare/v1.2.0...v1.2.1) (2026-02-04)


### 🐞 Bug Fixes

* implement native high-fidelity XACRO parser ([#97](https://github.com/arounamounchili/linkforge/issues/97)) ([55bff31](https://github.com/arounamounchili/linkforge/commit/55bff318ab7ebec066da9b3af2de62e1cc1d9fc7))
* resolve round-trip regressions and harden numerical stability ([#98](https://github.com/arounamounchili/linkforge/issues/98)) ([0afba33](https://github.com/arounamounchili/linkforge/commit/0afba33e94555c8c794dcce6a39d3c63f860c38a))
* **xacro:** autodetect numeric types in properties/args for math eval… ([#99](https://github.com/arounamounchili/linkforge/issues/99)) ([d62c023](https://github.com/arounamounchili/linkforge/commit/d62c023436baf681ed12583d7fe4453e28b846f7))


### 📚 Documentation

* add arounamounchili as a contributor for code, design, and 2 more ([#87](https://github.com/arounamounchili/linkforge/issues/87)) ([ae6a1ef](https://github.com/arounamounchili/linkforge/commit/ae6a1ef3411ad1ad2509a3a9d4a80d2639d7b293))
* Configure all-contributors to use smaller images and remove the usage link. ([8e5558b](https://github.com/arounamounchili/linkforge/commit/8e5558b04c5fef70a677cfd82a3985109e7a1cd4))
* finalize professional branding and vision ([#81](https://github.com/arounamounchili/linkforge/issues/81)) ([7409999](https://github.com/arounamounchili/linkforge/commit/7409999f2e8a011368479b08bbb4c3b7613c859d))
* generalize transform descriptions in URDF generator and add contributor acknowledgment to README. ([40b31da](https://github.com/arounamounchili/linkforge/commit/40b31dab3ceb0a4dbee635f1ddefa6c7a3a59352))
* Remove pip installation of the current project from Read the Docs build configuration. ([10b2f31](https://github.com/arounamounchili/linkforge/commit/10b2f319cb179975d209cf9671d67c8c0ffc5d21))
* Update README to detail new features like Mimic Joints and advanced sensors, clarify existing capabilities, and refine usage instructions. ([c9dcf9b](https://github.com/arounamounchili/linkforge/commit/c9dcf9bfb14b5a505fa0d4cd8fd976d682bbcab5))


### 🛠️ Refactors

* core data safety and test coverage ([#86](https://github.com/arounamounchili/linkforge/issues/86)) ([7776f9b](https://github.com/arounamounchili/linkforge/commit/7776f9b2aff04770f9e2c2e4581b88151a25b8da))
* finalize v1.2.0 universal architecture and zero-dependency core ([#83](https://github.com/arounamounchili/linkforge/issues/83)) ([1f0bc83](https://github.com/arounamounchili/linkforge/commit/1f0bc83084fbaf8f3d6d3b5c741ad9de6538c451))
* modernize Parser API & enhance architectural stability ([#84](https://github.com/arounamounchili/linkforge/issues/84)) ([1fdf000](https://github.com/arounamounchili/linkforge/commit/1fdf000b41505e1d884f135bfe388c158863f631))

## [1.2.0](https://github.com/arounamounchili/linkforge/compare/v1.1.0...v1.2.0) (2026-01-24)


### 🚀 Features

* add centralized ros2_control dashboard and refine export logic ([69894bc](https://github.com/arounamounchili/linkforge/commit/69894bcb544cc1f35f89514e69a42746dbb11911))
* add CODEOWNERS, citations, and refine guidelines ([#39](https://github.com/arounamounchili/linkforge/issues/39)) ([#40](https://github.com/arounamounchili/linkforge/issues/40)) ([2fd4752](https://github.com/arounamounchili/linkforge/commit/2fd4752ed9a1b6963bdf1603833c32d282fc86be))
* **blender:** expose inertial origin in physics panel ([#70](https://github.com/arounamounchili/linkforge/issues/70)) ([#71](https://github.com/arounamounchili/linkforge/issues/71)) ([45b9bc2](https://github.com/arounamounchili/linkforge/commit/45b9bc2d659090edf69a466032491db830c397de))
* **blender:** implement robust persistent inertia visualization with CoM sphere ([#74](https://github.com/arounamounchili/linkforge/issues/74)) ([6d35659](https://github.com/arounamounchili/linkforge/commit/6d35659dbd08bd0f4905d4719db141fd64c3dcff))
* **blender:** modernize GPU overlays and fix viewport drawing logic ([684d6a8](https://github.com/arounamounchili/linkforge/commit/684d6a88df810d4a4d13936bff2df705608404f2))
* full core coverage ([#67](https://github.com/arounamounchili/linkforge/issues/67)) ([217123d](https://github.com/arounamounchili/linkforge/commit/217123df48309801ffc71f255c4d202f11fdb1c4))
* implement professional virtual link handling and robust alignment ([#62](https://github.com/arounamounchili/linkforge/issues/62)) ([838f25c](https://github.com/arounamounchili/linkforge/commit/838f25cca3ed78555f0a15e9f717b2c73a949bf4))
* **core:** implement 100% dependency-free robotics logic
* **blender:** transition to dependency-free extension architecture for 90% size reduction
* **blender:** remove legacy DAE/Collada support in favor of production-ready GLB/OBJ/STL metrics
* **blender:** discontinue automatic transmission conversion in favor of explicit ROS 2 Control Dashboard configuration
* **docs:** exhaustive documentation audit and synchronization with v1.2.0 UI terminology
* **docs:** update all tutorials and how-to guides to reflect new "Create Link from Mesh" and "Create Sensor" labels
* **tests:** reorganize integration tests into specialized `parsers/`, `blender/`, and `features/` subdirectories
* **tests:** centralize path management via `examples_dir` fixture and create comprehensive `tests/README.md`
* **docs:** synchronize `ARCHITECTURE.md`, `CONTRIBUTING.md`, and Sphinx reference index with new test organization


### 🐞 Bug Fixes

* add missing register/unregister logic to import_ops.py ([3017eca](https://github.com/arounamounchili/linkforge/commit/3017eca93a01edf33f62b47cdc3bcde797564bcf))
* **blender:** handle Collada removal in Blender 5.0+ ([#68](https://github.com/arounamounchili/linkforge/issues/68)) ([8e2ffbf](https://github.com/arounamounchili/linkforge/commit/8e2ffbf1b9e017cea0c49cc975ab247c9c6b4b34))
* **blender:** restore missing bpy import and add type hints to utilities ([e4cbf56](https://github.com/arounamounchili/linkforge/commit/e4cbf56032ca3b20353984340e02345d4b18bcff))
* **ci:** simplify release-please extra-files by removing dynamic docs and README ([0812552](https://github.com/arounamounchili/linkforge/commit/0812552b21d082fa97ad2d44f25fbf0f384bf6a5))
* **ci:** split blender setup steps in release pipeline ([dab8293](https://github.com/arounamounchili/linkforge/commit/dab8293d07ee9e974a9435283309989aa984a6dc))
* **ci:** switch release-please to python type for robust versioning ([1d182fd](https://github.com/arounamounchili/linkforge/commit/1d182fdddc957e2e620b5fc517287b5b33109282))
* Collision import preventing accidental degradation during the import-export cycle ([18c2530](https://github.com/arounamounchili/linkforge/commit/18c2530e3736b885c157c118f101ade80d448599))
* **core:** correct string formatting in XACRO detection error msg ([d76f0f9](https://github.com/arounamounchili/linkforge/commit/d76f0f96d05edff8effdc6f9a93a8bdadc4df675))
* implement data-level mesh cloning for export robustness ([#59](https://github.com/arounamounchili/linkforge/issues/59)) ([#60](https://github.com/arounamounchili/linkforge/issues/60)) ([9ddb06a](https://github.com/arounamounchili/linkforge/commit/9ddb06a94f9409f182e44294b4a98d1c441fc9ad))
* make all property group registrations idempotent to prevent double registration errors ([1d54cd0](https://github.com/arounamounchili/linkforge/commit/1d54cd01ad872773022b5aabd62cce7f42f0c017))
* Xacro split-file generation and  manual qa doc ([#75](https://github.com/arounamounchili/linkforge/issues/75)) ([6719a70](https://github.com/arounamounchili/linkforge/commit/6719a708ad80b34eed779b67eb9c832149df5f61))


### 📚 Documentation

* **branding:** update social preview with new tagline ([dd89679](https://github.com/arounamounchili/linkforge/commit/dd89679b487b535f7aeded79479fcbb4d7b3567b))
* clarify coordinate system uses 1:1 mapping, not conversion ([#56](https://github.com/arounamounchili/linkforge/issues/56)) ([b025d57](https://github.com/arounamounchili/linkforge/commit/b025d572177f6f7f3f640156e432d227e03a1115))
* correct object mapping table ([#45](https://github.com/arounamounchili/linkforge/issues/45)) ([de008b5](https://github.com/arounamounchili/linkforge/commit/de008b5365faa1c611328421e64ad3583a52b986))
* finalize architecture and contributing guides for v1.1.0 release ([ffb8aa6](https://github.com/arounamounchili/linkforge/commit/ffb8aa695ab242aae796c43d74d89061ff64ec9e))
* improve pre-commit installation instructions in README and CONTRIBUTING ([fd3f746](https://github.com/arounamounchili/linkforge/commit/fd3f746e5ca0ae5c58ceda7e851740d364ca73df))
* make documentation version dynamic by importing from package ([1d8b393](https://github.com/arounamounchili/linkforge/commit/1d8b39324de9d8919cbd0e3de6060bf327811366))
* refine tagline and enhance technical specifications ([24ef9fd](https://github.com/arounamounchili/linkforge/commit/24ef9fd340a4904af982f129ca453feea8afe67f))
* sync architecture and API references with modular codebase ([cf91bd9](https://github.com/arounamounchili/linkforge/commit/cf91bd9259955c8d2854528fcfb52eefd39466b4))
* sync documentation version and add automation hint ([0e80499](https://github.com/arounamounchili/linkforge/commit/0e80499089cf9b895d0a15150c6bd30c3a6ebf65))
* synchronize all documentation with v1.2.0 features ([#78](https://github.com/arounamounchili/linkforge/issues/78)) ([7f10ca8](https://github.com/arounamounchili/linkforge/commit/7f10ca8ed1c626fb027c332f5c625485f3c89f56))
* update the roadmap ([fd8d3aa](https://github.com/arounamounchili/linkforge/commit/fd8d3aacb0b230399671328aaf67d6b9766dc13d))
* use standard version variable in conf.py for automation ([d3a08da](https://github.com/arounamounchili/linkforge/commit/d3a08daf88dc5730c9d3b533a5c82bb24a9efced))


### 🛠️ Refactors

* **core:** eliminate redundant index rebuilds and harmonize comm… ([#58](https://github.com/arounamounchili/linkforge/issues/58)) ([189b6a6](https://github.com/arounamounchili/linkforge/commit/189b6a6901d20f9ee242878a149d1edb52b34c9a))
* **core:** eliminate redundant index rebuilds and harmonize comments ([189b6a6](https://github.com/arounamounchili/linkforge/commit/189b6a6901d20f9ee242878a149d1edb52b34c9a))
* delete tools directory and remove related code references ([3039e9e](https://github.com/arounamounchili/linkforge/commit/3039e9e79e1b68ddce2702ef9f38772777753316))
* delete tools directory and remove related code references ([3a83616](https://github.com/arounamounchili/linkforge/commit/3a836165bd3cd3035d999e631dbc0ae37c30d80f))
* formalize hexagonal architecture and unify collision display ([#72](https://github.com/arounamounchili/linkforge/issues/72)) ([#73](https://github.com/arounamounchili/linkforge/issues/73)) ([92c8b75](https://github.com/arounamounchili/linkforge/commit/92c8b75b8ec5ec7bb5ba66fe20f9a015066ed6dd))
* polish error message formatting in urdf_parser.py ([0240a64](https://github.com/arounamounchili/linkforge/commit/0240a642b9315ad89017e0e2e4560502c6760296))
* remove tool reference from urdf_parser.py ([ce1d403](https://github.com/arounamounchili/linkforge/commit/ce1d403fe819a8b7d9316e91567b93be4f60076e))
* restructure test suite into tiered architecture ([#63](https://github.com/arounamounchili/linkforge/issues/63)) ([#64](https://github.com/arounamounchili/linkforge/issues/64)) ([6bfb262](https://github.com/arounamounchili/linkforge/commit/6bfb262c2728aba8a767657df186aeebba3b0381))
* **arch:** formalize monorepo structure with decoupled `core` and `blender` packages
* **blender:** implement policy-compliant vendorizing for core logic (Zero `sys.path` changes)
* separate import/export logic and refine xacro error message ([5649837](https://github.com/arounamounchili/linkforge/commit/56498379e8c11d2379c259904b3d778411d29ad2))

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
