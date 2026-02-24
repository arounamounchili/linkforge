# 🧪 LinkForge Manual QA Protocol

This protocol defines the mandatory manual testing steps required before every release of LinkForge. It complements the automated unit tests by verifying UI responsiveness, visual markers, and end-to-end integration within Blender.

---

## 🏗 Phase 1: Installation & Setup (The Smoke Test)
**Goal:** Ensure the extension installs cleanly and the UI is discoverable.

1.  [ ] **Clean Install**: Remove any existing LinkForge version and install the new `.zip` package.
    - *Expected:* No Python tracebacks in the console. LinkForge appears in the `Addons/Extensions` list.
2.  [ ] **Panel Visibility**: Check the `N-Panel` (Sidebar) in the 3D Viewport.
    - *Expected:* The **LinkForge** tab exists with panels: *Forge*, *Perceive*, *Control*, and *Validate & Export*.
3.  [ ] **Preferences**: Open `Edit > Preferences > Extensions > LinkForge`.
    - *Expected:* Settings for **Joint Visualization** (Unified Size) and **Inertia Visualization** are visible and functional.

---

## 📦 Phase 2: Link & Physics Workflow
**Goal:** Verify geometry processing and mass property configuration.

### 2.1 Creation & Setup
1.  [ ] **Create Link from Mesh**: Select a Mesh and click `Create Link from Mesh`.
    - *Expected:* The object **keeps its name**. A child mesh `[Object]_visual` is created.
2.  [ ] **Add Empty Link Frame**: Click `Add Empty Link Frame` (with nothing or a non-mesh selected).
    - *Expected:* An **Empty (Plain Axes)** is created at the cursor. It is marked as a link but has 0 visuals and 0 collisions.
3.  [ ] **Status Visualization**: Select an Empty Link.
    - *Expected:* The Link Panel shows properties but **Mass** is the only physics option. **Collision tools** are disabled until geometry is added.

### 2.2 Collision Handling
1.  [ ] **Generation**: Click `Generate Collision`.
    - *Expected:* A wireframe child named `[Object]_collision` is created.
2.  [ ] **Alignment (Offset Mesh)**: Move the `_visual` object away from the link origin (e.g., G X 1) and click `Regenerate Collision`.
    - *Expected:* The collision mesh is **perfectly aligned** with the visual mesh, not doubled or shifted.
3.  [ ] **Primitive Detection**: Create a Cube, Sphere, or Cylinder link and generate collision.
    - *Expected:* Detected as `BOX/SPHERE/CYLINDER`. **Collision Quality** slider is hidden.
4.  [ ] **Mesh (Simplified) (Forced)**: Create a Cylinder. Set `Collision Type` to `Mesh (Simplified)` and click `Regenerate`.
    - *Expected:* Heuristic shows **"Type: MESH"** (not Cylinder).
    - *Expected:* The wireframe is **perfectly aligned** with the cylinder, not at `0,0,0`.
5.  [ ] **Mesh (Simplified) (Complex)**: Use a complex mesh and generate collision.
    - *Expected:* Detected as `MESH`. **Collision Quality** slider is visible.
    - *Expected:* Moving the slider regenerates the wireframe mesh (decimation).
6.  [ ] **Merged Mesh**: Create a link with **two separate meshes** as children and click `Generate Collision`.
    - *Expected:* LinkForge generates a **single** simplified mesh that encapsulates both meshes.

### 2.3 Mass Properties (Physics)
1.  [ ] **Auto-Inertia**: Change `Mass` value with `Auto-Calculate` ON.
    - *Expected:* No visual markers appear (LinkForge handles this behind the scenes).
2.  [ ] **Manual Inertia**: Uncheck `Auto-Calculate Inertia`.
    - *Expected:* **CoM Gizmos (Sphere and Axes)** appear at origin.
3.  [ ] **Inertial Offset**: Change `Inertial Origin XYZ` values.
    - *Expected:* Gizmos move relative to the link origin in real-time.
    - *Expected:* Gizmos **stay visible** even if you select a different object.

---

## 🔗 Phase 3: Joint & Kinematics Workflow
**Goal:** Validate robot assembly and hierarchy detection.

### 3.1 Joint Creation & Setup
1.  [ ] **Joint Creation**: Select a Link and click `Create Joint` in the **Forge** or **Joints** panel.
    - *Expected:* A `Joint Empty` (Arrows) is created at the link's location.
2.  [ ] **Auto-Detect Parents**: Select a Joint and click the **Auto (Auto)** icon next to "Connection".
    - *Expected:* The nearest link becomes the **Child Link** (since joint origin = child origin).
    - *Expected:* The second-nearest link becomes the **Parent Link**.
    - *Note:* If you already manually set a Child, the tool is smart enough to keep it and only find the nearest **Parent**.
3.  [ ] **Manual Override**: Verify you can still manually change the `Parent/Child Link` in the dropdowns.
4.  [ ] **Joint Limits**: Set joint to `REVOLUTE` or `PRISMATIC`.
    - *Expected:* Limit fields (Lower, Upper, Effort, Velocity) appear **immediately**.
    - *Note:* There is no "Use Limits" checkbox for these types as URDF requires them.
5.  [ ] **Optional Limits**: Set joint to `CONTINUOUS`.
    - *Expected:* A `Use Limits` checkbox appears. Enabling it shows Effort/Velocity fields only.
6.  [ ] **Mimic Joints**: Set a joint to `Mimic` another joint.
    - *Expected:* Fields for `Multiplier` and `Offset` appear.
5.  [ ] **Enhanced Viz**: Enable `Show Joint Frames` in Preferences and move the `Frame Size` slider.
    - *Expected:* Large RGB arrows appear and scale smoothly in the viewport.

---

## 📡 Phase 4: Hardware (Perceive & Control)
**Goal:** Verify complex ROS 2 component metadata.

1.  [ ] **Sensor Attachment**: Select a Link and click `Create Sensor` in the **Perceive** panel.
    - *Expected:* Sensor empty is created. Type-specific settings (e.g., Camera resolution) appear when switching `Sensor Type`.
2.  [ ] **ROS 2 Control Config**: Go to the **Control** panel and enable `Use ROS2 Control`.
    - *Expected:* "Joint Interfaces" list appears.
    - *Action:* Click `+` (Add Joint) and select a joint.
    - *Expected:* The joint is added to the list. Clicking it reveals checkboxes for `Command Interfaces` (Position/Velocity/Effort) and `State Interfaces`. Verify you can toggle them.

---

## 🚀 Phase 5: Export & Validation
**Goal:** Ensure the exported output is valid and compliant.

1.  [ ] **Validation Hub**: Go to `Validate & Export` and click `Validate Robot`.
    - *Expected:* The Component Browser lists all Links, Joints, Sensors. No "Generic Error" icons.
2.  [ ] **URDF Export**: Click `Export URDF/XACRO`.
    - *Expected:* Successful file generation. Check the text file:
        - `xyz` and `rpy` values match Blender's transforms.
        - `mass` and `inertia` are non-zero.
        - Mesh paths are relative to the URDF folder.
3.  [ ] **XACRO Advanced**: Switch to XACRO format and enable **Extract Materials**, **Extract Dimensions**, and **Generate Macros**.
    - *Expected:* The output file uses `<xacro:macro>` and `<xacro:property>`.
    - *Expected:* Property names starting with digits (e.g., `001`) are automatically sanitized (e.g., `_001`).
4.  [ ] **XACRO Split Files**: Enable **Split Files**.
    - *Expected:* Generation completes successfully.
    - *Expected:* `*_robot.xacro` contains `<!-- Properties -->` and `<!-- Macros -->` comments above the corresponding `<xacro:include>` tags.
    - *Expected:* Split files (`*_properties.xacro`, `*_macros.xacro`) are created and contain their own `<robot>` root tags.
5.  [ ] **Mesh Staging**: Verify the `meshes/` folder is created next to the URDF if `Export Meshes` was checked.

---

## 🛟 Phase 6: Data Resilience
**Goal:** Verify round-trip integrity and data resilience.

1.  [ ] **Round-Trip Import**: Export your robot, then use `File > Import > LinkForge URDF (.urdf/.xacro)` to import it back into a clean scene.
    - *Expected:* The robot hierarchy is recreated exactly. Sensors are attached to the correct links.
    - *Expected:* Physics properties (Mass, Inertia) match the original values.
    - *Expected:* All links show the 🔒 **Locked** status for collision and inertia.
2.  [ ] **Undo/Redo Stress Test**: Create a Joint, move it, then press `Ctrl+Z` (Undo) and `Ctrl+Shift+Z` (Redo) multiple times.
    - *Expected:* The object disappears and reappears cleanly without Python errors.
3.  [ ] **Deletion Cleanup**: Delete a Link object (Empty) in the viewport.
    - *Expected:* Its children (Visuals/Collisions) remain but are no longer locked to the LinkForge system.
    - *Expected:* No "ghost" data remains in the scene properties.

---

## 🔐 Phase 7: Security & XACRO Advanced Features
**Goal:** Verify sandbox security and XACRO property substitution.

### 7.1 Sandbox Root & Sibling Folder Access
1.  [ ] **Standard URDF Structure**: Create a test URDF in `/my_robot/urdf/robot.urdf` with mesh paths like `../meshes/part.stl`.
    - *Expected:* Import succeeds. LinkForge auto-detects `/my_robot` as the sandbox root.
    - *Expected:* Meshes from the sibling `meshes/` folder load correctly.
2.  [ ] **Package.xml Detection**: Create a `package.xml` file in `/my_robot/` and a URDF in `/my_robot/config/deep/robot.urdf`.
    - *Expected:* LinkForge finds the package root by detecting `package.xml` up to 5 levels up.
    - *Expected:* Sibling folder access works from the package root.
3.  [ ] **Path Traversal Prevention**: Manually edit a URDF to include a mesh path like `../../../../etc/passwd`.
    - *Expected:* Import fails with a security error: "attempts to escape the sandbox root".

### 7.2 XACRO Property Substitution & Math
1.  [ ] **Property Substitution**: Create a XACRO file with `<xacro:property name="arm_length" value="2.0"/>` and use `${arm_length}` in an origin tag.
    - *Expected:* Import succeeds. The link origin uses the exact value `2.0`.
2.  [ ] **Math Expressions**: Use `${arm_length * 2}` in a XACRO origin tag.
    - *Expected:* Import succeeds. The value is correctly evaluated to `4.0`.
3.  [ ] **Nested Properties**: Define `<xacro:property name="base" value="1.0"/>` and `<xacro:property name="derived" value="${base * 3}"/>`.
    - *Expected:* Both properties are correctly substituted and evaluated.

---

## 🏁 Final Verification
- [ ] Robot survives a **File Save & Reload**.
- [ ] No errors in the **System Console** (`Window > Toggle System Console` on Windows/Linux or Terminal on Mac).
