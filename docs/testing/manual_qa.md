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
    - *Expected:* Settings for **Joint Visualization** and **Inertia Visualization** are visible and functional.

---

## 📦 Phase 2: Link & Physics Workflow
**Goal:** Verify geometry processing and mass property configuration.

1.  [ ] **Link Creation**: Select a Mesh object and click `Create Link`.
    - *Expected:* Object is renamed with `_link` suffix (or keeps custom name if set). Link properties appear in the panel.
2.  [ ] **Collision Generation**: In the Link panel, click `Generate Collision`.
    - *Expected:* A wireframe child object is created. Slider for `Collision Quality` regenerates the mesh on the fly.
3.  [ ] **Manual Inertia**: Uncheck `Auto-Calculate Inertia`.
    - *Expected:* **Yellow Wireframe Sphere** and **Orange/White Axes** appear at the link origin.
4.  [ ] **Inertial Origin**: Offset the `Inertial Origin XYZ`.
    - *Expected:* The visual markers move in sync with the coordinates. Visibility persists when selecting other objects.

---

## 🔗 Phase 3: Joint & Kinematics Workflow
**Goal:** Validate robot assembly and hierarchy detection.

1.  [ ] **Joint Creation**: Select a Link and click `Create Joint`.
    - *Expected:* A `Joint Empty` (Arrows) is created at the link's location.
2.  [ ] **Auto-Detect**: Move two links near a joint and click `Auto-Detect Links`.
    - *Expected:* Parent and Child fields are correctly populated in the Joint panel.
3.  [ ] **Limits**: Set joint to `REVOLUTE` and enable `Use Limits`.
    - *Expected:* Min/Max angle fields appear. Values are validated (Min < Max).
4.  [ ] **Enhanced Viz**: Enable `Show Joint Frames` in Preferences and move the `Frame Size` slider.
    - *Expected:* Large RGB arrows appear and scale smoothly in the viewport.

---

## 📡 Phase 4: Hardware (Sensors & Transmissions)
**Goal:** Verify complex ROS 2 component metadata.

1.  [ ] **Sensor Attachment**: Select a Link and click `Create Sensor` in the Perceive panel.
    - *Expected:* Sensor empty is created. Type-specific settings (e.g., Camera resolution) appear when switching `Sensor Type`.
2.  [ ] **Transmission Setup**: Select a Joint and click `Create Transmission` in the Control panel.
    - *Expected:* Transmission object is created. Linked to the correct joint. Hardware interfaces (Position/Velocity/Effort) are selectable.

---

## 🚀 Phase 5: Export & Validation
**Goal:** Ensure the final output is industry-ready.

1.  [ ] **Validation Hub**: Go to `Validate & Export` and click `Validate Robot`.
    - *Expected:* The Component Browser lists all Links, Joints, Sensors. No "Generic Error" icons.
2.  [ ] **URDF Export**: Click `Export URDF/XACRO`.
    - *Expected:* Successful file generation. Check the text file:
        - `xyz` and `rpy` values match Blender's transforms.
        - `mass` and `inertia` are non-zero.
        - Mesh paths are relative to the URDF folder.
3.  [ ] **Mesh Staging**: Verify the `meshes/` folder is created next to the URDF if `Export Meshes` was checked.

---

## 🏁 Final Verification
- [ ] Robot survives a **File Save & Reload**.
- [ ] No errors in the **System Console** (`Window > Toggle System Console` on Windows/Linux or Terminal on Mac).
