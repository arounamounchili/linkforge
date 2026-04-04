# 🧪 LinkForge Manual QA Protocol

This protocol defines the **Unified Flight Plan** required before every release. It ensures that the build is stable, the UI is responsive, and the synchronization logic is bulletproof.

---

## 📝 Test Session Metadata
*This section is filled out by the maintainer at the start of each release cycle.*

| Field | Value |
| :--- | :--- |
| **Maintainer Name** | (e.g., @arounamounchili) |
| **Date** | 2026-04-04 |
| **Blender / OS** | (e.g., Blender 4.2 LTS / macOS Sonoma 14.4) |
| **LinkForge Version** | (e.g., v1.3.0) |
| **Build Artifact** | (e.g., linkforge-blender-1.3.0-macos_arm64.zip) |

---

## 🚀 Scenario: The Lifecycle of a Robot
*Goal: Build a functional, synchronized robot from a single mesh to a verified URDF in 20 minutes.*

### 🛠 Step 1: Smoke & Installation
1.  [ ] **Clean Install**: Install the `.zip`. LinkForge appears in the `N-Panel`. Preferences are accessible.
2.  [ ] **First Link**: Create a Cube. Click `Create Link from Mesh`.
    - *Expected:* Hierarchy is automated (`[Name]_visual` is child of Link Empty). LinkForge tab is active.

### 🔗 Step 2: The Core Assembly (Kinematics & Sync)
1.  [ ] **Create Bridge**: Add a second `Empty Link`. Add a `Joint` object to the scene.
2.  [ ] **Automatic Detection**: Select the Joint. Use the **Auto (A)** picker for Parent/Child connection.
    - *Expected:* One link is assigned as `Parent` and the other as `Child`.
3.  [ ] **Kinematic Limits**: Set joint to `REVOLUTE`. Set `Lower` and `Upper` limits (e.g., -1.57 to 1.57).
4.  [ ] **Integrated Sync Test**: Rename the "Child" Link in the Outliner (e.g., `Arm_Link`).
    - *Expected:* **Joint properties**, **Sensor Attachments**, and **Control Dashboard** entries all update instantly to the new name.
4.  [ ] **Undo/Redo Resilience**: Press `Ctrl+Z` (Undo rename). Verify properties reverted. `Ctrl+Shift+Z` (Redo).

### 🧬 Step 3: Physics & Intelligence
1.  [ ] **Collision Generation**: Set `Collision Type` to `Mesh (Simplified)`. Click `Generate Collision`.
    - *Expected:* Wireframe appears. Moving the decimation slider updates the mesh **live** in the viewport.
2.  [ ] **Inertia Visualization**: Disable `Auto-Calculate Inertia`. Move the `Inertial Origin` sliders.
    - *Expected:* Viewport Gizmos (Sphere/Axes) move in real-time as values change.
3.  [ ] **Control Intelligence**: Enable `Use ROS2 Control`. Add the joint to the dashboard. Rename the joint.
    - *Expected:* The dashboard joint list reflects the new name instantly.
4.  [ ] **Perception**: Select a Link and click `Create Sensor`. Change type to `Camera`.
    - *Expected:* A Sensor Empty is created. Renaming the parent link updates the `Link Attachment` field.

### 🔍 Step 4: Validation & The Export Cycle
1.  [ ] **Live Hub**: Search for a component name in the **Component Browser**. Filter works.
2.  [ ] **Run Validation**: Run the validator.
    - *Expected:* The Component Browser shows all Links, Joints, and Sensors with green "OK" status (no error icons).
3.  [ ] **Export & Inspect**: Export to `URDF/XACRO` with `Split Files` enabled.
    - *Expected:* Folder structure is populated. XML contains the correct `xyz/rpy` and `mass/inertia` tags for your construction.
4.  [ ] **The "Cycle of Life"**: Import the exported file back into a NEW Blender file.
    - *Expected:* Hierarchy, Sensors, and Physics match the original 1:1. All links are **Locked** 🔒.

---

## 🛡️ Resilience & Security (Pre-Flight)
1.  [ ] **Sandbox Security**: Attempt to import a URDF with a mesh path escaping the package (e.g., `../../etc/passwd`).
    - *Expected:* LinkForge blocks the import with a security warning.
2.  [ ] **XACRO Math**: Import a file with `${prop * 2}` math expressions.
    - *Expected:* Values are correctly evaluated during the import process.

---

## 🏁 Final Verification
- [ ] No Python tracebacks in the **System Console**.
- [ ] Robot survives a **File Save & Reload**.

---

## 💡 Known Anomalies
*The following are intended architectural behaviors:*
- **Selection Flash**: Viewport may flash a selection outline during a property synchronization move.
- **Gizmo Delay**: Minor lag (>50ms) possible in extremely high-poly scenes (>5M tris).
- **Search Case**: Component Browser search is intentionally case-insensitive.

---

> [!IMPORTANT]
> If any step in the **Lifecycle Scenario** fails, the release IS BLOCKED. Manual verification is the final "Layer of Truth."
