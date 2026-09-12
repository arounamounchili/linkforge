# How-to: Customize Joint Visualization

LinkForge provides semantic visual feedback in the viewport to help you verify robot kinematics instantly.

## 1. The XYZ Frame
When a joint is created, LinkForge renders a small coordinate frame at the joint origin:
- **Red**: X-Axis
- **Green**: Y-Axis
- **Blue**: Z-Axis

## 2. Quick Viewport Overlays (Forge Panel)
Starting in LinkForge 1.5.3+, you can manage viewport visual clarity without opening Preferences.
In the **Forge** tab (Step 1), the **Viewport Overlays** box provides instant controls:

- **Collisions**: One-click toggle to show/hide all robot collision meshes (hidden by default).
- **Joints**: Toggle visibility of all joint coordinate frames and arrow empties.
- **Inertia**: Toggle Center of Mass and principal inertia tensor visualizations.
- **Fit Gizmos to Robot**: Automatically calculates the robot's bounding box and scales all gizmos proportionally to fit the robot's physical dimensions (clamped between 5mm and 50cm).
- **Size Slider**: Live slider to dynamically adjust joint gizmo sizes.

## 3. Global Visualization Settings (Preferences)
Fine-grained visualization settings are managed in Blender Preferences:

1. Open Blender **Preferences** (`Edit > Preferences`).
2. Go to the **Add-ons** (or **Extensions**) tab and find **LinkForge**.
3. Locate the **Joint Visualization** section.

### Unified Sizing
The **Joint Size** slider is a master control for your viewport clarity:
- **Empty Arrows**: Adjusts the physical size of standard Blender joint markers.
- **Enhanced Overlay**: Simultaneously scales the length of the high-visibility GPU arrows.
- **Default Scale**: Defaults to `0.05m` (down from `0.1m`) for a cleaner initial viewport.

### Progressive Disclosure (Display Mode)
To keep complex robots with dozens of kinematic links readable:
- **All Joints**: Renders coordinate frames for every joint across the robot hierarchy simultaneously.
- **Selected Only**: Renders joint coordinate frames exclusively for the actively selected link, keeping the rest of the robot visually clean.

### Depth Testing & Occlusion (Depth Mode)
- **Always On Top (X-Ray)**: Joint frames remain fully visible even when located inside opaque robot meshes.
- **Occluded by Meshes**: Uses hardware depth buffer testing (`LESS_EQUAL`) so joint frames are hidden behind external robot geometry, preventing visual confusion on closed structures.

### Enhanced GPU Overlay (RViz-style)
For a professional robotics look matching ROS/RViz:
1. Check **Enhanced Visualization (RViz-style)** in preferences.
2. This draws refined `2.5px` red/green/blue arrows with directional cones.
3. Automatically respects the **Joint Size** and **Occlusion Mode** settings.

## 4. Cleaning the View
To hide all LinkForge markers for a final render or clean presentation:
- Toggle the **Joints** button in the **Forge > Viewport Overlays** box.
- Or use Blender's standard **Hide Extras** toggle in the Viewport Overlays menu (top right of the 3D View). Since LinkForge joints are standard Empty objects, hiding "Extras" hides all joint, link, and sensor markers instantly.
