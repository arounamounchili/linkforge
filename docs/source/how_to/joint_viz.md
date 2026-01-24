# How-to: Customize Joint Visualization

LinkForge provides semantic visual feedback in the viewport to help you verify robot kinematics instantly.

## 1. The XYZ Frame
When a joint is created, LinkForge renders a small coordinate frame at the joint origin:
- **Red**: X-Axis
- **Green**: Y-Axis
- **Blue**: Z-Axis

## 2. Global Visualization Settings
Joint visualization is globally managed in Blender Preferences:

1. Open Blender **Preferences** (`Edit > Preferences`).
2. Go to the **Add-ons** (or **Extensions**) tab and find **LinkForge**.
3. Locate the **Joint Visualization** section.

### Unified Sizing
The **Joint Size** slider is a master control for your viewport clarity:
- **Empty Arrows**: Adjusts the physical size of the standard Blender joint markers.
- **Enhanced Overlay**: Simultaneously scales the length of the high-visibility GPU arrows.

### Enhanced GPU Overlay (RViz-style)
For a more professional look similar to ROS tools, you can enable a thick, arrowed overlay.
1. Check **Enhanced Visualization (RViz-style)** in the preferences.
2. This adds thicker red/green/blue arrows with cone tips that remain visible even when the robot geometry is obscured.
3. The size is automatically matched to your **Joint Size** setting for a consistent look.

## 3. Cleaning the View
To hide all LinkForge markers for a final render or clean view:
- Use the standard Blender **Hide Extras** toggle in the Viewport Overlays menu (top right of the 3D View).
- Since LinkForge joints are standard Empty objects, hiding "Extras" will hide all joint, link, and sensor markers instantly.
