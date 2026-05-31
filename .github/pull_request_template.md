# Pull Request

> [!IMPORTANT]
> This project uses **Conventional Commits** and **Release Please** for automated versioning.
> Please ensure your PR title follows the [Conventional Commits](https://www.conventionalcommits.org/) format (e.g., `feat: add lidar sensor`, `fix: core logic bug`).

## 📝 Description
<!-- Summarize the changes and link the relevant issue -->
- Fixes # (issue)

## 🖼️ Visual Proof
> [!TIP]
> - **Required for**: UI changes, 3D viewport features, or URDF exports.
> - **Optional for**: core logic changes, refactors, or documentation.
> Drag and drop screenshots or GIFs here.

## 🧪 How to Test
<!-- Describe the manual steps you took in Blender to verify this change -->
1. Open Blender...
2. Run operation X...
3. Verify result Y...

## 💻 Environment
- **LinkForge Version:** <!-- e.g., 1.1.0-dev -->
- **Blender Version:** <!-- e.g., 4.2.0 -->
- **Operating System:** <!-- e.g., macOS Sonoma, Windows 11 -->

## 🛠️ Type of change
- [ ] 🚀 New feature (feat)
- [ ] 🐞 Bug fix (fix)
- [ ] 🧪 Tests (test)
- [ ] 🧹 Refactor (refactor)
- [ ] 📚 Documentation (docs)
- [ ] 🎨 Style/Linting (style)
- [ ] ⚙️ Maintenance (chore)
- [ ] ⚠️ Breaking change

## 🤖 CI Compatibility
- [ ] Compatible with Python 3.11 / 3.12 / 3.13
- [ ] Compatible with Blender 4.2, 4.5, 5.1 (verified via CI matrix)
- [ ] No CI regressions expected

## ✅ Checklist
- [ ] `just test-core` passes (Core unit + integration)
- [ ] `just test-unit-blender` passes (Blender unit tests)
- [ ] `just pre-commit` passes (format, lint, type checks)
- [ ] I have verified the changes manually in the Blender viewport
- [ ] I have updated the documentation or verified no changes are needed
