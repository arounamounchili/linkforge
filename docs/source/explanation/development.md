# Development Guide

This guide covers how to set up LinkForge for development, running tests, and building the extension.

## 💻 Setup

LinkForge uses `uv` for dependency management and `just` as a command runner.

```bash
# 1. Install 'just' (Command Runner)
# macOS
brew install just
# Linux
sudo apt install just
# Windows
choco install just

# 2. Clone repository
git clone https://github.com/arounamounchili/linkforge.git
cd linkforge

# 3. Install dependencies
just install
```

## 🧪 Testing

We use `pytest` for unit and integration testing.

```bash
# Run all tests (Core + Blender)
just test

# Run only core tests
just test-core

# Run only Blender tests
just test-blender

# Run with coverage (Combined report)
just coverage
```

## ✨ Code Quality

To maintain high standards, we use `ruff` for linting and `mypy` for strict type checking.

```bash
# Check everything (Lint + Types)
just check

# Fix linting issues automatically
just fix
```

## 📦 Building & Distribution

To package LinkForge as a Blender extension:

```bash
# Build the production-ready .zip
just build
```

The package will be created in the `dist/` directory.
