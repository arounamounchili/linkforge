# Development Guide

This guide covers how to set up LinkForge for development, running tests, and building the extension.

## 💻 Setup

LinkForge uses `uv` for dependency management.

```bash
# Clone repository
git clone https://github.com/arounamounchili/linkforge.git
cd linkforge

# Install dependencies
uv sync
```

## 🧪 Testing

We use `pytest` for unit and integration testing.

```bash
# Run all tests
uv run pytest

# Run with coverage
uv run pytest --cov=linkforge_core --cov=platforms/blender/linkforge --cov-report=html
```

## ✨ Code Quality

To maintain high standards, we use `ruff` for linting and formatting, and `mypy` for type checking.

```bash
# Format code
uv run ruff format .

# Lint code
uv run ruff check .

# Type check
uv run mypy core/src/linkforge_core platforms/blender/linkforge

# Install all hooks (code quality and conventional commit messages)
uv run pre-commit install --hook-type pre-commit --hook-type commit-msg
```

## 📦 Building & Distribution

To package LinkForge as a Blender extension:

```bash
# Build the production-ready .zip
uv run python platforms/blender/scripts/build.py
```

The package will be created in the `dist/` directory.
