# LinkForge Developer Commands
# Standardizes workflows across macOS, Linux, and Windows


# Default: List available commands
default:
    @just --list

# --- Build ---

# Build all extensions (currently Blender only)
build: build-blender

# Build Blender Extension
build-blender:
    uv run python platforms/blender/scripts/build.py

# Sync Blender dependencies (downloads platform-specific wheels)
sync:
    uv run python platforms/blender/scripts/build.py sync

# --- Test ---

# Run all tests (Core + Blender)
test: test-core test-blender

# Run Core unit tests
test-core:
    uv run pytest tests/unit/core tests/integration/core

# Run Blender integration tests
test-blender:
    uv run python blender_launcher.py -- --cov=linkforge --cov-append

# Run tests with coverage
coverage:
	rm -f .coverage .coverage.*
	COVERAGE_FILE=.coverage.core uv run pytest tests/unit/core tests/integration/core --cov=linkforge_core
	COVERAGE_FILE=.coverage.blender uv run python blender_launcher.py -- --cov=linkforge --cov=linkforge_core
	uv run coverage combine
	uv run coverage html
	uv run coverage report

# Run all quality checks
check: lint type-check

# Run linter (Ruff)
lint:
    uv run ruff check .

# Fix linting issues automatically
fix:
    uv run ruff check . --fix
    uv run ruff format .

# Run type checker (MyPy)
type-check:
    uv run mypy core/src/linkforge_core platforms/blender/linkforge

# --- Maintenance ---

# Clean build artifacts, caches, and OS junk
clean:
    @rm -rf dist/ build/ *.egg-info
    @rm -rf .pytest_cache .mypy_cache .ruff_cache .codespell_cache
    @rm -rf htmlcov .coverage coverage.xml
    @find . -type d -name "__pycache__" -exec rm -rf {} +
    @find . -type f -name "*.py[co]" -delete
    @find . -name ".DS_Store" -delete
    @echo "✨ Project is clean."

# Deep clean: Includes virtual environment removal
clean-all: clean
    @echo "⚠️ Removing virtual environment..."
    @rm -rf .venv/ venv/
    @echo "💀 Everything has been removed. Run 'just install' to recover."

# Install/Sync dependencies
install:
    uv sync --all-extras
    uv run pre-commit install
