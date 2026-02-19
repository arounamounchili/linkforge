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

# --- Test ---

# Run all tests (Core + Blender)
test: test-core test-blender

# Run Core unit tests
test-core:
    uv run pytest tests/unit/core

# Run Blender integration tests
test-blender:
    uv run python run_blender_tests.py -- --cov=linkforge --cov-append

# Run tests with coverage
coverage:
	rm -f .coverage .coverage.*
	COVERAGE_FILE=.coverage.core uv run pytest tests/unit/core --cov=linkforge_core
	COVERAGE_FILE=.coverage.blender uv run python run_blender_tests.py -- --cov=linkforge --cov=linkforge_core
	uv run coverage combine
	uv run coverage html
	uv run coverage report

# --- Quality ---

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

# Clean build artifacts
clean:
    rm -rf dist/
    rm -rf build/
    rm -rf *.egg-info
    rm -rf **/__pycache__
    rm -rf .pytest_cache
    rm -rf .mypy_cache
    rm -rf .ruff_cache
    rm -rf htmlcov
    rm -f .coverage
    rm -f coverage.xml

# Install/Sync dependencies
install:
    uv sync --all-extras
    uv run pre-commit install
