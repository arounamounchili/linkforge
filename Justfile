# LinkForge Developer Commands
# Standardizes workflows across macOS, Linux, and Windows

# Default: List available commands
default:
	@just --list

# --- Setup & Maintenance ---

# Install/Sync dependencies and install git pre-commit hooks
install:
	uv sync --all-extras
	uv run pre-commit install || true

# Run all pre-commit hooks on all files
pre-commit:
	uv run pre-commit run --all-files

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


# --- Development ---

# Link Blender extension for development (Blender 4.2+)
develop:
	uv run python platforms/blender/scripts/build.py develop

# Sync Blender dependencies (downloads platform-specific wheels)
sync:
	uv run python platforms/blender/scripts/build.py sync


# --- Quality ---

# Run all quality checks (format, lint, type)
check: check-format lint type-check

# Check if code is formatted correctly
check-format:
	uv run ruff format --check .

# Run linter (Ruff)
lint:
	uv run ruff check .

# Fix linting and formatting issues automatically
fix:
	uv run ruff check . --fix
	uv run ruff format .

# Run all type checkers (MyPy + Pyright)
type-check: type-check-mypy type-check-pyright

# Run MyPy type checker
type-check-mypy:
	MYPYPATH=core/src:platforms/blender/src uv run mypy -p linkforge.core -p linkforge.blender

# Run Pyright type checker
type-check-pyright:
	uv run pyright


# --- Test ---

# Run all tests (Core + Blender)
test: test-core test-blender

# Run all unit tests (platform-independent + mock-blender)
test-unit: test-unit-core test-unit-blender

# Run all integration tests (Core + Blender)
test-integration: test-integration-core test-integration-blender

# Run Core tests (unit + integration)
test-core: test-unit-core test-integration-core

# Run Core unit tests
test-unit-core:
	uv run pytest tests/unit/core

# Run Core integration tests
test-integration-core:
	uv run pytest tests/integration/core

# Run Blender tests (unit + integration)
test-blender: test-unit-blender test-integration-blender

# Run Blender unit tests (Uses mock_bpy_env)
test-unit-blender:
	uv run pytest tests/unit/platforms/blender

# Run Blender integration tests (Requires real headlessly launched Blender)
test-integration-blender:
	uv run python scripts/blender_launcher.py -- --cov-append

# Run tests with coverage
coverage:
	@rm -f .coverage .coverage.*
	COVERAGE_FILE=.coverage.core uv run pytest tests/unit/core tests/integration/core
	COVERAGE_FILE=.coverage.blender_unit uv run pytest tests/unit/platforms/blender
	COVERAGE_FILE=.coverage.blender uv run python scripts/blender_launcher.py
	uv run coverage combine
	uv run coverage html
	uv run coverage report


# --- Build & Distribution ---

# Build all extensions (currently Blender only)
build: build-blender

# Build Blender Extension
build-blender:
	uv run python platforms/blender/scripts/build.py


# --- Documentation ---

# Build documentation (Sphinx)
docs:
	@cd docs && make html
	@echo "📖 Documentation built at docs/build/html/index.html"
