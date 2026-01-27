#!/bin/bash

# LinkForge Professional Cleanup Script
# Optimized for safety, clarity, and comprehensiveness.

# Exit on error, undefined vars, and pipe failures
set -euo pipefail

# --- Configuration ---
# Color codes for professional output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper for labeled output
info() { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# --- Safety Check ---
# Ensure we are in the project root
if [[ ! -f "pyproject.toml" ]]; then
    error "This script must be run from the LinkForge project root (pyproject.toml not found)."
fi

# --- Argument Parsing ---
CLEAN_VENV=false
if [[ "${1:-}" == "--all" ]]; then
    CLEAN_VENV=true
fi

echo -e "${BLUE}========================================"
echo -e "   🧹  LinkForge Project Cleanup  🧹"
echo -e "========================================${NC}"

# --- 1. Python Artifacts ---
info "Cleaning Python artifacts (__pycache__, .pyc, .pyo)..."
find . -type d -name "__pycache__" -exec rm -rf {} +
find . -type f -name "*.py[co]" -delete
find . -type f -name "*\$py.class" -delete

# --- 2. Build Artifacts ---
info "Cleaning build artifacts (dist, build, .egg-info)..."
# Clean recursively for all workspace members
find . -type d -name "dist" -exec rm -rf {} +
find . -type d -name "build" -exec rm -rf {} +
find . -type d -name "*.egg-info" -exec rm -rf {} +
rm -rf .wheels/

# --- 3. Test & Lint Caches ---
info "Cleaning tool caches (pytest, mypy, ruff, codespell)..."
rm -rf .pytest_cache/
rm -rf .mypy_cache/
rm -rf .ruff_cache/
rm -rf .coverage
rm -rf htmlcov/
rm -rf coverage.xml
rm -rf .codespell_cache

# --- 4. OS & Editor Junk ---
info "Cleaning OS & Editor clutter (.DS_Store, .idea, .vscode/ipch)..."
find . -name ".DS_Store" -delete
# Note: We keep .vscode/settings.json but usually clean temporary editor noise if needed

# --- 5. Optional Virtual Environment ---
if [ "$CLEAN_VENV" = true ]; then
    warn "Removing virtual environments (.venv, venv)..."
    rm -rf .venv/
    rm -rf venv/
else
    info "Skipping virtual environment cleanup (use '${NC}./clean.sh --all${BLUE}' to remove)."
fi

echo -e "${BLUE}----------------------------------------${NC}"
success "Cleanup complete! LinkForge is now sparkling clean."
echo -e "${BLUE}========================================${NC}"
