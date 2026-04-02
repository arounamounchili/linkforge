# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
import typing

sys.path.insert(0, os.path.abspath("../../core/src"))  # For linkforge_core
sys.path.insert(0, os.path.abspath("../../platforms/blender"))  # For linkforge (blender)

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

from linkforge_core import __version__

project = "LinkForge"
copyright = "2026, Arouna Patouossa Mounchili"  # noqa: A001
author = "Arouna Patouossa Mounchili"
# The full version, including alpha/beta/rc tags
release = __version__

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "sphinx_autodoc_typehints",
    "myst_parser",
    "sphinx_design",
    "sphinxcontrib.mermaid",
]

templates_path = ["_templates"]
exclude_patterns = ["examples"]
suppress_warnings = ["autodoc.typehints"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static", "../assets"]
html_logo = "../assets/linkforge_logo.png"

# -- Options for HTML output -------------------------------------------------

html_theme_options = {
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 3,
    "includehidden": True,
    "titles_only": False,
    "logo_only": False,
}

# SEO & Social Media Metadata
html_favicon = "../assets/linkforge_logo.png"
html_context = {
    "display_github": True,
    "github_user": "arounamounchili",
    "github_repo": "linkforge",
    "github_version": "main",
    "conf_py_path": "/docs/source/",
    "metatags": """
        <meta property="og:title" content="LinkForge - The Linter & Bridge for Robotics" />
        <meta property="og:description" content="A professional Blender extension for roboticists. Industrial-grade validation for URDF, XACRO, and beyond." />
        <meta property="og:image" content="https://linkforge.readthedocs.io/en/latest/_static/social_preview.png" />
        <meta name="twitter:card" content="summary_large_image" />
    """,
}

# Disable 'View page source' for a cleaner professional look
html_show_sourcelink = False


# Custom CSS
def setup(app: typing.Any) -> None:
    app.add_css_file("css/custom.css")


# -- Extension configuration -------------------------------------------------

# Napoleon settings (Google-style docstrings)
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = True
napoleon_use_admonition_for_notes = True
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = True
napoleon_use_rtype = True
napoleon_preprocess_types = True
napoleon_type_aliases = None
napoleon_attr_annotations = True

# Autodoc settings
autodoc_default_options = {
    "members": True,
    "member-order": "bysource",
    "special-members": "__init__",
    "undoc-members": True,
    "exclude-members": "__weakref__",
}

# Type hints settings
typehints_fully_qualified = False
always_document_param_types = True
typehints_document_rtype = True

# Intersphinx mapping
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
}

# MyST parser settings
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]

myst_heading_anchors = 3

myst_fence_as_directive = ["mermaid"]

# Mock imports for Blender-specific modules
autodoc_mock_imports = ["bpy", "bpy_extras", "mathutils", "gpu", "gpu_extras", "numpy"]
