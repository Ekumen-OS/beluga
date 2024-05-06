# Copyright 2024 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import subprocess

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Beluga"
copyright = "2022-2024 Ekumen, Inc."
author = "Ekumen Research"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}
language = "en"

extensions = [
    "myst_parser",
    "sphinx.ext.mathjax",
    "sphinx.ext.duration",
    "sphinx.ext.graphviz",
    "sphinx_copybutton",
    "sphinx.ext.inheritance_diagram",
    "sphinx_babel.autodox",
    "sphinxcontrib.bibtex",
    "sphinx_design",
]

bibtex_bibfiles = ["references.bib"]

myst_enable_extensions = [
    "attrs_inline",
    "colon_fence",
    "deflist",
    "dollarmath",
]
myst_heading_anchors = 4

autodox_projects = {
    "beluga": {
        "srcdir": "packages/beluga/docs",
        "outdir": "packages/beluga/docs/_doxygen/generated/reference",
    },
    "beluga_ros": {
        "srcdir": "packages/beluga_ros/docs",
        "outdir": "packages/beluga_ros/docs/_doxygen/generated/reference",
        "tagfiles": [
            (
                "packages/beluga/docs/_doxygen/generated/reference/html/tagfile.xml",
                "packages/beluga/docs/_doxygen/generated/reference/html",
            )
        ],
    },
    "beluga_amcl": {
        "srcdir": "packages/beluga_amcl/docs",
        "outdir": "packages/beluga_amcl/docs/_doxygen/generated/reference",
        "tagfiles": [
            (
                "packages/beluga/docs/_doxygen/generated/reference/html/tagfile.xml",
                "packages/beluga/docs/_doxygen/generated/reference/html",
            ),
            (
                "packages/beluga_ros/docs/_doxygen/generated/reference/html/tagfile.xml",
                "packages/beluga_ros/docs/_doxygen/generated/reference/html",
            ),
        ],
    },
}

exclude_patterns = [
    "README.md",
    "**/README.md",
    "venv",
    "**/venv",
    "_build",
    "**/_build",
    "**/_doxygen/*.md",
]
suppress_warnings = ["myst.header"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_book_theme"
html_theme_options = {
    "show_navbar_depth": 1,
    "collapse_navbar": True,
    "collapse_navigation": True,
    "repository_branch": "main",
    "repository_url": "https://github.com/Ekumen-OS/beluga",
    "use_repository_button": True,
    "use_edit_page_button": False,
    "home_page_in_toc": True,
    "logo": {
        "image_light": "_images/logo_with_name_light.png",
        "image_dark": "_images/logo_with_name_dark.png",
    },
}
html_last_updated_fmt = subprocess.check_output(
    ["git", "log", "--pretty=format:'%ad, %h'", "-n1"]
).decode("utf-8")
html_favicon = "_images/logo_200x200.png"
html_css_files = ["custom.css"]
html_js_files = ["custom.js"]
html_static_path = ["_static"]
