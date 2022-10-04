# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = '3Laws'
copyright = '2022, 3Laws Robotics Inc.'
author = '3Laws Robotics Inc'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx_rtd_theme',
    'sphinx.ext.todo',
    'sphinx.ext.githubpages',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinxcontrib.spelling',
    'sphinx_sitemap',
]

html_show_sourcelink = False

version = "0.2"

html_baseurl = "https://3lawsrobotics.github.io/3laws/"

templates_path = ['_templates']
html_static_path = ['_static']

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_context = {
    "version": version,
    "current_version": version,
    "display_lower_left": True,
}
html_theme = 'sphinx_rtd_theme'

# The master toctree document.
master_doc = 'index'

# -- Extension configuration -------------------------------------------------

import os

REPO_NAME = '3laws'

# POPULATE LINKS TO OTHER VERSIONS
html_context['versions'] = list()
html_context['versions'].append(("0.10", "/en/0.10/"))
html_context['versions'].append(("0.9", "/en/0.9/"))
html_context['versions'].append(("0.8", "/en/0.8/"))
html_context['versions'].append(("0.7", "/en/0.7/"))
html_context['versions'].append(("0.6", "/en/0.6/"))
html_context['versions'].append(("0.5", "/en/0.5/"))
html_context['versions'].append(("0.4", "/en/0.4/"))
html_context['versions'].append(("0.3", "/en/0.3/"))
html_context['versions'].append(("0.2", "/en/0.2/"))
html_context['versions'].append(("0.1", "/en/0.1/"))
