# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import json
import os

# Load version json
dir_path = os.path.dirname(os.path.realpath(__file__))
versionPath = os.path.join(dir_path, "metadata", "versions.json")
if not os.path.exists(versionPath):
    raise FileNotFoundError(f"Cannot find: {versionPath}")

with open(versionPath) as f:
    json_data = f.read()
    data = json.loads(json_data)
versions_dict = data['list']

if 'DOC_VERSION' in os.environ:
    current_version = os.environ['DOC_VERSION']
else:
    raise EnvironmentError("DOC_VERSION environment variable not found")

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

project = '3Laws'
copyright = '2022, 3Laws Robotics Inc.'
author = 'Thomas Gurriet (tgurriet@3laws.io)'
version = current_version
release = current_version
html_baseurl = "https://3lawsrobotics.github.io/3laws/"


extensions = [
    'sphinx_rtd_theme',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx_sitemap',
]

html_show_sourcelink = False

templates_path = ['_templates']
html_static_path = ['_static']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_context = {
    "version": current_version,
    "current_version": current_version,
    "versions_dict": versions_dict,
    "display_lower_left": True,
}
html_theme = 'sphinx_rtd_theme'

# The master toctree document.
master_doc = 'index'
