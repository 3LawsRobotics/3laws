# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import json
import os

project = '3Laws'
copyright = '2022, 3Laws Robotics Inc 2'
author = '3Laws Robotics Inc'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.todo',
    'sphinxcontrib.spelling',
    'sphinx_sitemap',
    'notfound.extension',
    "sphinx_multiversion",
]

version = "0.2"
release = u'0.2.0'


html_baseurl = "https://3lawsrobotics.github.io/3laws/"

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_context = {
    "display_github": True,  # Integrate GitHub
    "github_user": "3LawsRobotics",  # Username
    "github_repo": "3laws",  # Repo name
    "github_version": "master",  # Version
    "conf_py_path": "/docs",  # Path in the checkout to the docs root
}
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

smv_branch_whitelist = None
smv_outputdir_format = 'en/{config.version}'
smv_latest_version = 'dev'
smv_rename_latest_version = 'latest'
