# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = '3Laws'
copyright = '2022, 3Laws Robotics Inc 2'
author = '3Laws Robotics Inc'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx_rtd_theme',
    'sphinx.ext.todo',
    'sphinx.ext.githubpages',
    'sphinx.ext.viewcode',
    'sphinxcontrib.spelling',
    'sphinx_sitemap',
]

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
    "display_github": True,  # Integrate GitHub
    "github_user": "3LawsRobotics",  # Username
    "github_repo": "3laws",  # Repo name
    "github_version": "master",  # Version
    "conf_py_path": "/docs/",  # Path in the checkout to the docs root
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
html_context['versions'].append(("0.1", "/en/0.1/"))
html_context['versions'].append(("0.2", "/en/0.2/"))

# settings for creating PDF with rinoh
rinoh_documents = [
    (
        master_doc,
        'target',
        project + ' Documentation',
        '© ' + copyright,
    )
]
