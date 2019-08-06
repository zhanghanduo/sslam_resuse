#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import textwrap
# import sphinx_rtd_theme
# sys.path.insert(0, os.path.abspath('.'))

from sphinx.builders.html import StandaloneHTMLBuilder
StandaloneHTMLBuilder.supported_image_types = [
    'image/svg+xml',
    'image/gif',
    'image/png',
    'image/jpeg'
]

# -- Project information -----------------------------------------------------

project = 'SSLAM'
copyright = '2019, Zhang Handuo'
author = 'Zhang Handuo'

# The full version, including alpha/beta/rc tags
release = '0.0.5'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [ 'breathe',
               'exhale',
               'sphinx_rtd_theme',
               'sphinx.ext.mathjax'
               # 'sphinxcontrib.doxylink.doxylink'
]

# Breathe Configuration
breathe_projects = { "sslam": "../../../docs/xml/" }
breathe_default_project = "sslam"

# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "rootFileTitle":         "Library API",
    "doxygenStripFromPath":  "..",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": True,
    # "exhaleUseDoxyfile":     True
    "exhaleDoxygenStdin": textwrap.dedent('''
        # Tell Doxygen where the source code is (yours may be different).
        INPUT                 = ../../../pose_graph/src \
        INPUT                 = ../../../slam_estimator/src
        # INPUT               = ../../../camera_models/include
        EXCLUDE               = ../../../pose_graph/src/ThirdParty/ \
        EXCLUDE               = ../../../pose_graph/src/*.cpp \
        EXCLUDE               = ../../../slam_estimator/src/*.cpp \
        EXCLUDE               = ../../../slam_estimator/src/*/*.cpp 
        EXCLUDE_PATTERNS       = *.cpp, *.cc
        EXCLUDE_SYMBOLS        = std:: \
                                 Eigen:: \
                                 DBoW2:: \
                                 DVision:: \
                                 namespace std \
                                 namespace Eigen \
                                 namespace DBoW2 \
                                 namespace DVision
        # Doxygen chokes on `NAMESPACE_BEGIN`, predfine all of these
        PREDEFINED            += NAMESPACE_BEGIN(conf)="namespace conf {"
        PREDEFINED            += NAMESPACE_END(conf)="}"
        PREDEFINED            += DOXYGEN_SHOULD_SKIP_THIS
        PREDEFINED            += DOXYGEN_DOCUMENTATION_BUILD
    '''),
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The master toctree document.
master_doc = 'index'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['*.cpp']

supress_warnings = ['ref.term',
                    'ref.ref',
                    'ref.numref',
                    'ref.keyword',
                    'ref.option',
                    'ref.citation',
                    'ref.doc',
                    'misc.highlighting_failure',
                    'toc.secnum'
                    ]


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom themes here, relative to this directory.
html_theme_path = ["_themes",]
import sphinx_rtd_theme
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# html_css_files = [
#     '_static/custom.css',
# ]

# If false, no index is generated.
html_use_index = True

# If true, the index is split into individual pages for each letter.
html_split_index = False

# The name for this set of Sphinx documents.  If None, it defaults to
# "<project> v<release> documentation".
html_title = "SSLAM Package"

# If true, the index is split into individual pages for each letter.
html_split_index = False

# If true, links to the reST sources are added to the pages.
html_show_sourcelink = False

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
html_show_sphinx = False

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

def setup(app):
    app.add_stylesheet('custom.css')


# Output file base name for HTML help builder.
htmlhelp_basename = 'SSLAMdoc'

# -- Options for LaTeX output --------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #'preamble': '',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title, author, documentclass [howto/manual]).
latex_documents = [
    ('index', 'sslam.tex', u'SSLAM',
     u'Zhang Handuo', 'manual'),
]

# doxylink = {'sslam' : ('../SSLAM.tag', '../../../docs/html/')}

