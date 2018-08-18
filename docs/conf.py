# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import os

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.coverage',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.extlinks',
    'sphinx.ext.ifconfig',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
]
if os.getenv('SPELLCHECK'):
    extensions += 'sphinxcontrib.spelling',
    spelling_show_suggestions = True
    spelling_lang = 'en_US'

source_suffix = '.rst'
master_doc = 'index'
project = 'compas_fab'
year = '2018'
author = 'Gramazio Kohler Research'
copyright = '{0}, {1}'.format(year, author)
version = release = '0.1.0'

pygments_style = 'sphinx'
show_authors = True
add_module_names = True
templates_path = ['_templates', ]
extlinks = {
    'issue': ('https://github.com/gramaziokohler/compas_fab/issues/%s', '#'),
    'pr': ('https://github.com/gramaziokohler/compas_fab/pull/%s', 'PR #'),
}

# intersphinx options
intersphinx_mapping = {'python': ('https://docs.python.org/', None),
                       'compas': ('http://compas-dev.github.io/main/', None)}

# autodoc options
autodoc_default_flags = [
    'undoc-members',
    'private-members',
    'special-members',
    'show-inheritance',
]

autodoc_member_order = 'alphabetical'

# autosummary options
autosummary_generate = True

# on_rtd is whether we are on readthedocs.org
on_rtd = os.environ.get('READTHEDOCS', None) == 'True'
html_theme = 'compas'
html_theme_path = ['../../sphinx_compas_theme', '../temp/sphinx_compas_theme']
html_theme_options = {
    'navbar_active': 'packages',
}

html_split_index = False
html_short_title = '%s-%s' % (project, version)
html_context = {}
html_static_path = ['_static']
html_last_updated_fmt = '%b %d, %Y'
html_copy_source = False
html_show_sourcelink = False
html_add_permalinks = ''
html_experimental_html5_writer = True
html_compact_lists = True

# napoleon options
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = True
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = False
napoleon_use_admonition_for_notes = False
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = False
napoleon_use_rtype = False

