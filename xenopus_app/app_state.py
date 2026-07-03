# -*- coding: utf-8 -*-
"""Backward-compatible access to application state classes.

The actual implementations live in :mod:`xenopus_app.core.app_state`.
This module keeps older imports working while the project is split into
smaller packages.

@author: Courtand, Kadri 
"""

from xenopus_app.core.app_state import *  # noqa: F401,F403
