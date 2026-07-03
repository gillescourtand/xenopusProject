# -*- coding: utf-8 -*-
"""Tracking data models compatibility module.

``TrackingResult`` currently lives in ``xenopus_app.core.frame_packet`` because
it is exchanged directly between acquisition, tracking, display, and CSV
recording workers. This module re-exports it from the tracking namespace for
future compatibility.

@author: Courtand, Kadri 
"""

from xenopus_app.core.frame_packet import TrackingResult

__all__ = ["TrackingResult"]
