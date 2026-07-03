# -*- coding: utf-8 -*-
"""Compatibility import for the tracking result dataclass.

``TrackingResult`` is defined in ``frame_packet.py`` because the frame packet and
its result are exchanged together by the pipeline. This module keeps the cleaner
``xenopus_app.core.tracking_result`` import path available.

@author: Courtand, Kadri 
"""

from xenopus_app.core.frame_packet import TrackingResult
