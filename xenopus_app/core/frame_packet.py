# -*- coding: utf-8 -*-
"""Data structures exchanged by the real-time tracking pipeline.

The goal of these classes is to keep frame data and tracking results grouped in
single objects instead of using several unsynchronized lists. Each acquired frame
gets a frame id, a timestamp, optional OKR state and metadata. Each tracking
result keeps the same frame id, which makes CSV export and frame-loss detection
more reliable.

@author: Courtand, Kadri 
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class FramePacket:
    """Frame acquired by the camera or loaded from a video file.

    Parameters
    ----------
    frame_id:
        Unique frame identifier used to preserve order and detect missing
        frames.
    timestamp:
        Software timestamp associated with the acquired frame.
    image:
        Raw image data, usually a NumPy array.
    okr_state:
        Optional snapshot of the optokinetic stimulation state.
    camera_timestamp:
        Optional hardware timestamp from the camera when available.
    metadata:
        Additional information such as source, camera parameters or image size.
    """

    frame_id: int
    timestamp: float
    image: Any
    okr_state: Optional[Dict[str, Any]] = None
    camera_timestamp: Optional[float] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TrackingResult:
    """Tracking output for one analyzed frame.

    Each instance corresponds to one ``FramePacket``. It contains eye angles,
    eye positions, tail measurements and the experimental state associated with
    the frame.
    """

    frame_id: int
    timestamp: float

    # Eye measurements.
    eye1_angle: Optional[float] = None
    eye2_angle: Optional[float] = None
    eye1_y: Optional[float] = None
    eye2_y: Optional[float] = None

    # Legacy single-tail measurement.
    tail_angle: Optional[float] = None
    tail_x: Optional[float] = None
    tail_y: Optional[float] = None

    # Tail measurements from R / M / C arc regions.
    tail_R_angle: Optional[float] = None
    tail_R_x: Optional[float] = None
    tail_R_y: Optional[float] = None

    tail_M_angle: Optional[float] = None
    tail_M_x: Optional[float] = None
    tail_M_y: Optional[float] = None

    tail_C_angle: Optional[float] = None
    tail_C_x: Optional[float] = None
    tail_C_y: Optional[float] = None

    # Experimental state and tracking status.
    okr_state: Optional[Dict[str, Any]] = None
    valid: bool = True
    error: Optional[str] = None

    # Extra values used by overlays and review tools.
    metadata: Dict[str, Any] = field(default_factory=dict)
