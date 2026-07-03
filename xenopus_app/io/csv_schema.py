# -*- coding: utf-8 -*-
"""CSV schema definitions used by the Xenopus tracking recorders.

The schema is kept in a dedicated module so CSV headers stay consistent
between real-time tracking, imported-video analysis, and future exporters.

@author: Courtand, Kadri 
"""

CSV_DELIMITER = ";"

CSV_COLUMNS = [
    "frame_id",
    "timestamp",
    "eye1_angle",
    "eye2_angle",
    "eye1_y",
    "eye2_y",
    "tail_R_angle",
    "tail_R_x",
    "tail_R_y",
    "tail_M_angle",
    "tail_M_x",
    "tail_M_y",
    "tail_C_angle",
    "tail_C_x",
    "tail_C_y",
    "okr_active",
    "okr_pause",
    "stim_width",
    "stim_spacing",
    "stim_speed",
    "stim_switch_frequency",
    "stim_duration_cycle",
    "stim_duration_enabled",
    "stim_pattern",
    "stim_mode",
    "stim_direction",
    "stim_direction_text",
    "valid",
    "error",
]


def get_csv_columns():
    """Return a copy of the active CSV column list."""
    return list(CSV_COLUMNS)
