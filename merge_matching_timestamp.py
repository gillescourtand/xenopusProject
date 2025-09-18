# -*- coding: utf-8 -*-
"""
Created on Tue Sep  9 10:11:58 2025

@author: courtand
"""

import os
import pandas as pd

parent_dir=r"D:\Developpements\Python\Xenopus_project\Xenopus_project_2025\v17beta"
file_track=r"D:\Developpements\Python\Xenopus_project\Xenopus_project_2025\v17beta\track3.csv"
file_opto=r"D:\Developpements\Python\Xenopus_project\Xenopus_project_2025\v17beta\opto1.csv"


# Load your CSV files
df1 = pd.read_csv(file_track,skiprows=4)
df2 = pd.read_csv(file_opto)

# Convert timestamp to float (seconds)
df1["timestamp"] = pd.to_numeric(df1["timestamp"], errors="coerce")
df2["timestamp"] = pd.to_numeric(df2["timestamp"], errors="coerce")

# Sort by timestamp (required for merge_asof)
df1 = df1.sort_values("timestamp")
df2 = df2.sort_values("timestamp")

# Synchronize from the first timestamp in df1 (track)
t0 = df1["timestamp"].min()
df2 = df2[df2["timestamp"] >= t0]

# Merge on nearest timestamp
merged = pd.merge_asof(
    df1, df2,
    on="timestamp",       # column to merge on
    direction="nearest",  # can be 'backward', 'forward', or 'nearest'
    tolerance=0.05  # 50ms
)

# Fill missing values with last valid observation 
merged = merged.ffill()

print(merged.head())
result_name=os.path.join(parent_dir,"finalTrack3.csv")
merged.to_csv(result_name, index=False, header=True, sep=',', encoding='utf-8')