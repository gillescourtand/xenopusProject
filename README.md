# xenopus
automatic high speed video-tracking of body parts (eyes, tail segments, limb…) in aquatic small vertebrate such as larval frog [(Bacqué-Cazenave et al. Nat Commun 13, 2957 (2022)](https://doi.org/10.1038/s41467-022-30636-6)

Xenopus is a Python application for analyzing eye and tail movements in *Xenopus laevis* larvae.

The application supports both real-time camera acquisition and imported-video analysis. It can track eye movements, tail movements, optokinetic stimulation state, and export synchronized results to CSV.

The project was originally based on a large monolithic script. It has now been refactored into a modular architecture with separated packages for acquisition, tracking, UI, video analysis, stimulation, CSV export, and shared application state.

---

## Main features

### Real-time camera tracking

The application can acquire frames from a camera, display them in the interface, track the selected regions, and write the results to CSV.

The real-time mode is organized around a processing pipeline:

- acquisition produces image frames;
- tracking analyzes the frames;
- result recording writes CSV rows;
- display updates overlays and plots.

The different steps communicate through queues:

- `frame_queue`: sends frames from acquisition to tracking;
- `result_queue`: sends tracking results to CSV export;
- `display_queue`: keeps the display responsive without slowing down tracking.

This separation helps reduce frame loss and avoids blocking the camera acquisition when tracking or display updates take more time.

---

## R/M/C arc-based tail tracking

The previous rectangular tail ROI has been replaced by a more precise arc-based system.

The tail can now be tracked with three independent arc ROIs:

- `R`: rostral / first tail segment;
- `M`: middle segment;
- `C`: caudal segment.

Each arc can be enabled or disabled independently. Each arc has its own:

- threshold;
- curve setting;
- tracking point;
- root-to-tail visual line;
- output values in the CSV.

This makes the tail tracking more flexible and more adapted to curved movements than a simple rectangular ROI.

The older rectangular tail tracking path is still kept for compatibility, but the R/M/C arc workflow is now the main tracking method.

---

## Eye tracking

Eye tracking is based on OpenCV image processing.

The user places one or two eye ROIs on the image. For each eye, the software applies thresholding, contour detection, ellipse fitting, and angle correction relative to the body axis.

The result includes:

- eye angle;
- eye Y position;
- ellipse descriptor used for overlays;
- frame ID and timestamp.

Eye tracking is available in both real-time camera mode and imported-video mode.

---

## Imported video mode

The application can analyze a video file without using the live camera.

The imported-video mode allows the user to:

- open a video file;
- display the first frame;
- optionally crop the video before analysis;
- place eye ROIs and R/M/C tail arcs on the imported frame;
- analyze the full video frame by frame;
- write the tracking results to CSV;
- review analyzed frames one by one after or during processing.

The frame review system displays tracking overlays on already analyzed frames. This makes it possible to check whether the eye and tail tracking stayed correct throughout the video.

Imported-video mode uses the same tracking logic as real-time mode, so results remain consistent between both workflows.

---

## Optokinetic stimulation updates

The optokinetic stimulation panel controls the visual stimulation displayed with Pygame.

It supports:

- line patterns;
- grid patterns;
- random dots;
- green or white line displays;
- continuous or alternating movement;
- direction control;
- speed control;
- switch frequency;
- duration in cycles;
- current cycle display;
- screen selection for the stimulation window.

The current stimulation state is stored in an OKR state object. Each acquired or imported frame can keep a snapshot of this state, so the CSV can associate tracking results with stimulation parameters.

The project also includes FTDI trigger helpers for synchronization with external hardware.

---

## Save and load settings

The settings system has been updated to support both real-time and imported-video workflows.

Saved settings can include:

- analysis mode;
- eye ROIs;
- R/M/C tail arc parameters;
- thresholds;
- arc curve values;
- enabled or disabled tail arcs;
- imported-video crop;
- output folder;
- Xenopus stage;
- optional file number;
- camera UI values when available.

This makes it easier to reproduce the same analysis setup later without manually replacing every ROI and parameter.

---

## CSV output

Tracking results are exported to CSV.

The CSV can include:

- frame ID;
- timestamp;
- eye angles;
- eye Y positions;
- main tail angle and position;
- R/M/C tail angles and positions;
- OKR stimulation state;
- validity flag;
- error message if a frame could not be analyzed.

The CSV structure is centralized in the IO package to keep the output format consistent.

---

## Project structure

```text
xenopus_app/
├── acquisition/
├── controller/
├── core/
├── io/
├── pipeline/
├── rois/
├── stimulation/
├── tracking/
├── ui/
├── video/
├── constants.py
└── main_window.py

MotionAnalysis_Xenopus_v2026.py
Installation.md
requirements.txt
install_xenopus.bat
```

### `acquisition/`

Handles camera acquisition and the camera control panel.

`camera_widget.py` contains the camera interface: connection, acquisition settings, camera controls, trigger options, and live display controls.

`acquisition_worker.py` retrieves frames from the acquisition system, wraps them into `FramePacket` objects, and sends them into the pipeline queues.

### `controller/`

Coordinates the main application logic.

`app_controller.py` connects the UI, tracking pipeline, video-file analysis, CSV export, and result overlays. It prepares tracking parameters from the interface and calls the OpenCV tracking function.

### `core/`

Contains shared application data structures.

`frame_packet.py` defines `FramePacket` and `TrackingResult`, the main data objects used by the pipeline.

`app_state.py` keeps shared state classes used by the application.

`image_container.py` stores image-related runtime data.

`tracking_result.py` is kept for compatibility with older imports.

### `io/`

Handles files and persistence.

`csv_schema.py` defines the CSV columns.

`result_recorder.py` consumes `TrackingResult` objects from `result_queue` and writes them to CSV.

`settings_manager.py` saves and loads analysis settings, ROIs, arcs, thresholds, crop values, and output options.

### `pipeline/`

Handles real-time processing.

`realtime_pipeline.py` creates and connects acquisition, tracking, display, CSV recording, and performance monitoring.

`tracking_worker.py` reads frames from `frame_queue`, runs the tracking function, and sends results to `result_queue`.

`display_worker.py` updates overlays and plots without blocking acquisition or tracking.

`performance_monitor.py` tracks FPS, processing time, queue sizes, and missing frames.

### `rois/`

Handles graphical regions of interest.

`define_rois.py` contains historical ROI classes and image-display helpers.

`roi_items.py` contains graphical markers such as the root, nose, and tail points.

`tail_arc_roi.py` manages the new R/M/C arc ROIs used for tail tracking.

### `tracking/`

Contains tracking algorithms.

`opencv_tracker.py` is the main tracking module for eyes and tail.

`legacy_tracking.py` keeps older tracking helpers for compatibility and preview behavior.

`eye_tracking.py`, `tail_tracking.py`, and `tracking_models.py` are compatibility placeholders.

### `stimulation/`

Handles optokinetic stimulation and synchronization.

`optostim.py` controls the Pygame stimulation window and stimulation parameters.

`okr_state.py` stores the current stimulation state in a thread-safe way.

`ftdi_trigger.py` sends FTDI trigger signals for external synchronization.

### `ui/`

Contains interface panels and mixins.

`docks.py` switches between real-time camera mode and imported-video mode.

`imported_video_panel.py` manages video import, crop, analysis, progress, and frame-by-frame review.

`overlays.py` updates ROIs, labels, tail arcs, and tracking markers on the image.

`plot_panel.py` updates plots and handles safe camera/application closing.

`tracking_controls_panel.py` manages tracking buttons, thresholds, output settings, and preview updates.

`realtime_camera_panel.py` is a compatibility placeholder.

`styles.py` contains stylesheet helpers.

### `video/`

Handles video-related utilities.

`image_conversion.py` converts OpenCV images for pyqtgraph display.

`imported_video_worker.py` reads imported videos frame by frame, runs tracking, stores results, and writes CSV output.

`video_file_reader.py` is a compatibility placeholder.

`video_state.py` stores video and camera runtime state such as frame size, FPS, current frame, LUT, rotation, and camera parameters.

---

## Installation

See [Installation.md](Installation.md) for the full installation guide.

The recommended Windows workflow is:

```text
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe MotionAnalysis_Xenopus_v2026.py
```

You can also use the provided batch file:

```text
install_xenopus.bat
```

The `.bat` file creates the virtual environment if needed, installs or checks the dependencies from `requirements.txt`, and can launch the application.

---

## Running the application

From the project root:

```text
.venv\Scripts\python.exe MotionAnalysis_Xenopus_v2026.py
```

Or, after selecting the `.venv` interpreter in VS Code or Spyder, run:

```text
MotionAnalysis_Xenopus_v2026.py
```

---

## Build an executable

An executable can be generated with PyInstaller.

Recommended mode:

```text
.venv\Scripts\python.exe -m pip install pyinstaller
.venv\Scripts\python.exe -m PyInstaller --onedir --noconsole --name XenopusProject --add-data "Imagys_blue;Imagys_blue" MotionAnalysis_Xenopus_v2026.py
```

The executable will be generated in:

```text
dist\XenopusProject\XenopusProject.exe
```

For Basler cameras, the target computer still needs the Basler pylon software and camera drivers installed.

---

## Notes

The project is mainly designed for Windows with a Basler camera setup, but most image-processing code is based on OpenCV and can be adapted to other image sources.

The current architecture keeps some compatibility layers from the previous monolithic version. These compatibility modules are kept intentionally to avoid breaking older imports or UI callbacks while the application continues to be cleaned progressively.
