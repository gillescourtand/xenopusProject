# -*- coding: utf-8 -*-
"""
Created on Thu May 23 16:04:38 2024

@author: Courtand, Kadri 

Optokinetic stimulation control panel.

The module provides a Qt panel for configuring the stimulus and a Pygame window
used to display moving visual patterns.

Version notes:
- v8g: black/white lines and random dot pattern.
- v8i: optostimulation parameter export with timestamps.
- v9: result list updated when parameters change.
"""

import csv
import os
import random
import sys
import threading
import time
from multiprocessing import Value as MPValue

import pygame
import pyqtgraph as pg
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal


class Value:
    """Small mutable container used for shared non-numeric parameters."""

    def __init__(self, initial_value):
        """Store the initial value."""
        self.value = initial_value


class UIOptostim(pg.LayoutWidget):
    """Qt control panel for the optokinetic stimulation window."""

    stim_signal = pyqtSignal(int, int, int, int, str, str, str)
    pygame_finished = pyqtSignal()
    cycle_signal = pyqtSignal(int, int, bool)

    def __init__(self):
        """Create the panel, initialize shared values, and connect signals."""
        super().__init__()
        self.initUI()

        self.stim_width = MPValue('i', 5)
        self.stim_spacing = MPValue('i', 20)
        self.stim_speed = MPValue('i', 0)
        self.stim_switch_frequency = MPValue('i', 2)
        self.stim_duration_cycle = MPValue('i', 12)
        self.stim_pattern = Value('Lines')
        self.stim_mode = Value('Continue')
        self.stim_direction = MPValue('i', 1)

        self.stim_results_list = []

        self.is_running = True
        self.stimulation = False
        self.thread_list = []

        self.stim_signal.connect(self.update_gui)
        self.cycle_signal.connect(self.update_cycle_display)
        self.pygame_finished.connect(self.cleanup_pygame_thread)

    def initUI(self):
        """Build the Qt widgets used to configure the stimulation."""
        self.setWindowTitle('Optostimulation Control Panel')

        self.stim_mode_label = QtWidgets.QLabel('Display Mode:')

        self.stim_pattern_input = QtWidgets.QComboBox(self)
        self.stim_pattern_input.addItems([
            'White Lines',
            'Green Lines',
            'Grid',
            'Diagonal Grid',
            'Random dots',
        ])
        self.stim_pattern_input.currentIndexChanged.connect(self.update_values)

        self.stim_mode_input = QtWidgets.QComboBox(self)
        self.stim_mode_input.addItems(['Continue', 'Alternate'])
        self.stim_mode_input.currentIndexChanged.connect(self.update_values)

        self.stim_direction_input = QtWidgets.QComboBox(self)
        self.stim_direction_input.addItems(['Right', 'Left'])
        self.stim_direction_input.currentIndexChanged.connect(self.change_direction)

        self.open_on_screen_checkbox = QtWidgets.QCheckBox('Open on selected screen', self)
        self.open_on_screen_checkbox.setToolTip(
            "If checked, the optostimulation window opens on the selected monitor."
        )

        self.screen_select_combo = QtWidgets.QComboBox(self)
        self.screen_select_combo.setToolTip(
            "Select the monitor used for the optostimulation window"
        )

        self.refresh_screens_button = QtWidgets.QPushButton('↻', self)
        self.refresh_screens_button.setMaximumWidth(34)
        self.refresh_screens_button.setToolTip("Refresh monitor list")
        self.refresh_screens_button.clicked.connect(self.refresh_screen_list)

        self.refresh_screen_list()

        display_container = QtWidgets.QWidget()
        display_layout = QtWidgets.QHBoxLayout(display_container)
        display_layout.setContentsMargins(8, 6, 8, 4)
        display_layout.setSpacing(8)
        display_layout.addWidget(self.stim_mode_label)
        display_layout.addWidget(self.stim_pattern_input, 1)
        display_layout.addWidget(self.stim_mode_input, 1)
        display_layout.addWidget(self.stim_direction_input, 1)

        self.stim_width_label = QtWidgets.QLabel('Stim Width:')
        self.stim_width_input = QtWidgets.QLineEdit(self)
        self.stim_width_input.setText('5')
        self.stim_width_input.returnPressed.connect(self.update_values)

        self.stim_spacing_label = QtWidgets.QLabel('Stim Spacing:')
        self.stim_spacing_input = QtWidgets.QLineEdit(self)
        self.stim_spacing_input.setText('20')
        self.stim_spacing_input.returnPressed.connect(self.update_values)

        self.stim_speed_label = QtWidgets.QLabel('Stim Speed:')
        self.stim_speed_input = QtWidgets.QLineEdit(self)
        self.stim_speed_input.setText('2')
        self.stim_speed_input.returnPressed.connect(self.update_values)

        self.stim_switch_frequency_label = QtWidgets.QLabel('Switch Frequency (s):')
        self.stim_switch_frequency_input = QtWidgets.QLineEdit(self)
        self.stim_switch_frequency_input.setText('2')
        self.stim_switch_frequency_input.returnPressed.connect(self.update_values)

        self.stim_duration_label = QtWidgets.QLabel('Duration (cycle):')
        self.stim_duration_input = QtWidgets.QLineEdit(self)
        self.stim_duration_input.setText('12')
        self.stim_duration_input.returnPressed.connect(self.update_values)

        self.stim_duration_ckb = QtWidgets.QCheckBox(self)
        self.stim_duration_ckb.setToolTip(
            'Enable automatic stop after the selected number of cycles'
        )

        self.stim_current_cycle_label = QtWidgets.QLabel('Current cycle: 0 / 12')
        self.stim_current_cycle_label.setToolTip(
            'Current optokinetic cycle / selected duration cycle'
        )

        parameters_container = QtWidgets.QWidget()
        parameters_layout = QtWidgets.QGridLayout(parameters_container)
        parameters_layout.setContentsMargins(8, 4, 8, 4)
        parameters_layout.setHorizontalSpacing(8)
        parameters_layout.setVerticalSpacing(8)

        parameters_layout.addWidget(self.stim_width_label, 0, 0)
        parameters_layout.addWidget(self.stim_width_input, 0, 1)
        parameters_layout.addWidget(self.stim_switch_frequency_label, 0, 2)
        parameters_layout.addWidget(self.stim_switch_frequency_input, 0, 3)

        parameters_layout.addWidget(self.stim_spacing_label, 1, 0)
        parameters_layout.addWidget(self.stim_spacing_input, 1, 1)
        parameters_layout.addWidget(self.stim_duration_label, 1, 2)
        parameters_layout.addWidget(self.stim_duration_input, 1, 3)
        parameters_layout.addWidget(self.stim_duration_ckb, 1, 4)

        parameters_layout.addWidget(self.stim_speed_label, 2, 0)
        parameters_layout.addWidget(self.stim_speed_input, 2, 1)
        parameters_layout.addWidget(self.stim_current_cycle_label, 2, 2, 1, 3)

        parameters_layout.setColumnStretch(1, 1)
        parameters_layout.setColumnStretch(3, 1)

        self.pause_button = QtWidgets.QPushButton('Pause', self)
        self.pause_button.clicked.connect(self.toggle_stim)

        self.display_button = QtWidgets.QPushButton('screen/stop', self)
        self.display_button.setCheckable(True)
        self.display_button.clicked.connect(self.start_pygame)

        control_container = QtWidgets.QWidget()
        control_layout = QtWidgets.QHBoxLayout(control_container)
        control_layout.setContentsMargins(8, 8, 8, 6)
        control_layout.setSpacing(8)
        control_layout.addWidget(self.display_button, 1)
        control_layout.addWidget(self.pause_button, 1)
        control_layout.addWidget(self.open_on_screen_checkbox, 0)
        control_layout.addWidget(self.screen_select_combo, 2)
        control_layout.addWidget(self.refresh_screens_button, 0)

        self.addWidget(display_container, row=0, col=0, colspan=3)
        self.addWidget(parameters_container, row=1, col=0, colspan=3)
        self.addWidget(control_container, row=2, col=0, colspan=3)

        self.setLayout(self.layout)

    def refresh_screen_list(self):
        """Populate the screen selector with the monitors detected by Qt."""
        try:
            current_index = self.screen_select_combo.currentIndex()
        except Exception:
            current_index = 0

        self.screen_select_combo.clear()

        try:
            screens = QtWidgets.QApplication.screens()
        except Exception:
            screens = []

        if not screens:
            self.screen_select_combo.addItem("Screen 1", {
                "index": 0,
                "x": 0,
                "y": 0,
                "width": 1200,
                "height": 800,
            })
            return

        for idx, screen in enumerate(screens):
            try:
                geometry = screen.availableGeometry()
            except Exception:
                geometry = screen.geometry()

            name = screen.name() or "Screen"
            label = "Screen {} - {} ({}x{} @ {}, {})".format(
                idx + 1,
                name,
                geometry.width(),
                geometry.height(),
                geometry.x(),
                geometry.y(),
            )

            self.screen_select_combo.addItem(label, {
                "index": idx,
                "x": geometry.x(),
                "y": geometry.y(),
                "width": geometry.width(),
                "height": geometry.height(),
                "maximize": True,
            })

        if 0 <= current_index < self.screen_select_combo.count():
            self.screen_select_combo.setCurrentIndex(current_index)

    def get_selected_screen_config(self):
        """Return the selected monitor configuration, or ``None`` when disabled."""
        try:
            if not self.open_on_screen_checkbox.isChecked():
                return None

            data = self.screen_select_combo.currentData()

            if not data:
                return None

            return dict(data)

        except Exception:
            return None

    def update_cycle_display(self, current_cycle=0, total_cycle=0, duration_enabled=False):
        """Update the displayed stimulation cycle counter."""
        try:
            current_cycle = int(current_cycle)
            total_cycle = int(total_cycle)
        except Exception:
            current_cycle = 0
            total_cycle = 0

        if duration_enabled and total_cycle > 0:
            self.stim_current_cycle_label.setText(
                "Current cycle: {} / {}".format(current_cycle, total_cycle)
            )
        else:
            self.stim_current_cycle_label.setText(
                "Current cycle: {}".format(current_cycle)
            )

    def start_pygame(self):
        """Start or stop the Pygame stimulation window from the display button."""
        if self.display_button.isChecked():
            if len(self.thread_list) > 0:
                return

            self.is_running = True
            self.update_values()
            self.update_cycle_display(
                0,
                self.stim_duration_cycle.value,
                self.stim_duration_ckb.isChecked(),
            )

            screen_config = self.get_selected_screen_config()

            self.pygame_thread = threading.Thread(
                target=self.run_pygame,
                args=(
                    self.stim_width,
                    self.stim_spacing,
                    self.stim_speed,
                    self.stim_switch_frequency,
                    self.stim_pattern,
                    self.stim_mode,
                    self.stim_direction,
                    self.is_running,
                    self.update_gui,
                    screen_config,
                ),
            )
            self.pygame_thread.daemon = True
            self.pygame_thread.start()
            self.thread_list.append(self.pygame_thread)

        else:
            self.is_running = False
            try:
                pygame.event.post(pygame.event.Event(pygame.QUIT))
            except Exception:
                pass

    def cleanup_pygame_thread(self):
        """Reset the Qt button state after the Pygame window has closed."""
        self.thread_list.clear()
        self.is_running = True

        self.display_button.blockSignals(True)
        self.display_button.setChecked(False)
        self.display_button.blockSignals(False)

        self.update_cycle_display(
            0,
            self.stim_duration_cycle.value,
            self.stim_duration_ckb.isChecked(),
        )

        print("Optostimulation window stopped, ready to restart")

    def terminate_pygame_thread(self):
        """Ask the Pygame window to close."""
        self.is_running = False
        try:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        except Exception:
            pass

    def toggle_stim(self):
        """Toggle between paused and running stimulation speed."""
        if self.stim_speed.value == 0:
            self.start_stim()
        else:
            self.pause_stim()

    def start_stim(self):
        """Resume stimulation by restoring the speed from the input field."""
        self.pause_button.setText("Pause")
        self.stim_speed.value = int(self.stim_speed_input.text())
        self.update_values()

    def pause_stim(self):
        """Pause stimulation without closing the Pygame window."""
        self.pause_button.setText("Start")
        self.stim_speed.value = 0

    def save_values(self):
        """Save all recorded stimulation parameter changes to a CSV file."""
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save optostim values",
            "",
            ".csv",
        )
        csv_result = csv.writer(open(file_path + ".csv", "w", newline=''))
        csv_result.writerow([
            'timestamp',
            'pattern',
            'width',
            'spacing',
            'speed',
            'switch_frequency',
            'duration_cycle',
            'duration_enabled',
            'mode',
            'direction',
            'direction_value',
        ])
        for i in range(len(self.stim_results_list)):
            csv_result.writerow(self.stim_results_list[i])

    def change_direction(self):
        """Update the shared direction value from the direction combo box."""
        if self.stim_direction_input.currentText() == "Right":
            self.stim_direction.value = 1
        else:
            self.stim_direction.value = -1

    def update_values(self):
        """Read the Qt inputs and store the current stimulation parameters."""
        try:
            self.stim_width.value = int(self.stim_width_input.text())
            self.stim_spacing.value = int(self.stim_spacing_input.text())
            self.stim_speed.value = int(self.stim_speed_input.text())
            self.stim_switch_frequency.value = int(self.stim_switch_frequency_input.text())
            self.stim_duration_cycle.value = int(self.stim_duration_input.text())
            self.stim_pattern.value = self.stim_pattern_input.currentText()
            self.stim_mode.value = self.stim_mode_input.currentText()
            self.update_cycle_display(
                0,
                self.stim_duration_cycle.value,
                self.stim_duration_ckb.isChecked(),
            )
            timestamp = time.perf_counter()
            self.stim_results_list.append([
                timestamp,
                self.stim_pattern.value,
                self.stim_width.value,
                self.stim_spacing.value,
                self.stim_speed.value,
                self.stim_switch_frequency.value,
                self.stim_duration_cycle.value,
                self.stim_duration_ckb.isChecked(),
                self.stim_mode.value,
                self.stim_direction_input.currentText(),
                self.stim_direction.value,
            ])

        except ValueError:
            pass

    def update_gui(self):
        """Synchronize the Qt inputs with the current shared values."""
        self.stim_width_input.setText(str(self.stim_width.value))
        self.stim_spacing_input.setText(str(self.stim_spacing.value))
        self.stim_speed_input.setText(str(self.stim_speed.value))
        self.stim_switch_frequency_input.setText(str(self.stim_switch_frequency.value))
        self.stim_duration_input.setText(str(self.stim_duration_cycle.value))

        if self.stim_direction.value == 1:
            self.stim_direction_input.setCurrentText("Right")
        else:
            self.stim_direction_input.setCurrentText("Left")

        timestamp = time.perf_counter()
        self.stim_results_list.append([
            timestamp,
            self.stim_pattern.value,
            self.stim_width.value,
            self.stim_spacing.value,
            self.stim_speed.value,
            self.stim_switch_frequency.value,
            self.stim_duration_cycle.value,
            self.stim_duration_ckb.isChecked(),
            self.stim_mode.value,
            self.stim_direction_input.currentText(),
            self.stim_direction.value,
        ])
        print(self.stim_results_list[-1])

    def get_stim_direction(self):
        """Return the current stimulation direction value."""
        return self.stim_direction.value

    def run_pygame(
        self,
        stim_width,
        stim_spacing,
        stim_speed,
        stim_switch_frequency,
        stim_pattern,
        stim_mode,
        stim_direction,
        is_running,
        update_gui_callback,
        screen_config=None,
    ):
        """Run the optokinetic stimulus display loop in a background thread."""
        self.is_running = is_running

        if screen_config is not None:
            try:
                os.environ.pop('SDL_VIDEO_CENTERED', None)
                os.environ['SDL_VIDEO_WINDOW_POS'] = "{},{}".format(
                    int(screen_config.get("x", 0)),
                    int(screen_config.get("y", 0)),
                )
            except Exception:
                pass
        else:
            try:
                os.environ.pop('SDL_VIDEO_WINDOW_POS', None)
            except Exception:
                pass

        pygame.init()

        if screen_config is not None:
            screen_width = max(100, int(screen_config.get("width", 1200)))
            screen_height = max(100, int(screen_config.get("height", 800)))
        else:
            screen_width = 1200
            screen_height = 800

        num_dots = 200
        dot_positions = [
            (random.randint(0, screen_width), random.randint(0, screen_height))
            for _ in range(num_dots)
        ]

        white = (255, 255, 255)
        black = (0, 0, 0)
        green = (0, 200, 0)
        normal_color = black, white
        inverted_color = white, black
        green_color = green, black
        current_color_scheme = normal_color

        screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
        pygame.display.set_caption('Optostimulation')

        if screen_config is not None:
            try:
                if bool(screen_config.get("maximize", True)) and sys.platform.startswith("win"):
                    import ctypes
                    hwnd = pygame.display.get_wm_info().get("window")
                    if hwnd:
                        SW_MAXIMIZE = 3
                        ctypes.windll.user32.ShowWindow(hwnd, SW_MAXIMIZE)
            except Exception:
                pass

        def calculate_line_count(screen_width, stim_width, stim_spacing):
            """Return the number of line bands required to cover the window."""
            total_space_per_line = stim_width + stim_spacing
            return (screen_width // total_space_per_line) + 2

        line_position = 0
        last_switch_time = pygame.time.get_ticks()
        cycle = 0
        clock = pygame.time.Clock()

        try:
            self.cycle_signal.emit(
                cycle,
                int(self.stim_duration_input.text()),
                self.stim_duration_ckb.isChecked(),
            )
        except Exception:
            self.cycle_signal.emit(cycle, 0, False)

        while self.is_running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.is_running = False

                elif event.type == pygame.VIDEORESIZE:
                    screen_width = event.w
                    screen_height = event.h
                    screen = pygame.display.set_mode(
                        (screen_width, screen_height),
                        pygame.RESIZABLE,
                    )

            try:
                duration_enabled = self.stim_duration_ckb.isChecked()
                duration_cycle = int(self.stim_duration_input.text())
            except ValueError:
                duration_enabled = False
                duration_cycle = 0

            self.cycle_signal.emit(cycle, duration_cycle, duration_enabled)

            if duration_enabled and cycle >= duration_cycle:
                print("Optostimulation duration cycle reached")
                self.is_running = False
                break

            try:
                line_count = calculate_line_count(
                    screen_width,
                    stim_width.value,
                    stim_spacing.value,
                )
            except ZeroDivisionError:
                line_count = 1

            if stim_pattern.value == 'Green Lines':
                screen.fill(green_color[0])
                for i in range(line_count):
                    display_x = (
                        line_position + i * (stim_width.value + stim_spacing.value)
                    ) % screen_width
                    pygame.draw.line(
                        screen,
                        green_color[1],
                        (display_x, 0),
                        (display_x, screen_height),
                        stim_width.value,
                    )

            elif stim_pattern.value == 'White Lines':
                screen.fill(current_color_scheme[0])
                for i in range(line_count):
                    display_x = (
                        line_position + i * (stim_width.value + stim_spacing.value)
                    ) % screen_width
                    pygame.draw.line(
                        screen,
                        current_color_scheme[1],
                        (display_x, 0),
                        (display_x, screen_height),
                        stim_width.value,
                    )

            elif stim_pattern.value == 'Grid':
                screen.fill(current_color_scheme[0])
                point_spacing = stim_width.value + stim_spacing.value
                for x in range(0, screen_width, point_spacing):
                    for y in range(0, screen_height, point_spacing):
                        display_x = (line_position + x) % screen_width
                        pygame.draw.circle(
                            screen,
                            current_color_scheme[1],
                            (display_x, y),
                            stim_width.value,
                        )

            elif stim_pattern.value == 'Diagonal Grid':
                screen.fill(current_color_scheme[0])
                point_spacing = stim_width.value + stim_spacing.value
                for y in range(0, screen_height, point_spacing):
                    offset = point_spacing // 2 if (y // point_spacing) % 2 == 1 else 0
                    for x in range(0, screen_width, point_spacing):
                        display_x = (line_position + x + offset) % screen_width
                        pygame.draw.circle(
                            screen,
                            current_color_scheme[1],
                            (display_x, y),
                            stim_width.value,
                        )

            elif stim_pattern.value == 'Random dots':
                screen.fill(current_color_scheme[0])
                for x, y in dot_positions:
                    display_x = (x + line_position) % screen_width
                    pygame.draw.circle(
                        screen,
                        current_color_scheme[1],
                        (display_x, y),
                        stim_width.value,
                    )

            pygame.display.flip()

            current_time = pygame.time.get_ticks()
            paused = (stim_speed.value == 0)

            if paused:
                last_switch_time = current_time
            else:
                line_position += stim_speed.value * stim_direction.value
                if stim_pattern.value != 'Random dots':
                    line_position %= (stim_width.value + stim_spacing.value)

                if stim_mode.value == "Continue":
                    last_switch_time = current_time
                else:
                    if current_time - last_switch_time >= stim_switch_frequency.value * 1000:
                        stim_direction.value *= -1
                        last_switch_time = current_time

                        if stim_direction.value == 1:
                            direction = "Right"
                            cycle += 1
                            self.cycle_signal.emit(cycle, duration_cycle, duration_enabled)
                        else:
                            direction = "Left"

                        self.stim_signal.emit(
                            stim_width.value,
                            stim_spacing.value,
                            stim_speed.value,
                            stim_switch_frequency.value,
                            str(stim_pattern.value),
                            str(stim_mode.value),
                            direction,
                        )

            clock.tick(60)

        pygame.quit()
        self.pygame_finished.emit()


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    ex = UIOptostim()
    ex.show()
    sys.exit(app.exec_())
