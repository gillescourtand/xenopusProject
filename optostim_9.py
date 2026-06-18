# -*- coding: utf-8 -*-
"""
Created on Thu May 23 16:04:38 2024

@author: courtand



v8g : - lines black/white
      - random dots pattern  
v8i : - create file for optostim parameters with timestamp
v9 : - create result list with timestamp when parameters have changed

"""

import sys
import pygame
import threading
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox
from PyQt5 import QtWidgets,QtCore
from PyQt5.QtCore import pyqtSignal
#pyqtgraph
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui as QtgGui
from multiprocessing import Value as MPValue
import random
from datetime import datetime
import time

import csv

# Value class to store shared parameters
class Value:
    def __init__(self, initial_value):
        self.value = initial_value




# PyQt5 Application
class UIOptostim(pg.LayoutWidget):
    
    # stim_width, stim_spacing, stim_speed, stim_switch_frequency, stim_pattern, stim_mode, stim_direction
    stim_signal=pyqtSignal(int,int,int,int,str,str,str) #speed, direction
    pygame_finished = pyqtSignal()

    # Affichage du cycle courant dans l'interface Qt
    cycle_signal = pyqtSignal(int, int, bool)  # current_cycle, total_cycle, duration_enabled
        
    
    def __init__(self):
        super().__init__()
        self.initUI()

        # Initialize the shared values
        self.stim_width = MPValue('i',5)
        self.stim_spacing = MPValue('i',20)
        self.stim_speed = MPValue('i',0)
        self.stim_switch_frequency = MPValue('i',2)
        self.stim_duration_cycle = MPValue('i',12)
        self.stim_pattern = Value('Lines')
        self.stim_mode = Value('Continue')
        # self.stim_direction = Value(1)
        self.stim_direction = MPValue('i', 1)  # 'i' indicates an integer shared value
        
        self.stim_results_list=[]
        
        self.is_running = True #MPValue('b',False)
        self.stimulation = False
        self.thread_list=[]
        
        self.stim_signal.connect(self.update_gui)
        self.cycle_signal.connect(self.update_cycle_display)
        self.pygame_finished.connect(self.cleanup_pygame_thread)
        
        """
        Use Manager to create multiple shared objects, including dicts and lists. Use Manager to share data across computers on a network.
        Use Value or Array when it is not necessary to share information across a network and the types in ctypes are sufficient for your needs.
        Value is faster than Manager.

        Warning
        By the way, sharing data across processes/threads should be avoided if possible. The code above will probably run as expected, but increase the time it takes to execute foo and things will get weird. Compare the above with:

        def foo(data, name=''):
            print type(data), data.value, name
            for j in range(1000):
                data.value += 1
        """
        

    def initUI(self):
        self.setWindowTitle('Optostimulation Control Panel')

        self.stim_mode_label = QtWidgets.QLabel('Display Mode:')

        self.stim_pattern_input = QtWidgets.QComboBox(self)
        self.stim_pattern_input.addItems(['White Lines', 'Green Lines', 'Grid', 'Diagonal Grid', 'Random dots'])
        self.stim_pattern_input.currentIndexChanged.connect(self.update_values)

        self.stim_mode_input = QtWidgets.QComboBox(self)
        self.stim_mode_input.addItems(['Continue', 'Alternate'])
        self.stim_mode_input.currentIndexChanged.connect(self.update_values)

        self.stim_direction_input = QtWidgets.QComboBox(self)
        self.stim_direction_input.addItems(['Right', 'Left'])
        self.stim_direction_input.currentIndexChanged.connect(self.change_direction)

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
        self.stim_duration_ckb.setToolTip('Enable automatic stop after the selected number of cycles')

        self.stim_current_cycle_label = QtWidgets.QLabel('Current cycle: 0 / 12')
        self.stim_current_cycle_label.setToolTip('Current optokinetic cycle / selected duration cycle')

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

        self.addWidget(display_container, row=0, col=0, colspan=3)
        self.addWidget(parameters_container, row=1, col=0, colspan=3)
        self.addWidget(control_container, row=2, col=0, colspan=3)

        self.setLayout(self.layout)



    def update_cycle_display(self, current_cycle=0, total_cycle=0, duration_enabled=False):
        """
        Met à jour l'affichage du cycle courant.
        current_cycle commence à 0 puis augmente à chaque cycle complet.
        """
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
        if self.display_button.isChecked():
            if len(self.thread_list) > 0:
                return

            self.is_running = True
            self.update_values()
            self.update_cycle_display(
                0,
                self.stim_duration_cycle.value,
                self.stim_duration_ckb.isChecked()
            )

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
                )
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
        self.thread_list.clear()
        self.is_running = True

        self.display_button.blockSignals(True)
        self.display_button.setChecked(False)
        self.display_button.blockSignals(False)

        self.update_cycle_display(
            0,
            self.stim_duration_cycle.value,
            self.stim_duration_ckb.isChecked()
        )

        print("Optostimulation window stopped, ready to restart")

    def terminate_pygame_thread(self):
        self.is_running = False
        try:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        except Exception:
            pass

    def toggle_stim(self):
        if self.stim_speed.value==0:
            # If the thread is running, stop it
            self.start_stim()
        else:
            # If the thread is not running, start it
            self.pause_stim()

    def start_stim(self):
        # Run Pygame in a separate thread
        # self.is_running.value = True
        self.pause_button.setText("Pause")  # Change button text to 'Stop'
        self.stim_speed.value = int(self.stim_speed_input.text())
        
        self.update_values()


    def pause_stim(self):
        # Stop the pygame thread by setting the is_running flag to False
        # self.is_running.value = False
        self.pause_button.setText("Start")  # Change button text to 'Start
        self.stim_speed.value = 0
        # self.save_values()

    def save_values(self):
        filePath,_ = QtWidgets.QFileDialog.getSaveFileName(self,"Save optostim values", "", ".csv") 
        csvResult=csv.writer(open(filePath+".csv","w",newline=''))
        csvResult.writerow(['timestamp','pattern','width','spacing','speed','switch_frequency','duration_cycle','duration_enabled','mode','direction','direction_value'])
        for i in range(len(self.stim_results_list)):
            csvResult.writerow(self.stim_results_list[i])



    def change_direction(self):
        if self.stim_direction_input.currentText()=="Right":
            self.stim_direction.value = 1
        else:
            self.stim_direction.value = -1
        
    def update_values(self):
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
                self.stim_duration_ckb.isChecked()
            )
            # timestamp=datetime.now().timestamp()
            # self.stim_signal.emit(timestamp,self.stim_speed.value,self.stim_direction.value)
            timestamp= time.perf_counter()
            self.stim_results_list.append([timestamp,
                                     self.stim_pattern.value,
                                     self.stim_width.value,
                                     self.stim_spacing.value,
                                     self.stim_speed.value,
                                     self.stim_switch_frequency.value,
                                     self.stim_duration_cycle.value,
                                     self.stim_duration_ckb.isChecked(),
                                     self.stim_mode.value,
                                     self.stim_direction_input.currentText(),
                                     self.stim_direction.value])
            # print(self.stim_results_list[-1])
       
            
            
        except ValueError:
            pass  # Ignore invalid input

    def update_gui(self):
        self.stim_width_input.setText(str(self.stim_width.value))
        self.stim_spacing_input.setText(str(self.stim_spacing.value))
        self.stim_speed_input.setText(str(self.stim_speed.value))
        self.stim_switch_frequency_input.setText(str(self.stim_switch_frequency.value))
        self.stim_duration_input.setText(str(self.stim_duration_cycle.value))
        if self.stim_direction.value == 1 :
            self.stim_direction_input.setCurrentText("Right")
        else:
            self.stim_direction_input.setCurrentText("Left")
        
        timestamp= time.perf_counter()
        self.stim_results_list.append([timestamp,
                                 self.stim_pattern.value,
                                 self.stim_width.value,
                                 self.stim_spacing.value,
                                 self.stim_speed.value,
                                 self.stim_switch_frequency.value,
                                 self.stim_duration_cycle.value,
                                 self.stim_duration_ckb.isChecked(),
                                 self.stim_mode.value,
                                 self.stim_direction_input.currentText(),
                                 self.stim_direction.value])
        print(self.stim_results_list[-1])
            
        
        
    
    def get_stim_direction(self):
        """ Return the current value of stim_direction """
        return self.stim_direction.value


    # Function to run Pygame in a separate thread
    def run_pygame(self,stim_width, stim_spacing, stim_speed, stim_switch_frequency, stim_pattern, stim_mode, stim_direction, is_running, update_gui_callback):
        
        self.is_running=is_running
        # Initialize pygame
        pygame.init()
    
        # Screen dimensions
        screen_width = 6000
        screen_height = 800
        
        # For Moving Random Dots
        num_dots = 200  # Customize as needed
        dot_positions = [(random.randint(0, screen_width), random.randint(0, screen_height)) for _ in range(num_dots)]
    
        
        # Colors
        white = (255, 255, 255)
        black = (0, 0, 0)
        green = (0,200,0)
        normal_color = black, white
        inverted_color = white, black
        green_color = green, black
        current_color_scheme = normal_color
    
        # Set up the display
        screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
        pygame.display.set_caption('Optostimulation')
    
        
        def calculate_line_count(screen_width, stim_width, stim_spacing):
            # Calculate how many lines fit in the screen width
            total_space_per_line = stim_width + stim_spacing
            return (screen_width // total_space_per_line) + 2
    
        # Main loop
        # is_running.value = True
        line_position = 0
        last_switch_time = pygame.time.get_ticks()  # Get the current time
        cycle = 0
        clock = pygame.time.Clock()
    
        try:
            self.cycle_signal.emit(
                cycle,
                int(self.stim_duration_input.text()),
                self.stim_duration_ckb.isChecked()
            )
        except Exception:
            self.cycle_signal.emit(cycle, 0, False)

        while self.is_running :
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.is_running = False
                    
                elif event.type == pygame.VIDEORESIZE:
                    screen_width = event.w
                    screen_height = event.h
                    screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)

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
    
            # Calculate number of lines to fit the screen
            try:
                line_count = calculate_line_count(screen_width, stim_width.value, stim_spacing.value)
            except ZeroDivisionError:
                line_count = 1
    
            # # Clear the screen
            # screen.fill(current_color_scheme[0])
    
            # Draw based on the selected mode
            if stim_pattern.value == 'Green Lines':
                # vertical lines
                # Clear the screen
                screen.fill(green_color[0])
                # Draw vertical lines
                for i in range(line_count):
                    display_x = (line_position + i * (stim_width.value + stim_spacing.value)) % screen_width
                    pygame.draw.line(screen, green_color[1], (display_x, 0), (display_x, screen_height), stim_width.value)
            
            elif stim_pattern.value == 'White Lines':
                # vertical lines
                # Clear the screen
                screen.fill(current_color_scheme[0])
                # Draw vertical lines
                for i in range(line_count):
                    display_x = (line_position + i * (stim_width.value + stim_spacing.value)) % screen_width
                    pygame.draw.line(screen, current_color_scheme[1], (display_x, 0), (display_x, screen_height), stim_width.value)
            
            elif stim_pattern.value == 'Grid':
                # Clear the screen
                screen.fill(current_color_scheme[0])
                # Draw grid of points
                point_spacing = stim_width.value + stim_spacing.value
                for x in range(0, screen_width, point_spacing):
                    for y in range(0, screen_height, point_spacing):
                        display_x = (line_position + x) % screen_width
                        pygame.draw.circle(screen, current_color_scheme[1], (display_x, y), stim_width.value)
            elif stim_pattern.value == 'Diagonal Grid':
                # Clear the screen
                screen.fill(current_color_scheme[0])
                # Draw diagonal grid of points
                point_spacing = stim_width.value + stim_spacing.value
                for y in range(0, screen_height, point_spacing):
                    offset = point_spacing // 2 if (y // point_spacing) % 2 == 1 else 0
                    for x in range(0, screen_width, point_spacing):
                        display_x = (line_position + x + offset) % screen_width
                        pygame.draw.circle(screen, current_color_scheme[1], (display_x, y), stim_width.value)
    
            elif stim_pattern.value == 'Random dots':
                # Clear the screen
                screen.fill(current_color_scheme[0])
                
                for x, y in dot_positions:
                    display_x = (x + line_position) % screen_width
                    pygame.draw.circle(screen, current_color_scheme[1], (display_x, y), stim_width.value)
    
            # Update the display
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
                            direction
                        )

            clock.tick(60)
            
            # if self.start_button.text()=="start":
            #     self.is_running=False
    
        # Properly quit pygame
        pygame.quit()
        self.pygame_finished.emit()



if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    ex = UIOptostim()
    ex.show()
    sys.exit(app.exec_())
