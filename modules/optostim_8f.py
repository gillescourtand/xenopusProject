# -*- coding: utf-8 -*-
"""
Created on Thu May 23 16:04:38 2024

@author: courtand
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


# Value class to store shared parameters
class Value:
    def __init__(self, initial_value):
        self.value = initial_value

# Function to run Pygame in a separate thread
def run_pygame(stim_width, stim_spacing, stim_speed, stim_switch_frequency, stim_pattern, stim_mode, stim_direction, is_running, update_gui_callback):

    # Initialize pygame
    pygame.init()

    # Screen dimensions
    screen_width = 2000
    screen_height = 800
    
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
    is_running.value = True
    line_position = 0
    last_switch_time = pygame.time.get_ticks()  # Get the current time

    while is_running.value :
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                is_running.value = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    stim_speed.value = max(1, stim_speed.value - 1)  # Ensure speed doesn't go below 1
                elif event.key == pygame.K_RIGHT:
                    stim_speed.value += 1
                elif event.key == pygame.K_w:
                    stim_width.value += 1
                elif event.key == pygame.K_s:
                    stim_width.value = max(1, stim_width.value - 1)  # Ensure width doesn't go below 1
                elif event.key == pygame.K_a:
                    stim_spacing.value = max(1, stim_spacing.value - 1)  # Ensure space doesn't go below 1
                elif event.key == pygame.K_d:
                    stim_spacing.value += 1
                elif event.key == pygame.K_UP:
                    stim_switch_frequency.value += 1
                elif event.key == pygame.K_DOWN:
                    stim_switch_frequency.value = max(1, stim_switch_frequency.value - 1)  # Ensure frequency doesn't go below 1
                update_gui_callback()  # Update the GUI with the new values

                # Toggle color scheme on 'C' key press
                if event.key == pygame.K_c:
                    if current_color_scheme == normal_color:
                        current_color_scheme = inverted_color
                    else:
                        current_color_scheme = normal_color

            elif event.type == pygame.VIDEORESIZE:
                screen_width = event.w
                screen_height = event.h
                screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)

        # Calculate number of lines to fit the screen
        try:
            line_count = calculate_line_count(screen_width, stim_width.value, stim_spacing.value)
        except ZeroDivisionError:
            line_count = 1

        # # Clear the screen
        # screen.fill(current_color_scheme[0])

        # Draw based on the selected mode
        if stim_pattern.value == 'Lines':
            # vertical lines
            # Clear the screen
            screen.fill(green_color[0])
            # Draw vertical lines
            for i in range(line_count):
                display_x = (line_position + i * (stim_width.value + stim_spacing.value)) % screen_width
                pygame.draw.line(screen, green_color[1], (display_x, 0), (display_x, screen_height), stim_width.value)
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

        # Update the display
        pygame.display.flip()
        
         

        # Move the lines or points
        line_position += stim_speed.value * stim_direction.value
        line_position %= (stim_width.value + stim_spacing.value)

        # Check if it's time to switch direction
        current_time = pygame.time.get_ticks()
        if stim_mode.value=="Continue":
            last_switch_time = current_time  # Reset the timer
        else :
            if current_time - last_switch_time >= stim_switch_frequency.value * 1000:  # Convert seconds to milliseconds
                stim_direction.value *= -1  # Reverse direction
                last_switch_time = current_time  # Reset the timer
        # print(stim_speed.value)
        # Cap the frame rate
        pygame.time.Clock().tick(60)

    # Properly quit pygame
    pygame.quit()


# PyQt5 Application
class UIOptostim(pg.LayoutWidget):
    
    # stimSignal=pyqtSignal(int)
    
    
    def __init__(self):
        super().__init__()
        self.initUI()

        # Initialize the shared values
        self.stim_width = Value(5)
        self.stim_spacing = Value(20)
        self.stim_speed = Value(0)
        self.stim_switch_frequency = Value(2)
        self.stim_pattern = Value('Lines')
        self.stim_mode = Value('Continue')
        # self.stim_direction = Value(1)
        self.stim_direction = MPValue('i', 1)  # 'i' indicates an integer shared value
        
        self.is_running = Value(False)
        
        self.thread_list=[]
        

    def initUI(self):
        self.setWindowTitle('Optostimulation Control Panel')

        # self.layout = QVBoxLayout()

        self.stim_mode_label = QtWidgets.QLabel('Display Mode:')
        
        self.stim_pattern_input = QtWidgets.QComboBox(self)
        self.stim_pattern_input.addItems(['Lines', 'Grid', 'Diagonal Grid'])
        self.stim_pattern_input.currentIndexChanged.connect(self.update_values)
        
        self.stim_mode_input = QtWidgets.QComboBox(self)
        self.stim_mode_input.addItems(['Continue', 'Alternate'])
        self.stim_mode_input.currentIndexChanged.connect(self.update_values)
        
        self.stim_direction_input = QtWidgets.QComboBox(self)
        self.stim_direction_input.addItems(['Right', 'Left'])
        self.stim_direction_input.currentIndexChanged.connect(self.update_values)
        
        splitter_display = QtWidgets.QSplitter()
        splitter_display.setOrientation(QtCore.Qt.Horizontal)
        splitter_display.setMaximumWidth(150)
        splitter_display.addWidget(self.stim_mode_label)
        splitter_display.addWidget(self.stim_pattern_input)
        splitter_display.addWidget(self.stim_mode_input)
        splitter_display.addWidget(self.stim_direction_input)


        self.stim_width_label = QtWidgets.QLabel('Stim Width:')
        self.stim_width_input = QtWidgets.QLineEdit(self)
        self.stim_width_input.setText('5')
        self.stim_width_input.returnPressed.connect(self.update_values)
        
        splitterstim_width = QtWidgets.QSplitter()
        splitterstim_width.setOrientation(QtCore.Qt.Horizontal)
        splitterstim_width.setMaximumWidth(150)
        splitterstim_width.addWidget(self.stim_width_label)
        splitterstim_width.addWidget(self.stim_width_input)

        self.stim_spacing_label = QtWidgets.QLabel('Stim pacing:')
        self.stim_spacing_input = QtWidgets.QLineEdit(self)
        self.stim_spacing_input.setText('20')
        self.stim_spacing_input.returnPressed.connect(self.update_values)
        
        splitterstim_spacing = QtWidgets.QSplitter()
        splitterstim_spacing.setOrientation(QtCore.Qt.Horizontal)
        splitterstim_spacing.setMaximumWidth(150)
        splitterstim_spacing.addWidget(self.stim_spacing_label)
        splitterstim_spacing.addWidget(self.stim_spacing_input)
        

        self.stim_speed_label = QtWidgets.QLabel('Stim Speed:')
        self.layout.addWidget(self.stim_speed_label)
        self.stim_speed_input = QtWidgets.QLineEdit(self)
        self.stim_speed_input.setText('2')
        self.stim_speed_input.returnPressed.connect(self.update_values)
        
        splitterstim_speed = QtWidgets.QSplitter()
        splitterstim_speed.setOrientation(QtCore.Qt.Horizontal)
        splitterstim_speed.setMaximumWidth(150)
        splitterstim_speed.addWidget(self.stim_speed_label)
        splitterstim_speed.addWidget(self.stim_speed_input)
        
        self.stim_switch_frequency_label = QtWidgets.QLabel('Switch Frequency:')
        self.layout.addWidget(self.stim_switch_frequency_label)
        self.stim_switch_frequency_input = QtWidgets.QLineEdit(self)
        self.stim_switch_frequency_input.setText('2')
        self.stim_switch_frequency_input.returnPressed.connect(self.update_values)
        
        splitterstim_switch_frequency = QtWidgets.QSplitter()
        splitterstim_switch_frequency.setOrientation(QtCore.Qt.Horizontal)
        splitterstim_switch_frequency.setMaximumWidth(150)
        splitterstim_switch_frequency.addWidget(self.stim_switch_frequency_label)
        splitterstim_switch_frequency.addWidget(self.stim_switch_frequency_input)

        self.start_button = QtWidgets.QPushButton('Start', self)
        self.start_button.clicked.connect(self.toggle_stim)
        
        self.display_button = QtWidgets.QPushButton('screen', self)
        self.display_button.clicked.connect(self.start_pygame)
        
        self.addWidget(splitter_display,row=0,col=0,colspan=2)
        self.addWidget(splitterstim_width,row=1,col=0)
        self.addWidget(splitterstim_spacing,row=1,col=1)
        self.addWidget(splitterstim_speed,row=2,col=0)
        self.addWidget(splitterstim_switch_frequency,row=2,col=1)
        self.addWidget(self.start_button,row=3,col=1,colspan=2)
        self.addWidget(self.display_button,row=3,col=0)

        self.setLayout(self.layout)

    

    def start_pygame(self):
        # Run Pygame in a separate thread
        if len(self.thread_list)==0 :
            self.pygame_thread = threading.Thread(target=run_pygame, args=(
                self.stim_width, self.stim_spacing, self.stim_speed, self.stim_switch_frequency, self.stim_pattern, self.stim_mode, self.stim_direction, self.is_running, self.update_gui))
            self.pygame_thread.start()
            
            self.thread_list.append(self.pygame_thread)
        else :
            if self.pygame_thread and self.pygame_thread.is_alive():
                # If necessary, you could join the thread or signal it to exit
                self.terminate_pygame_thread()  # You should implement this method
                self.pygame_thread.join()
                self.thread_list.clear()
                

    def terminate_pygame_thread(self):
        # Implement a safe way to stop the 'run_pygame' loop
        # You can use an event or a flag inside 'run_pygame' to exit its loop
        pass
    

    def toggle_stim(self):
        if self.stim_speed.value==0:
            # If the thread is running, stop it
            self.start_stim()
        else:
            # If the thread is not running, start it
            self.stop_stim()

    def start_stim(self):
        # Run Pygame in a separate thread
        # self.is_running.value = True
        self.start_button.setText("Stop")  # Change button text to 'Stop'
        self.stim_speed.value = int(self.stim_speed_input.text())

    def stop_stim(self):
        # Stop the pygame thread by setting the is_running flag to False
        # self.is_running.value = False
        self.start_button.setText("Start")  # Change button text to 'Start
        self.stim_speed.value = 0


    def update_values(self):
        try:
            self.stim_width.value = int(self.stim_width_input.text())
            self.stim_spacing.value = int(self.stim_spacing_input.text())
            self.stim_speed.value = int(self.stim_speed_input.text())
            self.stim_switch_frequency.value = int(self.stim_switch_frequency_input.text())
            self.stim_pattern.value = self.stim_pattern_input.currentText()
            self.stim_mode.value = self.stim_mode_input.currentText()
            if self.stim_direction_input.currentText()=="Right":
                self.stim_direction.value = 1
            else:
                self.stim_direction.value = -1

        except ValueError:
            pass  # Ignore invalid input

    def update_gui(self):
        self.stim_width_input.setText(str(self.stim_width.value))
        self.stim_spacing_input.setText(str(self.stim_spacing.value))
        self.stim_speed_input.setText(str(self.stim_speed.value))
        self.stim_switch_frequency_input.setText(str(self.stim_switch_frequency.value))
        
    
    def get_stim_direction(self):
        """ Return the current value of stim_direction """
        return self.stim_direction.value

if __name__ == '__main__':
    app = QtgGui.QApplication([])
    ex = UIOptostim()
    ex.show()
    sys.exit(app.exec_())
