import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt, QSize

import redis
import numpy as np
import os

"""
TODOs
- reset all button might be nice to have
"""

class MainWindow(QWidget):
    def __init__(self):

        QWidget.__init__(self)
        # super().__init__()
        self.redis_client = redis.Redis()

        # set window size and position
        self.setWindowTitle("Conun-drum App")
        self.setGeometry(1000, 0, 1024, 512)

        # set constants and flags
        self.n_measures = 1 # number of measures
        self.beats_per_measure = 8 # beats per measure
        self.time_sig = 4 # time signature: 4 means 4/4
        self.is_playing = False # whether the simulation should be running
        self.tempo = 4 # tempo in BPM 

        # define and initialize redis keys
        self.IS_PLAYING_KEY = "gui::is_playing" # should simulation be running?
        self.redis_client.set(self.IS_PLAYING_KEY, 0)
        self.BPM_KEY = "gui::bpm" # BPM
        self.LOOPTIME_KEY = "gui::looptime" # seconds per loop
        
        self.RH_PLAYS_KEY = "gui::rh_plays"
        self.RF_PLAYS_KEY = "gui::rf_plays"
        self.LH_PLAYS_KEY = "gui::lh_plays"
        self.LF_PLAYS_KEY = "gui::lf_plays"
        self.redis_client.set(self.RH_PLAYS_KEY, "1")
        self.redis_client.set(self.RF_PLAYS_KEY, "1")
        self.redis_client.set(self.LH_PLAYS_KEY, "1")
        self.redis_client.set(self.LF_PLAYS_KEY, "1")


        # instruments and coordinates
        # ordered according to position on drum score (top to bottom)
        self.instrument_names = ["Tom 1", "Tom 2", "Snare", "Bass", "Hi-Hat Pedal"]
        self.instrument_names = np.array(self.instrument_names)
        self.n_instruments = len(self.instrument_names)

        # order must match self.instrument_names
        self.inst_to_coords = { 
            "Tom 1": np.array([0.72103, 0.18308, -0.35312]),
            "Tom 2": np.array([0.74235, -0.18308, -0.26023]),
            "Snare": np.array([0.36543, 0.32512, -0.50876]),
            # "Bass": np.array([0, 0, 0]),
            # "Hi-Hat Pedal": np.array([0, 0, 0]),
            }

        self.coords = np.array( list(self.inst_to_coords.values()) )

        # instruments from right to left: inst2_r2l["instrument name"] = 0 for right-most instrument 
        self.inst_r2l = dict(sorted(self.inst_to_coords.items(), key=lambda item: item[1][1]))
        for i, key in enumerate(self.inst_r2l.keys()):
            self.inst_r2l[key] = i

        self.rh_priority_idx = len(self.inst_r2l) // 2

        # list to store all buttons in measure grid
        self.measure_buttons = [ [] for i in range(self.n_instruments) ]

        # array containing button activation status
        self.button_activation = np.zeros((self.n_instruments, self.n_measures * self.beats_per_measure))

        # tempo slider and text objects
        self.tempo_slider = None
        self.tempo_text = None

        # error text object
        self.error_text = None

        self._initGUI()

    def _initGUI(self):
        """
        Initialize GUI layout and components
        """

        bg_image = QImage("gui_bgnd.png").scaled(QSize(1024, 512))
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(bg_image))
        self.setPalette(palette)


        # Root parent layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(30)

        # score grid layout
        score_layout = QGridLayout()
        score_layout.setVerticalSpacing(0)
        score_layout.setHorizontalSpacing(0)

        # Add instument names to each row
        for i, inst in enumerate(self.instrument_names):
            text = QLabel(inst)
            text.setStyleSheet("color : white")
            text.setFont(QFont("Arial", 14))
            score_layout.addWidget(text, i, 0)

        # Add score buttons
        btn_colors = ["7c04b8", "e0e0e0", "a553b5", "1e177a", "85d7e6"]

        for i in range(self.n_instruments):
            for j in range(self.beats_per_measure * self.n_measures):
                
                btn = QPushButton("")
                btn.setCheckable(True) # make button checkable
                btn.clicked.connect(self._note_callback)

                btn.setStyleSheet(f"background-color:#{btn_colors[i]}")

                # store button object
                self.measure_buttons[i].append(btn)
                
                score_layout.addWidget(btn, i, j+1)

        
        # Tempo selection layout
        tempo_layout = QHBoxLayout()
        tempo_layout.setSpacing(30)
        self.tempo_slider = QSlider(Qt.Horizontal)
        self.tempo_slider.setMinimum(4)
        self.tempo_slider.setMaximum(15)
        self.tempo_slider.setSingleStep(1)
        self.tempo_slider.setFocusPolicy(Qt.StrongFocus)
        self.tempo_slider.setTickPosition(QSlider.TicksBothSides)
        self.tempo_slider.setTickInterval(4)
        self.tempo_slider.valueChanged.connect(self._tempo_slider_callback)
        
        self.tempo_text = QLabel("4 BPM")
        self.tempo_text.setStyleSheet("color : white")
        self.tempo_text.setFont(QFont("Arial", 16))
        tempo_layout.addWidget(self.tempo_text)
        tempo_layout.addWidget(self.tempo_slider)
        
        # play stop clear layout
        play_stop_layout = QHBoxLayout()

        play_btn = QPushButton("PLAY")
        play_btn.setStyleSheet("background-color : #5cd699")
        play_btn.clicked.connect(self._play_callback)

        stop_btn = QPushButton("STOP")
        stop_btn.setStyleSheet("background-color : #de3165")
        stop_btn.clicked.connect(self._stop_callback)

        clear_btn = QPushButton("CLEAR")
        clear_btn.setStyleSheet("background-color : #38246e")
        clear_btn.clicked.connect(self._clear_callback)
  
        play_stop_layout.addWidget(play_btn)
        play_stop_layout.addWidget(stop_btn)
        play_stop_layout.addWidget(clear_btn)

        # add all child layouts to main
        title_text = QLabel("Welcome to Toro the Conun-Drummer's Livehouse!")
        title_text.setStyleSheet("color : white")
        title_text.setFont(QFont('Fixedsys', 28))
        directions_text = QLabel("Input a drum score for Toro to play, set the tempo, and press PLAY to see toro drum!\nThe entire grid represents a measure, and each button is an eigth note.")
        directions_text.setStyleSheet("color : white")
        directions_text.setFont(QFont("Arial", 16))
        
        self.error_text = QLabel("")
        self.error_text.setFont(QFont("Arial", 16))
        self.error_text.setStyleSheet("color:rgb(255,0,0)")

        main_layout.addWidget(title_text)
        main_layout.addWidget(directions_text)
        main_layout.addWidget(self.error_text)
        main_layout.addLayout(score_layout)
        main_layout.addLayout(tempo_layout)
        main_layout.addLayout(play_stop_layout)

        self.setLayout(main_layout)
        self.show()


    ################# Button and Slider Callback Functions ##################
    def _play_callback(self):

        """
        Button callback for PLAY button
        """

        print("Clicked PLAY")

        # make sure input is valid before exporting txt file
        if not self._is_valid_input():
            print("Input score is invalid.")
            self.error_text.setText("Toro only has 2 arms! Please try another beat!")
            return

        # if input is valid and the simulation is not already playing, save coordinate files and start simulation
        if not self.is_playing:
            self.error_text.setText("Watch Toro play your beat!!")
            self.save_score_text()
            self.is_playing = True
            self.redis_client.set(self.IS_PLAYING_KEY, 1)

    def _stop_callback(self):

        """
        Button callback for STOP button
        """

        print("Clicked STOP")
        if self.is_playing:
            self.error_text.setText("")
            self.is_playing = False
            self.redis_client.set(self.IS_PLAYING_KEY, 0)

    def _clear_callback(self):
        """
        Button callback for CLEAR button
        """
        print("CLEAR")

        for row in self.measure_buttons:
            for btn in row:
                btn.setChecked(False)

        self._update_btn_activation()

    def _note_callback(self):
        """
        Button callback for buttons in measure grid
        """

        self._update_btn_activation()

    def _tempo_slider_callback(self, value):
        """
        Called when slider value changes
        """
        self.tempo_text.setText(str(value) + " BPM")
        self.tempo = value
        

    ###################### Check Button Activation States ########################
    def _update_btn_activation(self):
        """
        Get binary array of button activation state
        """

        result = [[1 if (btn.isChecked()) else 0 for btn in sublist] for sublist in self.measure_buttons]
        self.button_activation = np.array(result)

    def _is_valid_input(self):
        """
        Check if input score is valid.
        Valid input has at most 4 instruments playing at each beat, and no more than 2 notes per beat can be played by arms.
        """

        n_active = np.sum(self.button_activation[:-2], axis=0)
        return np.all(n_active <= 2)


    ####################### Saving Score Array ################################
    def save_score_text(self):
        """
        Computes score array for each limb and saves them as txt files to be read by controller
        """
        # print(self.button_activation)

        # prepare time array from tempo and total number of beats
        dt = 60 / (self.tempo * self.beats_per_measure / self.time_sig) # time between each beat
        loop_time = dt * self.n_measures * self.beats_per_measure # seconds per loop
        
        self.redis_client.set(self.BPM_KEY, self.tempo) # upload BPM to redis 
        self.redis_client.set(self.LOOPTIME_KEY, loop_time)

        time = np.arange(0, loop_time, dt) # time array

        # left foot (hi-hat pedal)
        left_foot = time[self.button_activation[-1] == 1]
        left_foot = np.hstack( (left_foot[:, np.newaxis], np.zeros((left_foot.shape[0], 3))) )

        # right foot (bass)
        right_foot = time[self.button_activation[-2] == 1]
        right_foot = np.hstack( (right_foot[:, np.newaxis], np.zeros((right_foot.shape[0], 3))) )

        # arms:
        # if 1 hand instrument is selected: assign to hand that is closer to the instrument
        # if 2 hand instruments are selected: assign one that is further to the right to right hand, and the other to the left hand

        right_hand = []
        left_hand = []

        for i in range(self.n_measures * self.beats_per_measure):
            n_active = np.sum(self.button_activation[:-2, i])
            # if 1 arm instruments are selected for this beat
            if n_active == 1:                
                inst_idx = np.where(self.button_activation[:-2,i]==1)[0][0] # row number (on grid) of selected instrument
                inst_name = self.instrument_names[inst_idx] # name of instrument
                
                if self.inst_r2l[inst_name] < self.rh_priority_idx: # if right hand has priority
                    right_hand.append( np.append(time[i], self.inst_to_coords[inst_name]) )
                else:
                    left_hand.append( np.append(time[i], self.inst_to_coords[inst_name]) )

            # if 2 arm instruments are selected for this beat
            elif n_active == 2:
                inst_idx = np.where(self.button_activation[:-2,i] == 1)[0]
                inst_name = self.instrument_names[inst_idx]
                if self.inst_r2l[inst_name[0]] < self.inst_r2l[inst_name[1]]:
                    # first instrument is further to the right
                    right_hand.append( np.append(time[i], self.inst_to_coords[inst_name[0]]) )
                    left_hand.append( np.append(time[i], self.inst_to_coords[inst_name[1]]) )
                else:
                    # second instrument is further to the right
                    right_hand.append( np.append(time[i], self.inst_to_coords[inst_name[1]]) )
                    left_hand.append( np.append(time[i], self.inst_to_coords[inst_name[0]]) )

        right_hand = np.array(right_hand).flatten()
        left_hand = np.array(left_hand).flatten()

        if left_foot.size == 0:
            self.redis_client.set(self.LF_PLAYS_KEY, "0")
        if right_foot.size == 0:
            self.redis_client.set(self.RF_PLAYS_KEY, "0")
        if left_hand.size == 0:
            self.redis_client.set(self.LH_PLAYS_KEY, "0")
        if right_hand.size == 0:
            self.redis_client.set(self.RH_PLAYS_KEY, "0")

        os.chdir("../bin/Conundrum")
        np.savetxt("left_foot.txt", left_foot.flatten(), newline="\n", fmt='%1.5f')
        np.savetxt("right_foot.txt", right_foot.flatten(), newline="\n", fmt='%1.5f')
        np.savetxt("right_hand.txt", right_hand, newline="\n", fmt='%1.5f')
        np.savetxt("left_hand.txt", left_hand, newline="\n", fmt='%1.5f')

        print("Toro is ready to CONUN-DRUM!!!")
        os.chdir("../../Conundrum")            
      

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())
