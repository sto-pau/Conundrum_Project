import sys
from PyQt5.QtWidgets import *

import redis
import numpy as np

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.redis_client = redis.Redis()

        # set window size and position
        self.setWindowTitle("Conun-drum App")
        self.setGeometry(1000, 0, 1024, 512)

        # set constants and flags
        self.n_measures = 1 # number of measures
        self.beats_per_measure = 4 # beats per measure
        self.is_playing = False # whether the simulation should be running
        self.tempo = 4 # tempo in BPM - TODO: make this tunable

        # define and initialize redis keys
        self.IS_PLAYING_KEY = "gui::is_playing"
        self.redis_client.set(self.IS_PLAYING_KEY, 0)

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

        self._initGUI()

    def _initGUI(self):
        """
        Initialize GUI layout and components
        """
        # Root parent layout
        main_layout = QVBoxLayout()

        # score grid layout
        score_layout = QGridLayout()
        score_layout.setVerticalSpacing(0)
        score_layout.setHorizontalSpacing(0)

        for i, inst in enumerate(self.instrument_names):
            score_layout.addWidget(QLabel(inst), i, 0)

        for i in range(self.n_instruments):
            for j in range(self.beats_per_measure * self.n_measures):
                
                btn = QPushButton("")
                btn.setCheckable(True) # make button checkable
                btn.clicked.connect(self._note_callback)

                if i >= self.n_instruments - 2:
                    # make buttons for feet instruments red for clarity
                    btn.setStyleSheet("background-color: red")

                # store button object
                self.measure_buttons[i].append(btn)
                
                score_layout.addWidget(btn, i, j+1)

        # play and stop layout
        play_stop_layout = QHBoxLayout()

        play_btn = QPushButton("PLAY")
        play_btn.clicked.connect(self._play_callback)
        stop_btn = QPushButton("STOP")
        stop_btn.clicked.connect(self._stop_callback)
        play_stop_layout.addWidget(play_btn)
        play_stop_layout.addWidget(stop_btn)

        # add all child layouts to main
        main_layout.addLayout(score_layout)
        main_layout.addLayout(play_stop_layout)

        self.setLayout(main_layout)   
        self.show()


    ################# Button Callback Functions ##################
    def _play_callback(self):

        """
        Button callback for PLAY button
        """

        print("Clicked PLAY")

        # make sure input is valid before exporting txt file
        if not self._is_valid_input():
            print("Input score is invalid.")
            return

        # if input is valid and the simulation is not already playing, save coordinate files and start simulation
        if not self.is_playing:
            self.save_score_text()
            self.is_playing = True
            self.redis_client.set(self.IS_PLAYING_KEY, 1)

    def _stop_callback(self):

        """
        Button callback for STOP button
        """

        print("Clicked STOP")
        if self.is_playing:
            self.is_playing = False
            self.redis_client.set(self.IS_PLAYING_KEY, 0)

    def _note_callback(self):
        """
        Button callback for buttons in measure grid
        """

        self._update_btn_activation()
        

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
        dt = 60 / self.tempo # time between each beat
        time = np.arange(0, dt * self.n_measures * self.beats_per_measure, dt) # time array

        # left foot (hi-hat pedal)
        score_array = time[self.button_activation[-1] == 1]
        score_array = np.hstack( (score_array[:, np.newaxis], np.zeros((score_array.shape[0], 3))) )
        np.savetxt("left_foot", score_array.flatten(), delimiter=',')

        # right foot (bass)
        score_array = time[self.button_activation[-2] == 1]
        score_array = np.hstack( (score_array[:, np.newaxis], np.zeros((score_array.shape[0], 3))) )
        np.savetxt("right_foot", score_array.flatten(), delimiter=',')

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
        np.savetxt("right_hand", right_hand, delimiter=',')
        np.savetxt("left_hand", left_hand, delimiter=',')                 
      

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())