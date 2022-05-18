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
        self.is_valid = True # whether current score is valid
        self.tempo = 4 # tempo in BPM - TODO: make this tunable

        # define and initialize redis keys
        self.IS_PLAYING_KEY = "gui::is_playing"
        self.redis_client.set(self.IS_PLAYING_KEY, 0)

        # instruments and coordinates
        # from left to right: snare, tom1, tom2

        self.instrument_names = ["Tom 1", "Tom 2", "Snare", "Bass", "Hi-Hat Pedal"]
        self.n_instruments = len(self.instrument_names)

        self.inst_to_coords = { 
            "Bass": np.array([0, 0, 0]),
            "Snare": np.array([0.36543, 0.32512, -0.50876]),
            "Hi-Hat Pedal": np.array([0, 0, 0]),
            "Tom 1": np.array([0.72103, 0.18308, -0.35312]),
            "Tom 2": np.array([0.74235, -0.18308, -0.26023]),
            }

        self.coords = np.array( list(self.inst_to_coords.values()) )

        # list to store all buttons in measure grid
        self.measure_buttons = [ [] for i in range(self.n_instruments) ]

        # array containing button activation status
        self.button_activation = np.array((self.n_instruments, self.n_measures * self.beats_per_measure))

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

        self.save_score_text()

        if not self._is_valid_input():
            print("Input score is invalid. You cannot select more than 4 instruments on the same beat")
            return

        if not self.is_playing:
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

        print("Clicked button in measure grid")
        self._check_btn_activation()
        

    ###################### Check Button Activation States ########################
    def _check_btn_activation(self):
        """
        Get binary array of button activation state
        """

        result = [[1 if (btn.isChecked()) else 0 for btn in sublist] for sublist in self.measure_buttons]
        self.button_activation = np.array(result)

    def _is_valid_input(self):
        """
        Check if input score is valid
        """

        # number of activated buttons per beat
        n_active = np.sum(self.button_activation, axis=0)
        return np.all(n_active <= 4)

    ####################### Saving Score Array ################################
    def save_score_text(self):
        """
        Computes score array for each limb and saves them as txt files to be read by controller
        """
        # TODO - Currently returns dummy array for testing purposes

        score_array = np.zeros_like(self.n_instruments, self.n_measures * self.beats_per_measure, )
        score_array
        
        print("activation\n", self.button_activation)

        print("coords\n", self.coords)

        np.savetxt("dummy_score.txt", score_array, delimiter=',')

        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())