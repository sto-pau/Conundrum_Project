from email.charset import QP
from json.tool import main
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
        self.n_measures = 1
        self.beats_per_measure = 16
        self.n_instruments = 4
        self.is_playing = False

        # define and initialize redis keys
        self.IS_PLAYING_KEY = "gui::is_playing"
        self.redis_client.set(self.IS_PLAYING_KEY, 0)

        self.instruments = ["Bass", "Snare", "Hi-hat", "Tom"]

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

        for i, inst in enumerate(self.instruments):
            score_layout.addWidget(QLabel(inst), i, 0)

        for i in range(self.n_instruments):
            for j in range(self.beats_per_measure * self.n_measures):
                btn = QPushButton("")
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


    ################# Saving Score Array ########################
    def save_score_text(self):
        """
        Computes score array for each limb and saves them as txt files to be read by controller
        """
        # TODO - Currently returns dummy array for testing purposes
        score_array = np.array([
            [0, 1, 2, 3],
            [1, 2, 3, 4],
            [2, 3, 4, 5]
            ])

        np.savetxt("dummy_score.txt", score_array, delimiter=',')
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())