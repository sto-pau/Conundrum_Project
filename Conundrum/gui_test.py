#1/usr/bin/python

import sys
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import redis

# redis keys
IS_PLAYING_KEY = "gui::play" # whether toro should be playing

# flags
is_playing = 0 # 0 = not playing, 1 = playing


def main():
    app = QApplication(sys.argv)

    # define window
    w = QWidget()
    w.resize(1024, 512)
    w.move(1000, 0)
    w.setWindowTitle("simple")


    # define buttons
    b_play = QPushButton(w)
    b_play.setText("Play")
    b_play.clicked.connect(play_stop_callback)

    b_stop = QPushButton(w)
    b_stop.setText("Stop")
    b_stop.clicked.connect()
    # b1.setGeometry(QtCore.QRect(50, 200, 75, 23))

    # define layout
    layout = QHBoxLayout()
    layout.addWidget(b_play)
    w.setLayout(layout)

    w.show()

    sys.exit(app.exec_())

def play_stop_callback():

    print("clicked PLAY")

if __name__ == '__main__':
    main()