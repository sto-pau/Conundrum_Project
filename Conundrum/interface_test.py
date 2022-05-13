from gc import get_referents
from wsgiref import simple_server
import redis
import numpy as np
import struct


# redis keys
IS_PLAYING_KEY = "gui::is_running"

# start redis
r = redis.Redis()


# save dummy array to txt file
dummy_array = np.array([
    [0, 1, 2, 3],
    [1, 2, 3, 4],
    [2, 3, 4, 5]
    ])

np.savetxt("dummy.txt", dummy_array, delimiter=',')

