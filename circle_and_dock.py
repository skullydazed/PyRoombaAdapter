#!/usr/bin/env python3
"""
Pull forward a few cm, spin around, dock.
"""
from math import radians
from time import sleep

from pyroombaadapter import PyRoombaAdapter

PORT = "/dev/ttyUSB0"
adapter = PyRoombaAdapter(PORT)
adapter.change_mode_to_safe()

# Warn people we're coming
adapter.send_song_cmd(0, 9,
                      [69, 69, 69, 65, 72, 69, 65, 72, 69],
                      [40, 40, 40, 30, 10, 40, 30, 10, 80])
adapter.send_play_cmd(0)
sleep(5.0)

# Move forward
adapter.move(-0.2, 32768)
sleep(4)
adapter.move(0, 0)

# Spin left and right
adapter.move(0, -1)
sleep(2)
adapter.move(0, 1)
sleep(2)
adapter.move(0, 0)

# Dock
adapter.start_seek_dock()
