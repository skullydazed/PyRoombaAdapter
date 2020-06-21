#!/usr/bin/env python3
"""
    Play Darth Vader song
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter

PORT = "/dev/ttyUSB0"
adapter = PyRoombaAdapter(PORT)
adapter.change_mode_to_full()

print('Sending song')
adapter.send_song_cmd(0, 9,
                      [69, 69, 69, 65, 72, 69, 65, 72, 69],
                      [40, 40, 40, 30, 10, 40, 30, 10, 80])

print('Playing song')
adapter.send_play_cmd(0)

# Wait for it to start playing
while adapter.readings['song_playing'] != 1:
    sleep(0.15)
print('Song Started')

# Wait for it to stop playing
while adapter.readings['song_playing'] != 0:
    sleep(0.15)
print('Song Ended')

adapter.turn_off_power()
