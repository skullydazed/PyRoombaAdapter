#!/usr/bin/env python3
"""
    Play Darth Vader song
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter

vader_notes = (
# note, duration
    69, 40,
    69, 40,
    69, 40,
    65, 30,
    72, 10,
    69, 40,
    65, 30,
    72, 10,
    69, 80
)

PORT = "/dev/ttyUSB0"
adapter = PyRoombaAdapter(PORT)
adapter.change_mode_to_full()

print('Sending song')
adapter.send_song_cmd(0, vader_notes)

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
