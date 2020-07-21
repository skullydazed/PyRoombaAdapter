#!/usr/bin/env python3
"""Vader's Theme, composed by John Williams.

Adapted for Roomba by Zach White.
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter


vader_notes = (
#   note, duration
    'G4', '1/4',
    'G4', '1/4',
    'G4', '1/4',
    'Eb4', 'd1/8',
    'Bb4', '1/16',
    'G4', '1/4',
    'Eb4', 'd1/8',
    'Bb4', '1/16',
    'G4', '1/2'
)

PORT = "/dev/ttyUSB0"
roomba = PyRoombaAdapter(PORT)
roomba.change_mode_to_full()

print('Sending song')
roomba.send_song_cmd(0, vader_notes, 5)

print('Playing song')
roomba.send_play_cmd(0)

# Wait for it to start playing
while roomba.readings['song_playing'] != 1:
    sleep(0.15)
print('Song Started')

# Wait for it to stop playing
while roomba.readings['song_playing'] != 0:
    sleep(0.15)
print('Song Ended')

roomba.turn_off_power()
