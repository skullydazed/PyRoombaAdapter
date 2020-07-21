#!/usr/bin/env python3
"""Bridal Chorus, composed by Richard Wagner.

Adapted for Roomba by Zach White.
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter


notes = (
#   note, duration
    'G4', '1/4',
    'C5', 'd1/8',
    'C5', '1/16',
    'C5', '1/2',
    'G4', '1/4',
    'D5', 'd1/8',
    'B4', '1/16',
    'C5', '1/2',
    'G4', '1/4',
    'C5', 'd1/8',
    'E5', '1/16',
    'F5', '1/4',
    'E5', 'd1/8',
    'D5', '1/16',
    'C5', '1/4',
    'B4', 'd1/8',
    'C5', '1/16',
    'D5', '1/2'
)

PORT = "/dev/ttyUSB0"
roomba = PyRoombaAdapter(PORT)
roomba.change_mode_to_full()

print('Sending song')
roomba.send_song_cmd(0, notes, 5)

for i in range(2):
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
