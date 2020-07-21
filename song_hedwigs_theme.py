#!/usr/bin/env python3
"""Hedwig's Theme, by John Williams.

Adapted for Roomba by Zach White.
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter


notes0 = (
#   note, duration
    'E4', '1/4',
    'A4', 'd1/4',
    'C5', '1/8',
    'B4', '1/4',
    'A4', '1/2',
    'E5', '1/4',
    'D5', 'd1/2',
    'B4', 'd1/2',
    'A4', 'd1/4',
    'D5', '1/8',
    'C5', '1/4',
    'D#4', '1/2',
    'Bb4', '1/4',
    'E4', 'd1'
)
notes1 = (
)
notes2 = (
)
notes3 = (
)

PORT = "/dev/ttyUSB0"
roomba = PyRoombaAdapter(PORT)
roomba.change_mode_to_full()

print('Sending song')
roomba.send_song_cmd(0, notes0, 2)
#roomba.send_song_cmd(1, notes1, 5)
#roomba.send_song_cmd(2, notes2, 5)
#roomba.send_song_cmd(3, notes3, 5)

for song_num in range(1):
    print('Playing segment', song_num)
    roomba.send_play_cmd(song_num)

    # Wait for it to start playing
    while roomba.readings['song_playing'] != 1:
        sleep(0.05)
    print('Segment Started')

    # Wait for it to stop playing
    while roomba.readings['song_playing'] != 0:
        sleep(0.005)
    print('Segment Ended')

print('Song Ended')
roomba.turn_off_power()
