#!/usr/bin/env python3
"""The Star Spangled Banner, Composed by Francis Scott Key

Adapted for Roomba by Zach White
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter


notes0 = (
#   note, duration
    'G4', 'd1/8',
    'E4', '1/16',
    'C4', '1/4',
    'E4', '1/4',
    'G4', '1/4',
    'C5', '1/2',
    'E5', 'd1/8',
    'D5', '1/16',
    'C5', '1/4',
    'E4', '1/4',
    'F#4', '1/4',
    'G4', '1/2'
)
notes1 = (
    'G4', '1/8',
    'G4', '1/8',
    'E5', 'd1/4',
    'D5', '1/8',
    'C5', '1/4',
    'B4', '1/2',
    'A4', '1/8',
    'B4', '1/8',
    'C5', '1/4',
    'C5', '1/4',
    'G4', '1/4',
    'E4', '1/4',
    'C4', '1/4'
)

PORT = "/dev/ttyUSB0"
roomba = PyRoombaAdapter(PORT)
roomba.change_mode_to_full()

print('Sending song')
roomba.send_song_cmd(0, notes0, 5)
roomba.send_song_cmd(1, notes1, 5)

print('Playing song')
for song_num in 0, 1, 0, 1:
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
