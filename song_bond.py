#!/usr/bin/env python3
"""James Bond Theme, Composed by Monty Norman

Adapted for Roomba by Zach White
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter


notes0 = (
#   note, duration
    'B3', '1/2',
    'C4', '1/2',
    'C#4', '1/2',
    'C4', '1/2',
    'B3', '1/2',
    'C4', '1/2',
    'C#4', '1/2',
    'C4', '1/2',
)
notes1 = ( # 3 times
    'E4', '1/8',
    'F#4', '1/16',
    'F#4', '1/16',
    'F#4', '1/8',
    'F#4', '1/4',
    'E4', '1/8',
    'E4', '1/8',
    'E4', '1/8',
    'E4', '1/8',
    'G4', '1/16',
    'G4', '1/16',
    'G4', '1/8',
    'G4', '1/4',
    'F#4', '1/8',
    'F#4', '1/8',
    'F#4', '1/8',
)
notes2 = (
    'D#5', '1/8',
    'D5', '1/2',
    'B4', '1/8',
    'A4', '1/8',
    'B4', '1'
)
notes3 = (
    'E4', '1/8',
    'G4', '1/4',
    'D#5', '1/8',
    'D5', 'd1/4',
    'G4', '1/8',
    'A#4', '1/8',
    'B4', 'd1/2',
    'G4', '1/4',
    'A4', '1/16',
    'G4', '1/16',
    'F#4', 'd1/4',
    'B3', '1/8',
    'E4', '1/8',
    'C#4', '1'
)

PORT = "/dev/ttyUSB0"
roomba = PyRoombaAdapter(PORT)
roomba.change_mode_to_full()

print('Sending song')
roomba.send_song_cmd(0, notes0, 5)
roomba.send_song_cmd(1, notes1, 5)
roomba.send_song_cmd(2, notes2, 5)
roomba.send_song_cmd(3, notes3, 5)

for song_num in 0, 1, 1, 1, 2, 1, 1, 1, 2, 0, 3, 3, 0, 2:
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
