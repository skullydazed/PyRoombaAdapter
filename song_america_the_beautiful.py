#!/usr/bin/env python3
"""America The Beatiful, composed by Samuel Ward.

Adapted for Roomba by Zach White.
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter


notes0 = (
#   note, duration
    'G4', '1/4',
    'G4', 'd1/4',
    'E4', '1/8',
    'E4', '1/4',
    'G4', '1/4',
    'G4', 'd1/4',
    'D4', '1/8',
    'D4', '1/4',
    'E4', '1/4',
    'F4', '1/4',
    'G4', '1/4',
    'A4', '1/4',
    'B4', '1/4',
    'G4', 'd1/2',
)
notes1 = (
#   note, duration
    'G4', '1/4',
    'G4', 'd1/4',
    'E4', '1/8',
    'E4', '1/4',
    'G4', '1/4',
    'G4', 'd1/4',
    'D4', '1/8',
    'D4', '1/4',
    'D5', '1/4',
    'C#5', '1/4',
    'D5', '1/4',
    'E5', '1/4',
    'A4', '1/4',
    'D5', 'd1/2',
)
notes2 = (
    'G4', '1/4',
    'E5', 'd1/4',
    'E5', '1/8',
    'D5', '1/4',
    'C5', '1/4',
    'C5', 'd1/4',
    'B4', '1/8',
    'B4', '1/4',
    'C5', '1/4',
    'D5', '1/4',
    'B4', '1/4',
    'A4', '1/4',
    'G4', '1/4',
    'C5', 'd1/2',
)
notes3 = (
    'C5', '1/4',
    'C5', 'd1/4',
    'A4', '1/8',
    'A4', '1/4',
    'C5', '1/4',
    'C5', 'd1/4',
    'G4', '1/8',
    'G4', '1/4',
    'G4', '1/4',
    'A4', '1/4',
    'C5', '1/4',
    'G4', '1/4',
    'D5', '1/4',
    'C5', 'd1/2',
)

PORT = "/dev/ttyUSB0"
roomba = PyRoombaAdapter(PORT)
roomba.change_mode_to_full()

print('Sending song')
roomba.send_song_cmd(0, notes0, 5)
roomba.send_song_cmd(1, notes1, 5)
roomba.send_song_cmd(2, notes2, 5)
roomba.send_song_cmd(3, notes3, 5)

for song_num in range(4):
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
