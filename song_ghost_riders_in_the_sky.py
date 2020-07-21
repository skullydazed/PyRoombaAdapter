#!/usr/bin/env python3
"""America The Beatiful, composed by Samuel Ward.

Adapted for Roomba by Zach White.
"""
from time import sleep

from pyroombaadapter import PyRoombaAdapter


notes0 = (
#   note, duration
    'A3', '1/4', # An
    'E4', '1/4', # Old
    'E4', '1/4', # Cow
    'E4', '1/4', # Poke
    'F4', '1/4', # Went
    'G4', '1/4', # Rid-
    'G4', '1/4', # ing
    'G4', '1/4', # Out
    'E4', '1/4', # One
    'C4', '1/4', # Dark
    'C4', '1/4', # And
    'C4', '1/4', # Wind-
    'A3', '1/4', # y
    'C4', 'd1/2', # Day
)
notes1 = (
    'A3', '1/2', # yi-
    'A3', '1/2', # pi-
    'A3', '1/2', # ya-
    'C4', 'd1', # ay
    'rest', '1/2',
    'D4', '1/2',
    'D4', '1/2',
    'D4', '1/2',
    'A3', 'd1',
    'rest', '1/2',
    'A3', '1/2',
    'Bb3', '1',
    'Bb3', '1',
    'F4', 'd1',
    'F4', '1/2',
    'D4', 'd1',
)
notes2 = (
    'A4', '1/4', # When
    'A4', '1/4', # All
    'D5', '1/4', # At
    'D5', '1/4', # Once
    'D5', '1/4', # A
    'D5', '1/4', # Might-
    'B4', '1/4', # Y
    'B4', '1/4', # Herd
    'B4', '1/4', # Of
    'A4', '1/4', # Red
    'A4', '1/4', # Eyed
    'A4', '1/4', # Cows
    'F4', '1/4', # He
    'D4', '1/2', # Saw
)
notes3 = (
    'D4', '1/4', # A
    'Bb3', '1/4', # plough-
    'Bb3', '1/4', # in'
    'Bb3', '1/4', # thru
    'Bb3', '1/4', # the
    'F4', '1/4', # rag-
    'F4', '1/4', # ged
    'F4', '1', # skies,
    'rest', '1/4',
    'F4', '1/4', # and
    'F4', '1/4', # up
    'D4', '1/4', # a
    'D4', '1/4', # cloud-
    'D4', '1/4', # y
    'D4', 'd1', # draw
)

PORT = "/dev/ttyUSB0"
roomba = PyRoombaAdapter(PORT)
roomba.change_mode_to_full()

print('Sending song')
roomba.send_song_cmd(0, notes0, 3)
roomba.send_song_cmd(1, notes1, 2)
roomba.send_song_cmd(2, notes2, 3)
roomba.send_song_cmd(3, notes3, 3)

for song_num in 0, 0, 2, 3, 1:
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
