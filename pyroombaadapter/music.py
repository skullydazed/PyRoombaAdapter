#!/usr/bin/env python3
"""Music helpers.
"""
from time import sleep

base_beats = beats = {
    '1/32': 1,
    '1/16': 2,
    'd1/16': 3,
    '1/8': 4,
    'd1/8': 6,
    '1/4': 8,
    'd1/4': 12,
    '1/2': 16,
    'd1/2': 24,
    '1': 32
}

NOTES_FLAT = ['C', 'Db', 'D', 'Eb', 'E', 'F', 'Gb', 'G', 'Ab', 'A', 'Bb', 'B']
NOTES_SHARP = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']


def scale_bpm(scale):
    """Sets the scale factor for bpm.

    The roomba lets you set a note length from 1-255, in 15.625 ms increments. When a note length is 10 it will play for 156.25 ms.

    This gives us a minimum bpm of 64 and a maximum bpm of 960. This cleanly divides in most cases. Where it doesn't hopefully the delta is too small to hear.

    `scale` can be 1-14, with higher numbers resulting in a slower tempo. The math says 1 should be 960 bpm and 14 should be around 68 bpm, but it seems to be somewhere around 30 or 40 bpm.
    """
    global beats
    beats = {note: int(length * scale) for note, length in base_beats.items()}


def _note_to_midi(note):
    """Returns the MIDI note number for a given note.
    """
    key = note[:-1]  # eg C, Db
    octave = note[-1]   # eg 3, 4
    pos = NOTES_FLAT.index(key) if 'b' in key else NOTES_SHARP.index(key)
    note = pos + 12 * (int(octave) + 1)

    if note < 31 or note > 127:
        raise ValueError(f'Can only use notes from G1 to G9. {note} is out of range!')

    return note

def notes_to_song(notes):
    """Convert an array of note/length pairs from the human readable form.

    See https://jythonmusic.files.wordpress.com/2016/01/pitchesonstaff.png for a nice map of octaves to notes.
    """
    result = []

    for i, note in enumerate(notes):
        if i % 2 == 0:
            # Music Note
            if note.lower() == 'rest':
                result.append(30)
            else:
                result.append(_note_to_midi(note))
        else:
            result.append(beats[note])

    return result
