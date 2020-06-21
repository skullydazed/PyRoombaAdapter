"""
A Python adapter module for Roomba Open Interface

This module is based on the document: iRobot® Roomba 500 Open Interface (OI) Specification
- Link https://www.irobot.lv/uploaded_files/File/iRobot_Roomba_500_Open_Interface_Spec.pdf

"""
import math
import struct
import sys
from time import sleep, time

import serial  # pyserial

from . import sensors


class PyRoombaAdapter:
    """
    Adapter class for Roomba Open Interface

    The constructor connects serial port and change the mode to safe mode

    :param string port: Serial port path

    :param int baudrate: bau rate of serial connection (default=115200)

    :param float timeout_sec: read time out of serial connection [sec] (default=None)

    :param float wheel_span_mm: wheel span of Roomba [mm]  (default=235.0)

    Examples:
        >>> PORT = "/dev/ttyUSB0"
        >>> adapter = PyRoombaAdapter(PORT)

    """
    # Commands that can be executea. The key is a plain-text command name. Values:
    #     :param int id: The number Roomba expects to receive to invoke this command.
    #     :param str format: The struct.pack() format for this command and any arguments you need to pass. Use None if the format has to be built dynamically.
    commands = {
        "Start": {'id': 128, 'format': 'B'},
        "Baud": {'id': 129, 'format': 'BB'},
        "Control": {'id': 130, 'format': 'B'},
        "Safe": {'id': 131, 'format': 'B'},
        "Full": {'id': 132, 'format': 'B'},
        "Power": {'id': 133, 'format': 'B'},
        "Spot": {'id': 134, 'format': 'B'},
        "Clean": {'id': 135, 'format': 'B'},
        "Max": {'id': 136, 'format': 'B'},
        "Drive": {'id': 137, 'format': '>Bhh'},
        "Motors": {'id': 138, 'format': 'BB'},
        "Song": {'id': 140, 'format': None},
        "Play": {'id': 141, 'format': 'BB'},
        "Sensors": {'id': 142, 'format': 'BB'},
        "Seek Dock": {'id': 143, 'format': 'B'},
        "PWM Motors": {'id': 144, 'format': 'BbbB'},
        "Drive Direct": {'id': 145, 'format': '>Bhh'},
        "Drive PWM": {'id': 146, 'format': '>Bhh'},
        "Stream": {'id': 148, 'format': None},
        "Query List": {'id': 149, 'format': None},
        "Toggle Stream": {'id': 150, 'format': 'BB'},
        "Buttons": {'id': 165, 'format': 'BB'},
    }

    STRAIGHT_RADIUS = 32768
    MIN_RADIUS = -2000
    MAX_RADIUS = 2000
    MIN_VELOCITY = -500
    MAX_VELOCITY = 500

    def __init__(self, port, baudrate=115200, timeout_sec=None, wheel_span_mm=235.0, sensor_list=sensors.DEFAULT_SENSORS):
        self.debug = False
        self._read_thread = None
        self.wheel_span = wheel_span_mm

        try:
            self._connect_serial(port, baudrate, timeout_sec)
        except SerialException: 
            print("Cannot find serial port. Plase reconnect it.")
            sys.exit(1)

        #self.stream_samples(sensors.TEMPERATURE)
        self.stream_samples(*sensor_list)
        self.read_background()
        self.change_mode_to_passive()
        self.change_mode_to_safe()
        sleep(1.0)

    def __del__(self):
        """
        Destructor of PyRoombaAdapter class

        The Destructor make Roomba move to passive mode and close serial connection
        """
        # disconnect sequence
        self.turn_off_power()
        sleep(0.1)
        self.serial_port.close()

    def debug_print(self, *args, **kwargs):
        """Print if self.debug=True.
        """
        if self.debug:
            print(*args, **kwargs)

    def send_cmd(self, command, *args, format=None):
        """Send a command to the robot.
        """
        format = format or self.commands[command]['format']

        if not format:
            raise ValueError(f"The command {command} requires a format to be passed.")

        self.debug_print('send_cmd', command, self.commands[command]['id'], args)
        self.send_struct(format, self.commands[command]['id'], *args)

    def send_struct(self, format, *args):
        """Send a struct packed command to the robot.
        """
        self.serial_port.write(struct.pack(format, *args))
        sleep(0.1)

    def stream_samples(self, *sensors):
        """Starts streaming sensor data from the Roomba at a rate of one reading every 15ms (the Roomba's internal update rate).
        """
        packet_list = [sensor[0] for sensor in sensors]
        count = len(packet_list)
        format = 'BB' + ('B' * count)
        self.send_cmd('Stream', count, *packet_list, format=format)

    def pause_stream(self):
        """Pauses the sample stream (if any) coming from the Roomba"""
        self.send_cmd('Toggle Stream', 0)
        sleep(0.1)
        self.serial_port.flushInput()


    def resume_stream(self):
        """Resumes the sample stream with the previously requested set of sensors.
        """
        self.send_cmd('Toggle Stream', 1)


    def poll(self):
        """Reads a single sample from the current sample stream.
        """
        # Wait for the first byte of a packet (19)
        magic = ord(self.serial_port.read())
        while magic != 19:
            magic = ord(self.serial_port.read())

        # Read the length of the packet, which is always the next byte.
        length = ord(self.serial_port.read()) + 1

        # Read in the packet.
        packet = self.serial_port.read(length)

        # Validate the checksum.
        if (sum(struct.unpack('B' * length, packet), length + 18) & 0xff) != 0:
            print('*** Bad checksum while attempting to read sample!')
            return

        # Extract readings from the packet
        packet = packet[0:-1]
        readings = {}

        while len(packet) != 0:
            sensor_id = packet[0]
            id, format, name = sensors.SENSOR_ID_MAP[sensor_id]
            size = struct.calcsize(format)
            value = struct.unpack(format, packet[1:1+size])
            readings[name] = value[0]
            packet = packet[1+size:]

        return readings

    def start_cleaning(self):
        """
        Start the default cleaning

        - Available in modes: Passive, Safe, or Full
        - Changes mode to: Passive

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.start_cleaning()
        """
        self.send_cmd("Clean")

    def start_max_cleaning(self):
        """
        Start the max cleaning

        - Available in modes: Passive, Safe, or Full
        - Changes mode to: Passive

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.start_max_cleaning()
        """
        self.send_cmd("Max")

    def start_spot_cleaning(self):
        """
        Start spot cleaning

        - Available in modes: Passive, Safe, or Full
        - Changes mode to: Passive

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.start_spot_cleaning()
        """
        self.send_cmd("Spot")

    def start_seek_dock(self):
        """
        Start seek dock

        - Available in modes: Passive, Safe, or Full
        - Changes mode to: Passive

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.start_seek_dock()
        """
        self.send_cmd("Seek Dock")

    def change_mode_to_passive(self):
        """
        Change mode to passive mode

        Roomba beeps once to acknowledge it is starting from “off” mode.

        - Available in modes: Passive, Safe, or Full
        """
        self.send_cmd("Start")
        self.send_cmd("Control")

    def change_mode_to_safe(self):
        """
        Change mode to safe mode

        Safe mode turns off all LEDs.
        If a safety condition occurs, Roomba reverts automatically to Passive mode.

        - Available in modes: Passive, Safe, or Full
        """
        # send command
        self.send_cmd("Safe")

    def change_mode_to_full(self):
        """
        Change mode to full mode

        Full mode turns off the cliff, wheel-drop and internal charger safety features.
        In Full mode, Roomba executes any command that you send it, even if the internal charger is plugged in,
        or command triggers a cliff or wheel drop condition.

        - Available in modes: Passive, Safe, or Full
        """
        # send command
        self.send_cmd("Full")

    def turn_off_power(self):
        """
        Turn off power of Roomba

        The mode change to passive mode.

        - Available in modes: Passive, Safe, or Full

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.turn_off_power()
        """
        # send command
        self.send_cmd("Power")

    def read_forever(self):
        """Read data from the robot in a loop.
        """
        print('Reading serial responses in the background...')

        while True:
            start = time()
            readings = self.poll()
            if readings:
                self.readings = readings
                #print(readings)
            stop = time()
            remaining = 0.015 - (start - stop)
            if remaining > 0:
                sleep(remaining)

    def read_background(self):
        """Spawns a background thread that reads data from the serial port.
        """
        if not self._read_thread:
            from threading import Thread
            self._read_thread = Thread(target=self.read_forever, name='serial_read', daemon=True)
            self._read_thread.start()

    def move(self, velocity, yaw_rate):
        """
        control roomba at the velocity and the rotational speed (yaw rate)

        Note:
            The Roomba keep a control command until receiving next command

        - Available in modes: Safe or Full
        - Changes mode to: No Change

        - Special cases:
            - Straight = 32768 or 32767 = hex 8000 or 7FFF
            - Turn in place clockwise = -1
            - Turn in place counter-clockwise = 1

        :param float velocity: velocity (m/sec)

        :param float yaw_rate: yaw rate (rad/sec)

        Examples:
            >>> import numpy as np
            >>> import time
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.move(0, np.rad2deg(-10)) # rotate to right side
            >>> time.sleep(1.0) # keep rotate
            >>> adapter.move(0.1, 0.0) # move straight with 10cm/sec
        """
        if velocity == 0:  # rotation
            vel_mm_sec = math.fabs(yaw_rate) * (self.wheel_span / 2.0)
            if yaw_rate >= 0:
                radius_mm = 1
            else:  # default is 'CCW' (turning left)
                radius_mm = -1
        elif yaw_rate == 0:
            vel_mm_sec = velocity * 1000.0  # m/s -> mm/s
            radius_mm = self.STRAIGHT_RADIUS
        else:
            vel_mm_sec = velocity * 1000.0  # m/s -> mm/s
            radius_mm = vel_mm_sec / yaw_rate

        self.send_drive_cmd(vel_mm_sec, radius_mm)

    def send_drive_cmd(self, roomba_mm_sec, roomba_radius_mm, turn_in_place='no', drive_straight=False):
        """Control the Roomba’s drive wheels.

        The radius is measured from the center of the turning circle to the center of Roomba.

        A Drive command with a positive velocity Roomba drive forward. A negative velocity makes Roomba drive backward.

        A positive radius makes Roomba turn while driving forward. A negative radius makes Roomba turn toward the right. These directions are reversed when driving backwards.

        :param float roomba_mm_sec: the average velocity of the drive wheels in millimeters per second (-500 – 500 mm/s)

        :param float roomba_radius_mm: the radius in millimeters at which Roomba will turn. (-2000 – 2000 mm)

        :param str turn_in_place: Whether the roomba should turn in place. This will override the roomba_radius_mm parameter. ('no', 'clockwise', 'counter-clockwise') 

        :param bool drive_straight: Force the Roomba to drive straight. This will override the roomba_radius_mm parameter.

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.send_drive_cmd(-100, -1000) # back to the right side
            >>> sleep(2.0) # keep 2 sec
        """
        # Sanity checks
        if drive_straight and turn_in_place != 'no':
            raise ValueError('drive_straight and turn_in_place are mutually exclusive!')

        # Prepare data
        if turn_in_place == 'clockwise':
            roomba_radius_mm = -1
        elif turn_in_place == 'counter-clockwise':
            roomba_radius_mm = 1
        elif turn_in_place == 'no':
            roomba_radius_mm = self._min_max(roomba_radius_mm, self.MIN_RADIUS, self.MAX_RADIUS)
        else:
            raise ValueError('turn_in_place must be one of "clockwise", "counter-clockwise", or "no"')

        roomba_mm_sec = self._min_max(roomba_mm_sec, self.MIN_VELOCITY, self.MAX_VELOCITY)

        # Instruct the roomba to start driving
        self.send_cmd("Drive", roomba_mm_sec, roomba_radius_mm)

    def send_drive_direct(self, right_mm_sec, left_mm_sec):
        """
        send drive direct command

        This command lets you control the forward and backward motion of Roomba’s drive wheels independently.
        A positive velocity makes that wheel drive forward, while a negative velocity makes it drive backward.

        - Available in modes: Safe or Full

        :param float right_mm_sec: the velocity of the right drive wheels in millimeters per second (-500 – 500 mm/s)

        :param float left_mm_sec: the velocity of the left drive wheels in millimeters per second (-500 – 500 mm/s)

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.send_drive_direct(10, 100) # move right forward
            >>> sleep(2.0) # keep 2 sec
        """
        right_mm_sec = self._min_max(right_mm_sec, self.MIN_VELOCITY, self.MAX_VELOCITY)
        left_mm_sec = self._min_max(left_mm_sec, self.MIN_VELOCITY, self.MAX_VELOCITY)
        self.send_cmd("Drive Direct", right_mm_sec, left_mm_sec)

    def send_drive_pwm(self, right_pwm, left_pwm):
        """
        send drive pwm command

        This command lets you control the forward and backward motion of Roomba’s drive wheels independently.
        It takes four data bytes, which are interpreted as two 16-bit signed values using two’s complement.
        A positive PWM makes that wheel drive forward, while a negative PWM makes it drive backward.

        - Available in modes: Safe or Full

        :param int right_pwm: Right wheel PWM (-255 – 255)

        :param int left_pwm: Left wheel PWM (-255 - 255)

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.send_drive_pwm(-200, -200) # move backward
            >>> sleep(2.0) # keep 2 sec
        """
        right_pwm = self._min_max(right_pwm, -255, 255)
        left_pwm = self._min_max(left_pwm, -255, 255)

        self.send_cmd("Drive PWM", right_pwm, left_pwm)

    def send_motors_cmd(self, main_brush_on, main_brush_direction_is_ccw, side_brush_on, side_brush_direction_is_inward, vacuum_on):
        """
        send motors command

        This command controls the motion of Roomba’s main brush, side brush, and vacuum independently.
        Motor velocity cannot be controlled with this command, all motors will run at maximum speed when enabled.
        The main brush and side brush can be run in either direction. The vacuum only runs forward.

        :param bool main_brush_on: main brush on or off

        :param bool main_brush_direction_is_ccw: main brush direction, clockwise or counter-clockwise(default)

        :param bool side_brush_on: side brush on or off

        :param bool side_brush_direction_is_inward: side brush direction, inward(default) or outward

        :param bool side_brush_on: side brush on or off

        :param bool vacuum_on: vacuum on or off

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.send_motors_cmd(False, True, True, True, False) # side brush is on, and it rotates inward
            >>> sleep(2.0) # keep 2 sec
        """
        cmd = 0  # All initial bit is 0
        if side_brush_on:
            cmd |= 0b00000001
        if vacuum_on:
            cmd |= 0b00000010
        if main_brush_on:
            cmd |= 0b00000100
        if not side_brush_direction_is_inward:
            cmd |= 0b00001000
        if not main_brush_direction_is_ccw:
            cmd |= 0b00010000

        self.send_cmd("Motors", cmd)

    def send_pwm_motors(self, main_brush_pwm, side_brush_pwm, vacuum_pwm):
        """
        send pwm motors

        This command control the speed of Roomba’s main brush, side brush, and vacuum independently.
        With each data byte, you specify the duty cycle for the low side driver (max 128).
        For example, if you want to control a motor with 25% of battery voltage, choose a duty cycle of 128 * 25% = 32.
        The main brush and side brush can be run in either direction.
        The vacuum only runs forward. Positive speeds turn the motor in its default (cleaning) direction.
        Default direction for the side brush is counterclockwise.
        Default direction for the main brush/flapper is inward.

        - Available in modes: Safe or Full

        :param int main_brush_pwm: main brush PWM (-127 - 127)

        :param int side_brush_pwm: side brush PWM (-127 - 127)

        :param int vacuum_pwm: vacuum duty cycle (0 - 127)

        Examples:
            >>> adapter = PyRoombaAdapter("/dev/ttyUSB0")
            >>> adapter.send_pwm_motors(-55, 0, 0) # main brush is 55% PWM to opposite direction
            >>> sleep(2.0) # keep 2 sec
        """
        main_brush_pwm = self._min_max(main_brush_pwm, -127, 127)
        side_brush_pwm = self._min_max(side_brush_pwm, -127, 127)
        vacuum_pwm = self._min_max(vacuum_pwm, 0, 127)
        self.send_cmd("PWM Motors", main_brush_pwm, side_brush_pwm, vacuum_pwm)

    def send_buttons_cmd(self, clean=False, spot=False, dock=False, minute=False, hour=False, day=False, schedule=False, clock=False):
        """Trigger button presses.

        This command lets you push Roomba’s buttons. The buttons will automatically release after 1/6th of a second.

        Note:
            - This API doesn't work on Roomba 690 Model

        :param bool clean: clean button on
        :param bool spot: spot button on
        :param bool dock: dock button on
        :param bool minute: minute button on
        :param bool hour: hour button on
        :param bool day: day button on
        :param bool schedule: schedule button on
        :param bool clock: clock button on
        """
        buttons = 0

        if clean:
            buttons += 0b00000001
        if spot:
            buttons += 0b00000010
        if dock:
            buttons += 0b00000100
        if minute:
            buttons += 0b00001000
        if hour:
            buttons += 0b00010000
        if day:
            buttons += 0b00100000
        if schedule:
            buttons += 0b01000000
        if clock:
            buttons += 0b10000000

        self.send_cmd("Buttons", buttons)

    def send_song_cmd(self, song_number, note_list):
        """
        Send song command

        This command lets you specify up to four songs to the OI that you can play at a later time. Each song is associated with a song number. The Play command uses the song number to identify your song selection.

        Each song can contain up to sixteen notes. Each note is associated with a note number that uses MIDI note definitions and a duration that is specified in fractions of a second.

        - Available in modes: Passive, Safe, or Full

        :param int song_number: (0-4) The song number associated with the specific song.
                        If you send a second Song command, using the same song number, the old song is overwritten.

        :param list note_list: A collapsed list of note/duration bytes to play. Every even value defines a pitch to play. Every odd value defines how long to play the previous pitch.
                               Note Number (31 – 127) The pitch of the musical note Roomba will play, according to the MIDI note numbering scheme.
                               Roomba considers all musical notes outside the range of 31 – 127 as rest notes, and will make no sound during the duration of those notes.
                               Note Duration (0 – 255) The duration of a musical note, in increments of 1/64th of a second. Example: a half-second long musical note has a duration value of 32.

        Examples:
            >>> adapter = PyRoombaAdapter("/dev/ttyUSB0")
            >>> # note names
            >>> f4 = 65
            >>> a4 = 69
            >>> c5 = 72
            >>> # note lengths
            >>> MEASURE = 160
            >>> HALF = int(MEASURE / 2)
            >>> Q = int(MEASURE / 4)
            >>> Ed = int(MEASURE * 3 / 16)
            >>> S = int(MEASURE / 16)
            >>> adapter.send_song_cmd(0, 9,
            >>>             [a4, Q, a4, Q, a4, Q, f4, Ed, c5, S, a4, Q, f4, Ed, c5, S, a4, HALF]) # set song
            >>> adapter.send_play_cmd(0) # play song
            >>> sleep(10.0) # keep playing
        """
        if len(note_list) % 2 != 0:
            raise ValueError('note_list must have an even number of elements!')

        song_length = int(len(note_list) / 2)
        format = 'B' * (len(note_list) + 3)

        self.send_cmd('Song', song_number, song_length, *note_list, format=format)

    def send_play_cmd(self, song_number):
        """
        send play command

        This command lets you select a song to play from the songs added to Roomba using the Song command.
        You must add one or more songs to Roomba using the Song command in order for the Play command to work.

        :param int song_number: (0-4)

        Examples:
            >>> adapter = PyRoombaAdapter("/dev/ttyUSB0")
            >>> # note names
            >>> f4 = 65
            >>> a4 = 69
            >>> c5 = 72
            >>> # note lengths
            >>> MEASURE = 160
            >>> HALF = int(MEASURE / 2)
            >>> Q = int(MEASURE / 4)
            >>> Ed = int(MEASURE * 3 / 16)
            >>> S = int(MEASURE / 16)
            >>> adapter.send_song_cmd(0, 9,
            >>>             [a4, a4, a4, f4, c5, a4, f4, c5, a4],
            >>>             [Q, Q, Q, Ed, S, Q, Ed, S, HALF]) # set song
            >>> adapter.send_play_cmd(0) # play song
            >>> sleep(10.0) # keep playing
        """
        self.send_cmd("Play", song_number)

    def _connect_serial(self, port, baudrate, time_out):
        self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=time_out)
        if self.serial_port.isOpen():
            print('Serial port is open, presumably to a roomba...')
        else:
            print('Serial port did NOT open')

    @staticmethod
    def _min_max(val, min_val, max_val):
        """Returns a copy of val that is an integer between min_val and max_val.
        """
        val = val if val < max_val else max_val
        val = val if val > min_val else min_val

        return int(val)
