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
    cmd = {
        "Start": 128,
        "Baud": 129,
        "Control": 130,
        "Safe": 131,
        "Full": 132,
        "Power": 133,
        "Spot": 134,
        "Clean": 135,
        "Max": 136,
        "Drive": 137,
        "Moters": 138,
        "Song": 140,
        "Play": 141,
        "Sensors": 142,
        "Seek Dock": 143,
        "PWM Moters": 144,
        "Drive Direct": 145,
        "Drive PWM": 146,
        "Stream": 148,
        "Query List": 149,
        "Toggle Stream": 150,
        "Buttons": 165,
    }
    commands = {
        "Start": {'id': 128, 'format': '', },
        "Baud": {'id': 129, 'format': '', },
        "Control": {'id': 130, 'format': '', },
        "Safe": {'id': 131, 'format': '', },
        "Full": {'id': 132, 'format': '', },
        "Power": {'id': 133, 'format': '', },
        "Spot": {'id': 134, 'format': '', },
        "Clean": {'id': 135, 'format': '', },
        "Max": {'id': 136, 'format': '', },
        "Drive": {'id': 137, 'format': '', },
        "Moters": {'id': 138, 'format': '', },
        "Song": {'id': 140, 'format': '', },
        "Play": {'id': 141, 'format': '', },
        "Sensors": {'id': 142, 'format': 'BB', },
        "Seek Dock": {'id': 143, 'format': '', },
        "PWM Moters": {'id': 144, 'format': '', },
        "Drive Direct": {'id': 145, 'format': '', },
        "Drive PWM": {'id': 146, 'format': '', },
        "Stream": {'id': 148, 'format': '', },
        "Query List": {'id': 149, 'format': '', },
        "Toggle Stream": {'id': 150, 'format': '', },
        "Buttons": {'id': 165, 'format': '', },
    }

    STRAIGHT_RADIUS = 32768
    MIN_RADIUS = -2000
    MAX_RADIUS = 2000
    MIN_VELOCITY = -500
    MAX_VELOCITY = 500

    def __init__(self, port, baudrate=115200, timeout_sec=None, wheel_span_mm=235.0, sensor_list=sensors.DEFAULT_SENSORS):
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

    def send_cmd(self, command, *args, format=None):
        """Send a command to the robot.
        """
        if not format:
            format = self.commands[command]['format']

        self.send_struct(format, self.commands[command]['id'], *args)

    def send_struct(self, format, *args):
        """Send a struct packed command to the robot.
        """
        self.serial_port.write(struct.pack(format, *args))

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
        self._send_cmd("Clean")

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
        self._send_cmd("Max")

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
        self._send_cmd("Spot")

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
        self._send_cmd("Seek Dock")

    def change_mode_to_passive(self):
        """
        Change mode to passive mode

        Roomba beeps once to acknowledge it is starting from “off” mode.

        - Available in modes: Passive, Safe, or Full
        """
        self._send_cmd("Start")
        self._send_cmd("Control")

    def change_mode_to_safe(self):
        """
        Change mode to safe mode

        Safe mode turns off all LEDs.
        If a safety condition occurs, Roomba reverts automatically to Passive mode.

        - Available in modes: Passive, Safe, or Full
        """
        # send command
        self._send_cmd("Safe")

    def change_mode_to_full(self):
        """
        Change mode to full mode

        Full mode turns off the cliff, wheel-drop and internal charger safety features.
        In Full mode, Roomba executes any command that you send it, even if the internal charger is plugged in,
        or command triggers a cliff or wheel drop condition.

        - Available in modes: Passive, Safe, or Full
        """
        # send command
        self._send_cmd("Full")

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
        self._send_cmd("Power")

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

    def send_drive_cmd(self, roomba_mm_sec, roomba_radius_mm):
        """
        send drive command

        This command controls Roomba’s drive wheels.
        The radius is measured from the center of the turning circle to the center of Roomba.
        A Drive command with a positive velocity and a positive radius makes Roomba drive forward while turning left.
        A negative radius makes Roomba turn toward the right.
        Special cases for the radius make Roomba turn in place or drive straight.
        A negative velocity makes Roomba drive backward.

        :param float roomba_mm_sec: the average velocity of the drive wheels in millimeters per second (-500 – 500 mm/s)

        :param float roomba_radius_mm: the radius in millimeters at which Roomba will turn. (-2000 – 2000 mm)

        Examples:
            >>> PORT = "/dev/ttyUSB0"
            >>> adapter = PyRoombaAdapter(PORT)
            >>> adapter.send_drive_cmd(-100, -1000) # back to the right side
            >>> sleep(2.0) # keep 2 sec
        """
        # print(roomba_mm_sec, roomba_radius_mm, turn_dir)

        roomba_mm_sec = self._adjust_min_max(roomba_mm_sec, self.MIN_VELOCITY, self.MAX_VELOCITY)
        velHighVal, velLowVal = self._get_2_bytes(roomba_mm_sec)

        roomba_radius_mm = self._adjust_min_max(roomba_radius_mm, self.MIN_RADIUS, self.MAX_RADIUS)
        radiusHighVal, radiusLowVal = self._get_2_bytes(roomba_radius_mm)

        # send these bytes and set the stored velocities
        self._send_cmd("Drive", velHighVal, velLowVal, radiusHighVal, radiusLowVal)

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
        right_mm_sec = self._adjust_min_max(right_mm_sec, self.MIN_VELOCITY, self.MAX_VELOCITY)
        right_high, right_low = byte_tool.get_2_bytes(right_mm_sec)

        left_mm_sec = self._adjust_min_max(left_mm_sec, self.MIN_VELOCITY, self.MAX_VELOCITY)
        left_high, left_low = byte_tool.get_2_bytes(left_mm_sec)

        # send these bytes and set the stored velocities
        self._send_cmd("Drive Direct", right_high, right_low, left_high, left_low)

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
        right_pwm = self._adjust_min_max(right_pwm, -255, 255)
        right_high, right_low = byte_tool.get_2_bytes(right_pwm)

        left_pwm = self._adjust_min_max(left_pwm, -255, 255)
        left_high, left_low = byte_tool.get_2_bytes(left_pwm)

        # send these bytes and set the stored velocities
        self._send_cmd("Drive PWM", right_high, right_low, left_high, left_low)

    def send_moters_cmd(self, main_brush_on, main_brush_direction_is_ccw,
                        side_brush_on, side_brush_direction_is_inward,
                        vacuum_on
                        ):
        """
        send moters command

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
            >>> adapter.send_moters_cmd(False, True, True, True, False) # side brush is on, and it rotates inward
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

        self._send_cmd("Moters", cmd)

    def send_pwm_moters(self, main_brush_pwm, side_brush_pwm, vacuum_pwm):
        """
        send pwm moters

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
            >>> adapter.send_pwm_moters(-55, 0, 0) # main brush is 55% PWM to opposite direction
            >>> sleep(2.0) # keep 2 sec
        """
        main_brush_pwm = self._get_1_bytes(self._adjust_min_max(main_brush_pwm, -127, 127))
        side_brush_pwm = self._get_1_bytes(self._adjust_min_max(side_brush_pwm, -127, 127))
        vacuum_pwm = self._adjust_min_max(vacuum_pwm, 0, 127)
        self._send_cmd("PWM Moters", main_brush_pwm, side_brush_pwm, vacuum_pwm)

    def send_buttons_cmd(self, clean=False, spot=False, dock=False,
                         minute=False, hour=False, day=False, schedule=False, clock=False):
        """
        send buttons command

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
        buttons = 0  # All initial bit is 0
        if clean:
            buttons |= 0b00000001
        if spot:
            buttons |= 0b00000010
        if dock:
            buttons |= 0b00000100
        if minute:
            buttons |= 0b00001000
        if hour:
            buttons |= 0b00010000
        if day:
            buttons |= 0b00100000
        if schedule:
            buttons |= 0b01000000
        if clock:
            buttons |= 0b10000000

        self._send_cmd("Buttons", buttons)

    def send_song_cmd(self, song_number, song_length, note_number_list, note_duration_list):
        """
        Send song command

        This command lets you specify up to four songs to the OI that you can play at a later time.
        Each song is associated with a song number.
        The Play command uses the song number to identify your song selection.
        Each song can contain up to sixteen notes.
        Each note is associated with a note number that uses MIDI note definitions and a duration
        that is specified in fractions of a second.
        The number of data bytes varies, depending on the length of the song specified.
        A one note song is specified by four data bytes.
        For each additional note within a song, add two data bytes.

        - Available in modes: Passive, Safe, or Full

        :param int song_number: (0-4) The song number associated with the specific song.
                        If you send a second Song command, using the same song number, the old song is overwritten.

        :param int song_length: (1-16) The length of the song, according to the number of musical notes within the song.

        :param list note_number_list: Note Number (31 – 127) The pitch of the musical note Roomba will play,
                                      according to the MIDI note numbering scheme.
                                      The lowest musical note that Roomba will play is Note #31.
                                      Roomba considers all musical notes outside the range of 31 – 127 as rest notes,
                                      and will make no sound during the duration of those notes.

        :param list note_duration_list: Note Duration (0 – 255) The duration of a musical note,
                                        in increments of 1/64th of a second. Example: a half-second long
                                        musical note has a duration value of 32.

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
        args = [song_number, song_length]
        for (note_number, note_duration) in zip(note_number_list, note_duration_list):
            args.append(note_number)
            args.append(note_duration)
        self._send_cmd('Song', *args)

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
        self._send_cmd("Play", song_number)

    def _connect_serial(self, port, baudrate, time_out):
        self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=time_out)
        if self.serial_port.isOpen():
            print('Serial port is open, presumably to a roomba...')
        else:
            print('Serial port did NOT open')

    @staticmethod
    def _adjust_min_max(val, min_val, max_val):

        # integer cast
        if type(val) != int:
            val = int(val)

        if val < min_val:
            val = min_val
        elif val > max_val:
            val = max_val

        return val

    @staticmethod
    def _get_2_bytes(value):
        """ returns two bytes (ints) in high, low order
        whose bits form the input value when interpreted in
        two's complement
        """
        # if positive or zero, it's OK
        if value >= 0:
            eqBitVal = value
        # if it's negative, I think it is this
        else:
            eqBitVal = (1 << 16) + value

        return (eqBitVal >> 8) & 0xFF, eqBitVal & 0xFF

    @staticmethod
    def _get_1_bytes(value):
        """ returns one bytes (int)
        """
        # if positive or zero, it's OK
        if value >= 0:
            eqBitVal = value
        # if it's negative, I think it is this
        else:
            eqBitVal = (1 << 8) + value
        return eqBitVal & 0xFF

    def _send_cmd(self, command, *args):
        """Sends a command to the roomba.
        """
        cmd = [self.cmd[command], *args]
        self.serial_port.write(bytes(cmd))
        sleep(0.1)


def main():
    PORT = "/dev/ttyUSB0"
    adapter = PyRoombaAdapter(PORT)
    # adapter.send_drive_cmd(-100, -1000)
    # adapter.send_drive_direct(-100, 100)
    import numpy as np
    # adapter.move(0.1, np.deg2rad(-10))
    # adapter.send_drive_pwm(80, 80)
    # adapter.send_drive_pwm(-200, -200)
    # adapter.send_moters_cmd(False, True, True, True, False)
    # adapter.send_pwm_moters(-55, -25, 25)
    # adapter.send_buttons_cmd(dock=True)
    # sleep(1.0)

    adapter.move(0.2, np.deg2rad(0.0))
    sleep(1.0)
    adapter.move(0, np.deg2rad(-20))
    sleep(6.0)
    adapter.move(0.2, np.deg2rad(0.0))
    sleep(1.0)
    adapter.move(0, np.deg2rad(20))
    sleep(6.0)

    # adapter.move(-0.1, 0)
    # sleep(1.0)


if __name__ == '__main__':
    main()
