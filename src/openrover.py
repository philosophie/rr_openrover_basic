from time import sleep

import serial.threaded


class OpenRoverProtocol(serial.threaded.Protocol):
    SERIAL_START_BYTE = 253

    @staticmethod
    def checksum(values):
        return 255 - sum(values) % 255

    def __init__(self):
        self.buffer = bytearray()
        self.transport = None
        self.callback = (lambda k, v: None)

    def connection_made(self, transport):
        """Store transport"""
        self.transport = transport

    def connection_lost(self, exc):
        """Forget transport"""
        self.transport = None

    def data_received(self, data):
        """Called with snippets received from the serial port"""
        self.buffer.extend(data)

        while True:
            start_byte_index = self.buffer.find(bytearray([self.SERIAL_START_BYTE]))
            self.buffer[:start_byte_index] = []

            if len(self.buffer) < 5:
                return

            packet, self.buffer = bytearray(self.buffer[:5]), self.buffer[5:]

            if packet[4] == self.checksum(packet[1:4]):
                k = packet[1]
                v = (packet[2] << 8) + packet[3]
                cb = self.callback
                cb(k, v)
            else:
                pass  # packet is bad. ignore it

    @staticmethod
    def encode_speed(speed_as_float):
        return int(round(speed_as_float * 125) + 125)

    def write(self, motor_left, motor_right, flipper, arg1, arg2):
        payload = [self.encode_speed(motor_left),
                   self.encode_speed(motor_right),
                   self.encode_speed(flipper),
                   arg1,
                   arg2]
        packet = bytearray()
        packet.append(self.SERIAL_START_BYTE)
        packet.extend(payload)
        packet.append(self.checksum(payload))
        return self.transport.write(packet)

    def on_read(self, cb):
        self.callback = cb


class OpenRover:
    _motor_left = 0
    _motor_right = 0
    _motor_flipper = 0
    latest_data = None

    def __init__(self, protocol):
        self._motor_left = 0
        self._motor_right = 0
        self._motor_flipper = 0
        self.protocol = protocol
        self.latest_data = dict()
        protocol.on_read(self.on_new_openrover_data)

    def on_new_openrover_data(self, key, value):
        self.latest_data[key] = value

    def set_motor_speeds(self, left, right, flipper):
        assert -1 <= left <= 1
        assert -1 <= right <= 1
        assert -1 <= flipper <= 1
        self._motor_left = left
        self._motor_right = right
        self._motor_flipper = flipper

    def send_command(self, arg1, arg2):
        assert 0 <= arg1 <= 255
        assert 0 <= arg2 <= 255
        self.protocol.write(self._motor_left, self._motor_right, self._motor_flipper, arg1, arg2)

    def send_speed(self):
        self.send_command(0, 0)

    def set_fan_speed(self, fan_speed):
        self.send_command(20, fan_speed)

    def flipper_calibrate(self):
        self.send_command(250, 250)

    def request_data(self, index):
        self.send_command(10, index)

    def get_data(self, index):
        self.request_data(index)
        sleep(0.05)
        return self.latest_data.get(index)


if __name__ == '__main__':
    device_name = '/dev/rover'
    ser = serial.Serial(device_name, baudrate=57600, timeout=0.5, write_timeout=0.5, stopbits=2)
    try:
        with serial.threaded.ReaderThread(ser, OpenRoverProtocol) as orp:
            rover = OpenRover(orp)
            rover.request_data(40)

            for i in range(10):
                result = rover.get_data(40)
                assert (result == 40621)

            rover.set_motor_speeds(0, 0, 0)
            for i in range(5):
                rover.set_fan_speed(200)
                sleep(1)  # listen to the fan for a few seconds

            rover.set_fan_speed(0)
            pass
    except Exception as e:
        pass
