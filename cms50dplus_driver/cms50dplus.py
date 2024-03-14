#!/usr/bin/env python3
import sys
import serial
import time
import sys
import argparse

try:
    # if loaded as python module
    from repeated_timer import RepeatedTimer
except ImportError:
    # if loaded from ros2 framework
    from cms50dplus_driver.repeated_timer import RepeatedTimer

# Decoded with datasheet https://www.tranzoa.net/~alex/blog/wp-content/uploads/2018/08/CMS60DWCommunication-protocol-of-pulse-oximeter-V7.0.pdf

# 0x7D is package type for downlink commands to the device
# 0xA1 ask for continous real-time data
# 0x80 is unused data
CMD_QUERY = b'\x7d\x81\xa1\x80\x80\x80\x80\x80\x80'

# 0xAF Inform device beeing connected (every 5 seconds)
# CMD_MAINTAIN = b'\x7d\x81\xaf\x80\x80\x80\x80\x80\x80'


class CMS50DPlus:
    def __init__(self, serialport):
        try:
            self.serial_connection = serial.Serial(serialport,
                                                   baudrate=115200,
                                                   timeout=2,
                                                   xonxoff=1,
                                                   bytesize=serial.EIGHTBITS,
                                                   stopbits=serial.STOPBITS_ONE,
                                                   parity=serial.PARITY_NONE)

            print("Port: " + serialport, "Baudrate: "+str(self.serial_connection.baudrate),
                  'Connected: ' + str(self.serial_connection.is_open))
        except OSError as err:
            print("Cant open specified port: "+str(err))
            print("Device is not connected or the port is wrong or no sudo permissions")
            sys.exit(1)

        self.timer = RepeatedTimer(5, self.send_query)  # send cmd_maintain every 5sec to maintain the stream session
        print("Recording...\n^C to stop.")

    def loop(self) -> tuple[int, int, int] | None:
        try:
            raw_data = self.serial_connection.read(9)  # read each 9-byte package
        except serial.SerialException:
            print("Trying to reconnect...")
            self.serial_connection.close()
            time.sleep(1)
            self.timer.start()
            return None
        return self.decode_data(raw_data)

    def send_query(self):
        if self.serial_connection.is_open:
            self.serial_connection.write(CMD_QUERY)
            print("sending data query")
        else:
            try:
                self.serial_connection.open()
                time.sleep(0.1)
                self.serial_connection.flush()
                print('Connected: ' + str(self.serial_connection.is_open))
            except OSError as err:
                print("Serial connection is closed ", str(err))
                print("Device was disconnected")
                self.timer.stop()

    def decode_data(self, data) -> tuple[int, int, int]:
        # DIFFERS FROM THE DOCUMENTATION
        bytes_hex = [data.hex()[i:i+2] for i in range(0, len(data.hex()), 2)]
        bytes_binary = [bin(int(hex_value, 16))[2:].zfill(8) for hex_value in bytes_hex]

        # print(bytes_hex)
        # print(bytes_binary)
        # print([int(str(byte),2) for byte in bytes_binary])

        if len(bytes_hex) == 0:
            print("ERROR: No data recieved, Probably device is not turned on")
            return

        if len(bytes_hex) != 9:
            print("ERROR: Invalid data length:", len(bytes_hex))
            return

        # first byte, real-time data package must be 0x01 = 00000001
        if bytes_binary[0] != '00000001':
            print("ERROR: No real-time data package")
            return

        # following bytes have always bit7 set to 1 = 1xxxxxxx
        # second byte, status infos
        if bytes_binary[1][7] == '1':
            print("WARN: Finger is not in the device")
            return
        if bytes_binary[1][6] == '1':
            print("pulse beep sound")
        if bytes_binary[1][5] == '1':
            print("dropping of SpO2")
        if bytes_binary[1][4] == '1':
            print("searching time too long")
        signal_strength = min(int(str(bytes_binary[0])[1:4], 2), 8)  # if larger than 8, default to 8

        # fourth byte, waveform data
        wave = int(str(bytes_binary[3])[1:8], 2)

        # sixth byte, pulse rate value
        bpm = int(str(bytes_binary[5])[1:8], 2)

        # seventh byte, spo2 value
        spo2 = int(str(bytes_binary[6])[1:8], 2)
        if spo2 > 100:
            print("SpO2 is invalid")
            spo2 = -1

        return bpm, spo2, wave


def main():
    parser = argparse.ArgumentParser(description="cms50dplus.py - A driver for the Contec CMS50D+ pulse oximeter.")
    parser.add_argument("serialport", help="The device's virtual serial port.")
    args = parser.parse_args()

    driver = CMS50DPlus(args.serialport)

    while True:
        try:
            data = driver.loop()

            if data is not None:
                bpm, spo2, wave = data
                print(bpm, spo2, wave)

        except KeyboardInterrupt:
            driver.timer.stop()
            print('Recording stopped.')
            break


if __name__ == "__main__":
    main()
