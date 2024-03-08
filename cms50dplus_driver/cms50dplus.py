import sys
import serial
import calendar;
import time;
import signal
import sys
import os
from threading import Timer

# Decoded with datasheet https://www.tranzoa.net/~alex/blog/wp-content/uploads/2018/08/CMS60DWCommunication-protocol-of-pulse-oximeter-V7.0.pdf

# 0x7D is package type for downlink commands to the device
# 0xA1 ask for continous real-time data
# 0x80 is unused data
cmd_query = b'\x7d\x81\xa1\x80\x80\x80\x80\x80\x80'

# 0xAF Inform device beeing connected (every 5 seconds)
cmd_maintain = b'\x7d\x81\xaf\x80\x80\x80\x80\x80\x80'

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def signal_handler(sig, frame):
    tf = calendar.timegm(time.gmtime())
    print(tf-ts, ' Secondes recorded !')
    global rt
    global file
    rt.stop()
    with open(file, "a") as myfile:
        myfile.write(str(tf))
    sys.exit(0)

def send_query():
    if serial_connection.is_open:
        serial_connection.write(cmd_query)
        print("sending data query")
    else:
        try:
            serial_connection.open()
            time.sleep(0.1)
            serial_connection.flush()
            print('Connected: '+ str(serial_connection.is_open))
        except OSError as err:
            print("Serial connection is closed ", str(err))
            print("Device was disconnected")
            rt.stop()

def decode_data(data):
    #### DIFFERS FROM THE DOCUMENTATION
    bytes_hex = [data.hex()[i:i+2] for i in range(0 ,len(data.hex()) ,2)]
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
    signal_strength = min(int(str(bytes_binary[0])[1:4],2), 8) # if larger than 8, default to 8

    # fourth byte, waveform data
    wave = int(str(bytes_binary[3])[1:8], 2)

    # sixth byte, pulse rate value 
    bpm = int(str(bytes_binary[5])[1:8], 2)

    # seventh byte, spo2 value
    spo2 = int(str(bytes_binary[6])[1:8], 2)
    if spo2 > 100:
        print("SpO2 is invalid")
        spo2 = -1

    print(bpm, spo2, wave)


signal.signal(signal.SIGINT, signal_handler)
if os.path.exists(sys.argv[2]):
    os.remove(sys.argv[2])

try:
    serial_connection = serial.Serial(sys.argv[1],
                        baudrate=115200, #in my device, this is the baudrate where to open the serial, in other devices try 4800, 19200...
                        timeout=2,
                        xonxoff=1,
                        bytesize=serial.EIGHTBITS,
                        stopbits=serial.STOPBITS_ONE,
                        parity=serial.PARITY_NONE) #another important looking like parameter, setting the parity to NONE

    print("Port: "+sys.argv[1], "Baudrate: "+str(serial_connection.baudrate), 'Connected: '+ str(serial_connection.is_open))
except OSError as err:
    print("Cant open specified port: "+str(err))
    print("Device is not connected or the port is wrong")
    sys.exit(1)

# serial_connection.write(cmd_query)

file = sys.argv[2]
rt = RepeatedTimer(5, send_query) #send cmd_maintain every 5sec to maintain the stream session
print("Recording...\n^C to stop.")

ts = calendar.timegm(time.gmtime()) #10 characters to skip when decoding data
with open(sys.argv[2], "a") as myfile:
        myfile.write(str(ts))
while True:
    try:
        data = serial_connection.read(9) #read each byte
    except serial.SerialException:
        print("Trying to reconnect...")
        serial_connection.close()
        time.sleep(1)
        rt.start()
        continue
    decode_data(data)
    # print(data)
    with open(sys.argv[2], "a") as myfile:
        myfile.write(data.hex())
