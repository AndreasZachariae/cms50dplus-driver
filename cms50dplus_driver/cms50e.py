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
cmd_querry = b'\x7d\x81\xa1\x80\x80\x80\x80\x80\x80'

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

def maintain():
    serial.write(cmd_querry)
    print("sending data query")

def decode_data(data):
    bytes_hex = [data.hex()[i:i+2] for i in range(0 ,len(data.hex()) ,2)]
    bytes_binary = [bin(int(hex_value, 16))[2:].zfill(8) for hex_value in bytes_hex]

    if len(bytes_hex) != 9:
        print("ERROR: Invalid data length:", len(bytes_hex))
        return
    
    # first byte, real-time data package 0x01 = 0000 0001
    if bytes_binary[0] != '00000001':
        print("ERROR: No real-time data package")
        return

    # print(bytes_hex)
    # print(bytes_binary)
    # print([int(str(byte),2) for byte in bytes_binary])

    # following bytes have always first position set to 1 = 1... ....
    # second byte, status infos
    if bytes_binary[1][7] == '1':
        print("ERROR: Finger is not in the device")
        return
    if bytes_binary[1][6] == '1':
        print("pulse beep sound")
    if bytes_binary[1][5] == '1':
        print("dropping of SpO2")
    if bytes_binary[1][4] == '1':
        print("searching time too long")
    signal_strength = min(int(str(bytes_binary[0])[1:4],2), 8) # if larger than 8, default to 8

    # third byte
    if bytes_binary[2][7] == '1':
        print("searching for pulse")
    # waveform_data = int(str(bytes_binary[2])[1:7], 2)

    # fourth byte
    # 6-8 positions meaningless, reservation
    # if bytes_binary[3][5] == '1':
    #     print("PI data is invalid")
    # bar_graph_data = int(str(bytes_binary[3])[1:4], 2)

    # fifth byte
    # if bytes_binary[4] == '11111111':
    #     print("Pulse rate is invalid")
    # pulse_rate = int(str(bytes_binary[4])[1:8], 2)

    # sixth byte, pulse rate value #### DIFFERS FROM THE DOCUMENTATION
    pulse_rate = int(str(bytes_binary[5])[1:8], 2)

    # seventh byte, spo2 value #### DIFFERS FROM THE DOCUMENTATION
    spo2 = int(str(bytes_binary[6])[1:8], 2)
    if spo2 > 100:
        print("SpO2 is invalid")

    print(pulse_rate, spo2)


signal.signal(signal.SIGINT, signal_handler)
if os.path.exists(sys.argv[2]):
    os.remove(sys.argv[2])
serial = serial.Serial(sys.argv[1],
                    baudrate=115200, #in my device, this is the baudrate where to open the serial, in other devices try 4800, 19200...
                    timeout=2,
                    xonxoff=1,
                    bytesize=serial.EIGHTBITS,
                    stopbits=serial.STOPBITS_ONE,
                    parity=serial.PARITY_NONE) #another important looking like parameter, setting the parity to NONE
                                    
serial.write(cmd_querry)

file = sys.argv[2]
rt = RepeatedTimer(5, maintain) #send cmd_maintain every 5sec to maintain the stream session
print("Recording...\n^C to stop.")

ts = calendar.timegm(time.gmtime()) #10 characters to skip when decoding data
with open(sys.argv[2], "a") as myfile:
        myfile.write(str(ts))
while True:
    data = serial.read(9) #read each byte
    decode_data(data)
    # print(data)
    with open(sys.argv[2], "a") as myfile:
        myfile.write(data.hex())
