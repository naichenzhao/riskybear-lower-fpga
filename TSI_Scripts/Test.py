import struct
import time
import serial

port = "/dev/ttyUSB1"
baudrate = 115200

ser = serial.Serial(port=port, baudrate=baudrate)


CMD_READ = 0x00
CMD_WRITE = 0x01


def readWord(address):
    command = CMD_READ
    header = struct.pack("<LQQ", command, address, 0)
    print("read:", header)
    ser.write(header)

    received = ser.read(4)
    rx_data, = struct.unpack("<L", received)
    return rx_data

def writeWord(address, data):
    command = CMD_WRITE
    header = struct.pack("<LQQ", command, address, 0)
    payload = struct.pack("<L", data)
    buffer = header + payload
    print("write:", buffer)
    ser.write(buffer)


if __name__ == "__main__":

    writeWord(0x080000000, 0x12423)
    time.sleep(1)
    
    print(hex(readWord(0x080000000)))

