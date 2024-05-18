import struct
import time
import serial

port = "/dev/ttyUSB0"
baudrate = 115200

ser = serial.Serial(port=port, baudrate=baudrate)


CMD_READ = 0x00
CMD_WRITE = 0x01


def readWord(address):
    command = CMD_READ
    header = struct.pack("<LQQ", command, address, 0)
    
    
    # bytes_as_bits = header.decode('utf-8')
    for i in range(len(header)):
      print("read:", hex(header[i]))
      
    print("dpne")
    ser.write(header)

    received = ser.read(4)
    rx_data, = struct.unpack("<L", received)
    for i in range(len(received)):
      print("ret:", hex(received[i]))
    return rx_data

def writeWord(address, data):
    command = CMD_WRITE
    header = struct.pack("<LQQ", command, address, 0)
    payload = struct.pack("<L", data)
    buffer = header + payload
    # print("write:", buffer)
    
    for i in range(len(buffer)):
      print("write:", hex(buffer[i]))
    ser.write(buffer)


if __name__ == "__main__":

    # writeWord(0x13000120, 0x1)
    M = 0x700
    
    pos = 0x13000000 + M
    state = 0x13000004 + M
    enable = 0x13000008 + M
    speed = 0x13000020 + M
    dir = 0x1300000C + M
    presc = 0x13000024 + M
    
    writeWord(0x13000730, 0xFFFFFFFF)
    time.sleep(20)
    

    for i in range(1000):
      print(hex(readWord(0x13000030)), hex(readWord(0x13000130)), hex(readWord(0x13000230)), hex(readWord(0x13000330)), hex(readWord(0x13000430)), hex(readWord(0x13000530)), hex(readWord(0x13000630)), hex(readWord(0x13000730)))
      # print(int(readWord(0x13000730)), int(readWord(0x13000748)))
      time.sleep(0.2)

