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
    # print("read:", header)
    ser.write(header)

    received = ser.read(4)
    rx_data, = struct.unpack("<L", received)
    return rx_data

def writeWord(address, data):
    command = CMD_WRITE
    header = struct.pack("<LQQ", command, address, 0)
    payload = struct.pack("<L", data)
    buffer = header + payload
    # print("write:", buffer)
    ser.write(buffer)


if __name__ == "__main__":

    # writeWord(0x13000120, 0x1)
    M = 0x200
    
    addr = 0x13000000 + M
    writeWord(addr, 0x0FFF)
    print(hex(readWord(addr)))
    
    addr = 0x13000008 + M
    writeWord(addr, 0x01)
    print(hex(readWord(addr)))
    
    addr = 0x13000009 + M
    writeWord(addr, 0x00)
    print(hex(readWord(addr)))
    
    addr = 0x1300000C + M
    writeWord(addr, 0x0FFF)
    print(hex(readWord(addr)))
    
    addr = 0x13000010 + M
    writeWord(addr, 0x00FF)
    print(hex(readWord(addr)))
    

    

    
    # for i in range(1000):
    #   print(hex(readWord(0x13000018)), hex(readWord(0x13000118)), hex(readWord(0x13000218)), hex(readWord(0x13000318)), hex(readWord(0x13000418)), hex(readWord(0x13000518)), hex(readWord(0x13000618)), hex(readWord(0x13000718)))
    #   time.sleep(0.2)

