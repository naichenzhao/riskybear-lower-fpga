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
    M = 0x700
    
    pos = 0x13000000 + M
    state = 0x13000004 + M
    enable = 0x13000008 + M
    speed = 0x13000020 + M
    dir = 0x1300000C + M
    presc = 0x13000024 + M
    
    writeWord(pos, 0000)
    writeWord(state, 0x1)
    writeWord(enable, 0x1)
    writeWord(dir, 0x01)
    writeWord(presc, 40)
    writeWord(speed, 600)
    writeWord(0x13000044 + M, 2)
    

    # print("state:", hex(readWord(state)))
    print("state:", hex(readWord(state)))
    print("en:", hex(readWord(enable)))
    print("speed:", hex(readWord(speed)))
    

    

    
    for i in range(1000):
      writeWord(pos, 2000)
      time.sleep(2)
      writeWord(pos, 0000)
      time.sleep(2)
      
      
      # # print(hex(readWord(0x13000018)), hex(readWord(0x13000118)), hex(readWord(0x13000218)), hex(readWord(0x13000318)), hex(readWord(0x13000418)), hex(readWord(0x13000518)), hex(readWord(0x13000618)), hex(readWord(0x13000718)))
      # print(int(readWord(0x13000730)), int(readWord(0x13000748)))
      # time.sleep(0.2)

