## Felix Wilton
## Feb 19, 2025
## Cycles all 5  actuators position between min and max stroke with offsets

COM_PORT = 'COM4'  # Change to your serial port

import serial
import time

# Register address description, corresponding to the humanoid five-finger dexterous hand - RH56 user manual page 11, section 2.4 register description
regdict = {
    'ID'              : 2,    # ID
    'baudrate'        : 12,   # Baud rate setting
    'curLocat'        : 26,   # Current position
    'zeroCalibra'     : 31,   # Force sensor zero calibration setting
    'overCurproSet'   : 32,   # Overcurrent protection setting
    'tarLocatSet'     : 55,   # Target position setting
    'fSensorDada'     : 76,   # Force sensor data
    'fOriginalValue'  : 78,   # Force sensor raw value
    'forceAct'        : 98,   # Over-temperature protection setting
    'warmUpSta'       : 100,  # Warm-up activation setting
}

# Function description: Set the serial port number and baud rate, then open the serial port
# Parameters: port is the serial port number, baudrate is the baud rate
def openSerial(port, baudrate):
    ser = serial.Serial()  # Call serial communication function
    ser.port = port
    ser.baudrate = baudrate
    ser.open()             # Open serial port
    return ser

# Function description: Write operation for the actuator register
# Parameters: id is the actuator ID, add is the control table index, num is the data length, val is the data to be written into the register
def writeRegister(ser, id, add, num, val):
    bytes = [0x55, 0xAA]    # Frame header
    bytes.append(num + 2)   # Frame length
    bytes.append(id)        # ID number
    bytes.append(0x02)      # CMD_WR Write register command flag
    bytes.append(add)       # Control table index
    for i in range(num):
        bytes.append(val[i])
    checksum = 0x00         # Initialize checksum to 0
    for i in range(2, len(bytes)):
        checksum += bytes[i]  # Sum the data
    checksum &= 0xFF        # Keep the lower 8 bits of the checksum
    bytes.append(checksum)  # Append checksum
    ser.write(bytes)        # Write data to serial port
    time.sleep(0.01)        # Delay 10ms
    ser.read_all()          # Read and discard response frame without processing

# Function description: Read actuator register operation
# Parameters: id is the actuator ID, add is the start address, num is the data length
def readRegister(ser, id, add, num, mute=False):
    bytes = [0x55, 0xAA]    # Frame header
    bytes.append(num + 2)   # Frame length
    bytes.append(id)        # ID
    bytes.append(0x01)      # CMD_RD Read register command flag
    bytes.append(add)       # Control table index
    bytes.append(num)
    checksum = 0x00         # Initialize checksum to 0
    for i in range(2, len(bytes)):
        checksum += bytes[i]  # Sum the data
    checksum &= 0xFF        # Keep the lower 8 bits of the checksum
    bytes.append(checksum)  # Append checksum
    ser.write(bytes)        # Write data to serial port
    time.sleep(0.01)        # Delay 10ms
    recv = ser.read_all()   # Read bytes from port
    if len(recv) == 0:      # If the response length is 0, return immediately
        return []
    num = (recv[2] & 0xFF) - 2  # Number of register data returned
    val = []
    for i in range(num):
        val.append(recv[6 + i])
    if not mute:
        print('Read register values:', end=' ')
        for i in range(num):
            print(val[i], end=' ')
        print()
    return val

# Function description: Query actuator status
# Parameters: id is the actuator ID
def control(ser, id):
    bytes = [0x55, 0xAA]    # Frame header
    bytes.append(0x03)      # Frame length
    bytes.append(id)        # ID number
    bytes.append(0x04)      # CMD_MC Single control command
    bytes.append(0x00)      # Parameter 1 reserved
    bytes.append(0x22)      # Query actuator status
    checksum = 0x00         # Initialize checksum to 0
    for i in range(2, len(bytes)):
        checksum += bytes[i]  # Sum the data
    checksum &= 0xFF        # Keep the lower 8 bits of the checksum
    bytes.append(checksum)  # Append checksum
    ser.write(bytes)        # Write data to serial port
    time.sleep(0.01)        # Delay 10ms
    recv = ser.read_all()   # Read bytes from port
    if len(recv) == 0:      # If response length is 0, return immediately
        return []
    num = (recv[2] & 0xFF) - 3  # Number of register data returned
    val = []
    for i in range(num):
        val.append(recv[7 + i])
    print('Read register values:', end=' ')
    for i in range(num):
        print(val[i], end=' ')
    print()

# Function description: Write actuator position data
# Supported registers: zeroCalibra (force sensor zero calibration), overCurproSet (overcurrent protection), tarLocatSet (target position), forceAct (over-temperature protection), warmUpSta (warm-up activation)
def writePosition(ser, id, str, val):
    if str in ['zeroCalibra', 'overCurproSet', 'tarLocatSet', 'forceAct', 'warmUpSta']:
        val_reg = []
        for i in range(3):
          val_reg.append(val & 0xFF)
          val_reg.append((val >> 8) & 0xFF)
        writeRegister(ser, id, regdict[str], 6, val_reg)
    else:
        print('Function call error. Correct usage: str should be \'zeroCalibra\', \'overCurproSet\', \'tarLocatSet\', etc., and val should be a single-element list with values 0~1000, with -1 allowed as a placeholder.')

# Function description: Broadcast positioning mode
# Parameters: id is the actuator ID, num is the data length, val1-val6 are position values for actuators with IDs 1-6
def broadcast(ser, num, val1, val2, val3, val4, val5, val6):
    bytes = [0x55, 0xAA]               # Frame header
    bytes.append(1 + num * 3)          # Frame length
    bytes.append(0xff)                 # Broadcast ID
    bytes.append(0xf2)                 # Positioning flag
    actuator_values = [val1, val2, val3, val4, val5, val6]
    for i, val in enumerate(actuator_values):
        bytes.append(i + 1)             # Actuator ID
        bytes.append(val & 0XFF)        # Target position (low byte)
        bytes.append((val >> 8) & 0XFF) # Target position (high byte)
    checksum = sum(bytes[2:]) & 0xFF    # Compute checksum
    bytes.append(checksum)              # Append checksum
    ser.write(bytes)                    # Write data to serial port
    time.sleep(0.01)                    # Delay 10ms

# Main function: Open serial port, set actuator movement parameters
if __name__ == '__main__':
    print('Opening serial port!')
    serialPort = openSerial(COM_PORT, 921600)  # Change to your serial port and baud rate (default 921600)
    time.sleep(1)
    print('Setting positions for all 5 actuators in broadcast mode')

    
    def cycle(index):
        vals = [25, 500, 1000, 1500, 1750, 1500, 1000, 500]
        return vals[index % len(vals)]

    while True:
        for i in range(1, 9):
            broadcast(serialPort, 6, cycle(i), cycle(i-1), cycle(i-2), cycle(i-3), cycle(i-4), 0)
            time.sleep(0.25)
