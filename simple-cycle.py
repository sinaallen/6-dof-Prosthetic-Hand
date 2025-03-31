## Felix Wilton
## Feb 19, 2025
## Cycles a single actuator's position between min and max stroke

import serial
import time

# Register address description - RH56 user manual page 11, section 2.4 register description
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

# Main function: Open serial port, cycle actuator position between min and max stroke
if __name__ == '__main__':
    target_actuator = 3

    print('Opening serial port!')
    serialPort = openSerial('COM3', 921600)  # Change to your serial port and baud rate (default 921600)
    time.sleep(1)
    while True:
        writePosition(serialPort, target_actuator, 'tarLocatSet', 0)  
        time.sleep(1)
        writePosition(serialPort, target_actuator, 'tarLocatSet', 0)
        time.sleep(1)
