import serial
import time
import struct

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
# # Parameters: port is the serial port number, baudrate is the baud rate
# def openSerial(port, baudrate):
#     ser = serial.Serial()  # Call serial communication function
#     ser.port = port
#     ser.baudrate = baudrate
#     ser.open()             # Open serial port
#     return ser

# Function to open serial connection
def openSerial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Serial port {port} opened successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}")
        return None

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
# Function to query actuator status
def control(ser, id):
    """
    Queries the status of an actuator with the given ID.
    Returns tuple (current_pos, temp, current, force) or None if failed.
    """
    # Command frame: [header, length, ID, instruction, param1, param2, checksum]
    command = [0x55, 0xAA, 0x03, id, 0x04, 0x00, 0x22]
    checksum = sum(command[2:]) & 0xFF
    command.append(checksum)
    
    ser.write(bytes(command))
    time.sleep(0.01)  # Small delay for response
    print("reading response...")
    # Read response (22 bytes expected based on typical actuator protocol)
    response = ser.read(22)
    print("response read")
    if len(response) == 22 and response[0] == 0xAA and response[1] == 0x55:
        # Verify checksum
        calc_checksum = sum(response[2:21]) & 0xFF
        if calc_checksum == response[21]:
            # Parse response fields
            current_pos = struct.unpack('<h', response[9:11])[0]  # Signed 16-bit
            temp = struct.unpack('<b', response[11:12])[0]        # Signed 8-bit
            current = struct.unpack('<H', response[12:14])[0]     # Unsigned 16-bit
            force = struct.unpack('<h', response[14:16])[0]       # Signed 16-bit
            return current_pos, temp, current, force
        else:
            print(f"Checksum error for actuator {id}")
    else:
        print(f"Incomplete or invalid response from actuator {id}")
    return None

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

# Main script
def main():
   # Configuration
    port = 'COM10'  # Adjust to your serial port
    baudrate = 921600  # Adjust to your baud rate
    num_actuators = 5
    actuator_ids = list(range(1, num_actuators + 1))  # IDs 1 to 6
    
    # Open serial port
    ser = openSerial(port, baudrate)
    if ser is None:
        return
    
    time.sleep(1)  # Allow time for connection to stabilize
    
    # set initial position to 0
    target_position = 50
    broadcast(ser, 6, *[target_position] * 6)
    time.sleep(1)  

    # Initialize target position
    target_position = 1800
    print(f"Setting initial target to {target_position}")
    
    while True:
        # Send position command to all actuators
        broadcast(ser, 6, *[target_position] * 6)
        print(f"Sent command to move all actuators to {target_position}")
        
        # Wait until all actuators reach the target position
        while True:
            time.sleep(1)  # Query every second
            all_reached = True
            for id in actuator_ids:
                status = control(ser, id)
                if status:
                    current_pos, temp, current, force = status
                    print(f"Actuator {id}, current position: {current_pos}, temp: {temp}, current: {current}, force: {force}")
                    if abs(current_pos - target_position) > 10:
                        all_reached = False
                else:
                    print(f"Failed to get status for actuator {id}")
                    all_reached = False
            
            if all_reached:
                print(f"All actuators have reached position {target_position}")
                break
        
        # Switch target position
        target_position = 50 if target_position == 1800 else 1800
        print(f"Switching target to {target_position}")


if __name__ == "__main__":
    main()
