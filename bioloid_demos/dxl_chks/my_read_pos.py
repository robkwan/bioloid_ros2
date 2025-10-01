#!/usr/bin/env python3
from dynamixel_sdk import *
import time

def read_all_positions():
    # Port settings
    DEVICENAME = '/dev/ttyUSB0'
    BAUDRATE = 1000000
    PRESENT_POSITION_ADDR = 36
    PRESENT_POSITION_SIZE = 2
    
    # Initialize port
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(1.0)
    
    if not portHandler.openPort():
        print("Failed to open the port")
        return
    
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set the baudrate")
        portHandler.closePort()
        return
    
    print(f"Port opened successfully: {DEVICENAME}")
    print(f"Baudrate set to: {BAUDRATE}")
    print("-" * 80)
    
    previous_positions = None
    
    # Fixed values for ID07 to ID18
    fixed_values = "353 670 508 515 297 726 282 741 597 426 508 515"
    
    try:
        while True:
            positions = []
            
            # Read position from servos ID 1-6 only
            for servo_id in range(1, 7):
                present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
                    portHandler, servo_id, PRESENT_POSITION_ADDR
                )
                
                if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                    positions.append(present_position)
                else:
                    positions.append(0)  # Use 0 for error cases
            
            # Only print if positions changed from previous reading
            if positions != previous_positions:
                # Convert positions to strings for printing
                position_strings = [str(p) for p in positions]
                print(f"step=512 {' '.join(position_strings)} {fixed_values} 512 512 512 512 512 512 512 0.000 0.496")
                previous_positions = positions.copy()
            
            time.sleep(0.1)  # Add small delay between reads
            
    except KeyboardInterrupt:
        print("\n" + "-" * 80)
        print("Stopped by user")
    
    finally:
        portHandler.closePort()
        print("Port closed")

if __name__ == "__main__":
    read_all_positions()
