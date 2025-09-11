#!/usr/bin/env python3

from dynamixel_sdk import PortHandler, PacketHandler

# --- AX-12A compliance addresses ---
ADDR_CW_MARGIN = 26
ADDR_CCW_MARGIN = 27
ADDR_CW_SLOPE  = 28
ADDR_CCW_SLOPE = 29

CW_MARGIN  = 1    # Minimum, typically 1-3
CCW_MARGIN = 1
CW_SLOPE   = 32   # 1-254, higher = stiffer
CCW_SLOPE  = 32

# Set the desired latency timer value (in milliseconds)
LATENCY_TIMER = 1  # 1 ms

JOINT_IDS = list(range(1, 19))  # IDs of all 18 motors

portHandler = PortHandler('/dev/ttyUSB0')  # your USB port
packetHandler = PacketHandler(1)           # protocol 1.0

def main():
    # your code here
    print("Setting USB latency and compliance values")

    # Set the latency timer
    portHandler.setPortOption('LATENCY_TIMER', LATENCY_TIMER)

    if portHandler.openPort() and portHandler.setBaudRate(1000000):
        print("Port opened successfully")

        for dxl_id in JOINT_IDS:
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_CW_COMPLIANCE_MARGIN, CW_MARGIN)
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_CCW_COMPLIANCE_MARGIN, CCW_MARGIN)
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_CW_COMPLIANCE_SLOPE, CW_SLOPE)
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_CCW_COMPLIANCE_SLOPE, CCW_SLOPE)
            print("Compliance settings initialized for all joints")
    else:
        print("Failed to open port")

