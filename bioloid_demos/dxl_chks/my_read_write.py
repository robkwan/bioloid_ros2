#!/usr/bin/env python3

from dynamixel_sdk import *
import time

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(1.0)

if portHandler.openPort():
  print("Succeeded to open the port!")
else:
  print("Failed to open the port!")
  exit()

if portHandler.setBaudRate(1000000):
  print("Succeeded to change the baudrate!")
else:
  print("Failed to change the baudrate!")
  exit()
  
# Set packet timeout (important for AX-12A)
portHandler.setPacketTimeout(1000)  # 1000ms timeout

dxl_id = 1

torque_on_address = 24
data = 0
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)
time.sleep(0.1)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

status, _, _ = packetHandler.read1ByteTxRx(portHandler, dxl_id, 46)  # Status register
print(f"Status register: {bin(status)}")
print(f"Status register: {status}")

# Read load value
load, _, _ = packetHandler.read2ByteTxRx(portHandler, dxl_id, 40)  # Present Load
print(f"Current load: {load}")

# Check current limits
cw_limit, _, _ = packetHandler.read2ByteTxRx(portHandler, dxl_id, 6)
ccw_limit, _, _ = packetHandler.read2ByteTxRx(portHandler, dxl_id, 8)
print(f"Current limits - CW: {cw_limit}, CCW: {ccw_limit}")

while True:
    try:
        target_position = int(input("Enter target position (0 ~ 1023, -1 to exit): "))
    except ValueError:
        print("Please enter an integer.")
        continue

    if target_position == -1:
        break
    elif target_position < 0 or target_position > 1023:
        print("Position must be between 0 and 1023.")
        continue

    goal_position_address = 30
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, goal_position_address, target_position)
    time.sleep(0.1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    #while True:
    #    present_position_address = 36
    #    present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, present_position_address)
    #    time.sleep(0.1)
    #    if dxl_comm_result != COMM_SUCCESS:
    #        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #    elif dxl_error != 0:
    #        print("%s" % packetHandler.getRxPacketError(dxl_error))
    #    print(f"Current Position: {present_position}")

    #    if abs(target_position - present_position) <= 10:
    #        break

data = 0
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)
portHandler.closePort()



