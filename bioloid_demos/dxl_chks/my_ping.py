#!/usr/bin/env python3

from dynamixel_sdk import *

def test_servo_communication():
    # Port settings
    DEVICENAME = '/dev/ttyUSB0'
    BAUDRATE = 1000000

    # Initialize port
    portHandler = PortHandler(DEVICENAME)

    if not portHandler.openPort():
        print("Failed to open the port")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set the baudrate")
        return

    print(f"Port opened successfully: {DEVICENAME}")
    print(f"Baudrate set to: {BAUDRATE}")
    print("-" * 40)

    # Test with Protocol 1.0
    print("Testing with Protocol 1.0:")
    packetHandler1 = PacketHandler(1.0)

    for servo_id in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]:
        try:
            # Modern SDK returns (model_number, comm_result, error) for both protocols
            result = packetHandler1.ping(portHandler, servo_id)
            if isinstance(result, tuple):
                model_number, dxl_comm_result, dxl_error = result
                if dxl_comm_result == COMM_SUCCESS:
                    print(f"  Servo ID {servo_id}: SUCCESS (Model: {model_number})")
                else:
                    comm_error_names = {
                        1: "COMM_PORT_BUSY", 2: "COMM_TX_FAIL", 3: "COMM_RX_FAIL",
                        4: "COMM_TX_ERROR", 5: "COMM_RX_WAITING", 6: "COMM_RX_TIMEOUT",
                        7: "COMM_RX_CORRUPT", 8: "COMM_NOT_AVAILABLE", 12: "COMM_RX_TIMEOUT"
                    }
                    error_name = comm_error_names.get(dxl_comm_result, f"UNKNOWN_ERROR_{dxl_comm_result}")
                    print(f"  Servo ID {servo_id}: FAILED ({error_name})")
            else:
                if result == COMM_SUCCESS:
                    print(f"  Servo ID {servo_id}: SUCCESS")
                else:
                    print(f"  Servo ID {servo_id}: FAILED (comm_result: {result})")
        except Exception as e:
            print(f"  Servo ID {servo_id}: ERROR - {e}")

    print("-" * 40)

    # Close port
    portHandler.closePort()
    print("-" * 40)
    print("Port closed")

if __name__ == "__main__":
    test_servo_communication()
