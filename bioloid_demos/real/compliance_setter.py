#!/usr/bin/env python3
"""
Standalone script to set compliance values before starting ROS2 controllers.
Run this BEFORE starting ros2_control manager.
"""

import argparse
import os
import sys
from dynamixel_sdk import PortHandler, PacketHandler


class ComplianceSetter:
    def __init__(self):
        # Dynamixel settings
        self.ADDR_CW_SLOPE = 28
        self.ADDR_CCW_SLOPE = 29
        self.JOINT_IDS = list(range(1, 19))  # IDs of all 18 motors
        
        # Initialize Dynamixel communication
        self.portHandler = None
        self.packetHandler = None

    def initialize_dynamixel(self, port='/dev/ttyUSB0', baudrate=1000000):
        """Initialize Dynamixel communication."""
        try:
            self.portHandler = PortHandler(port)
            self.packetHandler = PacketHandler(1)  # Protocol 1.0
            
            if self.portHandler.openPort() and self.portHandler.setBaudRate(baudrate):
                print(f"? Dynamixel port opened successfully: {port}")
                return True
            else:
                print(f"? Failed to open Dynamixel port: {port}")
                return False
                
        except Exception as e:
            print(f"? Error initializing Dynamixel: {e}")
            return False

    def close_dynamixel(self):
        """Close Dynamixel communication."""
        if self.portHandler:
            self.portHandler.closePort()
            print("? Dynamixel port closed")

    def test_servo_communication(self, servo_id):
        """Test if we can communicate with a specific servo."""
        try:
            model_number, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, 0
            )
            
            if dxl_comm_result == 0 and dxl_error == 0:
                return True, model_number
            else:
                return False, f"Comm result: {dxl_comm_result}, Error: {dxl_error}"
                
        except Exception as e:
            return False, str(e)

    def set_compliance_values(self, compliance_values):
        """Set compliance values for all servos."""
        if len(compliance_values) < 19:
            print(f"? Compliance array too short ({len(compliance_values)}), need at least 19 elements")
            return False

        # Skip index 0 (dummy), use indices [1:19] for servo IDs 1-18
        compliance_servo_values = compliance_values[1:19]
        
        print(f"Setting compliance for {len(compliance_servo_values)} servos...")
        print(f"Compliance values: {compliance_servo_values}")

        success_count = 0
        for i, compliance_power in enumerate(compliance_servo_values):
            dxl_id = self.JOINT_IDS[i]  # IDs 1-18
            
            # Calculate slope value: 2^compliance_power
            slope_value = 2 ** compliance_power
            
            # Ensure value is within valid range (1-254)
            slope_value = max(1, min(254, slope_value))
            
            try:
                # Write to CW_SLOPE register
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_CW_SLOPE, slope_value
                )
                
                if dxl_comm_result != 0 or dxl_error != 0:
                    print(f"? Failed to write CW_SLOPE for ID {dxl_id}")
                    continue
                
                # Write to CCW_SLOPE register
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_CCW_SLOPE, slope_value
                )
                
                if dxl_comm_result != 0 or dxl_error != 0:
                    print(f"? Failed to write CCW_SLOPE for ID {dxl_id}")
                    continue
                
                success_count += 1
                print(f"? ID {dxl_id:2d}: 2^{compliance_power} = {slope_value:3d}")
                
            except Exception as e:
                print(f"? Exception setting compliance for ID {dxl_id}: {e}")
                continue

        print(f"\nResult: {success_count}/{len(self.JOINT_IDS)} servos configured successfully")
        return success_count == len(self.JOINT_IDS)

    def load_compliance_from_file(self, file_path):
        """Load compliance values from a motion file."""
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()
            
            for line in lines:
                line = line.strip()
                if line.startswith("compliance="):
                    compliance_str = line.split("=", 1)[1]
                    compliance_values = [int(x) for x in compliance_str.split()]
                    print(f"? Loaded compliance from {file_path}")
                    return compliance_values
            
            print(f"? No compliance= line found in {file_path}")
            return None
            
        except Exception as e:
            print(f"? Error loading file {file_path}: {e}")
            return None


def read_motion_list_file(list_file_path):
    """
    Read a list file containing motion filenames.
    Returns a list of motion filenames.
    """
    motion_files = []
    
    try:
        with open(list_file_path, 'r') as file:
            lines = file.readlines()
        
        for line_num, line in enumerate(lines, 1):
            line = line.strip()
            
            # Skip empty lines and comments (lines starting with #)
            if not line or line.startswith('#'):
                continue
            
            # Check if line contains a valid motion filename
            if not line.endswith('.mtn'):
                print(f"Warning: Line {line_num} doesn't end with '.mtn': {line}")
                print(f"  Adding '.mtn' extension automatically")
                line = line + '.mtn'
            
            motion_files.append(line)
            print(f"  {len(motion_files):2d}: {line}")
    
    except FileNotFoundError:
        raise FileNotFoundError(f"List file not found: {list_file_path}")
    except Exception as e:
        raise Exception(f"Error reading list file: {e}")
    
    return motion_files


def main():
    parser = argparse.ArgumentParser(description='Set Dynamixel compliance values before starting ROS2 controllers')
    parser.add_argument('filename', nargs='?', help='Motion filename (e.g., bow.mtn) or list file (e.g., sequence.txt)')
    parser.add_argument('--path', 
                       default='/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions',
                       help='Base path to motion files directory')
    parser.add_argument('--list', action='store_true', help='Treat input file as a list file')
    parser.add_argument('--values', help='Comma-separated compliance values (e.g., "0,5,5,6,6,4,4...")')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Dynamixel port')
    parser.add_argument('--baudrate', type=int, default=1000000, help='Dynamixel baudrate')
    parser.add_argument('--test', action='store_true', help='Test communication with servos')
    
    args = parser.parse_args()
    
    # Create compliance setter
    setter = ComplianceSetter()
    
    # Initialize Dynamixel
    if not setter.initialize_dynamixel(args.port, args.baudrate):
        print("Failed to initialize Dynamixel communication")
        sys.exit(1)
    
    try:
        # Test communication if requested
        if args.test:
            print("\nTesting servo communication...")
            for test_id in [1, 9, 18]:
                success, result = setter.test_servo_communication(test_id)
                if success:
                    print(f"? Servo ID {test_id}: Model {result}")
                else:
                    print(f"? Servo ID {test_id}: {result}")
            print()
        
        # Get compliance values
        compliance_values = None
        
        if args.values:
            # Parse from command line (highest priority)
            try:
                compliance_values = [int(x.strip()) for x in args.values.split(',')]
                print(f"? Using command line compliance values: {compliance_values}")
            except ValueError as e:
                print(f"? Error parsing compliance values: {e}")
                sys.exit(1)
        elif args.filename:
            # Determine file path
            input_file_path = args.filename
            if not os.path.isabs(input_file_path):
                # If relative path, check in motion directory first, then current directory
                motion_dir_path = os.path.join(args.path, input_file_path)
                current_dir_path = input_file_path
                
                if os.path.isfile(motion_dir_path):
                    input_file_path = motion_dir_path
                elif os.path.isfile(current_dir_path):
                    input_file_path = current_dir_path
                else:
                    print(f"? Input file not found: {input_file_path}")
                    print(f"Searched in:")
                    print(f"  Motion directory: {motion_dir_path}")
                    print(f"  Current directory: {current_dir_path}")
                    sys.exit(1)
            
            # Check if input file exists
            if not os.path.isfile(input_file_path):
                print(f"? Input file not found: {input_file_path}")
                sys.exit(1)
            
            # Determine if it's a list file or motion file
            is_list_file = args.list or not input_file_path.endswith('.mtn')
            
            if is_list_file:
                print(f"?? List mode: Reading motion list from {input_file_path}")
                try:
                    motion_files = read_motion_list_file(input_file_path)
                    print(f"Found {len(motion_files)} motion files in list")
                    
                    if not motion_files:
                        print("? No motion files found in list")
                        sys.exit(1)
                    
                    # Use the first motion file for compliance
                    first_motion_file = motion_files[0]
                    motion_file_path = os.path.join(args.path, first_motion_file)
                    
                    if not os.path.isfile(motion_file_path):
                        print(f"? First motion file not found: {motion_file_path}")
                        sys.exit(1)
                    
                    print(f"Using compliance from first motion file: {first_motion_file}")
                    compliance_values = setter.load_compliance_from_file(motion_file_path)
                    
                except Exception as e:
                    print(f"? Error processing list file: {e}")
                    sys.exit(1)
            else:
                # Single motion file mode
                print(f"?? Single motion mode: {input_file_path}")
                compliance_values = setter.load_compliance_from_file(input_file_path)
        else:
            # Use default values (power of 5 = 32 for all servos)
            compliance_values = [0] + [5] * 18  # dummy + 18 servos
            print("? Using default compliance values (all servos: 2^5 = 32)")
        
        if compliance_values is None:
            print("? No compliance values found or specified")
            sys.exit(1)
        
        # Set compliance values
        print(f"\nSetting compliance values: {compliance_values}")
        success = setter.set_compliance_values(compliance_values)
        
        if success:
            print("\n?? All compliance values set successfully!")
            print("You can now start the ROS2 controller manager.")
        else:
            print("\n??  Some compliance values failed to set.")
            print("Check servo connections and try again.")
            sys.exit(1)
    
    finally:
        setter.close_dynamixel()


if __name__ == '__main__':
    main()
