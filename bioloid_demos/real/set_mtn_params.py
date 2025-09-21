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
        self.ADDR_MOVING_SPEED = 32
        self.ADDR_TORQUE_LIMIT = 34
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

    def read_current_values(self, servo_ids=None):
        """
        Read and display current values from servos for debugging.
        """
        if servo_ids is None:
            servo_ids = self.JOINT_IDS[:6]  # Test first 6 servos
        
        print("?? Current servo register values:")
        print("ID | CW_SLOPE | CCW_SLOPE | MOVING_SPEED | TORQUE_LIMIT")
        print("---|----------|-----------|--------------|-------------")
        
        for dxl_id in servo_ids:
            try:
                # Read CW_SLOPE (addr 28)
                cw_slope, comm1, err1 = self.packetHandler.read1ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_CW_SLOPE
                )
                
                # Read CCW_SLOPE (addr 29) 
                ccw_slope, comm2, err2 = self.packetHandler.read1ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_CCW_SLOPE
                )
                
                # Read MOVING_SPEED (addr 32)
                moving_speed, comm3, err3 = self.packetHandler.read2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MOVING_SPEED
                )
                
                # Read TORQUE_LIMIT (addr 34)
                torque_limit, comm4, err4 = self.packetHandler.read2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_TORQUE_LIMIT
                )
                
                # Check for errors
                if any([comm1, comm2, comm3, comm4]) or any([err1, err2, err3, err4]):
                    print(f"{dxl_id:2d} | ERROR    | ERROR     | ERROR        | ERROR")
                else:
                    print(f"{dxl_id:2d} | {cw_slope:8d} | {ccw_slope:9d} | {moving_speed:12d} | {torque_limit:11d}")
                
            except Exception as e:
                print(f"{dxl_id:2d} | EXCEPTION: {e}")
                
        print()

    def set_play_param_values(self, play_param, speed_only=False, use_baseline=True):
        """
        Set play parameter values for all servos.
        play_param format: [param1, param2, param3, speed_factor, torque_factor]
        - speed_factor (4th param): multiply original MOVING_SPEED value
        - torque_factor (5th param): multiply by 8 and set as TORQUE_LIMIT
        """
        if len(play_param) < 5:
            print(f"??  Play param array too short ({len(play_param)}), need at least 5 elements")
            print("Using default values for missing parameters")
            # Pad with defaults if needed
            while len(play_param) < 5:
                play_param.append(1.0 if len(play_param) == 3 else 32)  # default speed=1.0, torque=32
        
        speed_factor = float(play_param[3])
        torque_factor = int(play_param[4])
        baseline_speed = 100  # Default speed for servos with 0 speed
        
        print(f"Play parameters: {play_param}")
        print(f"Speed factor: {speed_factor} (multiply original MOVING_SPEED)")
        if not speed_only:
            print(f"Torque factor: {torque_factor} (multiply by 8 ? {torque_factor * 8} for TORQUE_LIMIT)")
        else:
            print("?? Torque changes disabled (speed-only mode)")
        
        if use_baseline:
            print(f"Baseline speed: {baseline_speed} (used for servos with 0 speed)")
        else:
            print("?? Baseline speed disabled (keeping original 0 speeds)")
        
        # Read current values before changes
        print("\n?? BEFORE changes:")
        self.read_current_values([1, 2, 3, 4, 5, 6])
        
        success_count = 0
        zero_speed_servos = []
        
        for dxl_id in self.JOINT_IDS:
            try:
                # Read original MOVING_SPEED value
                original_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MOVING_SPEED
                )
                
                if dxl_comm_result != 0 or dxl_error != 0:
                    print(f"? Failed to read MOVING_SPEED for ID {dxl_id}")
                    continue
                
                # Handle servos with 0 speed
                if original_speed == 0:
                    zero_speed_servos.append(dxl_id)
                    if use_baseline:
                        # Use baseline speed instead of 0
                        effective_speed = baseline_speed
                        print(f"??  ID {dxl_id} has 0 speed, using baseline {baseline_speed}")
                    else:
                        # Keep original 0 speed
                        effective_speed = original_speed
                        print(f"?? ID {dxl_id} has 0 speed, keeping as 0")
                else:
                    effective_speed = original_speed
                
                # Calculate new speed value
                new_speed = int(effective_speed * speed_factor)
                new_speed = max(0, min(1023, new_speed))  # Clamp to valid range
                
                # Write new MOVING_SPEED value
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MOVING_SPEED, new_speed
                )
                
                if dxl_comm_result != 0 or dxl_error != 0:
                    print(f"? Failed to write MOVING_SPEED for ID {dxl_id}: comm={dxl_comm_result}, err={dxl_error}")
                    continue
                
                # Handle torque limit (only if not speed_only mode)
                if not speed_only:
                    # Calculate torque limit value
                    torque_limit = torque_factor * 8
                    torque_limit = max(0, min(1023, torque_limit))  # Clamp to valid range
                    
                    # Write TORQUE_LIMIT value
                    #dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                    #    self.portHandler, dxl_id, self.ADDR_TORQUE_LIMIT, torque_limit
                    #)
                    
                    #if dxl_comm_result != 0 or dxl_error != 0:
                    #    print(f"? Failed to write TORQUE_LIMIT for ID {dxl_id}: comm={dxl_comm_result}, err={dxl_error}")
                    #    continue
                
                success_count += 1
                
                # Show appropriate output format
                if speed_only:
                    if original_speed == 0 and use_baseline:
                        print(f"? ID {dxl_id:2d}: Speed 0({baseline_speed})?{new_speed}")
                    else:
                        print(f"? ID {dxl_id:2d}: Speed {original_speed}?{new_speed}")
                else:
                    torque_limit = torque_factor * 8
                    if original_speed == 0 and use_baseline:
                        print(f"? ID {dxl_id:2d}: Speed 0({baseline_speed})?{new_speed}, Torque?{torque_limit}")
                    else:
                        print(f"? ID {dxl_id:2d}: Speed {original_speed}?{new_speed}, Torque?{torque_limit}")
                
            except Exception as e:
                print(f"? Exception setting play params for ID {dxl_id}: {e}")
                continue
        
        # Read current values after changes
        print("\n?? AFTER changes:")
        self.read_current_values([1, 2, 3, 4, 5, 6])
        
        if zero_speed_servos:
            print(f"\n??  Found {len(zero_speed_servos)} servo(s) with 0 speed: {zero_speed_servos}")
            if use_baseline:
                print(f"These servos used baseline speed {baseline_speed} for calculation")
            else:
                print("These servos kept their original 0 speed")
        
        mode_desc = "speed-only" if speed_only else "speed and torque"
        print(f"\nResult: {success_count}/{len(self.JOINT_IDS)} servos configured for {mode_desc} parameters")
        return success_count == len(self.JOINT_IDS)

    def load_compliance_from_file(self, file_path):
        """Load compliance values and play parameters from a motion file."""
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()
            
            compliance_values = None
            play_param = None
            
            for line in lines:
                line = line.strip()
                
                if line.startswith("compliance="):
                    compliance_str = line.split("=", 1)[1]
                    compliance_values = [int(x) for x in compliance_str.split()]
                    print(f"?? Loaded compliance from {file_path}: {compliance_values}")
                
                elif line.startswith("play_param="):
                    play_param_str = line.split("=", 1)[1]
                    # Convert to appropriate types (mixed int/float)
                    params = play_param_str.split()
                    play_param = []
                    for param in params:
                        try:
                            # Try integer first
                            play_param.append(int(param))
                        except ValueError:
                            # If not integer, use float
                            play_param.append(float(param))
                    print(f"?? Loaded play_param from {file_path}: {play_param}")
            
            return compliance_values, play_param
            
        except Exception as e:
            print(f"? Error loading file {file_path}: {e}")
            return None, None


def read_motion_list_file(list_file_path):
    """
    Read a list file containing motion filenames.
    Supports both .txt and .lst extensions.
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


def is_list_file(file_path):
    """
    Determine if a file is a list file based on its extension.
    Supports .lst, .txt extensions and auto-detection.
    """
    # Check file extension
    if file_path.lower().endswith(('.lst', '.txt')):
        return True
    
    # If it doesn't end with .mtn, it's likely a list file
    if not file_path.lower().endswith('.mtn'):
        return True
    
    return False


def find_file_in_paths(filename, search_paths):
    """
    Search for a file in multiple paths and return the first match.
    Args:
        filename: Name of the file to search for
        search_paths: List of paths to search in
    Returns:
        Full path to the file if found, None otherwise
    """
    for path in search_paths:
        full_path = os.path.join(path, filename)
        if os.path.isfile(full_path):
            return full_path
    return None


def main():
    parser = argparse.ArgumentParser(description='Set Dynamixel compliance values before starting ROS2 controllers')
    parser.add_argument('filename', nargs='?', help='Motion filename (e.g., bow.mtn) or list file (e.g., sequence.lst, sequence.txt)')
    parser.add_argument('--path', 
                       default='/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions',
                       help='Base path to motion files directory')
    parser.add_argument('--list', action='store_true', help='Force treat input file as a list file (auto-detected for .lst/.txt files)')
    parser.add_argument('--values', help='Comma-separated compliance values (e.g., "0,5,5,6,6,4,4...")')
    parser.add_argument('--play-param', help='Comma-separated play param values (e.g., "0,0,1,1.0,32")')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Dynamixel port')
    parser.add_argument('--baudrate', type=int, default=1000000, help='Dynamixel baudrate')
    parser.add_argument('--test', action='store_true', help='Test communication with servos')
    parser.add_argument('--compliance-only', action='store_true', help='Set only compliance values, skip play parameters')
    parser.add_argument('--play-param-only', action='store_true', help='Set only play parameters, skip compliance values')
    parser.add_argument('--speed-only', action='store_true', help='Set only speed, skip torque changes')
    parser.add_argument('--no-baseline', action='store_true', help='Do not use baseline speed for zero-speed servos')
    
    args = parser.parse_args()
    
    print("=== DYNAMIXEL COMPLIANCE SETTER ===")
    print("Supports: .mtn files, .lst files, .txt files")
    print("=" * 40)
    
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
        
        # Get compliance values and play parameters
        compliance_values = None
        play_param = None
        
        if args.values or args.play_param:
            # Parse from command line (highest priority)
            if args.values:
                try:
                    compliance_values = [int(x.strip()) for x in args.values.split(',')]
                    print(f"?? Using command line compliance values: {compliance_values}")
                except ValueError as e:
                    print(f"? Error parsing compliance values: {e}")
                    sys.exit(1)
            
            if args.play_param:
                try:
                    play_param_strs = args.play_param.split(',')
                    play_param = []
                    for param_str in play_param_strs:
                        param_str = param_str.strip()
                        try:
                            play_param.append(int(param_str))
                        except ValueError:
                            play_param.append(float(param_str))
                    print(f"?? Using command line play parameters: {play_param}")
                except ValueError as e:
                    print(f"? Error parsing play parameters: {e}")
                    sys.exit(1)
                    
        elif args.filename:
            # Search for input file in multiple locations
            search_paths = [
                args.path,           # Motion directory (first priority)
                os.getcwd(),         # Current directory
                os.path.dirname(os.path.abspath(__file__))  # Script directory
            ]
            
            input_file_path = find_file_in_paths(args.filename, search_paths)
            
            if input_file_path is None:
                print(f"? Input file '{args.filename}' not found in any of these locations:")
                for i, path in enumerate(search_paths, 1):
                    full_path = os.path.join(path, args.filename)
                    print(f"  {i}. {full_path}")
                sys.exit(1)
            
            print(f"Found input file: {input_file_path}")
            
            # Determine if it's a list file or motion file
            is_list_mode = args.list or is_list_file(input_file_path)
            
            if is_list_mode:
                file_ext = os.path.splitext(input_file_path)[1].lower()
                print(f"?? List mode: Reading motion list from {input_file_path} ({file_ext} file)")
                try:
                    motion_files = read_motion_list_file(input_file_path)
                    print(f"Found {len(motion_files)} motion files in list")
                    
                    if not motion_files:
                        print("? No motion files found in list")
                        sys.exit(1)
                    
                    # Use the first motion file for compliance and play params
                    first_motion_file = motion_files[0]
                    motion_file_path = os.path.join(args.path, first_motion_file)
                    
                    if not os.path.isfile(motion_file_path):
                        print(f"? First motion file not found: {motion_file_path}")
                        sys.exit(1)
                    
                    print(f"Using parameters from first motion file: {first_motion_file}")
                    compliance_values, play_param = setter.load_compliance_from_file(motion_file_path)
                    
                except Exception as e:
                    print(f"? Error processing list file: {e}")
                    sys.exit(1)
            else:
                # Single motion file mode
                print(f"?? Single motion mode: {input_file_path}")
                compliance_values, play_param = setter.load_compliance_from_file(input_file_path)
        
        # Use defaults if nothing specified
        if compliance_values is None and play_param is None:
            compliance_values = [0] + [5] * 18  # dummy + 18 servos
            play_param = [0, 0, 1, 1.0, 32]  # default play parameters
            print("?? Using default values:")
            print(f"  Compliance: all servos 2^5 = 32")
            print(f"  Play param: {play_param}")
        
        # Set parameters based on flags
        compliance_success = True
        play_param_success = True
        
        if not args.play_param_only and compliance_values is not None:
            print(f"\n?? Setting compliance values: {compliance_values}")
            compliance_success = setter.set_compliance_values(compliance_values)
        
        if not args.compliance_only and play_param is not None:
            print(f"\n? Setting play parameters: {play_param}")
            play_param_success = setter.set_play_param_values(
                play_param, 
                speed_only=args.speed_only, 
                use_baseline=not args.no_baseline
            )
        
        # Overall success check
        if compliance_success and play_param_success:
            print("\n?? All servo parameters set successfully!")
            print("You can now start the ROS2 controller manager.")
        else:
            failed_parts = []
            if not compliance_success:
                failed_parts.append("compliance")
            if not play_param_success:
                failed_parts.append("play parameters")
            
            print(f"\n? Failed to set: {', '.join(failed_parts)}")
            print("Check servo connections and try again.")
            sys.exit(1)
    
    finally:
        setter.close_dynamixel()


if __name__ == '__main__':
    main()
