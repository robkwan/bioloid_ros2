#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState

class JointFeedbackTest(Node):
    def __init__(self):
        super().__init__('joint_feedback_test')
        
        self.joint_state_received = False
        self.dynamic_joint_state_received = False
        self.joint_positions = {}
        
        # Subscribe to both topics
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.dynamic_joint_state_sub = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_state_callback,
            10
        )
        
        print("Subscribed to /joint_states and /dynamic_joint_states")
        print("Waiting for messages...")
        
    def joint_state_callback(self, msg):
        if not self.joint_state_received:
            print(f"\n? /joint_states callback received!")
            print(f"  Joint count: {len(msg.name)}")
            print(f"  First 5 joints: {msg.name[:5]}")
            
        self.joint_state_received = True
        
        # Store positions
        for i, joint_name in enumerate(msg.name):
            self.joint_positions[joint_name] = msg.position[i]
    
    def dynamic_joint_state_callback(self, msg):
        if not self.dynamic_joint_state_received:
            print(f"\n? /dynamic_joint_states callback received!")
            print(f"  Joint count: {len(msg.joint_names)}")
            print(f"  First 5 joints: {msg.joint_names[:5]}")
            
        self.dynamic_joint_state_received = True
        
        # Store positions from dynamic joint states
        for i, joint_name in enumerate(msg.joint_names):
            try:
                interface_values = msg.interface_values[i]
                if 'position' in interface_values.interface_names:
                    pos_index = interface_values.interface_names.index('position')
                    self.joint_positions[joint_name + "_dynamic"] = interface_values.values[pos_index]
            except Exception as e:
                print(f"Error parsing {joint_name}: {e}")
    
    def run_test(self):
        print("Running 10-second test...")
        
        import time
        start_time = time.time()
        
        while (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Print status every 2 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0 and int(elapsed) != int(elapsed - 0.1):
                print(f"[{elapsed:.1f}s] Standard: {'?' if self.joint_state_received else '?'}, "
                      f"Dynamic: {'?' if self.dynamic_joint_state_received else '?'}, "
                      f"Positions: {len(self.joint_positions)}")
        
        print("\nTest Results:")
        print(f"  /joint_states received: {'?' if self.joint_state_received else '?'}")
        print(f"  /dynamic_joint_states received: {'?' if self.dynamic_joint_state_received else '?'}")
        print(f"  Total positions stored: {len(self.joint_positions)}")
        
        if len(self.joint_positions) > 0:
            print("\nSample positions:")
            count = 0
            for name, pos in self.joint_positions.items():
                if count < 5:
                    print(f"  {name}: {pos:.3f} rad")
                    count += 1
        
        return self.joint_state_received or self.dynamic_joint_state_received

def main():
    rclpy.init()
    
    try:
        node = JointFeedbackTest()
        success = node.run_test()
        
        if success:
            print("\n? Joint feedback is working!")
        else:
            print("\n? No joint feedback received")
            print("\nTroubleshooting:")
            print("1. Check if topics exist: ros2 topic list | grep joint")
            print("2. Check if data is being published: ros2 topic hz /joint_states")
            print("3. Check your robot/simulation is running")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
