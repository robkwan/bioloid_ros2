import rclpy
from rclpy.node import Node
import subprocess
import threading
import time

class AntiGravityForce(Node):
    def __init__(self):
        super().__init__('anti_gravity_force')
        self.running = True
        self.force_thread = threading.Thread(target=self.apply_force_loop)
        self.force_thread.daemon = True
        self.force_thread.start()
    
    def apply_force_loop(self):
        while self.running:
            cmd = [
                'gz', 'topic', '-t', '/world/default/wrench',
                '-m', 'gz.msgs.EntityWrench',
                '-p', 'entity: {name: "typea", type: MODEL}, wrench: {force: {z: 110.0}}'
            ]
            subprocess.run(cmd, capture_output=True)
            time.sleep(0.01)  # 100Hz
    
    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main():
    rclpy.init()
    node = AntiGravityForce()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
