import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os
import signal

JSON_PATH = os.path.join(os.path.expanduser("~"), "joint_states.json")

class RecordPath(Node):
    def __init__(self):
        super().__init__('record_path')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        signal.signal(signal.SIGINT, self.shutdown_handler)

        self.recorded_data = list()

    def callback(self, msg):
        data_add = {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort)
        }
        self.recorded_data.append(data_add)

    def shutdown_handler(self, signum, frame):
        with open(JSON_PATH, "w") as f:
            json.dump(self.recorded_data, f, indent=4)
        self.get_logger().info(f"Data written to {JSON_PATH} successfully.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    recorder = RecordPath()
    rclpy.spin(recorder)


if __name__ == '__main__':
    main()