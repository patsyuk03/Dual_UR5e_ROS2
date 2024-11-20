import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os
import signal

JSON_PATH_0 = os.path.join(os.path.expanduser("~"), "joint_states_arm_0.json")
JSON_PATH_1 = os.path.join(os.path.expanduser("~"), "joint_states_arm_1.json")


class RecordPath(Node):
    def __init__(self):
        super().__init__('record_path')
        self.subscription_arm_0 = self.create_subscription(JointState, '/arm_0/joint_states', self.callback_arm_0, 10)
        self.subscription_arm_1 = self.create_subscription(JointState, '/arm_1/joint_states', self.callback_arm_1, 10)
        signal.signal(signal.SIGINT, self.shutdown_handler)

        self.recorded_data_arm_0 = list()
        self.recorded_data_arm_1 = list()

    def callback_arm_0(self, msg):
        if len(self.recorded_data_arm_1) == len(self.recorded_data_arm_0):
            data_add = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort)
            }
            self.recorded_data_arm_0.append(data_add)

    def callback_arm_1(self, msg):
        if len(self.recorded_data_arm_0)-len(self.recorded_data_arm_1) == 1:
            data_add = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort)
            }
            self.recorded_data_arm_1.append(data_add)

    def shutdown_handler(self, signum, frame):
        with open(JSON_PATH_0, "w") as f:
            json.dump(self.recorded_data_arm_0, f, indent=4)
        self.get_logger().info(f"Data written to {JSON_PATH_0} successfully.")
        with open(JSON_PATH_1, "w") as f:
            json.dump(self.recorded_data_arm_1, f, indent=4)
        self.get_logger().info(f"Data written to {JSON_PATH_1} successfully.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    recorder = RecordPath()
    rclpy.spin(recorder)


if __name__ == '__main__':
    main()