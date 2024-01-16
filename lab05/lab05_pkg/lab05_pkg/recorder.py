import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import message_filters

import numpy as np
import math

import matplotlib.pyplot as plt



class RecorderNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Initializing my_node")

        # Create the subscribers and synchronize the messages
        self.odom_sub = message_filters.Subscriber(self, Odometry, "/diff_drive_controller/odom")
        self.filter_sub = message_filters.Subscriber(self, Odometry, "/odometry/filtered")
        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, "/ground_truth")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.filter_sub, self.ground_truth_sub], 10, 0.01
        )
        self.ts.registerCallback(self.store_data)

        self.odom = []
        self.filter = []
        self.ground_truth = []

    def yaw_calc(self, quaternion):
    # Convert quaternion to yaw (rotation around the Z-axis)
        t3 = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        t4 = 1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
        yaw = math.atan2(t3, t4)
        return yaw

    def store_data(self, odom, filter, ground_truth):
        self.get_logger().info(
            f"Storing data... Current lenght {len(self.odom)}", throttle_duration_sec=5.0
        )
        self.odom.append([odom.pose.pose.position.x, odom.pose.pose.position.y, self.yaw_calc(odom.pose.pose.orientation)])
        self.filter.append([filter.pose.pose.position.x, filter.pose.pose.position.y, self.yaw_calc(filter.pose.pose.orientation)])
        self.ground_truth.append([ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, self.yaw_calc(ground_truth.pose.pose.orientation)])

    def save_data(self):
        np.save("odom.npy", np.array(self.odom))
        np.save("filter.npy", np.array(self.filter))
        np.save("ground_truth.npy", np.array(self.ground_truth))



def metrics(data):
    mean = np.mean(data, axis = 0)
    std = np.std(data, axis = 0)
    return mean, std



def plot_data(data, title):
    plt.figure(figsize=(10, 5))
    time = np.arange(len(data))

    plt.subplot(3, 1, 1)
    plt.plot(time, data[:, 0], label='X')
    plt.title(f'{title} - X')
    plt.xlabel('Time')
    plt.ylabel('X')

    plt.subplot(3, 1, 2)
    plt.plot(time, data[:, 1], label='Y')
    plt.title(f'{title} - Y')
    plt.xlabel('Time')
    plt.ylabel('Y')

    plt.subplot(3, 1, 3)
    plt.plot(time, data[:, 2], label='Yaw')
    plt.title(f'{title} - Yaw')
    plt.xlabel('Time')
    plt.ylabel('Yaw')

    plt.tight_layout()
    plt.show()
    
def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()

        odom_data = np.load("odom.npy")
        filter_data = np.load("filter.npy")
        ground_truth_data = np.load("ground_truth.npy")

        print("Loaded Odom Data:")
        print(odom_data)

        print("Loaded Filter Data:")
        print(filter_data)

        print("Loaded Ground Truth Data:")
        print(ground_truth_data)

        odom_metrics = metrics(odom_data)
        filter_metrics = metrics(filter_data)
        ground_truth_metrics = metrics(ground_truth_data)

        print("odom_metrics:", odom_metrics)
        print("filter_metrics:", filter_metrics)
        print("ground_truth_metrics:", ground_truth_metrics)
        # Visualize the data
        plot_data(odom_data, 'Odom Data')
        plot_data(filter_data, 'Filter Data')
        plot_data(ground_truth_data, 'Ground Truth Data')



        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
