import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import csv
import os
import math
import sys

class RobotPoseLogger(Node):
    def __init__(self):
        super().__init__('save_path')
  
        self.path_csv = '/home/thanawat/amr_ws/src/follow_path/file_path/default_path.csv'

        self.first_pose_saved = False
        self.last_saved_x = None
        self.last_saved_y = None
        self.last_saved_qx = None
        self.last_saved_qy = None
        self.last_saved_qz = None
        self.last_saved_qw = None

        self.init_csv_file(self.path_csv)
    
        self.status_publisher = self.create_publisher(
            String,
            '/save_status',
            10
        )
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.save_file_subscriber = self.create_subscription(String, '/save_path', self.save_file_callback, 10)

    def publish_status(self, success):
        msg = String()
        msg.data = success
        self.status_publisher.publish(msg)

    def init_csv_file(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'X', 'Y', 'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W'])
        
    def pose_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.qx = msg.pose.pose.orientation.x
        self.qy = msg.pose.pose.orientation.y
        self.qz = msg.pose.pose.orientation.z
        self.qw = msg.pose.pose.orientation.w

        if not self.first_pose_saved:
            self.write_to_csv(self.path_csv, [timestamp, self.x, self.y, self.qx, self.qy, self.qz, self.qw])
            self.first_pose_saved = True
            self.last_saved_x = self.x
            self.last_saved_y = self.y
            self.last_saved_qx = self.qx
            self.last_saved_qy = self.qy
            self.last_saved_qz = self.qz
            self.last_saved_qw = self.qw
            return

        distance = math.sqrt((self.x - self.last_saved_x) ** 2 + (self.y - self.last_saved_y) ** 2)
        if distance >= 1.0:
            self.write_to_csv(self.path_csv, [timestamp, self.x, self.y, self.qx, self.qy, self.qz, self.qw])
            self.last_saved_x = self.x
            self.last_saved_y = self.y
            self.last_saved_qx = self.qx
            self.last_saved_qy = self.qy
            self.last_saved_qz = self.qz
            self.last_saved_qw = self.qw

    def save_file_callback(self, msg):
        data = msg.data.split(',')
        if len(data) != 2 or data[0].strip().lower() != 'true':
            self.publish_status('Invalid save_file message format!')
            return
            
        file_name = data[1].strip()

        if self.last_saved_x is not None:
            timestamp = self.get_clock().now().to_msg().sec
            self.write_to_csv(self.path_csv, [timestamp, self.x, self.y, self.qx, self.qy, self.qz, self.qw])

        os.rename(self.path_csv, f'/home/thanawat/amr_ws/src/follow_path/file_path/{file_name}')
        
        self.publish_status(f'Successfully Saved path as {file_name} End of work!')

        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    def write_to_csv(self, filename, data):
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)
        self.publish_status(f'Saved data')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 topic pub --once /save_file std_msgs/msg/String "data: 'true,path.csv'"