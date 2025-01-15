import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import tf_transformations

class BagToCsvNode(Node):
    def __init__(self):
        super().__init__('bag_to_csv_node')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/pf/viz/inferred_pose',  
            self.pose_callback,
            10
        )
        
        self.file_name = 'pose_data.csv'
        
        try:
            with open(self.file_name, mode='x', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'X', 'Y', 'Angle (Theta)'])
        except FileExistsError:
            pass  

    def pose_callback(self, msg: PoseStamped):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 
        x = msg.pose.position.x
        y = msg.pose.position.y
        angle = self.get_angle_from_quaternion(msg.pose.orientation)
        
        self.get_logger().info(f"Timestamp: {timestamp}, X: {x}, Y: {y}, Angle: {angle}")

        with open(self.file_name, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, x, y, angle])

    def get_angle_from_quaternion(self, quaternion):
        euler = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler[2]  

def main(args=None):
    rclpy.init(args=args)

    node = BagToCsvNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
