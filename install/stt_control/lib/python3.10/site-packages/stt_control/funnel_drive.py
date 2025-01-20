import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TransformStamped, Transform, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import tf_transformations
import matplotlib.pyplot as plt
import csv


class DrivePublisherNode(Node):
    def __init__(self):
        super().__init__('funnel_drive')
        self.control_vel_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Publish every 0.01 seconds
        # self.get_logger().info("DrivePublisherNode has started.")
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_callback, 10)

        self.declare_parameter("ulim", 1.0)
        self.declare_parameter("rlim", 0.4)
        self.declare_parameter("rhod_0", 2.0)
        self.declare_parameter("rhoo_0", 2.0)
        self.declare_parameter("u_av", 0.3)
        self.declare_parameter("decay", 0.02)
        self.declare_parameter("rhod_inf", 0.1)
        self.declare_parameter("rhod_lower", -0.1)
        self.declare_parameter("rhoo_inf", 0.01)

        self.ulim = self.get_parameter("ulim").value
        self.rlim = self.get_parameter("rlim").value
        
        self.rhod_0 = self.get_parameter("rhod_0").value
        self.rhoo_0 = self.get_parameter("rhoo_0").value
        
        self.u_av = self.get_parameter("u_av").value

        self.decay = self.get_parameter("decay").value

        self.rhod_inf = self.get_parameter("rhod_inf").value
        self.rhod_lower = self.get_parameter("rhod_lower").value
        self.rhoo_inf = self.get_parameter("rhoo_inf").value
        
        self.stt_val = []
        with open('/home/tripan/f1tenthsim_ws/f1tenth/src/stt_control/tubes/STT.csv', mode='r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                timestamp = float(row[0])
                x_lower = float(row[1])
                x_upper = float(row[2])
                y_lower = float(row[3])
                y_upper = float(row[4])
                
                # Append to lists
                self.stt_val.append([timestamp,x_lower,x_upper,y_lower,y_upper])

        # gamL2 = -tube_width * np.ones(len(time_arr))
        # gamU2 = tube_width * np.ones(len(time_arr))

        self.stt_val = np.array(self.stt_val)

        # self.use_sim_time = self.declare_parameter('use_sim_time', True)
        self.start_time = round(self.get_clock().now().nanoseconds/1e9, 4)
        self.pose = np.array((0,0,0))

        self.x_arr = []
        self.y_arr = []
        self.theta_arr = []
        self.ed_arr = []
        self.eo_arr = []
        self.rhod_arr = []
        self.rhoo_arr = []
        self.cmd_vel_arr = []
        self.steer_arr = []
        self.ref_lower_arr = []
        self.ref_upper_arr = []

    # def publish_vel(self):
    #     drive_msg = AckermannDriveStamped()
    #     drive_msg.header.stamp = self.get_clock().now().to_msg()
    #     drive_msg.header.frame_id = "base_link"

    #     drive_msg.drive.speed = 2.0  # m/s
    #     drive_msg.drive.steering_angle = 0.5  # radians

    #     # Log and publish the message
    #     self.get_logger().info(f"Publishing: speed={drive_msg.drive.speed}, "
    #                            f"steering_angle={drive_msg.drive.steering_angle}")
    #     self.publisher_.publish(drive_msg)

    def ego_odom_callback(self, ego_odom):
        x = ego_odom.pose.pose.position.x
        y = ego_odom.pose.pose.position.y
        orientation = ego_odom.pose.pose.orientation
        theta = tf_transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        
        self.pose = np.array((x,y,theta))
    
    def timer_callback(self):
        x = self.pose[0]
        y = self.pose[1]
        theta = self.pose[2]

        self.x_arr.append(x)
        self.y_arr.append(y)
        self.theta_arr.append(theta)

        t = round(self.get_clock().now().nanoseconds/1e9, 4) - self.start_time
        ref_lower, ref_upper = self.get_ref_pose(t)
        
        # print(ref_lower)
        # print(ref_upper)

        self.ref_lower_arr.append(ref_lower)
        self.ref_upper_arr.append(ref_upper)
        
        print("x ref", (ref_lower[0]+ref_upper[0])/2)
        print("y ref", (ref_lower[1]+ref_upper[1])/2)
        
        ex = ((ref_lower[0]+ref_upper[0])/2 - x)/(ref_upper[0] - ref_lower[0])
        ey = ((ref_lower[1]+ref_upper[1])/2 - y)/(ref_upper[1] - ref_lower[1])

        ed = np.sqrt(ex**2 + ey**2)
        eo = ex*np.sin(theta) - ey*np.cos(theta)

        self.ed_arr.append(ed)
        self.eo_arr.append(eo)

        rhod = self.rhod_inf + (self.rhod_0 - self.rhod_inf) * np.exp(-self.decay*t)
        Xid = 2*(ed - 0.5*(rhod + self.rhod_inf))/(rhod - self.rhod_lower)
        epsd = self.transform(Xid) * self.ulim
        cmd_vel = epsd

        self.rhod_arr.append(rhod)

        rhoo = self.rhoo_inf + (self.rhoo_0 - self.rhoo_inf) * np.exp(-self.decay*t)
        Xio = eo/rhoo
        epsr = self.transform(Xio) * self.rlim
        steer = -epsr

        self.cmd_vel_arr.append(cmd_vel)
        self.steer_arr.append(steer)

        self.rhoo_arr.append(rhoo)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"

        drive_msg.drive.speed = cmd_vel
        drive_msg.drive.steering_angle = steer

        self.get_logger().info(f"cmd_vel={drive_msg.drive.speed}, "
                               f"steer={drive_msg.drive.steering_angle}")
        self.control_vel_pub.publish(drive_msg)


    def transform(self, x):
        a = 2
        return (1-np.exp(-(a*x)**2)) * np.tanh(a*x)


    def get_ref_pose(self,t):
        gamL1 = self.stt_val[:,1]
        gamU1 = self.stt_val[:,2]

        gamL2 = self.stt_val[:,3]
        gamU2 = self.stt_val[:,4]

        time_arr = self.stt_val[:,0]

        tube_lower = np.array((np.interp(t, time_arr, gamL1), np.interp(t, time_arr, gamL2)))
        tube_upper = np.array((np.interp(t, time_arr, gamU1), np.interp(t, time_arr, gamU2)))

        return tube_lower, tube_upper
    
    def plot(self):
        fig, axs = plt.subplots(3, 2, figsize=(12, 10))

# Plot 1: Path of x and y, along with xref and yref
        axs[0, 0].plot(self.x_arr, self.y_arr, label='Path (x, y)', color='b')
        # axs[0, 0].plot((self.stt_val[1] + self.stt_val[2])/2, (self.stt_val[3] + self.stt_val[4])/2, label='Reference Path (xref, yref)', color='r', linestyle='--')
        self.ref_lower_arr = np.array(self.ref_lower_arr)
        self.ref_upper_arr = np.array(self.ref_upper_arr)

        axs[0, 0].plot(self.ref_lower_arr[:,0], self.ref_lower_arr[:,1], label = 'Tube Lower')
        axs[0, 0].plot(self.ref_upper_arr[:,0], self.ref_upper_arr[:,1], label = 'Tube Upper')

        axs[0, 0].set_title('Path: x vs y')
        axs[0, 0].set_xlabel('x')
        axs[0, 0].set_ylabel('y')
        axs[0, 0].legend()

        # Plot 2: ed vs time
        axs[0, 1].plot(self.ed_arr, label='ed (distance error)', color='g')
        axs[0, 1].plot(self.rhod_arr, label = 'funnel upper')
        axs[0, 1].plot(np.ones(len(self.ed_arr)) * self.rhod_lower, label = 'funnel lower')
        axs[0, 1].set_title('ed vs Time')
        axs[0, 1].set_xlabel('Time')
        axs[0, 1].set_ylabel('ed')
        axs[0, 1].legend()

        # Plot 3: eo vs time
        axs[1, 0].plot(self.eo_arr, label='eo (orientation error)', color='orange')
        axs[1, 0].plot(self.rhoo_arr, label = 'funnel upper')
        axs[1, 0].plot(-1 * self.rhoo_arr, label = 'funnel lower')
        axs[1, 0].set_title('eo vs Time')
        axs[1, 0].set_xlabel('Time')
        axs[1, 0].set_ylabel('eo')
        axs[1, 0].legend()

        # Plot 4: cmd_vel vs time
        axs[1, 1].plot(self.cmd_vel_arr, label='cmd_vel (commanded velocity)', color='purple')
        axs[1, 1].set_title('cmd_vel vs Time')
        axs[1, 1].set_xlabel('Time')
        axs[1, 1].set_ylabel('cmd_vel')
        axs[1, 1].legend()

        # Plot 5: Steering angle vs time
        axs[2, 0].plot(self.steer_arr, label='Steering angle', color='brown')
        axs[2, 0].set_title('Steering Angle vs Time')
        axs[2, 0].set_xlabel('Time')
        axs[2, 0].set_ylabel('Steering angle (rad)')
        axs[2, 0].legend()

        # Hide the unused subplot
        axs[2, 1].plot(self.theta_arr, label='Steering angle', color='red')
        axs[2, 1].set_title('Theta vs Time')
        axs[2, 1].set_xlabel('Time')
        axs[2, 1].set_ylabel('Theta')
        axs[2, 1].legend()        

        # Adjust layout to prevent overlap
        plt.tight_layout()

        # Show the plot
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = DrivePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
        node.plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
