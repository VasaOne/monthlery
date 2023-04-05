import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
#from nav_msgs.msg import Odometry
import tf2_ros
#from geometry_msgs.msg import PoseStamped, TransformStamped

class LaserScanToOdom(Node):
    def __init__(self):
        super().__init__('laser_scan_to_odom')

        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        #self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def scan_callback(self, scan_msg):
        self.get_logger().info(str(scan_msg.ranges)[500])
        '''try:
            # get the transform from the laser scanner frame to the odom frame
            transform = self.tf_buffer.lookup_transform('odom', scan_msg.header.frame_id, scan_msg.header.stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return'''

        ''' # create a pose message with the position of the laser scanner in the odom frame
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.orientation = transform.transform.rotation

        # create an odometry message with the pose information and publish it
        odom_msg = Odometry()
        odom_msg.header = scan_msg.header
        odom_msg.child_frame_id = scan_msg.header.frame_id
        odom_msg.pose.pose = pose_msg.pose
        self.pub.publish(odom_msg)'''

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToOdom()
    rclpy.spin(node)

if __name__ == '_main_':
    main()