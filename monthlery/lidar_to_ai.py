import rclpy
from sensor_msgs.msg import LaserScan
#from nav_msgs.msg import Odometry
import tf2_ros
#from geometry_msgs.msg import PoseStamped, TransformStamped

class LaserScanToOdom:
    def __init__(self):
        rclpy.init_node('laser_scan_to_odom')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = rclpy.Subscriber('/scan', LaserScan, self.scan_callback)
        #self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def scan_callback(self, scan_msg):
        self.get_logger().info(scan_msg)
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

def main():
    LaserScanToOdom()
    rclpy.spin()

if __name__ == '_main_':
    main()