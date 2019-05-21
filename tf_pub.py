import tf
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path


class tf_pub():
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.refresh_odom = True
        self.current_odom = PoseWithCovarianceStamped()
        rospy.init_node("tf_pub")
        self.init_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_callback)
        rospy.spin()


    def init_pose_callback(self, msg):
        self.refresh_odom = False
        self.current_odom = msg
        self.refresh_odom = True

        # print "Start Point:"
        self.loop()

    def loop(self):
        while self.refresh_odom:
            self.br.sendTransform((self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y, self.current_odom.pose.pose.position.z),
                                  (self.current_odom.pose.pose.orientation.x, self.current_odom.pose.pose.orientation.y, self.current_odom.pose.pose.orientation.z, self.current_odom.pose.pose.orientation.w),
                                  rospy.Time.now(),
                                  "odom",
                                  "map")

if __name__ == '__main__':
    tf_pub()