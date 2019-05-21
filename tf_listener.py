import tf
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


class tf_listener():
    def __init__(self):
        rospy.init_node("tf_listener")
        self.listener = tf.TransformListener()
        self.point_pub = rospy.Publisher('trans_pub', Twist, queue_size=15)
        # self.path_sub = rospy.Subscriber('')




        self.trans = None
        self.rot = None
        while not rospy.is_shutdown():
            try:
                (self.trans, self.rot) = self.listener.lookupTransform('map', 'odom', rospy.Time(0))
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue



        rospy.spin()


if __name__ == '__main__':
    tf_listener()