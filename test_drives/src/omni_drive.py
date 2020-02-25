#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class RBKairosMover(object):

    def __init__(self, real_robot=False):

        if not real_robot:
            self._cmd_vel_topic_name = "/robot/robotnik_base_control/cmd_vel"
            self._odometry_topic_name = "/robot/robotnik_base_control/odom"
        else:
            self._cmd_vel_topic_name = "/robot/robotnik_base_control/cmd_vel"
            self._odometry_topic_name = "/robot/robotnik_base_control/odom"

        self._movement_class = Twist()
        self._movement_class_stop = Twist()
        self._cmd_vel_pub = rospy.Publisher(self._cmd_vel_topic_name, Twist, queue_size=1)

        self._check_cmd_vel_pub()


        # Odometry subscriber
        self._odom_pose = self._check_odom_ready()
        rospy.Subscriber(self._odometry_topic_name, Odometry, self._odom_callback)

    def _check_cmd_vel_pub(self):
        """
        Checks that the cmd_vel publisher is available.
        This is vital to be sure that when you publish inside this topic, its reieved.
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to "+self._cmd_vel_topic_name+" yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is reseted, time when backwards.
                pass
        rospy.logdebug(self._cmd_vel_topic_name+" Publisher Ready")


    def _check_odom_ready(self):
        odom_value = None
        rospy.logdebug("Waiting for "+self._odometry_topic_name+" to be READY...")
        while odom_value is None and not rospy.is_shutdown():
            try:
                odom_value = rospy.wait_for_message(self._odometry_topic_name, Odometry, timeout=0.5)
                rospy.logdebug("Current "+self._odometry_topic_name+" READY=>")

            except:
                rospy.logerr("Current "+self._odometry_topic_name+" not ready yet, retrying for getting odom")

        return odom_value.pose.pose


    def _odom_callback(self, data):
        self._odom_pose = data.pose.pose

    def get_x_y_yaw(self, odom_pose):
        """
        It gets for us the x, y and yaw form a pose message
        """
        x_value = self._odom_pose.position.x
        y_value = self._odom_pose.position.y

        orientation_list = [self._odom_pose.orientation.x, self._odom_pose.orientation.y, self._odom_pose.orientation.z, self._odom_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        return x_value, y_value, yaw


    def check_movement(self, delta_x = None, delta_y = None, delta_angular = None, movement_object= None):
        """
        Check that the x,y an dangular values have changes from first call the delta given
        When None this value wont be considered
        delta_x = None
        delta_y = None
        delta_angular = None
        """
        x0_value, y0_value, yaw0 = self.get_x_y_yaw(self._odom_pose)
        movement_achieved = False
        rate = rospy.Rate(10)  # 10hz

        while not movement_achieved and not rospy.is_shutdown():
            rospy.logerr("Movement NOT ACHIEVED")

            x1_value, y1_value, yaw1 = self.get_x_y_yaw(self._odom_pose)

            if delta_x:
                xdelta_value = x1_value - x0_value
                rospy.loginfo("xdelta_value="+str(xdelta_value))
                if delta_x >= 0:
                    # Positive value
                    delta_x_ok = (xdelta_value >= delta_x)
                else:
                    # Negative Value
                    delta_x_ok = (xdelta_value <= delta_x)
            else:
                delta_x_ok = True

            if delta_y:
                ydelta_value = y1_value - y0_value
                rospy.loginfo("ydelta_value="+str(ydelta_value))
                if delta_y >= 0:
                    # Positive value
                    delta_y_ok = (ydelta_value >= delta_y)
                else:
                    # Negative Value
                    delta_y_ok = (ydelta_value <= delta_y)
            else:
                delta_y_ok = True


            if delta_angular:
                yaw_delta_value = yaw1 - yaw0
                rospy.loginfo("yaw_delta_value="+str(yaw_delta_value))
                if delta_angular >= 0:
                    # Positive value
                    delta_angular_ok = (yaw_delta_value >= delta_angular)
                else:
                    # Negative Value
                    delta_angular_ok = (yaw_delta_value < delta_angular)
            else:
                delta_angular_ok = True

            # We check if movement achieved
            movement_achieved = delta_x_ok and delta_y_ok and delta_angular_ok

            try:
                self._cmd_vel_pub.publish(movement_object)
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is reseted, time when backwards.
                pass

        if movement_achieved:
            rospy.loginfo("Movement Achived, Stopping")
            self._cmd_vel_pub.publish(self._movement_class_stop)
        else:
            rospy.logerr("Movement NOT ACHIEVED")
            

        return movement_achieved



    def move_in_square_omni(self):
        """
        Move RB-Kairos in a square
        """

    def move_rbkairos(self,dir, distance, l_speed=0.2, a_speed=0.5):

        lin_x = 0.0
        delta_x_distance = None
        lin_y = 0.0
        delta_y_distance = None
        ang_z = 0.0
        delta_angular_distance = None

        if dir == "forwards":
            lin_x = l_speed
            delta_x_distance = distance
        elif dir == "backwards":
            lin_x = -1*l_speed
            delta_x_distance = -1*distance
        elif dir == "left":
            lin_y = l_speed
            delta_y_distance = distance
        elif dir == "right":
            lin_y = -1*l_speed
            delta_y_distance = -1*distance
        elif dir == "turn_right":
            ang_z = a_speed
            delta_angular_distance = distance
        elif dir == "turn_left":
            ang_z = -1*a_speed
            delta_angular_distance = -1*distance
        else:
            rospy.logerr("Movement not supported")


        self._movement_class.linear.x = lin_x
        self._movement_class.linear.y = lin_y
        self._movement_class.angular.x = ang_z

        
        self.check_movement(delta_x = delta_x_distance,
                             delta_y = delta_y_distance,
                              delta_angular = delta_angular_distance,
                              movement_object = self._movement_class)
        



def odometry_check_test():
    rospy.init_node('talker', anonymous=True)
    rbkairos_movement = RBKairosMover()
    square_side = 1.0
    rbkairos_movement.check_movement(delta_x=square_side)
    rbkairos_movement.check_movement(delta_x=-1*square_side)
    rbkairos_movement.check_movement(delta_y=square_side)
    rbkairos_movement.check_movement(delta_y=-1*square_side)
    rbkairos_movement.check_movement(delta_angular=1.57)
    rbkairos_movement.check_movement(delta_angular=-1.57)


def move_square_test():
    rospy.init_node('talker', anonymous=True, log_level=rospy.DEBUG)
    rbkairos_movement = RBKairosMover()

    square_side = 0.5

    rbkairos_movement.move_rbkairos(dir="forwards", distance=square_side)
    rbkairos_movement.move_rbkairos(dir="left", distance=square_side)
    rbkairos_movement.move_rbkairos(dir="backwards", distance=square_side)
    rbkairos_movement.move_rbkairos(dir="right", distance=square_side)


if __name__ == '__main__':
    try:
        move_square_test()
    except rospy.ROSInterruptException:
        pass