#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import Float64

class FaceToFace():
    __slots__ = ('__cmd_vel', '__angular_gain')

    def __init__(self):
        sub = rospy.Subscriber("/pos_x_rate", Float64, self.rot_vel)
        self.__cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        rospy.on_shutdown(rospy.ServiceProxy('motor_off', Trigger).call)
        rospy.ServiceProxy('motor_on', Trigger).call()
        
        self.__angular_gain = rospy.get_param("/vision_control/angular_gain")
    
    def rot_vel(self, pos_x_rate):
        rot = -self.__angular_gain * pos_x_rate.data * math.pi
        
        m = Twist()
        m.linear.x = 0.0
        m.angular.z = rot
        self.__cmd_vel.publish(m)

if __name__ == "__main__":
    rospy.init_node("face_to_face")
    fd = FaceToFace()
    rospy.spin()
