#!/usr/bin/env python
import rospy, cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class FaceToFace():
    def __init__(self):
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.pub = rospy.Publisher("face", Image, queue_size = 1)
        self.bridge = CvBridge()
        self.image_org = None
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        rospy.on_shutdown(rospy.ServiceProxy('motor_off', Trigger).call)
        rospy.ServiceProxy('motor_on', Trigger).call()
        
        self.angular_gain = rospy.get_param("/vision_control/angular_gain")
        self.scale_factor = rospy.get_param("/vision_control/scale_factor")
        self.min_neighbors = rospy.get_param("/vision_control/min_neighbors")
        
        classifier = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        self.cascade = cv2.CascadeClassifier(classifier)
    
    def get_image(self, img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def monitor(self, rect, org):
        if rect is not None:
            cv2.rectangle(org, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 255), 4)
        
        self.pub.publish(self.bridge.cv2_to_imgmsg(org, "bgr8"))
    
    def detect_face(self):
        if self.image_org is None:
            return None, None
        
        org = self.image_org
        
        gimg = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
        face = self.cascade.detectMultiScale(gimg, self.scale_factor, self.min_neighbors, cv2.CASCADE_FIND_BIGGEST_OBJECT)
        
        if len(face) == 0:
            self.monitor(None, org)
            return None, org
        
        r = face[0]
        self.monitor(r, org)
        
        return r, org

    def rot_vel(self):
        r, image = self.detect_face()
        if r is None:
            return 0.0
        
        wid = image.shape[1] / 2
        wid = self.image_org.shape[1] / 2
        pos_x_rate = (r[0] + r[2] / 2 - wid) * 1.0 / wid
        rot = -self.angular_gain * pos_x_rate * math.pi
        rospy.loginfo("detected %.6f", rot)
        return rot
    
    def control(self):
        m = Twist()
        m.linear.x = 0.0
        m.angular.z = self.rot_vel()
        self.cmd_vel.publish(m)

if __name__ == "__main__":
    rospy.init_node("face_to_face")
    fd = FaceToFace()
    
    control_rate = rospy.get_param("/vision_control/control_rate")
    rate = rospy.Rate(control_rate)
    while not rospy.is_shutdown():
        fd.control()
        rate.sleep()
