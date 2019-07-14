#!/usr/bin/env python
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

class FaceDetection():
    __slots__ = ('__pub_face', '__pub_pos', '__cv_bridge', '__image_org', '__scale_factor', '__min_neighbors', '__cascade')

    def __init__(self):
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.__pub_face = rospy.Publisher("face", Image, queue_size = 1)
        self.__pub_pos = rospy.Publisher("pos_x_rate", Float64, queue_size = 1)
        self.__cv_bridge = CvBridge()
        self.__image_org = None
        
        self.__scale_factor = rospy.get_param("/vision_control/scale_factor")
        self.__min_neighbors = rospy.get_param("/vision_control/min_neighbors")
        
        classifier = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        self.__cascade = cv2.CascadeClassifier(classifier)
    
    def get_image(self, img):
        try:
            self.__image_org = self.__cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def monitor(self, rect, org):
        if rect is not None:
            cv2.rectangle(org, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 255), 4)
        
        self.__pub_face.publish(self.__cv_bridge.cv2_to_imgmsg(org, "bgr8"))
    
    def detect_face(self):
        if self.__image_org is None:
            return None, None
        
        org = self.__image_org
        
        gimg = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
        face = self.__cascade.detectMultiScale(gimg, self.__scale_factor, self.__min_neighbors, cv2.CASCADE_FIND_BIGGEST_OBJECT)
        
        if len(face) == 0:
            self.monitor(None, org)
            return None, org
        
        r = face[0]
        self.monitor(r, org)
        
        return r, org

    def control(self):
        r, image = self.detect_face()
        if r is None:
            return
        
        wid = image.shape[1] / 2
        pos_x_rate = (r[0] + r[2] / 2 - wid) * 1.0 / wid
        message = Float64()
        message.data = pos_x_rate;
        self.__pub_pos.publish(message)

if __name__ == "__main__":
    rospy.init_node("face_detection")
    fd = FaceDetection()
    
    control_rate = rospy.get_param("/vision_control/control_rate")
    rate = rospy.Rate(control_rate)
    while not rospy.is_shutdown():
        fd.control()
        rate.sleep()
