#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv.shape_test_2 as test

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image, self.callback)
    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        (return_image,msg)=test.get_shape(frame);
        #cv2.imshow("Image window", return_image)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(return_image, "bgr8"))
        except CvBridgeError as e:
          print(e)
def main(args):
  rospy.init_node('image_converter', anonymous=True)

  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
