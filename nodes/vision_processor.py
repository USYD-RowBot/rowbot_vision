#!/usr/bin/env python
import rospy
from rowbot_vision.msg import Object, ObjectArray



class ObjectPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('objects', ObjectArray)
        self.objects = ObjectArray()
        
    def add_object(self,my_object):
        self.objects.objects.append(my_object)
    def publish(self):
        self.pub.publish(self.objects)
        






if __name__ == '__main__':
    rospy.init_node('vision_processor')
    rate = rospy.Rate(30)
    my_object = Object()
    my_object.frame_id = "test"
    my_object.name = "test1"
    my_object_list = ObjectPublisher()
    my_object_list.add_object(my_object)
    while not rospy.is_shutdown():
        my_object_list.publish()
        rate.sleep()
        
