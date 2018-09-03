#!/usr/bin/env python
from visualization_msgs.msg import Marker
from rowbot_vision.msg import Object, ObjectArray
import rospy
publisher = rospy.Publisher('visulization/objects',Marker)
def callback(data):
    print("recieved object")
    object_list = data.objects
    id = 0;
    for i in object_list:
        marker = Marker()
        marker.header.frame_id = i.frame_id
        marker.header.stamp = rospy.rostime.Time.now()
        marker.ns = i.name
        marker.id = id
        id = id+1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x=0
        marker.pose.position.y=0
        marker.pose.position.z=0
        marker.pose.orientation.x=0
        marker.pose.orientation.y=0
        marker.pose.orientation.y=0
        marker.pose.orientation.z=1
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z=1
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        marker.lifetime = rospy.Duration()
        publisher.publish(marker)


if __name__ == '__main__':
    rospy.init_node('rviz_publisher')
    rospy.Subscriber("objects",ObjectArray, callback)

    rospy.spin()
        
