#!/usr/bin/env python
from rowbot_vision.msg import ObjectArray, Object
from visualization_msgs.msg import Marker
import tf
import rospy

class Obstacle():
    def __init__(self,tf_broadcaster):
        self.x = 0
        self.y = 0
        self.rot = tf.transformations.quaternion_from_euler(0,0,0)
        self.tf_broadcaster = tf_broadcaster
        self.object = Object()

    def broadcast(self):
        self.tf_broadcaster.sendTransform(
        (self.x,self.y,0),
        self.rot,
        rospy.Time.now(),
        self.object.frame_id,
        "map"
        )

class ObjectServer():
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.pub = rospy.Publisher("objects", ObjectArray)
        # self.vis_pub=rospy.Publisher("visualization_marker",Marker)
        self.objects = []

    def broadcast_objects(self):
        #print("Publishing", self.objects)
        objectlist = ObjectArray()
        for i in self.objects:
            i.broadcast()
            objectlist.objects.append(i.object)
        self.pub.publish(objectlist)

    def addObstacle(self,x,y,frame_id, name,guess):
        new_obstacle = Obstacle(self.tf_broadcaster)
        new_obstacle.x = x
        new_obstacle.y = y
        new_obstacle.object.frame_id = frame_id
        new_obstacle.object.name = name
        new_obstacle.object.best_guess = guess
        new_obstacle.object.types = [guess]
        new_obstacle.object.confidences = [1]
        self.objects.append(new_obstacle)
        """
        _marker=Marker()
        _marker.header.frame_id = "base_link"
        _marker.header.stamp = rospy.Time.now()
        _marker.ns = "sim_object_server"
        _marker.id = 0
        _marker.type = Marker.SPHERE
        _marker.action = Marker.ADD
        _marker.pose.position.x = x
        _marker.pose.position.y = y
        _marker.pose.position.z = 0
        _marker.pose.orientation.x = 0.0
        _marker.pose.orientation.y = 0.0
        _marker.pose.orientation.z = 0.0
        _marker.pose.orientation.w = 1.0
        _marker.scale.x = 1
        _marker.scale.y = 0.1
        _marker.scale.z = 0.1
        _marker.color.a = 1.0 
        _marker.color.r = 0.0
        _marker.color.g = 1.0
        _marker.color.b = 0.0
        _marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"
        self.vis_pub.publish( _marker )
        """


if __name__ == "__main__":
    rospy.init_node("object_server")
    os = ObjectServer()
    rate = rospy.Rate(5)
    os.addObstacle(10,10,"1","buoy1","red buoy")
    os.addObstacle(20,10,"2","buoy2","green buoy")
    os.addObstacle(10,30,"3","buoy3","red buoy")
    os.addObstacle(20,30,"4","buoy4","green buoy")

    while not rospy.is_shutdown():
        os.broadcast_objects()
        rate.sleep()