#!/usr/bin/env python
import rospy
import tf
import matplotlib.pyplot as plt
import scipy.cluster.hierarchy as hcluster
import numpy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from rowbot_vision.msg import ObjectArray, Object
import math
import random
from objhelper.qhull_2d import *
from objhelper.min_bounding_rect import *
import ImageServer

M_PI = 3.14159265359

class Obstacle():
    """Obstacle Class containing information and functions for different detected Obstacles"""
    def __init__(self,tf_broadcaster,tf_listener,image_server):
        self.x = 0
        self.y = 0
        self.time = rospy.Time.now()
        self.rot = tf.transformations.quaternion_from_euler(0,0,0)
        self.object = Object()
        self.points = None
        self.radius = None
        self.seen_by_camera = False
        self.camera_detection = ""
        self.tf_broadcaster = tf_broadcaster
        self.tf_listener = tf_listener
        self.image_server = image_server
        self.best_guess_conf = 0
    def broadcast(self):
        """Broadcast the object via tf"""
        self.tf_broadcaster.sendTransform(
        (self.x,self.y,0),
        self.rot,
        rospy.Time.now(),
        self.object.frame_id,
        "map"
        )
    def classify(self):
        """Try classify the object using a variety of means"""
        #TODO If two objects have a similar bearing, don't classify it.
        score_buoy = 0
        score_dock = 0
        types = []
        #If self.radius is < 1.2 then it is a buoy
        #TODO Use rosparam to get buoy size.
        if self.radius < 1.2:
            #Try get detected by camera
            result = False
            try:
                (trans,rot) = self.tf_listener.lookupTransform("front_camera_link",self.object.frame_id,rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(rot)
                #Get the bearing of the object relative to the camera
                bearing= -math.degrees(math.atan2(trans[1], trans[0]))
                #Classify using the camera.
                result = self.image_server.classify_buoy(bearing,0)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            if result == False:
                #Did not get seen by the camera
                #print("Did not get seen by camera")
                pass
            else:
                self.seen_by_camera = True
                types = result[0]
                confidences = result[1]

            if self.seen_by_camera == False:
                types = ["buoy"]
                confidences = [0.2]
            else:
                pass

        elif self.radius < 10:
            self.classify_dock()
            self.object.types = ["dock"]
            self.object.confidences = [1]
        else :
            self.object.types = ["land"]
            self.object.confidences = [0.5]

        #TODO LONG TERM: Allow these settings to be modular?

        #Apply only the maximum confidence even previous confidence.
        if (len(types) != 0):
            max_conf = max(confidences)
            best = types[confidences.index(max_conf)]
            best_guess = best
            if max_conf > self.best_guess_conf:
                self.object.best_guess = best
                self.object.types = types
                self.object.confidences = confidences
                self.best_guess_conf = max_conf

    def classify_dock(self):
        """Method to classify orientation of a dock"""
        if len(self.points) < 5 or self.radius < 3 :
            #print("Cant be a dock")
            return
        points = numpy.array(self.points)
        hull_points = qhull2D(points)
        hull_points = hull_points[::-1]
        (rot_angle, area, width, height, center_point, corner_points) = minBoundingRect(hull_points)
        if (width < height):
            self.rot =tf.transformations.quaternion_from_euler(0,0,rot_angle)
        else:
            self.rot =tf.transformations.quaternion_from_euler(0,0,rot_angle+1.5707)
        #TODO Find a way to keep orientation consistent. It still can be confused in 2 directions.

class ObjectServer():
    """Object Server class to handle objects detected """
    def __init__(self):
        self.image_server = ImageServer.ImageServer()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher("objects", ObjectArray)
        self.objects = []

    def classify_objects(self):
        """Classify the objects found so far using appropiate cameras."""
        objInFront = False
        my_info = {}
        for i in self.objects:
            try:
                (trans,rot) = self.tf_listener.lookupTransform("front_camera_link",i.object.frame_id,rospy.Time(0))
                bearing =math.degrees(math.atan2(trans[1], trans[0]))
                dist = math.sqrt(trans[1]**2 + trans[0]**2)
                my_info[i] = (bearing,dist)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        for i in my_info:
            (this_bearing, this_dist )= my_info[i]
            max_bearing = this_bearing+3.5
            min_bearing = this_bearing-3.5
            for key in my_info:
                if i is key:
                    pass
                elif my_info[key][0] > min_bearing and my_info[key][0] < max_bearing and my_info[key][1] < this_dist-0.5:
                    objInFront = True
                    break

            if(objInFront == False):
                i.classify()
    def broadcast_objects(self):
        """Broadcast the objects found"""
        #print("Publishing", self.objects)
        objectlist = ObjectArray()
        for i in self.objects:
            i.classify()
            i.broadcast()

            objectlist.objects.append(i.object)
        self.pub.publish(objectlist)

    def cleanup(self):
        """Method to clean up any objects that are old"""
        print("Cleaning")
        for i in self.objects:

            time_diff = rospy.Time.now().secs - i.time.secs
            print(i.object.frame_id, time_diff)
            if time_diff > 3:
                print("Removing")
                self.objects.remove(i)

    def callback(self,my_map):
        """Callback when a map is called."""
        points_x = []
        points_y = []
        my_data = []
        info = my_map.info
        r = 0
        c = 0
        count  = 0

        #Put map into a list of points.
        for i in my_map.data:
            if i == 100:
                points_x.append(r*info.resolution)
                points_y.append(c*info.resolution)
                my_data.append((r*info.resolution,c*info.resolution))
            r = r+1
            if r == info.width:
                r = 0
                c = c +1

        #Apply a distance Cluster on the objects.
        thresh = 3
        clust = hcluster.fclusterdata(my_data, thresh, criterion="distance")
        clusters = {}
        count = 0
        for point in my_data:
            cluster_num = clust[count]
            if cluster_num not in clusters:
                clusters[cluster_num] = [point]
            else:
                clusters[cluster_num].append(point)
            count = count+1

        #Iterate through the different colusters
        for cluster in clusters:
            sum_x = 0
            sum_y = 0
            for point in clusters[cluster]:
                sum_x = sum_x + point[0]
                sum_y = sum_y + point[1]
            avg = (sum_x/len(clusters[cluster]), sum_y/len(clusters[cluster]))
            max_dist = 0
            for point in clusters[cluster]:
                dist = math.sqrt((avg[0] - point[0])**2 + (avg[1] - point[1])**2)
                if dist>max_dist:
                    max_dist = dist

            #Get the Distance of the x and y axis
            x = avg[0] + info.origin.position.x
            y = avg[1] + info.origin.position.y

            #If the object is close to an already found object. Consider it the same object.
            updated = False
            current_frames = []
            name = ""
            for my_obj in self.objects:
                frame_id = my_obj.object.frame_id
                current_frames.append(frame_id)
                thresh_dist = 1
                dist = math.sqrt((my_obj.x-x)**2 + (my_obj.y-y)**2)
                if (dist<thresh_dist):
                    my_obj.x = x
                    my_obj.y = y
                    my_obj.radius = max_dist
                    my_obj.points = clusters[cluster]
                    updated = True
                    name = my_obj.object.frame_id
                    my_obj.time = rospy.Time.now()
                    break
            #If it is not close to any other objects then add it as a new object.
            if updated == False:
                #print("Adding new object")
                my_obj = Obstacle(self.tf_broadcaster, self.tf_listener, self.image_server)
                my_obj.x = x
                my_obj.y = y
                my_obj.radius = max_dist
                my_obj.points = clusters[cluster]
                msg_obj = Object()
                #TODO Check if object frame number is being used.
                msg_obj.frame_id = str(random.randint(1,10000))
                my_obj.object = msg_obj
                name = msg_obj.frame_id
                #Append threw new object to the servers object list.
                self.objects.append(my_obj)

if __name__ == "__main__":
    rospy.init_node("object_server")
    object_server = ObjectServer()
    rate = rospy.Rate(5)
    sub = rospy.Subscriber("map",OccupancyGrid,object_server.callback)
    while not rospy.is_shutdown():
        object_server.cleanup()
        object_server.classify_objects()
        object_server.broadcast_objects()
        rate.sleep()
