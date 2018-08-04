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
from qhull_2d import *
from min_bounding_rect import *
import ClassifyServer

class Obstacle():
    def __init__(self,tf_broadcaster,tf_listener,classify_server):
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
        self.classify_server = classify_server
    def broadcast(self):
        self.tf_broadcaster.sendTransform(
        (self.x,self.y,0),
        self.rot,
        rospy.Time.now(),
        self.object.frame_id,
        "map"
        )
    def classify(self):
        score_buoy = 0
        score_dock = 0
        #Try Detect by camera

        if self.radius < 1.2:
            #Try get detected by camera
            result = False
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.object.frame_id,"mycamera",rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(rot)
                bearing = euler[2]
                result = self.classify_server.classify_buoy(bearing)


            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            if result == False:
                print("Passing")
                pass
            else:
                self.seen_by_camera = True
                self.object.types = result[0]
                self.object.confidences = result[1]

            if self.seen_by_camera == False:
                self.object.types = ["buoy"]
                self.object.confidences = [1]
            else:
                #Keep it the same result
                pass
        elif self.radius < 10:
            self.classify_dock()
            self.object.types = ["dock"]
            self.object.confidences = [1]
        else :
            self.object.types = ["land"]
            self.object.confidences = [0.5]
        print(self.object.types, self.object.confidences)

        max_conf = max(self.object.confidences)
        best = self.object.types[self.object.confidences.index(max_conf)]
        self.object.best_guess = best

            #if indeterminiate then it could be a light bouy
    def classify_dock(self):
        if len(self.points) < 5 or self.radius < 3 :
            print("Cant be a bouy")
            return
        points = numpy.array(self.points)
        hull_points = qhull2D(points)
        hull_points = hull_points[::-1]
        print 'Convex hull points: \n', hull_points, "\n"
        (rot_angle, area, width, height, center_point, corner_points) = minBoundingRect(hull_points)

        print "Minimum area bounding box:"
        print "Rotation angle:", rot_angle, "rad  (", rot_angle*(180/math.pi), "deg )"
        print "Width:", width, " Height:", height, "  Area:", area
        print "Center point: \n", center_point # numpy array
        print "Corner points: \n", corner_points, "\n"  # numpy array


        #Get orientation and apply to rot
        if (width < height):
            self.rot =tf.transformations.quaternion_from_euler(0,0,rot_angle)
        else:
            self.rot =tf.transformations.quaternion_from_euler(0,0,rot_angle+1.5707)

class ObjectServer():

    def __init__(self):
        self.classify_server = ClassifyServer.ClassifyServer()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher("objects", ObjectArray)
        self.objects = []

    def classify_objects(self):
        for i in self.objects:
            i.classify()
    def broadcast_objects(self):
        #print("Publishing", self.objects)
        objectlist = ObjectArray()
        for i in self.objects:
            i.classify()
            i.broadcast()

            objectlist.objects.append(i.object)
        self.pub.publish(objectlist)
    def cleanup(self):
        print("Cleaning")
        for i in self.objects:

            time_diff = rospy.Time.now().secs - i.time.secs
            print(i.object.frame_id, time_diff)
            if time_diff > 3:
                print("Removing")
                self.objects.remove(i)


    def callback(self,my_map):
        points_x = []
        points_y = []
        my_data = []
        info = my_map.info
        r = 0
        c = 0
        count  = 0


        for i in my_map.data:
            if i == 100:
                points_x.append(r*info.resolution)
                points_y.append(c*info.resolution)
                my_data.append((r*info.resolution,c*info.resolution))
            r = r+1
            if r == info.width:
                r = 0
                c = c +1
        #print(points_x)

        #plt.scatter(points_x,points_y)

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
        #print(clusters)
        #plt.scatter(*numpy.transpose(my_data), c=clust)
        #plt.axis("equal")
        #title = "threshold: %f, number of clusters: %d" % (thresh, len(set(clust)))
        #plt.title("wow")
        #plt.show(block=False)

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
            #print(avg,max_dist)

            x = avg[0] + info.origin.position.x
            y = avg[1] + info.origin.position.y


            #if Close to a current object, update else: add new
            updated = False
            current_frames = []
            name = ""
            for my_obj in self.objects:
                frame_id = my_obj.object.frame_id
                current_frames.append(frame_id)
                thresh_dist = 1
                dist = math.sqrt((my_obj.x-x)**2 + (my_obj.y-y)**2)
                #print(dist)
                if (dist<thresh_dist):
                    #print("Updating Object")
                    my_obj.x = x
                    my_obj.y = y
                    my_obj.radius = max_dist
                    my_obj.points = clusters[cluster]
                    updated = True
                    name = my_obj.object.frame_id
                    my_obj.time = rospy.Time.now()
                    break
            if updated == False:
                print("Adding new object")
                my_obj = Obstacle(self.tf_broadcaster, self.tf_listener, self.classify_server)
                my_obj.x = x
                my_obj.y = y
                my_obj.radius = max_dist
                my_obj.points = clusters[cluster]
                msg_obj = Object()
                #TODO Check if object frame is being used.
                msg_obj.frame_id = str(random.randint(1,10000))
                my_obj.object = msg_obj
                name = msg_obj.frame_id
                self.objects.append(my_obj)




if __name__ == "__main__":
    rospy.init_node("object_server")
    object_server = ObjectServer()
    rate = rospy.Rate(1)
    sub = rospy.Subscriber("map",OccupancyGrid,object_server.callback)
    while not rospy.is_shutdown():
        object_server.cleanup()
        object_server.classify_objects()
        object_server.broadcast_objects()
        rate.sleep()
