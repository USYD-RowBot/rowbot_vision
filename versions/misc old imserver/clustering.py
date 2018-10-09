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

class Obstacle():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.rot = tf.transformations.quaternion_from_euler(0,0,0)
        self.object = Object()
        self.points = None
        self.radius = None
        self.tf_broadcaster = tf.TransformBroadcaster()
    def broadcast(self):
        self.tf_broadcaster.sendTransform(
        (self.x,self.y,0),
        self.rot,
        rospy.Time.now(),
        self.object.frame_id,
        "map"
        )


class Clusterer():
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher("objects", ObjectArray)
        self.objects = []
        self.objectlist = ObjectArray()

    def broadcast_objects(self):
        for i in self.objects:
            i.broadcast()
            self.objectlist.objects.append(i.object)
        self.pub.publish(self.objectlist)



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

        thresh = 1.5
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
                    my_obj.x = x
                    my_obj.y = y
                    my_obj.radius = max_dist
                    my_obj.points = clusters[cluster]
                    updated = True
                    name = my_obj.object.frame_id
                    break
            if updated == False:
                print("Adding new object")
                my_obj = Obstacle()
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
    rospy.init_node("map_clusterer")
    clust = Clusterer()
    rate = rospy.Rate(10)
    sub = rospy.Subscriber("map",OccupancyGrid,clust.callback)
    while not rospy.is_shutdown():
        clust.broadcast_objects()
        rate.sleep()
