#!/usr/bin/env python

import rospy
import roslaunch
import numpy as np
import time
import math

from geometry_msgs.msg import PoseStamped
from icuas23_competition.msg import poi

from std_msgs.msg import Bool

global is_tile

def tile_callback(data):
    global is_tile
    if data.data == True:
        is_tile = True

def distance(start_pose, end_pose, poi=False):
    if poi:
        dx = end_pose.x - start_pose.x
        dy = end_pose.y - start_pose.y
        dz = end_pose.z - start_pose.z
    else:
        dx = end_pose.pose.position.x - start_pose.pose.position.x
        dy = end_pose.pose.position.y - start_pose.pose.position.y
        dz = end_pose.pose.position.z - start_pose.pose.position.z
    distance = (dx*dx + dy*dy + dz*dz)**0.5

    return distance

def listener(poi_array):
    global is_tile
    is_tile = False
    # rospy.loginfo('hello')
    rospy.Subscriber('/yolov7/is_tile', Bool, tile_callback)
    initial_pose = rospy.wait_for_message('/red/carrot/pose', PoseStamped)
    poi_array.poi.insert(0, initial_pose.pose.position)

    sorted_poi_array = []
    #print("initial_pose",initial_pose.pose.position)

    current_node = 0
    min_dist = [0, 1000]
    visited = []
    for i in range(len(poi_array.poi)):
        i = min_dist[0]
        visited.append(i)
        min_dist = [0, 1000]
        for j in range(len(poi_array.poi)):
            if poi_array.poi[j] == -1 or i == j:
                continue
            dist = distance(poi_array.poi[i], poi_array.poi[j], True)
            if dist < min_dist[1]:
                min_dist[0] = j
                min_dist[1] = dist
        sorted_poi_array.append(poi_array.poi[min_dist[0]])
        poi_array.poi[i] = -1
    #print("initial_poses",sorted_poi_array[1])
    sorted_poi_array.pop()
    #sorted_poi_array.pop(3)
    #sorted_poi_array.pop(0)
    
    sorted_poi_array.append(initial_pose.pose.position)   #for last waypoint to be at takeoff position
    
    
    way_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    goto_pub = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=10)
    time.sleep(1)
    
    goto = PoseStamped()
    goto.header.seq = 1
    goto.header.stamp = rospy.Time.now()
    goto.header.frame_id = "world"

    goto.pose.position.x = initial_pose.pose.position.x
    goto.pose.position.y = initial_pose.pose.position.y
    goto.pose.position.z = initial_pose.pose.position.z+10

    goto.pose.orientation.x = 0
    goto.pose.orientation.y = 0
    goto.pose.orientation.z = 0.71
    goto.pose.orientation.w = 0.71

    goto_pub.publish(goto)
    time.sleep(5)
    

    #quarts = [[0,0,-1,0], [0,0,-0.71,0.71], [0,0,0,1], [0,0,0.71,0.71]]
    quarts = [[0,0,-0.23,0.97], [0,0,-0.97, 0.23],[0,0,0.71,0.71]]
    
    old_dist = 0
    for point in sorted_poi_array:
        if point.z < 1:
    	    point.z = 1 #margin
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/uav_ws/src/icuas23_competition/launch/yolov7.launch"])
        is_tile = False
        loop_iterations = 0
        next_loop = 0
        time.sleep(1)
        waypoint = PoseStamped()
        waypoint.header.seq = 1
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "world"

        waypoint.pose.position.x = point.x
        waypoint.pose.position.y = point.y
        waypoint.pose.position.z = point.z
        waypoint.pose.orientation.x = 0
        waypoint.pose.orientation.y = 0
        waypoint.pose.orientation.z = 0.71
        waypoint.pose.orientation.w = 0.71
        way_pub.publish(waypoint)
        print(point)
        curr_pose = rospy.wait_for_message('/red/carrot/pose', PoseStamped)
        dist = distance(curr_pose, waypoint)
        while (dist>1.5 and next_loop<3):
            curr_pose = rospy.wait_for_message('/red/carrot/pose', PoseStamped)
            dist = distance(curr_pose, waypoint)
            if dist == old_dist:  
                loop_iterations +=1 #CHANGE TO 1
            if loop_iterations>9:
                way_pub.publish(waypoint)
                time.sleep(1)
                loop_iterations = 0
                next_loop += 1
            old_dist = dist
            print(dist)
            time.sleep(1) #1.25
        if next_loop>5:
            print("next waypoint")
            continue
        print("reached")
        launch.start()
        time.sleep(2)
        print(is_tile)
        # break if tile is detected by deep learning model (yolo)
        for i in range(5):
            if is_tile == True:
                print("tile found")
                launch.shutdown()
                print("yolo shutdown")
                time.sleep(0.2)
                break
            goto = PoseStamped()
            goto.header.seq = 1
            goto.header.stamp = rospy.Time.now()
            goto.header.frame_id = "world"

            goto.pose.position.x = point.x
            goto.pose.position.y = point.y
            goto.pose.position.z = point.z+i

            goto.pose.orientation.x = 0
            goto.pose.orientation.y = 0
            goto.pose.orientation.z = 0.71
            goto.pose.orientation.w = 0.71

            goto_pub.publish(goto)
            time.sleep(3)
            for quart in quarts:
                goto = PoseStamped()
                goto.header.seq = 1
                goto.header.stamp = rospy.Time.now()
                goto.header.frame_id = "world"

                goto.pose.position.x = point.x
                goto.pose.position.y = point.y
                goto.pose.position.z = point.z+i

                goto.pose.orientation.x = quart[0]
                goto.pose.orientation.y = quart[1]
                goto.pose.orientation.z = quart[2]
                goto.pose.orientation.w = quart[3]

                goto_pub.publish(goto)
                time.sleep(3)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('waypoint_executor', anonymous=False)
    poi_array = rospy.wait_for_message('/red/poi', poi)
    challenge_started = rospy.wait_for_message('/red/challenge_started', Bool)
    print('started')
    #input()
    if challenge_started.data == True:
        listener(poi_array)


