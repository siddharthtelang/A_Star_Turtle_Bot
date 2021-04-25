#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import csv
import tf
import roslib
import numpy as np
from astar import *
import sys


wheel_radius = 0.038
wheel_distance = 0.354

def read_file(file_name):

    rows= []
    with open(file_name, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            rows.append(row)

    return rows


def update_path_msg(path_points):

    path = Path()
    path.header.frame_id = "/map"
    
    for point in path_points:
        x = float(point[0])
        y = float(point[1])

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        
        path.poses.append(pose)

    return path
 
def getLinearAnglularVel(w1, w2):
    v = (wheel_radius / 2) * (w1 + w2)
    w = (wheel_radius / wheel_distance) * (w1 - w2)
    return v, w

def get_twist_msg(move):
    vel = Twist()
    w1 = float(move[0])
    w2 = float(move[1])
    print(w1, w2)

    lin_vel, ang_vel = getLinearAnglularVel(w1, w2)

    vel.linear.x = lin_vel
    vel.linear.y = 0
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_vel

    return vel


def getNearestPoint(current_position, path_points):
    current_x = current_position.position.x
    current_y = current_position.position.y
    current_point = np.array([current_x, current_y]).reshape(-1,2)
    path_points = np.array(path_points, dtype = float).reshape(-1, 2)
    diff = path_points - current_point
    sd = np.sum(np.square(diff), axis = 1)
    idx = np.argmin(sd)
    min_dist = np.sqrt(sd[idx])
    delx, dely = diff[idx, 0], diff[idx, 1]

    if (delx < 0):
        min_dist = min_dist
    else:
        min_dist = -min_dist

    return path_points[idx, 0], path_points[idx, 1], min_dist


def getYaw(rot):
    euler = tf.transformations.euler_from_quaternion(rot)
    yaw = euler[2]
    return yaw

def getCurrectPosition(trans, rot):
    current_pose = Pose()
    current_pose.position.x = trans[0]
    current_pose.position.y = trans[1]
    current_pose.position.z = trans[2]

    current_pose.orientation.x = rot[0]
    current_pose.orientation.y = rot[1]
    current_pose.orientation.z = rot[2]
    current_pose.orientation.w = rot[3]

    return current_pose
    
def updateOmega(vel, d):
    k  = 1.7
    new_vel = vel
    omega = vel.angular.z
    omega_new = omega + k * d
    new_vel.angular.z = omega_new
    return new_vel

def path_publisher():

    rospy.sleep(3.)
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('path_publisher', anonymous=True)

    listener = tf.TransformListener()

    start_point = [int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])]
    goal_state = [int(sys.argv[4]), int(sys.argv[5])]
    clearance = int(sys.argv[6])
    rpm1, rpm2 = int(sys.argv[7]), int(sys.argv[8])

    AstarPath = PathPlanning(start_point, goal_state, rpm1, rpm2, clearance)
    moves, node_points, path_points = AstarPath.astarPath()
    rospy.sleep(3.)

    # path_file_name = "/home/siddharth/turtlebot_ws/src/path_publisher/src/path_points.csv"
    # node_file_name = "/home/siddharth/turtlebot_ws/src/path_publisher/src/path_nodes.csv"
    # move_file_name = "/home/siddharth/turtlebot_ws/src/path_publisher/src/vel_points.csv"
    #print('Using' , move_file_name)
    # moves = read_file(move_file_name)
    # node_points = read_file(node_file_name)
    # path_points = read_file(path_file_name)

    path = update_path_msg(path_points)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            current_pose = getCurrectPosition(trans, rot)
            dx, dy, min_dist = getNearestPoint(current_pose, path_points)
            #print(min_dist)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No tf available between map and baselink......................")
            rospy.sleep(1.)
            continue

        msg_str = "Publishing vel and path %s" % rospy.get_time()
        #rospy.loginfo(msg_str)

        if len(moves) == 0:
            move = [0,0]
        else:
            move = moves.pop(0)

        #print(move)

        vel = get_twist_msg(move)
        #update omega
        if (abs(min_dist) > 0.1 and len(moves) > 0):
           vel =  updateOmega(vel, min_dist)

        path_pub.publish(path)
        vel_pub.publish(vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass
