#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from ast import literal_eval  # lib to parse coordinates for me
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import numpy
import math


class Grid:
    def __init__(self, start, goal, occupancy_grid_data, width, height, resolution):
        self.grid = numpy.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.start = start
        self.goal = goal

    def cell_at(self, x, y):
        return self.grid[y, x]
        # return self.grid[x + y * self.width]


class Plan:
    def __init__(self, start, goal):
        rospy.init_node("planner")
        rospy.sleep(2)
        self.map = None  # the variable containing the map.
        self.start = start[0]
        self.goal = goal[0]
        self.prev = None
        self.dict = {}
        self.sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        self.pub = rospy.Publisher("pose_sequence", PoseStamped, queue_size=1)
        self.vispub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

    def goal_dist_calc(self, point_1, point_2):  # Pythagorean Theorem to get distance
        dist = math.sqrt(((point_2[0] - point_1[0]) * (point_2[0] - point_1[0]) + (point_2[1] - point_1[1]) * (
                    point_2[1] - point_1[1])))
        return dist

    def map_callback(self, msg):  # basically main function
        self.map = Grid(start, goal, msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        poses = self.a_star_search()
        self.publish_path(poses)

    def a_star_search(self):
        open_node = []
        closed_node = []
        start_dist = 0
        goal_dist = self.goal_dist_calc(self.goal, self.start)

        open_node.append([self.start[0], self.start[1], start_dist, goal_dist])  # adds on the node to list

        while len(open_node) > 0:  # While there are nodes available
            open_node.sort(key=lambda x: x[2] + x[3])
            current = open_node.pop(0)  # pop the first one off
            if (current[0], current[1]) in closed_node:
                continue
            closed_node.append((current[0], current[1]))  # Adds current node to place you have been

            print('Current Node is: ' + str(current))
            print('Goal Node is: ' + str(self.goal))

            if current[0] == self.goal[0] and current[1] == self.goal[
                1]:  # checks if current node is where we want to be
                path = []
                while current != (self.start[0], self.start[1]):  # while the path is not our start
                    path.append(current)
                    current = self.dict[
                        (current[0], current[1])]  # add current node then get previous node as current, linked list
                print(path[::-1])
                return path[::-1]  # return path in reverse order

            neighbors = [(current[0] + 1, current[1], current[2] + 1), (current[0] - 1, current[1], current[2] + 1),
                         (current[0], current[1] - 1, current[2] + 1), (current[0], current[1] + 1, current[
                    2] + 1)]  # create all the possible neighbors for that node

            for next in neighbors:  # does a loop in all of the neighbors
                map_value = self.map.cell_at(next[0], next[1])

                if map_value == 100:
                    closed_node.append((next[0], next[1]))  # Adds to closed nodes so it gets skipped
                    continue
                elif (next[0], next[1]) in closed_node:  # make sure the node isn't already in closed nodes
                    continue
                else:  # gets the previous point and puts it in a dictionary
                    self.dict[(next[0], next[1])] = (current[0], current[1])
                    result = self.goal_dist_calc(self.goal, next)
                    open_node.append([next[0], next[1], next[2], result])  # put node in open_node list

        return None

    def publish_path(self, path):
        for x in range(len(path) - 1):
            pose = Pose()
            marker = Marker() # Makes pose and marker
            current_node = path[x]

            if x != len(path) - 1: # makes sure it's not the last node
                next_node = path[x + 1]
                theta = math.atan2(next_node[1] - current_node[1], next_node[0] - current_node[0])
                pose.orientation.z = theta

            pose.position.x = current_node[0] * self.map.resolution
            pose.position.y = current_node[1] * self.map.resolution

            msg = PoseStamped()
            msg.pose = pose
            self.pub.publish(msg)

            marker.type = marker.SPHERE # pubishes the node and shows it on rviz
            marker.id = x
            marker.action = marker.ADD
            marker.pose = pose
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.header.seq = 1
            self.vispub.publish(marker)

            rospy.sleep(1)


if __name__ == "__main__":
    try:
        start = [literal_eval(coord) for coord in raw_input("Start Coords: ").split()]
    except ValueError:
        print("Please enter the coordinates in the format mentioned")
        exit()

    try:
        goal = [literal_eval(coord) for coord in raw_input("Goal Coords: ").split()]
    except ValueError:
        print("Please enter the coordinates in the format mentioned")
        exit()

    p = Plan(start, goal)
    rospy.spin()


